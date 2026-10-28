#include <Arduino.h>

#define FILTER_ORDER 8

// Атомарные флаги и данные
volatile uint16_t adc_raw_value = 0;
volatile uint8_t adc_ready_flag = 0;

uint32_t adc_buffer[FILTER_ORDER] = {0};
uint8_t buffer_index = 0;

// Счетчик для неблокирующей задержки логов (100 тиков АЦП = 1 секунда)
uint8_t log_timer_ticks = 0; 

// Инициализация UART на регистрах (9600 бод)
void uart_init(void) {
    UBRR0H = 0;
    UBRR0L = 103; // 16 МГц / (16 * 9600) - 1 = 103
    UCSR0B = (1 << TXEN0); // Включить только передатчик
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-N-1
}

// Отправка символа
void uart_print_char(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

// Отправка строки
void uart_print_str(const char* str) {
    while (*str) {
        uart_print_char(*str++);
    }
}

// Функция для вывода чисел через регистры (чтобы убрать Serial полностью)
void uart_print_num(uint32_t num) {
    char buf[11];
    uint8_t i = 0;
    if (num == 0) {
        uart_print_char('0');
        return;
    }
    while (num > 0) {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    while (i > 0) {
        uart_print_char(buf[--i]);
    }
}

void setup() {
  // Блокируем глобальные прерывания на время инициализации периферии
  cli();

  uart_init();

  // 1. Настройка GPIO на уровне регистров
  // Настройка порта PC0 (вход A0) и отключение цифрового буфера
  DDRC &= ~(1 << DDC0);
  DIDR0 |= (1 << ADC0D);

  // Настройка порта PB1 (вывод D9 / аппаратный ШИМ OC1A) как выхода
  DDRB |= (1 << DDB1);
  PORTB &= ~(1 << PORTB1);  // Исходное безопасное состояние (вывод выключен)

  // 2. Настройка АЦП (ADC)
  // Опорное напряжение Vcc (AVCC), выравнивание вправо, канал ADC0
  ADMUX = (1 << REFS0);
  // Включение АЦП, разрешение прерывания АЦП, предделитель частоты = 128 (16
  // МГц / 128 = 125 кГц)
  ADCSRA =
      (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // 3. Настройка Timer2 (Генерация прерываний выборки 100 Гц в режиме CTC)
  TCCR2A = (1 << WGM21);                             // Режим CTC
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);  // Предделитель 1024
  OCR2A = 155;                                       // Интервал ~10 мс (100 Гц)
  TIMSK2 |= (1 << OCIE2A);  // Разрешить прерывание по совпадению А

  // 4. Настройка Timer1 (Аппаратный ШИМ 1 Гц на выводе OC1A / D9)
  // Режим Fast PWM (Mode 14), TOP задается в ICR1. Неинвертирующий режим
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) |
           (1 << CS10);  // Предделитель 1024
  ICR1 = 15624;          // Точный период 1 секунда при частоте 16 МГц
  OCR1A = 0;             // Коэффициент заполнения 0% при старте

  // Разрешаем прерывания после завершения конфигурирования
  sei();
}

// Прерывание таймера выборки (100 Гц)
ISR(TIMER2_COMPA_vect) {
  // Аппаратный запуск преобразования АЦП
  ADCSRA |= (1 << ADSC);
}

// Прерывание окончания преобразования АЦП
ISR(ADC_vect) {
  // Автоматическое атомарное чтение регистра данных АЦП
  adc_raw_value = ADC;
  adc_ready_flag = 1;
}

// Фильтр: скользящее среднее по 8 точкам
uint32_t moving_average(uint32_t new_val) {
  static uint32_t sum = 0;
  sum -= adc_buffer[buffer_index];
  adc_buffer[buffer_index] = new_val;
  sum += adc_buffer[buffer_index];
  buffer_index = (buffer_index + 1) % FILTER_ORDER;
  return sum / FILTER_ORDER;
}

void loop() {
  uint16_t current_adc = 0;

  if (adc_ready_flag) {
    // Критическая секция: безопасное копирование двухбайтового значения
    cli();
    current_adc = adc_raw_value;
    adc_ready_flag = 0;
    sei();

    // Сглаживание сигнала фильтром
    uint32_t filtered_adc = moving_average(current_adc);

    // Флаг для отправки лога (станет true раз в секунду)
    bool should_log = false;
    log_timer_ticks++;
    if (log_timer_ticks >= 100) { // 100 тиков по 10 мс = 1000 мс (1 секунда)
        log_timer_ticks = 0;
        should_log = true;
    }

    switch (filtered_adc) {
      case 0 ... 204:
        TCCR1A &= ~(1 << COM1A1);  
        PORTB &= ~(1 << PORTB1);   
        if (should_log) {
            uart_print_str("ADC: "); uart_print_num(filtered_adc);
            uart_print_str(" | Zone 1 (0%)\r\n");
        }
        break;

      case 205 ... 409:
        TCCR1A |= (1 << COM1A1);  
        OCR1A = 3906;             
        if (should_log) {
            uart_print_str("ADC: "); uart_print_num(filtered_adc);
            uart_print_str(" | Zone 2 (25%) | OCR1A: 3906\r\n");
        }
        break;

      case 410 ... 614:
        TCCR1A |= (1 << COM1A1);
        OCR1A = 7812;  
        if (should_log) {
            uart_print_str("ADC: "); uart_print_num(filtered_adc);
            uart_print_str(" | Zone 3 (50%) | OCR1A: 7812\r\n");
        }
        break;

      case 615 ... 819:
        TCCR1A |= (1 << COM1A1);
        OCR1A = 11718;  
        if (should_log) {
            uart_print_str("ADC: "); uart_print_num(filtered_adc);
            uart_print_str(" | Zone 4 (75%) | OCR1A: 11718\r\n");
        }
        break;

      case 820 ... 1023:
      default:
        TCCR1A &= ~(1 << COM1A1);  
        PORTB |= (1 << PORTB1);    
        if (should_log) {
            uart_print_str("ADC: "); uart_print_num(filtered_adc);
            uart_print_str(" | Zone 5 (100%)\r\n");
        }
        break;
    }
  }
}