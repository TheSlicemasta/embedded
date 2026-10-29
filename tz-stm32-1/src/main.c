#include "stm32f1xx.h"
#include <stdbool.h>

#define FILTER_ORDER 8

// Атомарные флаги и данные
volatile uint16_t adc_raw_value = 0;
volatile uint8_t adc_ready_flag = 0;

uint32_t adc_buffer[FILTER_ORDER] = {0};
uint8_t buffer_index = 0;

// Счетчик для неблокирующей задержки логов (100 тиков по 10 мс = 1 секунда)
uint8_t log_timer_ticks = 0;

// Низкоуровневая инициализация UART1 (9600 бод при частоте шины 8 МГц)
void uart_init(void) {
    // 1. Включаем тактование GPIOA и USART1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
    
    // 2. Настройка пина TX (PA9) как альтернативная функция Push-Pull
    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    GPIOA->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0) | GPIO_CRH_CNF9_1;
    
    // 3. Расчет скорости 9600 бод: BRR = 8 000 000 / 9600 = 833.33 -> 833 (0x341)
    USART1->BRR = 0x341;

    // 3. НОВЫЙ РАСЧЕТ ПОД 72 МГц: 72 000 000 / 9600 = 7500 (0x1D4C)
    // USART1->BRR = 0x1D4C;
    
    // 4. Включение передатчика (TE) и самого модуля UART (UE)
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

// Отправка символа через UART1
void uart_print_char(char c) {
    while (!(USART1->SR & USART_SR_TXE)); // Ждем освобождения буфера передачи
    USART1->DR = c;
}

// Отправка строки текста
void uart_print_str(const char* str) {
    while (*str) {
        uart_print_char(*str++);
    }
}

// Отправка числа
void uart_print_num(uint32_t num) {
    char buf[10];
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

void System_Init(void) {
    // 1. Включение тактования GPIOA, ADC1, TIM1 и альтернативных функций (AFIO)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_AFIOEN;
    RCC->CFGR &= ~RCC_CFGR_ADCPRE; // Делитель частоты АЦП на 2 (8 МГц / 2 = 4 МГц)

    // Инициализация UART
    uart_init();

    // 2. Конфигурация GPIO
    // PA0 как Аналоговый вход (CNF0 = 00, MODE0 = 00)
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    
    // PA8 как Выход альтернативной функции Push-Pull для TIM1_CH1 (CNF8 = 10, MODE8 = 11)
    GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
    GPIOA->CRH |= (GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0) | GPIO_CRH_CNF8_1;
    GPIOA->ODR &= ~GPIO_ODR_ODR8; // Исходное выключенное состояние

    // 3. Конфигурация ADC1
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;               // Канал 0 (PA0) в очереди первый
    ADC1->CR1 |= ADC_CR1_EOCIE;   // Прерывание по окончанию преобразования
    ADC1->CR2 |= ADC_CR2_ADON;    // Пробуждение АЦП
    
    // Калибровка АЦП (обязательна для STM32F1 серии перед началом работы)
    SysTick->LOAD = 80000;        // Пауза для стабилизации АЦП
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    
    ADC1->CR2 |= ADC_CR2_CAL;     // Старт калибровки
    while (ADC1->CR2 & ADC_CR2_CAL);

    NVIC_EnableIRQ(ADC1_2_IRQn);  // Разрешаем прерывание АЦП в контроллере NVIC

    // 4. Конфигурация TIM1 (Аппаратный ШИМ 1 Гц)
    TIM1->PSC = 7999;             // Частота счета = 1 кГц
    TIM1->ARR = 999;              // Период = 1 секунда
    TIM1->CCR1 = 0;               // Заполнение 0% при старте
    
    // Режим ШИМ 1 (OC1M = 110), буферизация включена (OC1PE)
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    
    TIM1->CCER |= TIM_CCER_CC1E;  // Включить канал 1
    TIM1->BDTR |= TIM_BDTR_MOE;   // Разрешить главный выход (особенность TIM1)
    TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; // Запуск таймера

    // 5. Конфигурация SysTick на 100 Гц (выборка каждые 10 мс)
    SysTick->LOAD = 80000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

// Прерывание SysTick (100 Гц)
void SysTick_Handler(void) {
    ADC1->CR2 |= ADC_CR2_ADON; // Запуск АЦП
}

// Прерывание окончания преобразования АЦП
void ADC1_2_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) {
        adc_raw_value = ADC1->DR;
        adc_ready_flag = 1;
        
        // ДЕБАГ: инвертируем светодиод платы при КАЖДОМ успешном замере
        GPIOC->ODR ^= GPIO_ODR_ODR13; 
    }
}

uint32_t moving_average(uint32_t new_val) {
    static uint32_t sum = 0;
    sum -= adc_buffer[buffer_index];
    adc_buffer[buffer_index] = new_val;
    sum += adc_buffer[buffer_index];
    buffer_index = (buffer_index + 1) % FILTER_ORDER;
    return sum / FILTER_ORDER;
}

int main(void) {
    System_Init();
    
    uint32_t current_adc = 0;

    while (1) {
        if (adc_ready_flag) {
            // Критическая секция: атомарное чтение данных
            __disable_irq();
            current_adc = adc_raw_value;
            adc_ready_flag = 0;
            __enable_irq();
            
            // Фильтрация скользящим средним
            uint32_t filtered_adc = moving_average(current_adc);
            
            // Неблокирующий таймер вывода логов
            bool should_log = false;
            log_timer_ticks++;
            if (log_timer_ticks >= 100) {
                log_timer_ticks = 0;
                should_log = true;
            }
            
            // Логика переключения 5 зон АЦП (диапазон 0..4095 разделен на 5 отрезков по ~819 единиц)
            switch (filtered_adc) {
                // Зона 1: от 0 до 818 (0% заполнения)
                case 0 ... 818:
                    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; // Отключаем ШИМ (Force Low режим)
                    TIM1->CCMR1 |= (4U << TIM_CCMR1_OC1M_Pos); 
                    if (should_log) {
                        uart_print_str("ADC: "); uart_print_num(filtered_adc);
                        uart_print_str(" | Zone 1 (0%)\r\n");
                    }
                    break;
                    
                // Зона 2: от 819 до 1637 (25% заполнения)
                case 819 ... 1637:
                    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; // Возвращаем ШИМ режим 1
                    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
                    TIM1->CCR1 = 250; // 25% от ARR(999) = 250
                    if (should_log) {
                        uart_print_str("ADC: "); uart_print_num(filtered_adc);
                        uart_print_str(" | Zone 2 (25%) | CCR1: 250\r\n");
                    }
                    break;
                    
                // Зона 3: от 1638 до 2456 (50% заполнения)
                case 1638 ... 2456:
                    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
                    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
                    TIM1->CCR1 = 500; // 50% от ARR(999) = 500
                    if (should_log) {
                        uart_print_str("ADC: "); uart_print_num(filtered_adc);
                        uart_print_str(" | Zone 3 (50%) | CCR1: 500\r\n");
                    }
                    break;
                    
                // Зона 4: от 2457 до 3275 (75% заполнения)
                case 2457 ... 3275:
                    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
                    TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
                    TIM1->CCR1 = 750; // 75% от ARR(999) = 750
                    if (should_log) {
                        uart_print_str("ADC: "); uart_print_num(filtered_adc);
                        uart_print_str(" | Zone 4 (75%) | CCR1: 750\r\n");
                    }
                    break;
                    
                // Зона 5: от 3276 до 4095 (100% заполнения)
                case 3276 ... 4095:
                default:
                    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk; // Отключаем ШИМ (Force High режим)
                    TIM1->CCMR1 |= (5U << TIM_CCMR1_OC1M_Pos);
                    if (should_log) {
                        uart_print_str("ADC: "); uart_print_num(filtered_adc);
                        uart_print_str(" | Zone 5 (100%)\r\n");
                    }
                    break;
            }
        }
    }
}
