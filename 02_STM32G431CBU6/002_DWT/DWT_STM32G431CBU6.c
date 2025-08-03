/**-------------------------------------------------------------------
 \date  03.08.2025
 *
 | ** DWT                                                       ** |
 | ** Код написан в IDE SEGGER Embedded Studio for ARM 7.32a    ** |
 | ** J-Link Software and Documentation pack V7.90 [2023-08-02] ** |
 *
 *   STM32G431CBU6
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |        PC.6| ---->  LED _|¯|_  Period=200ms +Width=100ms -Width=100ms Freq=5.00 Hz
 *  |            |
 *  |            |
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 *
 * https://alexgyver.ru/lessons/soft-timer
 *
 * https://istarik.ru/blog/stm32/131.html
 *
 * !!!Необходимо нажать NRST  , либо войти в режим  Build --> Build and Debug!!!
 *
 *\ authors        ScuratovaAnna
 *\ сode debugging ScuratovaAnna
 */
//*************************** Пример первый **************************
#include "stm32g4xx.h"

#define Input_mode                  (0x0UL)
#define General_purpose_output_mode (0x1UL)
#define Alternate_function_mode     (0x2UL)
#define Analog_mode                 (0x3UL)

#define Output_push_pull            (0x0UL)
#define Output_open_drain           (0x1UL)

#define Low_speed                   (0x0UL)
#define Medium_speed                (0x1UL)
#define High_speed                  (0x2UL)
#define Very_high_speed             (0x3UL)

#define No_pull_up_No_pull_down     (0x0UL)
#define Pull_up                     (0x1UL)
#define Pull_down                   (0x2UL)

void DWT_Init(void);
void DWT_Delay_ms(volatile uint32_t au32_milliseconds);
void DWT_Delay_us(volatile uint32_t au32_microseconds);

void PortC_6_Init(void) {
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOCEN);                                                      // Запуск тактирования порта C
  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE6, General_purpose_output_mode << GPIO_MODER_MODE6_Pos); // Настройка GPIOC пин 6 на выход (output mode)
  MODIFY_REG(GPIOC->OTYPER, GPIO_OTYPER_OT6, Output_push_pull << GPIO_OTYPER_OT6_Pos);             // Настройка GPIOC пин 6 в режим Push-Pull
  MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED6, High_speed << GPIO_OSPEEDR_OSPEED6_Pos);        // Настройка GPIOC пин 6 в режим High_speed
  MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD6, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD6_Pos);     // Настройка GPIOC пин 6 в режим No_pull_up_No_pull_down
}
int main(void) {
  PortC_6_Init();
  DWT_Init();
  while (1) {
    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS6);
    DWT_Delay_ms(100);
    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR6);
    DWT_Delay_ms(100);
  }
}
// Инициализация DWT
void DWT_Init(void) {
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // Выключеник DWT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Включение DWT
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;            // Выключение счётчика
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Включение счетчика CYCCNT
  __ASM volatile("NOP");
  __ASM volatile("NOP");
  __ASM volatile("NOP");
}

void DWT_Delay_ms(volatile uint32_t au32_milliseconds) {
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (SystemCoreClock / 1000);
  au32_milliseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_milliseconds);
}

void DWT_Delay_us(volatile uint32_t au32_microseconds) {
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (SystemCoreClock / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds - au32_ticks);
}
/*************************** End of file ****************************/

//*************************** Пример второй **************************

/*************************** End of file ****************************/