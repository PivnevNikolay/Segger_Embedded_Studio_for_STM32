/**-------------------------------------------------------------------
 \date  10.07.2025
 *
 *   STM32G431CBU6
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |        PC.6| ---->  LED
 *  |            |
 *  |            |
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 *\ authors        ScuratovaAnna
 *\ сode debugging ScuratovaAnna
 */
//**************************  Пример первый **************************
#include "stm32g4xx.h"

void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms * 1000; i++) {
    __NOP();    // No operation instruction
  }
}

void PORTC_6_INIT(void) {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

  GPIOC->MODER &= ~GPIO_MODER_MODE6;         // Clear mode bits
  GPIOC->MODER |= GPIO_MODER_MODE6_0;        // Output mode (01)
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT6;         // Push-pull
  GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED6;    // High speed
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD6;         // No pull-up/pull-down
}
int main(void) {
  PORTC_6_INIT();
  while (1) {
    GPIOC->BSRR |= GPIO_BSRR_BS6;
    delay_ms(500);
    GPIOC->BSRR |= GPIO_BSRR_BR6;
    delay_ms(500);
  }
}

/*************************** End of file ****************************/
//**************************  Пример второй **************************
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

void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms * 1000; i++) {
    __NOP(); // No operation instruction
  }
}

void PORTC_6_INIT(void) {
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOCEN);                                                      // Запуск тактирования порта C
  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE6, General_purpose_output_mode << GPIO_MODER_MODE6_Pos); // Настройка GPIOC пин 6 на выход (output mode)
  MODIFY_REG(GPIOC->OTYPER, GPIO_OTYPER_OT6, Output_push_pull << GPIO_OTYPER_OT6_Pos);             // Настройка GPIOC пин 6 в режим Push-Pull
  MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED6, High_speed << GPIO_OSPEEDR_OSPEED6_Pos);        // Настройка GPIOC пин 6 в режим High_speed
  MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD6, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD6_Pos);     // Настройка GPIOC пин 6 в режим High_speed
}

int main(void) {
  PORTC_6_INIT();
  while (1) {
    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS6);
    delay_ms(200);
    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR6);
    delay_ms(200);
  }
}
/*************************** End of file ****************************/