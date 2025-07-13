/**-------------------------------------------------------------------
 \date  13.07.2025
 *
 *   STM32G431CBU6
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |        PC.6| ---->  LED
 *  |            |
 *  |        PB.0| <----  Button
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 *\ authors        ScuratovaAnna
 *\ сode debugging ScuratovaAnna
 */
//**************************  Пример первый **************************
#include "stdbool.h"
#include "stm32g4xx.h"

volatile bool condition = false;

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

void PORTC_6_INIT_Led(void) {
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOCEN);                                                      // Запуск тактирования порта C
  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE6, General_purpose_output_mode << GPIO_MODER_MODE6_Pos); // Настройка GPIOC пин 6 на выход (output mode)
  MODIFY_REG(GPIOC->OTYPER, GPIO_OTYPER_OT6, Output_push_pull << GPIO_OTYPER_OT6_Pos);             // Настройка GPIOC пин 6 в режим Push-Pull
  MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED6, High_speed << GPIO_OSPEEDR_OSPEED6_Pos);        // Настройка GPIOC пин 6 в режим High_speed
  MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD6, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD6_Pos);     // Настройка GPIOC пин 6 в режим High_speed
}

void PORTB_0_INIT_Button(void) {
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN);                                                      // Запуск тактирования порта B
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE0, Input_mode << GPIO_MODER_MODE0_Pos);                  // Настройка GPIOB пин 0 на выход (Input_mode)
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD0, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD0_Pos);     // Настройка GPIOB пин 0 в режим No_pull_up_No_pull_down
}

int main(void) {
  PORTC_6_INIT_Led();
  PORTB_0_INIT_Button();
  while (1) {
    condition = READ_BIT(GPIOB->IDR, GPIO_IDR_IDR_0);
    (condition == true) ? (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS6)) : (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR6));
  }
}
/*************************** End of file ****************************/