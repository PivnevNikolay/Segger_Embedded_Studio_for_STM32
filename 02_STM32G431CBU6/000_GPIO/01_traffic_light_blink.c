/**-------------------------------------------------------------------
 \date  10.07.2025
 *
 *  traffic light blink
 *
 *   STM32G431CBU6
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |        PA.0| ---->  LED R
 *  |        PA.1| ---->  LED Y
 *  |        PA.2| ---->  LED G
 *  |            |
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 * ссылка на модуль который использую в примере(LED светофорный модуль): 
 * https://aliexpress.ru/item/32881664301.html?spm=a2g2w.orderdetail.0.0.3ea24aa6tVe7MA&sku_id=65632741499&_ga=2.38745450.837217396.1752098094-1084817420.1750673554
 *\ authors        ScuratovaAnna
 *\ сode debugging ScuratovaAnna
 */
//**************************  Пример первый **************************
#include "stm32g4xx.h"

typedef uint32_t  u32;
typedef uint16_t  u16;
typedef uint8_t   u8;

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

volatile u8 CW[3]={0x1,0x2,0x4};

void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms * 1000; i++) {
    __NOP(); 
  }
}

void GPIO_INIT(void) {
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);                                                      // Запуск тактирования порта A

  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE0, General_purpose_output_mode << GPIO_MODER_MODE0_Pos); // Настройка GPIOA пин 0 на выход (output mode)
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE1, General_purpose_output_mode << GPIO_MODER_MODE1_Pos); // Настройка GPIOA пин 1 на выход (output mode)
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE2, General_purpose_output_mode << GPIO_MODER_MODE2_Pos); // Настройка GPIOA пин 2 на выход (output mode)

  MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT0, Output_push_pull << GPIO_OTYPER_OT0_Pos);             // Настройка GPIOA пин 0 в режим Push-Pull
  MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT1, Output_push_pull << GPIO_OTYPER_OT1_Pos);             // Настройка GPIOA пин 1 в режим Push-Pull
  MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT2, Output_push_pull << GPIO_OTYPER_OT2_Pos);             // Настройка GPIOA пин 2 в режим Push-Pull

  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED0, High_speed << GPIO_OSPEEDR_OSPEED0_Pos);        // Настройка GPIOA пин 0 в режим High_speed
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED1, High_speed << GPIO_OSPEEDR_OSPEED1_Pos);        // Настройка GPIOA пин 1 в режим High_speed
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED2, High_speed << GPIO_OSPEEDR_OSPEED2_Pos);        // Настройка GPIOA пин 2 в режим High_speed

  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD0, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD0_Pos);     // Настройка GPIOA пин 0 в режим No_pull_up_No_pull_down
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD1, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD1_Pos);     // Настройка GPIOA пин 1 в режим No_pull_up_No_pull_down
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD2, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD2_Pos);     // Настройка GPIOA пин 2 в режим No_pull_up_No_pull_down
}

int main(void) {
  GPIO_INIT();
  while (1) {
   for (u8 i=0;i<3;i++){
    GPIOA->ODR = *(CW+i);
    delay_ms(800);
    }
  }
}
/*************************** End of file ****************************/