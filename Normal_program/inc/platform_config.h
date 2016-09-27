#ifndef __PLATFORM_CONFIG_h__
#define __PLATFORM_CONFOG_h__

#include <stdint.h>

#define GPIOB_BSRR          (*(volatile unsigned *)0x40010C10)
#define GPIOB_BRR           (*(volatile unsigned *)0x40010C14)
#define RCC_APB2ENR         (*(volatile unsigned *)0x40021018)
#define GPIOB_CRL           (*(volatile unsigned *)0x40010C00)
#define GPIOB_CRH           (*(volatile unsigned *)0x40010C04)

#define RCC_APB2periph_AFIO     ((uint32_t)0x00000001)
#define RCC_APB2periph_GPIOA    ((uint32_t)0x00000004)
#define RCC_APB2periph_GPIOB    ((uint32_t)0x00000008)
#define RCC_APB2periph_GPIOC    ((uint32_t)0x00000010)
#define RCC_APB2periph_GPIOD    ((uint32_t)0x00000020)
#define RCC_APB2periph_GPIOE    ((uint32_t)0x00000040)
#define RCC_APB2periph_GPIOF    ((uint32_t)0x00000080)
#define RCC_APB2periph_GPIOG    ((uint32_t)0x00000100)
#define RCC_APB2periph_ADC1     ((uint32_t)0x00000200)
#define RCC_APB2periph_ADC2     ((uint32_t)0x00000400)
#define RCC_APB2periph_TIM1     ((uint32_t)0x00000800)
#define RCC_APB2periph_SPI1     ((uint32_t)0x00001000)
#define RCC_APB2periph_TIM8     ((uint32_t)0x00002000)
#define RCC_APB2periph_USART1   ((uint32_t)0x00004000)
#define RCC_APB2periph_ADC3     ((uint32_t)0x00008000)

#define GPIO_Pin_0          ((uint16_t)0x0001)  //Pin 0 Selected
#define GPIO_Pin_1          ((uint16_t)0x0002)  //Pin 1 Selected
#define GPIO_Pin_2          ((uint16_t)0x0004)  //Pin 2 Selected
#define GPIO_Pin_3          ((uint16_t)0x0008)  //Pin 3 Selected
#define GPIO_Pin_4          ((uint16_t)0x0010)  //Pin 4 Selected
#define GPIO_Pin_5          ((uint16_t)0x0020)  //Pin 5 Selected
#define GPIO_Pin_6          ((uint16_t)0x0040)  //Pin 6 Selected
#define GPIO_Pin_7          ((uint16_t)0x0080)  //Pin 7 Selected
#define GPIO_Pin_8          ((uint16_t)0x0100)  //Pin 8 Selected
#define GPIO_Pin_9          ((uint16_t)0x0200)  //Pin 9 Selected
#define GPIO_Pin_10         ((uint16_t)0x0400)  //Pin 10 Selected
#define GPIO_Pin_11         ((uint16_t)0x0800)  //Pin 11 Selected
#define GPIO_Pin_12         ((uint16_t)0x1000)  //Pin 12 Selected
#define GPIO_Pin_13         ((uint16_t)0x2000)  //Pin 13 Selected
#define GPIO_Pin_14         ((uint16_t)0x4000)  //Pin 14 Selected
#define GPIO_Pin_15         ((uint16_t)0x8000)  //Pin 15 Selected
#define GPIO_Pin_All        ((uint16_t)0xffff)  //Pin ALL Selected

#define GPIO_LED1_PIN       GPIO_Pin_9 //RED
#define GPIO_LED2_PIN       GPIO_Pin_5 //Yellow
#define GPIO_LED3_PIN       GPIO_Pin_8 //Green or Blue

typedef enum{
    GPIO_Speed_10Mhz = 1,
    GPIO_Speed_2Mhz,
    GPIO_Speed_50Mhz
} GPIOSpeed_TypeDef;

typedef enum{
    GPIO_Mode_Out_PP = 0x00,
    GPIO_Mode_Out_OD = 0x01,
    GPIO_Mode_AF_PP = 0x10,
    GPIO_Mode_AF_OD = 0x11
} GPIOMode_Output_TypeDef;

void LED_Test (void);
void delay_1_second(void);
#endif