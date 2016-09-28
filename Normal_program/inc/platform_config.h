#ifndef __PLATFORM_CONFIG_h__
#define __PLATFORM_CONFOG_h__

#include <stdint.h>

#define PERIPH_BASE         ((uint32_t)0x40000000)          //SRAM base address in the bit-band region

//Peripheral memory map
#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE      (PERIPH_BASE + 0x20000)

#define RCC_BASE            (AHBPERIPH_BASE + 0x1000)
#define GPIOA_BASE          (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE          (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE          (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE          (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE          (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE          (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE          (APB2PERIPH_BASE + 0x2000)

#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *)GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *)GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *)GPIOG_BASE)

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

#define _IO                volatile

//Custom Define in this HW
#define GPIO_LED            GPIOB
#define GPIO_LED1_PIN       GPIO_Pin_9 //RED
#define GPIO_LED2_PIN       GPIO_Pin_5 //Yellow
#define GPIO_LED3_PIN       GPIO_Pin_8 //Green or Blue

#define GPIO_KEY            GPIOA
#define GPIO_KEY1_PIN       GPIO_Pin_0  //left wkup
#define GPIO_KEY2_PIN       GPIO_Pin_1  //right user


typedef enum{
    GPIO_Speed_10Mhz =1,
    GPIO_Speed_2Mhz,
    GPIO_Speed_50Mhz
} GPIOSpeed_TypeDef;

typedef enum{
    GPIO_Mode_AIN = 0x0,
    GPIO_Mode_IN_FLOATING = 0x04,
    GPIO_Mode_IPD = 0x28,
    GPIO_Mode_IPU = 0x48,
    GPIO_Mode_Out_OD = 0x14,
    GPIO_Mode_Out_PP = 0x10,
    GPIO_Mode_AF_OD = 0x1C,
    GPIO_Mode_AF_PP = 0x18,
} GIOMode_TypeDef;

typedef enum{
    Bit_RESET =0,
    Bit_SET
} BitAction;

typedef struct{
    _IO uint32_t CR;
    _IO uint32_t CFGR;
    _IO uint32_t CIR;
    _IO uint32_t APB2RSTR;
    _IO uint32_t APB1RSTR;
    _IO uint32_t AHBENR;
    _IO uint32_t APB2ENR;
    _IO uint32_t APB1ENR;
    _IO uint32_t BDCR;
    _IO uint32_t CSR;
#ifdef STM32F10X_CL
    _IO uint32_t AHBRSTR;
    _IO uint32_t CFGR2;
#endif //STM32F10X_CL
} RCC_TypeDef;

typedef struct{
    _IO uint32_t CRL;
    _IO uint32_t CRH;
    _IO uint32_t IDR;
    _IO uint32_t ODR;
    _IO uint32_t BSRR;
    _IO uint32_t BRR;
    _IO uint32_t LCKR;
} GPIO_TypeDef;

typedef struct{
    uint16_t GPIO_Pin;
    GPIOSpeed_TypeDef GPIO_Speed;
    GIOMode_TypeDef GPIO_Mode;
} GPIO_InitTypeDef;

void LED_On_Red (void);
void LED_Off_Red (void);
void LED_On_Yellow (void);
void LED_Off_Yellow (void);
void LED_On_Blue (void);
void LED_Off_Blue (void);
void LED_On_All (void);
void LED_Off_All (void);

void LED_Test (void);
void Key_test(void);
void delay_1_second(void);
void delay_100_milli_second(void);

#endif