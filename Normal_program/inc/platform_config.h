#ifndef __PLATFORM_CONFIG_h__
#define __PLATFORM_CONFOG_h__

#include <stdint.h>
#include <stdio.h>

#define PERIPH_BASE         ((uint32_t)0x40000000)          //SRAM base address in the bit-band region

//Peripheral memory map
#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE      (PERIPH_BASE + 0x20000)

#define RCC_BASE            (AHBPERIPH_BASE + 0x1000)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)

#define GPIOA_BASE          (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE          (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE          (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE          (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE          (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE          (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE          (APB2PERIPH_BASE + 0x2000)
#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *)GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *)GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *)GPIOG_BASE)

#define USART1_BASE         (APB2PERIPH_BASE + 0x3800)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define GPIO_USART          GPIOA
#define GPIO_USART_Rx_Pin   GPIO_Pin_10
#define GPIO_USART_Tx_Pin   GPIO_Pin_9

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

#define USART_WordLength_8b                  ((uint16_t)0x0000)
#define USART_WordLength_9b                  ((uint16_t)0x1000)

#define USART_StopBits_1                     ((uint16_t)0x0000)
#define USART_StopBits_0_5                   ((uint16_t)0x1000)
#define USART_StopBits_2                     ((uint16_t)0x2000)
#define USART_StopBits_1_5                   ((uint16_t)0x3000)

#define USART_Parity_No                      ((uint16_t)0x0000)
#define USART_Parity_Even                    ((uint16_t)0x0400)
#define USART_Parity_Odd                     ((uint16_t)0x0600) 

#define USART_HardwareFlowControl_None       ((uint16_t)0x0000)
#define USART_HardwareFlowControl_RTS        ((uint16_t)0x0100)
#define USART_HardwareFlowControl_CTS        ((uint16_t)0x0200)
#define USART_HardwareFlowControl_RTS_CTS    ((uint16_t)0x0300)

#define USART_Mode_Rx                        ((uint16_t)0x0004)
#define USART_Mode_Tx                        ((uint16_t)0x0008)
#define USART_FLAG_TXE                       ((uint16_t)0x0080) 
#define USART_FLAG_RXNE                      ((uint16_t)0x0020)

#define HSI_Value                 ((uint32_t)8000000)

#define CR1_UE_Set                ((uint16_t)0x2000)  /*!< USART Enable Mask */

#define CR1_CLEAR_Mask            ((uint16_t)0xE9F3)  /*!< USART CR1 Mask */
#define CR2_STOP_CLEAR_Mask       ((uint16_t)0xCFFF)  /*!< USART CR2 STOP Bits Mask */
#define CR3_CLEAR_Mask            ((uint16_t)0xFCFF)  /*!< USART CR3 Mask */
#define _IO                       volatile

//Custom Define in this HW
#define GPIO_LED            GPIOB
#define GPIO_LED1_PIN       GPIO_Pin_9 //RED
#define GPIO_LED2_PIN       GPIO_Pin_5 //Yellow
#define GPIO_LED3_PIN       GPIO_Pin_8 //Green or Blue

#define GPIO_KEY            GPIOA
#define GPIO_KEY1_PIN       GPIO_Pin_0  //left wkup
#define GPIO_KEY2_PIN       GPIO_Pin_1  //right user

#define GPIO_7_SEG              GPIOC
#define GPIO_7_SEG_POWER_PIN    GPIO_Pin_8
#define GPIO_7_SEG_A_PIN        GPIO_Pin_0
#define GPIO_7_SEG_B_PIN        GPIO_Pin_1
#define GPIO_7_SEG_C_PIN        GPIO_Pin_2
#define GPIO_7_SEG_D_PIN        GPIO_Pin_3
#define GPIO_7_SEG_E_PIN        GPIO_Pin_4
#define GPIO_7_SEG_F_PIN        GPIO_Pin_5
#define GPIO_7_SEG_G_PIN        GPIO_Pin_7
#define GPIO_7_SEG_DP_PIN       GPIO_Pin_6

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

typedef enum
{
    RESET = 0,
    SET = !RESET
} FlagStatus;

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

typedef struct{
    _IO uint16_t SR;
    uint16_t RESERVED0;
    _IO uint16_t DR;
    int16_t RESERVED1;
    _IO uint16_t BRR;
    int16_t RESERVED2;
    _IO uint16_t CR1;
    int16_t RESERVED3;
    _IO uint16_t CR2;
    int16_t RESERVED4;
    _IO uint16_t CR3;
    int16_t RESERVED5;
    _IO uint16_t GTPR;
    int16_t RESERVED6;
} USART_TypeDef;

typedef struct
{
  uint32_t USART_BaudRate;            /*!< This member configures the USART communication baud rate.
                                           The baud rate is computed using the following formula:
                                            - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
                                            - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 16) + 0.5 */

  uint16_t USART_WordLength;          /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref USART_Word_Length */

  uint16_t USART_StopBits;            /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref USART_Stop_Bits */

  uint16_t USART_Parity;              /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref USART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */
 
  uint16_t USART_Mode;                /*!< Specifies wether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_Mode */

  uint16_t USART_HardwareFlowControl; /*!< Specifies wether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref USART_Hardware_Flow_Control */
} USART_InitTypeDef;

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
void Seven_Segment_Test(void);
void delay_1_second(void);
void delay_100_milli_second(void);


#endif