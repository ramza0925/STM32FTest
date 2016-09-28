#include "Platform_config.h"

static void delay_int_count(volatile unsigned int nTime){
    for(;nTime > 0; nTime--);
}

void delay_1_second(void){
    delay_int_count(806596);
}

void delay_100_milli_second(void){
    delay_int_count(80660);
}

void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct){
    uint32_t currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
    uint32_t tmpreg = 0x00, pinmask = 0x00;

    /*-------------------- GPIO Mode Configuration ---------------------*/
    currentmode = ((uint32_t)GPIO_InitStruct ->GPIO_Mode) & ((uint32_t)0x0f);
    if((((uint32_t)GPIO_InitStruct ->GPIO_Mode) & ((uint32_t)0x10)) != 0x00){
        //Output mode
        currentmode |= (uint32_t)GPIO_InitStruct ->GPIO_Speed;
    }

     /*-------------------- GPIO CRL Configuration ---------------------*/
     //Configuration the eight low port pins
     if(((uint32_t)GPIO_InitStruct ->GPIO_Pin & ((uint32_t)0x00ff)) != 0x00){
        tmpreg = GPIOx->CRL;
        for(pinpos = 0x00; pinpos < 0x08; pinpos++){
            pos = ((uint32_t)0x01) << pinpos;
            //Get the port pins position
            currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;
            if(currentpin == pos){
                pos = pinpos << 2;
                //Clear the corresponding low control register bits
                pinmask = ((uint32_t)0x0f) << pos;
                tmpreg &= ~pinmask;
                //Write the mode configuration in the corresponding bits
                tmpreg |= (currentmode << pos);
                //Reset the corresponding ODR bit
                if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD){
                    GPIOx->BRR = (((uint32_t)0x01) << pinpos);
                }
                else{
                    //Set the correspoinding ODR bit
                    if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU){
                        GPIOx->BSRR = (((uint32_t)0x01) << pinpos);
                    }
                }
            }
        }

        GPIOx->CRL = tmpreg;
     }

     /*-------------------- GPIO CRH Configuration ---------------------*/
     //Configuration the eight high port pins
     if(GPIO_InitStruct ->GPIO_Pin > 0x00ff){
        tmpreg = GPIOx->CRH;
        for(pinpos = 0x00; pinpos < 0x08; pinpos++){
            pos = (((uint32_t)0x01) << (pinpos + 0x08));
            //Get the port pins position
            currentpin = ((GPIO_InitStruct->GPIO_Pin) & pos);
            if(currentpin == pos){
                pos = pinpos << 2;
                //Clear the corresponding low control register bits
                pinmask = ((uint32_t)0x0f) << pos;
                tmpreg &= ~pinmask;
                //Write the mode configuration in the corresponding bits
                tmpreg |= (currentmode << pos);
                //Reset the corresponding ODR bit
                if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD){
                    GPIOx->BRR = (((uint32_t)0x01) << (pinpos + 0x08));
                }
                else{
                    //Set the correspoinding ODR bit
                    if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU){
                        GPIOx->BSRR = (((uint32_t)0x01) << (pinpos + 0x08));
                    }
                }
            }
        }

        GPIOx->CRH = tmpreg;
     }
}

void GPIO_Configuration(void){
    GPIO_InitTypeDef GPIO_InitStructure;

    //UART Configuration...

    //Configure USARTx_Tx as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_USART_Tx_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50Mhz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIO_USART, &GPIO_InitStructure);

    //Configure USARTx_Rx as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_USART_Rx_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIO_USART, &GPIO_InitStructure);

    /* Configure gpio as input : Button Left-WKUP */
    GPIO_InitStructure.GPIO_Pin = GPIO_KEY1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIO_KEY, &GPIO_InitStructure);
    
    /* Configure gpio as input : Button Right-USER */
    GPIO_InitStructure.GPIO_Pin = GPIO_KEY2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIO_KEY, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_LED1_PIN | GPIO_LED2_PIN | GPIO_LED3_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50Mhz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIO_LED, &GPIO_InitStructure);
}

void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct){
    uint32_t tmpreg = 0x00, apbclock = HSI_Value;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;

    //---------------USART CR2 Configuration
    tmpreg = USARTx->CR2;
    //Clear STOP[13:12] bits
    tmpreg &= CR2_STOP_CLEAR_Mask;
    //Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit
    //Set STOP[13:12] bits according to USART_StopBits value
    tmpreg |= (uint32_t)USART_InitStruct->USART_StopBits;
    //Write to USART CR2
    USARTx->CR2 = (uint16_t)tmpreg;

    //---------------USART CR1 Configuration
    tmpreg = USARTx->CR1;
    //Clear M, PCE, PS, TE and RE bits
    tmpreg &= CR1_CLEAR_Mask;
    //Configure the USART Word Length, Parity and mode
    //Set the M bits according to USART_WordLength value
    //Set PCE and PS bits according to USART_Parite value
    //Set TE and RE bits according to USART_Mode value
    tmpreg |= (uint32_t)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity | USART_InitStruct->USART_Mode;
    //Write to USART CR1
    USARTx->CR1 = (uint16_t)tmpreg;

    //---------------USART CR3 Configuration
    tmpreg = USARTx->CR3;
    //Clear CTSE and RESE bits
    tmpreg &= CR3_CLEAR_Mask;
    //Configure the USART HFC
    //Set CTSE and RTSE bits according to USART_HardwareFlowControl value
    tmpreg |= USART_InitStruct->USART_HardwareFlowControl;
    //Write to USART CR3
    USARTx->CR3 = tmpreg;

    //-------------------USART BRR Configuration
    //Configure the USART Baud Rate
    //Determine the integer part
    integerdivider = ((0x19*apbclock) / (0x4*(USART_InitStruct->USART_BaudRate)));
    tmpreg = (integerdivider / 0x64) << 0x04;
    //Determine the fractional part
    fractionaldivider = integerdivider - (0x64*(tmpreg >> 0x04));
    tmpreg |= ((((fractionaldivider*0x10) + 0x32) / 0x64))&((uint8_t)0x0f);
    //Write to USART BRR
    USARTx->BRR = tmpreg;
    
}

void USART1_Init(void){
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    USART1->CR1 |= CR1_UE_Set;
}

void USART_SendData(USART_TypeDef* USARTx, uint16_t Data){
    //Transmit Data
    USARTx->DR = (Data & (uint16_t)0x01ff);
}

FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG){
    FlagStatus bitstatus = RESET;

    if((USARTx->SR & USART_FLAG) != (uint16_t)RESET){
        bitstatus = SET;
    }
    else{
        bitstatus = RESET;
    }
    return bitstatus;
}

void SerialPutChar(uint8_t c){
    USART_SendData(USART1, c);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_PutString(uint8_t *s){
    while(*s != '\0'){
        SerialPutChar(*s);
        s++;
    }
}

int main(void){
    
    
    RCC->APB2ENR |= RCC_APB2periph_GPIOA;
	RCC->APB2ENR |= RCC_APB2periph_GPIOB;
    RCC->APB2ENR |= RCC_APB2periph_USART1;

    GPIO_Configuration();

    USART1_Init();

    Serial_PutString("\r\nHello World! Hello Cortex-M3!\r\n");

    
    //LED_Test();
    Key_test();
    while(1);
}
