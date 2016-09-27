#include "Platform_config.h"

static void delay_int_count(volatile unsigned int nTime){
    for(;nTime > 0; nTime--);
}

void delay_1_second(void){
    delay_int_count(806596);
}

void GPIO_B_Output_Init(uint16_t pinNum, GPIOSpeed_TypeDef speedValE, GPIOMode_Output_TypeDef modeE){
    uint32_t tmpVal = modeE << 2|speedValE;
    uint32_t pinpos = 0x00;

    //pin 0~7
    if((pinNum & 0xff) != 0){
        for(pinpos = 0x00 ; pinpos < 0x08 ; pinpos++){
            if(pinNum == ((uint32_t)0x01) << pinpos){
                pinpos = pinpos * 4;
                GPIOB_CRL &= ~(0xf << pinpos);
                GPIOB_CRL |= tmpVal << pinpos;
                break;
            }
        }
    }

    //pin 8~15
    else{
        pinNum = pinNum >> 8;
        for(pinpos = 0x00 ; pinpos < 0x08 ; pinpos++){
            if(pinNum == ((uint32_t)0x01) << pinpos){
                pinpos = pinpos * 4;
                GPIOB_CRH &= ~(0xf << pinpos);
                GPIOB_CRH |= tmpVal << pinpos;
                break;
            }
        }
    }
}

int main(void){
	RCC_APB2ENR|= RCC_APB2periph_GPIOB;

    GPIO_B_Output_Init(GPIO_LED1_PIN, GPIO_Speed_10Mhz, GPIO_Mode_Out_PP);
    GPIO_B_Output_Init(GPIO_LED2_PIN, GPIO_Speed_10Mhz, GPIO_Mode_Out_PP);
    GPIO_B_Output_Init(GPIO_LED3_PIN, GPIO_Speed_10Mhz, GPIO_Mode_Out_PP);

    LED_Test();
    while(1);
}