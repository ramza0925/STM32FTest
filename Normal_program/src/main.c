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

int main(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC -> APB2ENR |= RCC_APB2periph_GPIOA;
	RCC -> APB2ENR |= RCC_APB2periph_GPIOB;

    
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

    
    //LED_Test();
    Key_test();
    while(1);
}
