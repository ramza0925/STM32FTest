/*
 * (C) COPYRIGHT 2009 CRZ
 *
 * File Name : key.c
 * Author    : POOH
 * Version   : V1.0
 * Date      : 08/20/2009
 */

/* Includes */

#include "hw_config.h"

/* functions */

void KEY_Test (void)
{
    uint32_t i = 0;

    LED_Off_All();

    while(1)
    {
        delay_100_milli_second();

        if((i++ & 0x1) == 0x0)
        {
            LED_On_Blue();
        }
        else
        {
            LED_Off_Blue();
        }

        if(GPIO_ReadInputDataBit(GPIO_KEY, GPIO_KEY1_PIN) == Bit_SET)
        {
            LED_On_Red();
        }
        else
        {
            LED_Off_Red();
        }

        if(GPIO_ReadInputDataBit(GPIO_KEY, GPIO_KEY2_PIN) == Bit_SET)
        {
            LED_On_Yellow();
        }
        else
        {
            LED_Off_Yellow();
        }
    }
}

