/*
 * Calibration.c
 *
 *  Created on: 28-Mar-2025
 *      Author: embdengr2
 */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "main_loop.h"
#include "Calibration.h"

int get_Adapter_value(int i)
{
  switch (i)
  {
    case 1:

    	//Power_volt = ((float)(4*pow(10,-7)*Power_watts*Power_watts*calibration_value)+(Power_watts * 0.0012 * calibration_value) +0.027);
    	Power_volt = ((float)(Power_watts * m_value *calibration_value) + c_value);//0.00115 ;0.056
    	if(mh=='G')
    	{
    		Power_volt = 1.0*Power_volt;
    	}
    	else
    	{
    		Power_volt = 1.2*Power_volt;
    	}
    	Power_DAC = (int)((float)Power_volt * 1241.212121); // convert to DAC value
       /* if (Power_watts<800)
               {
                  Power_DAC =(int)(Power_DAC * calibration_value_upto_7);//No calibration_value_upto_7 here it is set to 1 , so its a kind of dummy
               }
        else
             Power_DAC =(int)(Power_DAC * calibration_value_upto_7);*/
        if(Power_DAC > 4090)
             {
         	     Power_DAC = 4095;
             }
      break;

    case 2://LIO

    	Power_volt = ((float)(Power_watts * m_value *calibration_value) + c_value);
    	Power_DAC = (int)((float)Power_volt * 1241.212121); // convert to DAC value

        if(Power_DAC > 4090)
             {
         	     Power_DAC = 4095;
             }

      break;

    case 3://ENO

    	Power_volt = ((float)(Power_watts * m_value *calibration_value) + c_value);
    	Power_DAC = (int)((float)Power_volt * 1241.212121); // convert to DAC value

        if(Power_DAC > 4090)
             {
         	     Power_DAC = 4095;
             }

          break;

    default:
      Power_DAC = 0;
      break;
  }
  return Power_DAC;

}
