/*
 * adc_stuff.h
 *
 *  Created on: May 2, 2020
 *      Author: Itsuki Sakamoto
 */

#ifndef INC_ADC_STUFF_H_
#define INC_ADC_STUFF_H_

void ADC_Init(void);
int ADC_Read(int channel);
int Get_Luminosity(int ADC_Lumen);
int Get_Battery_Percentage(int ADC_Battery);
int Convert_Fahrenheit(float Temperature_C);
float Get_Temperature(int ADC_Temperature);

#endif /* INC_ADC_STUFF_H_ */
