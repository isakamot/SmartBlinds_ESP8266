#include "main.h"

void ADC_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOA->MODER &= ~(3 << (2 * 4));

	GPIOA->MODER |= (3 << (2 * 4));			//Set PA4 to analog
	GPIOC->MODER |= (3 << (2 * 0));			//Set PC0 to analog
	GPIOC->MODER |= (3 << (2 * 1));			//Set PC1 to analog

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		//Enable ADC in RCC

	RCC->CR2 |= RCC_CR2_HSI14ON;

	while(!(RCC->CR2 & RCC_CR2_HSI14RDY));

	ADC1->CR |= ADC_CR_ADEN;

	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	while((ADC1->CR & ADC_CR_ADSTART));
}

int ADC_Read(int channel)
{
	ADC1->CHSELR = 0;
	ADC1->CHSELR |= 1 << channel;
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	ADC1->CR |= ADC_CR_ADSTART;
	while(!(ADC1->ISR & ADC_ISR_EOC));
	int x = ADC1->DR;
	return x;
}

int Get_Luminosity(int ADC_Lumen)
{
	//Returns 1 if light
	//Returns 0 if dark
	float Vcc = 5;
	float inputVoltage = ADC_Lumen;
	float Vo = (inputVoltage / 4096) * Vcc;

	if(Vo < 2.5)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

int Get_Battery_Percentage(int ADC_Battery)
{
	//float TrueVoltage = ADC_Battery * 2;
	float TrueVoltage = ((float) ADC_Battery / 4096) * 3;
	float Percentage = ((TrueVoltage) / 1.56) * 100;
	if (Percentage > 100)
	{
		return 100;
	}
	else
	{
		return Percentage;
	}
}

int Convert_Fahrenheit(float Temperature_C)
{
	float Temperature_F = round(Temperature_C * 1.8 + 32);
	return (int) Temperature_F;
}

float Get_Temperature(int ADC_Temperature)
{
	float Resistor = 22000; //10 kOhm resistors used
	float Vcc = 3;	//Vcc = 3V
	float Photocell;
	float inputVoltage = ADC_Temperature;
	float Vo = (inputVoltage/4096) * Vcc;
	Photocell = ((3 * Resistor) / Vo) - Resistor;
	float Resistance_Rx = Photocell - 20000;	//Find change in resistance from 25 degrees celcius
	//Temperature coefficient = -4.4% (20,000 * 0.044 = 880)/1 degree Celcius
	float Temperature_Rx = Resistance_Rx / (880);
	if(Temperature_Rx < 0)
	{
		return 25 + (-1 * Temperature_Rx);
	}
	else
	{
		return 25 - Temperature_Rx;
	}
}
