/*
===============================================================================
 Name        : EstacionRiego.c
 Author      : $ goltango@gmail.com
 Version     : 0.1
 Description : Implements a irrigation controller.
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#define LED2 22
#define T_CORTO 20
#define T_DISABLE 3600
#define RELAY_RIEGO 2

volatile int tickCounter=0;
volatile int flagLed=0;
volatile int adcResult = 0;
volatile int humedad = 0;
volatile int flagRiego = 0;


/**
 * Sends a char over UART2.
 * @param c char to send.
 */
void sendCharUart3(uint8_t c){
	while((LPC_UART3->LSR&(1<<5))==0){}
	LPC_UART3->THR=c;
}

/**
 * Sets a GPIO0 pin provided by argument
 * @param _pin pin to be set.
 */
void setGpio0(int _pin){
	LPC_GPIO0->FIOSET = (1 << _pin);
}

/**
 * Clears a GPIO0 pin provided by argument
 * @param _pin pin to be cleared.
 */
void clrGpio0(int _pin){
	LPC_GPIO0->FIOCLR = (1 << _pin);
}

/**
 * Make a togle on a specific GPIO0 pin.
 * @param _pin pin to togle.
 */
void togleGpio0Pin(int _pin){
	if(flagLed == 0){
		setGpio0(_pin);
		flagLed = 1;
	}else{
		clrGpio0(_pin);
		flagLed = 0;
	}
}

/**
 * ADC last coversion.
 * Inspects the ADDR0 register (AD0GDR) [15:4] searching
 * the result of last conversion.
 * @return the 12bit coversion value.
 */
uint32_t adcConvertion(void){
	return (LPC_ADC->ADDR0>>4)&0xfff;
}

/**
 * Percent conversion from a 0 to 4096 scale
 * @see testMe()
 * @param _input value to be converted.
 * @return percentage.
 */
int percentConversion(uint32_t _input){
	return _input * 100 / 4096;
}

/**
 * Timer0 basic configurations.
 * Two interrupts are enabled.
 * @param _tMR0 seconds to first interrupt.
 * @param _tMR1 seconds to second interrupt.
 */
void configurarTimer0(int _tMR0, int _tMR1){
	LPC_TIM0->CTCR=0;					//CTCR.0 = 0 Timer como contador de clocks
	LPC_TIM0->PR=25000000;				//Pre-scaler en 25M para una frecuencia de 25MHz

	LPC_TIM0->MR0= _tMR0;
	LPC_TIM0->MR1= _tMR1;

	LPC_TIM0->MCR|=(1<<2)|(1<<0);
	LPC_TIM0->MCR|=(1<<5)|(1<<4)|(1<<3);

	NVIC_EnableIRQ(TIMER0_IRQn);		//Configuracion de NVIC
	LPC_TIM0->TCR=1;					//Activa Timer0
}

/**
 * Activate the irrigation.
 * @see clrGpio0()
 * @see configurarTimer0()
 * @param _seg time to be open for valve.
 */
void activarRiego(int _seg){
	flagRiego = 1;
	clrGpio0(RELAY_RIEGO);				//Abre la valvula de riego
	configurarTimer0(_seg, T_DISABLE);	//Configura un retardo para cerrar la valvula
}

void enviarHumedadUart3(int _hum){
	sendCharUart3('H');
	sendCharUart3('u');
	sendCharUart3('m');
	sendCharUart3('e');
	sendCharUart3('d');
	sendCharUart3('a');
	sendCharUart3('d');
	sendCharUart3(':');
	sendCharUart3(' ');
	if( 0<=_hum && _hum <10) sendCharUart3('0');
	if(10<=_hum && _hum <20)	sendCharUart3('1');
	if(20<=_hum && _hum <30) sendCharUart3('2');
	if(30<=_hum && _hum <40) sendCharUart3('3');
	if(40<=_hum && _hum <50)	sendCharUart3('4');
	if(50<=_hum && _hum <60)	sendCharUart3('5');
	if(60<=_hum && _hum <70) sendCharUart3('6');
	if(70<=_hum && _hum <80) sendCharUart3('7');
	if(80<=_hum && _hum <90) sendCharUart3('8');
	if(90<=_hum && _hum <=100) sendCharUart3('9');
	sendCharUart3('0');
	sendCharUart3('%');
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);
	sendCharUart3(8);

}
/**
 * Cheks every second for humidity percentage, if necessary, irrigate and makes
 * UART communication.
 */
void SysTick_Handler(void) {
	tickCounter++;
	if(tickCounter < 25) clrGpio0(LED2);
	if(tickCounter == 100){
		tickCounter = 0;
		setGpio0(LED2);
		adcResult = adcConvertion();
		humedad = 100 - percentConversion(adcResult);
		enviarHumedadUart3(humedad);
		if(humedad < 40 && !flagRiego){
			activarRiego(T_CORTO);
		}
	}
}

/**
 * Close the valve if necessary and enable the irrigation sequence.
 */
void TIMER0_IRQHandler(void){

	if(LPC_TIM0->TC == T_CORTO){
		setGpio0(RELAY_RIEGO);	//cierra la valvula de riego
		LPC_TIM0->IR|=(1<<0);	//re-activa servicio de interrupcion
		LPC_TIM0->TCR=1;					//Activa Timer0
	}

	if(LPC_TIM0->TC == T_DISABLE){
		flagRiego = 0;
		LPC_TIM0->IR|=(1<<1);	//re-activa servicio de interrupcion
	}

}

/**
 * ADC configuration routine.
 */
void ADC_Config(){
	LPC_SC->PCONP |=(1<<12);
	LPC_ADC->ADCR|=1<<21;
	LPC_ADC->ADCR |=(1<<8);
	LPC_ADC->ADCR|=1<<16;			//habilitamos burst
	LPC_ADC->ADINTEN=0;				//bit 8 debe estar en cero para el modo burst.
	LPC_PINCON->PINMODE1 |= 1<<15;	//sin resistencias de pull up/down
	LPC_PINCON->PINSEL1 |= 1<<14;	//seleccionamos la función de conversor AD
}

/**
 * UART3 configuration routine.
 */
void UART3_Config(){
	LPC_SC->PCONP|=1<<25;	//Power Control for Pheriphals, Encendemos el periférico UART3
	LPC_UART3->LCR|=11;
	LPC_UART3->LCR|=(1<<7);
	LPC_UART3->DLL=0b10100001; //DLL = 161 -> 9585 baudios -> error de 14 -> 0.1458%
	LPC_UART3->DLM=0;
	LPC_UART3->LCR &= 0b01111111;
	LPC_UART3->IER=1; // Corresponde a la interrupción por Receive Data Available
	//Configuracion de NVIC
	NVIC_EnableIRQ(UART3_IRQn);
	//P0.0 como TX y P0.1 como RX
	LPC_PINCON->PINSEL0|=0b1010;
}

int main(void) {
	//Configuration and initialization
	SystemCoreClockUpdate();
	if (SysTick_Config(SystemCoreClock / 100)) {
		while (1);  // Capture error
	}
	ADC_Config();
	UART3_Config();

	//Board's LED2
	LPC_GPIO0->FIODIR |= (1 << LED2);
	//Board's GPIO0.0 config
	LPC_GPIO0->FIODIR |= (1 << RELAY_RIEGO);
	LPC_GPIO0->FIOSET |= (1 << RELAY_RIEGO);

	volatile int i = 0;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
    	i++;
    }
    return 0 ;
}
