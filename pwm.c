#include "pwm.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "utils/uartstdio.h"
#include "driverlib/uart.h"

#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
//#include "lab3.c"

/*
int frequency=0;
int period=0;
int duty=0;
int onOff=0;
*/

void initPWM(int Freq)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

	period=( ROM_SysCtlClockGet() / Freq );

	ROM_TimerLoadSet( TIMER3_BASE, TIMER_A, period-1 );

	ROM_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	ROM_IntEnable(INT_TIMER3A);
	ROM_TimerEnable(TIMER3_BASE, TIMER_A);
	ROM_TimerIntRegister(TIMER3_BASE, TIMER_A, pwmHandler);
}

void pwmHandler(void)
{

	//clear Interrupt
	ROM_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	onOff^=0xff;

	//first run, time = duty, second, time = period - duty
	(onOff) ? ROM_TimerLoadSet( TIMER3_BASE, TIMER_A, duty ) : ROM_TimerLoadSet( TIMER3_BASE, TIMER_A, period-duty );

	//Write the state out
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, onOff);
}

void pwmSetDuty(int duty_cycle)
{
	if (duty_cycle > 1000)
	{
		duty_cycle = 1000;
	}
	duty_cycle=duty_cycle*period;
	duty=duty_cycle/1000;			//corresponds to a 0-1000 duty cycle
}
