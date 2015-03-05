#include "linkedqueue.h"

// processor definitions
#include "inc/lm4f120h5qr.h"


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
#include "inc/hw_gpio.h"	//for unlocking

#include "pwm.h"

#define PA_0 GPIO_PIN_0	//0x00000001
#define PA_1 GPIO_PIN_1	//0x00000002
#define PA_2 GPIO_PIN_2	//0x00000004
#define PA_3 GPIO_PIN_3	//0x00000008
#define PA_4 GPIO_PIN_4	//0x00000010
#define PA_5 GPIO_PIN_5	//0x00000020
#define PA_6 GPIO_PIN_6	//0x00000040
#define PA_7 GPIO_PIN_7	//0x00000080

#define PB_0 GPIO_PIN_0	//0x00000001
#define PB_1 GPIO_PIN_1	//0x00000002
#define PB_2 GPIO_PIN_2	//0x00000004
#define PB_3 GPIO_PIN_3	//0x00000008
#define PB_4 GPIO_PIN_4	//0x00000010
#define PB_5 GPIO_PIN_5	//0x00000020
#define PB_6 GPIO_PIN_6	//0x00000040
#define PB_7 GPIO_PIN_7	//0x00000080

#define PC_0 GPIO_PIN_0	//0x00000001
#define PC_1 GPIO_PIN_1	//0x00000002
#define PC_2 GPIO_PIN_2	//0x00000004
#define PC_3 GPIO_PIN_3	//0x00000008
#define PC_4 GPIO_PIN_4	//0x00000010
#define PC_5 GPIO_PIN_5	//0x00000020
#define PC_6 GPIO_PIN_6	//0x00000040
#define PC_7 GPIO_PIN_7	//0x00000080

#define PD_0 GPIO_PIN_0	//0x00000001
#define PD_1 GPIO_PIN_1	//0x00000002
#define PD_2 GPIO_PIN_2	//0x00000004
#define PD_3 GPIO_PIN_3	//0x00000008
#define PD_4 GPIO_PIN_4	//0x00000010
#define PD_5 GPIO_PIN_5	//0x00000020
#define PD_6 GPIO_PIN_6	//0x00000040
#define PD_7 GPIO_PIN_7	//0x00000080

#define PE_0 GPIO_PIN_0	//0x00000001
#define PE_1 GPIO_PIN_1	//0x00000002
#define PE_2 GPIO_PIN_2	//0x00000004
#define PE_3 GPIO_PIN_3	//0x00000008
#define PE_4 GPIO_PIN_4	//0x00000010
#define PE_5 GPIO_PIN_5	//0x00000020
#define PE_6 GPIO_PIN_6	//0x00000040
#define PE_7 GPIO_PIN_7	//0x00000080

#define PF_0 GPIO_PIN_0	//0x00000001
#define PF_1 GPIO_PIN_1	//0x00000002
#define PF_2 GPIO_PIN_2	//0x00000004
#define PF_3 GPIO_PIN_3	//0x00000008
#define PF_4 GPIO_PIN_4	//0x00000010
#define PF_5 GPIO_PIN_5	//0x00000020
#define PF_6 GPIO_PIN_6	//0x00000040
#define PF_7 GPIO_PIN_7	//0x00000080

#define redled GPIO_PIN_1
#define bluled GPIO_PIN_2
#define grnled GPIO_PIN_3

#define btn1 PF_0
#define btn2 PF_4


//ALEXCODE

//extern void initPWM(int);
//extern void pwmHandler(void);
//extern void pwmSetDuty(int);


int delay(int time);
unsigned long ulPeriod;
int count = 0;
int flip = 0;
int readValue = 0;
int qerror[2] = {0, 0};

extern void Timer0IntHandler(void);
extern void PortAIntHandler(void);
extern void PortFIntHandler(void);
extern void UART0IntHandler(void);
extern void UART4IntHandler(void);
void UARTsend(char*);
void bt_UARTsend(char*);
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);
void bt_UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);
void PortUnlock(void);

int main(void) {

	//enable clock, 200MHz/5 = 40MHz
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	
	//enable clock, use crystal at 1MHz
	//ROM_SysCtlClockSet(SYSCTL_RCC_XTAL_1MHZ | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	// enable PORT A,B,D,E,F GPIO and Timer0 peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA|SYSCTL_PERIPH_GPIOB|SYSCTL_PERIPH_GPIOC|SYSCTL_PERIPH_GPIOD|SYSCTL_PERIPH_GPIOE|SYSCTL_PERIPH_GPIOF);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	ROM_SysCtlDelay(200000);

	//unlock PD7
	PortUnlock();

	// set LED PORT D pins as outputs
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, (PD_0 | PD_1 | PD_2 | PD_3 | PD_4 | PD_5));
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, (PA_5 | PA_6 | PA_7));
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, (PB_4));
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, redled|bluled|grnled);
	
	// set Port A 2,3,4 and Port B 6,7 as inputs
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, (PA_2 | PA_3 | PA_4));
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, (PD_6 | PD_7));
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, (btn2));

	//set timer period: 40MHz/10/2 = 2MHz = 0.5us
	//for 1000us = 1ms use Div by 0.1 and then 4, 40MHz / 0.1 / 4 = 1ms
	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	ulPeriod = (ROM_SysCtlClockGet() / 1 / 1);		//for 1ms
	ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod - 1);

	//GPIO interrupt setup
	ROM_GPIOPinIntEnable(GPIO_PORTA_BASE, PA_2);		//added
	ROM_GPIOPinIntEnable(GPIO_PORTF_BASE, (btn2));

	ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, PA_2, GPIO_FALLING_EDGE);
	ROM_GPIOPadConfigSet(GPIO_PORTA_BASE, PA_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	ROM_GPIOIntTypeSet(GPIO_PORTF_BASE, (btn2), GPIO_FALLING_EDGE);
	ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, (btn2), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	
	//initial interrupt clear
	ROM_GPIOPinIntClear(GPIO_PORTA_BASE, PA_2);
	ROM_GPIOPinIntClear(GPIO_PORTF_BASE, (btn2));

	ROM_IntEnable(INT_GPIOA);	//works!!!!!
	ROM_IntEnable(INT_GPIOF);

	//set up UART 
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
	//usb virtual uart
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	//bluetooth uart
	ROM_GPIOPinConfigure(GPIO_PC4_U4RX);
	ROM_GPIOPinConfigure(GPIO_PC5_U4TX);
	
	//define ports and pins used for uart
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, (PA_0|PA_1));
	ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, (PC_4|PC_5));
	
	//enable uart interrupts
	ROM_IntEnable(INT_UART0);
	ROM_IntEnable(INT_UART4);
	
	//enable input and output interrupts for uart
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_TX);
	ROM_UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_TX);
	
	//set up uart speeds, virtual usb uses 115200, bluetooth uses 9600 (can change using AT commands)
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8|UART_CONFIG_PAR_NONE|UART_CONFIG_STOP_ONE));
	ROM_UARTConfigSetExpClk(UART4_BASE, ROM_SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8|UART_CONFIG_PAR_NONE|UART_CONFIG_STOP_ONE));

	//send test uart
	bt_UARTsend("\033[2J");				//clear screen
	bt_UARTsend("Initializing System....\r\n");


	//Timer interrupt setup
	ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	ROM_IntEnable(INT_TIMER0A);
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	
	//enable all interrupts
	ROM_IntMasterEnable();
	
	//init pins as low
	ROM_GPIOPinWrite(GPIO_PORTD_BASE, 0xff, 0);
	
	//initialize Queue
	if(!QueueInit()) bt_UARTsend("Queue Initialized\r\n");


	//ALEXCODE
	initPWM(500);
	pwmSetDuty(10);
	// loop forever
	while(1)
	{

	}
}

void Timer0IntHandler(void)
{
	//clear interrupt
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);	

	switch(count)
	{
		case 0:	{
				//bt_UARTsend("case 0\r\n");
				ROM_GPIOPinWrite(GPIO_PORTA_BASE, PA_7, (PA_7*Queue[count]));
				ROM_GPIOPinWrite(GPIO_PORTA_BASE, PA_6, (PA_6*Queue[count+1]));
			} break;
		case 1: {
				//bt_UARTsend("case 1\r\n");
				ROM_GPIOPinWrite(GPIO_PORTA_BASE, PA_5, (PA_5*Queue[count+1]));
				ROM_GPIOPinWrite(GPIO_PORTB_BASE, PB_4, (PB_4*Queue[count+2]));
			} break;
		case 2: {
				//bt_UARTsend("case 2\r\n");
				ROM_GPIOPinWrite(GPIO_PORTD_BASE, PD_3, (PD_3*Queue[count+2]));
				ROM_GPIOPinWrite(GPIO_PORTD_BASE, PD_2, (PD_2*Queue[count+3]));
			} break;
		case 3: {
				ROM_GPIOPinWrite(GPIO_PORTA_BASE, (PA_4|PA_5|PA_6|PA_7), 0x00);
				ROM_GPIOPinWrite(GPIO_PORTB_BASE, PB_4, 0x00);
				ROM_GPIOPinWrite(GPIO_PORTD_BASE, (PD_2|PD_3), 0x00);
			} break;
		default: break;
	
	}

	//reset counter value back to ulPeriod
	ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod-1);

	//bt_UARTsend("timer\n");
	flip ^= 0xff;
	count++;
	count %= 4;
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, bluled, flip);
	
}

void PortAIntHandler(void)
{
	//moved interrupt clear to end to prevent bouncing since it's too fast
	//ROM_GPIOPinIntClear(GPIO_PORTA_BASE, PA_2);

	ROM_GPIOPinWrite(GPIO_PORTF_BASE, redled, flip);
	
	//re-init readValue
	readValue = 0;
	
	//read from pins
	/*
	if(ROM_GPIOPinRead(GPIO_PORTA_BASE, PA_3)) ROM_GPIOPinWrite(GPIO_PORTD_BASE, PD_3, PD_3);
	if(ROM_GPIOPinRead(GPIO_PORTA_BASE, PA_4)) ROM_GPIOPinWrite(GPIO_PORTD_BASE, PD_2, PD_2);
	if(ROM_GPIOPinRead(GPIO_PORTD_BASE, PD_7)) ROM_GPIOPinWrite(GPIO_PORTD_BASE, PD_1, PD_1);
	if(ROM_GPIOPinRead(GPIO_PORTD_BASE, PD_6)) ROM_GPIOPinWrite(GPIO_PORTD_BASE, PD_0, PD_0);
	*/
	
	if(ROM_GPIOPinRead(GPIO_PORTD_BASE, PD_7)) readValue |= 0x100;
	if(ROM_GPIOPinRead(GPIO_PORTA_BASE, PA_4)) readValue |= 0x010;
	if(ROM_GPIOPinRead(GPIO_PORTA_BASE, PA_3)) readValue |= 0x001;
	

	switch(readValue)
	{
		case 0x110: break;	//high level nothing read
		case 0x010: 	qerror[0] = QueuePut(0);		//0x10 is actually a remove and discard, not a put
				qerror[1] = QueuePut(1);
				if(!qerror[0] | !qerror[1])
				{
					bt_UARTsend("In: 0x10\r\n");
					break;
				} else if((qerror[0] == -1) | (qerror[1] == -1))
				{
					bt_UARTsend("Queue Full\r\n");
					break;
				}
		case 0x101: break;
		case 0x001: 	qerror[0] = QueuePut(1);
				qerror[1] = QueuePut(0);
				if(!qerror[0] | !qerror[1])
				{
					bt_UARTsend("In: 0x01\r\n");
					break;
				} else if((qerror[0] == -1) | (qerror[1] == -1))
				{
					bt_UARTsend("Queue Full\r\n");
					break;
				}
		case 0x100: break;
		case 0x000: 	qerror[0] = QueuePut(0);
				qerror[1] = QueuePut(0);
				if(!qerror[0] | !qerror[1])
				{
					bt_UARTsend("In: 0x00\r\n");
					break;
				} else if((qerror[0] == -1) | (qerror[1] == -1))
				{
					bt_UARTsend("Queue Full\r\n");
					break;
				}
		case 0x111: break;
		case 0x011: 	qerror[0] = QueuePut(1);
				qerror[1] = QueuePut(1);
				if(!qerror[0] | !qerror[1])
				{
					bt_UARTsend("In: 0x11\r\n");
					break;
				} else if((qerror[0] == -1) | (qerror[1] == -1))
				{
					bt_UARTsend("Queue Full\r\n");
					break;
				}
		default: break;
	}

	ROM_SysCtlDelay(2000000);
	ROM_GPIOPinIntClear(GPIO_PORTA_BASE, PA_2);
}

void PortFIntHandler(void)
{
	ROM_GPIOPinIntClear(GPIO_PORTF_BASE, (btn2));
	//ROM_GPIOPinWrite(GPIO_PORTF_BASE, bluled, bluled);
	//ROM_GPIOPinWrite(GPIO_PORTF_BASE, grnled, flip);
	
	//clear leds
	ROM_GPIOPinWrite(GPIO_PORTA_BASE, 0xff, 0);
	ROM_GPIOPinWrite(GPIO_PORTD_BASE, 0xff, 0);

	bt_UARTsend("\033[2J");		//clear screen
}

void UART0IntHandler(void)
{
	unsigned long ulStatus;

	ulStatus = ROM_UARTIntStatus(UART0_BASE, true);
	ROM_UARTIntClear(UART0_BASE, ulStatus);
	while(ROM_UARTCharsAvail(UART0_BASE))
	{
		ROM_UARTCharPutNonBlocking(UART0_BASE, ROM_UARTCharGetNonBlocking(UART0_BASE));
	}
}

void UART4IntHandler(void)
{
	unsigned long ulStatus;

	ulStatus = ROM_UARTIntStatus(UART4_BASE, true);
	ROM_UARTIntClear(UART4_BASE, ulStatus);
	while(ROM_UARTCharsAvail(UART4_BASE))
	{
		ROM_UARTCharPutNonBlocking(UART4_BASE, ROM_UARTCharGetNonBlocking(UART4_BASE));
	}
}

void UARTsend(char* input)
{
	//wait till uart ready
	while(ROM_UARTBusy(UART0_BASE));
	while(*input != '\0')
		ROM_UARTCharPut(UART0_BASE, *input++);
	//ROM_SysCtlDelay(ROM_SysCtlClockGet()/2);
}

void bt_UARTsend(char* input)
{
	//wait till uart ready
	while(ROM_UARTBusy(UART4_BASE));
	while(*input != '\0')
		ROM_UARTCharPut(UART4_BASE, *input++);
	//ROM_SysCtlDelay(ROM_SysCtlClockGet()/2);
}

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
	while(ulCount--)
	{
		ROM_UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
	}
}

void PortUnlock(void)
{
	//
	// First open the lock and select the bits we want to modify in the GPIO commit register.
	//
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

}
