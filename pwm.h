#ifndef DELAY_H_
#define DELAY_H_
#endif

int frequency=0;
int period=0;
int duty=0;
int onOff=0;

void initPWM(int);
void pwmHandler(void);
void pwmSetDuty(int);
