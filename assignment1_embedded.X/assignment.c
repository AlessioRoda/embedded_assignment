



#include "xc.h"
#include "assignment.h"
#include <stdio.h>
#include <stdlib.h>
//#include "p30F1010.h"

/*
void spi_put_char(char c){
    //polling until SPI buffer ready
    while(SPI1STATbits.SPITBF == 1); 
    SPI1BUF = c;
}

void spi_put_string(char* str){
    for(int i=0;str[i] != '\0';i++){
        spi_put_char(str[i]);
    }
}
*/

int choose_prescaler(int ms,int* pr,int* tckps)
{
    long ticks= (7372800/4L)/1000L*ms; //clock freql
    long ticks_no_presc=ticks;
    if(ticks<=65535)
    {
        *tckps=0;
        *pr=ticks;
        return 0;
    }
    //prescaler 1:8
    ticks=ticks_no_presc/8;
    if(ticks<=65535)
    {
        *tckps=1;
        *pr=ticks;
        return 0;
    }
    //prescaler 1:64
    ticks=ticks_no_presc/64;
    if(ticks<=65535)
    {
        *tckps=2;
        *pr=ticks;
        return 0;
    }
    //prescaler 256
    ticks=ticks_no_presc/256;
    if(ticks<=65535)
    {
        *tckps=3;
        *pr=ticks;
        return 0;
    }
    return 1;
}


int tmr_wait_ms(int timer, int ms)
{
    switch (timer){
    case TIMER1:
    {
        int pr,tckps;
        choose_prescaler(ms, &pr, &tckps);
        PR1=pr;
        T1CONbits.TCKPS = tckps;
        T1CONbits.TCS = 0;
        T1CONbits.TGATE = 0;

        T1CONbits.TON = 0;
        IFS0bits.T1IF = 0;
        TMR1 = 0;
        T1CONbits.TON = 1;
        while (IFS0bits.T1IF == 0);
            IFS0bits.T1IF = 0;
            T1CONbits.TON = 0;
        break;
    }
    case TIMER2:
    {
        int pr,tckps;
        choose_prescaler(ms, &pr, &tckps);
        PR2=pr;
        T2CONbits.TCKPS = tckps;
        T2CONbits.TCS = 0;
        T2CONbits.TGATE = 0;

        T2CONbits.TON = 0;
        IFS0bits.T2IF = 0;
        TMR2 = 0;
        T2CONbits.TON = 1;
        while (IFS0bits.T2IF == 0);
        IFS0bits.T2IF = 0;
        T2CONbits.TON = 0;
        break;
    }
    }
    return 0;
}



void tmr_setup_period(int timer,int ms)
{

    switch(timer){
    case TIMER1:
    {
        int pr,tckps;
        choose_prescaler(ms,&pr,&tckps);
        PR1=pr;//set the count
        T1CONbits.TCKPS=tckps; // set the prescaler
        T1CONbits.TCS=0; //set the internal clock
        T1CONbits.TGATE=0;
        TMR1=0; //reset timer 1
        T1CONbits.TON=1; //start the timer
        break;
    }
    case TIMER2:
    {
        int pr,tckps;
        choose_prescaler(ms,&pr,&tckps);
        PR2=pr;//set the count
        T2CONbits.TCKPS=tckps; // set the prescaler
        T2CONbits.TCS=0; //set the internal clock
        T2CONbits.TGATE=0;
        TMR2=0; //reset timer 2
        T2CONbits.TON=1; //start the timer
        break;
    }
    }
}




int tmr_wait_period(int timer)
{
    switch(timer){
    case TIMER1:
    {
        if(IFS0bits.T1IF==1){ //If when enter the timer is alreadu expires
        IFS0bits.T1IF=0; //Set the flag to zero and return error
        return 1;
        }
        while(IFS0bits.T1IF==0);
        IFS0bits.T1IF=0;
        break;
    }
    case TIMER2:
    {
        if(IFS0bits.T2IF==1){ //If when enter the timer is alreadu expires
        IFS0bits.T2IF=0; //Set the flag to zero and return error
        return 1;
        }
        while(IFS0bits.T2IF==0); //stay into the while until timer expires
        IFS0bits.T2IF=0;
        break;
    }
    }
    return 0;
}
