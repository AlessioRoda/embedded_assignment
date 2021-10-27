/*
 * File:   main.c
 * Author: Alessio
 *
 * Created on 27 ottobre 2021, 22.49
 */


#include <stdio.h>
#include <stdlib.h>
#include "p30F1010.h"
#include "xc.h"
#include "assignment.h"

void body();

int main(void) {
    
    
    body();

    return 0;
}

void body()
{
    tmr_setup_period(TIMER1,10);
    if(IFS0bits.T1IF==1){ //If when enter the timer is alreadu expires
        IFS0bits.T1IF=0; //Set the flag to zero and return error
        return 1;
    }
    tmr_wait_ms(TIMER1,7);
    
    while(IFS0bits.T1IF==0);
    IFS0bits.T1IF=0;
    body();

}

