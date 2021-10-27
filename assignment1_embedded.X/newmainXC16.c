/*
 * File:   main.c
 * Authors: Ermanno Girardo, Alessio Roda, Enzo Ubaldo Petrocco
 *
 * Created on 26 ottobre 2021, 22.42
 */
////////////////////////////////////////////////////////////////////////////////
/*
 * This is the main file of the assignment
 * Requisites:
 * 1)Simulate an algorithm that needs 7 ms for its execution, and
 *    needs to work at 100 Hz.
 * 2)Read characters from UART and display the characters received
 *   on the first row of the LCD.
 * 3)When the end of the row has been reached, clear the first row
 *   and start writing again from the first row first column
 * 4)Whenever a CR ?nr? or LF ?nn? character is received, clear the first
 *    row
 * 5)On the second row, write ?Char Recv: XXX?, where XXX is the
 *   number of characters received from the UART2.
 * 6)Whenever button S5 is pressed, send the current number of chars
 *   received to UART2
 * 7)Whenever button S6 is pressed, clear the first row and reset the
 *   characters received counter
 */
////////////////////////////////////////////////////////////////////////////////

#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
//#include "p30F1010.h"
#include "assignment.h"

int body();

int main(void) {
    
    
    body();

    return 0;
}

int body()
{
    tmr_setup_period(TIMER1,10);
    if(IFS0bits.T1IF==1){ //If when enter the timer is alreadu expires
        IFS0bits.T1IF=0; //Set the flag to zero and return error
        return 1;
    }
    tmr_wait_ms(TIMER2,7);
    
    while(IFS0bits.T1IF==0);
    IFS0bits.T1IF=0;
    body();

    return 0;
}