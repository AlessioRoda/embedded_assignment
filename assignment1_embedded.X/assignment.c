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
#include "assignment.h"
#include <stdio.h>


//function used to set the prescaler
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

//set and wait into the same function
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


//function used to set timer for a specific time in ms
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



//once setted the timer wait for the flag
int tmr_wait_period(int timer)
{
    switch(timer){
    case TIMER1:
    {
        if(IFS0bits.T1IF==1){ //If when enter the timer is already expires
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


//function to put a character into SPI
void spi_put_char(char c){
    while(SPI1STATbits.SPITBF == 1); //polling until SPI buffer ready
    SPI1BUF = c;
}


//Function to put a sring into SPI 
void spi_put_string(char* str){
    int i;
    for( i=0;str[i] != '\0';i++){
        spi_put_char(str[i]);
    }
}
void spi_move_cursor(int row, int column){
    switch(row){
        case 0:{
            spi_put_char(0x80 + column);
            return;
        }
        case 1:{
            spi_put_char(0xC0 + column);
            return;
        }
    }
}

void spi_clear_first_row(){
    spi_move_cursor(FIRST_ROW, 0);
    int i=0;
    for(i=0;i<16;i++){
        spi_put_char(' ');
    }
}