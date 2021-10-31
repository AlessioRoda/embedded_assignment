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
#include "p30F4011.h"
#include "assignment.h"

#define FIRST_ROW 0
#define SECOND_ROW 1

//global variable to take in mind the position of the cursor
int cursor_count=0;
//global variable to take in mind the character received from UART2
int character_count=0;
//string global variable
char string[16];


//ISR for the UART
void __attribute__((__interrupt__,__auto_psv__)) _U1RXInterrupt(){
    IFS0bits.U1RXIF = 0; //turn off the flag
    char c = U2RXREG; //take data from UART register
    //if is received the two special character clear the first row
    if((c=='\r') || (c=='\n')){
        spi_clear_first_row();
        cursor_count=0;
    }
    spi_put_char(c);  //write the character on SPI
    cursor_count++;
    //if the end of the first row has been reached
    if(cursor_count==15){
        //clear the first row
        spi_clear_first_row();
        //uppdate the counter
        cursor_count=0;
        //move the cursor into the fisrt row first column
        spi_move_cursor(FIRST_ROW,0);
    }
}

//ISR for the UART2 
void __attribute__((__interrupt__,__auto_psv__)) _U2RXInterrupt(){
    IFS1bits.U2RXIF = 0; //turn off the flag
    character_count++;
    //move the cursor in second row first column
    spi_move_cursor(SECOND_ROW,0);
    //write "Char Recv:"
    spi_put_string("Char Recv:");
    sprintf(string,"%d",character_count);
    //then the number of character received
    spi_put_string(string);
}




int main(void) {
    //first need to wait 1 second
    tmr_wait_ms(TIMER1, 1000);
    
    ////////////////INITIALIZATION//////////////////////////////////////////////
    //I/O initilization                                                       //
    TRISBbits.TRISB0 = 0;                                                     //
    TRISBbits.TRISB1 = 0;                                                     //
    //SPI initialization                                                      //
    SPI1CONbits.PPRE = 0b11;  // setup the primary prescaler to 1:1           //
    SPI1CONbits.SPRE = 0b110; // setup the secundary prescaler to 2:1         //
    //The two instructions above are needed becuase SPI works up to 1MHz      //
    SPI1CONbits.MSTEN = 1; //master                                           //
    SPI1CONbits.MODE16 = 0; // 8 bits                                         //
    SPI1STATbits.SPIEN = 1; // enable                                         //
                                                                              //
                                                                              //
    //UART INITIALIZATION                                                     //             
    U1BRG = 11; //set the baud rate register: (7372800 / 4) / (16 * 9600)-1   //
    U1MODEbits.STSEL = 0; // 1 stop bit                                       //
    U1MODEbits.PDSEL = 0b00; // 8 bit no parity                               //
    U1MODEbits.UARTEN = 1; // uart enable                                     //
                                                                              //
    U1STAbits.UTXEN = 1; // unable transmission                               //
                                                                              //
    //UART2 Initialization                                                    //
                                                                              //             
    U2BRG = 11; //set the baud rate register: (7372800 / 4) / (16 * 9600)-1   //
    U2MODEbits.STSEL = 0; // 1 stop bit                                       //
    U2MODEbits.PDSEL = 0b00; // 8 bit no parity                               //
    U2MODEbits.UARTEN = 1; // uart enable                                     //
                                                                              //
    U2STAbits.UTXEN = 1; // unable transmission                               //
  //////////////////////////////////////////////////////////////////////////////
    
    IEC0bits.U1RXIE = 1; // enable interrupt for UART reception 
    IEC1bits.U2RXIE = 1;   //enable interrupt for UART2 reception
    //execute the main program
    body();

    return 0;
}

int body()
{
    tmr_setup_period(TIMER1,10);
    if(IFS0bits.T1IF==1){ //If when enter the timer is already expires
        IFS0bits.T1IF=0; //Set the flag to zero and return error
        return 1;
    }
<<<<<<< HEAD
    tmr_wait_ms(TIMER1,7); //
=======
    tmr_wait_ms(TIMER2,7);
>>>>>>> d5fdf9a84ab7a5c5e8fc64f8c6f0429fd7d7c60d
    
    while(IFS0bits.T1IF==0);
    IFS0bits.T1IF=0;
    body();

    return 0;
}