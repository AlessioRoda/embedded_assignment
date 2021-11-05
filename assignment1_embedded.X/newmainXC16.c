/*
 * File:   main.c
 * Author: Alessio
 *
 * Created on 27 ottobre 2021, 22.49
 */


#include "xc.h"
#include <stdio.h>
#include <stdbool.h>
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
int buff[30]; //Circular buffer
int* W10 = (int*)&buff; //W10 points to the first element of buff
int* R10 = (int*)&buff; //R10 points to the first element of buff, to read the buffer

bool buttonS6 = false;


//ISR for the UART2 
void __attribute__((__interrupt__,__auto_psv__)) _U2RXInterrupt(){
    IFS1bits.U2RXIF = 0; //turn off the flag
    character_count++;
    
    //Take the character from the UART2 buffer
    char c = U2RXREG; //take data from UART register
    //Put into circular buffer
    *W10=(int)c;
    W10++;
}

//ISR when the button S5 is pressed
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){
    IFS0bits.INT0IF = 0; //turn off the flag
    //transmit to UART2 the string
    U2TXREG = character_count;
}

//ISR when the button S6 is pressed
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(){
    IFS1bits.INT1IF = 0; //turn off the flag
    //clear the first row
    spi_clear_first_row();
    //reset the character counter
    character_count = 0;
}

//main function
int main(void) {
   
    //first need to wait 1 second
    tmr_wait_ms(TIMER1, 1000);
    
    ////////////////INITIALIZATION//////////////////////////////////////////////
    int i; //Index buffer
    TRISD = 0;
    for(i = 0;i <= 29;i++) //Init buff
    buff[i] = NULL; 
    
    YMODSRT = (int)&buff; //YMODSRT points to the first element of buff
    YMODEND = (int)&buff+59; //YMODEND points to the end address of buff
    MODCON = (int)0x80AA; //Moduo address Y space
    
    //I/O initilizationfor buttons S5 S6                                      //
    TRISEbits.TRISE8 = 1; //button S5 as input                                //
    TRISDbits.TRISD0 = 1; //button S6 as input                                //                                                    //
    //SPI initialization                                                      //
    SPI1CONbits.PPRE = 0b11;  // setup the primary prescaler to 1:1           //
    SPI1CONbits.SPRE = 0b110; // setup the secundary prescaler to 2:1         //
    //The two instructions above are needed becuase SPI works up to 1MHz      //
    SPI1CONbits.MSTEN = 1; //master                                           //
    SPI1CONbits.MODE16 = 0; // 8 bits                                         //
    SPI1STATbits.SPIEN = 1; // enable                                         //
                                                                              //
                                                                              //                              //
                                                                              //
    //UART2 Initialization                                                    //
                                                                              //             
    U2BRG = 11; //set the baud rate register: (7372800 / 4) / (16 * 9600)-1   //
    U2MODEbits.STSEL = 0; // 1 stop bit                                       //
    U2MODEbits.PDSEL = 0b00; // 8 bit no parity                               //
    U2MODEbits.UARTEN = 1; // UART enable                                     //
                                                                              //
    U2STAbits.UTXEN = 1; // unable transmission                               //
    ////////////////////////////////////////////////////////////////////////////
    
    //////////Enable all the interrupts/////////////////////////////////////////
    IEC1bits.U2RXIE = 1;   //enable interrupt for UART2 reception             //
    IEC0bits.INT0IE = 1;   //enable interrupt for button S5                   //
    IEC1bits.INT1IE = 1;   //enable interrupt for button S6                   //
    ////////////////////////////////////////////////////////////////////////////
   
    //Put the cursor on the second row, column 0
    spi_move_cursor(SECOND_ROW,0);
    //in order to make the interrupt for UART2 faster
    //and not overwrite the string Char Recv
    spi_put_string("Char Recv:");
    //call the simulated algorithm
    
    while(1)
    {
        tmr_setup_period(TIMER1,10);
        tmr_wait_ms(TIMER2,7);
        
        IEC1bits.U2RXIE = 0; //Disable interrupt UART2
        
        for(;R10<=W10; R10++)
        {
            spi_move_cursor(FIRST_ROW,cursor_count);
                //if is received the two special character clear the first row
            if((*R10=='\r') || (*R10=='\n') || (cursor_count==15)){
                spi_clear_first_row();
                cursor_count=0;
            }

            
            else if(*R10!=NULL){
                char character = (char)*R10;
                spi_put_char(character);  //write the character on SPI
                cursor_count++;
                  
            }
        }
        
        //move the cursor in second row first column
        spi_move_cursor(SECOND_ROW,11);
        //write "Char Recv:"
        sprintf(string,"%d",character_count);
        //then the number of character received
        spi_put_string(string);
              
        IEC1bits.U2RXIE = 1;
               
        tmr_wait_period(TIMER1);
    }

    return 0;
}
