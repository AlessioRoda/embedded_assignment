/*
 * File:   newmainXC16.c
 * Author: aless
 *
 * Created on 30 dicembre 2021, 10.32
 */

#include <xc.h>

#include <stdio.h>
#include "stdlib.h"
#include "assignment.h"
#include "parser.h"
#include <string.h>


#define MAX_TASKS 4  //DA CAMBIARE!!!
#define CIRCULAR_BUFFER_SIZE 15

typedef struct {
int n;
int N;
} heartbeat;


typedef struct {
    char buffer[CIRCULAR_BUFFER_SIZE];
    int readIndex;
    int writeIndex;
} circular_buffer_t;

volatile circular_buffer_t circularBuffer;



void write_cb(volatile circular_buffer_t* cb, char byte) {
    cb->buffer[cb->writeIndex] = byte;
    cb->writeIndex = (cb->writeIndex + 1) % CIRCULAR_BUFFER_SIZE;
    if (cb->readIndex == cb->writeIndex) {
        // full buffer
        cb->readIndex++; // discard the oldest byte
    }
}

void read_cb(volatile circular_buffer_t* cb, char* byte) {
    if (cb->readIndex != cb->writeIndex) {
        *byte = cb->buffer[cb->readIndex];
        cb->readIndex = (cb->readIndex + 1) % CIRCULAR_BUFFER_SIZE;
    }
}

int avl_bytes_cb(volatile circular_buffer_t* cb) {
    if (cb->readIndex <= cb->writeIndex) {
        return cb->writeIndex - cb->readIndex;
    } else {
        return CIRCULAR_BUFFER_SIZE - cb->readIndex + cb->writeIndex;
    }
}


//ISR for the UART2 
void __attribute__((__interrupt__,__auto_psv__)) _U2RXInterrupt(){
    IFS1bits.U2RXIF = 0; //turn off the flag
    
    //Write on the circular buffer the current character from UART
    while (U2STAbits.URXDA == 1) {
        write_cb(&circularBuffer, U2RXREG);
    }
}


heartbeat schedInfo[MAX_TASKS];
    void scheduler() {
    int i ;
    int executed = 0;
    for ( i = 0; i <MAX_TASKS; i++) {
    schedInfo[i ]. n++;
    if (schedInfo[i]. n >= schedInfo[i].N) {
    switch(i) {
    case 0:
    // task1() ;
     break;
    case 1:
    // task2() ;
        break;
    case 2:
       // task3() ;
        break;
    }
    schedInfo[i]. n = 0;
    }
    }
}

    int extract_message(const char* str, int* n1, int* n2)
    {
        int len= strlen(str);
        char* string1;
        char* string2;
        
        int i=0;
        int j=0;
        int flag =1;
        for(;i<len; i++)
        {
            if(str[i]==',')
            {
                flag= extract_integer(string1, n1);
                
                if(flag==-1)
                {
                    return -1;
                }
                j=i;
            }
            
            if(str[i]=='\0')
            {
                flag= extract_integer(string2, n2);
                
                if(flag==-1)
                {
                    return -1;
                }
                
            }
            
            if(flag==1)
            {
                string1[i]=str[i];
            }
            else
            {
                string2[i-j]=str[i];
            }
        }
        
        return 0;
    }
    

int main(void) {
    
    //first need to wait 1 second
    tmr_wait_ms(TIMER1, 1000);
    
    float duty_cycle;
    int velocity_r; 
    int velocity_l;
    int avl;
    int count;
    int ret;
    
    
    //Initialize PWM////////////////////////////////////////////////////////////
    //PTPER = 1842; // 1 kHz
    PTCONbits.PTMOD = 0; // free running
    PTCONbits.PTCKPS = 0; // 1:1 prescaler
    //PTCONbits.PTCKPS = 1; // 1:4 prescaler
    PWMCON1bits.PEN2H = 1;
    //PWMCON1bits.PEN2L = 1;
    //NOTE THAT:
    //PTPER should be 920.6
    //1843,2 / 2 - 1 = 920.6
    //This introduce a computational error
    //Then when duty cycle is 100% the square wave is not exactly constant
    //In this way duty cycle ~= 99.9%
    PTPER = 920; // 2 KHz
    PTCONbits.PTEN = 1; // enable pwm
    
    //UART2 Initialization /////////////////////////////////////////////////////
                                                                              //             
    U2BRG = 11; //set the baud rate register: (7372800 / 4) / (16 * 9600)-1   //  VERIFICA CHE VADA BENE!!!!!!
    U2MODEbits.STSEL = 0; // 1 stop bit                                       //
    U2MODEbits.PDSEL = 0b00; // 8 bit no parity                               //
    U2MODEbits.UARTEN = 1; // UART enable                                     //
                                                                              //
    U2STAbits.UTXEN = 1; // unable transmission                               //
                                                                              //
    //////////Enable all the interrupts////////////////////////////////////////
    IEC1bits.U2RXIE = 1;   //enable interrupt for UART2 reception             //
    IEC0bits.T2IE = 1;     //enable interrupt for TIMER2                      //
    IEC0bits.T3IE = 1;     //enable interrupt fot TIMER3                      //
    ////////////////////////////////////////////////////////////////////////////
       
    // parser initialization////////////////////////////////////////////////////
    parser_state pstate;                                                      //
    pstate.state = STATE_DOLLAR;                                              //
    pstate.index_type = 0;                                                    //
    pstate.index_payload = 0;                                                 //
    ////////////////////////////////////////////////////////////////////////////
    
    
    
    while(1)
    {
        //In this section we read and convert from UART ////////////////////////
        //Disable the UART interrupt
        IEC1bits.U2RXIE = 0;
        //compute the available space in cb
        avl = avl_bytes_cb(&circularBuffer);
        //enable UART interrupt
        IEC1bits.U2RXIE = 1;
        //initialize a counter to compare the free space in cb
        count = 0;
        //while loop until we have available space in cb
        while(count<avl){
            //variable to store the current byte in cb
            char byte;
            //disable UART interrupt
            IEC1bits.U2RXIE = 0;
            //read from cb
            read_cb(&circularBuffer, &byte);
            //enable UART interrupt
            IEC1bits.U2RXIE = 1;
            ret = parse_byte(&pstate, byte);
            //if a new message is acquired
            if (ret == NEW_MESSAGE){
                // if the type of messagge is consistent
                if(strcmp(pstate.msg_type, "HLREF") == 0){
                    //then extract integer into the velocity
                    //ret = extract_integer(pstate.msg_payload, &velocity);
                    ret = extract_message(pstate.msg_payload, &velocity_l, &velocity_r);
                    // check if all goes well
                    if (ret == 0){
                        // if the velocity is bigger than 1000 RPM saturate it
                        if(velocity_l > 9000){
                            //saturate to 1000 RPM
                            velocity_l = 9000;
                        }
                        //if the velocity is 0 RPM limit it to 0
                        else if (velocity_l < -9000){
                            velocity_l = -9000;
                        }
                        
                        if(velocity_r > 9000){
                            //saturate to 1000 RPM
                            velocity_r = 9000;
                        }
                        //if the velocity is 0 RPM limit it to 0
                        else if (velocity_r < -9000){
                            velocity_r = -9000;
                        }
                    }
                }
            }
            count++;
        }
    }
    
    
    return 0;
}