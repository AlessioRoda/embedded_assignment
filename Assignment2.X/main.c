/*
 * File:   newmainXC16.c
 * Author: Alessio
 *
 * Created on 5 dicembre 2021, 10.12
 */


#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include "assignment.h"
#include "parser.h"
#include <string.h>



#define CIRCULAR_BUFFER_SIZE 15

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

void adc_configuration() {
    ADCON3bits.ADCS = 16;
    //ADCON1bits.ASAM = 0; // manual sampling start
    ADCON1bits.ASAM = 1; // automatic sampling start
    //ADCON1bits.SSRC = 0; // manual conversion start
    ADCON1bits.SSRC = 7; // automatic conversion start
    ADCON3bits.SAMC = 31; // fixed conversion time (Only if SSRC = 7)
    ADCON2bits.CHPS = 0; // CH0 only
    //ADCON2bits.CHPS = 1; // CH0 & CH1
    ADCHSbits.CH0SA = 2; // AN2 connected to CH0
    //ADCHSbits.CH0SA = 3; // AN3 connected to CH0
    //ADCHSbits.CH123SA = 1; // AN3 connected to CH1
    ADPCFG = 0xFFFF;
    ADPCFGbits.PCFG2 = 0; // AN2 as analog
    //ADPCFGbits.PCFG3 = 0; // AN3 as analog
    //ADCON2bits.SMPI = 1; // 2 sample/convert sequences
    //ADCON1bits.SIMSAM = 1;
    //ADCON2bits.CSCNA = 1; // scan mode;
    
    /*ADCSSL = 0;
    ADCSSLbits.CSSL2 = 1; // scan AN2
    ADCSSLbits.CSSL3 = 1; // scan AN3 */
    ADCON1bits.ADON = 1;    
}


int main(void) {
    
    //first need to wait 1 second
    tmr_wait_ms(TIMER1, 1000);
    
    
    float duty_cycle;
    int velocity;
    
    //UART2 Initialization                                                    //
                                                                              //             
    U2BRG = 11; //set the baud rate register: (7372800 / 4) / (16 * 9600)-1   //
    U2MODEbits.STSEL = 0; // 1 stop bit                                       //
    U2MODEbits.PDSEL = 0b00; // 8 bit no parity                               //
    U2MODEbits.UARTEN = 1; // UART enable                                     //
                                                                              //
    U2STAbits.UTXEN = 1; // unable transmission                               //
    
    // parser initialization
    parser_state pstate;
    pstate.state = STATE_DOLLAR;
    pstate.index_type = 0;
    pstate.index_payload = 0;
    ////////////////////////////////////////////////////////////////////////////
    
     //////////Enable all the interrupts/////////////////////////////////////////
    IEC1bits.U2RXIE = 1;   //enable interrupt for UART2 reception             //
    
    ////////////////////////////////////////////////////////////////////////////
    //Initialize PWM
    //PTPER = 1842; // 1 kHz
    PTCONbits.PTMOD = 0; // free running
    //PTCONbits.PTCKPS = 0; // 1:1 prescaler
    PTCONbits.PTCKPS = 1; // 1:4 prescaler
    PWMCON1bits.PEN2H = 1;
    PWMCON1bits.PEN2L = 1;
    PTPER = 9216; // 50 Hz
    PTCONbits.PTEN = 1; // enable pwm
    
    //adc config
    adc_configuration();
    //loop at 200 Hz - 5 ms
    tmr_setup_period(TIMER1,5);
    
    while(1)
    {
        
        //In this section we read and convert from UART ////////////////////////
        //Disable the UART interrupt
        IEC1bits.U2RXIE = 0;
        //compute the available space in cb
        int avl = avl_bytes_cb(&circularBuffer);
        //enable UART interrupt
        IEC1bits.U2RXIE = 1;
        //initialize a counter to compare the free space in cb
        int count = 0;
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
            int ret = parse_byte(&pstate, byte);
            //if a new message is acquired
            if (ret == NEW_MESSAGE){
                // if the type of messagge is consistent
                if(strcmp(pstate.msg_type, "MCREF") == 0){
                    //then extract integer into the velocity
                    ret = extract_integer(pstate.msg_payload, &velocity);
                    // check if all goes well
                    if (ret == 0){
                        // if the velocity is bigger than 1000 RPM saturate it
                        if(velocity > 1000){
                            //saturate to 1000 RPM
                            velocity = 1000;
                        }
                        //if the velocity is 0 RPM limit it to 0
                        else if (velocity < 0){
                            velocity = 0;
                        }
                    }
                }
            }
            count++;
        }
          
        //Now that we finished to read the data from cb
        //Compute duty cycle
        //Set the correct voltage vlaue on the basis of the RPM
        duty_cycle = 0.001 * (velocity);
        //apply the PWM
        PDC2 = duty_cycle * 2 * PTPER;
        
        
        
        ////////////////////////////////////////////////////////////////////////
        
        
        
        //In this section we simulate analog current sensor ////////////////////
        //wait until conversion done
        while(!ADCON1bits.DONE);
        //Variable to store the actual value of the potentiometer
        int adcValue = ADCBUF0;
        //scaling the potentiometer voltage
        double voltage_potentiometer = 5 * (adcValue / 1023.0);
        //compute the current
        double current = 10 * (voltage_potentiometer - 3);
        
        
        
        ////////////////////////////////////////////////////////////////////////
         
        
          
        
       
        
//        
        
        
        tmr_wait_period(TIMER1);
    }
    
    return 0;
}