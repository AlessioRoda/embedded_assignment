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

char vel; //Value of the velocity in charachter, must be converted in integer
float duty_cycle;

//ISR for the UART2 
void __attribute__((__interrupt__,__auto_psv__)) _U2RXInterrupt(){
    IFS1bits.U2RXIF = 0; //turn off the flag
    
    //Take the last velocity character value from the UART2 buffer
     vel = U2RXREG; 
    
}

// Convert velocity from char to integer
int convert_to_integer(char c)
{
    int integer = (int)c - 48;
    return integer;
}

float convert_velocity(int vel)
{
    if(vel>=1000)
    {
        return 5;
    }
    
    else if(vel<=0)
    {
        return 0;
    }
    
    else
    {
        return 0.005*vel;
    }
}

float evaluate_duty_cycle(float voltage)
{
    return voltage/5;
}


int main(void) {
    
    //first need to wait 1 second
    tmr_wait_ms(TIMER1, 1000);
    
    
    //UART2 Initialization                                                    //
                                                                              //             
    U2BRG = 11; //set the baud rate register: (7372800 / 4) / (16 * 9600)-1   //
    U2MODEbits.STSEL = 0; // 1 stop bit                                       //
    U2MODEbits.PDSEL = 0b00; // 8 bit no parity                               //
    U2MODEbits.UARTEN = 1; // UART enable                                     //
                                                                              //
    U2STAbits.UTXEN = 1; // unable transmission                               //
    ////////////////////////////////////////////////////////////////////////////
    
    //Initialize PWR
    //PTPER = 1842; // 1 kHz
    PTPER = 9216; // 50 Hz
    PTCONbits.PTEN = 1; // enable pwm
    
    
    tmr_setup_period(TIMER1,50);
    
    while(1)
    {
        // Convert to integer the velocity
        int int_vel=convert_to_integer(vel);
        //Set the correct voltage vlaue on the basis of the RPM
        float voltage = convert_velocity(int_vel);
        //Evaluate duty cycle
        duty_cycle=evaluate_duty_cycle(voltage);
        PDC2 = duty_cycle * 2 * PTPER;
        
        
        tmr_wait_period(TIMER1);
    }
    
    return 0;
}
