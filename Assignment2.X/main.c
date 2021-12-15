/*
 * File:   newmainXC16.c
 * Author: Alessio
 *
 * Created on 5 dicembre 2021, 10.12
 */


// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = FRC                // Oscillator Source (Internal Fast RC)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#include "xc.h"
#include <stdio.h>
#include "stdlib.h"
#include "assignment.h"
#include "parser.h"
#include <string.h>

///////////////////////GLOBAL VARIABLES/////////////////////
char send_msg[25];
int T3_fired = 0;
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
//ISR for the TIMER2
void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(){
    IFS0bits.T2IF=0; //turn of the flag
    LATBbits.LATB0= !LATBbits.LATB0; //toggle LED D3
}

//ISR for the TIMER3
void __attribute__((__interrupt__,__auto_psv__)) _T3Interrupt(){
    IFS0bits.T3IF=0; //turn of the flag
    //set the flag to one
    T3_fired = 1;
    
}
//Analog to digital converter initialization
void adc_configuration() {
    ADCON3bits.ADCS = 8;
    //ADCON1bits.ASAM = 0; // manual sampling start
    ADCON1bits.ASAM = 1; // automatic sampling start
    //ADCON1bits.SSRC = 0; // manual conversion start
    ADCON1bits.SSRC = 7; // automatic conversion start
    ADCON3bits.SAMC = 16; // fixed conversion time (Only if SSRC = 7)
    //ADCON2bits.CHPS = 0; // CH0 only
    ADCON2bits.CHPS = 1; // CH0 & CH1
    ADCHSbits.CH0SA = 2; // AN2 connected to CH0
    //ADCHSbits.CH0SA = 3; // AN3 connected to CH0
    ADCHSbits.CH123SA = 1; // AN3 connected to CH1
    ADPCFG = 0xFFFF;
    ADPCFGbits.PCFG2 = 0; // AN2 as analog
    ADPCFGbits.PCFG3 = 0; // AN3 as analog
    ADCON2bits.SMPI = 1; // 2 sample/convert sequences
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
    
    /////////////////////DECLARING VARIABLES////////////////////////////////////
    float duty_cycle;
    int velocity;
    int avl;
    int count;
    int ret;
    ////////DECLARE THE LED D3 as OUTPUT////////////////////////////////////////
    TRISBbits.TRISB0 = 0;
    ////////DECLARE THE LED D4 as OUTPUT////////////////////////////////////////
    TRISBbits.TRISB1 = 0;
    //UART2 Initialization /////////////////////////////////////////////////////
                                                                              //             
    U2BRG = 11; //set the baud rate register: (7372800 / 4) / (16 * 9600)-1   //
    U2MODEbits.STSEL = 0; // 1 stop bit                                       //
    U2MODEbits.PDSEL = 0b00; // 8 bit no parity                               //
    U2MODEbits.UARTEN = 1; // UART enable                                     //
                                                                              //
    U2STAbits.UTXEN = 1; // unable transmission                               //
    
    //SPI initialization                                                      //
    SPI1CONbits.PPRE = 0b11;  // setup the primary prescaler to 1:1           //
    SPI1CONbits.SPRE = 0b110; // setup the secundary prescaler to 2:1         //
    //The two instructions above are needed becuase SPI works up to 1MHz      //
    SPI1CONbits.MSTEN = 1; //master                                           //
    SPI1CONbits.MODE16 = 0; // 8 bits                                         //
    SPI1STATbits.SPIEN = 1; // enable                                         //
    ////////////////////////////////////////////////////////////////////////////
    
    // parser initialization////////////////////////////////////////////////////
    parser_state pstate;                                                      //
    pstate.state = STATE_DOLLAR;                                              //
    pstate.index_type = 0;                                                    //
    pstate.index_payload = 0;                                                 //
    ////////////////////////////////////////////////////////////////////////////
    
     //////////Enable all the interrupts////////////////////////////////////////
    IEC1bits.U2RXIE = 1;   //enable interrupt for UART2 reception             //
    IEC0bits.T2IE = 1;     //enable interrupt for TIMER2                      //
    IEC0bits.T3IE = 1;     //enable interrupt fot TIMER3                      //
    ////////////////////////////////////////////////////////////////////////////
    
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
    ////////////////////////////////////////////////////////////////////////////
    
    ///////////adc config///////////////////////////////////////////////////////
    adc_configuration();
    //loop at 200 Hz - 5 ms
    tmr_setup_period(TIMER1,5);
    
    /////////setup TIMER2 manually to 1000ms to blink D3////////////////////////
    tmr_setup_period(TIMER2,1000);
    /////////setup TIMER3 manually to 1000ms to send values to UART ////////////
    tmr_setup_period(TIMER3,1000);
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
        int duty_int = (int)(duty_cycle * 100);
        char duty_char[3];
        sprintf(duty_char,"%i",duty_int);
        //apply the PWM
        PDC2 = duty_cycle * 2 * PTPER;
        
        
        
        ////////////////////////////////////////////////////////////////////////
        
        
        
        //In this section we simulate analog current sensor ////////////////////
        //wait until conversion done                                          //
        while(!ADCON1bits.DONE);                                              //
        //Variable to store the actual value of the potentiometer             //
        int adcValue = ADCBUF0;                                               //
        //scaling the potentiometer voltage                                   //
        double voltage_potentiometer = 5 * (adcValue / 1024.0);               //
        //compute the current                                                 //
        double current = 10 * (voltage_potentiometer - 3);                    //
        char sign_current[1];
        short int sign_curr;
        if (current > 15){                                                    //
            //turn on LED D4                                                  //
            LATBbits.LATB1 = 1;                                               //   
        }                                                                     //
        else{                                                                 //
            //turn off LED D4                                                 //
            LATBbits.LATB1 = 0;                                               //
        }                                                                     //
        ////////////////////////////////////////////////////////////////////////
       
        //////////// In this section read from AN3 for the temperature /////////
         int adcValueTemp =ADCBUF1 ; // take the value from  ADC1 buffer 
         double voltageTemp = adcValueTemp / 1024.0 * 5.0;
         double temperature = (voltageTemp - 0.75) * 100.0  + 25; //[Celsius]
         //test
        // current = -12.34;
         //temperature = -56.78;
         char sign_temperature[1];
         short int sign_temp;
                 
       /////////////////////////////////////////////////////////////////////////
       
     //In this section we send to UART the values for the current & temperature/
         
         //disable interrupt T3
         IEC0bits.T3IE = 0;   
         char message[25] = "$MCFBK,";
         
         //build_message(current,temperature,message);  
         ///////////////BUILD THE MESSAGE///////////////////////////////////////
         //Message Format : $MCFBK,CURRENT,TEMP*
         //Temperature Format: sign + xxxxxx [mC°]
         //Current Format: sign  + xxxxx [mA]
         
         //Check the sign for current and temperature
         if (current < 0){
             sign_curr = 1;
             current = current * (-1);
         }
         else{
             sign_curr = 0;
         }
         
         if (temperature < 0){
             temperature = temperature * (-1);
             sign_temp = 1;
         }
         else{
             sign_temp = 0;
         }
          
         //Convert current & temperature in char
         char current_char[5];
         char temperature_char[6];
         //Convert to integer to decrease the time needed to execute sprint
         //In this way we convert the temperature and current in [cC°] [cA] 
         int current_int = (int)(current * 100);
         int temperature_int = (int)(temperature * 100);
         sprintf(current_char,"%i",current_int);
         sprintf(temperature_char,"%i",temperature_int);
         if (sign_curr){
         strcat(message,"-");
         }
         else{
             strcat(message,"+");
         }
         strcat(message,current_char);
         strcat(message,",");
         if(sign_temp){
             strcat(message,"-");
         }
         else{
             strcat(message,"+");
         }
         strcat(message,temperature_char);
         strcat(message,"*");
         //Copy the message into the global variable
         strcpy(send_msg,message);
         //enable T3 interrupt
         IEC0bits.T3IE = 1;    
         ///////////////////////////////////////////////////////////////////////
         
         ////////NOW SEND THE MESSAGE TO UART IN T3 INTERRUPT///////////////////
         if(T3_fired){
            int length = strlen(send_msg);
            int i;
            for (i = 0;i<length;i++){
                while(U2STAbits.UTXBF);
                U2TXREG = send_msg[i];
            }
            //write on SPI
            spi_clear_first_row();
            spi_move_cursor(FIRST_ROW,0);
            spi_put_string(duty_char);
            //reset the interrupt flag to 0
            T3_fired = 0;
         }
         
         
         
        tmr_wait_period(TIMER1);
    }
    
    return 0;
}