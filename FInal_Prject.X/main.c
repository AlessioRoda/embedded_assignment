 /*
 * File:   newmainXC16.c
 * Authors: Alessio Roda,Ermanno Girardo,Enzo Ubaldo Petrocco
 *
 * Created on 30 dicembre 2021, 10.32
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

#include <stdio.h>
#include "stdlib.h"
#include "assignment.h"
#include "parser.h"
#include <string.h>

//It is important to do some calculatoins:
//If the baud rate is 9600 --> 9600bits/s are transferred into the communication channel
//9600bits --> 1200 byte
//Our main loop is 20ms (50Hz frequency)
//In a single loop we can fill 1200/50=24 slot of the cb
//In order to stay in the boundary we declare the cb capacity ceil(2*24) = 50
//Considering that the firware may have to send to the pc up to 3 messages
//The capacity of the cb is approximated to 50

#define CIRCULAR_BUFFER_SIZE 50

///////////////////////////GLOBAL VARIABLES/////////////////////////////////////
int standby=0;                                                                //
char* temp_message_ptr;                                                       //
short int safe_mode = 0;                                                      //
short int display=0; //If 0 not pressed                                       //
int velocity_r=0;                                                             //
int velocity_l=0;                                                             //
char ack_enable[4] = "ENA";                                                   //
char ack_saturation[4] = "SAT";                                               //
////////////////////////////////////////////////////////////////////////////////


//Circular buffer struct
typedef struct {
    char buffer[CIRCULAR_BUFFER_SIZE];
    int readIndex;
    int writeIndex;
} circular_buffer_t;

//Cb to take bytes from PC
volatile circular_buffer_t circularBuffer;
//Cb to send bytes to PC
volatile circular_buffer_t cbSendToPc;

//function to write into cb
void write_cb(volatile circular_buffer_t* cb, char byte) {
    cb->buffer[cb->writeIndex] = byte;
    cb->writeIndex = (cb->writeIndex + 1) % CIRCULAR_BUFFER_SIZE;
    if (cb->readIndex == cb->writeIndex) {
        // full buffer
        cb->readIndex++; // discard the oldest byte
    }
}

//function to read from cb
int read_cb(volatile circular_buffer_t* cb, char* byte) {
    if (cb->readIndex != cb->writeIndex) {
        *byte = cb->buffer[cb->readIndex];
        cb->readIndex = (cb->readIndex + 1) % CIRCULAR_BUFFER_SIZE;
        return 0;
    }
    else
    {
        return -1;
    }
}

//function to count available bytes in cb
int avl_bytes_cb(volatile circular_buffer_t* cb) {
    if (cb->readIndex <= cb->writeIndex) {
        return cb->writeIndex - cb->readIndex;
    } else {
        return CIRCULAR_BUFFER_SIZE - cb->readIndex + cb->writeIndex;
    }
}


//ISR for the UART2 reception
void __attribute__((__interrupt__,__auto_psv__)) _U2RXInterrupt(){
    IFS1bits.U2RXIF = 0; //turn off the flag
    
    //Write on the circular buffer the current character from UART
    while (U2STAbits.URXDA == 1) {
        write_cb(&circularBuffer, U2RXREG);
    }
}

//ISR for the UART2 transmission
void __attribute__((__interrupt__,__auto_psv__)) _U2TXInterrupt(){
    IFS1bits.U2TXIF = 0; //turn off the flag
    char byte;
    int ret=read_cb(&cbSendToPc,&byte);
    if (ret==0)
    {
        U2TXREG = byte;
    }
}

//ISR when the button S5 is pressed
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){
    IFS0bits.INT0IF = 0; //turn off the flag
    safe_mode = 1;
    //Stop immediately the motors
    //Setting velocity to zero -->
    // --> Is equal to have duty cycle 50%
    velocity_l = 0;
    velocity_r = 0;
    //Update PWM
    PDC1 = 50 * 2 * PTPER;
    PDC2 = 50 * 2 * PTPER;
}

//ISR when the button S6 is pressed
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(){
    IFS1bits.INT1IF = 0; //turn off the flag
   
    //Toggle the state of the display mode
    if(display==0)
    {
        display=1;
    }
    else
    {
        display=0;
    }
}

    
//Analog to digital converter initialization
void adc_configuration() {
    ADCON3bits.ADCS = 8; // Tad=8*Tcy
    //ADCON1bits.ASAM = 0; // manual sampling start
    ADCON1bits.ASAM = 1; // automatic sampling start
    //ADCON1bits.SSRC = 0; // manual conversion start
    ADCON1bits.SSRC = 7; // automatic conversion start
    ADCON3bits.SAMC = 16; // fixed conversion time (Only if SSRC = 7)--> 16* Tad
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
    ADCON1bits.ADON = 1;    //enable to start
}
    
//function to extract the RPM values from a string
//Format of the string: RPM_left,RPM_right
//n1 and n2 could be saturation values for the motors
int extract_message(const char* str, int* n1, int* n2)
{
    int len= strlen(str);
    char string1[len];
    char string2[len];
    int i;
    int j=0;
    int flag =1;
    for(i=0;i<len+1; i++)
    {
        if(str[i]==',')
        { 
            char string_to_send[i+1];
            int k=0;
            for(;k<i; k++)
            {
                string_to_send[k]=string1[k];
            }
            string_to_send[i]='\0';

            flag= extract_integer(string_to_send, n1);

            if(flag==-1)
            {
                return -1;
            }
            i++;
            j=i;
        }

        if(str[i]=='\0')
        {
           // char* string_to_send= string2;
            char string_to_send[i-j+1];
            int k=0;
            for(;k<i-j; k++)
            {
                string_to_send[k]=string2[k];
            }
            string_to_send[i-j]='\0';
            flag= extract_integer(string_to_send, n2);

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
    

//Function to send enable ack
//if message type is 1 we want to send enable_ack
//if message type is 0 we want to send saturation_ack
void send_ack(int msg_type,int value){
    //Build message
    char message[15] = "$MCACK,";
    char value_char[1];
    sprintf(value_char,"%i",value);
    
    if(msg_type==0)
    {
        strcat(message,ack_saturation);
    }
    else
    {
        strcat(message,ack_enable);
    }
    
    strcat(message,",");
    strcat(message,value_char);
    strcat(message,"*");
    //Write into the cb the string
    //In order to do not wait the availability of the transmit register
    //Polling once the availability
    //Use _U2TXInterrupt in order to write to PC without waiting
    int k = 0;
    int str_len = strlen(message);
    for (;k<str_len;k++){
        //disable interrupt for transmission
        IEC1bits.U2TXIE = 0;
        //write on the buffer the characters
        write_cb(&cbSendToPc,message[k]);
        //polling UART to send the message
        if(U2STAbits.UTXBF==0)
        {
            char byte;
            int ret=read_cb(&cbSendToPc, &byte);
            if(ret==0)
            {
                U2TXREG = byte;
            }
        }
        //enable interrupt for transmission
        IEC1bits.U2TXIE = 1;
    }
}



//MCFBK ack to send via UART to the PC
void send_MCFBK_ack(int n1,int n2){
    
    int state = 0;
    if(safe_mode==1){
        state=2;
    }
    else if(standby==1)
    {
        state=1;
    }
    else
    {
        state=0;
    }
    //Build the message:
    char message[15] = "$MCFBK,";
    char value_char_n1[5] ;
    char value_char_n2[5];
    char state_msg[1];
    sprintf(value_char_n1,"%d",n1);
    sprintf(value_char_n2,"%d",n2);
    sprintf(state_msg,"%i",state);

    strcat(message,value_char_n1);
    strcat(message,",");
    strcat(message,value_char_n2);
    strcat(message,",");
    strcat(message,state_msg);
    strcat(message,"*");
    //Write the message into the cb
    int k = 0;
    int str_len = strlen(message);
    for (;k<str_len;k++){
        //disable interrupt for transmission
        IEC1bits.U2TXIE = 0;
        
        //write on the buffer the characters
        write_cb(&cbSendToPc,message[k]);
        
        //polling transmit UART register
        if(U2STAbits.UTXBF==0)
        {
            char byte;
            int ret=read_cb(&cbSendToPc, &byte);
            if(ret==0)
            {
                U2TXREG = byte;
            }
        }
        
        //enable interrupt for transmission
        IEC1bits.U2TXIE = 1;
    }
}

//Function to display the values on the LCD as default mode 
void display_0(int temperature, int rpm_l, int rpm_r)
{
    //build the message for the first row:
    char message[15]= "ST: ";
    if(safe_mode==1){
        strcat(message,"H");
    }
    else if(standby==1)
    {
        strcat(message,"T");
    }
    else
    {
        strcat(message,"C");
    }
    strcat(message,"; T: ");
    char temp_char[6];
    sprintf(temp_char, "%d", temperature);
    strcat(message,temp_char);
    
    //write first row on SPI
    spi_clear_first_row();
    spi_move_cursor(FIRST_ROW,0);
    spi_put_string(message);
    
    //build the message for the second row
    char message2[15]= "R: ";
    char rpm_r_char[5];
    char rpm_l_char[5];
    sprintf(rpm_r_char, "%d", rpm_r);
    sprintf(rpm_l_char, "%d", rpm_l);
    strcat(message2,rpm_l_char);
    strcat(message2,"; ");
    strcat(message2,rpm_r_char);
    
    //write in the second row
    spi_clear_second_row();
    spi_move_cursor(SECOND_ROW,0);
    spi_put_string(message2);
}


//Function to display the values on the LCD when the button S6 is pressed
void display_1(int min, int max)
{
    //build the message for the first row
    char message[15]= "SA: ";
    char max_char[5];
    char min_char[5];
    
    sprintf(max_char, "%d", max);
    sprintf(min_char, "%d", min);
    strcat(message,min_char);
    strcat(message,"; ");
    strcat(message,max_char);
    
    //write first row on SPI
    spi_clear_first_row();
    spi_move_cursor(FIRST_ROW,0);
    spi_put_string(message);
    
    //build the message for the second row
    char message2[15]= "R: ";
    char duty_char_1[5];
    char duty_char_2[5];
    sprintf(duty_char_1, "%d", PDC1);
    sprintf(duty_char_2, "%d", PDC2);
    strcat(message2,duty_char_1);
    strcat(message2,"; ");
    strcat(message2,duty_char_2);
    //write in the second row
    spi_clear_second_row();
    spi_move_cursor(SECOND_ROW,0);
    spi_put_string(message2);
}

//main function of the project
//Since no scheduling structure for tasks is implemented:
//-The period of the main function is set to 20ms
//This period is valid for two reasons:
//1)The execution of main project is faster than 20ms
//2)All the other tasks have periods that are multiples of the main one
//-The variable main_period is the counter for the other tasks
int main(void) {
    
    //first need to wait 1 second
    tmr_wait_ms(TIMER1, 1000);
    ////////////////////////LOCAL VARIABLES/////////////////////////////////////
    float duty_cycle_l;                                                       //
    float duty_cycle_r;                                                       //
    int max=9000;                                                             //    
    int min=-9000;                                                            //
    int user_max=9000;                                                        //
    int user_min=-9000;                                                       //
    int avl;                                                                  //
    int count;                                                                //
    int count_send_feedback=0;                                                //
    int blink_D3=0;                                                           //
    int ret;                                                                  //
    int main_period=20;                                                       //
    int temp_count=0;                                                         //    
    int standby_count=0;                                                      //
    float average;                                                            //    
    short int sign_temp;                                                      //
    int average_count_loop = 0;                                               //    
    int send_average_count = 0;                                               //        
    float temperature_array[10];                                              //    
                                                                              //
    ////////DECLARE THE LED D4 as OUTPUT////////////////////////////////////////
    TRISBbits.TRISB1 = 0;                                                     //                
    //Initialize LED D4 off                                                   //
    LATBbits.LATB1 = 0;                                                       //
    ////////DECLARE THE LED D3 as OUTPUT////////////////////////////////////////   
    TRISBbits.TRISB0 = 0;                                                     //
    //Initialize LED D3 off                                                   //
    LATBbits.LATB0 = 0;                                                       //
    ////////////////DECLARE BUTTON S5 as INPUT//////////////////////////////////
    TRISEbits.TRISE8 = 1; //button S5 as input                                //
    TRISDbits.TRISD0 = 1; //button S6 as input                                //
    ////////////////////////INITIALIZE PWM//////////////////////////////////////   
    PTPER = 1842; // 1 kHz                                                    //
    PTCONbits.PTMOD = 0; // free running                                      //
    PTCONbits.PTCKPS = 0; // 1:1 prescaler                                    //
    //PTCONbits.PTCKPS = 1; // 1:4 prescaler                                  //
    //High and Low of the first PWM signal                                    //
    PWMCON1bits.PEN2H = 1;                                                    //
    PWMCON1bits.PEN2L = 1;                                                    //
    //High and Low of the second PWM signal                                   //
    PWMCON1bits.PEN1H = 1;                                                    //
    PWMCON1bits.PEN1L = 1;                                                    //
    //It is important to use both the high and low part of the register       //
    //because a dead time is needed to prevent short circuit of H bridge      //
    //NOTE THAT:                                                              //    
    //PTPER should be 920.6                                                   //    
    //1843,2 / 2 - 1 = 920.6                                                  //    
    //This introduce a computational error                                    //
    //Then when duty cycle is 100% the square wave is not exactly constant    //
    //In this way duty cycle ~= 99.9%                                         //    
    //PTPER = 920; // 2 KHz                                                   //
    PTCONbits.PTEN = 1; // enable pwm                                         //
    DTCON1bits.DTA=9; //This gives about 5 micro seconds                      //
    //DTA = DeadTime/(Prescaler * Tcy)                                        //
    //Dead time prescaler                                                     //
    DTCON1bits.DTAPS=0; //No approximation                                    //
                                                                              //
    ////////////////////SPI initialization//////////////////////////////////////
    SPI1CONbits.PPRE = 0b11;  // setup the primary prescaler to 1:1           //
    SPI1CONbits.SPRE = 0b110; // setup the secundary prescaler to 2:1         //
    //The two instructions above are needed becuase SPI works up to 1MHz      //
    SPI1CONbits.MSTEN = 1; //master                                           //
    SPI1CONbits.MODE16 = 0; // 8 bits                                         //
    SPI1STATbits.SPIEN = 1; // enable                                         //
                                                                              //
    //////////////////UART2 Initialization /////////////////////////////////////
                                                                              //             
    //U2BRG = 11; //set the baud rate register: (7372800 / 4) / (16 * 9600)-1 //
    U2BRG = 23; //set the baud rate register: (7372800 / 4) / (16 * 4800)-1   //
    U2MODEbits.STSEL = 0; // 1 stop bit                                       //
    U2MODEbits.PDSEL = 0b00; // 8 bit no parity                               //
    U2MODEbits.UARTEN = 1; // UART enable                                     //
                                                                              //
    U2STAbits.UTXEN = 1; // unable transmission                               //
    // interrupt fires when at least one character can be written             //
    U2STAbits.UTXISEL = 0;                                                    //                                                                 
    //////////////PARSER INITIALIZATION/////////////////////////////////////////
    parser_state pstate;                                                      //
    pstate.state = STATE_DOLLAR;                                              //
    pstate.index_type = 0;                                                    //
    pstate.index_payload = 0;                                                 //
    ////////////////////////////////////////////////////////////////////////////
    
    //////////////Enable all the interrupts/////////////////////////////////////
    IEC1bits.U2TXIE = 1;    //enable interrupt for UART2 transmission         //
    IEC1bits.U2RXIE = 1;   //enable interrupt for UART2 reception             //
    IEC0bits.T2IE = 1;     //enable interrupt for TIMER2                      //
    IEC0bits.T3IE = 1;     //enable interrupt fot TIMER3                      //
    IEC0bits.INT0IE = 1;   //enable interrupt for button S5                   //
    IEC1bits.INT1IE =1;   //enable interrupt for button S6                    //
    ////////////////////////////////////////////////////////////////////////////
    
    //setup the timer for the main project
    tmr_setup_period(TIMER1, main_period);
    
    while(1)
    {   
        //In this section we read and convert from UART2////////////////////////
        //Disable the UART interrupt
        IEC1bits.U2RXIE = 0;
        if(standby_count>=5000)
        {
            //Set velocity motors to zero
            velocity_r=0; 
            velocity_l=0;
            standby=1;
       
            //Blink LED D4
            //In this way D4 blinks at 50Hz
            //We cannot visibly see the toggling but for our scope is ok
            LATBbits.LATB1 = !LATBbits.LATB1;
        }
       
        
        
        //compute the available space in cb
        avl = avl_bytes_cb(&circularBuffer);
        //enable UART interrupt
        IEC1bits.U2RXIE = 1;
        //initialize a counter to compare the free space in cb
        count = 0;
        //while loop until we have available space in cb
        while(count<avl && standby_count<5000){
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
                // if the type of messagge is consistent to HLREF
                if(strcmp(pstate.msg_type, "HLREF") == 0 && safe_mode == 0){
                    //then extract integers from the msg_payload
                    ret = extract_message(pstate.msg_payload, &velocity_l, &velocity_r);
                    // check if all goes well
                    if (ret == 0){
                        //Reset the counter for time out mode
                        standby_count=0;
                        standby=0; //Turns off the stanby mode
                        LATBbits.LATB1 = 0; // Turn off the led
                        // if the velocity is bigger than max saturate it
                        if(velocity_l > max){
                            //saturate to 1000 RPM
                            velocity_l = max;
                        }
                        //if the velocity is smaller than min saturate it
                        else if (velocity_l < min){
                            velocity_l = min;
                        }
                        
                        //do them for the right motor
                        if(velocity_r > max){
                            //saturate
                            velocity_r = max;
                        }
                    
                        else if (velocity_r < min){
                            velocity_r = min;
                        }
                        
                    }
                }
                // if the type of messagge is consistent to HLENA
                if(strcmp(pstate.msg_type, "HLENA") == 0){
                    //disable S5 interrupt
                    IEC0bits.INT0IE = 0;
                    safe_mode = 0;
                    send_ack(1,1); //Send enable ack, 1 as first argument
                    IEC0bits.INT0IE = 1;
                }
                 // if the type of messagge is consistent to HLSAT
                if(strcmp(pstate.msg_type, "HLSAT") == 0)
                {
                    //acquire new saturation values
                    ret = extract_message(pstate.msg_payload, &user_min, &user_max);
                    //check the consistency of the new saturation values
                    if (ret==0)
                    {
                        //Check that the values are in the correct range
                        if(user_min<=0 && user_min>=-9000 && user_max<=9000 && user_max>=0)
                        {
                            //Check that the min, max values are correctly set (i.e., min < max).
                            if(user_min<user_max)
                            {
                                send_ack(0, 1); //Send positive saturation ack
                                max=user_max;
                                min=user_min;
                                
                                //Saturate velocities if necessary
                                if(velocity_l > max){
                                    //saturate to 1000 RPM
                                    velocity_l = max;
                                }
                                //if the velocity is 0 RPM limit it to 0
                                else if (velocity_l < min){
                                    velocity_l = min;
                                }

                                if(velocity_r > max){
                                    //saturate to 1000 RPM
                                    velocity_r = max;
                                }
                                //if the velocity is 0 RPM limit it to 0
                                else if (velocity_r < min){
                                    velocity_r = min;
                                }
                                
                            }
                            else{
                                send_ack(0, 0);//Send negative saturation ack                                
                            }
                        }
                        else{
                            send_ack(0, 0);//Send negative saturation ack
                        }
                    }
                }
            }
            //increment the counter
            count++;
        }
        
        //In this section we have to generate the two PWM signals///////////////
        //Compute duty cycle
        //Set the correct voltage value on the basis of the RPM
        duty_cycle_l = 1.0/24000 * velocity_l + 0.5;
        duty_cycle_r = 1.0/24000 * velocity_r + 0.5;
        int duty_int_l = (int)(duty_cycle_l * 100);
        int duty_int_r = (int)(duty_cycle_r * 100);
        char duty_char_l[3];
        char duty_char_r[3];
        sprintf(duty_char_l,"%i",duty_int_l);
        sprintf(duty_char_r,"%i",duty_int_r);
        //apply the PWM
        //since main loop is 50Hz
        //10 Hz refreshing requisite is satisfied
        PDC1 = duty_cycle_l * 2 * PTPER;
        PDC2 = duty_cycle_r * 2 * PTPER;
        
        //////////// In this section read from AN3 for the temperature /////////
        int adcValueTemp = ADCBUF1 ; // take the value from  ADC1 buffer 
        double voltageTemp = adcValueTemp / 1024.0 * 5.0;
        double temperature = (voltageTemp - 0.75) * 100.0  + 25; //[Celsius]
         
        //we have to acquire one temperature value each 100 ms
        if(average_count_loop == 100){
            temperature_array[temp_count]=temperature;
            temp_count+=1;
            average=0;
            //if we have collected 10 values
            //compute the temperature average
            if(temp_count==10)
            {
                int k=0;
                float sum=0;
                for(; k<10; k++)
                {
                    sum=temperature_array[k]+sum;
                }
                average=sum/10;
                temp_count=0;

                sign_temp=0;

                if (average < 0){
                    average = average * (-1);
                    sign_temp = 1;
                }

                else{
                    sign_temp = 0;
                }
            }
            average_count_loop = 0;
        }     
        //each 1s we have to send to PC the temperature average
        if(send_average_count == 1000){
            //build the message
            char temp_message[15] = "MCTEM,";
            //Average of the temperature in centi celsius in order to speed up 
            //the sprintf function
            int average_int = (int)(average * 100);
             
            if(sign_temp){
                strcat(temp_message,"-");
            }
            else{
                strcat(temp_message,"+");
            }
             
            char current_char[5];
            sprintf(current_char,"%i",average_int);
            strcat(temp_message,current_char);
            strcat(temp_message,"*");
            //write the message into cb
            int q=0;
            for(;q<strlen(temp_message);q++){
                char temp_byte=NULL;
                //disable interrupt
                IEC1bits.U2TXIE = 0;
                write_cb(&cbSendToPc,temp_byte);
                //polling UART2 transmit register
                if(U2STAbits.UTXBF==0)
                {
                    char byte;
                    int ret=read_cb(&cbSendToPc, &byte);
                    if(ret==0)
                    {
                        U2TXREG = byte;
                    }
                }          
                //enable interrupt
                IEC1bits.U2TXIE = 1;
            }
            send_average_count = 0;
        }
        
        

        //Each 5Hz = 200 ms the feedback is sent to the PC
        if (count_send_feedback==200)
        {
           send_MCFBK_ack(velocity_l,velocity_r);
           count_send_feedback=0;
        }
         
        //Each 1Hz = 1000 ms blink D3 led
        if (blink_D3==500)
        {
           LATBbits.LATB0 = !LATBbits.LATB0;
           blink_D3=0;
        }
         
        //Write on LCD on the basis of button S6
        IEC1bits.INT1IE =0;
        if(display==0)
        {
            display_0(temperature, velocity_l, velocity_r);
        }
        else
        {
            display_1(min, max);
        }
        IEC1bits.INT1IE =1;
        
        /////////////UPDATES COUNTER VARIABLES//////////////////////////////////
        count_send_feedback += main_period;
        blink_D3 += main_period;
        average_count_loop += main_period;
        send_average_count += main_period; 
        standby_count += main_period;
        /////////////WAIT UNTIL main period is elapsed//////////////////////////
        tmr_wait_period(TIMER1);
    }
    return 0;
}