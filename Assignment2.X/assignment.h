/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#define TIMER1 1
#define TIMER2 2  
#define TIMER3 3
#define TIMER_BODY 10
#define FIRST_ROW 0
#define SECOND_ROW 1

//declaration of the body of the algorithm
int body();
//function to set and wait ms
int tmr_wait_ms(int timer, int ms);
//function used to set the prescaler
int choose_prescaler(int ms,int* pr,int* tckps);
//set the timer for ms
void tmr_setup_period(int timer, int ms);
//wait until the timer expires
int tmr_wait_period(int timer);
//function used to write a character on LCD
void spi_put_char(char c);
//function used to write a string on LCD
void spi_put_string(char* str);
//function used to move the cursor
void spi_move_cursor(int row, int column);
//function used to clear the first row
void spi_clear_first_row();
//function to build the message to send to UART
void build_message(double current,double temp,char* message);
#endif	/*ASSIGNMENT_H */
