/******************************************************************************
MSP430F2272 Project Creator 4.0

ME 461 - S. R. Platt
Fall 2010

Updated for CCSv4.2 Rick Rekoske 8/10/2011

Written by: Steve Keres
College of Engineering Control Systems Lab
University of Illinois at Urbana-Champaign
*******************************************************************************/

#include "msp430x22x2.h"
#include "UART.h"

char newprint = 0;
unsigned int timecnt = 0;
unsigned int fastcnt = 0;
unsigned int stateCnt = 0;
unsigned int sortAttempt = 0;

int i = 0;
int nBlob = 0;
int nSorted = 0;

// Create your global variables here:
int adcVal = 0;
long vAdc = 0;
long rVolt = 0;
long bVolt = 0;
long gVolt = 0;

unsigned char rxData[8] = {0,0,0,0,0,0,0,0};
unsigned char  txData[8] = {0,0,0,0,0,0,0,0};
int dataNum = 0;
int skittleColor = -1; // 0 = red, 1 = green, 2 = yellow, 3 = purple, 4 = orange
int threshold = 100;
int time = 500;


int rThreshold[6] = {2000,2400,700,1200,275,600}; //rLower, rUpper, gLower, gUpper, bLower, bUpper
int gThreshold[6] = {1000,1500,1420,2000,300,650}; //rLower, rUpper, gLower, gUpper, bLower, bUpper
int yThreshold[6] = {2875,3350,1850,2490,100,600}; //rLower, rUpper, gLower, gUpper, bLower, bUpper
int pThreshold[6] = {950,1400,800,1175,325,575}; //rLower, rUpper, gLower, gUpper, bLower, bUpper
int oThreshold[6] = {2800,3350,1000,1400,375,700}; //rLower, rUpper, gLower, gUpper, bLower, bUpper


/* Skittle threshold colors
* Red: R = [2580,3140]; G = [1650,2150]; B = [990,1450]
* Green: R = [1800,2120]; G = [2300,2630]; B = [1000,1250]
* Yellow: R = [3300,3400]; G = [2750,3170]; B = [1100,1380]
* Purple: R = [1620,2200]; G = [1520,1870]; B = [951,1280]
* Orange: R = [3300,3400]; G = [1700,2230]; B = [1110,1500]
*/


int locUpper = 2900;
int locLower = 4700;
int state = 0;
void main(void) {
	
	WDTCTL = WDTPW + WDTHOLD; // Stop WDT
	
	if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);
	                                             
	DCOCTL  = CALDCO_16MHZ; // Set uC to run at approximately 16 Mhz
	BCSCTL1 = CALBC1_16MHZ; 
		
	//P1IN 		    Port 1 register used to read Port 1 pins setup as Inputs
	P1SEL &= ~0xFF; // Set all Port 1 pins to Port I/O function
	P1REN &= ~0xFF; // Disable internal resistor for all Port 1 pins
	P1DIR |= 0x3D;   // Set Port 1 Pin 0 (P1.0), P1.2, P1.3, P1.4, P1.5 as an output.  Leaves Port1 pin 1 through 7 unchanged
	P1OUT &= ~0xFF; // Initially set all Port 1 pins set as Outputs to zero
    P1IFG &= ~0xFF;  // Clear Port 1 interrupt flags
    P1IES &= ~0xFF; // If port interrupts are enabled a High to Low transition on a Port pin will cause an interrupt
    P1IE &= ~0xFF; // Disable all port interrupts


	// Timer A Config
	TACCTL0 = CCIE;              // Enable Timer A interrupt
	TACCR0  = 16000;             // period = 1ms   
	TACTL   = TASSEL_2 + MC_1;   // source SMCLK, up mode
	

	// ADC10 Config (P2.1)
	ADC10CTL1 = ADC10SSEL_0 + INCH_1;                      // Conversion code singed format
	ADC10CTL0 = SREF_0 + ADC10SHT_2 + ADC10ON + ADC10IE; // ADC10ON, interrupt enabled
	ADC10AE0 |= BIT1;                         // P2.1 ADC option select

	//P4 TB1 and TB2
	P4DIR |= BIT1 + BIT2; //P4.1 and P4.2
	P4SEL |= BIT1 + BIT2; //P4.1 and P4.2

	// Timer B Config
	 TBCCTL0 = 0;
	 TBCCR0 = 20000;             // period = 1.25ms
	 TBCTL = TASSEL_2 + MC_1 + ID_3;   // source SMCLK, up mode

	//Servo 1 Config
	 TBCCTL1 = OUTMOD_7 + CLLD_1;         // reset/set, loads when TBR is 0
	 TBCCR1 = locUpper; //Duty Cycle send rxData here

	//Servo 2 Config
     TBCCTL2 = OUTMOD_7 + CLLD_1;         // reset/set, loads when TBR is 0
     TBCCR2 = locLower; //Duty Cycle send rxData here

     //USCB0 Config
      P3SEL |= 0x06; //P3.1 and P3.2 UCB Setup
      UCB0CTL1 = UCSWRST;
      UCB0CTL0 = UCMODE_3 + UCSYNC;
      UCB0I2COA = 0x25;
      UCB0CTL1 &= ~UCSWRST;
      IE2 |= UCB0RXIE;


	Init_UART(9600, 1);	// Initialize UART for 9600 baud serial communication

	_BIS_SR(GIE); 	    // Enable global interrupt

	while(1) {
	    if (nBlob == 0 && rxData[1] > 0 && rxData[1] < 7) {
	        nBlob = rxData[1];
	    }
		if(newmsg) {
			//my_scanf(rxbuff,&var1,&var2,&var3,&var4);
			newmsg = 0;
		}

		if (newprint)  { 
//			P1OUT ^= 0x2; // Blink LED
// 			UART_printf("%ld,%ld,%ld,%d,%d \n\r",rVolt,gVolt,bVolt,state,stateCnt); //  %d int, %ld long, %c char, %x hex form, %.3f float 3 decimal place, %s null terminated character array
//          UART_printf("%ld \n\r",rVolt); //  %d int, %ld long, %c char, %x hex form, %.3f float 3 decimal place, %s null terminated character array

			// UART_send(1,(float)timecnt);
			
			timecnt++;  // Just incrementing this integer for default print out.
			newprint = 0;
		}

	}
}


// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
	fastcnt++; // Keep track of time for main while loop. 
	if (fastcnt == 500) {
		fastcnt = 0;
		newprint = 1;  // flag main while loop that .5 seconds have gone by.  
	}
    if(nBlob > 0) stateCnt++;
//	stateCnt++;
	// Put your Timer_A code here:
    //ADC Voltage
    ADC10CTL0 |= ADC10SC + ENC;    //Trigger ADC10 Conversion





}



// ADC 10 ISR - Called when a sequence of conversions (A7-A0) have completed
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    //Converts light from LED into voltage values
    adcVal = ((int)ADC10MEM);
    vAdc = ((long)adcVal*3600L)/1023;
    //Place skittle sorter code here
    // TBCCR1 = [1200, 4900], TBCCR2 = [1200,4700]
    /* TB Locations of interest
     * TBCCR1 locUpper
     * 5100: LED, Hole 1
     * 4100: Hole 2
     * 3200: Hole 3, skittle hole (start here)
     * 2300: Hole 4
     * 1400: Hole 5
     *
     * TBCCR2 locLower
     * 4700: Hole 5, start here
     * 3800: Hole 4
     * 2900: Hole 3
     * 2050: Hole 2
     * 1100: Hole 1
     *
     */

    if(nSorted >= nBlob) state=0;

    //Different states
    // State flow: 0 -> 1 -> 2 -> 3 -> 4 -> (5 || 6 || 7 || 8 || 9) -> 0
    switch (state) {
        //Initial position
        case 0:
            locUpper = 2900;
            locLower = 4700;
            TBCCR1 = locUpper;
            TBCCR2 = locLower;
            if (stateCnt >= 2000) {
                state = 1;
                stateCnt = 0;
            } else
            state = 0;
            break;
        //Upper near LED
        case 1:
            locUpper = 4800;
            locLower = 4700;
            TBCCR1 = locUpper;
            TBCCR2 = locLower;
            if (stateCnt >= 1000) {
                state = 2;
                stateCnt = 0;
            } else {
                state = 1;
            }
            break;

        //Turn on red LED
        case 2:
            P1OUT = 0x8;
            rVolt = vAdc;

            //Make sure rVolt is reading a skittle value
            if (rVolt < threshold || stateCnt <= 1000) {
                state = 2;
            } else {
                state = 3;
                stateCnt = 0;
            }

            break;
        //Turn on blue LED
        case 3:
            P1OUT = 0x4;
            bVolt = vAdc;
            //Make sure bVolt is reading a skittle value
            if (bVolt < threshold || stateCnt <= 1000) {
                state = 3;
            } else {
                state = 4;
                stateCnt = 0;
            }
            break;
        //Turn on green LED
        case 4:
            P1OUT = 0x1;
            gVolt = vAdc;
            //Make sure gVolt is reading a skittle value
            if (gVolt < threshold || stateCnt <= 1000) {
                state = 4;
                }

            //Different states depending on threshold values:

            else{
                stateCnt = 0;
                //Red
                if (rVolt > rThreshold[0] && rVolt < rThreshold[1] && gVolt > rThreshold[2] && gVolt < rThreshold[3] && bVolt > rThreshold[4] && bVolt < rThreshold[5] ) {
                    state = 5;
                }
                //Green
                else if (rVolt > gThreshold[0] && rVolt < gThreshold[1] && gVolt > gThreshold[2] && gVolt < gThreshold[3] && bVolt > gThreshold[4] && bVolt < gThreshold[5] ) {
                    state = 6;
                }
                //Yellow
                else if (rVolt > yThreshold[0] && rVolt < yThreshold[1] && gVolt > yThreshold[2] && gVolt < yThreshold[3] && bVolt > yThreshold[4] && bVolt < yThreshold[5] ) {
                    state = 7;
                }
                //Purple
                else if (rVolt > pThreshold[0] && rVolt < pThreshold[1] && gVolt > pThreshold[2] && gVolt < pThreshold[3] && bVolt > pThreshold[4] && bVolt < pThreshold[5] ) {
                    state = 8;
                }
                //Orange
                else if (rVolt > oThreshold[0] && rVolt < oThreshold[1] && gVolt > oThreshold[2] && gVolt < oThreshold[3] && bVolt > oThreshold[4] && bVolt < oThreshold[5] ) {
                    state = 9;
                }
                else {
                    sortAttempt++;
                    if (sortAttempt == 3) {
                        state = 0;
                        sortAttempt = 0;
                    } else {
                        state = 2;
                    }
                }
            }

            break;
        //Hole 1: Red
        case 5:
            locUpper = 4800;
            locLower = 1200;
            TBCCR1 = locUpper;
            TBCCR2 = locLower;
            if (stateCnt >= 1000) {
                stateCnt = 0;
                state = 0;
                nSorted++;
                skittleColor = 0;
            } else {
                state = 5;
            }
            break;
        //Hole 2: Green
        case 6:
            locUpper = 3800;
            locLower = 2050;
            TBCCR1 = locUpper;
            TBCCR2 = locLower;
            if (stateCnt == 1000) {
                stateCnt = 0;
                state = 0;
                nSorted++;
                skittleColor = 1;
            } else {
                state = 6;
            }
            break;
        //Hole 3: Yellow
        case 7:

            if(stateCnt < 200){
                           locUpper = 3550;
                           locLower = 2415;
           }
            else if(stateCnt < 500){
                locUpper = 3350;
                locLower = 2415;
            }
            else if(stateCnt < 1000){
                locLower = 2900;
                locUpper = 4800;
            }

//            locUpper = 2900;
//            locLower = 2900;
            TBCCR1 = locUpper;
            TBCCR2 = locLower;
            if (stateCnt >= 1500) {
                stateCnt = 0;
                state = 0;
                nSorted++;
                skittleColor = 2;
            } else {
                state = 7;
            }
            break;
        //Hole 4: Purple
        case 8:
            locUpper = 2000;
            locLower = 3800;
            TBCCR1 = locUpper;
            TBCCR2 = locLower;
            if (stateCnt == 1000) {
                stateCnt = 0;
                state = 0;
                nSorted++;
                skittleColor = 3;
            } else {
                state = 8;
            }
            break;
        //Hole 5: Orange
        case 9:
            locUpper = 1100;
            locLower = 4700;
            TBCCR1 = locUpper;
            TBCCR2 = locLower;
            if (stateCnt == 1000) {
                stateCnt = 0;
                state = 0;
                nSorted++;
                skittleColor = 4;
            } else {
                state = 9;
            }
            break;
    }


}

// USCI Transmit ISR - Called when TXBUF is empty (ready to accept another character)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {

    if((IFG2&UCA0TXIFG) && (IE2&UCA0TXIE)) {        // USCI_A0 requested TX interrupt
        if(printf_flag) {
            if (currentindex == txcount) {
                senddone = 1;
                printf_flag = 0;
                IFG2 &= ~UCA0TXIFG;
            } else {
            UCA0TXBUF = printbuff[currentindex];
            currentindex++;
            }
        } else if(UART_flag) {
            if(!donesending) {
                UCA0TXBUF = txbuff[txindex];
                if(txbuff[txindex] == 255) {
                    donesending = 1;
                    txindex = 0;
                } else {
                    txindex++;
                }
            }
        }

        IFG2 &= ~UCA0TXIFG;
    }

    if((IFG2&UCB0RXIFG) && (IE2&UCB0RXIE)) {    // USCI_B0 RX interrupt occurs here for I2C
        // put your RX code here.
        rxData[dataNum] = UCB0RXBUF;
        if(dataNum == 0) {txData[dataNum] = nSorted;}
        else if (dataNum == 7) {txData[dataNum] = skittleColor;}
        else{ txData[dataNum] = rxData[dataNum];}
//        P1OUT ^= 0x1;     // Blink LED
        dataNum += 1;
        if(dataNum == 8){
            dataNum = 0;
            newprint = 1;
            IE2 &= ~UCB0RXIE;
            IE2 |= UCB0TXIE;
        }

    } else if ((IFG2&UCB0TXIFG) && (IE2&UCB0TXIE)) { // USCI_B0 TX interrupt
        // put your TX code here.
        UCB0TXBUF = txData[dataNum];
//        P1OUT ^= 0x2;
        dataNum += 1;
        if(dataNum == 8){
            dataNum = 0;
            IE2 &= ~UCB0TXIE;
            IE2 |= UCB0RXIE;
        }

    }
}


// USCI Receive ISR - Called when shift register has been transferred to RXBUF
// Indicates completion of TX/RX operation
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {

    if((IFG2&UCA0RXIFG) && (IE2&UCA0RXIE)) {  // USCI_A0 requested RX interrupt (UCA0RXBUF is full)

        if(!started) {  // Haven't started a message yet
            if(UCA0RXBUF == 253) {
                started = 1;
                newmsg = 0;
            }
        } else {    // In process of receiving a message
            if((UCA0RXBUF != 255) && (msgindex < (MAX_NUM_FLOATS*5))) {
                rxbuff[msgindex] = UCA0RXBUF;

                msgindex++;
            } else {    // Stop char received or too much data received
                if(UCA0RXBUF == 255) {  // Message completed
                    newmsg = 1;
                    rxbuff[msgindex] = 255; // "Null"-terminate the array
                }
                started = 0;
                msgindex = 0;
            }
        }
        IFG2 &= ~UCA0RXIFG;
    }




    if((UCB0I2CIE&UCNACKIE) && (UCB0STAT&UCNACKIFG)) { // I2C NACK interrupt

        UCB0STAT &= ~UCNACKIFG;
    }
    if((UCB0I2CIE&UCSTPIE) && (UCB0STAT&UCSTPIFG)) { // I2C Stop interrupt

        UCB0STAT &= ~UCSTPIFG;
    }
    if((UCB0I2CIE&UCSTTIE) && (UCB0STAT&UCSTTIFG)) { //  I2C Start interrupt

        UCB0STAT &= ~UCSTTIFG;
    }
    if((UCB0I2CIE&UCALIE) && (UCB0STAT&UCALIFG)) {  // I2C Arbitration Lost interrupt

        UCB0STAT &= ~UCALIFG;
    }
}



