/*
 * Stop_Watch.c
 *
 *  Created on: Sep 10, 2024
 *      Author: Mohamed Bahaa
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
unsigned char hours=0,seconds=0,minutes=0 ;     //Global Variables represents Hours, Minutes, Seconds
unsigned char count_mood=0;                     //Variable represents Condition for Toggle
unsigned char flag = 0;     // button flag PB0
unsigned char flag1 = 0;    // button flag PB0
unsigned char flag2 = 0;    // button flag PB1
unsigned char flag3 = 0;    // button flag PB3
unsigned char flag4 = 0;    // button flag PB4
unsigned char flag5 = 0;    // button flag PB5
unsigned char flag6 = 0;    // button flag PB6


// Timer1 initialization function for 1-second delay
void Timer1_init()
{
    TCNT1 = 0;               // Initialize Timer1 counter to 0
    OCR1A = 15624;           // Set output compare register to 15624 for 1 second
    TCCR1A = (1 << FOC1A);   // Set Timer1 to compare mode (force output compare)
    TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS12); // CTC mode, 1024 pre-scaler
    TIMSK |= (1 << OCIE1A);  // Enable Timer1 Compare A Interrupt
    sei();                   // Enable global interrupts
}


// Function to display time (hours, minutes, seconds) on six 7-segment displays
void display_time(unsigned char h, unsigned char m, unsigned char s)
{
	unsigned char digits[6];  // Array to hold the digits for the six displays

	// Split hours, minutes, and seconds into individual digits
	digits[0] = h/10;  // Tens digit of hours
	digits[1] = h%10;  // Ones digit of hours
	digits[2] = m/10;  // Tens digit of minutes
	digits[3] = m%10;  // Ones digit of minutes
	digits[4] = s/10 ;  // Tens digit of seconds
	digits[5] = s %10;  // Ones digit of seconds

	// Loop through the 6 digits (for 6 displays)
	for (unsigned char i = 0; i < 6; i++)
	{
		// Disable all displays first to avoid ghosting
		PORTA = 0x00;  // Set all pins of PORTA high to disable all displays

		// Enable the correct 7-segment display (only one at a time)
		PORTA = (1<<i);  // Enable display i (active low control on PORTA)
		// Load the lower nibble of PORTC with the digit value (preserving the upper 4 bits)
		PORTC = (PORTC & 0xF0) | (digits[i] & 0x0F);  // Send lower 4 bits to the decoder (first 4 pins)
		// Small delay to give the display time to show the digit (adjust if needed)
		_delay_ms(2);  // A 2ms delay gives the effect of all displays being on simultaneously
	}
}

// Function to increment the stop-watch (count-up mode)
void inc()
{
	PORTD&=~(1<<PD0);   //Clear Buzzer
	seconds++;
	// Roll over logic for seconds, minutes, and hours
	if (seconds >= 60) {
		seconds = 0;
		minutes++;
	}
	if (minutes >= 60) {
		minutes = 0;
		hours++;
	}
	if (hours >= 24) {
		hours = 0;
	}
}
// Function to decrement the stop-watch (count-down mode)
void dec()
{
	if (seconds == 0 && minutes == 0 && hours == 0)
	{
		// Trigger buzzer here when count-down reaches zero
		PORTD |= (1 << PD0); // Buzzer ON
		TCCR1B&=0xFA; // Automatically pause when count-down finishes
		return;
	}
	else
	{
		PORTD&=~(1<<PD0);     //Clear Buzzer

		 // Decrement seconds, minutes, and hours with roll-over logic
		if (seconds == 0)
		{
			seconds = 59;
			if (minutes == 0)
			{
				minutes = 59;
				hours--;
			}
			else
			{
				minutes--;
			}
		}
		else
		{
			seconds--;
		}
	}

}

//Function To Reset Stop Watch
void Reset()
{
	hours=0;
	minutes=0;
	seconds=0;
	PORTD&=~(1<<PD0); //Stop Buzzer
}

// External interrupt 0 initialization (Reset on button press)
void Int0_init()
{
    DDRD &= ~(1 << PD2);     // Set PD2 as input
    PORTD |= (1 << PD2);     // Enable pull-up resistor on PD2
    MCUCR |= (1 << ISC01);   // Trigger on falling edge
    MCUCR &= ~(1 << ISC00);
    GICR |= (1 << INT0);     // Enable external interrupt INT0
    sei();                   // Enable global interrupts

}

// External interrupt 1 initialization (Pause on button press)
void Int1_init()
{
	DDRD &= ~(1 << PD3);     // Set PD3 as input
	MCUCR |= (1 << ISC10) | (1 << ISC11);  // Trigger on rising edge
	GICR |= (1 << INT1);     // Enable external interrupt INT1
	sei();                   // Enable global interrupts
}

// External interrupt 2 initialization (Resume on button press)
void Int2_init()
{
	DDRB &= ~(1 << PB2);     // Set PB2 as input
	PORTB |= (1 << PB2);     // Enable pull-up resistor on PB2
	MCUCSR &= ~(1 << ISC2);  // Trigger on falling edge
	GICR |= (1 << INT2);     // Enable external interrupt INT2
	sei();                   // Enable global interrupts

}

// Interrupt service routine for INT2 (Resume Timer)
ISR(INT2_vect)
{

	TCCR1B|=(1<<CS10)|(1<<CS12);       //Resume the timer by setting the Clock Source (Pre-scale 1024)

}

// Interrupt service routine for INT1 (Pause Timer)
ISR(INT1_vect)
{

	TCCR1B&=0xFA;       //Pause the timer by Clearing the Clock Source
}

// Interrupt Service Routine for Timer1 CTC Mode called every 1 second
ISR(TIMER1_COMPA_vect)
{
	//Condition for 2 Modes
	if(count_mood==1)
	{
		dec();
	}
	else
	{
		inc();
	}

}

// Interrupt service routine for INT0 (Reset Timer)
ISR(INT0_vect)
{
	Reset();        //Call Function to Reset stop watch
}

//Function To adjust the timer settings

void adjust_time()
{
	if(!(PINB & (1<<PB0))) // check if the push button at PB0 is pressed or not
	{
		_delay_ms(20);
		if(!(PINB & (1<<PB0))) //second check due to switch de-bouncing
		{
			if(flag1 == 0)
			{

				if (hours == 0) {
					hours = 59;  // Wrap around if Hours go below 0
				} else {
					hours--;
				}
				//set the button flag value to 1 to not enter here again until the button is released.
				flag1 = 1;
			}
		}
	}
	else
	{
		// button is released reset the button flag to value 0 again.
		flag1 = 0;
	}


	if(!(PINB & (1<<PB1))) // check if the push button at PB0 is pressed or not
	{
		_delay_ms(20);
		if(!(PINB & (1<<PB1))) //second check due to switch de-bouncing
		{
			if(flag2 == 0)
			{
				hours++;
				if (hours >= 60) hours = 0;
				//set the button flag value to 1 to not enter here again until the button is released.
				flag2 = 1;
			}
		}
	}
	else
	{
		// button is released reset the button flag to value 0 again.
		flag2 = 0;
	}
	if(!(PINB & (1<<PB3))) // check if the push button at PB0 is pressed or not
	{
		_delay_ms(20);
		if(!(PINB & (1<<PB3))) //second check due to switch de-bouncing
		{
			if(flag3 == 0)
			{

				if (minutes == 0) {
					minutes = 59;  // Wrap around if minutes go below 0
				} else {
					minutes--;
				}
				//set the button flag value to 1 to not enter here again until the button is released.
				flag3 = 1;
			}
		}
	}
	else
	{
		// button is released reset the button flag to value 0 again.
		flag3 = 0;
	}
	if(!(PINB & (1<<PB4))) // check if the push button at PB0 is pressed or not
	{
		_delay_ms(20);
		if(!(PINB & (1<<PB4))) //second check due to switch de-bouncing
		{
			if(flag4 == 0)
			{
				minutes++;
				if (minutes >= 60) minutes = 0;
				//set the button flag value to 1 to not enter here again until the button is released.
				flag4 = 1;
			}
		}
	}
	else
	{
		// button is released reset the button flag to value 0 again.
		flag4 = 0;
	}
	if(!(PINB & (1<<PB5))) // check if the push button at PB0 is pressed or not
	{
		_delay_ms(20);
		if(!(PINB & (1<<PB5))) //second check due to switch de-bouncing
		{
			if(flag5 == 0)
			{

				if (seconds == 0) {
					seconds = 59;  // Wrap around if seconds go below 0
				} else {
					seconds--;
				}
				//set the button flag value to 1 to not enter here again until the button is released.
				flag5 = 1;
			}
		}
	}
	else
	{
		// button is released reset the button flag to value 0 again.
		flag5 = 0;
	}
	if(!(PINB & (1<<PB6))) // check if the push button at PB0 is pressed or not
	{
		_delay_ms(20);
		if(!(PINB & (1<<PB6))) //second check due to switch de-bouncing
		{
			if(flag6 == 0)
			{

				seconds++;
				if (seconds >= 60) seconds = 0;
				//set the button flag value to 1 to not enter here again until the button is released.
				flag6 = 1;
			}
		}
	}
	else
	{
		// button is released reset the button flag to value 0 again.
		flag6 = 0;
	}


}

//Main Program
int main(void)
{
	// Configure I/O ports and initialize interrupts, timer, and flags
	DDRC |= 0x0F;  // Set PORTC lower nibble as output (for 7-segment display)
	DDRA = 0xFF;   // Set PORTA as output (for display selection)
	DDRD |= (1 << PD0) | (1 << PD4) | (1 << PD5);  // Set PD0, PD4, PD5 as outputs (buzzer, LEDs)
	DDRB = 0xFB;   // Set PB2 as input for INT2, and PB7 as input for mode switch
	PORTB = 0xFF;  // Enable pull-ups for all PORTB pins

	Int0_init();  // Initialize INT0 (Reset)
	Int1_init();  // Initialize INT1 (Pause)
	Int2_init();  // Initialize INT2 (Resume)
	Timer1_init();  // Initialize Timer1

	// Main Loop
	for(;;)
	{

		display_time(hours,minutes,seconds);       //Function to continuously display time
		_delay_ms(2);                              //delay to allow for simultaneously display
		adjust_time();                             //Function to allow for time adjustment

		// Check for mode switch (PB7) to toggle between increment and count-down modes
		if(!(PINB&(1<<PB7)))
		{
			_delay_ms(30);
			if(!(PINB&(1<<PB7)))
			{
				if(flag==0)
				{
					count_mood^=1;   //Toggle Mode each time button is pressed
					flag=1;          //flag used to toggle again only when the button is released and pressed
				}
			}
		}
		else
		{
			flag=0;            		// button is released reset the button flag to value 0 again.
		}


		// Update LEDs based on current mode
		if (count_mood) {
			PORTD |= (1 << PD5); // Turn on Count-down LED
			PORTD &= ~(1 << PD4); // Turn off Increment LED
		} else {
			PORTD |= (1 << PD4); // Turn on Increment LED
			PORTD &= ~(1 << PD5); // Turn off Count-down LED
		}
	}
}
