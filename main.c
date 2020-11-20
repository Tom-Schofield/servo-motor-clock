#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>

#define TRUE 1
#define FALSE 0
#define READ 0
#define WRITE 1


typedef enum {
	OFF=0,
	ON=1
} alarm_state_t;

typedef enum {
	STOP=0,
	START=1,
	TRANSMITTING=2,
	RECEIVING=3,
} i2c_state_t;

alarm_state_t alarm_state = OFF;
uint32_t alarm_on_time = 0;
uint32_t alarm_off_time = 0; 
uint32_t alarm_duration = 1000; // On / Off time duration (ms)

uint8_t pb_state = OFF;
uint32_t db_up = 0;
uint32_t db_down = 0;

volatile uint32_t ticks = 0; // Incremented in ISR
				/*	 REG1  REG2  REG3 */
char reg_data [3][3] = {{0xFF, 0xFF, 0xFF}, \
					{0xFF, 0xFF, 0x00}, \
					{0xFF, 0x00, 0x00}}; // Holds the PWM values for the sseg digits
volatile pwm_state = 0;
volatile uint8_t on_time = FALSE;
volatile uint8_t tx_complete = FALSE;

volatile uint8_t i2c_data = 0; // Data byte
volatile uint8_t i2c_command = 0; // Command / Control byte
volatile i2c_state_t i2c_state = STOP; // Current state of the hardware
volatile i2c_rw = WRITE; // Read or write oprtation
volatile uint8_t i2c_register; // Register to read
volatile uint8_t i2c_recv[30]; // Receive buffer
volatile uint8_t i2c_trans[30]; // Transmit buffer
volatile uint8_t i2c_trans_cnt = 0; // The total number of bytes to receive
volatile uint8_t i2c_trans_i = 0; // The index of the latest received byte
volatile uint8_t * i2c_trans_ptr = &i2c_trans[0];
volatile uint8_t * i2c_recv_ptr = &i2c_recv[0]; // Ptr to the the next element
volatile uint8_t i2c_recv_cnt = 0; // The total number of bytes to receive
volatile uint8_t i2c_recv_i = 0; // The index of the latest received byte

uint8_t rtc_reg[7] = {0}; // Holds most recent read of the timekeeping registers

uint8_t sseg_values[] = {0b1111110,0b0110000,0b1101101,0b1111001,0b0110011, \
	0b1011011,0b1011111,0b1110000,0b1111111,0b1110011};

uint8_t bin_2_bcd(uint8_t bin){
	return (((bin / 10) << 4) | (bin % 10));
}

uint8_t bcd_2_bin(uint8_t bcd){
	return ((bcd & 0xF0) >> 4)*10 + (bcd & 0x0F);
}

uint8_t count = 0;

/*
 * Updates the PWM values with the sseg values based on the RTC register data
 */
void update_time_display(void){

	count = (count + 1) % 10;
	uint8_t min_ones = rtc_reg[1] & 0x0F; // Minutes, ones
	uint8_t min_tens = (rtc_reg[1] & 0x70) >> 4; // Minutes, tens
	uint8_t hr_ones = rtc_reg[2] & 0x0F; // Hours, ones
	uint8_t hr_tens = rtc_reg[2] & 0x10 >> 4; // Hours, tens
	// reg_data[2][2] = ((sseg_values[min_ones] << 1) | hr_tens);
	// reg_data[2][1] = ((sseg_values[min_tens] << 1));
	// reg_data[2][0] = (sseg_values[hr_ones] << 1);
	
	// reg_data[1][2] = ((sseg_values[min_ones] << 1) | hr_tens);
	// reg_data[1][1] = ((sseg_values[min_tens] << 1));
	// reg_data[1][0] = (sseg_values[hr_ones] << 1);

	// reg_data[0][2] = 0xFF;
	// reg_data[0][1] = 0xFF;
	// reg_data[0][0] = 0xFF;

	// reg_data[1][2] = 0xFF;
	// reg_data[1][1] = 0xFF;
	// reg_data[1][0] = 0xFF;
	
	reg_data[2][2] = 0xFF;
	reg_data[2][1] = 0xFF;
	reg_data[2][0] = (sseg_values[min_ones] << 1);

	reg_data[1][2] = 0xFF;
	reg_data[1][1] = 0xFF;
	reg_data[1][0] = (sseg_values[min_tens] << 1) | hr_tens;

	reg_data[0][2] = 0xFF;
	reg_data[0][1] = 0xFF;
	reg_data[0][0] = (sseg_values[hr_ones] << 1) | hr_tens;

	// uint8_t datax = (sseg_values[min_tens] << 1);
	// reg_data[2][2] = 0xFF;
	// reg_data[2][1] = datax;
	// reg_data[2][0] = datax;
}

/*
 * Resets the timer count to zero
 */
void reset_global_timer(void){
	ticks = 0;
	TCNT0 = 0;
}

/*
 * Initialises a 1ms timer generated via TIM1 interrupt
 */
void init_timer(void) {
	//TCCR0A = TCCR0A & (~(1 << COM0A1)) | (~(1 << COM0A0));
	//TCCR0A = TCCR0A & (~(1 << COM0B1)) | (~(1 << COM0B0));
	TCCR1A = (TCCR0A | (1 << COM1A0));
	TCCR1B = TCCR1B |(1 << WGM12)| (0 << CS12) | (0 << CS11)|(1 << CS10);
	TCNT1 = 0;
	OCR1A = 18500; // 1kHz
	TIMSK1 = (1 << OCIE1A);
}

/*
 *  Initialises the GPIO used in the project
 */
void init_gpio(void) {
	DDRC = (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
	DDRD = ((1 << PD6) | (1 << PD0) | (1 << PD5)) & ~(1 << PD7);
	DDRB = (1 << PB5) | (1 << PB3) | (1 << PB2) | (1 << PB0);
	PORTC = (PORTC & (~(1 << PC0)));
	PORTC = (PORTC & (~(1 << PC1)));
	PORTC = (PORTC & (~(1 << PC2)));
	PORTC = (1 << PC3); 
	PORTD = (1 << PD0) & ~(1 << PD5);
}



/*
 * Initialises SPI for driving shift registers
 */
void init_spi(void) {
	/* Enable SPI, Master, set clock rate fck/4 */
	SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR0);
	SPSR = (1 << SPI2X); // Enable double speed
	PORTB |= (1 << PB2); // Set SS high
}

/*
 * Initialises I2C for reading RTC value
 */
void init_i2c(void) {
	TWBR |= 12; //10kHz I2C (1/16 prescaler)
	TWSR |= (0 << TWPS1); // 1/16 prescaler
	TWCR |= (1 << TWINT) | (1 << TWIE) | (1 << TWEN); // Enable interrupts
}

/* 
 * Sends data over the i2C lines
 */
void i2c_send_data(uint8_t command, uint8_t reg, uint8_t* data_byte, uint8_t rw, uint8_t cnt){
	// Clear the TWINT bit
	i2c_command = command;
	i2c_register = reg;
	i2c_rw = rw;

	if(rw == READ) {
		i2c_recv_cnt = cnt;
		i2c_recv_i = 0;
		i2c_recv_ptr = &i2c_recv[0];
	} else if (rw == WRITE){
		i2c_trans_cnt = cnt;
		i2c_trans_i = 0;
		memcpy(&i2c_trans, data_byte, cnt);
		i2c_trans_ptr = &i2c_trans[0];
	}

	i2c_state = START;

	TWCR |= (1 << TWINT) |(1<<TWEN)| (1<<TWSTA) | (1 << TWIE); // Send the START condition
}

/* 
 * Locking SPI transmit
 */
void SPI_transmit(uint8_t data) {
	SPDR = data;/* Wait for transmission complete */
	while(!(SPSR & (1 <<SPIF))){
	}
}

volatile uint8_t flag = 0;

/*
 * I2C Interrupt vector
 */
ISR(TWI_vect)
{
	uint8_t i2c_status = (TWSR & 0xF8);
	flag = flag ^ 1;
	switch(i2c_status){
		case 0x08:
			// Start condition transmitted
			TWDR = i2c_command; // Write the control word
			i2c_state = TRANSMITTING;
			TWCR |= (1<<TWINT) |(1<<TWEN);
			break;
		case 0x10:
			//PORTB |= 1;
			//TWDR = i2c_data;
			//TWCR |= (1<<TWINT)|(1<<TWEN);
			TWDR = i2c_command;
			TWCR |= (1<<TWINT);
			break;
		case 0x18:
			//PORTB |= 1;
			// Control byte transmitted
			i2c_state = TRANSMITTING;
			TWDR = i2c_register; // Load address byte
			TWCR = (1<<TWINT)  | (TWCR & ~(1 << TWSTA) & ~(1 << TWSTO));
			break;
		case 0x20:
			//PORTB |= 1;
			break;
		case 0x28:
			// Address byte transmitted, ACK received
			if(i2c_rw == READ){
				// Send repeated start
				i2c_command = i2c_command | (1);
				TWCR = ((1<<TWINT) |(1 << TWSTA)| (TWCR & ~(1 << TWSTO)));
				//i2c_state = STOP;
				//TWCR = ((1<<TWINT) |(1 << TWSTO)| (TWCR & ~(1 << TWSTA)));
			} else if (i2c_rw == WRITE) {
				// Transmit the data byte
				if((i2c_state == TRANSMITTING) && (i2c_trans_i < i2c_trans_cnt)) {
					TWDR = *i2c_trans_ptr++;
					TWCR = (1 << TWINT) | (TWCR & ~(1 << TWSTA) & ~(1 << TWSTO));
					i2c_trans_i++;
				} else {
					TWCR = ((1 << TWINT) |(1 << TWSTO)| (TWCR & ~(1 << TWSTA)));
					i2c_state = STOP;
				}
			}
			break;
		case 0x30:
			//PORTB |= 1;
			break;
		case 0x38:
			//PORTB |= 1;
			break;
		case 0x40:
			//PORTB |= 1;
			if(i2c_recv_cnt  == 1){
				TWCR = (1 << TWINT) | (TWCR & ~(1 << TWSTA) & ~(1 << TWSTO)); // Single receive
			} else{
				TWCR = (1 << TWEA) | (1 << TWINT) | (TWCR & ~(1 << TWSTA) & ~(1 << TWSTO)); // Bulk read
			}
			break;
		case 0x50:
			if(i2c_recv_i < i2c_recv_cnt-2){ // Receive next byte and send ACK (More bytes to receive)
				*i2c_recv_ptr++ = TWDR;
				i2c_recv_i++;
				TWCR = (1 << TWEA) | (1<<TWINT) | (TWCR & ~(1 << TWSTA) & ~(1 << TWSTO));
			} else if(i2c_recv_i == i2c_recv_cnt-2) { // Last byte send NACK
				*i2c_recv_ptr++ = TWDR;
				i2c_recv_i++;
				TWCR = (1<<TWINT) | (TWCR & ~(1 << TWSTA) & ~(1 << TWSTO) & ~(1 << TWEA));
			}
			break;
		case 0x58:
			// Data byte received NACK has been returned, send STOP condition
			*i2c_recv_ptr++ = TWDR;
			i2c_recv_i++;
			TWCR = ((1<<TWINT) |(1 << TWSTO)) | (TWCR & ~(1 << TWSTA));
			i2c_state = STOP;
			break;
		default:
			//PORTB |= 1;
			break;
	}
}

/*
 * Timer 0 compare A interrupt handler
 */
ISR(TIMER1_COMPA_vect)
{
	
	switch(pwm_state) {
		case 0:
			OCR1A = 998;
			on_time = TRUE;
			tx_complete = FALSE;
			pwm_state++;
			//PORTC &= ~((1 << PC2) | (1 << PC1)| (1 << PC0));
			break;
		case 1:
			OCR1A = 499;
			on_time = TRUE;
			tx_complete = FALSE;
			pwm_state++;
			//PORTC &= ~((1 << PC2) | (1 << PC1)| (1 << PC0));
			break;
		case 2:
			OCR1A = 499;
			on_time = TRUE;
			tx_complete = FALSE;
			//PORTC &= ~((1 << PC2) | (1 << PC1)| (1 << PC0));
			pwm_state++;
			break;
		case 3:
			OCR1A = 18000;
			on_time = FALSE;
			tx_complete = FALSE;
			//PORTC &= ~((1 << PC2) | (1 << PC1)| (1 << PC0));
			pwm_state++;
			break;
	}

}

/*
 * Initialises the MCP7940 
 */
void init_rtc(void){
	uint8_t data [] = {(1 << 7), 0x00, (1 << 6)};
	i2c_send_data(0b11011110, 0x00, &data[0], WRITE, 1); // Start RTC by writing 1 to ST bit
	for(int i=0; i < 50; i++){}
}

/*
 * 
 */
ISR(SPI_STC_vect)
{
	// flag = flag ^ 1;
    // PORTB = (PORTB & (~(1 << 0))) | (flag << 0); // Set bit
}
uint8_t init=0;
/*
 * Main loop
 */
int main(int argc, char** argv){
	
	init_gpio();
	init_spi();
	init_i2c();
	init_timer();

	sei(); // Enable interrupts
	
	while(1){
			if(on_time && !tx_complete && (pwm_state <= 3)){
				PORTC &= ~((1 << PC2) | (1 << PC1)| (1 << PC0));
				SPI_transmit(reg_data [0][pwm_state-1]);
				PORTC |= (1 << PC1);
				SPI_transmit(reg_data [1][pwm_state-1]);
				PORTC |= (1 << PC2);
				SPI_transmit(reg_data [2][pwm_state-1]);
				PORTC |= (1 << PC0);
				tx_complete = TRUE;
			} else if(!on_time && !tx_complete && (pwm_state > 3)){
				PORTC &= ~((1 << PC2) | (1 << PC1)| (1 << PC0));
				SPI_transmit(0x00);
				PORTC |= (1 << PC1);
				SPI_transmit(0x00);
				PORTC |= (1 << PC2);
				SPI_transmit(0x00);
				PORTC |= (1 << PC0);
				tx_complete = TRUE;
				if(pwm_state == 4){ // PWM off period
					pwm_state = 0;
					if(i2c_state != TRANSMITTING) {
						//i2c_send_data(0b11011110, 0x00, (1 << 7), WRITE);
						if(init == 0){
							init_rtc(); // Initialises the MCP7940 RTC
							init = 1;
						} else{
							i2c_send_data(0b11011110, 0x00, 0x00, READ, 7); // Read RTC registers
							while(!i2c_state == STOP);
							memcpy(&rtc_reg, &i2c_recv,7);
							update_time_display();
						}
					}
				}

				if((PORTD & (1 << 7)) && (pb_state == OFF)){
					db_up++;
				} else if(!(PORTD & (1 << 7)) && (pb_state == ON)) {
					db_down++;
				}

				if(deb_up > 500){
					pb_state = ON;
					db_up = 0;
				} else if(db_down > 500) {
					pb_state = OFF:
					db_down = 0;
				}
			}
			
	}
}