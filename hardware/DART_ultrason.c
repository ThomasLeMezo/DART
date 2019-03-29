/*
Hardware:
	16F876  SOIC28
	pin 1     MCR/
	pin 2     RA0 ---> Sortie commande entree TRIGGER du capteur ultrason 1
	pin 3     RA1 ---> Sortie commande entree TRIGGER du capteur ultrason 2
	pin 4     RA2 ---> Sortie commande entree TRIGGER du capteur ultrason 3
	pin 5     RA3 ---> Sortie commande entree TRIGGER du capteur ultrason 4
	pin 6     RA4 ---> LED1
	pin 7     RA5 ---> LED2
	pin 8     VSS alim 0V
	pin 9     OSC1 ---> quartz 20MHz
	pin 10    OSC2/RA6 ---> quartz 20MHz
	pin 11    RC0
	pin 12    RC1
	pin 13    RC2
	pin 14    RC3 ---> SCL I2C vers SCL maÃ®tre, resistance de tirage de 2.4k au VCC
	
	pin 15    RC4 ---> SDA I2C vers SDA maÃ®tre, resistance de tirage de 2.4k au VCC
	pin 16    RC5
	pin 17    RC6
	pin 18    RC7
	pin 19    VSS alim 0V
	pin 20    VDD Alim +5V
	pin 21    RB0 ---> LED
	pin 22    RB1 ---> LED
	pin 23    RB2 ---> LED
	pin 24    RB3 ---> LED
	pin 25    RB4 ---> Entree sortie ECHO capteur ultrason 4
	pin 26    RB5/PGM ---> Entree sortie ECHO capteur ultrason 3
	pin 27    RB6/PGC ---> Entree sortie ECHO capteur ultrason 2
	pin 28    RB7/PGD ---> Entree sortie ECHO capteur ultrason 1


	speed of sound : 340 m/s
	max distance : 4m
	max time : 4/340 = 12ms
	max timer count : 600000 (unsigned short long)
 */

#define CODE_VERSION 0x01

// I2C
const unsigned short ADDRESS_I2C = 0x38; // linux I2C Adresse
#define SIZE_RX_BUFFER 8
unsigned short rxbuffer_tab[SIZE_RX_BUFFER];
unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

void init_i2c();

// Hardware
sbit LED_SONAR_1 at RA0_bit;
sbit LED_SONAR_2 at RA1_bit;
sbit LED_SONAR_3 at RA2_bit;
sbit LED_SONAR_4 at RA3_bit;

sbit ECHO_SONAR_1 at RB7_bit;
sbit ECHO_SONAR_2 at RB6_bit;
sbit ECHO_SONAR_3 at RB5_bit;
sbit ECHO_SONAR_4 at RB4_bit;

sbit TRIGGER_SONAR_1 at RB0_bit;
sbit TRIGGER_SONAR_2 at RB1_bit;
sbit TRIGGER_SONAR_3 at RB2_bit;
sbit TRIGGER_SONAR_4 at RB3_bit;

// Watchdog
// unsigned short watchdog_restart = 60;
// unsigned short watchdog_restart_default = 60; // 3 s

// Timer : time base 0.2us
unsigned int time_overflow[5]; // Sonar 1-4 + Global
unsigned int time_trigger[5]; // Sonar 1-4 + Global
unsigned int time_echo[4];
unsigned char start_synchronous[4];

#define TIME_MAX 116000 // 400cm * 58/0.2
unsigned long time_max[4] = {TIME_MAX, TIME_MAX, TIME_MAX, TIME_MAX};

unsigned short distance[4];

#define TRIGGER_TIME 50 // 10us/0.2
#define CONVERSION_CM 290 // 58/0.2us

enum STATE {STATE_IDLE, STATE_TRIGGER, STATE_WAIT_ECHO, STATE_ECHO};
enum MODE {MODE_IDLE, MODE_ONE_SHOT, MODE_SYNCHRONOUS, MODE_ASYNCHRONOUS};
unsigned char sonar_mode[4] = {MODE_ASYNCHRONOUS, MODE_ASYNCHRONOUS, MODE_ASYNCHRONOUS, MODE_ASYNCHRONOUS};
unsigned char sonar_state[4] = {STATE_IDLE, STATE_IDLE, STATE_IDLE, STATE_IDLE};

static inline unsigned long time_since(unsigned int last_time, unsigned int time_overflow){
	unsigned long time_now = TMR1L | (TMR1H << 8);
	if(time_overflow==0){
		return time_now-last_time;
	}
	else{
		return (time_overflow-1)<<16 + time_now + (0xFFFF-last_time);
	}
}

void set_trigger(unsigned char id, unsigned char val){
	switch(id){
		case 0:
			TRIGGER_SONAR_1 = val;
			LED_SONAR_1 = val;
			break;
		case 1:
			TRIGGER_SONAR_2 = val;
			LED_SONAR_2 = val;
			break;
		case 2:
			TRIGGER_SONAR_3 = val;
			LED_SONAR_3 = val;
			break;
		case 3:
			TRIGGER_SONAR_4 = val;
			LED_SONAR_4 = val;
			break;
		default:
			break;
	}
}

unsigned short get_echo(unsigned char id){
	switch(id){
		case 0:
			return ECHO_SONAR_1;
			break;
		case 1:
			return ECHO_SONAR_2;
			break;
		case 2:
			return ECHO_SONAR_3;
			break;
		case 3:
			return ECHO_SONAR_4;
			break;
		default:
			break;
	}
	return 0;
}

void machine_state(unsigned char id){

	switch (sonar_state[id]){
		case STATE_IDLE:

			if(sonar_mode[id] == MODE_ASYNCHRONOUS || sonar_mode[id] == MODE_ONE_SHOT || (sonar_mode[id] == MODE_SYNCHRONOUS && start_synchronous[id] == 1)){
				sonar_state[id] = STATE_TRIGGER;
				set_trigger(id, 0);
				time_overflow[id] = 0;
				time_trigger[id] = TMR1L | (TMR1H << 8);
        if(time_trigger[id]<2) // Case overflow occurs between two previous line
          time_overflow[id] = 0;

				if(sonar_mode[id]==MODE_ONE_SHOT)
					sonar_mode[id]=MODE_IDLE;
				else if(sonar_mode[id]==MODE_SYNCHRONOUS)
					start_synchronous[id] = 0;
			}
			break;

		case STATE_TRIGGER:

			if(time_since(time_trigger[id], time_overflow[id])>=50){
				set_trigger(id, 1);
				sonar_state[id] = STATE_WAIT_ECHO;
			}
			break;

		case STATE_WAIT_ECHO:
			if(get_echo(id)==1){

				sonar_state[id] = STATE_ECHO;
				time_overflow[id] = 0;
				time_echo[id] = TMR1L | (TMR1H << 8);
        if(time_echo[id]<2) // Case overflow occurs between two previous line
          time_overflow[id] = 0;
			}
			else if(time_since(time_trigger[id], time_overflow[id])>=time_max[id]){
				sonar_mode[id] = STATE_IDLE;
				distance[id] = 0xFFFF;
			}
			break;

		case STATE_ECHO:
			if(get_echo(id)==0){
				distance[id] = time_since(time_echo[id], time_overflow[id])/CONVERSION_CM;
				sonar_state[id] = STATE_IDLE;
			}
			// else if(time_since(time_trigger[id], time_overflow[id])>=time_max[id]){
			// 	sonar_mode[id] = STATE_IDLE;
			// 	distance[id] = 0xFFFF;
			// }
			break;

		default:
			break;
	}
}


void i2c_read_data_from_buffer(){
		unsigned short i = 0;
		short nb_data = nb_rx_octet-1;

	for(i=0; i<nb_data; i++){
		switch(rxbuffer_tab[0]+i){
			case 0x01:
				break;
			case 0xB0:
				sonar_mode[0] = rxbuffer_tab[i+1];
				break;
			case 0xB1:
				sonar_mode[1] = rxbuffer_tab[i+1];
				break;
			case 0xB2:
				sonar_mode[2] = rxbuffer_tab[i+1];
				break;
			case 0xB3:
				sonar_mode[3] = rxbuffer_tab[i+1];
				break;
			default:
					break;
		}
	}
	//watchdog_restart = watchdog_restart_default;
}

/**
 * @brief Fonction qui crÃ©er la trame I2C
 * @param num
 */
void i2c_write_data_to_buffer(unsigned short nb_tx_octet){

		switch(rxbuffer_tab[0]+nb_tx_octet){
		case 0x00:
				SSPBUF = distance[0];
				break;
		case 0x01:
				SSPBUF = distance[0] >> 8;
				break;
		case 0x02:
				SSPBUF = distance[1];
				break;
		case 0x03:
				SSPBUF = distance[1] >> 8;
				break;
		case 0x04:
				SSPBUF = distance[2];
				break;
		case 0x05:
				SSPBUF = distance[2] >> 8;
				break;
		case 0x06:
				SSPBUF = distance[3];
				break;
		case 0x07:
				SSPBUF = distance[3] >> 8;
				break;

		case 0xA0:
				SSPBUF = time_max[0];
				break;
		case 0xA1:
				SSPBUF = time_max[0] >> 8;
				break;
		case 0xA2:
				SSPBUF = time_max[1];
				break;
		case 0xA3:
				SSPBUF = time_max[1] >> 8;
				break;
		case 0xA4:
				SSPBUF = time_max[2];
				break;
		case 0xA5:
				SSPBUF = time_max[2] >> 8;
				break;
		case 0xA6:
				SSPBUF = time_max[3];
				break;
		case 0xA7:
				SSPBUF = time_max[3] >> 8;
				break;
		
		case 0xB0:
				SSPBUF = sonar_mode[0];
				break;
		case 0xB1:
				SSPBUF = sonar_mode[1];
				break;
		case 0xB2:
				SSPBUF = sonar_mode[2];
				break;
		case 0xB3:
				SSPBUF = sonar_mode[3];
				break;

		case 0xB4:
				SSPBUF = sonar_state[0];
				break;
		case 0xB5:
				SSPBUF = sonar_state[1];
				break;
		case 0xB6:
				SSPBUF = sonar_state[2];
				break;
		case 0xB7:
				SSPBUF = sonar_state[3];
				break;

		case 0xC0:
				SSPBUF = CODE_VERSION;
				break;                    

		default:
				SSPBUF = 0x00;
				break;
		}
}

void init_timer1(){
	// Timer configuration : 20Mhz/(4*prescale) -> 0.2us

	T1CON.T1CKPS0 = 0b0; // Prescale 0b10 = 1:1
	T1CON.T1CKPS1 = 0b0; 


	// Mode : (counter : 0b0)
	T1CON.TMR1CS = 0b0; // Timer1 Clock Source Select bit (0=Fosc/4)
	T1CON.TMR1ON = 0b1; // Timer1 On bit

	// Counter value (always start reading the L before H for buffer issue)
	TMR1L = 0;
	TMR1H = 0;

	// Overflow interuption
	PIE1.TMR1IE = 1;
}

void init_io(){
	ANSEL = 0x00;    // PORTA digital
	ANSELH = 0x00;    // PORTB digital

	TRISA0_bit = 0; // RA0 en sortie (an open drain output)
	TRISA1_bit = 0; // RA1 en sortie
	TRISA2_bit = 0; // RA2 en sortie
	TRISA3_bit = 0; // RA3 en sortie
	TRISA4_bit = 0; // RA4 en sortie (an open drain output)
	TRISA5_bit = 0; // RA5 en sortie

	TRISC0_bit = 0; // RC0 en sortie
	TRISC1_bit = 0; // RC1 en sortie
	TRISC2_bit = 0; // RC2 en sortie

	TRISC3_bit = 1; // RC3 en entrée
	TRISC4_bit = 1; // RC4 en entrée

	TRISB0_bit = 0; // RB0 en sortie
	TRISB1_bit = 0; // RB1 en sortie
	TRISB2_bit = 0; // RB2 en sortie
	TRISB3_bit = 0; // RB3 en sortie
	TRISB4_bit = 1; // RB4 en entrée
	TRISB5_bit = 1; // RB5 en entrée
	TRISB6_bit = 1; // RB6 en entrée
	TRISB7_bit = 1; // RB7 en entrée

	WPUB = 0x00;
	
	INTCON.RBIF = 0;   // RB Port Change Interrupt Flag bit
	INTCON.RBIE = 0;  // Enables the RB port change interrupt
}

/**
 * @brief main
 */
void main(){
	unsigned char i=0;
	//OSCCON = 0b01110010;   // 0=4xPLL OFF, 111=IRCF<2:0>=16Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block

	//asm CLRWDT;// Watchdog
	//SWDTEN_bit = 1; //armement du watchdog

	init_io(); // Initialisation des I/O
	init_i2c(); // Initialisation de l'I2C en esclave
	init_timer1(); // Initialisation du TIMER0 toutes les 1 secondes
	LED_SONAR_1 = 0;
	LED_SONAR_2 = 0;
	LED_SONAR_3 = 0;
	LED_SONAR_4 = 0;
	TRIGGER_SONAR_1 = 1;
	TRIGGER_SONAR_2 = 1;
	TRIGGER_SONAR_3 = 1;
	TRIGGER_SONAR_4 = 1;
	UART1_Init(115200);
	Delay_ms(3000);

	while(1){
		//asm CLRWDT;
		for(i=0; i<4;i++)
			machine_state(i);

		if(time_since(time_trigger[5], time_overflow[5])>=time_max[5]){
			for(i=0; i<4;i++)
				start_synchronous[i]=1;
			time_overflow[5] = 0;
		}

		// I2C
		if(nb_rx_octet>1 && SSPSTAT.P == 1){
			i2c_read_data_from_buffer();
			nb_rx_octet = 0;
		}
	}
}

/**
 * @brief initialisation de l'I2C en mode esclave
 */
void init_i2c(){

	// **** IO I2C **** //
	RC3_bit = 1; // RC3 en entree
	RC4_bit = 1; // RC4 en entree

	// **** Interruptions **** //
	PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
	PIR1.SSPIF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave

	PIR2.BCLIE = 1;
	PIR2.BCLIF = 0;

	// **** ADDRESS **** //
	SSPADD = (ADDRESS_I2C << 1); // Address Register, Get address (7-1 bit). Lsb is read/write flag
	SSPMSK = 0xFF; // A zero (â€˜0â€™) bit in the SSPMSK register has the effect of making
								 // the corresponding bit in the SSPSR register

	// **** SSPSTAT **** //
	SSPSTAT.SMP = 1; // Slew Rate Control bit
	// 1 = Slew rate control disabled for standard Speed mode (100 kHz and 1 MHz)
	// 0 = Slew rate control enabled for High-Speed mode (400 kHz)
	SSPSTAT.CKE = 1; // // SMBusTM Select bit (1 = Enable SMBus specific inputs)

	// **** SSPCON2 **** //
	SSPCON2 = 0x00;
	SSPCON2.GCEN = 0; // General Call Enable bit (0 = disabled)
	SSPCON2.SEN = 1; // Start Condition Enable/Stretch Enable bit (1 = enabled)

	// **** SSPCON1 **** //
	SSPCON.WCOL = 0; // Write Collision Detect bit
	SSPCON.SSPOV = 0; // Receive Overflow Indicator bit
	SSPCON.CKP = 1; // SCK Release Control bit (1=Release clock)
	SSPCON.SSPM3 = 0b0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled (1-> with S/P, 0 -> without)
	SSPCON.SSPM2 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	SSPCON.SSPM1 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	SSPCON.SSPM0 = 0b0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled

	// (START the I2C Module)
	SSPCON.SSPEN = 1; // Synchronous Serial Port Enable bit
}

/**
 * @brief interrupt_low
 */
void interrupt_low(){
	if (PIR1.SSPIF){  // I2C Interrupt

			if(SSPCON.SSPOV || SSPCON.WCOL){
					SSPCON.SSPOV = 0;
					SSPCON.WCOL = 0;
					tmp_rx = SSPBUF;
			}

			//****** receiving data from master ****** //
			// 0 = Write (master -> slave - reception)
			if (SSPSTAT.R_W == 0){
				if(SSPSTAT.P == 0){
					if (SSPSTAT.D_A == 0){ // Address
						nb_rx_octet = 0;
						tmp_rx = SSPBUF;
					}
					else{ // Data
						if(nb_rx_octet < SIZE_RX_BUFFER){
							rxbuffer_tab[nb_rx_octet] = SSPBUF;
							nb_rx_octet++;
						}
						else{
							tmp_rx = SSPBUF;
						}
					}
				}
			}
			//******  transmitting data to master ****** //
			// 1 = Read (slave -> master - transmission)
			else{
					if(SSPSTAT.D_A == 0){
						nb_tx_octet = 0;
						tmp_rx = SSPBUF;
					}

					// In both D_A case (transmit data after receive add)
					i2c_write_data_to_buffer(nb_tx_octet);
					Delay_us(20);
					nb_tx_octet++;
			}

		SSPCON.CKP = 1;
		PIR1.SSPIF = 0; // reset SSP interrupt flag
	}
}

// ToDo : timer1 (TMR1IE=1)
void interrupt(){
		unsigned char i=0;

		if (PIE1.TMR1IF){ // Overflow
			for(i=0;i<5;i++)
				time_overflow[i] += 1;
			PIE1.TMR1IF = 0;
		}
}