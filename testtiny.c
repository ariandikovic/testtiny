/*-------------------------------
This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>
--------------------------------


 test program for RFM22 and ATTINY2313 P1 prototype
 !! write crystal calibration manually with the command  W 09 xx, where xx is calibration value in HEX !!
 power on default: 433.92 MHz, 20 kbps, deviation 40 kHz, AFC ON, AGC ON, TX pwr 8dBm
 data whitening OFF, FIFO DATA modulation source
 
data interface: RS232 (use adapter!), 19200 bit/s, 8 bits data, no parity, 1 stop bit
--------------------------------*/
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>


#define USART_BAUDRATE 19200 //Baudrate definition
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) //magic number for baudrate setup :)

unsigned char scratch [12];	//scratchpad for text input
volatile unsigned char flag;	// we need this one for interrupt service routine

static inline void usartInit (void)//initialising USART; static inline! doing it only once!
{	
UBRRL=BAUD_PRESCALE ;//kload 8 lower bits of baud prescaler
UBRRH=(BAUD_PRESCALE >> 8);	//shift for 8 places right
UCSRC=0b00000110;//asynchroneous comm,no parity, 1 stop bit, 8 bit character size, clock polarity 0
UCSRB=0b00011000 ;//no interrupts 000, RX enable 1, TX enable 1, no ninth bit 000
}

static inline void spiInit (void) //initialise hardware SPI interface
{
PORTB = PORTB | (1<< PB4);//set SS high, do not touch other bits
return;
}

static inline void attinyinit (void) //microcontroller peripherial setup
{
DDRB = 0b11010000;			//PB7-SCK, PB6-D0,PB4-SS output, PB5-DI input, PB3-PB0 inputs
PORTB = PORTB | 0b00001111; //activate pull-up on PB3-PB0
//use logical OR to activate pull up where needed, leave the rest alone

DDRD = 0b00011000; //PD7-non existent,PD6 and PD5 inputs,PD4 and PD3 outputs,PD2-PD0 inputs
PORTD = PORTD | 0b01100011; //pull up on PD6,PD5,PD1,PD0

PORTD = PORTD | (1<< PD3);				//set PD3 high, LED RED off
PORTD = PORTD | (1<< PD4);				//set PD4 high, LED GREEN off

MCUCR = 0x00; 	//low level INT0 generates interrupt
GIMSK = 0b01000000; //allow interrupts from INT0
return;
}

/*
void spiReadBurst128 (char addr,unsigned char *buf) //reads SPI bus in BURST mode (see RFM22 datasheet)
{						//input:start adress and pointer to data field
unsigned char i;				//loop counter
PORTB = PORTB & (~(1<< PB4));	//SS low, everything else stays
SPDR= (addr & 0b01111111);		//start SPI adress for burst read + MSB=0 for read
while (!(SPSR & (1<<SPIF)));	//wait for SPIF, byte sent to SPI
for(i=0 ; i<128; i++){ 			//loop copies 128 registers to scratch
SPDR=0;							//something to SPDR, RFM22 will ignore this one
while (!(SPSR & (1<<SPIF)));	//wait for SPDR loaded wit data from RFM22
buf[i]=SPDR; 					//write data to scratch
}
PORTB = PORTB | (1<< PB4);		//set SS high
return;
}

void usartSendString128 (unsigned char *s) //sends string to usart
{
unsigned char i;
for(i=0 ; i<128; i++){ 			//count loop until 128 chars sent
while (!(UCSRA & (1<<UDRE))) ; 		//check if data gone, wait for UDRE to be set
UDR=s[i]; 				//if data gone, put next char
}
return; 
}
*/

void usartHEXmyoutput (unsigned char i) //will convert to nice HEX output human looking at usart terminal
{
unsigned char j;	//we need one more variable
j=i & 0b00001111;	//4 lower bits in j, second digit of hex value
i=i>>4;				//upper 4 bits shifted right gives us the first digit of hex value
if (i<10){			//less than then, write ascii
	i=i+48;			//that means value + 48
	}else {			//greater or equal to ten, we need to output ABCDEF
	i=i+55;			//what gives us value plus 55 (uppercase)
	}
if (j<10){			//same as for i...
	j=j+48;
	}else {
	j=j+55;
	}
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent,wait UDRE to be set
UDR=i;							//i goes out
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent,wait UDRE to be set
UDR=j;							//j goes out
return;
}

void usartPutchar (char c)	//writes one char to USART
{
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent, wait UDRE to be set
UDR=c;							//char goes out
return;
}

void usartPutCRLF (void)	//writes CRLF to USART
{
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent,wait UDRE to be set
UDR=13;							//Carriage return 
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent,wait UDRE to be set
UDR=10;							//New line feed
return;
}



void usartSendstring (char *s) //sends string to USART,feed it with a string pointer
{
unsigned char i;
for(i=0 ; s[i] != '\0'; i++){ //loop sending chars until NULL encountered
while (!(UCSRA & (1<<UDRE))) ; //check if char is sent
UDR=s[i]; //if yes get next char for output
}
return; 
}

unsigned char getDIPvalue (void)
{
unsigned char a,value = 0;
a=PIND;	//read PORTD
a=a<<1,		//shift left a
a=a & 0b11000000;//leave 2 highest bits (PD6,PD7)
value=value | a; //paste it to value

a=PIND;	//read PORTD
a=a<<4,		//shift 4 places left
a=a & 0b00110000;//leave 2 bits
value=value | a; //paste to value

a=PINB;//read PORTB
a=a & 0b00001111;//leave lowest 4 bita (PB0-PB3)
value=value | a; //paste to  value

value = value ^ 0xFF; //invert bits -> when switch ON pin is read as 0!!!

return value;
}

void spiWrite (unsigned char addr,unsigned char data) //writes data to address addr to SPI
{
PORTB = PORTB & (~(1<< PB4));	//SS low, leave others

USIDR = (addr | (1<<7));		//SPI address for write + MSB=1 for write
USISR = (1<<USIOIF); 
   do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);

USIDR = data;	//load data to USIDR
USISR = (1<<USIOIF); 	//write to USIOIF to clear it and reset the counter
do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);

PORTB = PORTB | (1<< PB4);		//set SS high
return;
}

unsigned char spiRead (unsigned char addr) //read from address
{
PORTB = PORTB & (~(1<< PB4));	//SS low, leave others

USIDR = (addr & 0b01111111);		//start SPI adress for burst read + MSB=0 for read
USISR = (1<<USIOIF); 	//write to USIOIF to clear it and reset the counter
do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);

USIDR = 0; //just some data, will ignore it
USISR = (1<<USIOIF); 	//write to USIOIF to clear it and reset the counter
do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);

PORTB = PORTB | (1<< PB4);		//set SS high
return USIDR;					//return data from address addr
}


void usartSendDump (void) 	// dump all RFM22 registers to USART
{
unsigned char i,a;
for(i=0 ; i<128; i++){ 			//loop until 128 registers sent
a=spiRead (i);					//read register
usartHEXmyoutput (i);			//print register address in hex 
usartPutchar (':');				//print":"
usartPutchar (' ');				//then space...			
usartHEXmyoutput (a);			//write value in hex format
usartPutCRLF ();				//new line...
}
return; 
}

static inline void RFM22init (void) //initialize RFM22 module
{
_delay_ms(150);			//wait 150 ms, init...
spiWrite(0x07, 0x80);		//software reset RFM22
_delay_ms(150);			//wait another 150 ms to be sure

spiWrite(0x05, 0x00);	//disable all interrupts on RFM
spiWrite(0x06, 0x00);	//disable all interrupts on RFM

spiRead (0x03);//read interrupt status 1 registar, to clear interrupt flag if set
spiRead (0x04);//read interrupt status 2 registar, to clear interrupt flag if set

//Si4432 V2 silicon specific
spiWrite(0x5A, 0x7F); //write 0x7F to the VCO Current Trimming register
spiWrite(0x58, 0x80); //write 0xD7 to the ChargepumpCurrentTrimmingOverride register
spiWrite(0x59, 0x40); //write 0x40 to the Divider Current Trimming register
//best receiver performances setup
spiWrite(0x6A, 0x0B); //write 0x0B to the AGC Override 2 register
spiWrite(0x68, 0x04); //write 0x04 to the Deltasigma ADC Tuning 2 register
spiWrite(0x1F, 0x03); //write 0x03 to the Clock Recovery Gearshift Override register

//XTAL loading capacity -->>manually set, differs from module to module!!
spiWrite(0x09, 0x7D);//7D or 7F for start value
spiWrite(0x0A, 0x00);//microcontroller clock 30 Mhz, no clock tail, no Low freq clock
spiWrite(0x0B, 0xD2);//GPIO 0 - strong drive (HH), no pullup, TX state
spiWrite(0x0C, 0xD5);//GPIO 1 - strong drive (HH), no pullup, RX state
spiWrite(0x0D, 0x00);//GPIO 2 - strong drive (HH), no pullup, CLK output

spiWrite(0x0F, 0x70);//ADC input ->GND
spiWrite(0x10, 0x00);//ADC offset ->0
spiWrite(0x12, 0x00);//temp. sensor calibration off
spiWrite(0x13, 0x00);//temp. sensor offset ->0

spiWrite(0x1C, 0x04);//IF filter bandwith -> RFM datasheet page 44
spiWrite(0x1D, 0x40);//AFC enable
spiWrite(0x1E, 0x05);//AFC timing -> ?

spiWrite(0x20, 0xC8);//clock recovery oversampling
spiWrite(0x21, 0x00);//clock recovery offset 2
spiWrite(0x22, 0xA3);//clock recovery offset 1
spiWrite(0x23, 0xD7);//clock recovery offset 0
spiWrite(0x24, 0x00);//clock recovery timing loop 1
spiWrite(0x25, 0xA6);//clock recovery timing loop 0

spiWrite(0x30, 0x8E);//CRC-16 on,TX packet handling on,CRC over entire packet,RX packet handling on
spiWrite(0x32, 0x00);//no header check
spiWrite(0x33, 0x02);//NO header, sync word 3 and 2 ON -> 2D, D4, variable packet lenght on
spiWrite(0x34, 0x10);//preamble 16 nibbles ->64 bits
spiWrite(0x35, 0x30);//preamble detection 4 nibbles -> 24 bits
spiWrite(0x36, 0x2D);//sync word 3
spiWrite(0x37, 0xD4);//sync word 2
spiWrite(0x69, 0x20);//AGC enable
//-----------------------------------------------------
spiWrite(0x3E, 0x02);//packet lenght 2 byte (payload)
spiWrite(0x6D, 0x00);//output power 8 dBm

spiWrite(0x70, 0x20);//whitening OFF, DATA RATE under 30 kbps!!!- > bit 5 SET!!!!!
spiWrite(0x71, 0x23);//GFSK, FIFO mode

spiWrite(0x72, 0x40);//freq. deviation 40 khz

spiWrite(0x6E, 0xA3);//TX data rate  1-> 20 kbps
spiWrite(0x6F, 0xD7);//TX data rate  0-> 20 kbps

spiWrite(0x73, 0x00);//no frequency offset
spiWrite(0x74, 0x00);//no frequency offset
spiWrite(0x79, 0x00);//no frequency hopping
spiWrite(0x7A, 0x00);//no frequency hopping

spiWrite (0x75,0x53);//freq. band select 430-440 MHz

spiWrite (0x76,0x62);//carrier 433.92 MHz
spiWrite (0x77,0x00);//carrier 433.92 MHz

spiWrite(0x05, 0x07);//interrupt enable packet sent,crc error, valid packet receive

spiWrite (0x08,0x01);//clear TX FIFO
spiWrite (0x08,0x00);//clear TX FIFO

//resetirat RX FIFO
spiWrite (0x08,0x02);//write to addr 08 one, ffclrrx=1
spiWrite (0x08,0x00);//write to addr 08 zero, ffclrrx=0

return;
}

void usartFillbuffer (void)	//load buffer until enter pressed
{
unsigned char i=0,c=0;
usartPutCRLF ();		//CR LF for new line
usartPutchar ('>');		//command prompt...

while(c!='\r')			//repeat until enter hit
{
while (!(UCSRA & (1 << RXC)));	//wait for char from USART
c=UDR;							//char goes to c
scratch [i++]=c;				//move c to scratch and increment index
usartPutchar (c);				//echo char back to USART
}
scratch [--i]='\0';				//when enter hit, owerwrite \n with \0 so we have a proper string
usartPutchar ('\n');			//new line to be ready for print
return;
}


void setParam (void){
unsigned char i;
/getDIPvalue returns value on DIP switch, we need only SW 1,2,3,4
//if everything OFF, power on default remains

i= getDIPvalue();//read DIP SW
i=i&0b00001111;	// leave lower 4 bits


if (i==0) return;

if (i==1) {	//2.4 kBaud, 4.8 khz 
spiWrite (0x70,0x20);//data rate under 30 kBaud, set bit 5 in 0x70
spiWrite (0x1C,0x33);
spiWrite (0x20,0xD0);
spiWrite (0x21,0x00);
spiWrite (0x22,0x9D);
spiWrite (0x23,0x49);
spiWrite (0x24,0x00);
spiWrite (0x25,0xA0);
}

if (i==2) {	//2.4 kBaud, 36 khz 
spiWrite (0x70,0x20);//data rate under 30 kBaud, set bit 5 in 0x70
spiWrite (0x1C,0x07);
spiWrite (0x20,0x83);
spiWrite (0x21,0xC0);
spiWrite (0x22,0x13);
spiWrite (0x23,0xA9);
spiWrite (0x24,0x00);
spiWrite (0x25,0x05);
}

if (i==3) {	//4.8 kBaud, 4.8 khz 
spiWrite (0x70,0x20);//data rate under 30 kBaud, set bit 5 in 0x70
spiWrite (0x1C,0x34);
spiWrite (0x20,0x68);
spiWrite (0x21,0x01);
spiWrite (0x22,0x3A);
spiWrite (0x23,0x93);
spiWrite (0x24,0x02);
spiWrite (0x25,0x78);
}

if (i==4) {	//4.8 kBaud, 45 khz 
spiWrite (0x70,0x20);//data rate under 30 kBaud, set bit 5 in 0x70
spiWrite (0x1C,0x04);
spiWrite (0x20,0x41);
spiWrite (0x21,0x60);
spiWrite (0x22,0x27);
spiWrite (0x23,0x52);
spiWrite (0x24,0x00);
spiWrite (0x25,0x0A);
}


if (i==5) {	//10 kBaud, 5 khz 
spiWrite (0x70,0x20);//data rate under 30 kBaud, set bit 5 in 0x70
spiWrite (0x1C,0x21);
spiWrite (0x20,0x64);
spiWrite (0x21,0x01);
spiWrite (0x22,0x47);
spiWrite (0x23,0xAE);
spiWrite (0x24,0x05);
spiWrite (0x25,0x21);
}

if (i==6) {	//10 kBaud, 40 khz 
spiWrite (0x70,0x20);//data rate under 30 kBaud, set bit 5 in 0x70
spiWrite (0x1C,0x03);
spiWrite (0x20,0x90);
spiWrite (0x21,0x20);
spiWrite (0x22,0x51);
spiWrite (0x23,0xEC);
spiWrite (0x24,0x00);
spiWrite (0x25,0x2B);
}

if (i==7) {	//20 kBaud, 10 khz 
spiWrite (0x70,0x20);//data rate under 30 kBaud, set bit 5 in 0x70
spiWrite (0x1C,0x11);
spiWrite (0x20,0x64);
spiWrite (0x21,0x01);
spiWrite (0x22,0x47);
spiWrite (0x23,0xAE);
spiWrite (0x24,0x05);
spiWrite (0x25,0x21);
}


if (i==8) {	//40 kBaud, 20 khz 
spiWrite (0x70,0x00);//bit 5 cleared in 0x70 for transfer rate > 30 kBaud
spiWrite (0x1C,0x01);
spiWrite (0x20,0x64);
spiWrite (0x21,0x01);
spiWrite (0x22,0x47);
spiWrite (0x23,0xAE);
spiWrite (0x24,0x05);
spiWrite (0x25,0x21);
}

if (i==9) {	//40 kBaud, 40 khz 
spiWrite (0x70,0x00);//bit 5 cleared in 0x70 for transfer rate > 30 kBaud
spiWrite (0x1C,0x05);
spiWrite (0x20,0x64);
spiWrite (0x21,0x01);
spiWrite (0x22,0x47);
spiWrite (0x23,0xAE);
spiWrite (0x24,0x02);
spiWrite (0x25,0x91);
}

if (i==10) {	//100 kBaud, 50 khz 
spiWrite (0x70,0x00);//bit 5 cleared in 0x70 for transfer rate > 30 kBaud
spiWrite (0x1C,0x0F);
spiWrite (0x20,0x78);
spiWrite (0x21,0x01);
spiWrite (0x22,0x11);
spiWrite (0x23,0x11);
spiWrite (0x24,0x04);
spiWrite (0x25,0x46);
}

if (i==11) {	//100 kBaud, 300 khz 
spiWrite (0x70,0x00);//bit 5 cleared in 0x70 for transfer rate > 30 kBaud
spiWrite (0x1C,0x0E);
spiWrite (0x20,0x78);
spiWrite (0x21,0x01);
spiWrite (0x22,0x11);
spiWrite (0x23,0x11);
spiWrite (0x24,0x00);
spiWrite (0x25,0xB8);
}

if (i==12) {	//20 kBaud, 40 khz 
spiWrite (0x70,0x20);//data rate under 30 kBaud, set bit 5 in 0x70
spiWrite (0x1C,0x04);
spiWrite (0x20,0xC8);
spiWrite (0x21,0x00);
spiWrite (0x22,0xA3);
spiWrite (0x23,0xD7);
spiWrite (0x24,0x00);
spiWrite (0x25,0xA6);
}

return;
}

void parseBuffer (unsigned char *str) //very ugly and rude, NO additional syntax checks!!!
{				//dump, read RFM22 or write...
unsigned char i,a,z=0;		//variables

if (str[0]=='W'){				//same as for R only 2 numbers more - > format W HH HH
		if (str[2]>47 && str[2]<58){		//write needs an address too
			i=(str[2]-48);
			}else {
			i=(str[2]-55);
			}
		if (str[3]>47 && str[3]<58){
			a=(str[3]-48);
			}else {
			a=(str[3]-55);
			}
			i=i<<4;
			z=(i | a);				// this is the write address
	if (str[5]>47 && str[5]<58){ 	//calculate the data
			i=(str[5]-48);
			}else {
			i=(str[5]-55);
			}
		if (str[6]>47 && str[6]<58){
			a=(str[6]-48);
			}else {
			a=(str[6]-55);
			}
			i=i<<4;	
			i=(i | a);				//i contains the data
			spiWrite (z,i);

		}		

if (str[0]=='D'){					//dump, write 128 registers of RFM22
		usartSendDump ();		//send to USART	
		}

if (str[0]=='S'){					//stop TX, going to READY mode...
		spiWrite (0x07,0x01);	
		}

if (str[0]=='T'){					//start TX...
		setParam ();
		spiWrite (0x07,0x09);	
		}

if (str[0]=='P'){					//set PN9 generator
		i=spiRead(0x71);			//read adress 0x71
		a=(i|0b00110000);			//set bits 4 i 5
		spiWrite (0x71,a);			//write it back
		}
if (str[0]=='C'){					//set CW carrier
		i=spiRead(0x71);			//read adress 0x71
		a=(i & 0b11111100);			//clear bits 0 i 1
		spiWrite (0x71,a);			//write it back
		}
if (str[0]=='G'){					//set GFSK modulation
		i=spiRead(0x71);			//read adress 0x71
		a=(i|0b00000011);			//set bits 0 i 1
		spiWrite (0x71,a);			//write it back
		}

if (str[0]=='O'){					//set OOK modulation
		i=spiRead(0x71);			//read address 0x71
		a=(i | 0b00000001);			//set bit 0
		a=(a & 0b11111101);			//clear bit 1
		spiWrite (0x71,a);			//write it back
		}

if (str[0]=='F'){					//set FSK modulation
		i=spiRead(0x71);			//read adress 0x71
		a=(i | 0b00000010);			//set bit 1
		a=(a & 0b11111110);			//clear bit 0
		spiWrite (0x71,a);			//write it back
		}

if (str[0]=='N'){					//set FIFO mode
		i=spiRead(0x71);			//read adress 0x71
		a=(i | 0b00100000);			//set bit 5
		a=(a & 0b11101111);			//clear bit 4
		spiWrite (0x71,a);			//write it back
		}

if (str[0]=='R'){		//read from address,converting from R HH to get address
		if (str[2]>47 && str[2]<58){	//first digit, see if it is a number
			i=(str[2]-48);				//if yes, from ASCII value to decimal....
			}else {
			i=(str[2]-55);				//if not then its ABCDEF
			}
		if (str[3]>47 && str[3]<58){   //check second digit
			a=(str[3]-48);
			}else {
			a=(str[3]-55);
			}
			i=i<<4;					//first number shift left, and OR with second number
			i=(i | a);				//address is now in variable i
			usartHEXmyoutput(spiRead(i));	//give address for spiRead and write as hex to USART
	}	

//postavljanje TX moda, transmit paketa sa podacima 6A i 45...
if (str[0]=='1'){				
	
	spiWrite(0x05, 0x07);//interrupt enable: packet sent,crc error, valid packet receive
	setParam ();
	PORTD = PORTD & (~(1<< PD4));//pin PD4 low, green LED ON

while (PIND & 0x20)
	{
	flag=0;
	spiWrite (0x7F,0x6A);//data 6A
	spiWrite (0x7F,0x45);//data 45
	
	spiWrite (0x07,0x09);//start TX 
	
	while (!flag);
	_delay_ms(150);	//wait 150 ms
		}
	PORTD = PORTD | (1<< PD4);	//pin PD4 high, green LED OFF
	spiWrite(0x05, 0x00);	//disable all interrupts
	spiWrite (0x07,0x01);//enable READY mode
	
}

if (str[0]=='2'){					
		
		spiWrite(0x05, 0x07);//interrupt enable packet sent,crc error, valid packet receive
		setParam ();
		spiWrite (0x07,0x05);			//enable READY mode,RX on
			
	while (PIND & 0x20)	;//wait for button T1 -->PD5 on port D
	
	spiWrite(0x05, 0x00);//disable all interrupts
	spiWrite (0x07,0x01);//enable READY mode
	
	}	

	return;
}


ISR(INT0_vect) 
{ 
unsigned char i;

	spiWrite (0x07,0x01);//enable READY mode 
	
	i=spiRead (0x03);//read interrupt/status registar 1
	spiRead (0x04);	//read interrupt/status registar 2
	i=i & 0b00000111;//leave first 3 bits

	//procedure for RX FIFO reset, read address 08...
	spiWrite (0x08,0x02);//write to addr 08 one, ffclrrx=1
	spiWrite (0x08,0x00);//write to addr 08 zero, ffclrrx=0
	
	spiWrite (0x08,0x01);//clear TX FIFO
	spiWrite (0x08,0x00);//clear TX FIFO

if (i>3) flag=1;

//last 2 bits on address 0x03 show CRC error or packet OK
if (i==1) {
		PORTD = PORTD | (1<< PD4);	//pin PD4 high, green OFF
		PORTD = (PORTD & (~(1<< PD3)));//pin PB1 low, red ON
		_delay_ms(20); //light for 20 ms
		PORTD = (PORTD | (1<< PD3));	//pin PB1 high, red OFF
		spiWrite (0x07,0x05);			//enable READY mode,RX on
		}

if (i==2) {
		PORTD = (PORTD | (1<< PD3));	//pin PB1 high, red OFF
		PORTD = PORTD & (~(1<< PD4));//pin PD4 low, green ON
		_delay_ms(20); //light for 20 ms
		PORTD = PORTD | (1<< PD4);	//pin PD4 high, green OFF
		spiWrite (0x07,0x05);			//enable READY mode,RX on
		}

}

int main (void)
{
	attinyinit ();	//init microcontroller peripherials
	usartInit (); 	//init USART
	spiInit ();		// init SPI 
	RFM22init ();	//init RFM 22 
	sei ();			//global interrupt enable
	
while (1)	//main program loop
{
	usartFillbuffer ();
	parseBuffer (scratch);
}	


	return 1;
	}


