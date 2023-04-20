//Thomas Braun, Richard White, Tristan Braum
//Final Project: Swamp Cooler

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>

#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* my_ADMUX   = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB  = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA  = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char *portDDRB = (unsigned char *) 0x24;
volatile unsigned char *portB    = (unsigned char *) 0x25;
volatile unsigned char *portH    = (unsigned char *) 0x102;
volatile unsigned char *portDDRH = (unsigned char *) 0x101;

int state = 0;

//Setup dht
const int DHT_PIN = 7;
const int DHT_TYPE = DHT11;
DHT dht(DHT_PIN, DHT_TYPE);
float humidity = 0.0;
float temperature = 0.0;

//Setup stepper
const int STEPS = 64; //64 steps per revolution
Stepper stepper(STEPS, 23, 27, 25, 29); //num steps and pins connected to stepper
int potVal = 0;
int prevVal = 0;

//Setup LCD
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {

  U0init(9600);

  //setup lcd
  lcd.begin(16, 2); // two rows displayed. 16 columns

  dht.begin();
  adc_init();
  stepper.setSpeed(100);
  *portDDRB |= 0b11110000;
  *portDDRH |= 0b01100000;
  attachInterrupt(digitalPinToInterrupt(2), isr, RISING);//?
}

void loop() {
  // put your main code here, to run repeatedly:
  if(state == 0){
    disable();
  }
  else{

    *portH != 1<<7;



  }


}

void disable(){

}


void print_int(unsigned int out_num){
}

void motor_intialization(){
  *portDDRB |= 0b10000000;
}


void U0init(unsigned long U0baud) {
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit() {
   return *myUCSR0A & RDA;
}

unsigned char U0getchar() {
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata) {
  while ((*myUCSR0A & TBE)==0);
    *myUDR0 = U0pdata;
}


//timer setup function
void setup_timer_regs() {
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;

  *myTIFR1  |= 0x01; 
  *myTIMSK1 |= 0x01; 
}

//initialize ADC registers
void adc_init() {
  *my_ADCSRA |= 0b10000000;
  *my_ADCSRA &= 0b11011111; 
  *my_ADCSRA &= 0b11110111; 
  *my_ADCSRA &= 0b11111000; 
  *my_ADCSRB &= 0b11110111; 
  *my_ADCSRB &= 0b11111000; 
  *my_ADMUX  &= 0b01111111; 
  *my_ADMUX  |= 0b01000000; 
  *my_ADMUX  &= 0b11011111; 
  *my_ADMUX  &= 0b11100000; 
}

//read analog values from ADC channel
unsigned int adc_read(unsigned char adc_channel_num) {
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11110111;
  if(adc_channel_num > 7) {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  
  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

//ISR
ISR(TIMER1_OVF_vect) {
  *myTCCR1B &= 0xF8; 
  *myTCNT1 = (unsigned int) (65535 - (unsigned long) (currentTicks));
  *myTCCR1B |= 0x01;
  if (currentTicks != 65535) {
    *port_b ^= 0x40;
  }
}