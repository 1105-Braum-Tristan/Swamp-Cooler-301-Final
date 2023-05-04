//Thomas Braun, Richard White, Tristan Braum
//Final Project: Swamp Cooler

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>

#define RDA 0x80
#define TBE 0x20  

//blue led 10
//green 9
//yellow 8
//red 7
//A1 water level sensor

volatile unsigned char *myUCSR0A = (unsigned char*)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char*)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char*)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int*) 0x00C4;

volatile unsigned char *my_ADMUX    = (unsigned char*) 0x7C;
volatile unsigned char *my_ADCSRB   = (unsigned char*) 0x7B;
volatile unsigned char *my_ADCSRA   = (unsigned char*) 0x7A;
volatile unsigned int  *my_ADC_DATA = (unsigned int*)  0x78;

volatile unsigned char *portDDRB = (unsigned char*) 0x24;
volatile unsigned char *portB    = (unsigned char*) 0x25;
volatile unsigned char *portH    = (unsigned char*) 0x102;
volatile unsigned char *portDDRH = (unsigned char*) 0x101;

int state = 1;//0?

//Setup dht
const int DHT_PIN = 13;
const int DHT_TYPE = DHT11;
DHT dht(DHT_PIN, DHT_TYPE);
float humidity = 0.0;
float temperature = 0.0;

//Setup stepper
const int STEPS = 4096;
Stepper stepper(STEPS / 2, 23, 27, 25, 29); //num steps and pins connected to stepper 
const int potPin = A0;
int potVal = 0;
int motorSpeed = 0;
int currentPosition = 0;
int previousPosition = 0;

//Setup LCD
LiquidCrystal lcd(12, 11, 6, 5, 4, 3);
const int backlightPin = 15; //pin number for the backlight pin

void setup() {

  U0init(9600);

  //setup lcd
  lcd.begin(16, 2);//16 columns, two rows
  pinMode(backlightPin, OUTPUT);
  digitalWrite(backlightPin, HIGH);
  analogWrite(backlightPin, 255);

  dht.begin();
  adc_init();

  stepper.setSpeed(0);
  pinMode(potPin, INPUT);

  *portDDRB |= 0xF0;
  *portDDRH |= 0x60;

  attachInterrupt(digitalPinToInterrupt(2), isr, RISING);
}

void loop() {

  if(state == 0){
    disable();
  }
  else{


    delay(1000);//delay readings
    //get values using sensor
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    unsigned int waterLevel = adc_read(5);
    unsigned int potRead = adc_read(0);



    potVal = adc_read(potPin);
    motorSpeed = map(potVal, 0, 1023, 0, 100);
    stepper.setSpeed(motorSpeed);
    moveStepper();

    lcd.clear();//reset screen
    lcd.setCursor(0,0);//move curser back to starting position
    lcd.print("Temp is ");
    lcd.print(temperature);
    lcd.setCursor(0,1);//set curser to the start of the second row
    lcd.print("Humidity is");
    lcd.print(humidity);

    //turns on fan
    fan(temperature);
  }
}

void isr() {
  if(state != 0){
    disable();
    state = 0;
  }
  else{
    state = 1;
    *portB &= ~(1<<4);
    *portH |= 1<<6;
  }
}

void fan(float temp) {
  if (temp > 20) {
    *portH |= 0x20; //set pin 37 PH5 to high
  } else {
    *portH &= 0xDF;//clear pin 13 PB5 to low
  }
}

void disable(){
  //turn everything off
  *portB |= 0x10;
  *portB &= 0x1F;
  *portH &= 0x9F;
  lcd.clear();
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

void moveStepper() {
  //calc num steps to move stepper
  int stepsToMove = abs(currentPosition - previousPosition);
  previousPosition = currentPosition;
  
  if (currentPosition < previousPosition) {
    stepper.step(stepsToMove);
  } else if (currentPosition > previousPosition) {
    stepper.step(-stepsToMove);
  }
  
  //Update the current position
  currentPosition += stepsToMove;
  
  //Make sure it doesnt go over half a rotation
  if (currentPosition >= STEPS / 2) {
    currentPosition = STEPS / 2;
  } else if (currentPosition <= -STEPS / 2) {
    currentPosition = -STEPS / 2;
  }
}

