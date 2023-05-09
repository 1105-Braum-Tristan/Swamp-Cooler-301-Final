//Richie White, Thomas Braum, Tristan Braum
//May 9, 2023
//CPE 301
//Final Project: Swamp Cooler

//i think the port for the motor is wrong
//i dont have the ports for the buttons setup yet either
//we should defo check all the ports to make sure they are correct
//bool test added until i can figure out how to press a button
  
//What on earth does this mean from the requirements 
//"Record the time and date every time the motor is turned on or off. This information should be transmitted to a host computer (over USB)"

//How do we satisfy this requirement? Instead of isr function, we now use disable() and reset()
//"Start button should be monitored using an ISR"

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>

#define RDA 0x80
#define TBE 0x20  

//A1 water level sensor , PF1
//STOP button: pwm 2 , PE0
//RESET button: digital 52 , PB1
//Potentiometer: Analog 0, PF0

volatile unsigned char *myUCSR0A = (unsigned char*) 0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char*) 0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char*) 0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int*)  0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;


volatile unsigned char *my_ADMUX    = (unsigned char*) 0x7C;
volatile unsigned char *my_ADCSRB   = (unsigned char*) 0x7B;
volatile unsigned char *my_ADCSRA   = (unsigned char*) 0x7A;
volatile unsigned int  *my_ADC_DATA = (unsigned int*)  0x78;

volatile unsigned char *portB    = (unsigned char*) 0x25;
volatile unsigned char *portE    = (unsigned char*) 0x2E;
volatile unsigned char *portF    = (unsigned char*) 0x31;
volatile unsigned char *portH    = (unsigned char*) 0x102;
volatile unsigned char *portDDRB = (unsigned char*) 0x24;
volatile unsigned char *portDDRE = (unsigned char*) 0x2D;
volatile unsigned char *portDDRF = (unsigned char*) 0x30;
volatile unsigned char *portDDRH = (unsigned char*) 0x101;

bool test = false;

//DHT Setup
const int DHT_PIN = 13;
const int DHT_TYPE = DHT11;
DHT dht(DHT_PIN, DHT_TYPE);

//Stepper motor setup
const int STEPS = 4096;//Steps for full rotation
Stepper stepper(STEPS / 2, 23, 27, 25, 29);//num steps and pins connected to stepper 
const int potPin = A0;//analog pin for potentiometer
int currentPosition = 0;
int previousPosition = 0;

//LCD Setup
LiquidCrystal lcd(12, 11, 6, 5, 4, 3);
const int backlightPin = 15;//pin number for the backlight pin, used for brightness

//Fan Motor
int motorState = 0;//0 off, 1 on
unsigned long motorTime = 0;//Time of when motor state was changed

//RTC_DS3231 rtc;//Initialize the RTC for time stamp


void setup() {

  U0init(9600);

  //setup lcd
  lcd.begin(16, 2);//16 columns, two rows

  //sets lcd to max brightness
  //pinMode(backlightPin, OUTPUT);
  //digitalWrite(backlightPin, HIGH);
  //analogWrite(backlightPin, 255);

  dht.begin();
  adc_init();

  //rtc.begin();

  stepper.setSpeed(0);
  pinMode(potPin, INPUT);

  //?
  *portDDRB |= 0xF0;
  *portDDRH |= 0x60;

  //set STOP button to interrupt
  attachInterrupt(digitalPinToInterrupt(2), disable, RISING);
  //set REST button to interrupt for true / false
  attachInterrupt(digitalPinToInterrupt(20) , reset , RISING);
}

void loop() {
  if(getWaterLevel < 120) {
    changeLEDState(1); //error
    stopFan();

    //print error message to LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Water Low!");
  }
  else {
    
    //if (reset()) { reset() is void??
      printTempHumidity();
      moveStepper();
      
      if (getTemp() >= 24) {
        changeLEDState(3); //running
        startFan();
      }
      else {
        changeLEDState(2); //idle
        stopFan();
      }
    }
  //}
}


//returns temperature data
float getTemp() {
  return dht.readTemperature();
}

//returns humidity data
float getHumidity() {
  return dht.readHumidity();
}

//returns water level of resevoir
unsigned int getWaterLevel() {
  return adc_read(1);
}

//returns potentiometer reading
unsigned int getPot() {
  return adc_read(0);
}

//Start fan motor, set digital pin 37 to HIGH
void startFan() {
  motorTime = millis(); // Record the time
  logMotorTime();
  *portH |= 0x20;
}

//Stop fan motor, clear digital pin 37 to LOW
void stopFan() {
  motorTime = millis(); // Record the time
  *portH &= 0xDF;
}
//Logs the time the state of the motor changes
void logMotorTime(){
  //somehow send the time to a usb?
}

//Changes the state of the LED when requested from the loop function
//sets all unused LED's to LOW
void changeLEDState(int numLED){
  switch(numLED){
    case 1: //red
      redLED(1);
      greenLED(0);
      blueLED(0);
      yellowLED(0);
      break;
    case 2: //green
      redLED(0);
      greenLED(1);
      blueLED(0);
      yellowLED(0);
      break;
    case 3: //blue
      redLED(0);
      greenLED(0);
      blueLED(1);
      yellowLED(0);
      break;
    case 4: //yellow
      redLED(0);
      greenLED(0);
      blueLED(0);
      yellowLED(1);
      break;
    default:
        break;
  }
}

//print temperature and humidity data
void printTempHumidity() {
  lcd.clear();            
  lcd.setCursor(0,0);     
  lcd.print("Temp is ");
  lcd.print(getTemp());
  lcd.setCursor(0,1);     
  lcd.print("Humidity is ");
  lcd.print(getHumidity());
}

//Disable motor fan and set LED to yellow
void disable() {
  changeLEDState(4); //yellow
  stopFan();
  *portB &= ~(1<<4); //turn off LCD? 
  moveStepper();
}

void reset() {
  test = true;
}

//moves stepper motor
void moveStepper(){
  //set speed
  stepper.setSpeed(map(getPot(), 0, 1023, 0, 100));

  //calc num steps to move stepper
  int stepsToMove = abs(currentPosition - previousPosition);
  previousPosition = currentPosition;
  
  if (currentPosition < previousPosition){
    stepper.step(stepsToMove);
  } else if (currentPosition > previousPosition){
    stepper.step(-stepsToMove);
  }
  
  //Update the current position
  currentPosition += stepsToMove;
  
  //Make sure it doesnt go over half a rotation
  if (currentPosition >= STEPS / 2){
    currentPosition = STEPS / 2;
  } else if (currentPosition <= -STEPS / 2){
    currentPosition = -STEPS / 2;
  }
}

//set comms rate
void U0init(unsigned long U0baud) {
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

//display output to serial monitor
void U0putchar(unsigned char U0pdata) {
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

//Initialize ADC
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

//Read analog data
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

//set LED to HIGH or LOW based on input
void blueLED(bool on){
  if(on = 1){
    *portB |= 1<<2;//turns on pin 10
  }else{
    *portB &= ~(1<<2);//turns off pin 10
  }
}
void greenLED(bool on){
  if(on = 1){
    *portB |= 1<<1;//turns on pin 9
  }else{
    *portB &= ~(1<<1);//turns off pin 9
  }
}
void yellowLED(bool on){
  if(on = 1){
    *portB |= 1<<0;//turns on pin 8
  }else{
    *portB &= ~(1<<0);//turns off pin 8
  }
}
void redLED(bool on){
  if(on = 1){
    *portB |= 1<<7;//turns on pin 7
  }else{
    *portB &= ~(1<<7);//turns off pin 7
  }
}