//Richie White, Thomas Braum, Tristan Braum
//May 9, 2023
//CPE 301
//Final Project: Swamp Cooler

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>

#define RDA 0x80
#define TBE 0x20

volatile unsigned char *myUCSR0A = (unsigned char*) 0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char*) 0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char*) 0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int*)  0x00C4;
volatile unsigned char *myUDR0   = (unsigned char*) 0x00C6;

volatile unsigned char *my_ADMUX    = (unsigned char*) 0x7C;
volatile unsigned char *my_ADCSRB   = (unsigned char*) 0x7B;
volatile unsigned char *my_ADCSRA   = (unsigned char*) 0x7A;
volatile unsigned int  *my_ADC_DATA = (unsigned int*)  0x78;

volatile unsigned char *portB    = (unsigned char*) 0x25;
volatile unsigned char *portC    = (unsigned char*) 0x27;
volatile unsigned char *portD    = (unsigned char*) 0x2B;
volatile unsigned char *portE    = (unsigned char*) 0x2E;
volatile unsigned char *portF    = (unsigned char*) 0x31;
volatile unsigned char *portH    = (unsigned char*) 0x102;
volatile unsigned char *portDDRB = (unsigned char*) 0x24;
volatile unsigned char *portDDRC = (unsigned char*) 0x28;
volatile unsigned char *portDDRD = (unsigned char*) 0x2A;
volatile unsigned char *portDDRE = (unsigned char*) 0x2D;
volatile unsigned char *portDDRF = (unsigned char*) 0x30;
volatile unsigned char *portDDRH = (unsigned char*) 0x101;

bool printData = true;
unsigned long previousTime = 0;
unsigned long currentTime = 0;

//rtc variable
RTC_DS1307 rtc;

//DHT Setup
const int DHT_PIN = 13;
const int DHT_TYPE = DHT11;
DHT dht(DHT_PIN, DHT_TYPE);

//Stepper motor setup
const int STEPS = 2038;//Steps for full rotation
Stepper stepper(STEPS, 23, 27, 25, 29);//num steps and pins connected to stepper 
const int potPin = 0;//analog pin for potentiometer
unsigned int potVal = 1;
unsigned int prevPotVal = 0;

//LCD Setup
LiquidCrystal lcd(12, 11, 6, 5, 4, 3);

void setup() {
  U0init(9600);

  adc_init();

  //set fan port to output
  *portDDRC |= 1 << 0;// pin 37 output


  lcd.begin(16, 2);//lcd setup, 16 columns, two rows
  dht.begin();
  rtc.begin();
  
  rtc.adjust(DateTime(2023 , 5 , 9 , 18 , 0 , 0)); //2023 , May 9th , 6:00 pm
  

  //stepper motor setup
  stepper.setSpeed(15);

  //print temperature and humidity first time around
  printTempHumidity();

  //set STOP button to interrupt
  attachInterrupt(digitalPinToInterrupt(2), disable , RISING);
  //set RESET button to interrupt for true / false
  attachInterrupt(digitalPinToInterrupt(19) , reset , RISING);
}

void loop() {

  //update temperature and humidity reading once per minute
  currentTime = millis();
  if(printData && (currentTime - previousTime >= 60000)) {
    printTempHumidity();
    previousTime = currentTime;
  }


  //change state to error if water level is too low
  if (getWaterLevel() < 120) {
    changeState(1); //Red error state
  }
  else if(getTemp() < 24){
    changeState(2);//Green Idle
  }
  /*else if(button is pressed){
    changeState(4);//Yellow Disabled
  }*/
  else{
    changeState(3);//blue running
  }
}

//Changes the state of the machine based on data / interrupts
//1 = Error (Red)
//2 = Idle (Green)
//3 = Running (Blue)
//4 = Disabled (Yellow)
void changeState(int state){
  DateTime now;

  switch(state){
    case 1: //red

      //Prints time
      now = rtc.now();
      Serial.print("Error at: " + String(now.timestamp()));
      stopFan();

      //Print error to LCD
      //lcd.clear();
      //lcd.setCursor(0,0);
      lcd.print("Water Low!");
      //change LED color
      redLED(1);
      greenLED(0);
      blueLED(0);
      yellowLED(0);

      while(getWaterLevel() < 120){
        moveVent();
      } 
        break;

    case 2: //green
      //Prints time
      now = rtc.now();
      Serial.print("Idle at: " + String(now.timestamp()));
      stopFan();

      printTempHumidity();

      //change LED color
      redLED(0);
      greenLED(1);
      blueLED(0);
      yellowLED(0);

      while(getTemp() < 24){
        moveVent();
        if(getWaterLevel() < 120){
          break;
        }
      }
        break;

    case 3: //blue
      //Prints time
      now = rtc.now();
      Serial.print("Running at: " + String(now.timestamp()));
      startFan();

      printTempHumidity();

      //change LED color
      redLED(0);
      greenLED(0);
      blueLED(1);
      yellowLED(0);
        break;

    case 4: //yellow
      //Prints time
      now = rtc.now();
      Serial.print("Disabled at: " + String(now.timestamp()));
      stopFan();
      lcd.clear();            
      lcd.setCursor(0,0);
      printData = false;      
      
      //change LED color
      redLED(0);
      greenLED(0);
      blueLED(0);
      yellowLED(1);
        break;

    default:
        break;
  }
}

void moveVent(){
  //Move vent
  potVal = adc_read(potPin);
  moveStepper(potVal);
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
  return adc_read(1);//A1
}

//returns potentiometer reading
unsigned int getPot() {
  return adc_read(potPin);//A0
}

//Start fan motor, set digital pin 37 to HIGH
void startFan() {

  *portC |= 1 << 0; //pin 37 high

  //Prints time
  DateTime now = rtc.now();
  serialPrint("The fan turned on at: " + String(now.timestamp()));
}

//Stop fan motor, clear digital pin 37 to LOW
void stopFan() {
  *portC &= ~(1 << 0); //pin 37 low

  //Prints time
  DateTime now = rtc.now();
  serialPrint("The fan turned on at: " + String(now.timestamp()));
}

//print temperature and humidity data
void printTempHumidity() {
  lcd.clear();            
  lcd.setCursor(0,0);     
  lcd.print("Hello");
  lcd.print("Temp is ");
  lcd.print(getTemp());
  lcd.setCursor(0,1);     
  lcd.print("Humidity is ");
  lcd.print(getHumidity());
}

//Disable motor fan and set LED to yellow
void disable() {
  Serial.println("Disable Pressed.");
  changeState(4); //yellow
}

//Set the swamp cooler to a disabled state
void reset(){
  Serial.println("Reset pressed.");
  changeState(2);
}

void moveStepper(int potVal){

  unsigned int steps = map(potVal, 0, 1023, 0, STEPS);
  
  if(steps != prevPotVal) {
    stepper.step(steps - prevPotVal);
    prevPotVal = steps;
  }

  delay(10);
}

void blueLED(bool on){
  if(on){
    *portB |= 1<<4;//turns on pin 10
  }else{
    *portB &= ~(1<<4);//turns off pin 10
  }
}
void greenLED(bool on){
  if(on){
    *portH |= 1<<6;//turns on pin 9
  }else{
    *portH &= ~(1<<6);//turns off pin 9
  }
}
void yellowLED(bool on){
  if(on){
    *portH |= 1<<5;//turns on pin 8
  }else{
    *portH &= ~(1<5);//turns off pin 8
  }
}
void redLED(bool on){
  if(on){
    *portH |= 1<<4;//turns on pin 7
  }else{
    *portH &= ~(1<<4);//turns off pin 7
  }
}

void serialPrint(String phrase) {
  for(int i=0; phrase[i] != '\0'; i++) {
    U0putchar(phrase[i]);
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