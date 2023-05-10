//Richie White, Thomas Braun, Tristan Braum
//May 9, 2023
//CPE 301
//Final Project: Swamp Cooler

//i think the port for the motor is wrong, should come from port C (already defined it)
//we should defo check all the ports to make sure they are correct
//bool test added until i can figure out how to press a button

//CLOCK STUFF
  //copied some stuff straight from a library, should work, but uses library functions such as serial.print
  //the time span calculation is done from a method at the bottom of the file

//LCD stuff idk if its useful to us at all
  //sets lcd to max brightness
  //pinMode(backlightPin, OUTPUT);
  //digitalWrite(backlightPin, HIGH);
  //analogWrite(backlightPin, 255);

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>

#define RDA 0x80
#define TBE 0x20

//STOP button: pwm 2 , PE0
//RESET button: comms 20 , PD1
//Potentiometer: Analog 0, PF0

volatile unsigned char *myUCSR0A = (unsigned char*) 0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char*) 0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char*) 0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int*)  0x00C4;
volatile unsigned char *myUDR0   = (unsigned char*)0x00C6;


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

//until i figure this RESET button thing out
bool test = false;

//Global timekeeping variable
RTC_DS1307 rtc;
//DateTime start;

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
//const int backlightPin = 15;//pin number for the backlight pin, used for brightness

void setup() {
  
  Wire.begin();
  U0init(9600);

  lcd.begin(16, 2);//lcd setup, 16 columns, two rows
  dht.begin();
  adc_init();

  /*
  //rtc setup
  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  if (! rtc.begin()) {
    //Serial.println("Couldn't find RTC");
    //Serial.flush();
    while (1) delay(10);
  }
  if (! rtc.isrunning()) {
    //Serial.println("RTC is NOT running, let's set the time!");
    rtc.adjust(DateTime(2023 , 5 , 9 , 18 , 0 , 0)); //2023 , May 9th , 6:00 pm
  }
  */

  //stepper motor setup
  stepper.setSpeed(0);
  pinMode(potPin, INPUT);

  //set STOP button to interrupt
  attachInterrupt(digitalPinToInterrupt(2), disable , RISING);
  //set RESET button to interrupt for true / false
  attachInterrupt(digitalPinToInterrupt(19) , reset , RISING);
}

void loop() {
  //need to find a way to print this using U0putchar instead of serial.print
  //char buffer[] = "YYMMDD-hh:mm:ss";
  //Serial.println(now.toString(buf2));

  if(getWaterLevel < 120) {
    changeLEDState(1); //error
    stopFan();

    //print error message to LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Water Low!");
  }
  else {
    if (test) {
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
  }
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
  return adc_read(0);//A0
}

//Start fan motor, set digital pin 37 to HIGH
void startFan() {
  //*portH |= 0x20;//old
  *portC |= 1 << 0;//Fan port

  //Prints time
  DateTime now = rtc.now();
  serialPrint("The fan turned on at: " + String(now.timestamp()));
  //Serial.print("The fan turned on at: " + String(now.timestamp()));

}

//Stop fan motor, clear digital pin 37 to LOW
void stopFan() {
  //*portH &= 0xDF;  //90% sure this is supposed to be //old
  *portC &= !(1 << 0);//Fan port


  //Prints time
  DateTime now = rtc.now();
  serialPrint("The fan turned on at: " + String(now.timestamp()));
}


//Changes the state of the LED when requested from the loop function
//sets all unused LED's to LOW
void changeLEDState(int numLED){
  DateTime now;
  switch(numLED){
    case 1: //red
      redLED(1);
      greenLED(0);
      blueLED(0);
      yellowLED(0);

      //Prints time
      now = rtc.now();
      Serial.print("Error at: " + String(now.timestamp()));

        break;
    case 2: //green
      //Prints time
      now = rtc.now();
      Serial.print("Idle at: " + String(now.timestamp()));

      redLED(0);
      greenLED(1);
      blueLED(0);
      yellowLED(0);
        break;
    case 3: //blue
      //Prints time
      now = rtc.now();
      Serial.print("Running at: " + String(now.timestamp()));

      redLED(0);
      greenLED(0);
      blueLED(1);
      yellowLED(0);
        break;
    case 4: //yellow
      //Prints time
      now = rtc.now();
      Serial.print("Disabled at: " + String(now.timestamp()));

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

  lcd.clear();            
  lcd.setCursor(0,0);

  moveStepper();
}

//Set the swamp cooler to a disabled state
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

  //Prints time
  DateTime now = rtc.now();
  Serial.print("The stepper has moved at: " + String(now.timestamp()));
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
  if(on){
    *portB |= 1<<5;//turns on pin 10
  }else{
    *portB &= ~(1<<5);//turns off pin 10
  }
}
void greenLED(bool on){
  if(on){
    *portH |= 1<<7;//turns on pin 9
  }else{
    *portH &= ~(1<<7);//turns off pin 9
  }
}
void yellowLED(bool on){
  if(on){
    *portH |= 1<<6;//turns on pin 8
  }else{
    *portH &= ~(1<<6);//turns off pin 8
  }
}
void redLED(bool on){
  if(on){
    *portH |= 1<<5;//turns on pin 7
  }else{
    *portH &= ~(1<<5);//turns off pin 7
  }
}

void serialPrint(String phrase) {
  for(int i=0; phrase[i] != '\0'; i++) {
    U0putchar(phrase[i]);
  }
}

//display the time span between two times
/*
void showTimeSpan(const char* txt, const TimeSpan& ts) {
    serialPrint(txt);
    serialPrint(" ");
    //convert to decial
    serialPrint(ts.days(), DEC);
    serialPrint(" days ");
    //convert decimal
    serialPrint(ts.hours(), DEC);
    serialPrint(" hours ");
    //convert decimal
    serialPrint(ts.minutes(), DEC);
    serialPrint(" minutes ");
    serialPrint(ts.seconds(), DEC);
    serialPrint(" seconds (");
    serialPrint(ts.totalseconds(), DEC);
    serialPrint(" total seconds)");
    serialPrint('\n');
}
*/