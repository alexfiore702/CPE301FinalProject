// Alex Fiore William Leon
//CPE 301 Final Project


#include <DHT.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <Stepper.h>

#define RDA 0x80
#define TBE 0x20

//UART
volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;

//port A- motor control
volatile unsigned char *portA     = (unsigned char *) 0x22;
volatile unsigned char *portDDRA  = (unsigned char *) 0x21;
volatile unsigned char *pinA      = (unsigned char *) 0x20;

//port B- leds
volatile unsigned char *portB     = (unsigned char *) 0x25;
volatile unsigned char *portDDRB  = (unsigned char *) 0x24;

//port C- vent control buttons
volatile unsigned char *portC     = (unsigned char *) 0x28;
volatile unsigned char *portDDRC  = (unsigned char *) 0x27;
volatile unsigned char *pinC      = (unsigned char *) 0x26;

//port D- start/stop and reset
volatile unsigned char *portD     = (unsigned char *) 0x2B;
volatile unsigned char *portDDRD  = (unsigned char *) 0x2A;
volatile unsigned char *pinD      = (unsigned char *) 0x29;

//E3 = port E- LCD RS and temp/humidity
volatile unsigned char *portE     = (unsigned char *) 0x2E;
volatile unsigned char *portDDRE  = (unsigned char *) 0x2D;
volatile unsigned char *pinE      = (unsigned char *) 0x2C;

//port F- water sensor
volatile unsigned char *portF     = (unsigned char *) 0x31;
volatile unsigned char *portDDRF  = (unsigned char *) 0x30;
volatile unsigned char *pinF      = (unsigned char *) 0x2F;

//port G- lcd e
volatile unsigned char *portG     = (unsigned char *) 0x34;
volatile unsigned char *portDDRG  = (unsigned char *) 0x33;
volatile unsigned char *pinG      = (unsigned char *) 0x32;

//port H- LCD data
volatile unsigned char *portH     = (unsigned char *) 0x102;
volatile unsigned char *portDDRH  = (unsigned char *) 0x101;
volatile unsigned char *pinH      = (unsigned char *) 0x100;

//ADC
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//timer
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// Stepper motor
const int IN1=23, IN2=25, IN3=27, IN4=29;
const int stepsRev = 2038;
Stepper myStepper = Stepper(stepsRev, IN1, IN3, IN2, IN4);

//RTC
RTC_DS1307 myrtc;

// LCD
const int RS=53, EN=51, D4=49, D5=47, D6=45, D7=43;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Temp/humidity sensor
#define DHT11_PIN 17
#define tempMax 26
DHT dht(DHT11_PIN, DHT11);

// Water threshold value
#define waterMin 100
int waterLevel;

// Start/stop/reset 
#define STARTSTOP 19
#define RESET 18

int currentState, newState;
char stateNames[4][9] = {"Disabled", "Idle", "Running", "Error"};


void setup() {
//initialize serial
  U0Init(9600);
//initialize ADC
  adc_init();
  
//leds
  *portDDRB |= 0b11110000; // set PB4-7 (10-13) output
  *portB &= 0b00001111; // set low 
//fan motor
  *portDDRA |= 0b00000100; // set PA2 (24) output
  *portA &= 0b11111011; // set low
//vent controls
  *portDDRC &=0b01110111; // set PC3 (30) and PC7 (34) input
  *portC &= 0b01110111; // disable pullup
//start/stop
  *portDDRD &= 0b00000100; // set PD2 (19) input
  *portD &= 0b11111011; // disable pullup  

  currentState = 0;
  newState = 0;

//attach interrupt for start/stop and reset buttons
  attachInterrupt(digitalPinToInterrupt(STARTSTOP), ssPressed, RISING);
  attachInterrupt(digitalPinToInterrupt(RESET), resetPressed, RISING);
  
//start rtc at upload time
  myrtc.begin();
  myrtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

//initialize LCD display
  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0,0);
  
//initialize stepper
  myStepper.setSpeed(7);
//initialize temp/humidity
  dht.begin();
}

void loop() {
  if(currentState != newState){
    serialTransition();
    if(newState == 2){
      serialfanOn();
    }else if(currentState == 2){
      serialfanOff();
    }
  }
  currentState = newState;
  if(currentState == 0){
  //disabled
    ledWrite(7, 1);
    ledWrite(6, 0);
    ledWrite(5, 0);
    ledWrite(4, 0);

    //stop fan motor
    *portA &= 0b11111011;

    //update display
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Disabled");
  
  }else if (currentState == 1){
    // Idle
    ledWrite(7, 0);
    ledWrite(6, 1);
    ledWrite(5, 0);
    ledWrite(4, 0);
//read and display temp/humidity
    int chk = dht.readTemperature();
    displayValues();
    // Read water level
    waterLevel = adc_read(0);
  //stop fan
    *portA &= 0b11111011;
  //vent CW
    while(*pinC & 0b10000000){
      serialventCW();
      myStepper.step(100);
    }
  //vent CCW
    while(*pinC & 0b00001000){
      serialventCCW();
      myStepper.step(-100);
    }

 //test state
    if(waterLevel <= waterMin){
      newState = 3;
    }else if(dht.readTemperature() > tempMax){
      newState = 2;
    }
  }else if(currentState == 2){
  //Running
    ledWrite(7, 0);
    ledWrite(6, 0);
    ledWrite(5, 1);
    ledWrite(4, 0);
  // Read and display temp/humidity
    int chk = dht.readTemperature();
    displayValues();
  //read water level
    waterLevel = adc_read(0);
  //start fan
    *portA |= 0b00000100;

  //vent CW
    while(*pinC & 0b10000000){
      serialventCW();
      myStepper.step(100);
    }
  //vent CCW
    while(*pinC & 0b00001000){
      serialventCCW();
      myStepper.step(-100);
    }

    if(waterLevel < waterMin){
      newState = 3;

    }else if(dht.readTemperature() <= tempMax){
      newState = 1; //idle
    }
  }else if (currentState == 3){
  //Error
    ledWrite(7, 0);
    ledWrite(6, 0);
    ledWrite(5, 0);
    ledWrite(4, 1);

  //stop fan
    *portA &= 0b11111011;

  // update LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Error: Water");
    lcd.setCursor(0,1);
    lcd.print("level is too low");

  //read vent button
  //move vent up
    while(*pinC & 0b10000000){
      serialventCW();
      myStepper.step(100);
    }
  //move vent down
    while(*pinC & 0b00001000){
      serialventCCW();
      myStepper.step(-100);
    }
  }
  my_delay(1);
}

// UART functions
void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char getChar()
{
  return *myUDR0;
}
void putChar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void ledWrite(unsigned char ledPin, unsigned char currentState){
  if(currentState == 0){
    *portB &= ~(0x01 << ledPin);
  }
  else{
    *portB |= 0x01 << ledPin;
  }
}

void serialTime(){
  DateTime now = myrtc.now();
  putChar(0x30 + (now.hour()/10)%10);
  putChar(0x30 + now.hour()%10);
  putChar(':');
  putChar(0x30 + (now.minute()/10)%10);
  putChar(0x30 + now.minute()%10);
  putChar(':');
  putChar(0x30 + (now.second()/10)%10);
  putChar(0x30 + now.second()%10);
}


void serialTransition(){
  const char string1[] = "Transition from state ";
  const char string2[] = " to state ";
  const char string3[] = " at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  putChar(0x30 + currentState);
  for(int i =0; i < strlen(string2); i++ ) {
    char c = string2[i];
    putChar(c);
  }
  putChar(0x30 + newState);
  for(int i =0; i < strlen(string3); i++ ) {
    char c = string3[i];
    putChar(c);
  }
  serialTime();
  putChar('\n');
}

void serialventCW(){
  const char string1[] = "Moving vent CW at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  serialTime();
  putChar('\n');
}

void serialventCCW(){
  const char string1[] = "Moving vent CCW at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  serialTime();
  putChar('\n');
}

void serialfanOn(){
  const char string1[] = "Fan turned on at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  serialTime();
  putChar('\n');
}

void serialfanOff(){
  const char string1[] = "Fan turned off at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  serialTime();
  putChar('\n');
}

// ISR start/stop button
void ssPressed(){
  if(currentState == 0){
    newState = 1;
  }else{
    newState = 0;
  }
}

// ISR reset button
void resetPressed(){
  waterLevel = adc_read(0);
  if(currentState == 3 && waterLevel > waterMin){
    newState = 1;
  }
}

void displayValues(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(dht.readTemperature());
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(dht.readHumidity());
  lcd.print("%");
}

void my_delay(unsigned int freq)
{
  
  double period = 1.0/double(freq);
  double half_period = period/ 2.0f;
  double clk_period = 0.0000000625;
  unsigned int ticks = half_period / clk_period;

  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);

  * myTCCR1A = 0x0;
  * myTCCR1B |= 0b00000001;

  while((*myTIFR1 & 0x01)==0);
  *myTCCR1B &= 0xF8;         
  *myTIFR1 |= 0x01;
}
void adc_init(){
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num){
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11110111;
  if(adc_channel_num > 7)
  {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

