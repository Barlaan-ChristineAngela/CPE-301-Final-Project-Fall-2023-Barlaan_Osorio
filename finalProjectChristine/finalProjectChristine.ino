#include <dht.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>

//temp and humidity sensor
dht DHT;
#define DHT11_PIN 23

//LCD
const int RS = 46, EN = 44, D4 = 40, D5 = 38, D6 = 36, D7 = 34;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 60000;

//ADC, water sensor
volatile unsigned char *portF = (unsigned char*) 0x31;        //enable high (|= make 1s) or low (&= make 0s);
volatile unsigned char *portDDRF = (unsigned char*) 0x30;     //input (&= make 0s), output (|= make 1s)
volatile unsigned char *pin_f = (unsigned char*) 0x2F;        //read the state of the pin
volatile unsigned char *portH = (unsigned char*) 0x102;        //enable high (|= make 1s) or low (&= make 0s);
volatile unsigned char *portDDRH = (unsigned char*) 0x101;     //input (&= make 0s), output (|= make 1s)
volatile unsigned char *pin_h = (unsigned char*) 0x100;  
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//fan motor, stop button
volatile unsigned char *portA = (unsigned char*) 0x22;        //enable high (|= make 1s) or low (&= make 0s);
volatile unsigned char *portDDRA = (unsigned char*) 0x21;     //input (&= make 0s), output (|= make 1s)
volatile unsigned char *pin_a = (unsigned char*) 0x20;        //read the state of the pin

//stepper motor and LEDs
const int stepsPerRevolution = 2038; //stepper motor
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11); //stepper motor
volatile unsigned char *portB = (unsigned char *) 0x25; //stepper motor
volatile unsigned int *portDDRB = (unsigned *) 0x24;
volatile unsigned char *pin_b= (unsigned char *) 0x23;

//RTC module
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {
  "Sunday",
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday"
};

//UART
#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char*)0xC0;
volatile unsigned char *myUCSR0B = (unsigned char*)0xC1;
volatile unsigned char *myUCSR0C = (unsigned char*)0xC2;
volatile unsigned int  *myUBRR0  = (unsigned int*) 0xC4;
volatile unsigned char *myUDR0   = (unsigned char*)0xC6;

//start button, ISR
volatile unsigned char *portE = (unsigned char*) 0x2E;        //enable high (|= make 1s) or low (&= make 0s);
volatile unsigned char *portDDRE = (unsigned char*) 0x2D;     //input (&= make 0s), output (|= make 1s)
volatile unsigned char *pin_e = (unsigned char*) 0x2C;        //read the state of the pin
volatile bool disabled = false;

void setup() {
  
  Serial.begin(9600);

  //UART
  U0init(9600);

  //LCD
  lcd.begin(16,2);

  //ADC, water sensor
  adc_init();
  //set ph4(pin 7) as output
  *portDDRH |= 0b00010000;
  //set ph4(pin 7) to HIGH
  *portH |= 0b00010000;

  //stepper motor
  //set pb6(pin 12) and pb7(pin 13) to input, stepper motor button
  *portDDRB &= 0x3F;
  //enable pullup resistor on pb6(pin 12) and pb7(pin 13), stepper motor button
  *portB |= 0xC0;

  //fan motor
  //set PA0(pin 22), PA2(pin24), PA4(pin26) to output
  *portDDRA |= 0b00010101;

  //start button and stop button, ISR
  //set PE4(pin 2) to input, the start button; PE5(pin 3) to input, the stop button
  *portDDRE &= 0b11001111;
  //enable pullup on PE4(pin 2) and PE5(pin 3)
  *portE |= 0b00110000;
  attachInterrupt(digitalPinToInterrupt(2), StartButtonISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), StopButtonISR, CHANGE);

  //LEDs, all outputs
  //Red, PB3(pin 50); Yellow, PB2(pin 51); Blue, PB1(pin 52); Green, PB0(pin 53)
  *portDDRB |= 0b00001111;

  //RTC Module
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {

  FanMotor();
  TempAndHumiditySensor();
  WaterSensor();
  StepperMotor();

  if(disabled == true){
    //high for PB2 yellow LED
    *portB |= 0b000000100;
  }
  if(disabled == false){
    //low for PB2 yellow LED
    *portB &= 0b11111011;
    //high for PB1 blue LED
    *portB |= 0b00000010;
  }
}

//water sensor
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  |= 0b11111111; // set voltage reference to internal reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

//UART
void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
void printString(const char* message){
  for(int i = 0; message[i] != '\0'; i++){
    U0putchar(message[i]);
  }
}


//temp and humdity sensor
void TempAndHumiditySensor(){

  int chk = DHT.read11(DHT11_PIN);
  unsigned long currentTime = millis();

  if(disabled == true){
    lcd.clear();
  }
  if(disabled == false){

    bool firstIteration = true;
    if(firstIteration == true){
      lcd.setCursor(0,0);
      lcd.print("Temperature=");
      lcd.print(DHT.temperature);
      lcd.setCursor(0,1);
      lcd.print("Humidity = ");
      lcd.print(DHT.humidity);

      firstIteration = false;
    }
    if(currentTime - lastUpdateTime >= updateInterval){
      //print to LCD
      lcd.setCursor(0,0);
      lcd.print("Temperature=");
      lcd.print(DHT.temperature);
      lcd.setCursor(0,1);
      lcd.print("Humidity = ");
      lcd.print(DHT.humidity);

      lastUpdateTime = currentTime;
    }
  }
  
}

//water sensor
void WaterSensor(){
  if(disabled == false){
    unsigned int waterVal= adc_read(0);
    checkWaterLevel(waterVal);
    if(waterVal > 350){
      //set PB3 high for red LED
      *portB |= 0b00001000;
      //set PB2, PB1, PB0 to low
      *portB &= 0b11111000;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ERROR");
    }
  }
  if(disabled == true){
    //set PB0 low for green LED
    *portB &= 0x111111110;
  }
}
void checkWaterLevel(unsigned int input){
  if(input < 300){
    Serial.println("Water level is too low");
    //set PB0 low for green LED
    *portB &= 0b111111110;
    //set PB3 high for red LED
    *portB |= 0b000001000;
    //delay(1000);
  }
  else{
    //set PB0 high for green LED
    *portB |= 0b000000001;
    //set PB3 low for red LED
    *portB &= 0b11110111; 
  }
}

//fan motor
void FanMotor(){
  if(disabled == true){
    *portA &= 0b11111011;
    *portA &= 0b11101111;
    *portA &= 0b11111110;
  }
  if(disabled == false){
    *portA |= 0b00000100;
    *portA &= 0b11101111;
    *portA |= 0b00000001;
  }
}

//stepper motor
void StepperMotor(){
  if(disabled == false){
    if(*pin_b & 0x80){
      Serial.print("Vent position change at:");
      displayCurrentTime();
      myStepper.setSpeed(10);
      myStepper.step(stepsPerRevolution);
    }
    if(*pin_b & 0x40){
      Serial.print("Vent position change at:");
      displayCurrentTime();
      myStepper.setSpeed(10);
      myStepper.step(-stepsPerRevolution);
    }
  }
}

//RTC module
void RTCModule(){
  //RTC module
  if (! rtc.begin()) {
    printString("Couldn't find RTC");
    Serial.flush();
    while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void displayCurrentTime(){
  DateTime now = rtc.now();
  Serial.print("Date & Time: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);
}

//start button, ISR
void StartButtonISR(){
  if((*pin_e & 0b00010000) == 0){
    disabled = false;
  }
}

void StopButtonISR(){
  if((*pin_e & 0b00100000) == 0){
    disabled = true;
  }
}