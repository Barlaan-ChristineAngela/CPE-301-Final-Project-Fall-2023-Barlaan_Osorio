#include <dht.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

//temp and humidity sensor
dht DHT;
#define DHT11_PIN 23

//LCD
const int RS = 46, EN = 44, D4 = 40, D5 = 38, D6 = 36, D7 = 34;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 60000;

//ADC, water sensor
volatile unsigned char *portH = (unsigned char *) 0x102;
volatile unsigned int *portDDRH = (unsigned *) 0x0101;
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//fan motor
volatile unsigned char *portA = (unsigned char*) 0x22;        //enable high (|= make 1s) or low (&= make 0s);
volatile unsigned char *portDDRA = (unsigned char*) 0x21;     //input (&= make 0s), output (|= make 1s)
volatile unsigned char *pin_a = (unsigned char*) 0x20;        //read the state of the pin

//stepper motor
const int stepsPerRevolution = 2038; //stepper motor
Stepper myStepper = Stepper(stepsPerRevolution, 2, 4, 3, 5); //stepper motor
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

//button, ISR
volatile unsigned char *portD = (unsigned char*) 0x2B;        //enable high (|= make 1s) or low (&= make 0s);
volatile unsigned char *portDDRD = (unsigned char*) 0x2A;     //input (&= make 0s), output (|= make 1s)
volatile unsigned char *pin_d = (unsigned char*) 0x29; 
volatile bool buttonPressed = false;

void setup() {

  //fan motor
  //set PA0(pin 22), PA2(pin24), PA4(pin26) to output
  *portDDRA |= 0b00010101;

  //LCD
  lcd.begin(16,2);

  //ADC, water sensor, stepper motor
  adc_init();
  //set pb6 and pb7 to input
  *portDDRB= 0x3F;
  //set ph4 and ph3 to output
  *portDDRH= 0x18;
  //set ph4 and to high for water sensor
  *portH= 0x10;
  //enable pullup resistor on pb6 and pb7
  *portB= 0xC0;

  //UART
  U0init(9600);

  //button, ISR
  //set PD3(pin 18) to input, becomes the button pin
  *portDDRD &= 0b11110111;
  attachInterrupt(digitalPinToInterrupt(18), StartStopButtonISR, LOW);
}

void loop() {

  TempAndHumiditySensor();

  FanMotor();

  StepperMotor();

  WaterSensor();

  RTCModule();
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

void TempAndHumiditySensor(){

  int chk = DHT.read11(DHT11_PIN);

  unsigned long currentTime = millis();
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

void WaterSensor(){
  unsigned int waterVal= adc_read(0);
  checkWaterLevel(waterVal);
}

void checkWaterLevel(unsigned int input){
  if(input < 300){
    printString("Water level is too low");
    //set ph3 high
    *portH |= 0x8;
    //delay(1000);
  }
  else{
    //set ph3 low
    *portH &= 0xF7;
  }
}

void FanMotor(){

  /*
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);
  analogWrite(speedPin, HIGH);
  */

  *portA |= 0b00000100;
  *portA &= 0b11101111;
  *portA |= 0b00000001;
}

void StepperMotor(){
  if(*pin_b & 0x80){
    printString("Vent position change at:");
    displayCurrentTime();
    myStepper.setSpeed(10);
    myStepper.step(stepsPerRevolution);
  }
  if(*pin_b & 0x40){
    printString("Vent position change at:");
    displayCurrentTime();
    myStepper.setSpeed(10);
    myStepper.step(-stepsPerRevolution);
  }
}

//RTC module
void RTCModule(){
  //RTC module
  if (! rtc.begin()) {
    printString("Couldn't find RTC");
    //Serial.flush();
    while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}
void displayCurrentTime(){
  DateTime now = rtc.now();
  printString("Date & Time: ");
  printString(now.year(), DEC);
  printString('/');
  printString(now.month(), DEC);
  printString('/');
  printString(now.day(), DEC);
  printString(" (");
  printString(daysOfTheWeek[now.dayOfTheWeek()]);
  printString(") ");
  printString(now.hour(), DEC);
  printString(':');
  printString(now.minute(), DEC);
  printString(':');
  printString(now.second(), DEC);
}

//button, ISR
void StartStopButtonISR(){
  //if pin18 is LOW
  if((*pin_a & 0b00001000) == 0){
    buttonPressed = true;8
  }
  else{
    buttonPressed = false;
  }
}
