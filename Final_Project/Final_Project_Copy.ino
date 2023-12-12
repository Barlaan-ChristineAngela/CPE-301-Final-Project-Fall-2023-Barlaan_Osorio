#include <dht.h> //temp and humidity sensor
#include <LiquidCrystal.h> //LCD
#include <Stepper.h>
#include <RTClib.h>

dht DHT; //temp and humidity sensor
RTC_DS1307 rtc; //RTC module

#define DHT11_PIN 52 //temp and humidity sensor
#define BUTTON_PIN 18 //start stop button

const int RS = 46, EN = 44, D4 = 40, D5 = 38, D6 = 36, D7 = 34; //LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7); //LCD

char daysOfTheWeek[7][12] = {
  "Sunday",
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday"
}; //RTC module

volatile bool buttonPressed = false; //button

unsigned long lastUpdateTime = 0; //LCD
const unsigned long updateInterval = 60000; //LCD

int speedPin = 22; //fan motor
int dir1 = 24; //fan motor
int dir2 = 26 //fan motor
int mSpeed = 90; //fan motor

const int stepsPerRevolution = 2038; //stepper motor
Stepper myStepper = Stepper(stepsPerRevolution, 2, 4, 3, 5); //stepper motor

volatile unsigned char *portB = (unsigned char *) 0x25; //RTC module
volatile unsigned int *portDDRB = (unsigned *) 0x24;
volatile unsigned char *pin_b= (unsigned char *) 0x23;

volatile unsigned char *portH = (unsigned char *) 0x102;
volatile unsigned int *portDDRH = (unsigned *) 0x0101;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


void setup() {
  Serial.begin(9600);

  //fan motor
  pinMode(speedPin, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  //button
  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), StartStopButtonISR, CHANGE);

  //RTC module
  adc_init();
  //set pb6 and pb7 to input
  *portDDRB= 0x3F;
  //set ph4 and ph3 to output
  *portDDRH= 0x18;
  //set ph4 and to high for water sensor
  *portH= 0x10;
  //enable pullup resistor on pb6 and pb7
  *portB= 0xC0;

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

}

void loop() {

  TempAndHumiditySensor();
  FanMotor();

  //water sensor
  unsigned int waterVal= adc_read(0);
  checkWaterLevel(waterVal);

  //stepper motor
  if(*pin_b & 0x80){
    Serial.println("Vent position change at:");
    displayCurrentTime();
    myStepper.setSpeed(10);
    myStepper.step(stepsPerRevolution);
  }
  if(*pin_b & 0x40){
    Serial.println("Vent position change at:");
    displayCurrentTime();
    myStepper.setSpeed(10);
    myStepper.step(-stepsPerRevolution);
  }

}

//RTC module
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

//RTC module
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
  
//Christine
void TempAndHumiditySensor(){

  int chk = DHT.read11(DHT11_PIN);
  unsigned long currentTime = millis();
  if(currentTime - lastUpdateTime >= updateInterval){
    //print to LCD display
    lcd.begin(16,2);
    lcd.setCursor(0,0);
    lcd.print("Temperature=");
    lcd.print(DHT.temperature);
    lcd.setCursor(0,1);
    lcd.print("Humidity = ");
    lcd.print(DHT.humidity);

    lastUpdateTime = currentTime;
  }
}

void checkWaterLevel(unsigned int input){
  if(input < 300){
    Serial.println("Water level is too low");
    //set ph3 high
    *portH |= 0x8;
    delay(1000);
  }
  else{
    //set ph3 low
    *portH &= 0xF7;
  }
}

//Christine
void FanMotor(){
  if(buttonPressed){
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(speedPin, mSpeed);
  }
  else if(!buttonPressed){
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(speedPin, 0);
  }
}

//RTC module
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

//Christine
void StartStopButtonISR() {
  int button = digitalRead(BUTTON_PIN);
  if (button == HIGH) {
    buttonPressed = true;
  }
  else{
    buttonPressed = false;
  }
}

 