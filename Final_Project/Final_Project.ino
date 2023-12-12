#include <dht.h> //temp and humidity sensor
#include <LiquidCrystal.h> //LCD

#define DHT11_PIN 13 //temp and humidity sensor
#define BUTTON_PIN 2 //start stop button

dht DHT; //temp and humidity sensor

const int RS = 11, EN = 12, D4 = 44, D5 = 46, D6 = 48, D7 = 50; //LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7); //LCD

int speedPin = 9; //fan motor
int dir1 = 8; //fan motor
int dir2 = 7; //fan motor
int mSpeed = 90; //fan motor

volatile bool buttonPressed = false; //button

unsigned long lastUpdateTime = 0; //LCD
const unsigned long updateInterval = 60000; //LCD


void setup() {
  Serial.begin(9600);

  //fan motor
  pinMode(speedPin, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  //button
  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), StartStopButtonISR, CHANGE);

}

void loop() {

  TempAndHumiditySensor();
  FanMotor();

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

void WaterSensor(){
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

void RTCModule(){
}

void StepperMotor(){
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

 