#include <dht.h> //temp and humidity sensor
#include <LiquidCrystal.h> //LCD

dht DHT; //temp and humidity sensor
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5; //LCD pins
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7); //LCD

#define DHT11_PIN 2 //temp and humidity sensor
#define POWER_PIN 3 //water sensor
#define SIGNAL_PIN A1 //water sensor

void setup() {
  Serial.begin(9600);

}

void loop() {

  TempAndHumiditySensor();

}

void TempHumiditySensor(){
  int chk = DHT.read11(DHT11_PIN);

  /*for testing; do not include in final program
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  delay(1000);
  */

  //print to LCD display
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temperature = ");
  lcd.print(DHT.temperature);
  lcd.setCursor(1,0);
  lcd.print("Humidity = ");
  lcd.print(DHT.humidity);
  delay(60000); //update every 1 minute (60 seconds)
}

//must change code to use ADC method
  void WaterSensor(){
  int value = 0;

  //from example slides
  digitalWrite(POWER_PIN, HIGH);
  delay(10);
  value = analogRead(SIGNAL_PIN);
  digitalWrite(POWER_PIN, LOW);

  //for testing; do not include in final program
  Serial.print ("Sensor value: ");
  Serial.println(value);
  delay(1000);
}

void FanMotor(){

}

/*void LCDDisplay(){

}
*/

void RTCModule(){

}

void StepperMotor(){

}
