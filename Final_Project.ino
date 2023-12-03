#include <dht.h>

dht DHT; //temp and humidity sensor

#define DHT11_PIN 2 //temp and humidity sensor
#define POWER_PIN 3 //water sensor
#define SIGNAL_PIN A1 //water sensor

void setup() {
  Serial.begin(9600);

}

void loop() {

  TempAndHumiditySensor();

}

//change to print on LCD display
void TempHumiditySensor(){
  int chk = DHT.read11(DHT11_PIN);  //begin temp/humidity sensor
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  delay(1000);                      //end temp/humidity sensor
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

void LCDDisplay(){

}

void RTCModule(){

}

void StepperMotor(){

}
