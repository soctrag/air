#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#include <DHT.h>
#define DHT11_PIN 7
#define DHTTYPE DHT11
#include "MQ135.h"
#include <SharpGP2Y10.h>
DHT dht(DHT11_PIN, DHTTYPE);
#define MQ2_PIN A1
#define MQ135_PIN A0
MQ135 mq135_sensor(MQ135_PIN);
int voPin = A2;
int ledPin = 4;
float dustDensity = 0;
SharpGP2Y10 dustSensor(voPin, ledPin);
void init_gsm();
void gprs_connect();
boolean gprs_disconnect();
boolean is_gprs_connected();
boolean waitResponse(String expected_answer="OK", unsigned int timeout=2000);
#include <SoftwareSerial.h>
#define rxPin 2
#define txPin 3
SoftwareSerial SIM800(rxPin,txPin);
const String APN  = "internet";
const String USER = "";//tên người dùng thẻ sim
const String PASS = "";
const String THING_SPEAK_API_URL  = "https://api.thingspeak.com/update";
const String THING_SPEAK_API_KEY  = "0TQQZ41QZJKGBUO1";
String request_url = "";
#define USE_SSL true
#define DELAY_MS 500
void setup() {
  Serial.begin(9600);
  dht.begin();
  lcd.begin(20, 4);
  lcd.backlight ();
  SIM800.begin(9600);
  Serial.println("Initializing SIM800...");
  SIM800.println("AT");
  waitResponse();
  delay(DELAY_MS);
 pinMode(MQ135_PIN, INPUT);
 pinMode(MQ2_PIN, INPUT);
  SIM800.println("AT+CPIN?");
  delay(DELAY_MS);
}
void loop() {
  float mq2_val = analogRead(MQ2_PIN);
    if (isnan(mq2_val)){
    Serial.println("Failed to read from MQ-2 sensor!");
    return;
  }
  mq2_val = mq2_val/1023*100;
  Serial.println("MQ-2 Data: " + String(mq2_val));
  delay(2000);
  float huumidity = dht.readHumidity();
  float temprature = dht.readTemperature();
  if (isnan(huumidity) or isnan(temprature)){
    Serial.println("Failed to read from DHT11 sensor!");
    return;
  }
  Serial.println("Humidity Data: " + String(huumidity));
  Serial.println("Temperature Data: " + String(temprature));
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM( temprature, huumidity)/100;
    if (isnan(correctedPPM)){
    Serial.println("Failed to read from MQ-135 sensor!");
    return;
  }
   Serial.println("AQI Data: " + String(correctedPPM));
  delay(2000); 
   dustDensity = dustSensor.getDustDensity();
    if (isnan(dustDensity)){
    Serial.println("Failed to read from PM sensor!");
    return;
  }
   Serial.println("PM Data: " + String(dustDensity));
  lcd.clear();
  lcd.setCursor(0, 0);lcd.print("CHAT LUONG KHONG KHI");
  lcd.setCursor(0, 1); lcd.print("AQI:");
  lcd.setCursor(4, 1); lcd.print(correctedPPM);
  lcd.setCursor(1, 2); lcd.print("ND:");
  lcd.setCursor(4, 2); lcd.print(temprature);
  lcd.setCursor(8, 2); lcd.print("*C");
  lcd.setCursor(12, 2); lcd.print("DA:");
  lcd.setCursor(15, 2); lcd.print(huumidity);
  lcd.setCursor(19, 2); lcd.print("%");
  lcd.setCursor(0, 3); lcd.print("GAS:");
  lcd.setCursor(4, 3); lcd.print(mq2_val);
  lcd.setCursor(7, 3); lcd.print("%");
  lcd.setCursor(12, 3); lcd.print("PM:");
  lcd.setCursor(15, 3); lcd.print(dustDensity);
  request_url = THING_SPEAK_API_URL;
  request_url += "?key=" + THING_SPEAK_API_KEY;
  request_url += "&field1=";
  request_url += temprature;
  request_url += "&field2=";
  request_url += huumidity;
  request_url += "&field3=";
  request_url += correctedPPM;
  request_url += "&field4=";
  request_url += mq2_val;
  request_url += "&field5=";
  request_url += dustDensity;

  if(!is_gprs_connected()){
    gprs_connect();
  }
  SIM800.println("AT+HTTPINIT");
  waitResponse();
  delay(DELAY_MS);
 if(USE_SSL == true){
    SIM800.println("AT+HTTPSSL=1");
    waitResponse();
    delay(DELAY_MS);
  }
  SIM800.println("AT+HTTPPARA=\"URL\","+request_url);
  waitResponse();
  delay(DELAY_MS);
  SIM800.println("AT+HTTPACTION=0");
  for (uint32_t start = millis(); millis() - start < 20000;){
    while(SIM800.available() > 0){
      String response = SIM800.readString();
      Serial.println(response);
      if(response.indexOf("+HTTPACTION:") > 0){
        goto OutFor;
      }
    }
  }
  OutFor:
  delay(DELAY_MS);
  SIM800.println("AT+HTTPREAD");
  waitResponse("OK");
  delay(DELAY_MS);
  SIM800.println("AT+HTTPTERM");
  waitResponse("OK",1000);
  delay(DELAY_MS);
  delay(5000);
}
void init_gsm()
{
  SIM800.println("AT");
  waitResponse();
  delay(DELAY_MS);
  SIM800.println("AT+CPIN?");
  waitResponse("+CPIN: READY");
  delay(DELAY_MS);
  SIM800.println("AT+CFUN=1");
  waitResponse();
  delay(DELAY_MS);
  SIM800.println("AT+CREG?");
  waitResponse("+CREG: 0,");
  delay(DELAY_MS);
}
void gprs_connect()
{
  SIM800.println("AT+CGATT?");
  waitResponse("OK",2000);
  delay(DELAY_MS);
  SIM800.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  waitResponse();
  delay(DELAY_MS);
  SIM800.println("AT+SAPBR=3,1,\"APN\","+APN);
  waitResponse();
  delay(DELAY_MS);
  if(USER != ""){
    SIM800.println("AT+SAPBR=3,1,\"USER\","+USER);
    waitResponse();
    delay(DELAY_MS);
  }
 if(PASS != ""){
    SIM800.println("AT+SAPBR=3,1,\"PASS\","+PASS);
    waitResponse();
    delay(DELAY_MS);
  }
  SIM800.println("AT+SAPBR=1,1");
  waitResponse("OK", 30000);
  delay(DELAY_MS);
  SIM800.println("AT+SAPBR=2,1");
  waitResponse("OK");
  delay(DELAY_MS);
 }
boolean is_gprs_connected()
{
  SIM800.println("AT+SAPBR=2,1");
  if(waitResponse("0.0.0.0") == 1) { return false; }
  return true;
}
boolean gprs_disconnect()
{
  SIM800.println("AT+CGATT=0");
  waitResponse("OK",60000);
  return true;
}
boolean waitResponse(String expected_answer, unsigned int timeout)
{
  uint8_t x=0, answer=0;
  String response;
  unsigned long previous;
  while( SIM800.available() > 0) SIM800.read();
  previous = millis();
  do{
   if(SIM800.available() != 0){
        char c = SIM800.read();
        response.concat(c);
        x++;
   if(response.indexOf(expected_answer) > 0){
            answer = 1;
        }
    }
  }while((answer == 0) && ((millis() - previous) < timeout));
  Serial.println(response);
  return answer;
}
