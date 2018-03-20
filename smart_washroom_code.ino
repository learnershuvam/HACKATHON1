#define BLYNK_PRINT Serial
#include <aJSON.h>
#include <WiFi.h>
#include <Wire.h>
#include <BMA222.h>
#include "SPI.h"
#include "DHT.h"
#include "M2XStreamClient.h"
#include <PubSubClient.h>
#include <BlynkSimpleEnergiaWiFi.h>
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 11 
#define pin A2
int count=0;

#define ENERGIA_PLATFORM
const int trigPin=17;
const int echoPin=18;
DHT dht(DHTPIN, DHTTYPE);
int val;
int udistance;
long duration;
int co2=0;
int ledPin = 29;                // Connect LED on pin 13, or use the onboard one
int KEY = 4;                 // Connect Touch sensor on Digital Pin 2
const int buttonPir = 5;    
const int ledPir =  10;
int c=0;
int buzz=7;


char auth[] = "02b1c3a90de24cc3857fb710cee36831";
char ssid[] = "WLAN"; //  your network SSID (name)
char pass[] = "00000001";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
char deviceId[] = "744be907c2d8c14a9730a07c1dcb30cd"; // Device you want to push to
char streamName1[] ="humidity";
char streamName2[] ="temperature"; // Stream you want to push to
char streamName3[] ="light";
char streamName4[] ="pir";
char streamName5[] ="carbon";
char m2xKey[] = "f91fcad6d4c5c744033d81618fa49b8a"; // Your M2X access key
BMA222 mySensor;
WiFiClient client;
M2XStreamClient m2xClient(&client, m2xKey);
void flow();
void fuse();
void temphumid();
void distance();
void lightsensor();
void firedet();
void printWifiStatus();
float light;
int light_value;
void touchsensor();
void pulseCounter();
void pirsensor();
void co2sensor();
byte statusLed    = 29;
byte sensorInterrupt = 10;  // 0 = digital pin 2
byte sensorPin       = 10;
float calibrationFactor = 4.5;
volatile byte pulseCount;  
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;

int fire=8;
 

void setup() {

  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  dht.begin();
  mySensor.begin();
  uint8_t chipID = mySensor.chipID();
  Serial.print("chipID: ");
  Serial.println(chipID);

  delay(1000);
  
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to Network named: ");
  // print the network name (SSID);
  Serial.println(ssid); 
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  WiFi.begin(ssid, pass);
  while ( WiFi.status() != WL_CONNECTED) {
    // print dots while we wait to connect
    Serial.print(".");
    delay(300);
  }
  
  Serial.println("\nYou're connected to the network");
  Serial.println("Waiting for an ip address");
  
  while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for an ip addresss
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nIP Address obtained");
  
  // you're connected now, so print out the status  
  printWifiStatus();
  pinMode(fire,OUTPUT);
//  pinMode(val, OUTPUT);
pinMode(6, INPUT);
   pinMode(buzz,OUTPUT);
   //pinMode (6, INPUT);
   pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
   pinMode(ledPin, OUTPUT);      // Set ledPin to output mode
   pinMode(KEY, INPUT);       //Set touch sensor pin to input mode
   pinMode(ledPir, OUTPUT);      
   pinMode(buttonPir, INPUT);
   pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, HIGH);  // We have an active-low LED attached
    pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);
  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;
  
   attachInterrupt(sensorInterrupt,pulseCounter, FALLING);
   
}

void loop() {
Blynk.run();

sensor();
//delay(1000);
if(digitalRead(KEY)==HIGH) {      //Read Touch sensor signal
        Serial.println("----------");
           // if Touch sensor is HIGH, then turn on
        flow();
     }

      if(udistance<=10){
    Serial.println("Dustbin in Full");
    Blynk.email("shuvamkumarsk1@gmail.com","humans","Please clear the dustbin");
  }
  }

void sensor()
{
temphumid();
lightsensor();
distance();
touchsensor();
pirsensor();
co2sensor();
fuse();

}
void temphumid()
 {
  float h = dht.readHumidity();
float t = dht.readTemperature();
Blynk.virtualWrite(V0,t);
Blynk.virtualWrite(V1,h);

    
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  } else {
    int response1 = m2xClient.updateStreamValue(deviceId, streamName1, h);
    Serial.print("HUMIDITY");
    Serial.println(h);
    int response2 = m2xClient.updateStreamValue(deviceId, streamName2, t);
    Serial.print("TEMP");
    Serial.println(t);
    if(h>60)
    {
      Blynk.email("humidity","aukat se bahar ho gaya h humidity");
      Blynk.email("shuvamkumarsk1@gmail.com","humidity","aukat se bahar ho gaya h humidity");
    }
    Serial.print("M2x temp humid ");
    Serial.println(response1);
    Serial.println(response2);
    
    if (response1 == -1 && response2 == -1)
      while (1)
        ;

    delay(500);
 }
 }
 
void lightsensor()
{
     int lv = analogRead(6);
  light = lv * 0.0976;// percentage calculation 
  Serial.println(light);
  delay(100);
      Blynk.virtualWrite(V2,val);
      int response3 = m2xClient.updateStreamValue(deviceId, streamName3, val);
      
      Serial.print("M2x light ");
      Serial.println(response3);        
      delay(50);
      if (response3 == -1)
      while (1)
        ;

    delay(50);
}

void distance()
{
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration=pulseIn(echoPin,HIGH);
  udistance=duration*0.034/2;
  Blynk.virtualWrite(V4,udistance);
  Serial.println("Dustbin");
  Serial.println(udistance);
  if(udistance<=10){
    Serial.println("Dustbin in Full");
    Blynk.email("shuvamkumarsk1@gmail.com","humans","Please clear the dustbin");
  }
 
}

double get_gp2d12 (double valu) {
  if (valu < 10){ valu = 10;
  return ((67870.0 / (valu - 3.0)) - 40.0);
}
}
void touchsensor()
{
  Serial.println("TOUCH SENSOR");
  if(digitalRead(KEY)==HIGH) {      //Read Touch sensor signal
        Serial.println("1");
        count++;
        if(count==3){
          Serial.println("Washroom is not cleen Send sweeper");
           Blynk.email("shuvamkumarsk1@gmail.com","humans","Washroom is not clean send sweeper");
          flow();
        digitalWrite(ledPin, HIGH);
        }
        }
        
           // if Touch sensor is HIGH, then turn on
        
     
   else{
        Serial.println("0");
        digitalWrite(ledPin, LOW);    // if Touch sensor is LOW, then turn off the led
        
     }
}
void pirsensor()
{
  if (digitalRead(buttonPir) == HIGH) 
     {   
      c++;  
      digitalWrite(ledPir, HIGH);
      Serial.println("HUMANS DETECTED"); 
      Serial.println("NO OF PERSON-  ");
      Blynk.virtualWrite(V5,c);
      int response4 = m2xClient.updateStreamValue(deviceId, streamName4, ledPir);
      if(c>10)
      {
        Blynk.email("shuvamkumarsk1@gmail.com","humans","Exceeded the number of persons in washroom");
        digitalWrite(buzz,HIGH);
      }
      Serial.print("M2x person ");
    Serial.println(response4);
      Serial.print(c);
      Serial.println();
      delay(100);
      if (response4 == -1)
      while (1)
        ;

    delay(500);
     } 
     else {
       digitalWrite(ledPir, LOW); 
     }
}
void co2sensor()
{
  co2=analogRead(24);//Read Gas value from analog 0
  Serial.println("CO2 in Washroom  ");
  Serial.println(co2,DEC);//Print the value to serial port
  Blynk.virtualWrite(V3,co2);
  int response5 = m2xClient.updateStreamValue(deviceId, streamName5, co2);
  
      Serial.print("M2x co2 ");
    Serial.println(response5);
    if (response5 == -1)
      while (1)
        ;
  delay(1000);
}




 void fuse()
{
  if(buttonPir==HIGH && val==LOW)
  {
    Serial.println("bulb has fused");
  }
  else
  {
    Serial.println("Light is on");
  }
}
void flow(){
  if((millis() - oldTime) > 1000)    // Only process counters once per second
  { 
        detachInterrupt(sensorInterrupt);
        
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
        oldTime = millis();
            flowMilliLitres = (flowRate / 60) * 1000;
         totalMilliLitres += flowMilliLitres;
         unsigned int frac;
        Serial.print("Flow rate: ");
    Serial.print(int(flowRate));  // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t");       // Print tab space
    Serial.print("Output Liquid Quantity: ");        
    Serial.print(totalMilliLitres);
    Serial.println("mL"); 
    Serial.print("\t");       // Print tab space
  Serial.print(totalMilliLitres/1000);
  Serial.print("L");
      pulseCount = 0;
       attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  }
}

void pulseCounter()
{
  // Increment the pulse counter
  pulseCount++;
}
void firedet(){
  if(digitalRead(fire)==HIGH){
    Serial.println("Fire !Fire! Fire!!!");
    Blynk.email("shuvamkumarsk1@gmail.com","humans","Fire!!!!Warning!!!!!");
    digitalWrite(buzz,HIGH);
    
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

