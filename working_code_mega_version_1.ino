#include "Wire.h"       
/****For Roll-over accident****/
#include "I2Cdev.h"     
#include "MPU6050.h"

//nodeMcu stuff
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
//Initialise Arduino to NodeMCU (5=Rx & 6=Tx)
SoftwareSerial nodemcu(25, 23);
float latitude,longitude;
int Victims;
String registration, AccidentType,currentDate,currentTime;



String vehicleReg;
String message,messageSend;
//gps module
//Connect with pin 18 and 19
#include <TinyGPS.h>
//long   lat,lon; // create variable for latitude and longitude object
float lat,lon;
TinyGPS gps; // create gps object

unsigned long age, date, time, chars;
int year;
byte month, day, hour, minute, second, hundredths;

String Date;
String Time;

#include <SoftwareSerial.h>
SoftwareSerial mySerial(11, 10); 

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
struct MyData {
  byte X;
  byte Y;
  byte Z;
};
MyData data;
/****end of roll over***/

/****For Impact detection***/
const int frontSwitchPin = 13; //the switch connect to pin 12
const int rearSwitchPin = 12;
const int leftSideSwitchPin = 9;
const int rightSideSwitchPin = 8;
int frontSwitchState = 0;         // variable for reading the pushbutton status
int rearSwitchState = 0;
int leftSideSwitchState = 0;
int rightSideSwitchState = 0;
String accidentType;
/****end of impact***/

/****Victim numbers****/
const int driverPin = 7; //the switch connect to pin 12
const int firstPassengerPin = 6; 
const int secondPassengerPin = 5;
const int thirdPassengerPin = 4;
int driverState = 0; 
int firstPassengerState = 0;
int secondPassengerState = 0;
int thirdPassengerState = 0;  
int victims;
/****End of Victim numbers***/
int ledPin = 27;


void setup()
{
  Serial.begin(9600);
  /****Roll-over****/
  Wire.begin();
  mpu.initialize();
  /***end of roll-over***/
vehicleReg = "KBU 333z";
  /****For Impact detection***/
  pinMode(frontSwitchPin, INPUT); //initialize thebuttonPin as input
  pinMode(rearSwitchPin, INPUT);
  pinMode(leftSideSwitchPin, INPUT);
  pinMode(rightSideSwitchPin, INPUT);
   /****end of Impact detection***/

   /****Victim numbers****/
  pinMode(driverPin, INPUT); //initialize thebuttonPin as input
  pinMode(firstPassengerPin, INPUT);
  pinMode(secondPassengerPin, INPUT);
  pinMode(thirdPassengerPin, INPUT);
   /****End of Victim Numbers***/
pinMode(ledPin, OUTPUT);
//gps module
Serial.begin(9600); // connect serial
Serial.println("The GPS Received Signal:");
Serial1.begin(9600); // connect gps sensor


     mySerial.begin(9600);
  mySerial.println("AT"); 
  updateSerial();
  mySerial.println("AT+CMGF=1");
  updateSerial();
 mySerial.println("AT+CMGS=\"+254798223888\"");
  updateSerial();
  mySerial.print("Automatic Accident Detection Enabled!"); //text content
  updateSerial();
  mySerial.write(26); 

  //node mcu stuff
  nodemcu.begin(9600);
  delay(1000);
  Serial.println("Program started");

}
void loop()
{

  
  /***GPS module ***/
 while(Serial1.available()){ // check for gps data
    if(gps.encode(Serial1.read()))// encode gps data
    { 
    gps.f_get_position(&lat,&lon); // get latitude and longitude
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

    Date = String(day) + "/" + String(month) + "/" + String(year);
Time = String(hour)+ ":" + String(minute) + ":" + String(second);
/*
    Serial.print("Position: ");
    
    //Latitude
    Serial.print("Latitude: ");
    Serial.print(lat,6);
    
    Serial.print(",");
    
    //Longitude
    Serial.print("Longitude: ");
    Serial.println(lon,6); 
  */  
   }
  }

 /***End of gps module***/
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  data.X = map(ax, -17000, 17000, 0, 255 ); // X axis data
  data.Y = map(ay, -17000, 17000, 0, 255); 
  data.Z = map(az, -17000, 17000, 0, 255);  // Y axis data
  delay(500);
  
  /****For Impact detection***/
  //read the state of the switch value
  frontSwitchState = digitalRead(frontSwitchPin);
  rearSwitchState = digitalRead(rearSwitchPin);
  leftSideSwitchState = digitalRead(leftSideSwitchPin);
  rightSideSwitchState = digitalRead(rightSideSwitchPin);
  /****end of Impact detection***/

  /***Victim numbers***/
  driverState = digitalRead(driverPin);
  firstPassengerState = digitalRead(firstPassengerPin);
  secondPassengerState = digitalRead(secondPassengerPin);
  thirdPassengerState = digitalRead(thirdPassengerPin);
  /***End of victims***/

  /****Impact detection statements****/
   if (frontSwitchState == HIGH) //if it is,the state is HIGH
  {   
   accidentType = "Front collision";
   cutPower();
   getVictims();
   displayAccident();
   
   sendSms();
   nodeMcu();
  }
  else if(rearSwitchState == HIGH)
  {
    accidentType = "Rear collision";
    cutPower();
    getVictims();
    displayAccident();
   sendSms();
   nodeMcu();
  }
   else if(leftSideSwitchState == HIGH)
  {
     accidentType = "Left side collision";
     cutPower();
     getVictims();
     displayAccident();
   sendSms();
   nodeMcu();
  }
   else if(rightSideSwitchState == HIGH)
  {
   accidentType = "Right side collision";
   cutPower();
   getVictims();
   displayAccident();
   sendSms();
   nodeMcu();
  }
  /*******end of impact detection****/
  
  /*****Roll-over detction statements****/
 else if (data.X < 5 && data.X >= 0){
    accidentType = "Roll-Over";
    cutPower();
    getVictims();
    displayAccident();
    sendSms(); 
    nodeMcu();  
    }
    else if (data.X > 245){
      accidentType = "Roll-Over";
      cutPower();
      getVictims();
      displayAccident();
     sendSms();
     nodeMcu();
      }
    else if (data.X < 15 && data.X > 5){
      accidentType = "Roll-Over";
      cutPower();
      getVictims();
      displayAccident();
     sendSms(); 
     nodeMcu();      
      }
       else if (data.X >= 144 && data.X < 152){
        accidentType = "Roll-Over";
        cutPower();
        getVictims();
        displayAccident();
        sendSms();
        nodeMcu();       
      }
    else if (data.Y < 5 && data.Y >= 0){
      accidentType = "Roll-Over";
      cutPower();
      getVictims();
      displayAccident();
      sendSms();
      nodeMcu();
      }
    else if (data.Y >= 240 && data.Y <= 250){
      accidentType = "Roll-Over";
      cutPower();
      getVictims();
      displayAccident();
      sendSms();
      nodeMcu();
      }
      else if (data.Y >= 90 && data.Y < 110){
        accidentType = "Roll-Over";
        cutPower();
        getVictims();
        displayAccident();
        sendSms();
        nodeMcu();
      } 
    else if (data.Z >= 145 && data.Z <= 165){
      accidentType = "Roll-Over";
      cutPower();
      getVictims();
      displayAccident();
      sendSms();
      nodeMcu();
      }
   /****End of accident detection****/
     else {
      Serial.println("No accident");
      digitalWrite(ledPin, HIGH);
      }
 
message = "Accident at: " + String(lat,6) + ","+ String(lon,6) + "\n Reg Number: "+ String(vehicleReg)+"\n Accident Type:" + String(accidentType) + "\n No of Victims: " + String(victims) + "\n Date: "+ Date + "\n Time: " + Time; 
  latitude = lat,6;
  longitude = lon,6;
  registration = vehicleReg;
  AccidentType = accidentType;
  Victims = victims;
  currentDate = Date;
  currentTime = Time;
  messageSend = "Accident at: " + String(latitude) + ","+ String(longitude) + "\n Reg Number: "+ registration+"\n Accident Type:" + AccidentType + "\n No of Victims: " + Victims + "\n Date: "+ currentDate + "\n Time: " + currentTime;
}

void getVictims()
{
  if (driverState == HIGH &&thirdPassengerState == LOW && secondPassengerState == LOW && firstPassengerState == LOW  ) //if it is,the state is HIGH
  {  
    victims = 1;
  } 
else if (firstPassengerState == HIGH && driverState == HIGH &&thirdPassengerState == LOW && secondPassengerState == LOW)
{
      victims = 2; 
  }
  else if (secondPassengerState == HIGH && firstPassengerState == HIGH && driverState == HIGH && thirdPassengerState == LOW)
  {
        victims = 3; 
    }
  else if (thirdPassengerState == HIGH && secondPassengerState == HIGH && firstPassengerState == HIGH && driverState == HIGH) 
  {
        victims = 4; 
    }
    else if (secondPassengerState == HIGH && driverState == HIGH &&thirdPassengerState == LOW && firstPassengerState == LOW )
    {
      victims = 2; 
    }
   else if (thirdPassengerState == HIGH && driverState == HIGH && secondPassengerState == LOW && firstPassengerState == LOW)
    {
      victims = 2; 
     }
   else if (thirdPassengerState == HIGH && driverState == HIGH && secondPassengerState == LOW && firstPassengerState == HIGH)
    {
      victims = 3; 
     }
     else if (thirdPassengerState == HIGH && driverState == HIGH && secondPassengerState == HIGH && firstPassengerState == LOW)
    {
      victims = 3; 
     }
  else{
        victims = 0;
    }
  
}

void cutPower()
{
digitalWrite(ledPin, LOW);

}

void nodeMcu(){

  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();


  //Serial.println(sensorValue);
  latitude = lat,6;
  longitude = lon,6;
  registration = vehicleReg;
  AccidentType = accidentType;
  Victims = victims;
  currentDate = Date;
  currentTime = Time;
  //Assign collected data to JSON Object

  
  data["latitude"] = latitude;
  data["longitude"] = longitude;
  data["Registration"] = registration;
  data["accidentType"] = AccidentType;
  data["Victims"] = Victims;
  data["Date"] = currentDate;
  data["Time"] = currentTime;
 
  //Send data to NodeMCU
  data.printTo(nodemcu);
  jsonBuffer.clear();

  delay(2000);
  
  }

////to be removed in final code
void displayAccident()
{
 Serial.print(messageSend);
 delay(1000);
 /*
 Serial.print("Registration: ");
Serial.println(vehicleReg);
  Serial.print("Accident Type: ");
Serial.println(accidentType);
Serial.print("Number of victims: ");
Serial.println(victims); 

    Serial.print("Location: ");    
    //Latitude
    Serial.print("Latitude: ");
    Serial.print(lat,6);    
    Serial.print(",");    
    //Longitude
    Serial.print("Longitude: ");
    Serial.println(lon,6); 
   delay(2000); 
   */ 
  }
void sendSms(){
     mySerial.begin(9600);
  mySerial.println("AT"); 
  updateSerial();
  mySerial.println("AT+CMGF=1");
  updateSerial();
 mySerial.println("AT+CMGS=\"+254798223888\"");
  updateSerial();
  mySerial.print(messageSend); //text content
  updateSerial();
  mySerial.write(26);
 // sendSmsPolice();
  }
  void sendSmsPolice()
  {
         mySerial.begin(9600);
  mySerial.println("AT"); 
  updateSerial();
  mySerial.println("AT+CMGF=1");
  updateSerial();
 mySerial.println("AT+CMGS=\"+254702864432\"");
  updateSerial();
  mySerial.print(messageSend); //text content
  updateSerial();
  mySerial.write(26);
 // sendSmsFireDep();
    }
    void sendSmsFireDep()
    {
         mySerial.begin(9600);
  mySerial.println("AT"); 
  updateSerial();
  mySerial.println("AT+CMGF=1");
  updateSerial();
 mySerial.println("AT+CMGS=\"+254702864432\"");
  updateSerial();
  mySerial.print(messageSend); //text content
  updateSerial();
  mySerial.write(26);
  //makeCallHospital();
      }
  void makeCallHospital()
  {
   mySerial.println("AT"); //Once the handshake test is successful, i t will back to OK
  updateSerial();
  
 mySerial.println("ATD+ +25479822388;"); //  change ZZ with country code and xxxxxxxxxxx with phone number to dial
  updateSerial();
  delay(10000); // wait for 20 seconds...
  mySerial.println("ATH"); //hang up
  updateSerial();
  //makeCallPolice();
    }  
      void makeCallPolice()
  {
   mySerial.println("AT"); //Once the handshake test is successful, i t will back to OK
  updateSerial();
  
 mySerial.println("ATD+ +254702864432;"); //  change ZZ with country code and xxxxxxxxxxx with phone number to dial
  updateSerial();
  delay(10000); // wait for 20 seconds...
  mySerial.println("ATH"); //hang up
  updateSerial();
  //makeCallFire();
    } 
          void makeCallFire()
  {
   mySerial.println("AT"); //Once the handshake test is successful, i t will back to OK
  updateSerial();
  
 mySerial.println("ATD+ +254798223888;"); //  change ZZ with country code and xxxxxxxxxxx with phone number to dial
  updateSerial();
  delay(10000); // wait for 20 seconds...
  mySerial.println("ATH"); //hang up
  updateSerial();
    } 

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}
/****GPS module****/
