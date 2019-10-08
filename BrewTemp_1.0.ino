//# Copyright (c) 2019 Luis Pérez Manzanal.
//
//# BrewFerm 1.0.1
//
//# Author: Luis Pérez Manzanal
//
//# Description:    Program to control temperature of a fermentation chamber.
//#                 It uses Ubidots library and server to store data and control 
//#                 temperature variables.
//#                 It uses AutoPid library to control temperature tuning actuaction time as output.
//#                 It uses Dallas temperature sensor and therefore is  needed to use its library.
//
//#################################################################
//####################  C H A N G E     L O G  ####################
//#################################################################
//###
//###   - Version 1.0 (2019/09/17)    Description:  -- First version
//###   - Version 1.0.1 (2019/10/08)  Description:  -- Adjust gap between sensor
//###                                               -- Better minumum Freezer temp
//###                                               -- Full PID interval
//###
//###
//###################################################################################
//###################################################################################

/****************************************
 * Include Libraries
 ****************************************/
#include <AutoPID.h>
#include "UbidotsESPMQTT.h"
#include <OneWire.h>
#include <DallasTemperature.h>

/****************************************
 * Define Constants
 ****************************************/
#define TOKEN "YYYYYYYYYYYYYYYYYYYYYYYY" // Your Ubidots TOKEN
#define HTTPSERVER "things.ubidots.com"     // Ubidots Educational URL

#define WIFINAME "XXXXXXXXXXXXXXXXXX" //Your SSID
#define WIFIPASS "XXXXXXXXXXXXXXXXXX" // Your Wifi Pass

#define DEVICE_LABEL  "brewtemp1"  // Put here your Ubidots device label
#define FERMENTER_LABEL "tempferm" // Your variable label
#define FREEZER_LABEL "tempcong" // Your variable label
#define MODE_LABEL "mode" // Your variable label
#define TEMPSET_LABEL "tempset" // Your variable label
#define OFFSET_LABEL "offset" // Your variable label
#define OUTPUT_LABEL "outputtime" // Your variable label
#define INTERVAL_SECONDS 30 // Your variable label
#define NUM_INTERVAL 10 // Your variable label


#define ONE_WIRE_FERM 5  // Digital port D1
#define ONE_WIRE_CONG 4  // Digital port D2

#define RELAY_CALE 16  // Digital port D0
#define RELAY_CONG 12  // Digital port D6

#define RELAY_ON LOW  // Relays are activated in low
#define RELAY_OFF HIGH  // Relays are activated in low

//pid settings and gains
#define OUTPUT_MIN -INTERVAL_SECONDS
#define OUTPUT_MAX INTERVAL_SECONDS

#define KPHEATER 2      // PID parameters
#define KPFREEZER 4     // PID parameters
#define KIHEATER 0.7    // PID parameters
#define KIFREEZER 0.8   // PID parameters

#define TEMPOFFSET 0.1    // Default Temperature Offset
#define MIN_LIMIT 0.1     // Minimum Temperature
#define FREEZER_DIFF 3.2  // Maximum difference between temperature and freezer
#define MAX_LIMIT 30      // Maximum Temperature
#define SENSORGAP_FERMCONG 0.37 // Temperature gap between fermentor sensor and freezer
#define ISPINDEL_TEMPGAP 0.35 // Temperature gap between temperature sensor in relation with ISpindel device

double temp = 10;
double cong = 10;

boolean onTrackTemp = true;
boolean resetNeeded = false;
int resetDirection = 0;

double previousTimer = 0;
unsigned long previousMillis = 0;
unsigned long timeStamp = 0;
int intCount = 0;

double temperature = 0;
double freezer = 0;
double setPoint = 0;
double outputVal = 0;
double myTimeInterval = 0;
float modeSwitch = 0.00;
float preModeSwitch = 0.00;
float tempSet = 0.00;
float preTempSet = 0.00;
float offSet = 0.00;
float preOffSet = 0.00;

Ubidots client(TOKEN);
OneWire oneWire_ferm(ONE_WIRE_FERM);
OneWire oneWire_cong(ONE_WIRE_CONG);

DallasTemperature sensor_ferm(&oneWire_ferm);
DallasTemperature sensor_cong(&oneWire_cong);

//input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&temperature, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, 0,0,0);

/****************************************
 * Auxiliar Functions
 ****************************************/
// Callback functions to retrieve input settings from Control-Slide variables 
void callback(char* topic, byte* payload, unsigned int length) {
  char myValue[6];
  char *pstr;

  for (int i=0;i<length;i++) {
    myValue[i] = payload[i];
  }  

  pstr = strstr( topic,DEVICE_LABEL);
  if(pstr != NULL) {
    pstr = strstr( topic,MODE_LABEL);
    if(pstr != NULL) {
      modeSwitch = stringTofloat( myValue );
      setMode();
      if( preModeSwitch != modeSwitch ) {
        preModeSwitch = modeSwitch;
        resetNeeded = true;
        resetDirection = 0;
      }
      Serial.print(modeSwitch);
      Serial.print(" ");
      Serial.println(topic);  
    }
    pstr = strstr( topic,TEMPSET_LABEL);
    if(pstr != NULL) {
      tempSet = stringTofloat( myValue );
      if( preTempSet != tempSet ) {
        preTempSet = tempSet;
        resetNeeded = true;        
        resetDirection = 0;
      }
      Serial.print(tempSet);
      Serial.print(" ");
      Serial.println(topic);   
    }
    pstr = strstr( topic,OFFSET_LABEL);
    if(pstr != NULL) {
      offSet = stringTofloat( myValue );
      if( preOffSet != offSet ) {
        preOffSet = offSet;
        resetNeeded = true;        
        resetDirection = 0;
      }
      Serial.print(offSet);
      Serial.print(" ");
      Serial.println(topic);  
    }
  }
  setMyPID();
}
// ********************************************************
// Function to convert char* to float
float stringTofloat(char* myString) {
  char myChar[sizeof(myString) + 1];
  for (uint8_t j = 0; j < sizeof(myString); j++) {
      myChar[j] = myString[j];
  }
  return atof(myChar);
}

// ********************************************************
// Function to setup PID direction
void setMyPID() {
  if( resetNeeded ) {
    if( resetDirection == 1 ) {
      myPID.stop(); 
      myPID.setGains(KPHEATER,KIHEATER,KPHEATER);
      myPID.reset(); 
      myPID.run(); 
      Serial.print(" reset1 ");
    }
    else if( resetDirection == -1 ) {
      myPID.stop(); 
      myPID.setGains(KPFREEZER,KIFREEZER,KPFREEZER);    
      myPID.reset(); 
      myPID.run(); 
      Serial.print(" reset2 ");
    }
    else {
      myPID.reset(); 
      Serial.print(" reset3 ");
    }
    resetNeeded = false; 
  }
}
// ********************************************************

// Function to setup PID direction
void pidDirection() {
  if( temperature < setPoint ) {
    if( resetDirection != 1 ) {
      resetNeeded = true;
      resetDirection = 1; 
      Serial.print(" H ");
    }
  }
  else if( temperature > setPoint ) {
    if( resetDirection != -1 ) {
      resetNeeded = true;
      resetDirection = -1;
      Serial.print(" F ");
    }
  }
  else {
    resetNeeded = false; 
    resetDirection = 0;
    Serial.print(" N ");
  }
  setMyPID();    
}
// ********************************************************
void setStandBy() {
  
  if( modeSwitch == 0.00 ) {
    Serial.println(" Error - StandBy");
    myPID.stop();
    digitalWrite(RELAY_CALE, RELAY_OFF);            
    digitalWrite(RELAY_CONG, RELAY_OFF);            
  }
  else {
    resetNeeded = true; 
    resetDirection = 0;
  }        
}

// ********************************************************
void setMode() {
  
  if( modeSwitch == 0.00 ) {
    Serial.println(" Error - StandBy");
    myPID.stop();
    digitalWrite(RELAY_CALE, RELAY_OFF);            
    digitalWrite(RELAY_CONG, RELAY_OFF);            
  }
  else if( modeSwitch == 1.00 ) {
    Serial.println(" Only Heating ");
    digitalWrite(RELAY_CONG, RELAY_OFF);            
  }        
  else if( modeSwitch == 2.00 ) {
    Serial.println(" Only Cooling ");
    digitalWrite(RELAY_CALE, RELAY_OFF);            
  }        
  else {                          // Heating and Cooling
    Serial.println(" Both Heating and Cooling ");
    resetNeeded = true; 
    resetDirection = 0;
  }        
}

// ********************************************************
void mainRun() {

  double outputSent = 0;
  myPID.run(); //call every loop, updates automatically at certain time interval
  myTimeInterval = outputVal;
  Serial.print(" ");
  onTrackTemp = myPID.atSetPoint(offSet);
  Serial.print( onTrackTemp );

  timeStamp = millis();

  if( !onTrackTemp ) {
    if( myTimeInterval > 19*INTERVAL_SECONDS/20 ) {
      if( modeSwitch == 1.00 ) {
        previousTimer = INTERVAL_SECONDS*NUM_INTERVAL*1000;
        digitalWrite(RELAY_CALE, RELAY_ON);
        digitalWrite(RELAY_CONG, RELAY_OFF);
      }        
      else if( modeSwitch == 2.00 ) {
        previousTimer = 0;
        digitalWrite(RELAY_CALE, RELAY_OFF);
        digitalWrite(RELAY_CONG, RELAY_OFF);
      }        
      else {
        previousTimer = INTERVAL_SECONDS*NUM_INTERVAL*1000;
        digitalWrite(RELAY_CALE, RELAY_ON);
        digitalWrite(RELAY_CONG, RELAY_OFF);
      }        
      outputSent = previousTimer;
      Serial.print(" E1 ");
    }
    else if( (myTimeInterval < 19*INTERVAL_SECONDS/20) && (myTimeInterval > INTERVAL_SECONDS/20) ) {
      if( modeSwitch == 1.00 ) {
        previousTimer = myTimeInterval*NUM_INTERVAL*1000;
        digitalWrite(RELAY_CALE, RELAY_ON);
        digitalWrite(RELAY_CONG, RELAY_OFF);
      }        
      else if( modeSwitch == 2.00 ) {
        previousTimer = 0;
        digitalWrite(RELAY_CALE, RELAY_OFF);
        digitalWrite(RELAY_CONG, RELAY_OFF);
      }        
      else {
        previousTimer = myTimeInterval*NUM_INTERVAL*1000;
        digitalWrite(RELAY_CALE, RELAY_ON);
        digitalWrite(RELAY_CONG, RELAY_OFF);
      }        
      outputSent = previousTimer;
      Serial.print(" E2 ");
    }      
    else if( myTimeInterval < -19*INTERVAL_SECONDS/20) {
      if( modeSwitch == 1.00 ) {
        previousTimer = 0;
        digitalWrite(RELAY_CALE, RELAY_OFF);
        digitalWrite(RELAY_CONG, RELAY_OFF);
      }        
      else if( modeSwitch == 2.00 ) {
        previousTimer = INTERVAL_SECONDS*NUM_INTERVAL*1000;
        digitalWrite(RELAY_CALE, RELAY_OFF);
        digitalWrite(RELAY_CONG, RELAY_ON);
      }        
      else {
        previousTimer = INTERVAL_SECONDS*NUM_INTERVAL*1000;
        digitalWrite(RELAY_CALE, RELAY_OFF);
        digitalWrite(RELAY_CONG, RELAY_ON);
      }        
      outputSent = -previousTimer;
      Serial.print(" E3 ");
    }
    else if( (myTimeInterval < -INTERVAL_SECONDS/20) && (myTimeInterval > -19*INTERVAL_SECONDS/20) ) {
      if( modeSwitch == 1.00 ) {
        previousTimer = 0;
        digitalWrite(RELAY_CALE, RELAY_OFF);
        digitalWrite(RELAY_CONG, RELAY_OFF);
      }        
      else if( modeSwitch == 2.00 ) {
        previousTimer = -myTimeInterval*NUM_INTERVAL*1000;
        digitalWrite(RELAY_CALE, RELAY_OFF);
        digitalWrite(RELAY_CONG, RELAY_ON);
      }        
      else {
        previousTimer = -myTimeInterval*NUM_INTERVAL*1000;
        digitalWrite(RELAY_CALE, RELAY_OFF);
        digitalWrite(RELAY_CONG, RELAY_ON);
      }        
      outputSent = -previousTimer;
      Serial.print(" E4 ");
    }
    else {
      previousTimer = 0;
      digitalWrite(RELAY_CALE, RELAY_OFF);
      digitalWrite(RELAY_CONG, RELAY_OFF);      
      outputSent = previousTimer;
      Serial.print(" E5 ");      
    }
  }
  else {
    previousTimer = 0;
    digitalWrite(RELAY_CALE, RELAY_OFF);
    digitalWrite(RELAY_CONG, RELAY_OFF);      
    outputSent = previousTimer;
    Serial.print(" E6 ");      
  } 
  sendCaleCongState(outputSent/1000);
  Serial.print(" ");
  Serial.print( outputSent/1000 );

}
// ********************************************************

void getTemperature() {
  Serial.println("");
  sensor_ferm.requestTemperatures();
  sensor_cong.requestTemperatures();
  
  temp = sensor_ferm.getTempCByIndex(0) + ISPINDEL_TEMPGAP;
  cong = sensor_cong.getTempCByIndex(0) + ISPINDEL_TEMPGAP;
  temp = temp - (SENSORGAP_FERMCONG - temp/80)/2;
  cong = cong + (SENSORGAP_FERMCONG - cong/80)/2;  
  Serial.print(" ");
  Serial.print(temp);
  Serial.print(" ");
  Serial.print(cong);

  temperature = temp;
  freezer = cong;
  setPoint = tempSet;
}
// ********************************************************

void sendCaleCongState(double myOutput) {
  client.add(OUTPUT_LABEL, myOutput); //Insert your variable Labels and the value to be sent
  client.ubidotsPublish(DEVICE_LABEL);
}

// ********************************************************

void sendCaleTemp() {
  client.add(FERMENTER_LABEL, temperature); //Insert your variable Labels and the value to be sent
  client.ubidotsPublish(DEVICE_LABEL); 
}
// ********************************************************

void sendCongTemp() {
  client.add(FREEZER_LABEL, freezer); //Insert your variable Labels and the value to be sent
  client.ubidotsPublish(DEVICE_LABEL); 
}
// ********************************************************

/****************************************
 * Main Functions
 ****************************************/

void setup() {
// put your setup code here, to run once:
  client.ubidotsSetBroker(HTTPSERVER); // Sets the broker properly for the educational account
  client.setDebug(false); // Pass a true or false bool value to activate debug messages
  Serial.begin(115200);
  client.wifiConnection(WIFINAME, WIFIPASS);
  client.begin(callback);
  client.ubidotsSubscribe(DEVICE_LABEL, TEMPSET_LABEL); //Insert the dataSource and Variable's Labels
  client.ubidotsSubscribe(DEVICE_LABEL, MODE_LABEL); //Insert the dataSource and Variable's Labels
  client.ubidotsSubscribe(DEVICE_LABEL, OFFSET_LABEL); //Insert the dataSource and Variable's Labels

  // Init Temperature sensors
  sensor_ferm.begin();
  sensor_cong.begin();

  // Define Digital Outputs for Relays
  pinMode(RELAY_CALE, OUTPUT);
  digitalWrite(RELAY_CALE, RELAY_OFF);
  pinMode(RELAY_CONG, OUTPUT);
  digitalWrite(RELAY_CONG, RELAY_OFF);

  myPID.setBangBang(10*TEMPOFFSET);
  myPID.setTimeStep(1000*NUM_INTERVAL*INTERVAL_SECONDS);
}

void loop() {
// put your main code here, to run repeatedly:
unsigned long currentMillis = millis();
  
  if(!client.connected()){
    digitalWrite(RELAY_CONG, RELAY_OFF);            
    digitalWrite(RELAY_CALE, RELAY_OFF);            
    client.reconnect();
    client.ubidotsSubscribe(DEVICE_LABEL, TEMPSET_LABEL); //Insert the dataSource and Variable's Labels
    client.ubidotsSubscribe(DEVICE_LABEL, MODE_LABEL); //Insert the dataSource and Variable's Labels
    client.ubidotsSubscribe(DEVICE_LABEL, OFFSET_LABEL); //Insert the dataSource and Variable's Labels
  }
  if (currentMillis - previousMillis >= INTERVAL_SECONDS*1000 ) {
    previousMillis = currentMillis;
    getTemperature();
    pidDirection();
    if( intCount == 0 ) {
      sendCaleTemp();         
      sendCongTemp();         
    }
    if( modeSwitch != 0.00 ) {
      if( temperature < MIN_LIMIT ) {
        digitalWrite(RELAY_CONG, RELAY_OFF);            
        timeStamp = millis();
        if( intCount == 0 ) {
          sendCaleCongState(0);
        }
      }
      else if( temperature > setPoint && cong < setPoint - temperature - FREEZER_DIFF ) {
        digitalWrite(RELAY_CONG, RELAY_OFF);            
        timeStamp = millis();
        if( intCount == 0 ) {
          sendCaleCongState(0);
        }
      }
      else if( temperature > MAX_LIMIT ) {
        digitalWrite(RELAY_CALE, RELAY_OFF);            
        timeStamp = millis();
        if( intCount == 0 ) {
          sendCaleCongState(0);
        }
      } 
      else {
        if( intCount == 0 ) {
          mainRun();
        }
      }
    }
    else {
      setStandBy();        
      timeStamp = millis();
      if( intCount == 0 ) {
        sendCaleCongState(0);
      }
    }
    Serial.print(" ");
    Serial.print(intCount);
    intCount++;
    if( intCount > NUM_INTERVAL - 1 ) {
      intCount = 0;
    }
  }

  if (millis() - timeStamp >= previousTimer ) {
    previousTimer = INTERVAL_SECONDS*NUM_INTERVAL*1000;
    if( myTimeInterval > 0 ) {
      digitalWrite(RELAY_CALE, RELAY_OFF);            
    }
    else {
      digitalWrite(RELAY_CONG, RELAY_OFF);            
    }
  }
  client.loop();
}
