//# Copyright (c) 2019 Luis Pérez Manzanal.
//
//# BrewFerm 1.1.2
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
//###   - Version 1.0.2 (2019/10/09)  Description:  -- Improve initialization mechanism
//###                                               -- Concentration values sent in loop
//###                                               -- Concentration relay activity in loop
//###   - Version 1.0.3 (2019/10/09)  Description:  -- Repair minumum Freezer temp
//###   - Version 1.1.0 (2019/10/30)  Description:  -- Create persistent configuration data
//###                                               -- DoubleResetClic
//###                                               -- Configuration WIFI AP
//###                                               -- Initial Configuration web form
//###   - Version 1.1.1 (2019/11/06)  Description:  -- Green led blinks in AP Configuration mode
//###   - Version 1.1.2 (2019/11/07)  Description:  -- Callback mistakes and strings
//###                                               -- Improve response to temp and setting changes
//###
//###################################################################################
//###################################################################################

/****************************************
 * Include Libraries
 ****************************************/
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <DoubleResetDetector.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include "Globals.h"

#include <AutoPID.h>
#include "UbidotsESPMQTT.h"
#include <OneWire.h>
#include <DallasTemperature.h>


#define VERSION "1.1.2" // BrewTemp version


ESP8266WebServer server(HTTPPORT);
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

String my_Wifi;
String my_WPass;
String my_Token;
String my_Name;
String my_Interval;
String my_TempSet;
String my_TempOffset;
String my_SensorOffset;
String my_IspindelOffset;
String my_Mode;
String my_KpHeater;
String my_KpFreezer;
String my_KiHeater;
String my_KiFreezer;

const char formHead[] = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" charset=\"utf-8\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/><title>BrewTemp Configuration</title></head>";
const char formStyle[] = "<style>.container{width: 90%;clear: both;text-align:top;padding: 12px 20px;margin: 8px 0;display: inline-block;border: 3px solid #ccc;border-radius: 4px;box-sizing: border-box;}body{background: #f1f1f1 no-repeat right top;font: 15px verdana;}.container input{width: 100%;clear: both;}.btn{background: #2b5;border-radius: 4px;border: 0;color: #fff;cursor: pointer;display: inline-block;margin: 2px 0;padding:10px 14px 11px;width: 100%}.btn:hover{background: #04c}.btn:active,.btn:focus{background: #08b}select{outline: 0;border: 1px solid #555;margin: 4px 0;padding: 2px;width: 50%}</style>";

Ubidots* pUbiclient;
OneWire oneWire_ferm(ONE_WIRE_FERM);
OneWire oneWire_cong(ONE_WIRE_CONG);

DallasTemperature sensor_ferm(&oneWire_ferm);
DallasTemperature sensor_cong(&oneWire_cong);

AutoPID* pAutoPID;

boolean onTrackTemp = true;
boolean resetNeeded = false;
int resetDirection = 0;
double previousTimer = 0;
unsigned long previousMillis = 0;
unsigned long timeStamp = 0;
int intCount = -1;
double maxOutput = 0;
float modeSwitch;
float tempSet;
float offSet;
double temperature = 0;
double freezer = 0;
double setPoint = 0;
double outputVal = 0;
double myTimeInterval = 0;
double temp;
double cong;
double sensorOffset;
double iSpindelOffset;
char* deviceName;
double actTime = 0;
int state = 0;

/****************************************
 * Configuration web server Functions
 ****************************************/
void handleRoot() {
  String my_Main = formHead;
  my_Main += formStyle;
//  my_Main += "<body><div class=\"container\"><h2>BrewTemp Setup</h2><form method=\"get\" action=\"showForm\"><button class=\"btn\" type=\"submit\" formaction=\"factoryDefaults\">RESET TO FACTORY DEFAULTS</button><button class=\"btn\" type=\"submit\">CONFIGURATION</button></form></div></body></html>";
  my_Main += "<body><div class=\"container\"><h2>BrewTemp Setup</h2><form method=\"get\" action=\"showForm\"><fieldset><legend><b>RESET TO FACTORY DEFAULTS</b></legend><p>This implies to remove internal configuration file, format internal Flash Memory and create an empty configuration file.</p><p>This action will take about 10 seconds to be completed. BrewTemp is restarted. </p><button class=\"btn\" type=\"submit\" formaction=\"factoryDefaults\">RESET TO FACTORY DEFAULTS</button></fieldset><fieldset><legend><b>BrewTemp Configuration</b></legend><p>BrewTemp gets several configuration parameters than can not be handled directed by Ubidots IoT platform, such as network setup (SSID and passphrase), Ubitdots Key-Token and device name. Other parameters as PID setting could be handled by Ubidots but are omitted due to overhead to manage too many control parameters. Only the most important temperature parameters can be modified at configuration mode as well as at runtime from Ubidots, these parameters are temperature, offset and operation mode.</p><p>This action will take about 10 seconds to be completed. BrewTemp is restarted. BrewTemp will start in operational mode.</p><button class=\"btn\" type=\"submit\">CONFIGURATION</button></fieldset></form></div></body></html>";
  server.send(200, "text/html", my_Main);

}
// ********************************************************

void handleNotFound() {
//  digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
}
// ********************************************************

void showForm()
{ 
Serial.println("showForm");
  
  if(getConfig() != FILENOTFOUND ) {
    String my_Form = formHead;
    my_Form += formStyle;
    my_Form += "<body><div class=\"container\"><h2>BrewTemp Configuration</h2><br/><form method=\"get\" action=\"settings\"><fieldset><legend><b>Network Settings</b></legend><label>Wireless Network (SSID)</label><input id=\"my_Wifi\" name=\"my_Wifi\" length=32 placeholder=\"SSID\" value=\"";
    my_Form += my_Wifi;
  
    my_Form += "\" required><label>Password</label><input id=\"my_WPass\" name=\"my_WPass\" length=64 placeholder=\"password\" value=\"";
    my_Form += my_WPass;
 
    my_Form += "\" required></fieldset><fieldset><legend><b>Ubidots Settings</b></legend><label>Ubidots Token</label><input id=\"my_Token\" name=\"my_Token\" length=64 placeholder=\"Token\" value=\"";
    my_Form += my_Token;
  
    my_Form += "\" required><label for=\"my_Name\">BrewTemp Device Name</label><input id=\"my_Name\" name=\"my_Name\" maxlength=10 placeholder=\"BrewTemp Name\" value=\"";
    my_Form += my_Name;
  
    my_Form += "\" required><label for=\"my_Interval\">Measurement Interval (secs)</label><input id=\"my_Interval\" name=\"my_Interval\" length=3 placeholder=\"Measurement Interval (secs)\" value=\"";
    my_Form += my_Interval;
  
    my_Form += "\" type=\"number\" min=\"100\" max=\"600\" step=\"1\" required></fieldset><fieldset><legend><b>Temperature Settings</b></legend><label for=\"my_TempSet\">Temperature</label><input id=\"my_TempSet\" name=\"my_TempSet\" maxlength=4 placeholder=\"Temperature\" value=\"";
    my_Form += my_TempSet;
  
    my_Form += "\" type=\"number\" min=\"0\" max=\"30\" step=\"0.1\" required><label for=\"my_TempOffset\">Temperature Offset</label><input id=\"my_TempOffset\" name=\"my_TempOffset\" maxlength=4 placeholder=\"Offset\" value=\"";
    my_Form += my_TempOffset;
  
    my_Form += "\" type=\"number\" min=\"0\" max=\"1\" step=\"0.05\" required><label for=\"my_SensorOffset\">Sensor Gap (Fermentor - Freezer)</label><input id=\"my_SensorOffset\" name=\"my_SensorOffset\" maxlength=4 placeholder=\"SensorOffset\" value=\"";
    my_Form += my_SensorOffset;
  
    my_Form += "\" type=\"number\" min=\"-1\" max=\"1\" step=\"0.01\" required><label for=\"my_IspindelOffset\">ISpindel Gap (ISpindel - Fermentor)</label><input id=\"my_IspindelOffset\" name=\"my_IspindelOffset\" maxlength=4 placeholder=\"ISpindelOffset\" value=\"";
    my_Form += my_IspindelOffset;
  
    my_Form += "\" type=\"number\" min=\"-1\" max=\"1\" step=\"0.01\" required><label for=\"my_Mode\">Operation Mode </label><select name=\"my_Mode\">";
  
    switch(atoi(my_Mode.c_str())) {
      case 0:
        my_Form += "<option value=0 selected>(0) Standby</option><option value=1>(1) Only Heater</option><option value=2>(2) Only Freezer</option><option value=3>(3) Both Heater and Freezer</option>";
        break;
      case 1:
        my_Form += "<option value=0>(0) Standby</option><option value=1 selected>(1) Only Heater</option><option value=2>(2) Only Freezer</option><option value=3>(3) Both Heater and Freezer</option>";
        break;
      case 2:
        my_Form += "<option value=0>(0) Standby</option><option value=1>(1) Only Heater</option><option value=2 selected>(2) Only Freezer</option><option value=3>(3) Both Heater and Freezer</option>";
        break;
      case 3:
        my_Form += "<option value=0 selected>(0) Standby</option><option value=1>(1) Only Heater</option><option value=2>(2) Only Freezer</option><option value=3 selected>(3) Both Heater and Freezer</option>";
        break;
      default:
        my_Form += "<option value=0>(0) Standby</option><option value=1>(1) Only Heater</option><option value=2>(2) Only Freezer</option><option value=3>(3) Both Heater and Freezer</option>";
    }

    my_Form += "</select></fieldset><fieldset><legend><b>PID Settings</b></legend><label for=\"my_KpHeater\">KpHeater</label><input id=\"my_KpHeater\" name=\"my_KpHeater\" maxlength=4 placeholder=\"Offset\" value=\"";  
    my_Form += my_KpHeater;
  
    my_Form += "\" type=\"number\" min=\"0\" max=\"4\" step=\"0.05\" required><label for=\"my_KpFreezer\">KpFreezer</label><input id=\"my_KpFreezer\" name=\"my_KpFreezer\" maxlength=4 placeholder=\"Offset\" value=\"";
    my_Form += my_KpFreezer;

    my_Form += "\" type=\"number\" min=\"0\" max=\"4\" step=\"0.05\" required><label for=\"my_KiHeater\">KiHeater</label><input id=\"my_KiHeater\" name=\"my_KiHeater\" maxlength=4 placeholder=\"Offset\" value=\"";
    my_Form += my_KiHeater;
  
    my_Form += "\" type=\"number\" min=\"0\" max=\"4\" step=\"0.05\" required><label for=\"my_KiFreezer\">KiFreezer</label><input id=\"my_KiFreezer\" name=\"my_KiFreezer\" maxlength=4 placeholder=\"Offset\" value=\"";
    my_Form += my_KiFreezer;

    my_Form += "\" type=\"number\" min=\"0\" max=\"4\" step=\"0.05\" required></fieldset><button class=\"btn\" type=\"submit\">SAVE</button></form></div></body></html>";

    server.send(200, "text/html", my_Form);
  }
  else
    handleNotFound();
}
// ********************************************************

/****************************************
 * Configuration data/file Functions
 ****************************************/

int getConfig() {
Serial.println("getConfig");

  Serial.println("Mounting FS...");

  if (SPIFFS.begin())
  {
    Serial.println(" Mounted!");
    if (SPIFFS.exists(CONFIGFILE))
    {
      // file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = SPIFFS.open(CONFIGFILE, "r");
      if (configFile)
      {
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());

        if (json.success())
        {
          if (json.containsKey("Wifi"))
            my_Wifi = (const char *)json["Wifi"];
          if (json.containsKey("Pass"))
            my_WPass = (const char *)json["Pass"];
          if (json.containsKey("Token"))
            my_Token = (const char *)json["Token"];
          if (json.containsKey("Name"))
            my_Name = (const char *)json["Name"];
          if (json.containsKey("Interval"))
            my_Interval = (const char *)json["Interval"];
          if (json.containsKey("TempSet"))
            my_TempSet = (const char *)json["TempSet"];
          if (json.containsKey("TempOffset"))
            my_TempOffset = (const char *)json["TempOffset"];
          if (json.containsKey("SensorOffset"))
            my_SensorOffset = (const char *)json["SensorOffset"];
          if (json.containsKey("IspindelOffset"))
            my_IspindelOffset = (const char *)json["IspindelOffset"];
          if (json.containsKey("Mode"))
            my_Mode = (const char *)json["Mode"];
          if (json.containsKey("KpHeater"))
            my_KpHeater = (const char *)json["KpHeater"];
          if (json.containsKey("KpFreezer"))
            my_KpFreezer = (const char *)json["KpFreezer"];
          if (json.containsKey("KiHeater"))
            my_KiHeater = (const char *)json["KiHeater"];
          if (json.containsKey("KiFreezer"))
            my_KiFreezer = (const char *)json["KiFreezer"];

          Serial.println("Parsed config:");
          if( my_Wifi == "\0" ) {
            Serial.println("WARNING: Configuration file is empty");
            return FILEEMPTY;
          }  
          else {
            json.printTo(Serial);
            Serial.println();
            return FILEOK;
          }
        }
        else
        {
          Serial.println("ERROR: Failed to load json config");
          return FILENOTFOUND;
        }
      }
      Serial.println("ERROR: Unable to open config file");
      return FILENOTFOUND;
    }
    else 
    {
      Serial.println("ERROR: Config file does not exist");
      return FILENOTFOUND;
    }
  }
  else
  {
    Serial.println(" ERROR: Failed to mount FS!");
    return FSNOTMOUNT;
  }
}
// ********************************************************


void formatSpiffs()
{
  Serial.println("\nNeed to format SPIFFS: ");
  SPIFFS.end();
  SPIFFS.begin();
  Serial.println(SPIFFS.format());
}
// ********************************************************

bool saveConfig()
{
  Serial.println("Saving config...");

  // if SPIFFS is not usable
  if (!SPIFFS.begin() || !SPIFFS.exists(CONFIGFILE) ||
      !SPIFFS.open(CONFIGFILE, "w"))
    formatSpiffs();

  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();

  json["Wifi"] = my_Wifi;
  json["Pass"] = my_WPass;
  json["Token"] = my_Token;
  json["Name"] = my_Name;
  json["Interval"] = my_Interval;
  json["TempSet"] = my_TempSet;
  json["TempOffset"] = my_TempOffset;
  json["SensorOffset"] = my_SensorOffset;
  json["IspindelOffset"] = my_IspindelOffset;
  json["Mode"] = my_Mode;
  json["KpHeater"] = my_KpHeater;
  json["KpFreezer"] = my_KpFreezer;
  json["KiHeater"] = my_KiHeater;
  json["KiFreezer"] = my_KiFreezer;

  File configFile = SPIFFS.open(CONFIGFILE, "w+");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
    SPIFFS.end();
    return false;
  }
  else
  {
    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    SPIFFS.end();
    Serial.println("saved successfully");
    return true;
  }
}
// ********************************************************

void settings() {
Serial.println("Settings");

  my_Wifi = server.arg("my_Wifi").c_str();
  my_WPass = server.arg("my_WPass").c_str();
  my_Token = server.arg("my_Token").c_str();
  my_Name = server.arg("my_Name").c_str();
  my_Interval = server.arg("my_Interval").c_str();  
  my_Interval = server.arg("my_Interval").toInt();  
  my_TempSet = server.arg("my_TempSet").toFloat();
  my_TempOffset = server.arg("my_TempOffset").toFloat();
  my_SensorOffset = server.arg("my_SensorOffset").toFloat();
  my_IspindelOffset = server.arg("my_IspindelOffset").toFloat();
  my_Mode = server.arg("my_Mode").toInt();
  my_KpHeater = server.arg("my_KpHeater").toFloat();
  my_KpFreezer = server.arg("my_KpFreezer").toFloat();
  my_KiHeater = server.arg("my_KiHeater").toFloat();
  my_KiFreezer = server.arg("my_KiFreezer").toFloat();
  
  if(saveConfig()) {
//    Serial.println(" System is going to be restarted in 10 secs .....");
//    delay(10000);
    ESP.restart();
  }

}
// ********************************************************

void factoryDefaults() {
Serial.println("Factory Defaults");
  my_Wifi.remove(0);
  my_WPass.remove(0);
  my_Token.remove(0);
  my_Name.remove(0);
  my_Interval.remove(0);
  my_TempSet.remove(0);
  my_TempOffset.remove(0);
  my_SensorOffset.remove(0);
  my_IspindelOffset.remove(0);
  my_Mode.remove(0);
  my_KpHeater.remove(0);
  my_KpFreezer.remove(0);
  my_KiHeater.remove(0);
  my_KiFreezer.remove(0);

  Serial.println("Mounting FS...");

  if (SPIFFS.begin())
  {
    Serial.println(" Mounted!");
    if (SPIFFS.exists(CONFIGFILE)) {
      SPIFFS.remove(CONFIGFILE);
      if (SPIFFS.exists(CONFIGFILE))
        Serial.println(" Configuration file is not deleted");
      else
        Serial.println(" Configuration file is deleted");       
    }
    else {             
      Serial.println("ERROR: Config file does not exist");
    }
  }
  else
  {
    Serial.println(" ERROR: Failed to mount FS!");
  }
  if(saveConfig()) {
//    Serial.println(" System is going to be restarted in 10 secs .....");
//    delay(10000);
    ESP.restart();
  }
}
// ********************************************************


/****************************************
 * Auxiliar Functions
 ****************************************/
// Callback functions to retrieve input settings from Control-Slide variables 
void callback(char* topic, byte* payload, unsigned int length) {
  char myValue[6];
  char *pstr;
  float preModeSwitch = stringTofloat((char*)my_Mode.c_str());
  float preTempSet = stringTofloat((char*)my_TempSet.c_str());
  float preOffSet = stringTofloat((char*)my_TempOffset.c_str());

  for (int i=0;i<length;i++) {
    myValue[i] = payload[i];
  }  
      Serial.println(topic);  

  pstr = strstr( topic,(char*)my_Name.c_str());
  if(pstr != NULL) {
    pstr = strstr( topic, MODE_LABEL);
    if(pstr != NULL) {
      modeSwitch = (int)stringTofloat( myValue );
      if( preModeSwitch != modeSwitch ) {
        my_Mode.remove(0);
        my_Mode = modeSwitch;
        setMode(modeSwitch);
        resetNeeded = true;
        resetDirection = 0;
      }
      Serial.print(" ");
      Serial.print(modeSwitch);
      Serial.print(" ");
      Serial.println(topic);  
    }
    pstr = strstr( topic,TEMPSET_LABEL);
    if(pstr != NULL) {
      tempSet = stringTofloat( myValue );
      if( preTempSet != tempSet ) {
        my_TempSet.remove(0);
        my_TempSet = tempSet;
        resetNeeded = true;        
        resetDirection = 0;
      }
      Serial.print(" ");
      Serial.print(tempSet);
      Serial.print(" ");
      Serial.println(topic);   
    }
    pstr = strstr( topic,OFFSET_LABEL);
    if(pstr != NULL) {
      offSet = stringTofloat( myValue );
      if( preOffSet != offSet ) {
        my_TempOffset.remove(0);
        my_TempOffset = offSet;
        resetNeeded = true;        
        resetDirection = 0;
      }
      Serial.print(" ");
      Serial.print(offSet);
      Serial.print(" ");
      Serial.println(topic);  
    }
    saveConfig();
    intCount = 0;
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

void setStandBy() {
  
  if( modeSwitch == 0.00 ) {
    Serial.println(" Error - StandBy");
    pAutoPID->stop();
    digitalWrite(RELAY_CALE, RELAY_OFF);            
    digitalWrite(RELAY_CONG, RELAY_OFF);            
  }
  else {
    resetNeeded = true; 
    resetDirection = 0;
  }        
}
// ********************************************************

void setMode(float modeSwitch) {
  
  if( modeSwitch == 0.00 ) {
    Serial.println(" Error - StandBy");
    pAutoPID->stop();
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

// Function to setup PID direction
void setMyPID() {
  double kPHeater = (double)stringTofloat((char*)my_KpHeater.c_str());
  double kPFreezer = (double)stringTofloat((char*)my_KpFreezer.c_str());
  double kIHeater = (double)stringTofloat((char*)my_KiHeater.c_str());
  double kIFreezer = (double)stringTofloat((char*)my_KiFreezer.c_str());
  
  if( resetNeeded ) {
    if( resetDirection == 1 ) {
      pAutoPID->setGains(kPHeater,kIHeater,kPHeater);
      pAutoPID->reset(); 
      pAutoPID->run(); 
      Serial.print(" reset1 ");
    }
    else if( resetDirection == -1 ) {
      pAutoPID->setGains(kPFreezer,kIFreezer,kPFreezer);    
      pAutoPID->reset(); 
      pAutoPID->run(); 
      Serial.print(" reset2 ");
    }
    else {
      pAutoPID->reset(); 
      Serial.print(" reset3 ");
    }
    outputVal = 0; 
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

void mainRun() {

  double outputSent = 0;
  pAutoPID->run(); //call every loop, updates automatically at certain time interval
  myTimeInterval = outputVal;
  Serial.print(" ");
  onTrackTemp = pAutoPID->atSetPoint(offSet);
  Serial.print( onTrackTemp );

  timeStamp = millis();

  if( !onTrackTemp ) {
    if( myTimeInterval > 19*maxOutput/20 ) {
      if( modeSwitch == 1.00 ) {
        previousTimer = maxOutput*NUM_INTERVAL*1000;
        state = 1;
      }        
      else if( modeSwitch == 2.00 ) {
        previousTimer = 0;
        state = 0;
      }        
      else {
        previousTimer = maxOutput*NUM_INTERVAL*1000;
        state = 1;
      }        
      outputSent = previousTimer;
      Serial.print(" E1 ");
    }
    else if( (myTimeInterval < 19*maxOutput/20) && (myTimeInterval > maxOutput/20) ) {
      if( modeSwitch == 1.00 ) {
        previousTimer = myTimeInterval*NUM_INTERVAL*1000;
        state = 1;
      }        
      else if( modeSwitch == 2.00 ) {
        previousTimer = 0;
        state = 0;
      }        
      else {
        previousTimer = myTimeInterval*NUM_INTERVAL*1000;
        state = 1;
      }        
      outputSent = previousTimer;
      Serial.print(" E2 ");
    }      
    else if( myTimeInterval < -19*maxOutput/20) {
      if( modeSwitch == 1.00 ) {
        previousTimer = 0;
        state = 0;
      }        
      else if( modeSwitch == 2.00 ) {
        previousTimer = maxOutput*NUM_INTERVAL*1000;
        state = 2;
      }        
      else {
        previousTimer = maxOutput*NUM_INTERVAL*1000;
        state = 2;
      }        
      outputSent = -previousTimer;
      Serial.print(" E3 ");
    }
    else if( (myTimeInterval < -maxOutput/20) && (myTimeInterval > -19*maxOutput/20) ) {
      if( modeSwitch == 1.00 ) {
        previousTimer = 0;
        state = 0;
      }        
      else if( modeSwitch == 2.00 ) {
        previousTimer = -myTimeInterval*NUM_INTERVAL*1000;
        state = 2;
      }        
      else {
        previousTimer = -myTimeInterval*NUM_INTERVAL*1000;
        state = 2;
      }        
      outputSent = -previousTimer;
      Serial.print(" E4 ");
    }
    else {
      previousTimer = 0;
      state = 0;
      outputSent = previousTimer;
      Serial.print(" E5 ");      
    }
  }
  else {
    previousTimer = 0;
    state = 0;
    outputSent = previousTimer;
    Serial.print(" E6 ");      
  } 
  actTime = outputSent/1000;

}
// ********************************************************

void getTemperature() {
  Serial.println("");
  sensor_ferm.requestTemperatures();
  sensor_cong.requestTemperatures();
  
  temp = sensor_ferm.getTempCByIndex(0) + iSpindelOffset;
  cong = sensor_cong.getTempCByIndex(0) + iSpindelOffset;
  temp = temp - (sensorOffset - temp/80)/2;
  cong = cong + (sensorOffset - cong/80)/2;  
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
  pUbiclient->add(OUTPUT_LABEL, myOutput); //Insert your variable Labels and the value to be sent
  pUbiclient->ubidotsPublish(deviceName);
}
// ********************************************************

void sendCaleTemp() {
  pUbiclient->add(FERMENTER_LABEL, temperature); //Insert your variable Labels and the value to be sent
  pUbiclient->ubidotsPublish(deviceName); 
}
// ********************************************************

void sendCongTemp() {
  pUbiclient->add(FREEZER_LABEL, freezer); //Insert your variable Labels and the value to be sent
  pUbiclient->ubidotsPublish(deviceName); 
}
// ********************************************************

/****************************************
 * Configuration Wifi AP Function
 ****************************************/
void initConf() {
  char *ssid = STASSID;
  char *password = STAPSK;
  
 // You can remove the password parameter if you want the AP to be open.
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/", handleRoot);
  server.on("/factoryDefaults", factoryDefaults);
  server.on("/showForm", showForm);
  server.on("/settings", settings);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP Configuration server started");

}
// ********************************************************

/****************************************
 * Wifi Client connection
 ****************************************/
/****************************************
 * BrewTemp setup
 ****************************************/
void initWifiClient() {
  WiFi.mode(WIFI_STA);

  modeSwitch = stringTofloat((char*)my_Mode.c_str());
  tempSet = stringTofloat((char*)my_TempSet.c_str());
  offSet = stringTofloat((char*)my_TempOffset.c_str());

  setPoint = tempSet;
  setMode(modeSwitch);
  maxOutput = ((double)stringTofloat((char*)my_Interval.c_str()))/NUM_INTERVAL;

  deviceName = (char*)my_Name.c_str();
  pUbiclient = new Ubidots((char*)my_Token.c_str());

  pUbiclient->ubidotsSetBroker(HTTPSERVER); // Sets the broker properly for the educational account
  pUbiclient->setDebug(false); // Pass a true or false bool value to activate debug messages

  pUbiclient->wifiConnection((char*)my_Wifi.c_str(), (char*)my_WPass.c_str());
  pUbiclient->begin(callback);
  pUbiclient->ubidotsSubscribe(deviceName, TEMPSET_LABEL); //Insert the dataSource and Variable's Labels
  pUbiclient->ubidotsSubscribe(deviceName, MODE_LABEL); //Insert the dataSource and Variable's Labels
  pUbiclient->ubidotsSubscribe(deviceName, OFFSET_LABEL); //Insert the dataSource and Variable's Labels

  // Init Temperature sensors
  sensorOffset = (double)stringTofloat((char*)my_SensorOffset.c_str());
  iSpindelOffset = (double)stringTofloat((char*)my_IspindelOffset.c_str());

  sensor_ferm.begin();
  sensor_cong.begin();

  // Define Digital Outputs for Relays
  pinMode(RELAY_CALE, OUTPUT);
  digitalWrite(RELAY_CALE, RELAY_OFF);
  pinMode(RELAY_CONG, OUTPUT);
  digitalWrite(RELAY_CONG, RELAY_OFF);
  
  pAutoPID = new AutoPID(&temperature, &setPoint, &outputVal, -maxOutput, maxOutput, 0,0,0); 
  pAutoPID->setBangBang(10*offSet);
  pAutoPID->setTimeStep(1000*maxOutput);
}
// ********************************************************

/****************************************
 * BrewTemp loop
 ****************************************/
void brewloop() {
// put your main code here, to run repeatedly:
unsigned long currentMillis = millis();
  
  
  if(!pUbiclient->connected()){
    digitalWrite(RELAY_CONG, RELAY_OFF);            
    digitalWrite(RELAY_CALE, RELAY_OFF);            
    digitalWrite(GREEN_LED, LOW);
    delay(BLINK_INTERVAL*3);
    digitalWrite(GREEN_LED, HIGH);
    delay(BLINK_INTERVAL*3);
    pUbiclient->reconnect();
    pUbiclient->ubidotsSubscribe(deviceName, TEMPSET_LABEL); //Insert the dataSource and Variable's Labels
    pUbiclient->ubidotsSubscribe(deviceName, MODE_LABEL); //Insert the dataSource and Variable's Labels
    pUbiclient->ubidotsSubscribe(deviceName, OFFSET_LABEL); //Insert the dataSource and Variable's Labels
  }
  if (currentMillis - previousMillis >= maxOutput*1000 ) {
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
      else if( temperature > setPoint && cong < 2*setPoint - temperature - FREEZER_DIFF ) {
          Serial.print(" MINFREEZER");
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
        if( intCount == -1 ) {
          mainRun();  
        }
        if( intCount == 0 ) {
          mainRun(); 
          pAutoPID->setTimeStep(999*NUM_INTERVAL*maxOutput);
          switch ( state ) {
            case 0 :
              digitalWrite(RELAY_CALE, RELAY_OFF);
              digitalWrite(RELAY_CONG, RELAY_OFF);
              break;

            case 1 :
              digitalWrite(RELAY_CALE, RELAY_ON);
              digitalWrite(RELAY_CONG, RELAY_OFF);
              break;

            case 2 :
              digitalWrite(RELAY_CALE, RELAY_OFF);
              digitalWrite(RELAY_CONG, RELAY_ON);
              break;

            default :
              digitalWrite(RELAY_CALE, RELAY_OFF);
              digitalWrite(RELAY_CONG, RELAY_OFF);
          }         
          sendCaleCongState(actTime);
          Serial.print(" ");
          Serial.print(actTime);
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
    previousTimer = maxOutput*NUM_INTERVAL*1000;
    if( myTimeInterval > 0 ) {
      digitalWrite(RELAY_CALE, RELAY_OFF);            
    }
    else {
      digitalWrite(RELAY_CONG, RELAY_OFF);            
    }
  } 
}
// ********************************************************

void setup(void) {
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);
  Serial.println();
  Serial.print("BrewTemp version: ");
  Serial.println(VERSION);

  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);

delay(1100);
  if (drd.detectDoubleReset()) {
    Serial.println("Double Reset Detected");
    drd.stop();
    initConf();
  }
  else {
    Serial.println("No Double Reset Detected");
    switch (getConfig()) {
      case FILEOK : {
        Serial.println("FILEOK");
        initWifiClient();
      }
        break;
      case FILEEMPTY : {
        Serial.println("FILEEMPTY");
        initConf();
      }
        break;
      case FILENOTFOUND : {
        Serial.println("FILENOTFOUND");
        initConf();
      }
        break;
      case FSNOTMOUNT : {
        Serial.println("FSNOTMOUNT");
        initConf();
      }
        break;
      default :
        Serial.println("default");
        initWifiClient();        
    }
  }
}
// ********************************************************

void loop(void) {
  drd.loop();
  MDNS.update();

  int wmode = WiFi.getMode();
  switch ( wmode ) {
    case WIFI_AP :
      digitalWrite(GREEN_LED, LOW);
      delay(BLINK_INTERVAL);
      digitalWrite(GREEN_LED, HIGH);
      delay(BLINK_INTERVAL);
      server.handleClient();
      break;
    case WIFI_STA :
      digitalWrite(GREEN_LED, LOW);
      brewloop();
      pUbiclient->loop();
      break;
    case WIFI_AP_STA :
      digitalWrite(GREEN_LED, LOW);
      delay(BLINK_INTERVAL*3);
      digitalWrite(GREEN_LED, HIGH);
      delay(BLINK_INTERVAL*3);
      WiFi.softAPdisconnect(true);
      break;
    default :
      digitalWrite(GREEN_LED, LOW);
      delay(BLINK_INTERVAL*3);
      digitalWrite(GREEN_LED, HIGH);
      delay(BLINK_INTERVAL*3);
      Serial.println("WIFI_OFF");
  }
}
