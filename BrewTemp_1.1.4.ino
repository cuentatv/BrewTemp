//# Copyright (c) 2019 Luis Pérez Manzanal.
//
//# BrewFerm 1.1.15
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
//###   - Version 1.1.3 (2019/11/14)  Description:  -- Blink green led when sending data
//###                                               -- Timeout for Wifi AP Configuration to avoid AP mode due to power off
//###                                               -- Automatic restart without Wifi connection
//###   - Version 1.1.4 (2019/11/16)  Description:  -- Blink green led shows states:
//###                                               -- AP->300ms * 90 secs w/o Configuration, prevents AP mode due to power off
//###                                               -- AP->300ms * 90 secs w Configuration, oo
//###                                               -- CheckWifi->1500ms * 45 secs w/o Wifi
//###                                               -- CheckUbidots->1500ms * oo
//###                                               -- Send Data->100ms * 4,5 secs
//###                                               -- Receive Data->66,6ms * 3 secs
//###   - Version 1.1.5 (2019/11/29)  Description:  -- Use temperature promedian for better acurrancy
//###   - Version 1.1.6 (2019/12/04)  Description:  -- Execution without connection
//###   - Version 1.1.7 (2019/12/08)  Description:  -- Remove modification 1.1.5
//###   - Version 1.1.8 (2020/01/03)  Description:  -- Addition temperature ramp, number between 0 and 99
//###                                               -- Ex.: tempset to 20, and cold crash to 0,2ºC, time to reach such temp 48 hours
//###                                               -- rampHours number is: 48
//###   - Version 1.1.9 (2020/01/05)  Description:  -- Remove offset as Ubidots variable
//###   - Version 1.1.10 (2020/01/10) Description:  -- Put all PID variables inside code and web configuration
//###   - Version 1.1.11 (2020/01/23) Description:  -- External log, code and configuration
//###   - Version 1.1.12 (2020/01/28) Description:  -- Update ramphours in Ubidots when ramp is finished.
//###   - Version 1.1.13 (2020/02/13) Description:  -- Remove iSpindelOffset, add CongOffset variable to calibrate better temp sensors
//###   - Version 1.1.14 (2020/03/13) Description:  -- Improve ramp actuation when Ubidots connection is broken
//###   - Version 1.1.15 (2020/03/17) Description:  -- Create rampTempSet variable to store initial setup for ramp
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
#include <TickerScheduler.h>


#define VERSION "1.1.15" // BrewTemp version


ESP8266WebServer server(HTTPPORT);
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

String my_Wifi;
String my_WPass;
String my_Token;
String my_Name;
String my_Interval;
String my_TempSet;
String my_RampTempSet;
String my_RampHours;
String my_TempOffset;
String my_FermOffset;
String my_CongOffset;
String my_Mode;
String my_KpHeater;
String my_KpFreezer;
String my_KiHeater;
String my_KiFreezer;
String my_KdHeater;
String my_KdFreezer;
String my_Uri;

const char formHead[] = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" charset=\"utf-8\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/><title>";
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
boolean sendFlag = false;
int intCount = -1;
double maxOutput = 0;
float modeSwitch;
float rampHours = 0;
float tempSet;
float offSet;
double temperature = 0;
double freezer = 0;
double setPoint = 0;
double outputVal = 0;
double myTimeInterval = 0;
double temp;
double cong;
double fermOffset;
double congOffset;
char* deviceName;
double actTime = 0;
int state = 0;

boolean waitFlag = false;

unsigned long timeStampRamp = 0;
double rampTempSet;

boolean uriFlag = false;
String my_Buffer;
String my_AuxBuffer;
int extLogCount = 10;

WiFiClient client;

TickerScheduler ts(5);
/****************************************
/****************************************
 * Logging Functions
 ****************************************/
void logSerial(char* text) {
  my_Buffer += String(text);
}
void logSerial(String text) {
  my_Buffer += text;
}
void logSerial(int text) {
  my_Buffer += String(text);
}
void logSerial(double text) {
  my_Buffer += String(text);
}
void logSerial(float text) {
  my_Buffer += String(text);
}
void logSerial(bool text) {
  my_Buffer += String(text);
}
void logSerial(unsigned text) {
  my_Buffer += String(text);
}
// ********************************************************

void logSerialLn(char* text) {
  my_Buffer += String(text);
  sendLog();
}
void logSerialLn(String text) {
  my_Buffer += text;
  sendLog();
}
void logSerialLn(int text) {
  my_Buffer += String(text);
  sendLog();
}
void logSerialLn(double text) {
  my_Buffer += String(text);
  sendLog();
}
void logSerialLn(float text) {
  my_Buffer += String(text);
  sendLog();
}
void logSerialLn(boolean text) {
  my_Buffer += String(text);
  sendLog();
}
void logSerialLn(unsigned text) {
  my_Buffer += String(text);
  sendLog();
}
// ********************************************************

void sendLog() {

  Serial.println(my_Buffer);
  String _buffer = "line=";
  
  if(my_AuxBuffer.length() > 1000 )
    my_AuxBuffer.remove(0);
  my_AuxBuffer += my_Buffer;
  
  if( !uriFlag && (extLogCount < 3)) {
    if ( my_Uri.length() > 0 ) {
      if( checkUri() )
        uriFlag = true;
      my_AuxBuffer += "\n";  
    }
  }
  if( uriFlag ){
    // Use WiFiClient class to create TCP connections
    extLogCount = 0;
    String logHost = my_Uri;
    _buffer += my_AuxBuffer;
    logHost.replace("http://","");
    int _index = logHost.indexOf("/");
    String _host = logHost.substring(0, _index);
    if (client.connect( _host.c_str(), HTTPPORT)) {
    
      // We now send data via POST
      //  curl -i -d line=my_Buffer http://host:port/ejemplo.php
      // This will send the request to the server
      String _request = "POST ";
      _request += logHost.substring(_index);
      _request += " HTTP/1.1\r\n";
      _request += "Host: " + _host + "\r\n";
      _request += "Content-Length: ";
      _request.concat(_buffer.length());
      _request += "\r\n";
      _request += "Content-Type: application/x-www-form-urlencoded\r\n\r\n";
      _request += _buffer + "\r\n\r\n";
      client.print(_request);
      my_AuxBuffer.remove(0);  
    }
  }
  my_Buffer.remove(0);
  client.stop();      
}
// ********************************************************

bool checkUri() {

  String logHost = my_Uri;
  logHost.replace("http://","");
  int _index = logHost.indexOf("/");
  String _host = logHost.substring(0, _index);
  if ( !client.connect( _host.c_str(), HTTPPORT) ) {
    Serial.println("External log is not accessible!!!");
    extLogCount++;
    return false;
  }
  else {
    return true;
  }
  client.stop();
}
// ********************************************************
/****************************************
 * Configuration web server Functions
 ****************************************/
void handleRoot() {
  String my_Main = formHead;
  my_Main += "BrewTemp Setup</title></head>";
  my_Main += formStyle;
  my_Main += "<body><div class=\"container\"><h2>BrewTemp Setup</h2><form method=\"get\" action=\"showForm\"><fieldset><legend><b>RESET TO FACTORY DEFAULTS</b></legend>";
  my_Main += "<p>This implies to remove internal configuration file, format internal Flash Memory and create an empty configuration file.</p>";
  my_Main += "<p>This action will take about 10 seconds to be completed. BrewTemp is restarted. </p>";
  my_Main += "<button class=\"btn\" type=\"submit\" formaction=\"factoryDefaults\">RESET TO FACTORY DEFAULTS</button></fieldset>";
  my_Main += "<fieldset><legend><b>BrewTemp Configuration</b></legend>";
  my_Main += "<p>BrewTemp gets several configuration parameters than can not be handled directed by Ubidots IoT platform, such as network setup (SSID and passphrase), Ubitdots Key-Token and device name.</p>";
  my_Main += "<p>Other parameters as PID setting could be handled by Ubidots but are omitted due to overhead to manage too many control parameters. Only the most important temperature parameters can be modified at configuration mode as well as at runtime from Ubidots, these parameters are temperature, temperature ramp, offset and operation mode. However, temperature offset is omitted due to variable is rarely modified during fermentation process.</p>";
  my_Main += "<p>An external log server can be used to log all output data from Serial Output, it only works when Uri text field is filled.</p>";
  my_Main += "<p>This action will take about 10 seconds to be completed when it is pressed SAVE button. BrewTemp is restarted and will start in operational mode.</p>";
  my_Main += "<button class=\"btn\" type=\"submit\">CONFIGURATION</button></fieldset></form></div></body></html>";
  server.send(200, "text/html", my_Main);

}
// ********************************************************

void handleNotFound() {
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
  logSerialLn("showForm");
  
  if(getConfig() != FILENOTFOUND ) {
    String my_Form = formHead;
    my_Form += "BrewTemp Configuration</title></head>";
    my_Form += formStyle;
    my_Form += "<body><div class=\"container\"><h2>BrewTemp Configuration</h2><br/><form method=\"get\" action=\"settings\">";

    my_Form += "<fieldset><legend><b>Network Settings</b></legend><label>Wireless Network (SSID)</label><input id=\"my_Wifi\" name=\"my_Wifi\" length=32 placeholder=\"SSID\" value=\"";
    my_Form += my_Wifi + "\" required>";  
    
    my_Form += "<label>Password</label><input id=\"my_WPass\" name=\"my_WPass\" length=64 placeholder=\"password\" value=\"";
    my_Form += my_WPass + "\" required></fieldset>";
 
    my_Form += "<fieldset><legend><b>Ubidots Settings</b></legend><label>Ubidots Token</label><input id=\"my_Token\" name=\"my_Token\" length=64 placeholder=\"Token\" value=\"";
    my_Form += my_Token + "\" required>";
  
    my_Form += "<label for=\"my_Name\">BrewTemp Device Name</label><input id=\"my_Name\" name=\"my_Name\" maxlength=10 placeholder=\"BrewTemp Name\" value=\"";
    my_Form += my_Name + "\" required>";
  
    my_Form += "<label for=\"my_Interval\">Measurement Interval (secs)</label><input id=\"my_Interval\" name=\"my_Interval\" length=3 placeholder=\"Measurement Interval (secs)\" value=\"";
    my_Form += my_Interval + "\" type=\"number\" min=\"100\" max=\"600\" step=\"1\" required></fieldset>";
  
    my_Form += "<fieldset><legend><b>Temperature Settings</b></legend><label for=\"my_TempSet\">Temperature</label><input id=\"my_TempSet\" name=\"my_TempSet\" maxlength=4 placeholder=\"Temperature\" value=\"";
    my_Form += my_TempSet + "\" type=\"number\" min=\"0\" max=\"30\" step=\"0.1\" required>";
  
    my_Form += "<label for=\"my_RampHours\">Temperature Ramp Hours</label><input id=\"my_RampHours\" name=\"my_RampHours\" maxlength=2 placeholder=\"RampHours\" value=\"";
    my_Form += my_RampHours + "\" type=\"number\" min=\"0\" max=\"99\" step=\"1\" required>";
  
    my_Form += "<label for=\"my_TempOffset\">Temperature Offset</label><input id=\"my_TempOffset\" name=\"my_TempOffset\" maxlength=4 placeholder=\"Offset\" value=\"";
    my_Form += my_TempOffset + "\" type=\"number\" min=\"0\" max=\"1\" step=\"0.05\" required>";
  
    my_Form += "<label for=\"my_FermOffset\">Fermentor Sensor Deviation</label><input id=\"my_FermOffset\" name=\"my_FermOffset\" maxlength=4 placeholder=\"FermOffset\" value=\"";
    my_Form += my_FermOffset + "\" type=\"number\" min=\"-5\" max=\"5\" step=\"0.01\" required>";
  
    my_Form += "<label for=\"my_CongOffset\">Freezer Sensor Deviation</label><input id=\"my_CongOffset\" name=\"my_CongOffset\" maxlength=4 placeholder=\"CongOffset\" value=\"";
    my_Form += my_CongOffset + "\" type=\"number\" min=\"-5\" max=\"5\" step=\"0.01\" required>";
  
    my_Form += "<label for=\"my_Mode\">Operation Mode </label><select name=\"my_Mode\">";  
    switch( my_Mode.toInt() ) {
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

    my_Form += "</select></fieldset>";
    
    my_Form += "<fieldset><legend><b>PID Settings</b></legend><label for=\"my_KpHeater\">KpHeater</label><input id=\"my_KpHeater\" name=\"my_KpHeater\" maxlength=4 placeholder=\"KpHeater\" value=\"";  
    my_Form += my_KpHeater + "\" type=\"number\" min=\"0\" max=\"10\" step=\"0.05\" required>";
  
    my_Form += "<label for=\"my_KiHeater\">KiHeater</label><input id=\"my_KiHeater\" name=\"my_KiHeater\" maxlength=4 placeholder=\"KiHeater\" value=\"";
    my_Form += my_KiHeater + "\" type=\"number\" min=\"0\" max=\"2\" step=\"0.05\" required>";
  
    my_Form += "<label for=\"my_KdHeater\">KdHeater</label><input id=\"my_KdHeater\" name=\"my_KdHeater\" maxlength=4 placeholder=\"KdHeater\" value=\"";
    my_Form += my_KdHeater + "\" type=\"number\" min=\"0\" max=\"10\" step=\"0.05\" required>";
  
    my_Form += "<label for=\"my_KpFreezer\">KpFreezer</label><input id=\"my_KpFreezer\" name=\"my_KpFreezer\" maxlength=4 placeholder=\"KpFreezer\" value=\"";
    my_Form += my_KpFreezer + "\" type=\"number\" min=\"0\" max=\"10\" step=\"0.05\" required>";

    my_Form += "<label for=\"my_KiFreezer\">KiFreezer</label><input id=\"my_KiFreezer\" name=\"my_KiFreezer\" maxlength=4 placeholder=\"KiFreezer\" value=\"";
    my_Form += my_KiFreezer + "\" type=\"number\" min=\"0\" max=\"2\" step=\"0.05\" required>";

    my_Form += "<label for=\"my_KdFreezer\">KdFreezer</label><input id=\"my_KdFreezer\" name=\"my_KdFreezer\" maxlength=4 placeholder=\"KdFreezer\" value=\"";
    my_Form += my_KdFreezer + "\" type=\"number\" min=\"0\" max=\"10\" step=\"0.05\" required></fieldset>";

    my_Form += "<fieldset><legend><b>External log</b></legend><label>URI</label><input id=\"my_Uri\" name=\"my_Uri\" length=64 placeholder=\"Uri\" value=\"";
    my_Form += my_Uri + "\"></fieldset><button class=\"btn\" type=\"submit\">SAVE</button></form></div></body></html>";

    server.send(200, "text/html", my_Form);
    ts.remove(2);
  }
  else
    handleNotFound();
}
// ********************************************************

/****************************************
 * Configuration data/file Functions
 ****************************************/

int getConfig() {
  logSerialLn("getConfig");
  logSerialLn("Mounting FS...");

  if (SPIFFS.begin())
  {
    logSerialLn(" Mounted!");
    if (SPIFFS.exists(CONFIGFILE))
    {
      // file exists, reading and loading
      logSerialLn("Reading config file");
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
          if (json.containsKey("RampTempSet"))
            my_RampTempSet = (const char *)json["RampTempSet"];
          if (json.containsKey("RampHours"))
            my_RampHours = (const char *)json["RampHours"];
          if (json.containsKey("TempOffset"))
            my_TempOffset = (const char *)json["TempOffset"];
          if (json.containsKey("FermOffset"))
            my_FermOffset = (const char *)json["FermOffset"];
          if (json.containsKey("CongOffset"))
            my_CongOffset = (const char *)json["CongOffset"];
          if (json.containsKey("Mode"))
            my_Mode = (const char *)json["Mode"];
          if (json.containsKey("KpHeater"))
            my_KpHeater = (const char *)json["KpHeater"];
          if (json.containsKey("KiHeater"))
            my_KiHeater = (const char *)json["KiHeater"];
          if (json.containsKey("KdHeater"))
            my_KdHeater = (const char *)json["KdHeater"];
          if (json.containsKey("KpFreezer"))
            my_KpFreezer = (const char *)json["KpFreezer"];
          if (json.containsKey("KiFreezer"))
            my_KiFreezer = (const char *)json["KiFreezer"];
          if (json.containsKey("KdFreezer"))
            my_KdFreezer = (const char *)json["KdFreezer"];
          if (json.containsKey("Uri"))
            my_Uri = (const char *)json["Uri"];

          logSerialLn("Parsed config:");
          if( my_Wifi == "\0" ) {
            logSerialLn("WARNING: Configuration file is empty");
            return FILEEMPTY;
          }  
          else {
            String _flash;
            json.printTo(_flash);
            logSerialLn(_flash);
            logSerialLn("");
            return FILEOK;
          }
        }
        else
        {
          logSerialLn("ERROR: Failed to load json config");
          return FILENOTFOUND;
        }
      }
      logSerialLn("ERROR: Unable to open config file");
      return FILENOTFOUND;
    }
    else 
    {
      logSerialLn("ERROR: Config file does not exist");
      return FILENOTFOUND;
    }
  }
  else
  {
    logSerialLn(" ERROR: Failed to mount FS!");
    return FSNOTMOUNT;
  }
}
// ********************************************************

void formatSpiffs()
{
  logSerialLn("\nNeed to format SPIFFS: ");
  SPIFFS.end();
  SPIFFS.begin();
  logSerialLn(SPIFFS.format());
}
// ********************************************************

bool saveConfig()
{
  logSerialLn("Saving config...");

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
  json["RampTempSet"] = my_RampTempSet;
  json["RampHours"] = my_RampHours;
  json["TempOffset"] = my_TempOffset;
  json["FermOffset"] = my_FermOffset;
  json["CongOffset"] = my_CongOffset;
  json["Mode"] = my_Mode;
  json["KpHeater"] = my_KpHeater;
  json["KiHeater"] = my_KiHeater;
  json["KdHeater"] = my_KdHeater;
  json["KpFreezer"] = my_KpFreezer;
  json["KiFreezer"] = my_KiFreezer;
  json["KdFreezer"] = my_KdFreezer;
  json["Uri"] = my_Uri;

  File configFile = SPIFFS.open(CONFIGFILE, "w+");
  if (!configFile)
  {
    logSerialLn("failed to open config file for writing");
    SPIFFS.end();
    return false;
  }
  else
  {
    String _flash;
    json.printTo(configFile);
    json.printTo(_flash);
    logSerialLn(_flash);

    configFile.close();
    SPIFFS.end();
    logSerialLn("saved successfully");
    return true;
  }
}
// ********************************************************

void settings() {
  logSerialLn("Settings");

  my_Wifi = server.arg("my_Wifi");
  my_WPass = server.arg("my_WPass");
  my_Token = server.arg("my_Token");
  my_Name = server.arg("my_Name");
  my_Interval = server.arg("my_Interval");  
  my_TempSet = server.arg("my_TempSet");
  my_RampTempSet = my_TempSet;
  my_RampHours = server.arg("my_RampHours");
  my_TempOffset = server.arg("my_TempOffset");
  my_FermOffset = server.arg("my_FermOffset");
  my_CongOffset = server.arg("my_CongOffset");
  my_Mode = server.arg("my_Mode");
  my_KpHeater = server.arg("my_KpHeater");
  my_KiHeater = server.arg("my_KiHeater");
  my_KdHeater = server.arg("my_KdHeater");
  my_KpFreezer = server.arg("my_KpFreezer");
  my_KiFreezer = server.arg("my_KiFreezer");
  my_KdFreezer = server.arg("my_KdFreezer");
  my_Uri = server.arg("my_Uri");

  if(saveConfig()) {
    ESP.restart();
  }

}
// ********************************************************

void factoryDefaults() {
logSerialLn("Factory Defaults");
  my_Wifi.remove(0);
  my_WPass.remove(0);
  my_Token.remove(0);
  my_Name.remove(0);
  my_Interval.remove(0);
  my_TempSet.remove(0);
  my_RampTempSet.remove(0);
  my_RampHours.remove(0);
  my_TempOffset.remove(0);
  my_FermOffset.remove(0);
  my_CongOffset.remove(0);
  my_Mode.remove(0);
  my_KpHeater.remove(0);
  my_KiHeater.remove(0);
  my_KdHeater.remove(0);
  my_KpFreezer.remove(0);
  my_KiFreezer.remove(0);
  my_KdFreezer.remove(0);
  my_Uri.remove(0);

  logSerialLn("Mounting FS...");

  if (SPIFFS.begin())
  {
    logSerialLn(" Mounted!");
    if (SPIFFS.exists(CONFIGFILE)) {
      SPIFFS.remove(CONFIGFILE);
      if (SPIFFS.exists(CONFIGFILE))
        logSerialLn(" Configuration file is not deleted");
      else
        logSerialLn(" Configuration file is deleted");       
    }
    else {             
      logSerialLn("ERROR: Config file does not exist");
    }
  }
  else
  {
    logSerialLn(" ERROR: Failed to mount FS!");
  }
  if(saveConfig()) {
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
  float preModeSwitch = my_Mode.toFloat();
  float preTempSet = my_TempSet.toFloat();
  float preRampTempSet = my_RampTempSet.toFloat();
  float preRampHours = my_RampHours.toFloat();
  

  for (int i=0;i<length;i++)
    myValue[i] = payload[i];

  pstr = strstr( topic,(char*)my_Name.c_str());
  if(pstr != NULL) {
    pstr = strstr( topic, RAMP_LABEL);
    
    if(pstr != NULL) {
      rampHours = (int)stringTofloat( myValue );
      logSerial(rampHours);
      logSerial(" ");
      logSerialLn(topic);  

      if(rampHours >= 0 && rampHours <= 99 ) {
        if( preRampHours != rampHours ) {        
          if( (preRampHours == 0) && (rampHours != 0) ) {
            my_RampHours.remove(0);
            my_RampHours = rampHours;
            timeStampRamp = millis() - RAMP_INTERVAL;
            logSerialLn("Ramp 1");
          }
          else if( (preRampHours != 0) && (rampHours != 0) ) {
            rampHours = preRampHours;
            logSerialLn("Ramp 2");
          }
          else if( (preRampHours != 0) && (rampHours == 0) ) {
            my_RampHours.remove(0);
            my_RampHours = rampHours;
            timeStampRamp = millis() - RAMP_INTERVAL;
            logSerialLn("Ramp 3");
          }
          else {
            rampHours = 0;
            logSerialLn("Ramp 4");
          }
        }
      }           
    }

    pstr = strstr( topic, MODE_LABEL);
    if(pstr != NULL) {
      modeSwitch = (int)stringTofloat( myValue );
      logSerial(modeSwitch);
      logSerial(" ");
      logSerialLn(topic);  

      if(modeSwitch >= 0 && modeSwitch <=3 ) {
        if( preModeSwitch != modeSwitch ) {
          my_Mode.remove(0);
          my_Mode = modeSwitch;
          setMode(modeSwitch);
          resetNeeded = true;
          resetDirection = 0;
          saveConfig();
          intCount = 0;
        }
      }
    }

    pstr = strstr( topic,TEMPSET_LABEL);
    if(pstr != NULL) {
      double auxTempSet = stringTofloat( myValue );
      auxTempSet = (float)((int)(auxTempSet*10 + 0.5))/10;
      logSerial(auxTempSet);
      logSerial(" ");
      logSerialLn(topic);   

      if(auxTempSet >= 0 && auxTempSet <=30 ) {
        if( preTempSet !=  auxTempSet ) {
          if( rampHours == 0) {
            tempSet = auxTempSet;
            rampTempSet = auxTempSet;
            my_TempSet.remove(0);
            my_TempSet = tempSet;
            my_RampTempSet.remove(0);
            my_RampTempSet = rampTempSet;
            saveConfig();
            
            resetNeeded = true;        
            resetDirection = 0;
            intCount = 0;
          }
          else {
            my_RampTempSet.remove(0);
            my_RampTempSet = preRampTempSet;
          }
        }  
      }
    }
  }
  ts.add(3, BLINK_INTERVAL/4, [&](void *) { blink0(); }, nullptr, true);
  ts.add(4, BLINK_INTERVAL*10, [&](void *) { blink2(); }, nullptr, false);
  setMyPID();
}
// ********************************************************

// Function to convert char* to float
float stringTofloat(char* myString) {
  char myChar[sizeof(myString) + 1];
  for (uint8_t j = 0; j < sizeof(myString)+1; j++) {
      myChar[j] = myString[j];
  }
  return atof(myChar);
}
// ********************************************************

void setRamp() {

  boolean saveFlag = false;
  float preTempSet = my_TempSet.toFloat();
  float preRampHours = my_RampHours.toFloat();
  float preRampTempSet = my_RampTempSet.toFloat();

  tempSet = preTempSet;
  rampTempSet = preRampTempSet;
  rampHours = preRampHours;
  logSerial(" Old TempSet:");
  logSerial(tempSet);
  logSerial(" ");
  logSerial(rampTempSet);
  logSerial(" ");
  logSerialLn(rampHours);

  if( rampHours > 0 ) {
    if( tempSet == rampTempSet )
      tempSet = temperature -( temperature - rampTempSet )/(rampHours--);    
    else
      tempSet = tempSet -( tempSet - rampTempSet )/(rampHours--);    

    tempSet = (float)((int)(tempSet*10 + 0.5))/10;
    my_TempSet.remove(0);
    my_TempSet = tempSet;
    logSerial(" New TempSet:");
    logSerialLn(tempSet);
  }
  else {
    tempSet = rampTempSet;
  }
  if( tempSet != preTempSet ) {
    my_TempSet.remove(0);
    my_TempSet = tempSet; 
    saveFlag = true;
    sendTempSet();
  }
  if( rampHours != preRampHours ) {
    my_RampHours.remove(0);
    my_RampHours = rampHours;
    saveFlag = true;
    sendRamp();
  }
  if( saveFlag )
    saveConfig();
  
  resetNeeded = true;        
  resetDirection = 0;
  intCount = 0;  

}
// ********************************************************

void setStandBy() {
  
  if( modeSwitch == 0.00 ) {
    logSerialLn(" Error - StandBy");
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
    logSerialLn(" Error - StandBy");
    pAutoPID->stop();
    digitalWrite(RELAY_CALE, RELAY_OFF);            
    digitalWrite(RELAY_CONG, RELAY_OFF);            
  }
  else if( modeSwitch == 1.00 ) {
    logSerialLn(" Only Heating ");
    digitalWrite(RELAY_CONG, RELAY_OFF);            
  }        
  else if( modeSwitch == 2.00 ) {
    logSerialLn(" Only Cooling ");
    digitalWrite(RELAY_CALE, RELAY_OFF);            
  }        
  else {                          // Heating and Cooling
    logSerialLn(" Both Heating and Cooling ");
    resetNeeded = true; 
    resetDirection = 0;
  }        
}
// ********************************************************

// Function to setup PID direction
void setMyPID() {

  double kPHeater = (double)my_KpHeater.toFloat();
  double kIHeater = (double)my_KiHeater.toFloat();
  double kDHeater = (double)my_KdHeater.toFloat();
  double kPFreezer = (double)my_KpFreezer.toFloat();
  double kIFreezer = (double)my_KiFreezer.toFloat();
  double kDFreezer = (double)my_KdFreezer.toFloat();
  
  if( resetNeeded ) {
    if( resetDirection == 1 ) {
      pAutoPID->setGains(kPHeater,kIHeater,kDHeater);
      pAutoPID->reset(); 
      pAutoPID->run(); 
      logSerial(" reset1 ");
    }
    else if( resetDirection == -1 ) {
      pAutoPID->setGains(kPFreezer,kIFreezer,kDFreezer);    
      pAutoPID->reset(); 
      pAutoPID->run(); 
      logSerial(" reset2 ");
    }
    else {
      pAutoPID->reset(); 
      logSerial(" reset3 ");
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
      logSerial(" H ");
    }
  }
  else if( temperature > setPoint ) {
    if( resetDirection != -1 ) {
      resetNeeded = true;
      resetDirection = -1;
      logSerial(" F ");
    }
  }
  else {
    resetNeeded = false; 
    resetDirection = 0;
    logSerial(" N ");
  }
  setMyPID();    
}
// ********************************************************

void mainRun() {

  double outputSent = 0;
  pAutoPID->run(); //call every loop, updates automatically at certain time interval
  myTimeInterval = outputVal;
  logSerial(" ");
  onTrackTemp = pAutoPID->atSetPoint(offSet);
  logSerial(onTrackTemp);

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
      logSerial(" E1 ");
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
      logSerial(" E2 ");
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
      logSerial(" E3 ");
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
      logSerial(" E4 ");
    }
    else {
      previousTimer = 0;
      state = 0;
      outputSent = previousTimer;
      logSerial(" E5 ");      
    }
  }
  else {
    previousTimer = 0;
    state = 0;
    outputSent = previousTimer;
    logSerial(" E6 ");
  } 
  actTime = outputSent/1000;

}
// ********************************************************

void getTemperature() {
  sensor_ferm.requestTemperatures();
  sensor_cong.requestTemperatures();
  
  temp = sensor_ferm.getTempCByIndex(0)*(1 + fermOffset/100);
  cong = sensor_cong.getTempCByIndex(0)*(1 + congOffset/100);
  logSerial(" ");
  logSerial(temp);
  logSerial(" ");
  logSerial(cong);
  logSerial(" ");
  logSerial(tempSet);
  logSerial(" ");
  logSerial(intCount);

  temperature = temp;
  freezer = cong;
  setPoint = tempSet;

}
// ********************************************************

void sendCaleCongState(double myOutput) {
  pUbiclient->add(OUTPUT_LABEL, myOutput); //Insert your variable Labels and the value to be sent
  pUbiclient->ubidotsPublish(deviceName);
  sendFlag = true;
  logSerial(" ");
  logSerial(myOutput);
  logSerialLn(" Sending Data ....");
}
// ********************************************************

void sendCaleTemp() {
  pUbiclient->add(FERMENTER_LABEL, temp); //Insert your variable Labels and the value to be sent
  pUbiclient->ubidotsPublish(deviceName); 
  sendFlag = true;
}
// ********************************************************

void sendCongTemp() {
  pUbiclient->add(FREEZER_LABEL, freezer); //Insert your variable Labels and the value to be sent
  pUbiclient->ubidotsPublish(deviceName); 
  sendFlag = true;
}
// ********************************************************

void sendRamp() {
  pUbiclient->add(RAMP_LABEL, rampHours); //Insert your variable Labels and the value to be sent
  pUbiclient->ubidotsPublish(deviceName); 
  logSerial(" Temperature ramp changed: ");
  logSerialLn(rampHours);
}
// ********************************************************

void sendTempSet() {
  pUbiclient->add(TEMPSET_LABEL, tempSet); //Insert your variable Labels and the value to be sent
  pUbiclient->ubidotsPublish(deviceName); 
  logSerial(" Temperature Setup changed: ");
  logSerialLn(tempSet);
}
// ********************************************************

void blink0() {
    digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
}
// ********************************************************

void blink1() {
  ts.remove(0);
  ts.remove(1);
  digitalWrite(GREEN_LED, LOW);
}
// ********************************************************

void wifiAPUp() {
  logSerialLn("WIFIAP Timeout");
  ts.remove(2);
  ESP.restart();
}
// ********************************************************

void blink2() {
  ts.remove(3);
  ts.remove(4);
  digitalWrite(GREEN_LED, LOW);
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
//  IPAddress myIP = WiFi.softAPIP();
  logSerial("AP IP address: ");
//  logSerialLn(myIP);
  logSerialLn(WiFi.softAPIP().toString());
  server.on("/", handleRoot);
  server.on("/factoryDefaults", factoryDefaults);
  server.on("/showForm", showForm);
  server.on("/settings", settings);
  server.onNotFound(handleNotFound);
  server.begin();
  logSerialLn("HTTP Configuration server started");

}
// ********************************************************

/****************************************
 * Wifi Client connection
 ****************************************/

bool checkWifiConnection( char* ssid, char* pass ) {
    WiFi.begin(ssid, pass);
    for( uint8_t i = 0; i < 11; i++) {
      if (WiFi.status() != WL_CONNECTED) {
        digitalWrite(GREEN_LED, LOW);
        delay(BLINK_INTERVAL*5);
        digitalWrite(GREEN_LED, HIGH);
        delay(BLINK_INTERVAL*5);    
        digitalWrite(GREEN_LED, LOW);
        if( i == 10 )
          ESP.restart();
      }
      else
        break;       
    }
    WiFi.disconnect();
}
// ********************************************************

/****************************************
 * BrewTemp setup
 ****************************************/
void initWifiClient() {
  WiFi.mode(WIFI_STA);

  extLogCount = 0;
  
  modeSwitch = (double)my_Mode.toFloat();
  tempSet = (double)my_TempSet.toFloat();
  offSet = (double)my_TempOffset.toFloat();
    
  rampTempSet = tempSet;
  setPoint = tempSet;
  setMode(modeSwitch);
  maxOutput = (double)(my_Interval.toFloat()/NUM_INTERVAL);

  deviceName = (char*)my_Name.c_str();
  pUbiclient = new Ubidots((char*)my_Token.c_str());

  pUbiclient->ubidotsSetBroker(HTTPSERVER); // Sets the broker properly for the educational account
  pUbiclient->setDebug(false); // Pass a true or false bool value to activate debug messages

  checkWifiConnection((char*)my_Wifi.c_str(), (char*)my_WPass.c_str());

  pUbiclient->wifiConnection((char*)my_Wifi.c_str(), (char*)my_WPass.c_str());
  
  logSerial("BrewTemp local IP Address: ");
  logSerialLn(WiFi.localIP().toString());

  byte mac[6];
  WiFi.macAddress(mac);
  char macAddr[18];
  sprintf(macAddr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  logSerial("BrewTemp local MAC Address: ");
  logSerialLn(macAddr);
  
  pUbiclient->begin(callback);
  pUbiclient->ubidotsSubscribe(deviceName, RAMP_LABEL); //Insert the dataSource and Variable's Labels
  pUbiclient->ubidotsSubscribe(deviceName, TEMPSET_LABEL); //Insert the dataSource and Variable's Labels
  pUbiclient->ubidotsSubscribe(deviceName, MODE_LABEL); //Insert the dataSource and Variable's Labels

  // Init Temperature sensors
  fermOffset = (double)my_FermOffset.toFloat();
  congOffset = (double)my_CongOffset.toFloat();

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

  uriFlag = false;
  
}
// ********************************************************

/****************************************
 * BrewTemp loop
 ****************************************/
void brewloop() {
// put your main code here, to run repeatedly:
  
  
  if(!pUbiclient->connected()){
    logSerialLn(" Ubidots not connected ....");
    digitalWrite(GREEN_LED, HIGH);
    delay(BLINK_INTERVAL*3);
    digitalWrite(GREEN_LED, LOW);
    delay(BLINK_INTERVAL);
    digitalWrite(GREEN_LED, HIGH);
    delay(BLINK_INTERVAL*3);
    digitalWrite(GREEN_LED, LOW);
    delay(BLINK_INTERVAL);
    digitalWrite(GREEN_LED, HIGH);
    delay(BLINK_INTERVAL*3);
    digitalWrite(GREEN_LED, LOW);
    if ( pUbiclient->reconnectWOLoop() ) {
      logSerialLn(" Ubidots connected ....");
      pUbiclient->reconnect();
      pUbiclient->ubidotsSubscribe(deviceName, RAMP_LABEL); //Insert the dataSource and Variable's Labels
      pUbiclient->ubidotsSubscribe(deviceName, TEMPSET_LABEL); //Insert the dataSource and Variable's Labels
      pUbiclient->ubidotsSubscribe(deviceName, MODE_LABEL); //Insert the dataSource and Variable's Labels
    }
  }
  
  unsigned long currentMillis = millis();
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
        logSerial(" MINFREEZER");
        digitalWrite(RELAY_CONG, RELAY_OFF); 
        waitFlag = true;           
        timeStamp = millis();
        if( intCount == 0 )
          sendCaleCongState(0);
      }
      else if( ( temperature > setPoint && cong < 2*setPoint - temperature - FREEZER_DIFF/2 ) && waitFlag ) {
        logSerial(" FREEZER WAIT STATE");
        digitalWrite(RELAY_CONG, RELAY_OFF);            
        timeStamp = millis();
        if( intCount == 0 ) 
          sendCaleCongState(0);
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
          waitFlag = false;
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
    logSerialLn("");
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

  if ( millis() - timeStampRamp >= RAMP_INTERVAL ) {
    timeStampRamp = millis();
    logSerialLn(" Ramp Timeout...");
    extLogCount = 2;
    logSerialLn("");
    setRamp();
  } 

  if (sendFlag) {
    ts.add(0, BLINK_INTERVAL/3, [&](void *) { blink0(); }, nullptr, true);
    ts.add(1, BLINK_INTERVAL*15, [&](void *) { blink1(); }, nullptr, false);
    sendFlag = false;
  }    
}
// ********************************************************

void setup(void) {
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);

  logSerialLn("");
  logSerial("BrewTemp version: ");
  logSerialLn(VERSION);

  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);

  if (drd.detectDoubleReset()) {
    drd.stop();
    delay(5000);
    logSerialLn("Double Reset Detected");
    initConf();
    ts.add(2, BLINK_INTERVAL*300, [&](void *) { wifiAPUp(); }, nullptr, false);
  }
  else {
    logSerialLn("No Double Reset Detected");
    switch (getConfig()) {
      case FILEOK : {
        logSerialLn("FILEOK");
        initWifiClient();
      }
        break;
      case FILEEMPTY : {
        logSerialLn("FILEEMPTY");
        initConf();
      }
        break;
      case FILENOTFOUND : {
        logSerialLn("FILENOTFOUND");
        initConf();
      }
        break;
      case FSNOTMOUNT : {
        logSerialLn("FSNOTMOUNT");
        initConf();
      }
        break;
      default :
        logSerialLn("default");
        initWifiClient();        
    }
  }
}
// ********************************************************

void loop(void) {
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
      brewloop();
      if(pUbiclient->connected())
        pUbiclient->loop();
      drd.loop();        
      break;
    case WIFI_AP_STA :
      ESP.restart();
      break;
    default :
      ESP.restart();
  }
  ts.update();

}
