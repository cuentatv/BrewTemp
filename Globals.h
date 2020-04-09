/**************************************************************
//#    "BrewTemp"
//# Copyright (c) 2019 Luis Pérez Manzanal.
//#
//# Author: Luis Pérez Manzanal
//#
 **************************************************************/

/****************************************
 * Define Constants
 ****************************************/
// Initial WIFI data for BrewTemp Configuration AP
#ifndef STASSID
#define STASSID "BrewTemp"
#define STAPSK  ""
#define HTTPPORT  80
#endif


// Configuration file, persistent storage
#define CONFIGFILE "/brewtemp.json"

// Number of seconds after reset during which a 
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 1
// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

// SPIFFS STATES
#define FILEOK 0
#define FILEEMPTY 1
#define FILENOTFOUND 2
#define FSNOTMOUNT 3

#define HTTPSERVER "industrial.api.ubidots.com"     // Industrial  URL
//#define HTTPSERVER "things.ubidots.com"     //  Educational URL
#define FERMENTER_LABEL "tempferm" // Fermentor temperature variable label
#define FREEZER_LABEL "tempcong" // Freezer temperature variable label
#define MODE_LABEL "mode" // Mode variable label: 0 StandBy, 1 Only Heat, 2 Only Cool and 3 Heat-Cool. Control variable
#define TEMPSET_LABEL "tempset" // Temperature Objective variable label. Control variable
#define OUTPUT_LABEL "outputtime" // Time at seconds for actuation variable label, Heat > 0 , Cool < 0. It is dependant of Interval value
#define RAMP_LABEL "ramphours" // Time in hours to reach a tempset, variable label. Control variable

#define ONE_WIRE_FERM 5  // Digital port D1
#define ONE_WIRE_CONG 4  // Digital port D2

#define GREEN_LED 13  // Digital port D7, Several blink sequence specifies status

#define RELAY_CALE 16  // Digital port D0, for Heat actuation and red led
#define RELAY_CONG 12  // Digital port D6, for Cool actuation and blue led

#define RELAY_ON LOW  // Relays are activated in low
#define RELAY_OFF HIGH  // Relays are activated in low

#define NUM_INTERVAL 10 // Number of intervals inside of Measurement Interval
#define BLINK_INTERVAL 300 // Number of intervals inside of Measurement Interval

#define MIN_LIMIT 0.1     // Minimum Temperature, to avoid congelation
#define FREEZER_DIFF 3.2  // Maximum difference between temperature and freezer, to avoid excesive deviation when system is cooling
#define MAX_LIMIT 30      // Maximum Temperature, to avoid overheating

#define RAMP_INTERVAL 3600000 // One hour in milisecs, interval when ramp hours is discounted

#define HTTPPORT 80      // http port for external log server

#define NUM_FAILS 100   // Number of loops without Ubidots connection, after that a new resubscription is performed.
