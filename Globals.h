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
#define DOUBLERESETDETECTOR_FLAG_SET 0xD0D01234
#define DOUBLERESETDETECTOR_FLAG_CLEAR 0xD0D04321

#define WIFIENADDR 1
#define RTCVALIDFLAG 0xCAFEBABE
#define RTCNOTVALIDFLAG 0xCAFEBEBA

// SPIFFS STATES
#define FILEOK 0
#define FILEEMPTY 1
#define FILENOTFOUND 2
#define FSNOTMOUNT 3

#define HTTPSERVER "industrial.api.ubidots.com"     // Industrial Educational URL
//#define HTTPSERVER "things.ubidots.com"     // Industrial Educational URL
#define FERMENTER_LABEL "tempferm" // Your variable label
#define FREEZER_LABEL "tempcong" // Your variable label
#define MODE_LABEL "mode" // Your variable label
#define TEMPSET_LABEL "tempset" // Your variable label
#define OFFSET_LABEL "offset" // Your variable label
#define OUTPUT_LABEL "outputtime" // Your variable label

#define ONE_WIRE_FERM 5  // Digital port D1
#define ONE_WIRE_CONG 4  // Digital port D2

#define GREEN_LED 13  // Digital port D7

#define RELAY_CALE 16  // Digital port D0
#define RELAY_CONG 12  // Digital port D6

#define RELAY_ON LOW  // Relays are activated in low
#define RELAY_OFF HIGH  // Relays are activated in low

#define NUM_INTERVAL 10 // Number of intervals inside of Measurement Interval

#define BLINK_INTERVAL 200 // Number of intervals inside of Measurement Interval

#define MIN_LIMIT 0.1     // Minimum Temperature
#define FREEZER_DIFF 3.2  // Maximum difference between temperature and freezer
#define MAX_LIMIT 30      // Maximum Temperature
