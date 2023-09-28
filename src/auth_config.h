// change this file wih your actual parametrs 

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <NTPClient.h>


//*
//*      local Blynk configuration 
//*
#if defined(TEST_VERSION)
char auth[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";             // token for training device
IPAddress   blynk_server2(64, 233, 164, 102);                 // Blynk server IP address exposed to WAN
#else
char auth[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";             // token for working sensor
IPAddress   blynk_server2(10, 10, 15, 15);                    //  IP address in local network
#endif
const char* blynk_server1 = "xxx.yyy.net";                    //  Blynk server domain name

//*
//*     MQTT configuration
//*
#if defined(TEST_VERSION)
IPAddress mqtt_server(64, 233, 164, 102);                      // MQTT server IP address exposed to WAN
#else
IPAddress mqtt_server(10, 10, 15, 15);                          //  IP address in local network
#endif
const char* mqtt_server2 = "xxx.yyy.net";                      //  MQTT server domain name


const char* mqtt_clientId = "Weather_Sensor";
const char* mqtt_username = "username";
const char* mqtt_password = "password";               
const char* topic1 = "tele/Weather_01/SENSOR";                  //Tasmota style MQTT topics
//const char* topic2 = "stat/Weather_01/POWER";
//const char* topic3 = "cmnd/Weather_01/POWER";             


//*
//*       NTP configuration
//*
const long utcOffsetInSeconds = 10800;                          //time zone Moscow  UTC+3
const char* ntp_server3 = "ntp4.stratum2.ru"; 
const char* ntp_server2 = "ru.pool.ntp.org"; 
const char* ntp_server1 = "ntp.ix.ru";
String formatedTime = "2023-09-20T12:00:00";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntp_server1, utcOffsetInSeconds);



//*        
//*       WiFi configuration
//*
String ssid = "main_ssid";                                      // WiFi networks, allowed to connect
String ssid_a1 = "alt1_ssid";
String ssid_a2 = "alt2_ssid";
String ssid_a3 = "alt3_ssid";
char pass[] = "password";
char null_pass[] = "";
IPAddress device_ip(10, 10, 15, 16);                           //static IP in local network
IPAddress gateway_ip(10, 10, 15, 1);
IPAddress dns1_ip(77, 88, 8, 8);
IPAddress dns2_ip(77, 88, 8, 1);
IPAddress subnet_mask(255, 255, 255, 0);


//*
//*        OTA configuration
//*
const char* ota_server = "xxx.yyy.net";                         // http server - location of firmvare file
const uint16_t ota_port = 8080;
const char* ota_file = "/ota_bin/weather_sensor.bin";           // firmware path and filename
