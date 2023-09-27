//
//
//#define BLYNK_PRINT Serial                      //defines output for Blynk mesages
//#define BLYNK_DEBUG  
//#define DEBUG_ESP_WIFI
//#define DEBUG_ESP_PORT Serial
#define TEST_VERSION
#define CURRENT_VER_N "2023-09-27 v10.09"



#include <Arduino.h>
#include <Wire.h>
#include <GyverBME280.h>
#include <MAX44009.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <NTPClient.h>


//*
//* Constants
//*
#define MARK_P 64
#define PARAM_P 65
#define BASE_P 70
#define NET_P 72
#define BUF_BEGIN 80
#define MAX_TRIES 60
#define SLEEP_TIME 15e6                                       // 1 sleep quantum of time (microseconds)
#define VDD_MAX 4.25f
#define VDD_MIN 3.1f

#ifdef TEST_VERSION
#define TIME_CHECK_PERIOD 0x0003                              // periods to skip before next pool NTP
#define STANDARD_ERROR 0                                      // msecs per minute
#else
#define TIME_CHECK_PERIOD 0x01FF                              // periods to skip before next pool NTP
#define STANDARD_ERROR -2000                                  // msecs per minute
#endif

#include <auth_config.h>                                      // authentication and private information

//*
//*        Varaibles for deepsleep mode
//*
uint32_t marker = 0;
uint32_t WAKE_TEST = 0x0FF0AAC3;
uint32_t AWAKE = 0x0;


//*
//*        Sensor configuration
//*
GyverBME280 bme; // I2C
MAX44009 light;

float h;                                                      //humidity
float t;                                                      //temperature
float l;                                                      //luminance
float p;                                                      //pressure
long rssi;
float vdd;
float charge=100.0;
uint32_t tt;
byte NO_PRINT=0;
byte OTA_REQ=0;

uint32_t hFilter=BUF_BEGIN;                                   // 5 readings buffer (float)
uint32_t tFilter=BUF_BEGIN+6; 
uint32_t lFilter=BUF_BEGIN+12; 
uint32_t pFilter=BUF_BEGIN+18; 
float buffer[5];
float buff;
uint32_t count = 0;

struct saved_params_t {                     // flags and paraneters structure
  uint32_t localEpochTime :32;              // Epoch time counted locally
  uint32_t lastTimeCheck :32;               // Epoch time last checked by NTP
  byte NO_PRINT :1;                         // don't send informaion to serial
  byte OTA_REQ :1;                          // start OTA upgrade session
  byte s1 :1;                               // I2C first sensor present
  byte s2 :1;                               // I2C second sensor present
  byte use_old_network :1;                  // need to find new network
  byte poolNtp :1;                          // get time fron NTP
  uint32_t sleep :7;                        // current quantums of sleeping
  uint32_t d :14;                           // time duration of the last cycle operations (milliseconds)
  uint32_t cnt :21;                         // cycles counter
  int32_t error :16;                        // error of deepsleep timing
  
};

static union {
  //uint64_t flags;
  uint32_t f_array[4];
  saved_params_t flg;
};

struct saved_net_config_t {                 // network parameters structure
  char ssid[13];
  uint8_t bssid[6];
  IPAddress ip4;
  uint8_t channel :7;
  bool open :1;
} net_config;


 
uint8_t paramConvV11[] = {0, 2, 4, 8, 20, 40, 120};
uint8_t temp_counter=0;


int32_t oneTicCount()
{
  return ((int32_t)SLEEP_TIME / 1e6 * (int32_t)flg.sleep );
}


/*****************************************************************************************************************************************/
void showFlags()
{
  Serial.print("no print:         ");
  Serial.println(flg.NO_PRINT);
  Serial.print("OTA request:      ");
  Serial.println(flg.OTA_REQ);
  Serial.print("I2C sensor 1:     ");
  Serial.println(flg.s1);
  Serial.print("I2C sensor 2:     ");
  Serial.println(flg.s2);
  Serial.print("actual network:   ");
  Serial.println(flg.use_old_network);
  Serial.print("sleep:            ");
  Serial.println(flg.sleep);
  Serial.print("counter:          ");
  Serial.println(flg.cnt);
  Serial.print("duration:         ");
  Serial.println(flg.d);
  Serial.print("check time:       ");
  Serial.println(flg.poolNtp);
  Serial.print("epoch time:       ");
  Serial.println(flg.localEpochTime);
  Serial.print("last check:       ");
  Serial.println(flg.lastTimeCheck);
  Serial.print("error:       ");
  Serial.println(flg.error);

}
/*****************************************************************************************************************************************/

void saveParams()
{
  #ifdef TEST_VERSION
  showFlags();
  #endif
  ESP.rtcUserMemoryWrite(PARAM_P, &f_array[0], sizeof(flg));
}


/*****************************************************************************************************************************************/
void initSensor()
{
  if (!flg.NO_PRINT) Serial.println("Initialising peripheria");
  
  if (!flg.NO_PRINT) Serial.print("BME280 sensor ");
  bme.setMode(FORCED_MODE);
  for (int i=4; i >= 0; i--)
  {
    if (!bme.begin(0x76))
    {
       if (!flg.NO_PRINT) Serial.print(".");
       flg.s1 = 0;
       delay(100); 
    }
    else
    {
       if (!flg.NO_PRINT) Serial.println("is active.");
       flg.s1 = 1;
       break;
    }
    if (i==0) {if (!flg.NO_PRINT) Serial.println(" not found.");}
  }

  if (!flg.NO_PRINT) Serial.print("MAX44009 sensor ");
  for (int i=4; i >= 0; i--)
  {
    if (light.begin()==0)
    {
        if (!flg.NO_PRINT) Serial.println("is active.");
        flg.s2 = 1;
        break;
    }
    else
    {
       if (!flg.NO_PRINT) Serial.print(".");
       flg.s2 = 0;
       delay(100);
    }
    if (i==0) {if (!flg.NO_PRINT) Serial.println(" not found.");}
  }
}
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
void sendToBlynk() 
{
  if (!flg.NO_PRINT) Serial.println("Configuring Blynk ");
  Blynk.config(auth, blynk_server2, 8080);
  
  if (!flg.NO_PRINT) Serial.println("Connecting to Blynk");
  Blynk.connect(10000);

  
  if (Blynk.connected() != true)
  {
    if (!flg.NO_PRINT) Serial.println("Failed to connect Blynk by IP. Trying by name ");
    Blynk.config(auth, blynk_server1, 8080);
    Blynk.connect(10000);
  }
  
  if (Blynk.connected() == true)
  {
    if (!flg.NO_PRINT) Serial.println("Syncronising pins ");
    //Blynk.syncAll();
    Blynk.syncVirtual(V1);
    Blynk.syncVirtual(V2);
    Blynk.syncVirtual(V11);
    
  

    if (!flg.NO_PRINT) Serial.println("Start sending sensors");
    //rssi = WiFi.RSSI();
    Blynk.virtualWrite(V3, t);
    Blynk.virtualWrite(V4, h);
    if (p>600 && p<850) Blynk.virtualWrite(V5, p);
    Blynk.virtualWrite(V6, l);
    Blynk.virtualWrite(V7, float(WiFi.RSSI()));
    Blynk.virtualWrite(V8, vdd);
    Blynk.virtualWrite(V9, float(flg.d));
    
    if (flg.cnt <3) Blynk.virtualWrite(V13, CURRENT_VER_N);    //переписать!!!
    if (flg.use_old_network==0)
    {
      
      Blynk.virtualWrite(V20, WiFi.localIP().toString());
      Blynk.virtualWrite(V21, WiFi.SSID());
      Blynk.virtualWrite(V23, WiFi.subnetMask().toString());
      Blynk.virtualWrite(V24, WiFi.gatewayIP().toString());
      flg.use_old_network=1;
    }
  }
}
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
void sendToSerial()
{
  
  Serial.print("temp: ");
  Serial.println(t);
  Serial.print("humid: ");
  Serial.println(h);
  Serial.print("pressure: ");
  Serial.println(p);
  Serial.print("luminance: ");
  Serial.println(l);
  Serial.print("Rssi: ");
  Serial.println(rssi);
  Serial.print("Vdd: ");
  Serial.println(vdd);
  Serial.print("time: ");
  Serial.println(flg.d);
    

}
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
void checkTime()
{
  flg.localEpochTime+=oneTicCount();
  if (flg.poolNtp)
  {   
    timeClient.begin();
    if (timeClient.forceUpdate())
    {
      flg.poolNtp=0;
      #ifdef TEST_VERSION
      Serial.println("Got time fron 1 server");
      #endif
      
    }
    else
    {
      #ifdef TEST_VERSION
      Serial.println("1 NTP server failed, trying 2");
      #endif
      
      timeClient.end();
      timeClient.setPoolServerName(ntp_server2);
      timeClient.begin();
      if (timeClient.forceUpdate())
      {
        flg.poolNtp=0;
        #ifdef TEST_VERSION
        Serial.println("Got time fron 2 server");
        #endif
      }
      else
      {
        #ifdef TEST_VERSION
        Serial.println("2 NTP server failed, trying 3");
        #endif
                
        timeClient.end();
        timeClient.setPoolServerName(ntp_server3);
        timeClient.begin();
        if (timeClient.forceUpdate())
        {
          flg.poolNtp=0;
          #ifdef TEST_VERSION
          Serial.println("Got time fron 3 server");
          #endif
        }
        else
        {
          #ifdef TEST_VERSION
          Serial.println("Time not set");
          #endif
          flg.poolNtp=1;
            
          #ifdef TEST_VERSION
          Serial.println("Time increased locally");
          #endif
        }
      }
    }
    timeClient.end();
    if (!flg.poolNtp)
    { 
      if (flg.cnt>1) 
      {            
        flg.error+=  (float)((int64_t)timeClient.getEpochTime()-(int64_t)flg.localEpochTime)
                    *(SLEEP_TIME / 1e3F * (float)flg.sleep)
                    /(float)(timeClient.getEpochTime()-flg.lastTimeCheck)
                    ;
      }
      flg.localEpochTime = timeClient.getEpochTime();
      flg.lastTimeCheck = timeClient.getEpochTime();
      Blynk.virtualWrite(V10, flg.error);
      #ifdef TEST_VERSION
      Serial.print("Error:   ");
      Serial.println(flg.error);
      #endif
    }
    

  }
  else
  {
    
    #ifdef TEST_VERSION
    Serial.println("Time increased locally");
    #endif
  }
  formatedTime = timeClient.getFormattedDate(flg.localEpochTime).substring(0,19);
  #ifdef TEST_VERSION
  Serial.println(formatedTime);
  #endif

}

/*****************************************************************************************************************************************/



/*****************************************************************************************************************************************/
void sendToMqtt()
{
  
 
  
  StaticJsonDocument<220> j_doc;
  char j_send_string[220];

  j_doc["Time"]=formatedTime;
  j_doc["BMP280"]["temperature"]= t;
  j_doc["BMP280"]["humidity"]= h;
  j_doc["BMP280"]["pressure"]= p;
  j_doc["TempUnit"]="C";
  j_doc["MAX44009"]["illuminance"]=l;
  j_doc["Battery"]["voltage"]=vdd;
  j_doc["Battery"]["charge"]=charge;

  serializeJson(j_doc, j_send_string);

  #ifdef TEST_VERSION
  Serial.println(j_send_string);
  #endif
  
  WiFiClient mqtt_client;
  PubSubClient ps_client(mqtt_server, 1883, mqtt_client);

    if (ps_client.connect(mqtt_clientId, mqtt_username, mqtt_password)) 
    {
      #ifndef TEST_VERSION
      ps_client.publish(topic1, j_send_string, true);
      #endif
      flg.use_old_network=1;
    }
    else
    {
      if (!flg.NO_PRINT) Serial.println("Failed to connect MQTT by IP. Trying by name ");
      ps_client.setServer(mqtt_server2, 1883);
      if (ps_client.connect(mqtt_clientId, mqtt_username, mqtt_password)) 
      {
        #ifndef TEST_VERSION
        ps_client.publish(topic1, j_send_string, true);
        #endif
        flg.use_old_network=1;
      }
      else
      {
        if (!flg.NO_PRINT) Serial.println("MQTT message not sent");
      }

    }
    
}
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
BLYNK_WRITE(V1) // Вывод в Serial
{
  flg.NO_PRINT = param.asInt();
  //saveParams();
}
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
BLYNK_WRITE(V2) // Start OTA
{
  flg.OTA_REQ = param.asInt();
  //saveParams();
}
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
BLYNK_WRITE(V11) // Change sleep time
{
  flg.error = (float)flg.error/(float)flg.sleep*(float)paramConvV11[param.asInt()];
  flg.sleep = paramConvV11[param.asInt()];
  //saveParams();
}
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
void connect_new_network()
{
  if (!flg.NO_PRINT) Serial.println("Searching another network");
  WiFi.config(0U, 0U, 0U, 0U, 0U);
  byte q = WiFi.scanNetworks();
  for (byte i = 0; i < q; i++)
  {
    if (!flg.NO_PRINT) Serial.printf("%d: %s, Ch:%d (%ddBm) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "");
    if (   WiFi.SSID(i) == ssid
        || WiFi.SSID(i) == ssid_a1
        || WiFi.SSID(i) == ssid_a2
        #ifdef TEST_VERSION 
        || WiFi.SSID(i) == ssid_a3
        #endif
        )
    {
      if (!flg.NO_PRINT) Serial.print("Connecting ");
      
      WiFi.SSID(i).toCharArray(net_config.ssid, 12);
      net_config.channel = WiFi.channel(i);
      net_config.bssid[0] = WiFi.BSSID(i)[0];
      net_config.bssid[1] = WiFi.BSSID(i)[1];
      net_config.bssid[2] = WiFi.BSSID(i)[2];
      net_config.bssid[3] = WiFi.BSSID(i)[3];
      net_config.bssid[4] = WiFi.BSSID(i)[4];
      net_config.bssid[5] = WiFi.BSSID(i)[5];

      if (WiFi.encryptionType(i) == ENC_TYPE_NONE)
      {
        if (!flg.NO_PRINT) Serial.print("without password>");
        net_config.open = true;
        WiFi.begin(net_config.ssid);
      }
      else
      {
        if (!flg.NO_PRINT) Serial.print("with password>");
        net_config.open = false;
        WiFi.begin(net_config.ssid, pass);
      }
      
      while (WiFi.status() != WL_CONNECTED && WiFi.status() != WL_CONNECT_FAILED)
      {
        delay(100);
        if (!flg.NO_PRINT) Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) 
      {   
        net_config.ip4 = WiFi.localIP();
        ESP.rtcUserMemoryWrite(NET_P, (uint32_t*) &net_config, sizeof(net_config));
        return;
      }
      else
      {
        if (!flg.NO_PRINT) Serial.println(" connect failed ");
      }

    }

  }
}
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
float filtered(uint32_t _memPos, float newVal) 
{
   
     
  uint32_t _memPos2 = _memPos + 5;
    
  ESP.rtcUserMemoryRead(_memPos, (uint32_t*) &buffer, 20);
  ESP.rtcUserMemoryRead(_memPos2, &count, 4);
  buffer[count] = newVal;
  ESP.rtcUserMemoryWrite(_memPos, (uint32_t*) &buffer, 20);
    
    
  for (int j=4; j>=1; j--) {
    for (int i=0; i<j; i++) {
      if (buffer[i] > buffer[i + 1]) {
        buff = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = buff;
      }
    }
  }
    
  if (++count >= 5) count = 0;
  ESP.rtcUserMemoryWrite(_memPos2, &count, 4);
      
  return ((buffer[1] + buffer[3]) / 2 + buffer[2]) / 2;
}
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
void readSensor()
{
  if (!flg.NO_PRINT) Serial.println("Start reading sensors");
  bme.oneMeasurement();
  while (bme.isMeasuring());
  t = filtered(tFilter, flg.s1 ? bme.readTemperature()                   : (((float)random(30))/100.0 + 10.0));
  p = filtered(pFilter, flg.s1 ? (bme.readPressure() * 0.00750061683F)   : (((float)random(70))/100.0 + 20.0));
  h = filtered(hFilter, flg.s1 ? bme.readHumidity()                      : (((float)random(100))/100.0 + 40.0*(float)(flg.cnt>>5)));
  l = filtered(lFilter, flg.s2 ? light.get_lux()                         : 5.0F*(float)sin((float)flg.cnt/10.186F) + (float)random(100)/30.0F);
  vdd = analogRead(A0) * 4.59433e-3;                                      // Замена одним коэффициентом / (1023.0 / 4.7)   R=150kOhm;
  charge = 101.0F - (101.0F / pow(1.0F + pow(1.33F * (vdd - VDD_MIN)/(VDD_MAX - VDD_MIN), 4.5F), 3.0F));    // уровень заряда в процентах                            
  if (!flg.NO_PRINT) Serial.println("Sensors read");
}
/*****************************************************************************************************************************************/





void setup() {
  

  tt = millis();
  ESP.rtcUserMemoryRead(MARK_P, &marker, 4); //checking reset status
  ESP.rtcUserMemoryWrite(MARK_P, &AWAKE, 4); // marking as Awake

  if (marker == WAKE_TEST)
    {
    ESP.rtcUserMemoryRead(PARAM_P, &f_array[0], sizeof(flg)); //read flags
    }
    else
    {
    f_array[0] = 0;
    f_array[1] = 0;
    f_array[2] = 0;
    f_array[3] = 0;
    flg.sleep = 4;
    flg.error = STANDARD_ERROR ;
    saveParams(); //reset flags

    byte n[96];
    for (int q=0;q<96;q++) n[q]=0;
    ESP.rtcUserMemoryWrite(BUF_BEGIN, (uint32_t*) &n, sizeof(n)); //reset buffer
    }

  

  #ifdef TEST_VERSION
  if (((flg.cnt & TIME_CHECK_PERIOD) == 0) || flg.cnt==5)
  #else
  if (((flg.cnt & TIME_CHECK_PERIOD) == 0) || flg.cnt==10 || flg.cnt==30) 
  #endif
  {
    flg.poolNtp=1;
  }    
  flg.cnt++;

  
  
  Serial.begin(74880);
  //Serial.setDebugOutput(true);  
  #ifdef TEST_VERSION
  showFlags();
  #endif

}

void loop() {
  
  initSensor(); 
  readSensor();
  
  #ifdef WIFI_IS_OFF_AT_BOOT
    enableWiFiAtBootTime();
  #endif
  
  
  if (WiFi.getAutoConnect())
    {
      Serial.println("Disable autoconnect");
      WiFi.setAutoConnect(false);
      WiFi.setAutoReconnect(false);
    }
  WiFi.forceSleepWake();
  WiFi.persistent(false);
  
  delay(1);
  WiFi.mode( WIFI_STA );
  
  delay(1);
  if (flg.use_old_network)
  {
    if (!flg.NO_PRINT) {
    Serial.println("Wake up after deep sleep");
    Serial.print("Connecting");
    }
    
    ESP.rtcUserMemoryRead(NET_P, (uint32_t*) &net_config, sizeof(net_config));
    gateway_ip = net_config.ip4;
    gateway_ip[3] = 1;

    WiFi.config(net_config.ip4, gateway_ip, subnet_mask, dns1_ip, dns2_ip);
    WiFi.begin(net_config.ssid, net_config.open ? null_pass : pass, net_config.channel, net_config.bssid);
    temp_counter=0;
    
    while (WiFi.status() != WL_CONNECTED)
      {
        delay(25);
        if (!flg.NO_PRINT) Serial.print("*");
        if (++temp_counter == MAX_TRIES) break;
      }
    if (temp_counter == MAX_TRIES)
      {
      flg.use_old_network=0; 
      }

    if (!WiFi.isConnected())
    {
      if (!flg.NO_PRINT) Serial.print("Failed to connect old network. ");
      connect_new_network();
    }
  }
  else
  {
    if (!flg.NO_PRINT) Serial.print("After first PowerOn ");
    connect_new_network();
  }

  if (WiFi.isConnected() == false)
  {
    if (!flg.NO_PRINT) Serial.println("Going to sleep not connected");
    ESP.rtcUserMemoryWrite(MARK_P, &WAKE_TEST, 4);
    flg.use_old_network=0;
    flg.d = millis()-tt;
    saveParams();
    ESP.deepSleep((int)SLEEP_TIME * flg.sleep, WAKE_RF_DEFAULT);
  }
 
  if (!flg.NO_PRINT) {
  Serial.println();
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  }
  
  sendToBlynk();
  checkTime();
  if (!flg.NO_PRINT) sendToSerial();
  sendToMqtt();
  

  if (!flg.OTA_REQ)
    {
    flg.d = millis()-tt;
    uint64_t time_to_sleep = ((int)SLEEP_TIME * flg.sleep - flg.d * 1000 - flg.error*1e3);  
    if (!flg.NO_PRINT) Serial.printf("Going to sleep %d sec\n", (int)(time_to_sleep >> 20));
    ESP.rtcUserMemoryWrite(MARK_P, &WAKE_TEST, 4);
    saveParams();
    delay( 1 );
    ESP.deepSleep(time_to_sleep, (flg.cnt & 0xF) == 0 ? WAKE_RF_DEFAULT : WAKE_NO_RFCAL);               // sleeping standart time
    delay(1000);
    }

  Serial.println("Begining OTA sesion");
  Blynk.virtualWrite(V2, 0);
  Blynk.syncVirtual(V2);
  delay(100);

  flg.OTA_REQ=0;
  flg.cnt=0;
  saveParams();
  ESP.rtcUserMemoryWrite(MARK_P, &WAKE_TEST, 4);

  if((WiFi.status() == WL_CONNECTED)) 
    {     
        Serial.println("Ready");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        
        WiFiClient ota_client;

        t_httpUpdate_return ret = ESPhttpUpdate.update(ota_client, ota_server, ota_port, ota_file);

        switch(ret) 
        {
            case HTTP_UPDATE_FAILED:
                Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                break;

            case HTTP_UPDATE_NO_UPDATES:
                Serial.println("HTTP_UPDATE_NO_UPDATES");
                break;

            case HTTP_UPDATE_OK:
                Serial.println("HTTP_UPDATE_OK");
                break;
        }
        
        
    }
  else
    {
      Serial.println("Network suddenly disconnected...");
    }


}