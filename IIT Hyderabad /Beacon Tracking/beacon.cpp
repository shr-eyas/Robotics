#include <Arduino.h>

#include <WiFi.h>

const int MLA = 0;  
const int MLB = 1;  
const int MRA = 2;  
const int MRB = 3; 

/**
 * This is a header file which contains WiFi network credentials. 
 * Its contents are:
 *
 * const char *ssid = "YOUR-SSID-HERE";
 * const char *password = "YOUR-PASSWORD-HERE";
 *
*/
#include "credentials.h"

void setup() {

  ledcSetup(MLA, 5000, 8);
  ledcSetup(MLB, 5000, 8);
  ledcSetup(MRA, 5000, 8);
  ledcSetup(MRB, 5000, 8);

  ledcAttachPin(16, MLA);
  ledcAttachPin(17, MLB);
  ledcAttachPin(18, MRA);
  ledcAttachPin(19, MRB);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED);
}

void loop() 
{
  int Frssi = check_RSSI();
  while (Frssi>114)
  {
    int Frssi = check_RSSI();
  }
  rightTurn();
  int Rrssi = check_RSSI();
  leftTurn();
  leftTurn();
  int Lrssi = check_RSSI();
  int mxOf=maxof(Frssi,Rrssi,Lrssi);

  for (int i=0;i<mxOf;i++)
  {
    rightTurn();
  }
  forward();
  stop();
  delay(500);
}

void rightTurn()
{
  right();
  delay(400);
  stop();
  delay(500);
}

void leftTurn()
{
  left();
  delay(400);
  stop();
  delay(500);
}

int maxof(int a,int b,int c)
{
  int mx = 1;
  if (b>a)
  {
    mx = 2;
    if (c>=b)
    {
      mx = 0;   
    }
  }
 else if (c>=a)
 {
  mx=0;
 }
 return mx;
}

void front() {
  ledcWrite(MLA, 127);
  ledcWrite(MLB, 0); 
  ledcWrite(MRA, 127);
  ledcWrite(MRB, 0); 
}

void right() {
  ledcWrite(MLA, 127);
  ledcWrite(MLB, 0); 
  ledcWrite(MRA, 0);
  ledcWrite(MRB, 0); 
}

void left() {
  ledcWrite(MLA, 0);
  ledcWrite(MLB, 0); 
  ledcWrite(MRA, 127);
  ledcWrite(MRB, 0); 
}

void back() {
  ledcWrite(MLA, 0);
  ledcWrite(MLB, 127); 
  ledcWrite(MRA, 0);
  ledcWrite(MRB, 127); 
}

void stop() {
  ledcWrite(MLA, 0);
  ledcWrite(MLB, 127); 
  ledcWrite(MRA, 0);
  ledcWrite(MRB, 127); 
}

double check_RSSI() {
  int rssi=0;
  for(int i=0; i<20; i++)
  {
  while((rssi = WiFi.RSSI())>0);
  rssi = rssi + WiFi.RSSI();
  }
  rssi = rssi / 40;
  return rssi;
}
