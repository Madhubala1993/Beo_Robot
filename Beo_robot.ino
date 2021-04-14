//////////////Beo Robot combined programs ///////////
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(3, 2);        // TX, RX  tx=d2, rx=d3
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);

#define SERVOMIN  125  // Minimum pulse length count out of 4096.
#define SERVOMAX  500 // Maximum pulse length count out of 4096.

char ssid[] = "beorobo";            // your network SSID (name)
char pass[] = "12345678";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

char packetBuffer[255];          // buffer to hold incoming packet
char ReplyBuffer[] = "ACK";      // a string to send back
unsigned int localPort = 100;
WiFiEspUDP Udp;
String a, action, action1, esp_data, i;
int statuss, exercise, delayTime, activity, count;
unsigned long delayStart = 0;
bool delayRunning = false;
int servoNum;
float angle;
int  timeDelay, cmd;
char actionset[100];
int x = 1, b = 0, steps = 0;
String dir;
int m;
int y[5];
unsigned long start, finished, elapsed;


#define SERVOMIN  140  // Minimum pulse length count out of 4096.
#define SERVOMAX  500 // Maximum pulse length count out of 4096.

int LS, RS, CS;
int value = 700;
#define LM1 2       // left motor   front
#define LM2 4     // left motor     front
#define LM3 5       // left motor   back
#define LM4 3     // left motor     back
#define RM1 30       // right motor  front  
#define RM2 33       // right motor  front
#define RM3 31       // right motor   back
#define RM4 32       // right motor   back

const int irrx1 = 15;
const int irrx2 = 14;
const int irrx3 = 50;
const int irrx4 = 51;
const int irrx5 = 52;
const int irrx6 = 24;

const int irtx1 = 12;
const int irtx2 = 13;


void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  WiFi.init(&Serial1);
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("WiFi shield not present");
    while (true);
  }
  Serial.print("Attempting to start AP ");
  Serial.println(ssid);
  status = WiFi.beginAP(ssid, 10, pass, ENC_TYPE_WPA2_PSK);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println("Access point started");
  Serial.println("Server started");
  Serial.println("Connected to wifi");
  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);
  Serial.print("Listening on port ");
  Serial.println(localPort);

  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(60);
  pwm1.setPWMFreq(60);

  pinMode(irrx1, INPUT);
  pinMode(irrx2, INPUT);
  pinMode(irrx3, INPUT);
  pinMode(irrx4, INPUT);
  pinMode(irrx5, INPUT);
  pinMode(irrx6, INPUT);
  pinMode(irtx1, OUTPUT);
  pinMode(irtx2, OUTPUT);

  pinMode(LS, INPUT);
  pinMode(CS, INPUT);
  pinMode(RS, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(LM3, OUTPUT);
  pinMode(LM4, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(RM3, OUTPUT);
  pinMode(RM4, OUTPUT);


  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(60);
  pwm1.setPWMFreq(60);
  //pwm.setPWM(1, 0, pulseWidth(90));
  for (servoNum = 1; servoNum <= 15;  servoNum++ )
  {
    pwm.setPWM(servoNum, 0, 150);
  }
  for (servoNum = 1; servoNum <= 15;  servoNum++ )
  {
    pwm1.setPWM(servoNum, 0, 150);
  }
  delay(1000);
}

void loop()
{
  comm();
  a = "";
}

void comm()
{
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    int len = Udp.read(packetBuffer, 255);
    if (len > 0)
    {
      packetBuffer[len] = 0;
    }
    Serial.print("Received Data : ");
    Serial.println(packetBuffer);
    a = String(a + packetBuffer);
    action = a.substring(0, 3);
    exercise = action.toInt();
    Serial.println(a);
    Serial.print("a  :  ");
    Serial.print(a);
    Serial.print("  exercise  :  ");
    Serial.println(exercise);
    condition();
  }
}

void condition()
{
  if (exercise == 500)
  {
    delay(1000);
    Serial.println("Line follower");
    line_follower();
  }
  else if (exercise == 501)
  {
    delay(1000);
    //obstacle_detector();
  }
  else if (exercise == 502)
  {
    delay(1000);
    //    camera_streaming();
  }
  else if (exercise == 503)
  {
    delay(1000);
    Serial.println("Plant Watering");
    plant_watering();
  }
  else if (exercise == 504)
  {
    delay(1000);
    Serial.println("Smart Switch");
    smart_switch();
  }
  else if (exercise == 505)
  {
    delay(1000);
    Serial.println("ventilator");
    ventilator();
  }
  else if (exercise == 506)
  {
    delay(1000);
    Serial.println("Smart bowler");
    smart_bowler();
  }
  else if (exercise == 507)
  {
    delay(1000);
    Serial.println("Churning curd");
   // churning_curd();
  }
  else if (exercise == 508)
  {
    delay(1000);
    Serial.println("Videography");
    videography();
  }
  else if (exercise == 509)
  {
    delay(1000);
    Serial.println("Inclined Plane");
    inclined_plane();
  }
  else if (exercise == 510)
  {
    delay(1000);
    Serial.println("Perpetual Motion");
    perpetual_motion();
  }
  else if (exercise == 511)
  {
    delay(1000);
    Serial.println("Dress Folding");
    dress_folding();
  }
  else if (exercise == 512)
  {
    delay(1000);
    Serial.println("Bed Robot");
    bed_robot();
  }
  else if (exercise == 513)
  {
    delay(1000);
    Serial.println("Bolt Assembling");
    //bolt_assembling();
  }
  else if (exercise == 514)
  {
    delay(1000);
    Serial.println("Arm holding");
    arm_holding();
  }
   else if (exercise == 514)
  {
    delay(1000);
    Serial.println("Gear system");
    gear_system();
  }
  
}
