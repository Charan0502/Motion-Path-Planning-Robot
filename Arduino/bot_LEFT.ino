#include "I2Cdev.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#define INTERRUPT_PIN A0
#include "MPU6050_6Axis_MotionApps20.h"
const char* ssid = "living Room wifi 5g";
const char* password = "plky4202";
 const char* mqtt_server = "192.168.1.25";
 int mqttf=0;
Servo drop;
int tempS=0;
int tempSR=0;
int tempSL=0;
int flagS=0; 
WiFiClient espClient;
int intial_speed_LEFT=500;
int intial_speed_RIGHT=500;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
int ij =-1;
double Kp=22, Ki=-0.0, Kd=80;
//double Kp2=6, Ki2=0.0001, Kd2=60;
double steer;
void setup_wifi() {
   delay(100);
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");
    }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

int x=0;

#define IN1 D0
#define IN2 D3
#define IN3 D4
#define IN4 D5
#define ENA D6
#define ENB D7
#define sp D8
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
double myftarget=0;
 

double yaw;
double currentyaw=0.0;
double initialpose=0.0;
int firstTime=0;
double setpoint=0.0;
double yawSetpoint=0.0;
double lastError=0.0;


double milliold=0;
double millinew;
double dt;

double area=0;

#define OUTPUT_READABLE_YAWPITCHROLL


#define LED_PIN D2

#define LED_PIN2 D4

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

const int StabilizeSeconds = 20;

void setYaw(double setpoint) {
  // computes a real yaw from a relative yaw
  // cleans up angle issues
  yawSetpoint = setpoint + initialpose;
//  if (yawSetpoint > 90.0)
//     yawSetpoint -= 360.0;
//  else if (yawSetpoint < -90.0)
//     yawSetpoint += 360.0;
}

//double Kp=8, Ki=-0.0, Kd=16;
//double Kp2=6, Ki2=0.0001, Kd2=60;
//double steer;
//double steer2;
PID myPID(&currentyaw, &steer, &yawSetpoint, Kp, Ki, Kd, DIRECT);
//PID myPID2(&currentyaw, &steer2, &yawSetpoint, Kp2, Ki2, Kd2, DIRECT);

void findyaw()
{
  // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("yaw :");
            yaw=ypr[0] * 180/M_PI;
            currentyaw=round(yaw-initialpose);
            //Serial.println(currentyaw);
            //Serial.print("/t");
         if(currentyaw<-120 && currentyaw>-179)
             {
              currentyaw=currentyaw+360;
              
             }
        #endif

        // blink LED to indicate activity
       
    }
}


double compute()
{
    setYaw(myftarget);
    steer=0;
    dt=millinew-milliold;
    int error = yawSetpoint - currentyaw;
    int steer = Kp*error + Kd*(error - lastError) + Ki*area;
//    if(steer>223)
//      steer = 223;
//    else if(steer<-223)
//      steer = -223; 
  //  area+=error;
    lastError = error;
    milliold=millinew;
    millinew=millis();
    //change();
    return steer;
}


void forward()
{
Serial.println(myftarget);
  if(ij==0){
    
    digitalWrite(IN1,1);
    digitalWrite(IN2,0);
    digitalWrite(IN3,1);
    digitalWrite(IN4,0);   
    if(flagS%2==0)
    {steer=compute();
    Serial.println(steer);
      analogWrite(ENA,intial_speed_LEFT -steer);
    analogWrite(ENB,intial_speed_RIGHT + steer);}
else if(flagS%2==1)
{
    Serial.println(steer);
   
    analogWrite(ENA,tempSR);
    analogWrite(ENB,tempSL);}
}
else if(ij==1)
{ //digitalWrite(IN1,1);
//    digitalWrite(IN2,0);
//    digitalWrite(IN3,0);
//    digitalWrite(IN4,1);
//     analogWrite(ENA,1023);
//    analogWrite(ENB,1023);
//    delay(300);
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    digitalWrite(IN3,0);
    digitalWrite(IN4,1);   
    steer=compute();
    Serial.println(steer);
   
    analogWrite(ENA,intial_speed_LEFT + steer);
    analogWrite(ENB,intial_speed_RIGHT - steer);
//  digitalWrite(IN1,0);
//    digitalWrite(IN2,1);
//    digitalWrite(IN3,0);
//    digitalWrite(IN4,1);   
//    steer=compute();
//    Serial.println(steer);
//    client.publish("post1","TXT2");
//    analogWrite(ENA,intial_speed_LEFT - steer);
//    analogWrite(ENB,intial_speed_RIGHT + steer);
}
  }

void callback(char* topic, byte* payload, unsigned int length) 
{
//  Serial.print("Command from MQTT broker is : [");
//  Serial.print(topic);
//  int p =(char)payload[0]-'0';
 char funde[length];
  int p;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
   // Serial.print((char)payload[i]);
  funde[i]=(char)payload[i];}
  
  Serial.println();
//for (int j = 0; j< length; j++)
//    Serial.print(funde[j]);
p=atoi(funde);
Serial.println(p);
  if(p==0) 
  {
  digitalWrite(IN1,0);
  digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,0);
  delay(2000);
  //exit(0); 
  Serial.println("Stop" );
  } 
if(p==100) 
flagS=0;
  if(p>100) 
  {flagS=1;
    if(tempS%2==0)
  {tempSR=p;
  tempS++;
    }
    else if(tempS%2==1)
  {tempSL=p;
  tempS++;
    }
 
  } 
   if(p==50) 
  {
  Kd=Kd/2;
  } 
  if(p==51) 
  {
  Kd=Kd*2;
  } 
   if(p==52) 
  {
  Kp=Kp/2;
  } 
  if(p==53) 
  {
  Kp=Kp*2;
  } 
  if(p==1)//right
  { 
    
    digitalWrite(IN1,0);
    digitalWrite(IN2,0);
    digitalWrite(IN3,0);
    digitalWrite(IN4,0);
    
    delay(1000);
    digitalWrite(IN1,1);
    digitalWrite(IN2,0);
    digitalWrite(IN3,0);
    digitalWrite(IN4,1);//right snip
     analogWrite(ENA,800);
    analogWrite(ENB,800);
    delay(100);
myftarget=90;
Serial.println("Forward Acceleration");

  }
if(p==2)
{
    digitalWrite(IN1,0);
    digitalWrite(IN2,0);
    digitalWrite(IN3,0);
    digitalWrite(IN4,0);
    delay(1000);
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    digitalWrite(IN3,1);
    digitalWrite(IN4,0);//left snip
     analogWrite(ENA,800);
    analogWrite(ENB,800);
    delay(100);
  myftarget=180;

Serial.println("Backward Acceleration");
  }
  if(p==3)
  {
//forward(90.0);
 myftarget=0;
Serial.println("Right Turn");

ij=0;
  }
  if(p==4)
  {
ij++;
  }
  if(p==5)
  {
digitalWrite(IN1,0);
digitalWrite(IN2,0);
digitalWrite(IN3,0);
digitalWrite(IN4,0);
for (int i=180;i>50;i--)
{
drop.write(i);
delay(5);
};

delay(1000);
//ij++;
 digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    digitalWrite(IN3,1);
    digitalWrite(IN4,0);
     analogWrite(ENA,1023);
    analogWrite(ENB,1023);
    delay(500);
    myftarget=-90;
digitalWrite(D8,LOW);

    Serial.println("Servo Stop n Drop");
  }
  if(p==6)
{myftarget=45;

Serial.println("Backward Acceleration");
  }
  if(p==7)
{
  myftarget=-45;
  

  Serial.println("Backward Acceleration");
}
  if(p==8)
{
   ij=-1;
}
 if(p==9)
{
   exit(0);
}
  Serial.println();
} 

void reconnect() {
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    String clientId = "Botpink-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      client.subscribe("botpink");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(6000);
    }
  }
}

void setup() {
  
  Serial.println("MQTT");
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(sp, OUTPUT);
    drop.attach(D8);
    Serial.begin(115200);    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
  
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    client.publish("post1","Testing device connections...");
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    if (!(mpu.testConnection()))
      client.publish("post1","MPU connection Failed");
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    client.publish("post1","Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(78);
    mpu.setYGyroOffset(111);
    mpu.setZGyroOffset(-40);
    mpu.setZAccelOffset(1023); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        client.publish("post1","Enabling DMP...");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
         client.publish("post1","DMP connection Failed");
         Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    
    //analogWrite(LED_PIN2,0);

    //pinMode(IN4, OUTPUT);
    Serial.println(F("Stabilizing..."));
      
    for (uint8_t i = 0; i < 15; i++) {
      //digitalWrite(LED_PIN, HIGH);
      delay(500);
      //digitalWrite(LED_PIN, LOW);
      delay(500);
    }
    Serial.println("yaw:\tsteer:\tsteer2:");
    myPID.SetMode(AUTOMATIC);
    //myPID2.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-223,223);
    //myPID2.SetOutputLimits(0, 223);
    millinew=millis();
    stopaf();
   drop.write(180); 
 
}

void loop() {

 if (!client.connected()) {
    reconnect();
       
  }
   findyaw();
   forward();
   printaf();
  client.loop();   
    if (firstTime==0) {
            initialpose = (double)ypr[0] * 180/M_PI;
            firstTime = 1;
   client.publish("post1","Ready.....Botpink"); }

    
}





//void right()
//{
//    digitalWrite(IN1,0);
//    digitalWrite(IN2,0);
//    digitalWrite(IN3,1);
//    digitalWrite(IN4,0);
//    steer=compute(90.0);
//    analogWrite(ENA,intial_speed_LEFT - steer);
//    analogWrite(ENB,intial_speed_RIGHT + steer);
//    
//}
//
//void left()
//{
//    digitalWrite(IN1,1);
//    digitalWrite(IN2,0);
//    digitalWrite(IN3,0);
//    digitalWrite(IN4,0);
//    steer=compute(-90.0);
//    analogWrite(ENA,intial_speed_LEFT - steer);
//    analogWrite(ENB,intial_speed_RIGHT + steer);
//}
//
void reverse()
{
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    digitalWrite(IN3,0);
    digitalWrite(IN4,1);
    steer=compute();
    analogWrite(ENA,intial_speed_LEFT +steer);
    analogWrite(ENB,intial_speed_RIGHT - steer);

}

void change()
{
  if(steer>223)
    steer = 223;
   else if(steer<-223)
    steer = -223; 
}

void stopaf()
{
    digitalWrite(IN1,0);
    digitalWrite(IN2,0);
    digitalWrite(IN3,0);
    digitalWrite(IN4,0);

}

void printaf()
{
   Serial.println("currentyaw : ");
   Serial.println(currentyaw);
   Serial.println("steer");
   Serial.println(steer);
   Serial.println("");
   Serial.println(steer);
   
}
