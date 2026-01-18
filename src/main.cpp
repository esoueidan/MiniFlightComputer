#include <Arduino.h>
#include <Adafruit_MPU6050.h>

#define led 2 
Adafruit_MPU6050 mpu;

struct PID {
    float Kp, Ki, Kd;
    float cumul;
    float prec;
    float speed;
    float PID;
    char axe;
};
typedef struct {
    float x,y,z;
} SETPOINT;


// function declarations:
void calcPID(float,float, PID&,float);


//global variable:
unsigned long previousMillis;
int stage[5] ={100, 100,100 ,1000};
int i =0;
unsigned long lastLogTime = 0;
unsigned long printtime = 0;
float finalAngleX = 0;
float finalAngleY = 0;

float Error=0;
float throttle = 1500.0;
PID rollPID = {1.0, 0.05, 0.25, 0, 0, 0, 0,'x'};
PID pitchPID = {1.0, 0.05, 0.25, 0, 0, 0, 0,'y'};
SETPOINT setpoint = {10,0,0};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(led,OUTPUT);
  digitalWrite(led, LOW);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  previousMillis= millis();
  lastLogTime = millis();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  finalAngleX = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  finalAngleY = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  Serial.println("Hello, ESP32!");
}

void loop() {
  unsigned long current = millis(); 
  if(current-previousMillis >= stage[i]){
    digitalWrite(led,  !digitalRead(led));
    previousMillis = current;
    i= (i+1)%4;
  }
  if (current - lastLogTime >= 10) {
    float dt = (current - lastLogTime)/1000.0;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
    float roll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;

    finalAngleX = 0.98 *(finalAngleX + (g.gyro.x * 180/M_PI *dt)) + 0.02 *(roll);
    finalAngleY = 0.98 * (finalAngleY + (g.gyro.y * 180/M_PI *dt)) +0.02 * (pitch);

    calcPID(finalAngleX, dt, rollPID, setpoint.x);
    calcPID(finalAngleY, dt, pitchPID, setpoint.y);
    float FLmotor = constrain( throttle + rollPID.PID -pitchPID.PID ,1000,2000);
    float FRmotor = constrain( throttle - rollPID.PID -pitchPID.PID  ,1000,2000);
    float RLmotor = constrain( throttle + rollPID.PID +pitchPID.PID ,1000,2000);
    float RRmotor = constrain( throttle - rollPID.PID +pitchPID.PID ,1000,2000);

    if (current - printtime >= 1000) {
      //Serial.printf("final angle X : %.2f° PID: %.2f Left motor: %.2f Right motor: %.2f \n", finalAngleX,P+I+D,motorGauche,motorDroit); 
      Serial.print("Angle:");
      Serial.print(finalAngleX);
      Serial.print("\t");
      Serial.print("MoteurFL:");
      Serial.print(FLmotor);
      Serial.print("\t");
      Serial.print("MoteurFR:");
      Serial.print(FRmotor);
      Serial.print("\t");
      Serial.print("MoteurRL:");
      Serial.print(RLmotor);
      Serial.print("\t");
      Serial.print("MoteurRR:");
      Serial.println(RRmotor);
      printtime = current;
    }
    lastLogTime = current;
  }
}

void calcPID(float finalAngle, float dt, PID &tmp, float setpoint){
    Error = setpoint - finalAngle;
    float P = tmp.Kp*Error;

    tmp.cumul += Error * dt;
    if (tmp.cumul > 100){tmp.cumul =100;}
    if (tmp.cumul < -100){tmp.cumul = -100;}
    float I = tmp.Ki*tmp.cumul;

    tmp.speed = (Error - tmp.prec)/dt;
    float D= tmp.Kd*tmp.speed;
    tmp.PID = P+I+D;
    tmp.prec = Error;
}
