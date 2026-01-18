#include <Arduino.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

#define led 2 
// put function declarations here:
unsigned long previousMillis;
int stage[5] ={100, 100,100 ,1000};
int i =0;
unsigned long lastLogTime = 0;
unsigned long printtime = 0;

float finalAngleX = 0;
float setpoint = 0;
float Kp = 1.0; float Ki = 0.05; float Kd = 0.25;
float Ecumul = 0; float Eprec = 0 ; float Espeed = 0;
float P =0 , I =0, D= 0;
float Error=0;

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
  Serial.println("Hello, ESP32!");
  previousMillis= millis();
  lastLogTime = millis();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  finalAngleX = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;;
}

void loop() {
  unsigned long current = millis(); 
  if(current-previousMillis >= stage[i]){
    digitalWrite(led,  !digitalRead(led));
    previousMillis = current;
    i= (i+1)%4;
  }
  if (current - lastLogTime >= 10) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
    float roll = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;

    float dt = (current - lastLogTime)/1000.0;
    float angle_gyroX = g.gyro.x * 180/M_PI *dt;

    finalAngleX = 0.98 *(finalAngleX + angle_gyroX) + 0.02 *(roll);

    Error = setpoint-finalAngleX;
    P = Kp*Error;

    Ecumul += Error * dt;
    if (Ecumul > 100){Ecumul =100;}
    if (Ecumul < -100){Ecumul = -100;}
    I = Ki*Ecumul;

    Espeed = (Error - Eprec)/dt;
    D= Kd*Espeed;

    if (current - printtime >= 1000) {
      Serial.printf("final angle X : %.2f° PID: %.2f\n", finalAngleX,P+I+D); 
      printtime = current;
    }
    lastLogTime = current;
    Eprec = Error;
  }
}

