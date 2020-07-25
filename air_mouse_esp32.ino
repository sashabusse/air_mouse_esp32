#include "BLEMouse.h"
#include "MPU6050_mod.h"
#include "GYRO.h"

//MPU6050_MOD mpu;
GYRO gyro;
BLEMouse mouse;


void setup()
{
  Serial.begin(115200);
  
  mouse.init();
  gyro.init();
  
  /*while(!mpu.begin(MPU6050_SCALE_1000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("not connected!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(2);*/
}
 
void loop()
{
  mouse.start_advertise();
  Serial.println("start advertise");

  while(!mouse.is_connected());
  Serial.println("connected");

  gyro.set_x(0);
  gyro.set_y(0);
  while(mouse.is_connected())
  {
    vTaskDelay(10);

    float dx = gyro.get_x();
    float dy = gyro.get_y();

    signed char mx, my;
    mx = (int)dx;
    mx = min(max((signed char)-126,mx),(signed char)126);
      
    my = (int)dy;
    my = min(max((signed char)-126,my),(signed char)126);

    mouse.move(mx, my);

    gyro.add_x(-mx);
    gyro.add_y(-my);

  }
  
  delay(10);
}



void ble_setup()
{
  Serial.begin(115200);
  mouse.init();
}
void ble_loop()
{
  mouse.start_advertise();
  Serial.println("start advertise");
  
  while(!mouse.is_connected());
  Serial.println("connected");
  
  while(mouse.is_connected())
  {
    unsigned long startTime;

    Serial.println("Move mouse pointer up");
    startTime = millis();
    while(millis()<startTime+2000) {
      mouse.move(0,-1);
      delay(10);
    }
    delay(500);

    Serial.println("Move mouse pointer down");
    startTime = millis();
    while(millis()<startTime+2000) {
      mouse.move(0,1);
      delay(10);
    }
    delay(500);

    Serial.println("Move mouse pointer left");
    startTime = millis();
    while(millis()<startTime+2000) {
      mouse.move(-1,0);
      delay(10);
    }
    delay(500);

    Serial.println("Move mouse pointer right");
    startTime = millis();
    while(millis()<startTime+2000) {
      mouse.move(1,0);
      delay(10);
    }
    delay(500);
  }
}
