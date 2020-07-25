#include "GYRO.h"
#include "Arduino.h"

GYRO::GYRO():
  m_x(0),
  m_y(0)
{
  
}

GYRO::~GYRO()
{
}

void GYRO::init()
{
  while(!m_mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_16G))
  {
    Serial.println("not connected!");
    delay(500);
  }
  m_mpu.calibrateGyro();
  m_mpu.setThreshold(0.1);

  m_x_mutex = xSemaphoreCreateMutex();
  m_y_mutex = xSemaphoreCreateMutex();
  
  xTaskCreate(work_task, "gyro_work_task", 2048, (void *)this, 5, NULL);
}

void GYRO::work_task(void* param)
{
  GYRO* inst = (GYRO*)param;

  while(true)
  {
    for(int i = 0;i<100;i++)
    {
      Vector normGyro = inst->m_mpu.readNormalizeGyro();
      
      inst->add_x(-normGyro.ZAxis/50.0);
      inst->add_y(normGyro.YAxis/50.0);
    }
    
    vTaskDelay(10);
  }
  
}


void GYRO::set_x(float new_val)
{
  if( xSemaphoreTake( m_x_mutex, portMAX_DELAY ) == pdTRUE )
  {

    m_x = new_val;
       
    xSemaphoreGive( m_x_mutex );
  }
  else
  {
    Serial.println("failed to get mutex");
    while(true);
  }
}

void GYRO::add_x(float val)
{
  if( xSemaphoreTake( m_x_mutex, portMAX_DELAY ) == pdTRUE )
  {

    m_x += val;
       
    xSemaphoreGive( m_x_mutex );
  }
  else
  {
    Serial.println("failed to get mutex");
    while(true);
  }
}

float GYRO::get_x()
{
  float tmp;
  if( xSemaphoreTake( m_x_mutex, portMAX_DELAY ) == pdTRUE )
  {

    tmp = m_x;
       
    xSemaphoreGive( m_x_mutex );
  }
  else
  {
    Serial.println("failed to get mutex");
    while(true);
  }
  return tmp;
}


void GYRO::set_y(float new_val)
{
  if( xSemaphoreTake( m_y_mutex, portMAX_DELAY ) == pdTRUE )
  {

    m_y = new_val;
       
    xSemaphoreGive( m_y_mutex );
  }
  else
  {
    Serial.println("failed to get mutex");
    while(true);
  }
}

void GYRO::add_y(float val)
{
  if( xSemaphoreTake( m_y_mutex, portMAX_DELAY ) == pdTRUE )
  {

    m_y += val;
       
    xSemaphoreGive( m_y_mutex );
  }
  else
  {
    Serial.println("failed to get mutex");
    while(true);
  }
}

float GYRO::get_y()
{
  float tmp;
  if( xSemaphoreTake( m_y_mutex, portMAX_DELAY ) == pdTRUE )
  {

    tmp = m_y;
       
    xSemaphoreGive( m_y_mutex );
  }
  else
  {
    Serial.println("failed to get mutex");
    while(true);
  }
  return tmp;
}
