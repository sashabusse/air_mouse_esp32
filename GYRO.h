#include "MPU6050_mod.h"


class GYRO
{
  public:
    GYRO();
    ~GYRO();

    void init();
    static void work_task(void* param);

    void set_x(float new_val);
    void add_x(float val);
    float get_x();

    void set_y(float new_val);
    void add_y(float val);
    float get_y();
    
    
  private:
  public:
    MPU6050_MOD m_mpu;
    
  private:

    xSemaphoreHandle m_x_mutex;
    xSemaphoreHandle m_y_mutex;
    float m_x, m_y;
};
