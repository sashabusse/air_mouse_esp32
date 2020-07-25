#pragma once
#include "HIDService.h"
#include "BLEMouseCallbacks.h"


class BLEMouse
{
  public:
    BLEMouse();
    ~BLEMouse();

    void init();
    void start_advertise();

    void move(signed char x, signed char y);

    bool is_connected();
  private:
  public:
  private:
    BLEMouseCallbacks* m_callbacks;
  
    HIDService* m_hid_service;
    BLEServer* m_server;
    BLEAdvertising* m_advertising;

    BLECharacteristic* m_input_report_ch;
    uint8_t m_buttons;
};
