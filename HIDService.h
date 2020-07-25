#pragma once
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>

#define BLE_HID_SERVICE_UUID (BLEUUID((uint16_t) 0x1812))

#define HID_INFO_CH_UUID ((uint16_t) 0x2a4a)
#define HID_CONTROL_CH_UUID ((uint16_t) 0x2a4c)
#define HID_REPORT_MAP_UUID ((uint16_t) 0x2a4b)
#define HID_PROTOCOL_MOD_UUID ((uint16_t) 0x2a4e)

#define HID_MOUSE 0x03C2

class HIDService
{
  public:
    HIDService(BLEServer* server);
    ~HIDService();

    void init(uint8_t country = 0x00, uint8_t flags = 0x02, uint8_t mode = 0x01);

    void start_service();

    void set_report_map(uint8_t* p_val, uint16_t sz);
    void set_hid_info(uint8_t country, uint8_t flags);

    BLECharacteristic* create_input_report_ch(uint8_t reportID);

    BLEService* get_service();
  private:
  public:
    
  private:
    BLEServer* m_server;

    BLEService* m_hid_service;

    BLECharacteristic* m_hid_info_ch; //Mandatory
    BLECharacteristic* m_hid_control_ch; //Mandatory
    BLECharacteristic* m_report_map_ch; //Mandatory
    BLECharacteristic* m_protocol_mode_ch; //C4
};
