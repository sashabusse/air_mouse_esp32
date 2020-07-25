#include "BLEMouse.h"
#include "Arduino.h"

#include "hid_def.h"
static const uint8_t mouse_hid_report_descriptor[] = {
  HID_USAGE_PAGE (GENERIC_DESKTOP),
  HID_USAGE (MOUSE),
  HID_COLLECTION (APPLICATION),
    HID_USAGE (POINTER),
    HID_COLLECTION (PHYSICAL),
      HID_REPORT_ID(1),
      
      HID_USAGE_PAGE (BUTTONS),
      HID_USAGE_MINIMUM (1, 1),
      HID_USAGE_MAXIMUM (1, 3),
      HID_LOGICAL_MINIMUM (1, 0),
      HID_LOGICAL_MAXIMUM (1, 1),
      HID_REPORT_COUNT (3),
      HID_REPORT_SIZE (1),
      HID_INPUT (DATA, VARIABLE, ABSOLUTE),
      HID_REPORT_COUNT (1),
      HID_REPORT_SIZE (5),
      HID_INPUT (CONSTANT),
      
      HID_USAGE_PAGE (GENERIC_DESKTOP),
      HID_USAGE (X),
      HID_USAGE (Y),
      HID_LOGICAL_MINIMUM (1, -127),
      HID_LOGICAL_MAXIMUM (1, 127),
      HID_REPORT_SIZE (8),
      HID_REPORT_COUNT (2),
      HID_INPUT (DATA, VARIABLE, RELATIVE),
      
    HID_END_COLLECTION (PHYSICAL),
  HID_END_COLLECTION (APPLICATION),
};


BLEMouse::BLEMouse():
  m_server(nullptr),
  m_hid_service(nullptr),
  m_callbacks(new BLEMouseCallbacks()),
  m_buttons(0)
{
}

BLEMouse::~BLEMouse()
{
  if(m_hid_service)
    delete m_hid_service;
  if(m_callbacks)
    delete m_callbacks;
}


void BLEMouse::init()
{
  
  BLEDevice::init("ESP32 Mouse1");
  m_server = BLEDevice::createServer();
  m_server->setCallbacks(m_callbacks);

  m_hid_service = new HIDService(m_server);
  m_hid_service->init();

  m_input_report_ch = m_hid_service->create_input_report_ch(1);
  m_callbacks->set_input_report_ch(m_input_report_ch);

  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

  m_hid_service->set_report_map((uint8_t*)mouse_hid_report_descriptor, sizeof(mouse_hid_report_descriptor));
  m_hid_service->start_service();

  m_advertising = m_server->getAdvertising();
  m_advertising->setAppearance(HID_MOUSE);
  m_advertising->addServiceUUID(m_hid_service->get_service()->getUUID());
}



void BLEMouse::start_advertise()
{
  m_advertising->start();
}



void BLEMouse::move(signed char x, signed char y)
{
  if (is_connected())
  {
    uint8_t m[3];
    m[0] = m_buttons;
    m[1] = x;
    m[2] = y;
    m_input_report_ch->setValue(m, 3);
    m_input_report_ch->notify();
  }
}


bool BLEMouse::is_connected()
{
  return m_callbacks->is_connected();
}
