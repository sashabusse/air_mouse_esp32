#include "HIDService.h"


HIDService::HIDService(BLEServer* server):
  m_server(server),
  m_hid_service(NULL),
  m_hid_info_ch(NULL),
  m_hid_control_ch(NULL),
  m_report_map_ch(NULL),
  m_protocol_mode_ch(NULL)
{
}

HIDService::~HIDService() 
{
}

void HIDService::init(uint8_t country, uint8_t flags, uint8_t mode)
{
  m_hid_service = m_server->createService(BLE_HID_SERVICE_UUID, 40);


  m_hid_info_ch = m_hid_service->createCharacteristic(HID_INFO_CH_UUID, BLECharacteristic::PROPERTY_READ);
  set_hid_info(country,flags);
  
  m_hid_control_ch = m_hid_service->createCharacteristic(HID_CONTROL_CH_UUID, BLECharacteristic::PROPERTY_WRITE_NR);
  
  m_report_map_ch = m_hid_service->createCharacteristic(HID_REPORT_MAP_UUID, BLECharacteristic::PROPERTY_READ);
  
  m_protocol_mode_ch = m_hid_service->createCharacteristic(HID_PROTOCOL_MOD_UUID, BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_READ);
  uint8_t pMode[] = { mode };
  m_protocol_mode_ch->setValue((uint8_t*) pMode, 1);
}

void HIDService::start_service()
{
  m_hid_service->start();
}

void HIDService::set_report_map(uint8_t* p_val, uint16_t sz)
{
  m_report_map_ch->setValue(p_val, sz);
}


BLECharacteristic* HIDService::create_input_report_ch(uint8_t reportID) 
{
  BLECharacteristic* input_report_ch = m_hid_service->createCharacteristic((uint16_t) 0x2a4d, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  
  BLEDescriptor* input_report_desc = new BLEDescriptor(BLEUUID((uint16_t) 0x2908));
  BLE2902* p2902 = new BLE2902();
  
  input_report_ch->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  input_report_desc->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  p2902->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);

  uint8_t desc1_val[] = { reportID, 0x01 };
  input_report_desc->setValue((uint8_t*) desc1_val, 2);
  
  input_report_ch->addDescriptor(p2902);
  input_report_ch->addDescriptor(input_report_desc);

  return input_report_ch;
}

BLEService* HIDService::get_service()
{
  return m_hid_service;
}

void HIDService::set_hid_info(uint8_t country, uint8_t flags) {
  uint8_t info[] = { 0x11, 0x1, country, flags };
  m_hid_info_ch->setValue(info, sizeof(info));
}
