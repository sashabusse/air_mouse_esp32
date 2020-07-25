#include "sdkconfig.h"
#include "BLEMouseCallbacks.h"
#include "Arduino.h"
#include "BLEMouse.h"

BLEMouseCallbacks::BLEMouseCallbacks(void) 
{
}

void BLEMouseCallbacks::onConnect(BLEServer* pServer)
{
  Serial.println("connect start");
  m_connected = true;
  BLE2902* desc = (BLE2902*)m_input_report_ch->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  desc->setNotifications(true);
  Serial.println("connect end");
}

void BLEMouseCallbacks::onDisconnect(BLEServer* pServer)
{
  Serial.println("disconnect start");
  m_connected = false;
  m_disconnected = false;
  BLE2902* desc = (BLE2902*)m_input_report_ch->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  desc->setNotifications(false);
  Serial.println("disconnect end");
}

bool BLEMouseCallbacks::is_connected()
{
  return m_connected;
}


void BLEMouseCallbacks::set_input_report_ch(BLECharacteristic* ch)
{
  m_input_report_ch = ch;
}


bool BLEMouseCallbacks::has_disconnected()
{
  return m_disconnected;
}

void BLEMouseCallbacks::clear_disconnected()
{
  m_disconnected = false;
}
