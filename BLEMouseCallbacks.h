#pragma once
#include "sdkconfig.h"
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLECharacteristic.h"


class BLEMouseCallbacks : public BLEServerCallbacks
{
  public:
    BLEMouseCallbacks(void);
    
    void onConnect(BLEServer* pServer);
    void onDisconnect(BLEServer* pServer);

    bool has_disconnected();
    void clear_disconnected();
    bool is_connected();

    void set_input_report_ch(BLECharacteristic* ch);
  private:
    bool m_connected = false;
    bool m_disconnected = false;
    BLECharacteristic* m_input_report_ch;
};
