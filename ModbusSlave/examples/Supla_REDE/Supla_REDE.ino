#include <SuplaDevice.h>
#include <supla/control/button.h>
#include <supla/device/status_led.h>
#include <supla/device/supla_ca_cert.h>
#include <supla/network/esp_web_server.h>
#include <supla/network/esp_wifi.h>
#include <supla/network/html/device_info.h>
#include <supla/network/html/protocol_parameters.h>
#include <supla/network/html/wifi_parameters.h>
#include <supla/storage/littlefs_config.h>

#include <supla/clock/clock.h>
#include <supla/network/html/time_parameters.h>

#include <ModbusSlave.h>

#define PINOUT_LED   13
#define PINOUT_BUTTON 0

Supla::Device::StatusLed statusLed(PINOUT_LED, true);
Supla::ESPWifi wifi;
Supla::EspWebServer suplaServer;
Supla::LittleFsConfig configSupla;

auto suplaButtonCfg = new Supla::Control::Button(PINOUT_BUTTON, true, true);

#define SLAVE_BAUD       9600
#define SLAVE_ID            1
#define SLAVE_ADDRESS_START 0
#define SLAVE_ADDRESS_SIZE  5
#define SLAVE_TIMEOUT    5000
#define PINOUT_REDE         3

#define HOUR                0
#define MINUTES             1
#define DAY                 2
#define MONTH               3
#define YEAR                4

uint16_t slaveTable[SLAVE_ADDRESS_SIZE];

ModbusSlave Slave(&Serial, SLAVE_BAUD, SLAVE_ID, SLAVE_ADDRESS_START, slaveTable, SLAVE_ADDRESS_SIZE, SLAVE_TIMEOUT);

void setup() 
{
  suplaButtonCfg->configureAsConfigButton(&SuplaDevice);
	
  new Supla::Html::DeviceInfo(&SuplaDevice);
  new Supla::Html::WifiParameters;
  new Supla::Html::ProtocolParameters;
  
  new Supla::Clock;
  new Supla::Html::TimeParameters(&SuplaDevice);

  Slave.REDE(PINOUT_REDE);

  SuplaDevice.setSuplaCACert(suplaCACert);
  SuplaDevice.setSupla3rdPartyCACert(supla3rdCACert);

  SuplaDevice.begin();
}

void loop()
{
  SuplaDevice.iterate();

  if(SuplaDevice.getCurrentStatus() == STATUS_REGISTERED_AND_READY)
  {
    if(Slave.Update() == ALARM_COMMUNICATION)
    {

    }

    static uint32_t lastTime = 0;
    if(millis() - lastTime > 1000) 
    {
      lastTime = millis();

      slaveTable[HOUR] = Supla::Clock::GetHour();
      slaveTable[MINUTES] = Supla::Clock::GetMin();
      slaveTable[DAY] = Supla::Clock::GetDay();
      slaveTable[MONTH] = Supla::Clock::GetMonth();
      slaveTable[YEAR] = Supla::Clock::GetYear();
    }
  }
}