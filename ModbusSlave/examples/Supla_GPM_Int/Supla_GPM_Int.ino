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

#include <supla/sensor/general_purpose_measurement.h>

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
#define SLAVE_ADDRESS_SIZE 10
#define SLAVE_TIMEOUT    5000

#define VARIABLE_0          0
#define VARIABLE_1          1
#define VARIABLE_2          2
#define VARIABLE_3          3
#define VARIABLE_4          4
#define VARIABLE_5          5
#define VARIABLE_6          6
#define VARIABLE_7          7
#define VARIABLE_8          8
#define VARIABLE_9          9

uint16_t slaveTable[SLAVE_ADDRESS_SIZE];

ModbusSlave Slave(&Serial, SLAVE_BAUD, SLAVE_ID, SLAVE_ADDRESS_START, slaveTable, SLAVE_ADDRESS_SIZE, SLAVE_TIMEOUT);

auto suplaGpm0 = new Supla::Sensor::GeneralPurposeMeasurement();
auto suplaGpm1 = new Supla::Sensor::GeneralPurposeMeasurement();
auto suplaGpm2 = new Supla::Sensor::GeneralPurposeMeasurement();
auto suplaGpm3 = new Supla::Sensor::GeneralPurposeMeasurement();
auto suplaGpm4 = new Supla::Sensor::GeneralPurposeMeasurement();
auto suplaGpm5 = new Supla::Sensor::GeneralPurposeMeasurement();
auto suplaGpm6 = new Supla::Sensor::GeneralPurposeMeasurement();
auto suplaGpm7 = new Supla::Sensor::GeneralPurposeMeasurement();
auto suplaGpm8 = new Supla::Sensor::GeneralPurposeMeasurement();
auto suplaGpm9 = new Supla::Sensor::GeneralPurposeMeasurement();

void setup() 
{
  suplaButtonCfg->configureAsConfigButton(&SuplaDevice);
	
  new Supla::Html::DeviceInfo(&SuplaDevice);
  new Supla::Html::WifiParameters;
  new Supla::Html::ProtocolParameters;

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

      suplaGpm0->setValue(slaveTable[VARIABLE_0]);
      suplaGpm1->setValue(slaveTable[VARIABLE_1]);
      suplaGpm2->setValue(slaveTable[VARIABLE_2]);
      suplaGpm3->setValue(slaveTable[VARIABLE_3]);
      suplaGpm4->setValue(slaveTable[VARIABLE_4]);
      suplaGpm5->setValue(slaveTable[VARIABLE_5]);
      suplaGpm6->setValue(slaveTable[VARIABLE_6]);
      suplaGpm7->setValue(slaveTable[VARIABLE_7]);
      suplaGpm8->setValue(slaveTable[VARIABLE_8]);
      suplaGpm9->setValue(slaveTable[VARIABLE_9]);
    }
  }
}