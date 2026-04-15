#include "config.h"
#include <Arduino.h>
#include "At_Config.hpp"

// Receive command data buffer
uint8_t cmd_data_buf[244] = { 0 };

// Receive command data size
uint8_t cmd_data_size = 0;

// static const uint32_t rfswitch_dio_pins[] = {
//   RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
//   RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
// };

// static const Module::RfSwitchMode_t rfswitch_table[] = {
//   // mode                  DIO5  DIO6
//   { LR11x0::MODE_STBY,   { LOW,  LOW  } },
//   { LR11x0::MODE_RX,     { HIGH, LOW  } },
//   { LR11x0::MODE_TX,     { HIGH, HIGH } },
//   { LR11x0::MODE_TX_HP,  { LOW,  HIGH } },
//   { LR11x0::MODE_TX_HF,  { LOW,  LOW  } },
//   { LR11x0::MODE_GNSS,   { LOW,  LOW  } },
//   { LR11x0::MODE_WIFI,   { LOW,  LOW  } },
//   END_OF_MODE_TABLE,
// };


void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;           // Wait for serial to be initialised
  delay(2000);  // Give time to switch to the serial monitor
  SPI.setPins(47, 45, 46);
  SPI.begin();
  at_config.begin();
  int16_t state = 0;  // return value for calls to RadioLib
  state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, false);
  // radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);
  // radio.setCurrentLimit(140.0);
  radio.setTCXO(3.3);
  Serial.println(F("\nAT Begin"));
}

void loop() {
  if (at_config.receiveSerialCmd(cmd_data_buf, &cmd_data_size)) {
    // Parsing command
    at_config.parseCmd((char *)cmd_data_buf, cmd_data_size);

    // Clear buffer & size
    memset(cmd_data_buf, 0, cmd_data_size);
    cmd_data_size = 0;
  }
  ParameterSetting();
}

void ParameterSetting() {
  if (ABP_Join_Flag == 1) {
    ABP_Join_Flag = 0;
    Serial.println("");
    Serial.println("Join ABP");
    int16_t state = 0;
    uint8_t joinDR = 0;
    //LoRaWAN v1.1
    //node.beginABP(app_param.lora_info.JoinEui, app_param.lora_info.FnwksKey, app_param.lora_info.SnwksKey, app_param.lora_info.NwkSKey, app_param.lora_info.AppSKey);
    //LoRaWAN v1.0.3
    node.beginABP(app_param.lora_info.DevAddr, NULL, NULL, app_param.lora_info.NwkSKey, app_param.lora_info.AppSKey);
    node.activateABP(joinDR);
    debug(state != RADIOLIB_ERR_NONE, F("Activate ABP failed"), state, false);
  }
  if (OTAA_Join_Flag == 1) {
    OTAA_Join_Flag = 0;
    Serial.println("");
    Serial.println("Join OTAA");
    int16_t state = 0;

    uint8_t joinDR = 0;
    if (app_param.lora_info.ActiveRegion == 0) {
      joinDR = 2;
    } else {
      joinDR = 0;
    }

    //LoRaWAN v1.1
    //node.beginOTAA(app_param.lora_info.JoinEui, app_param.lora_info.DevEui, app_param.lora_info.NwkKey, app_param.lora_info.AppKey);
    //LoRaWAN v1.0.3
    node.beginOTAA(app_param.lora_info.JoinEui, app_param.lora_info.DevEui, NULL, app_param.lora_info.AppKey);
    Serial.println("Join ('login') the LoRaWAN Network");
    state = node.activateOTAA(joinDR);
    debug(state != RADIOLIB_LORAWAN_NEW_SESSION, F("Join failed"), state, false);
    Serial.print("[LoRaWAN] DevAddr: ");
    Serial.println((unsigned long)node.getDevAddr(), HEX);
    if (app_param.lora_info.ADR == 1) {
      node.setADR(true);
    } else {
      node.setADR(false);
    }
    node.setDatarate(app_param.lora_info.DR);
    if (app_param.lora_info.dutyCycleEnabled == 1) {
      node.setDutyCycle(true, app_param.lora_info.msPerHour);
    } else {
      node.setDutyCycle(false, app_param.lora_info.msPerHour);
    }
    node.setDwellTime(true, 400);
  }

  if (Send_Flag == 1) {
    Send_Flag = 0;
    if (app_param.lora_info.ActivationType == 1)  //ABP
    {
      Serial.println("Send ABP OK");
      int state = node.sendReceive(MsgTemp, sizeof(MsgTemp));
      debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);

      // Check if a downlink was received
      // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
      if (state > 0) {
        Serial.println(F("Received a downlink"));
      } else {
        Serial.println(F("No downlink received"));
      }
    }
    if (app_param.lora_info.ActivationType == 2)  //OTAA
    {
      int16_t state = RADIOLIB_ERR_NONE;
      Serial.println("Send OTAA OK");
      uint8_t downlinkPayload[10];
      size_t downlinkSize;
      LoRaWANEvent_t uplinkDetails;
      LoRaWANEvent_t downlinkDetails;
      // uint8_t data[] = {0x1, 0x2, 0x3, 0x4};
      if (Ack_State == 1) {
        Serial.println(F("and requesting LinkCheck and DeviceTime"));
        node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_LINK_CHECK);
        node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);
        // state = node.sendReceive(MsgTemp, Port_Number, downlinkPayload, &downlinkSize, true, &uplinkDetails, &downlinkDetails);

         state = node.sendReceive(MsgTemp, MsgTemp_size -1, Port_Number, downlinkPayload, &downlinkSize, true, &uplinkDetails, &downlinkDetails);
      } else {
        // state = node.sendReceive(MsgTemp, Port_Number, downlinkPayload, &downlinkSize, false, &uplinkDetails, &downlinkDetails);

        state = node.sendReceive(MsgTemp, MsgTemp_size -1, Port_Number, downlinkPayload, &downlinkSize, true, &uplinkDetails, &downlinkDetails);
      }
      debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);
      if (state > 0) {
        Serial.println(F("Received a downlink"));
        if (downlinkSize > 0) {
          Serial.println(F("Downlink data: "));
          arrayDump(downlinkPayload, downlinkSize);
        } else {
          Serial.println(F("<MAC commands only>"));
        }
      } else {
        Serial.println(F("[LoRaWAN] No downlink received"));
      }
    }
  }
  if (ADR_set_Flag == 1) {
    ADR_set_Flag = 0;
    if (app_param.lora_info.ADR == 1) {
      node.setADR(true);
    } else {
      node.setADR(false);
    }
  }
  if (DR_set_Flag == 1) {
    DR_set_Flag = 0;
    node.setDatarate(app_param.lora_info.DR);
  }
  if (DCS_set_Flag == 1) {
    DCS_set_Flag = 0;
    if (app_param.lora_info.dutyCycleEnabled) {
      node.setDutyCycle(true, app_param.lora_info.msPerHour);
    } else {
      node.setDutyCycle(false, app_param.lora_info.msPerHour);
    }
  }
  if (TXP_set_Flag == 1) {
    TXP_set_Flag = 0;
    node.setTxPower(app_param.lora_info.txPower);
  }
  if (RX2DR_set_Flag == 1) {
    RX2DR_set_Flag = 0;
    node.setRx2Dr(app_param.lora_info.rx2dr);
  }
  if (Band_set_Flag == 1) {
    Band_set_Flag = 0;
    if (app_param.lora_info.ActiveRegion == 0) {
      Serial.println(F("Set Regional Choices EU868"));
      node = LoRaWANNode(&radio, &Region_868, subBand);
    } else if (app_param.lora_info.ActiveRegion == 1) {
      Serial.println(F("Set Regional Choices US915"));
      node = LoRaWANNode(&radio, &Region_915, subBand);
    }
  }
}
