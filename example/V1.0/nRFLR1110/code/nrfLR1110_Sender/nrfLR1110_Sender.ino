#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <DHT20.h>
#include <Adafruit_TinyUSB.h>  // for Serial
#include <RadioLib.h>
#include <IRremote.hpp>

TwoWire Wire1(NRF_TWIM1, 
              NRF_TWIS1, 
              SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, 
              /* data=*/29, 
              /* clock=*/28);

// LSM6DS3TR register addresses
#define LSM6DS3TR_ADDR      0x6B        // Default I2C address for LSM6DS3TR
#define CTRL1_XL            0x10        // Accelerometer control register
#define CTRL2_G             0x11        // Gyroscope control register
#define OUTX_L_XL           0x28        // Accelerometer X-axis data (low byte)
#define ACCEL_SENSITIVITY   0.000061    // Accelerometer sensitivity factor for 4g range (g -> m/s^2)

// Display Setup
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, 
                                        /* clock=*/4, 
                                        /* data=*/5, 
                                        /* reset=*/U8X8_PIN_NONE);  // All Boards without Reset of the Display

DHT20 dht20;
// Sensor variables
float temperature = 0;
float humidity = 0;

// Transmission state
int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false;

// IR Receiver setup
const uint8_t IR_RECEIVER_PIN = 20;           // Recommend GPIO4 (supporting PWM pins)

// Constants for button and LED control
const int buttonPin = 12;                     // the number of the pushbutton pin
const int ledPin    = 26;                     // the number of the LED pin

volatile bool ledState = true;                // LED state flag to handle button press
volatile bool buttonPressedFlag = false;

unsigned long lastButtonPressTime = 0;
unsigned long debounceDelay = 300;            // 300ms debounce delay

unsigned long previousMillis = 0;             // Record the time of the last task execution
const long interval = 1000;                   // Set interval time (1 second)

// Record the last pressed IR key value 
// to prevent continuous firing from long press
uint16_t lastIRCommand = 0xFFFF; 

// Accelerometer functions
void writeRegister(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(LSM6DS3TR_ADDR);
  Wire1.write(reg);
  Wire1.write(value);
  Wire1.endTransmission();
}

void readRegister(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire1.beginTransmission(LSM6DS3TR_ADDR);
  Wire1.write(reg);
  Wire1.endTransmission(false);
  Wire1.requestFrom(LSM6DS3TR_ADDR, length);
  for (int i = 0; i < length; i++) {
    data[i] = Wire1.read();
  }
}

// LR1110 Radio Setup
LR1110 radio = new Module(44, 40, 42, 43);

// New: Define the sensor data structure
struct SensorData {
  float temperature;
  float humidity;
  float accelX;
  float accelY;
  float accelZ;
};

static const uint32_t rfswitch_dio_pins[] = {
  RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
  RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  { LR11x0::MODE_STBY,  { LOW , LOW   } },
  { LR11x0::MODE_RX,    { HIGH, LOW   } },
  { LR11x0::MODE_TX,    { HIGH, HIGH  } },
  { LR11x0::MODE_TX_HP, { LOW , HIGH  } },
  { LR11x0::MODE_TX_HF, { LOW , LOW   } },
  { LR11x0::MODE_GNSS,  { LOW , LOW   } },
  { LR11x0::MODE_WIFI,  { LOW , LOW   } },
  END_OF_MODE_TABLE,
};

// ================= Interrupts and Handler Functions =================
void setFlag(void) {
  transmittedFlag = true;
}

void toggleLED() {
  buttonPressedFlag = true; // Only set the flag inside the interrupt
}

void handleIRCommand() {
  // Check if it is a repeat code (continuous firing from long press), if so, discard it directly
  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
    IrReceiver.resume();
    return;
  }

  uint16_t currentCommand = IrReceiver.decodedIRData.command;

  // If the currently pressed key is the same as the last pressed key, it means it is a continuous firing from long press, discard it without processing!
  if (currentCommand == lastIRCommand) {
    IrReceiver.resume();
    return;
  }
  
  // If execution reaches here, it is a brand new key press (or a different key is pressed)
  lastIRCommand = currentCommand;

  Serial.println("====== Detected Infrared Signal ======");
  Serial.print("- press -\t");
  switch (IrReceiver.decodedIRData.command) {
    case 0x45: Serial.println("[CH-]");         break;
    case 0x46: Serial.println("[CH]");          break;
    case 0x47: Serial.println("[CH+]");         break;
    case 0x44: Serial.println("[PREV]");        break;
    case 0x40: Serial.println("[NEXT]");        break;
    case 0x43: Serial.println("[PLAY/PAUSE]");  break;
    case 0x07: Serial.println("[VOL-]");        break;
    case 0x15: Serial.println("[VOL+]");        break;
    case 0x09: Serial.println("[EQ]");          break;
    case 0x16: Serial.println("[0]");           break;
    case 0x19: Serial.println("[100+]");        break;
    case 0xD:  Serial.println("[200+]");        break;
    case 0xC:  Serial.println("[1]");           break;
    case 0x18: Serial.println("[2]");           break;
    case 0x5E: Serial.println("[3]");           break;
    case 0x8:  Serial.println("[4]");           break;
    case 0x1C: Serial.println("[5]");           break;
    case 0x5A: Serial.println("[6]");           break;
    case 0x42: Serial.println("[7]");           break;
    case 0x52: Serial.println("[8]");           break;
    case 0x4A: Serial.println("[9]");           break;

    case 0x00:                                  break; 
    default:   Serial.println("[UNKNOWN]");     break;
  }
  IrReceiver.resume();
}

void setup() {
  // Initialize the display
  u8g2.begin();

  Serial1.setPins(22, 24);
  // Initialize serial communication
  Serial.begin(9600);
  Serial1.begin(9600);  // Initialize Serial1 for communication

  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVER_PIN, DISABLE_LED_FEEDBACK); 

  // Initialize DHT20 sensor
  Wire.setPins(13, 14);
  Wire.begin();
  if (!dht20.begin()) {  
      Serial.println("Initialize DHT20 sensor failed");
  } else {
      Serial.println("DHT20 Initialized!");
  }
  
  // Initialize I2C1 (28,29) for the accelerometer
  Wire1.begin();
  // Configure LSM6DS3TR-C accelerometer: 0x40 means 104Hz sampling rate, 4g range, low power disabled
  writeRegister(CTRL1_XL, 0x40); 

  // Initialize button and LED pins
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); 
  // Attach interrupt for button press (RISING means button pressed)
  attachInterrupt(digitalPinToInterrupt(buttonPin), toggleLED, FALLING);

  // Initialize LR1110 Radio
  SPI.setPins(47, 45, 46);
  SPI.begin();
  int state = radio.begin(915.0, 125.0, 7, 7, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, 22, 8, 3.3);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio Initialized!"));
  } else {
    Serial.print(F("Radio Initialization Failed, code "));
    Serial.println(state);
    while (true) { delay(2000); }
  }
  radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);
  radio.setPacketSentAction(setFlag);

  // Start transmission of the first packet
  transmissionState = radio.startTransmit("Hello World!");
}

void loop() {
  // ==========================================
  // 1. Instant response: Handle button press (placed first to ensure ultra-fast response)
  // ==========================================
  if (buttonPressedFlag) {
    buttonPressedFlag = false; 
    unsigned long currentMillis = millis();
    if (currentMillis - lastButtonPressTime > debounceDelay) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState ? HIGH : LOW); 
      lastButtonPressTime = currentMillis;         
    }
  }

  // ==========================================
  // 2. Tasks requiring real-time processing: IR, Serial (cannot be blocked)
  // ==========================================
  if (IrReceiver.decode()) {
    handleIRCommand();
  }

  if (Serial1.available()) {
    while (Serial1.available()) {
      char incomingByte = Serial1.read();
      Serial.print(incomingByte);
    }
  }

  if (Serial.available()) {
    Serial1.write(Serial.read());
  }

  // ==========================================
  // 3. Radio state handling (asynchronous, cannot be blocked)
  // ==========================================
  if (transmittedFlag) {
    transmittedFlag = false;
    if (transmissionState == RADIOLIB_ERR_NONE) {
      Serial.println(F("Transmission finished!"));
    } else {
      Serial.print(F("Transmission failed, code "));
      Serial.println(transmissionState);
    }
    radio.finishTransmit();
  }

  // ==========================================
  // 4. Low-frequency task: Sensor reading and sending executed every 1 second
  // ==========================================
  unsigned long currentMillis = millis();
  
  // If the time difference reaches 1000 milliseconds
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Reset the timer

    // --- The code below will only be executed once every 1 second ---
    SensorData data;
    data.temperature = dht20.getTemperature();
    data.humidity = dht20.getHumidity() * 100;

    // Read accelerometer
    uint8_t accelData[6];
    // writeRegister(CTRL1_XL, 0x40);
    readRegister(OUTX_L_XL, accelData, 6);
    data.accelX = (int16_t)(accelData[0] | (accelData[1] << 8)) * ACCEL_SENSITIVITY * 9.80;
    data.accelY = (int16_t)(accelData[2] | (accelData[3] << 8)) * ACCEL_SENSITIVITY * 9.80;
    data.accelZ = (int16_t)(accelData[4] | (accelData[5] << 8)) * ACCEL_SENSITIVITY * 9.80;

    // Serial output
    Serial.print("Temperature: ");
    Serial.print(data.temperature, 1);
    Serial.print(" \xC2\xB0" "C");    
    Serial.print("  Humidity: ");
    Serial.print(data.humidity, 1);    
    Serial.println(" %RH");
    
    Serial.print("\t\tAccel X: ");  Serial.print(data.accelX, 2);
    Serial.print(" \tY: ");         Serial.print(data.accelY, 2);
    Serial.print(" \tZ: ");         Serial.print(data.accelZ, 2);
    Serial.println(" m/s^2 ");

    // --- Display screen update ---
    u8g2.setFont(u8g2_font_6x13_tf);
    
    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 20);
      u8g2.print("Temp: ");
      u8g2.print(data.temperature, 1);
      u8g2.print(" \xb0" "C");  // ℃

      u8g2.setCursor(0, 40);
      u8g2.print("Humi: ");
      u8g2.print(data.humidity , 1);
      u8g2.print(" %");

      // Acceleration display on the same line
      u8g2.setCursor(0, 60); // Placed on the bottom line
      u8g2.print("X:");   u8g2.print(data.accelX, 1);
      u8g2.print(" Y:");  u8g2.print(data.accelY, 1);
      u8g2.print(" Z:");  u8g2.print(data.accelZ, 1);
      
    } while (u8g2.nextPage());

    // Assemble data and send (The data sent over the air still retains 2 decimal places to ensure accuracy)
    String payload =  "Temp:"   + String(data.temperature, 1) + 
                      ",Humi:"  + String(data.humidity, 1)    + 
                      ",X:"     + String(data.accelX, 2)      + 
                      ",Y:"     + String(data.accelY, 2)      + 
                      ",Z:"     + String(data.accelZ, 2)      ;
      
    transmissionState = radio.startTransmit(payload);
    Serial.println("Sent: " + payload);
  }
}
