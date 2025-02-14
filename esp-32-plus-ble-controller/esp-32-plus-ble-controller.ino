#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <FastLED.h> // http://librarymanager/All#FastLED

#include <Wire.h> // Needed for I2C
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library
#include <SparkFun_Qwiic_Button.h>
#include <UUID.h>
#include <Preferences.h>
#include <ArduinoBLE.h>
#include "driver/rtc_io.h"


// Status LED Settings.
#define LED_PIN     46 //Pin 46 on Thing Plus C S3 is connected to WS2812 LED
#define COLOR_ORDER GRB
#define CHIPSET     WS2812
#define NUM_LEDS    1

#define MAX_BRIGHTNESS 60
#define MIN_BRIGHTNESS 10
#define DEFAULT_BRIGHTNESS  25

CRGB leds[NUM_LEDS];

// Init flags

bool ErrorOnInit = false;
// BLE Specific Flags
bool isConnected = false;
bool isScanning = false;

// UUID
UUID uuid;

// Preferences
Preferences preferences;
String device_uuid;


// Timeout to sleep
unsigned long lastInputTime = 0;
#define SLEEP_TIMEOUT_MINUTES 30

// BLE
#define FORCE_BLE_RESET false

const char* remoteControl = "06f3d410-1f9d-4020-b6c0-ad20d295d869";
const char* remoteControlEnabled = "cc33b6ac-e38e-45ab-a5f4-d39b77928ec9";
const char* remoteControlStop = "2f81b369-7d9f-4d5e-92fa-1c00c6651b5d";
const char* remoteControlJoystickX = "a204e8f5-cea7-47e8-89eb-59bcc2ba28d1";
const char* remoteControlJoystickY = "2e55d9fb-094e-4a92-a1ca-fe5055d0c54e";

BLEDevice peripheral;


//Input vars
bool estop = false;
int joystick_deflection_h = 0;
int joystick_deflection_v = 0;

// Button
QwiicButton button;

// ================================================================================
int STATUS_LED_MODE;
#define DEFAULT 0
void StatusLED_Default()
{
    leds[0] = CRGB::Red;
    FastLED.show();
    vTaskDelay(200);

    leds[0] = CRGB::Green;
    FastLED.show();
    vTaskDelay(200);

    leds[0] = CRGB::Blue;
    FastLED.show();
    vTaskDelay(200);

    leds[0] = CRGB::Black;
    FastLED.show();
    vTaskDelay(100);

    leds[0] = CRGB::White;
    FastLED.show();
    vTaskDelay(2000);

    leds[0] = CRGB::Black;
    FastLED.show();
    vTaskDelay(100);
}

#define JOYSTICK_MALFUNCTION 1
void StatusLED_JoystickMalfunction()
{
  FastLED.setBrightness(BRIGHTNESS);
  leds[0] = CRGB::Red;
  FastLED.show();
  vTaskDelay(200);

  leds[0] = CRGB::Yellow;
  FastLED.show();
  vTaskDelay(200);
}

#define FUEL_LEVEL_MALFUNCTION 2
void StatusLED_FuelLevelMalfunction()
{
  leds[0] = CRGB::Red;
  FastLED.show();
  vTaskDelay(500);

  leds[0] = CRGB::Green;
  FastLED.show();
  vTaskDelay(500);
}

int brightness = MAX_BRIGHTNESS;

#define LOW_BATTERY 3
void StatusLED_LowBattery()
{
  leds[0] = CRGB::Red;
  FastLED.show();
  vTaskDelay(50);
  
  brightness = brightness - 2;
  if (brightness <= MIN_BRIGHTNESS)
    brightness = MAX_BRIGHTNESS;

}

#define OFF 4
void StatusLED_Off()
{
  leds[0] = CRGB::Black;
  FastLED.show();
  vTaskDelay(500);
}

bool increment = false;
#define BLE_SCAN 5
void StatusLED_BLEScan()
{
  FastLED.setBrightness(brightness);
  leds[0] = CRGB::Blue;
  FastLED.show();
  vTaskDelay(100);

  if (increment)
    brightness = brightness + 2;
  else
    brightness = brightness - 2;
  
  if (brightness <= MIN_BRIGHTNESS)
  {
    brightness = MIN_BRIGHTNESS;
    increment = !increment;
  }
  if (brightness >= MAX_BRIGHTNESS)
  {
    brightness = MAX_BRIGHTNESS;
    increment = !increment;
  }
}

#define NORMAL_MODE 6
bool led_green = false;
void StatusLED_NormalMode()
{
  if (led_green)
    leds[0] = CRGB::Green;
  else
    leds[0] = CRGB::Blue;

  FastLED.setBrightness(brightness);
  FastLED.show();
  vTaskDelay(100);

  if (increment)
    brightness = brightness + 2;
  else
    brightness = brightness - 2;
  
  if (brightness <= MIN_BRIGHTNESS)
  {
    brightness = MIN_BRIGHTNESS;
    increment = !increment;
  }
  if (brightness >= MAX_BRIGHTNESS)
  {
    brightness = MAX_BRIGHTNESS;
    increment = !increment;
    led_green = !led_green;
  }
}

#define CHARGING_MODE 7
void StatusLED_ChargingMode()
{
  leds[0] = CRGB::Yellow;
  FastLED.setBrightness(brightness);
  FastLED.show();
  vTaskDelay(100);

  if (increment)
    brightness = brightness + 2;
  else
    brightness = brightness - 2;
  
  if (brightness <= MIN_BRIGHTNESS)
  {
    brightness = MIN_BRIGHTNESS;
    increment = !increment;
  }
  if (brightness >= MAX_BRIGHTNESS)
  {
    brightness = MAX_BRIGHTNESS;
    increment = !increment;
  }
}

#define FULL_CHARGE_MODE 8
void StatusLED_FullChargeMode()
{
  leds[0] = CRGB::Green;
  FastLED.setBrightness(brightness);
  FastLED.show();
  vTaskDelay(100);

  if (increment)
    brightness = brightness + 2;
  else
    brightness = brightness - 2;
  
  if (brightness <= MIN_BRIGHTNESS)
  {
    brightness = MIN_BRIGHTNESS;
    increment = !increment;
  }
  if (brightness >= MAX_BRIGHTNESS)
  {
    brightness = MAX_BRIGHTNESS;
    increment = !increment;
  }
}

void Task_StatusLED(void *pvParameters)
{
  while (1)
  {
    switch(STATUS_LED_MODE)
    {
      default:
      case DEFAULT:
        FastLED.setBrightness(BRIGHTNESS);
        StatusLED_Default();
        break;
      case JOYSTICK_MALFUNCTION:
        FastLED.setBrightness(DEFAULT_BRIGHTNESS);
        StatusLED_JoystickMalfunction();
        break;
      case FUEL_LEVEL_MALFUNCTION:
        FastLED.setBrightness(DEFAULT_BRIGHTNESS);
        StatusLED_FuelLevelMalfunction();
        break;
      case LOW_BATTERY:
        StatusLED_LowBattery();
        break;
      case OFF:
        FastLED.setBrightness(DEFAULT_BRIGHTNESS);
        StatusLED_Off();
        break;
      case BLE_SCAN:
        StatusLED_BLEScan();
        break;
      case NORMAL_MODE:
        StatusLED_NormalMode();
        break;
      case CHARGING_MODE:
        StatusLED_ChargingMode();
        break;
      case FULL_CHARGE_MODE:
        StatusLED_FullChargeMode();
        break;
    }
  }
}

void setupStatusLED()
{
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  xTaskCreatePinnedToCore(Task_StatusLED, "Task_StatusLED", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
}

// ===================================================================

JOYSTICK joystick;
int buttonDownStart;
bool prevButtonDown;
bool enabled = false;
bool deadzone_enabled = false;
#define BUTTON_DOWN_PAIR_TIME 3000
#define BUTTON_DOWN_RESET_TIME 9000

// Joystick Settings.
#define DEADZONE_H 50
#define DEADZONE_V 50
#define CENTER_H 513
#define CENTER_V 510

#define LEFT_FULL 0
#define RIGHT_FULL 1023
#define FWD_FULL 0
#define REV_FULL 1023

void Task_Joystick(void *pvParameters)
{
  buttonDownStart = -1;
  prevButtonDown = false;
  while (1)
  {
    // Joystick specific
    int raw_h = joystick.getHorizontal();
    int raw_v = joystick.getVertical();
    bool button = joystick.getButton();

    Serial.printf("[X: %4d Y: %4d B: %d] => ", raw_h, raw_v, button);

    if(abs(raw_h - CENTER_H) > DEADZONE_H)
    {
      lastInputTime = millis();
      deadzone_enabled = true;
      if (raw_h < CENTER_H)
      {
        // Left Defelection
        joystick_deflection_h = map(raw_h + DEADZONE_H, LEFT_FULL, CENTER_H, -100, -10 );
        joystick_deflection_h = constrain(joystick_deflection_h, -100, 0);
      }
      else
      {
        // Right Defelection
        joystick_deflection_h = map(raw_h - DEADZONE_H, CENTER_H, RIGHT_FULL, 10, 100 );
        joystick_deflection_h = constrain(joystick_deflection_h, 0, 100);
      }
    }

    if(abs(raw_v - CENTER_V) > DEADZONE_V)
    {
      lastInputTime = millis();
      deadzone_enabled = true;
      if (raw_v < CENTER_V)
      {
        // Forward Defelection
        joystick_deflection_v = map(raw_v + DEADZONE_V, FWD_FULL, CENTER_V, 100, 10 );
        joystick_deflection_v = constrain(joystick_deflection_v, 0, 100);
      }
      else
      {
        // Reverse Defelection
        joystick_deflection_v = map(raw_v - DEADZONE_V, CENTER_V, REV_FULL, -10, -100 );
        joystick_deflection_v = constrain(joystick_deflection_v, -100, 0);
      }
    }

    if (abs(raw_v - CENTER_V) < DEADZONE_V && abs(raw_h - CENTER_H) < DEADZONE_H)
    {
      deadzone_enabled = false;
    }

    Serial.printf("[%4d %4d]\n", joystick_deflection_h, joystick_deflection_v);

    // Button Code

    // This is the first-time we have pushed the button down.
    if (!button && !prevButtonDown)
    {
      lastInputTime = millis();
      prevButtonDown = true;
      buttonDownStart = millis();
    }
    if (button && prevButtonDown)
    {
      if (buttonDownStart > 0)
      {
        // We started counting how long the button was down for.
        int delta = millis() - buttonDownStart;
        buttonDownStart = -1;
        prevButtonDown = false;
        Serial.print("Delta: ");
        Serial.print(delta);
        Serial.println();
        if (delta > BUTTON_DOWN_RESET_TIME)
        {
          Serial.println("Restarting ESP...");
          ESP.restart();
          delta = 0;
        }
        else if (BUTTON_DOWN_RESET_TIME > delta && delta > BUTTON_DOWN_PAIR_TIME)
        {
          Serial.println("Starting BLE Scan Mode...");
          startBLEScanMode();
        }
        else
        {
          // This was a click. What should we do here?
          Serial.println("Enabled Control");
          enabled != enabled;
        }
      }
    }
    vTaskDelay(200);
  }
}

void setupJoystick()
{
  if (joystick.begin() == false)
  {
    Serial.println("Joystick does not appear to be connected. Please check wiring.");
    STATUS_LED_MODE = JOYSTICK_MALFUNCTION;
    ErrorOnInit = true;
  }
  else
  {
    xTaskCreatePinnedToCore(Task_Joystick, "Task_Joystick", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
  }
}

// ===================================================================

SFE_MAX1704X lipo; // Defaults to the MAX17043

double voltage = 0; // Variable to keep track of LiPo voltage
double soc = 0; // Variable to keep track of LiPo state-of-charge (SOC)
bool alert; // Variable to keep track of whether alert has been triggered

void Task_PowerManagement(void *pvParameters)
{
  while(1)
  {
    // lipo.getVoltage() returns a voltage value (e.g. 3.93)
    voltage = lipo.getVoltage();
    // lipo.getSOC() returns the estimated state of charge (e.g. 79%)
    soc = lipo.getSOC();
    // lipo.getAlert() returns a 0 or 1 (0=alert not triggered)
    alert = lipo.getAlert();

    // Print the variables:
    Serial.print(voltage);  // Print the battery voltage
    Serial.print(" V | ");

    Serial.print(soc); // Print the battery state of charge
    Serial.print(" % | ");

    if (alert)
      Serial.println("[!]");
    else
      Serial.println("[ ]");

    if(alert)
    {
      Serial.println("Low Battery...");
      STATUS_LED_MODE = LOW_BATTERY;
    }
    else
    {
      if (soc >= 100.0)
      {
        // Charging complete.
        Serial.println("Fully charged...");
        if (!isConnected && !isScanning)
          STATUS_LED_MODE = FULL_CHARGE_MODE;
      }
      else
      {
        if (soc < 100.0 && voltage > 4.0)
        {
          // Charging.
          Serial.println("Charging...");
          if (!isConnected && !isScanning)
            STATUS_LED_MODE = CHARGING_MODE;
        }
      }
    }

    vTaskDelay(1000);
  }
}

void setupPowerManagement()
{
  // Set up the MAX17043 LiPo fuel gauge:
  if (lipo.begin() == false) // Connect to the MAX17043 using the default wire port
  {
    Serial.println(F("MAX17043 not detected. Please check wiring."));
    STATUS_LED_MODE = FUEL_LEVEL_MALFUNCTION;
    ErrorOnInit = true;
  }
  else
  {
    lipo.quickStart();
    lipo.setThreshold(20);
    xTaskCreatePinnedToCore(Task_PowerManagement, "Task_PowerManagement", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
  }
}
// ============================================================================
void startBLEScanMode()
{
  STATUS_LED_MODE = BLE_SCAN;
  isScanning = true;
  xTaskCreatePinnedToCore(Task_BLEScan, "Task_BLEScan", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
}

void Task_BLEScan(void *pvParameters)
{
  while(1)
  {
    if (isScanning)
    {
      Serial.println("Running BLE SCAN...");

      do
      {
        BLE.scanForUuid(remoteControl);
        peripheral = BLE.available();
        vTaskDelay(1000);
      } while (!peripheral);

      if (peripheral) {
        Serial.println("* Peripheral device found!");
        Serial.print("* Device MAC address: ");
        Serial.println(peripheral.address());
        Serial.print("* Device name: ");
        Serial.println(peripheral.localName());
        Serial.print("* Advertised service UUID: ");
        Serial.println(peripheral.advertisedServiceUuid());
        Serial.println(" ");
      }
      BLE.stopScan();

      preferences.putString("peer_mac",peripheral.address());
      isScanning = false;
    }
    vTaskDelay(1000);
  }
}

void setupBLE()
{
  if (!BLE.begin()) {
    Serial.println("[BLE] Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  isConnected = false;
  xTaskCreatePinnedToCore(Task_BLE, "Task_BLE", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
}

void connectToCar()
{
  while(!isConnected)
  {
    STATUS_LED_MODE = BLE_SCAN;
    do
    {
      BLE.scanForUuid(remoteControl);
      peripheral = BLE.available();
      vTaskDelay(1000);
    } while (!peripheral);
    String peer_mac = preferences.getString("peer_mac","");
    if (peer_mac == peripheral.address())
    {
      BLE.stopScan();
      if (peripheral.connect())
      {
        isConnected = true;
        Serial.println("[BLE] Connected to peripheral device!");
      }
    }
  }

}

void Task_BLE(void* pvParams)
{
  while(1)
  {
    String peer_mac = preferences.getString("peer_mac","");
    if (peer_mac == "")
    {
      // We don't have a peer device. We should skip any transmission
      Serial.println("[BLE] No Peer Device Mac found in Flash... Waiting for peer...");
      vTaskDelay(1000);
      continue;
    }
    else
    {
      if(!isConnected)
      {
        connectToCar();
      }
      else
      {
        STATUS_LED_MODE = NORMAL_MODE;
        // We are connected and we can do things. Lets send over all pert data.
        BLECharacteristic rc_Enable = peripheral.characteristic(remoteControlEnabled);
        BLECharacteristic rc_Stop = peripheral.characteristic(remoteControlStop);
        BLECharacteristic rc_X = peripheral.characteristic(remoteControlJoystickX);
        BLECharacteristic rc_Y = peripheral.characteristic(remoteControlJoystickY);

        rc_Enable.writeValue((uint8_t)(enabled || deadzone_enabled));
        rc_Stop.writeValue((uint8_t)estop);
        rc_X.writeValue((short) joystick_deflection_h);
        rc_Y.writeValue((short) joystick_deflection_v);
      }
    }
  }
}

// ============================================================================

void printDevice(byte addr)
{
  if (addr == 0x20 )
  {
    Serial.print(" = Joystick");
  }
  else if (addr == 0x36)
  {
    Serial.print(" = MAX1704X");
  }
  else if (addr == 0x6F)
  {
    Serial.print(" = LED Button");
  }
  else
  {
    Serial.print(" = UNKNOWN");
  }
}

void listDevices()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      printDevice(address);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

// ============================================================================

#define MAX_BUTTON_BRIGHTNESS 200
#define MIN_BUTTON_BRIGHTNESS 20
#define BUTTON_BRIGHTNESS_STEP 10
uint8_t current_button_brightness = 50;
int current_button_status = 0;
bool up = true;

bool released = true;
#define BUTTON_LED_OFF 0
#define BUTTON_LED_SOLID 1
#define BUTTON_LED_RAPID 2
#define BUTTON_LED_SLOW 3

void setup_stop_button()
{
  if (button.begin() == false) {
    Serial.println("Button did not acknowledge! Freezing.");
    while (1);
  }

  xTaskCreatePinnedToCore(Task_StopButton, "Task_StopButton", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(Task_StopButtonLED, "Task_StopButtonLED", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
}

void Task_StopButton(void *pvParameters)
{
  while(1)
  {
    // Check the input
    if (button.isPressed() == true) 
    {      // Issue an emergency STOP
      lastInputTime = millis();
      if (!estop && released)
      {
        estop = true;
        released = false;
        Serial.println("STOP");
        current_button_status = BUTTON_LED_SOLID;
      }
      else if (estop && released)
      {
        estop = false;
        released = false;
        Serial.println("Release");
        current_button_status = BUTTON_LED_OFF;
      }
    }
    else
    {
      released = true;
    }
    vTaskDelay(200);
  }
}

void Task_StopButtonLED(void *pvParameters)
{
  while(1)
  {
    if (current_button_status == BUTTON_LED_OFF)
    {
      button.LEDoff();
      vTaskDelay(1000);
    }
    else if (current_button_status == BUTTON_LED_SOLID)
    {
      button.LEDon(150);
      vTaskDelay(1000);
    }
    else
    {
      // We are pulsing the LED.
      if(up)
      {
        current_button_brightness += BUTTON_BRIGHTNESS_STEP;
      }
      else
      {
        current_button_brightness -= BUTTON_BRIGHTNESS_STEP;
      }

      if (current_button_brightness >= MAX_BUTTON_BRIGHTNESS)
      {
        current_button_brightness = MAX_BUTTON_BRIGHTNESS;
        up != up;
      }
      else if (current_button_brightness <= MIN_BUTTON_BRIGHTNESS)
      {
        current_button_brightness = MIN_BUTTON_BRIGHTNESS;
        up != up;
      }

      button.LEDon(current_button_brightness);
      if (current_button_status == BUTTON_LED_RAPID)
      {
        vTaskDelay(200);
      }
      else
      {
        vTaskDelay(500);
      }
    }
  }
}

// ============================================================================
#define WAKEUP_GPIO              GPIO_NUM_4
void setupSleepTimer()
{
  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1);  //1 = High, 0 = Low
  xTaskCreatePinnedToCore(Task_SleepTimer, "Task_SleepTimer", 4096, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
}

void Task_SleepTimer(void* pvParams)
{
  while(1)
  {
    if(millis() - lastInputTime > SLEEP_TIMEOUT_MINUTES * 60 * 60)
    {
      STATUS_LED_MODE = OFF;
      Serial.println("Going to sleep now");
      esp_deep_sleep_start();
    }
    vTaskDelay(10000);
  }
}

// ============================================================================
uint64_t id;
#define FORCE_REGENERATE_DEV_UUID false
void setup()
{
  Serial.begin(115200);
  while (Serial == false); //Wait for serial monitor to connect before printing anything
  id = ESP.getEfuseMac();
  Serial.printf("\nCHIP MAC: %012llx\n", ESP.getEfuseMac());

  // Check if we have a UUID for ourselves.
  preferences.begin("enmed-gbg", false);
  device_uuid = preferences.getString("dev_uuid","");
  if (device_uuid == "" || FORCE_REGENERATE_DEV_UUID)
  {
    Serial.println("Missing Device UUID! Generating...");
    uint32_t s1 = id >> 32;
    uint32_t s2 = id & 0x00000000FFFFFFFF;
    uuid.seed(s1, s2);
    device_uuid = String(uuid.toCharArray());
    preferences.putString("dev_uuid", device_uuid);
  }
  Serial.printf("Device UUID: %s \n", device_uuid.c_str());

  // Scan Devices...
  Wire.begin();
  listDevices();
  
  // Init Devices...
  ErrorOnInit = false;
  setupStatusLED();
  setupPowerManagement();
  setup_stop_button();
  setupBLE();
  setupSleepTimer();
  if (!ErrorOnInit)
    setupJoystick();
  lastInputTime = millis();
}

void loop()
{

}