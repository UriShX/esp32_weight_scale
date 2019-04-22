/*********
   ESP32 IoT weight scale for baking bread using baker percentage.
   Using google sheets as databse, pulling percent of ingredient in recipe and calculate
   against load cell measurement taken using hx711 ADC w/amp for load cell.
   Displays next ingredient and how much still needs to be placed on scale
   on I2C based 2004 alphanumeric LCD.
   Uses ESP32's touch buttons for navigation.
   v0.1 by Uri Shani, 2018. No guarantees provided with this software.
   Licensed under ?? LGPL???

   Breadsheet read / write scale *Star Simpson @starsandrobots qoute on Twitter

   Credits:
   Google sheets use IgorF2 https://www.instructables.com/id/IoT-Wallet-smart-Wallet-With-Firebeetle-ESP32-Ardu/
   HX711 on ESP32 Andreas Spiess https://github.com/SensorsIot/Weight-Sensors
   WiFi configuration over BLE https://desire.giesecke.tk/index.php/2018/04/06/esp32-wifi-setup-over-ble/
   thunkable android app based on https://www.instructables.com/id/ESP32-BLE-Android-App-Arduino-IDE-AWESOME/

   Future:
   use 128*64 OLED screen
   use HTTPclient and markdown.js to display further instructions
*********/
//#include <Arduino.h>
#include <esp_wifi.h>
#include <WiFi.h>             // Library for Wi-Fi
#include <WiFiMulti.h>        // Library for multiple Wi-Fi connections
#include <WiFiClientSecure.h> // Library for creating a https client
//#include <HTTPSRedirect.h>
#include <ArduinoJson.h>      // Library for reading data from json
#include <Wire.h>             // Library for I2C communication to OLED display
#include "esp_deep_sleep.h"   // Library for ESP32 deep sleep
#include <HX711_ADC.h>        // Library for HX711 load cell ADC
#include "soc/rtc.h"          // Library for ESP32 RTC (set to 80 MHz for communication w/ HX711)
#include <U8g2lib.h>          // Library for dot matrix screen
//                            // Includes for BLE
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>          // BLE notify & indicate descriptor handler
//                            // EEPROM libraries
#include <Preferences.h>
#include <nvs.h>
#include <nvs_flash.h>
// include additional files
// #include "display.h"


// ----------------------------
// Function declarations
// ----------------------------

void initBLE(bool onOff);
void gotIP(system_event_id_t event);
void lostCon(system_event_id_t event);
void sendBLEdata(/*String _mode, */float _var1, float _var2);
void wifiListenerLoop(void * parameter);
bool scanWiFi();
void connectWiFi();
bool statusWIFI();
void IRAM_ATTR ISR();
void readScale();
void percentageCount(bool _percentSet, bool _keepPercentage, bool _tarePercentage);
void scaleCount(bool _countSet, bool _keepCount);
void unitSwitch();
byte doGet(String _host, uint16_t _port, String _toSend);
void readGoogleSheetTitles();
void readGoogleSheet();
void send2GoogleSheets();
void buttonCheck(void *parameter);
void menuManager (void *parameter);
void displayManager(void * parameter);
void drawScrollString(U8G2_SSD1306_128X64_NONAME_2_HW_I2C &screen0, int16_t offset, const char *s);
void drawHeader(U8G2_SSD1306_128X64_NONAME_2_HW_I2C &screen1, const char *s);
void drawIcons(U8G2_SSD1306_128X64_NONAME_2_HW_I2C &screen);
void drawWeather(U8G2_SSD1306_128X64_NONAME_2_HW_I2C &screen2, String symbol, String degree);
bool drawMainScreen(const char *s, bool scroll, String weight, String unit);
void drawInfoScreen(const char *s, String weight, String unit);
int8_t check_button_event(int8_t button_event);
int8_t drawFormula();
int8_t drawDBData();
byte drawMenu(byte menuPlace);
void setLeft();
void setRight();
void setUp();
void setDown();
void doSelect();
void helpScreen();

// ----------------------------
// FreeRTOS defines
// ----------------------------
TaskHandle_t displayTask;
TaskHandle_t menuTask;
TaskHandle_t buttonTask;
TaskHandle_t wifiListenerTask;
QueueHandle_t queue;
QueueHandle_t controlCase;
QueueHandle_t state;
EventGroupHandle_t caseEventGroup;

struct queueStruct {
  float mainMeasurement;
  float secMeasurement;
  byte currentCase;
};

// struct for states
struct stateStruct{
    bool countSet = false;
    bool keepCount = true;
    bool tareNow = false;
};

byte setCase;

#define BIT_CASE    B00000000//BIT0
#define BIT_WIFI    B00000001//BIT1
#define BIT_BLE     B00000010//BIT2
#define BIT_MENU    B00000100//BIT3
#define BIT_WIFI_ON B00001000//BIT4
#define BIT_BLE_ON  B00010000//BIT5
#define BIT_INHERIT B00100000//BIT6
#define BIT_READY   B10000000//BIT7
#define MASK_CASE B00001111

// ----------------------------
// Configurations
// ----------------------------
// The following will have to be updated according to your hardware and accounts
// ------------------------------------------------------------------------------------

/** Build time */
const char compileDate[] = __DATE__ " " __TIME__;

/** Unique device name */
char apName[] = "ESP32 scale Test";//"ESP32-xxxxxxxxxxxx";
/** Selected network
    true = use primary network
    false = use secondary network
*/
bool usePrimAP = true;
/** Flag if stored AP credentials are available */
bool hasCredentials = false;
/** Connection status */
volatile bool isConnected = false;
/** Connection change status */
bool connStatusChanged = false;

/**
   Create unique device name from MAC address
 **/
void createName() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  // Write unique name into apName
  //BLEDevice::init("ESP32 scale Test"); // Give it a name
  sprintf(apName, "ESP32 scale Test", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

// List of Service and Characteristic UUIDs
// {"m":byte,"w":float(4bytes),"v2":float(4bytes)}
#define SERVICE_UUID  "0000aaaa-ead2-11e7-80c1-9a214cf093ae"
#define MODE_UUID "00006666-ead2-11e7-80c1-9a214cf093ae"
#define MEASUREMENTS_UUID "00007777-ead2-11e7-80c1-9a214cf093ae"
#define WIFI_UUID     "00005555-ead2-11e7-80c1-9a214cf093ae"

/** SSIDs of local WiFi networks */
String ssidPrim;
String ssidSec;
/** Password for local WiFi network */
String pwPrim;
String pwSec;

/** WiFi constructor **/
WiFiMulti WiFiMulti;

WiFiClientSecure wificlient; // Create https client

//HTTPSRedirect* client = nullptr;

/** Characteristic for digital output */
BLECharacteristic *pCharacteristicWiFi;
BLECharacteristic *pCharacteristicMeas;
/** BLE Advertiser */
BLEAdvertising* pAdvertising;
/** BLE Service */
BLEService *pService;
/** BLE Server */
BLEServer *pServer;

bool bleOnOff = false;
bool bleConnected = false;
byte bleInitCounter = 0;
bool deviceConnected = false; // BLE connection status

bool wifiOnOff = false;


/** Buffer for JSON string */
// MAx size is 51 bytes for frame:
// {"ssidPrim":"","pwPrim":"","ssidSec":"","pwSec":""}
// + 4 x 32 bytes for 2 SSID's and 2 passwords
//StaticJsonBuffer<JSON_OBJECT_SIZE(4) + 40> BLEjsonBuffer;
StaticJsonBuffer<200> BLEjsonBuffer;
StaticJsonBuffer<200> jsonBuffer;
StaticJsonBuffer<500> spreadsheetBuffer;
const size_t bufferSize = JSON_ARRAY_SIZE(1) + JSON_ARRAY_SIZE(5) + JSON_OBJECT_SIZE(3) + 100;  // buffer for json message
//const size_t sheetBufferSize = JSON_ARRAY_SIZE(2) + 5*JSON_OBJECT_SIZE(1);


// ----------------------------


#define Threshold 400   // Touch sensor sensitivity

long timedSend = millis();
long displayTimer = millis();

/** HX711 load cell defines **/
const int doutPin = 18; //mcu > HX711 dout pin, must be external interrupt capable
const int sckPin = 5; //mcu > HX711 sck pin
//HX711 constructor:
HX711_ADC scale(doutPin, sckPin);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // Supposedly, help make external interrupts possible

/** Google sheets connection setup **/
String host = "script.google.com";
String googleAPI = "sheets.googleapis.com";
const uint16_t httpsPort = 443;

String spreadSheetID = "185nivAiZCSP7wAxyEsiGpCG9iFRDAQEIe9eeO0uELeM";  // ID from Google spreadsheet
String key = "key=AIzaSyDQ2lzlrUXEBTyEImagyatppqkKOCjVdMU";            // Google API key
// The ID below comes from Google Sheets.
// Towards the bottom of this page, it will explain how this can be obtained
const char *GScriptId = "AKfycbzbM1shFEMwv7N7XB4llUZdtGvo1uYSS-7-q84SHYQ0vq6qzLg";
//https://script.google.com/macros/s/AKfycbzbM1shFEMwv7N7XB4llUZdtGvo1uYSS-7-q84SHYQ0vq6qzLg/exec

String selectedSheet;
String toSend = "";
bool redirected = false;
bool recievedResponse = false;
byte connectionCounter = 0;

//String getString = "GET /v4/spreadsheets/" + spreadSheetID + "/values/";
//String getSheets = "GET /v4/spreadsheets/" + spreadSheetID + "?fields=sheets.properties.title&";// + key + " HTTP/1.1";
//String getSheetsScript = "GET /macros/s/AKfycbzbM1shFEMwv7N7XB4llUZdtGvo1uYSS-7-q84SHYQ0vq6qzLg/exec?";
#define SHEET_SCRIPT_URI "/macros/s/AKfycbzbM1shFEMwv7N7XB4llUZdtGvo1uYSS-7-q84SHYQ0vq6qzLg/exec?"

#define MAX_SHEETS 5
#define MAX_INGREDIENTS 10
String sheets[MAX_SHEETS];
String ingredients[MAX_INGREDIENTS];
float quantities[MAX_INGREDIENTS];
float measuredQuantities[MAX_INGREDIENTS];

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

/** UI handlers **/
String operationMode = "single";
bool inheritCase = false;

// Screen constructors
#define SCREEN1_RES 19
#define SCREEN2_RES 23
#define SCREEN1_ADR 120
#define SCREEN2_ADR 122

U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/ /*U8X8_PIN_NONE*/SCREEN1_RES, SCL, SDA);
U8G2_SSD1306_128X64_NONAME_2_HW_I2C dataScreen(U8G2_R0, /* reset=*/ /*U8X8_PIN_NONE*/SCREEN2_RES, SCL, SDA);

/** menu items **/
static const uint8_t NUM_MENU_ITEMS = 6;
const char* menu_items[] = {
  "Single",
  "Count",
  "Percent",
  "Get formulas",
  "Get ingredients",
  "Save measurements"
};

/** button assignment **/
#define LEFT_BUT 15
#define RIGHT_BUT 0
#define UP_BUT 2
#define DOWN_BUT 4
#define SELECT_BUT 12
// interrupt service routine vars
bool left_set = false;
bool right_set = false;
bool up_set = false;
bool down_set = false;
bool select_set = false;
long left_millis = millis();
long right_millis = millis();
long up_millis = millis();
long down_millis = millis();
long select_millis = millis();
// Marker for menu
bool inMenu = false;
bool mainScreenScroll = false;

/** placeholders for measurements **/
float keepWeight = 0.0f;
float keepPercent = 0.0f;
float percentage = 0.0f;
int oldCount = 0;
float oldAvgWeight = 0.0f;
byte selection = 0;
bool baseIngredient = false;
// unit selection variables
byte unitSelect = 0;
#define UNIT_STRINGS 3
String unitStrings[UNIT_STRINGS] = {"g", "kg", "lb"};

RTC_DATA_ATTR int bootCount = 0; // Store boot count

void setup() {
  Serial.begin(115200); // Start serial communication

  queue = xQueueCreate(2, sizeof(struct queueStruct));
  
  if(queue == NULL){
    Serial.println("Error creating queueStruct");
  }

  state = xQueueCreate(2, sizeof(struct stateStruct));

  if(state == NULL){
    Serial.println("Error creating stateStruct");
  }

  controlCase = xQueueCreate(1, sizeof(setCase));

  if (controlCase == NULL){
    Serial.println("Error creating case queue");
  }

  xTaskCreatePinnedToCore(
    menuManager,
    "menuTask",
    2048,
    NULL,
    3,
    &menuTask,
    1
  );
  delay(500);

  xTaskCreatePinnedToCore(
    displayManager,
    "displayTask",
    2048,
    NULL,
    2,
    &displayTask,
    1);
  delay(500);  // needed to start-up task1

  xTaskCreatePinnedToCore(
    buttonCheck,
    "buttonTask",
    1024,
    NULL,
    3,
    &buttonTask,
    1);
  delay(500);  // needed to start-up task1

  caseEventGroup  = xEventGroupCreate();

  xTaskCreatePinnedToCore(
    wifiListenerLoop,
    "wifiListenerTask",
    10024,
    NULL,
    1,
    &wifiListenerTask,
    0);
  delay(500);

  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M); // Set SoC RTC to 80MHz, for HX711 communication

  // Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  // Create unique device name (for BLE)
  createName();

  // store / retreive WiFi credentials from EEPROM
  Preferences preferences;
  preferences.begin("WiFiCred", false);
  bool hasPref = preferences.getBool("valid", false);
  if (hasPref) {
    ssidPrim = preferences.getString("ssidPrim", "");
    ssidSec = preferences.getString("ssidSec", "");
    pwPrim = preferences.getString("pwPrim", "");
    pwSec = preferences.getString("pwSec", "");

    if (ssidPrim.equals("")
        || pwPrim.equals("")
        || ssidSec.equals("")
        || pwPrim.equals("")) {
      Serial.println("Found preferences but credentials are invalid");
    } else {
      Serial.println("Read from preferences:");
      Serial.println("primary SSID: " + ssidPrim + " password: " + pwPrim);
      Serial.println("secondary SSID: " + ssidSec + " password: " + pwSec);
      hasCredentials = true;
    }
  } else {
    Serial.println("Could not find preferences, need send data over BLE");
  }
  preferences.end();

  /*
    Start BLE server
    initBLE();

  if (hasCredentials) {
    // Check for available AP's
    if (!scanWiFi()) {
      Serial.println("Could not find any AP");
    } else {
      // If AP was found, start connection
      connectWiFi();

      bool isConnected = statusWIFI();
      while (!isConnected) {
        isConnected = statusWIFI();
      }
    }
  }
  */

  delay(10);

  // Scale initialization
  Serial.println("Initializing the scale");
  float calValue; // calibration value
  calValue = 481.815; // uncomment this if you want to set this value in the sketch
  #if defined(ESP8266)
    //EEPROM.begin(sizeof calValue); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
  #endif
    //EEPROM.get(eepromAdress, calValue); // uncomment this if you want to fetch the value from eeprom

  scale.begin();
  int stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  scale.start(stabilisingtime);
  if (scale.getTareTimeoutFlag()) {
    Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    scale.setCalFactor(calValue); // set calibration value (float)
    Serial.println("Startup + tare is complete");
  }
  attachInterrupt(doutPin, ISR, FALLING);

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.getSingleConversion());

  Serial.print("read average: \t\t");
  Serial.println(scale.getData());

  Serial.print("get tare offset: \t\t");
  Serial.println(scale.getTareOffset());

  inputString.reserve(20); // malloc for serial input
  toSend.reserve(500);
  for (byte sheetCounter = 0; sheetCounter < MAX_SHEETS; sheetCounter++) {
    sheets[sheetCounter].reserve(50);
  }
  for (byte ingCounter = 0; ingCounter < MAX_INGREDIENTS; ingCounter++) {
    ingredients[ingCounter].reserve(50);
    // quantities[ingCounter].reserve(4);
  }

  Serial.printf("Internal Total heap %d, internal Free Heap %d\n",ESP.getHeapSize(),ESP.getFreeHeap());
    //Internal RAM
    //SPI RAM
  Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n",ESP.getPsramSize(),ESP.getFreePsram());
 
  Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n",ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());

  Serial.printf(" Flash Size %d, Flash Speed %d\n",ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
  
  helpScreen(); // serial usage helper

  //  touchAttachInterrupt(T0, callback, Threshold);  // Enable touch sensor wake up
  //  esp_deep_sleep_enable_touchpad_wakeup();        // Enable touch sensor wake up

  btStop(); // Turn off bluetooth for saving battery
}

void loop() {

  byte memCase = setCase;

  if(queue == NULL){
    Serial.println("structQueue was not created");
    return;
  }

  if (state == NULL){
    Serial.println("stateQueue was not created");
    return;
  }

  struct queueStruct sendQueue;
  sendQueue.mainMeasurement = oldAvgWeight;
  sendQueue.secMeasurement = percentage;
  sendQueue.currentCase = memCase;

  xQueueSend(queue, &sendQueue, 0);
  // Serial.printf("Loop core %d\n ", xPortGetCoreID());

  struct stateStruct sendState;
  xQueueReceive(state, &sendState, 0);

  //receive from serial terminal
  //should be handled in ISR, no simple implementation in Arduino IDE for ESP32
  if (Serial.available() > 0) {
    float i;
    char inByte = (char)Serial.read();
    inputString += inByte;
    if (inByte == '\n') {
      stringComplete = true;
    }
  }

  /* Handle strings when newline arrives
     Arduino terminal adds "\r\n" to each recieved string
     ' '      space
     '\t'     horizontal tab
     '\n'     newline
     '\v'     vertical tab
     '\f'     feed
     '\r'     carriage return
  */
  if (stringComplete) {
    if ((inputString[0] == 't' || inputString[0] == ' ') && inputString[1] == '\r') scale.tareNoDelay();
    else if (inputString[0] == 's' && inputString[1] == '\r') inheritCase = false, setCase = 0;
    else if (inputString[0] == 'c' && inputString[1] == '\r') {
      Serial.println("place single quantity item on scale, and type 'set' in terminal");
      inheritCase = false;
      setCase = 1;
    }
    else if (inputString[0] == 'p' && inputString[1] == '\r') {
      Serial.println("place 100% amount on scale, and type either 'set' for setting 100%");
      Serial.println("or 'setTare' for both setting & tare the scale.");
      Serial.println("this is useful if measuring components added to scale platform, such as in a bowl");
      Serial.println("further tare operations can be achieved by sending 't' or spacebar through the terminal");
      inheritCase = false;
      setCase = 2;
    }
    else if (inputString[0] == 'r' && inputString[1] == '\r') setCase = 3;
    else if ((inputString[0] == 'h' && inputString[1] == '\r') || inputString[0] == '\r') helpScreen();
    else if (inputString == "ble\r\n") {
      bleOnOff = (bleOnOff)?false:true;
      Serial.printf("ble: %d\n", bleOnOff);
      setCase = 7;
    }
    else if (inputString == "wifi\r\n") {
      wifiOnOff = (wifiOnOff)?false:true;
      Serial.printf("wifi: %d\n", wifiOnOff);
      setCase = 6;
    }
    else if (inputString == "set\r\n") sendState.countSet = true, sendState.keepCount = false;
    else if (inputString == "setTare\r\n") sendState.countSet = true, sendState.keepCount = false, sendState.tareNow = true;
    else if (inputString[0] >= 49 && inputString[0] <= 57 && inputString[1] == '\r') {
      selection = inputString.toInt() - 1;
      if (selection <= MAX_INGREDIENTS) {
        Serial.print("\n");
        Serial.print(ingredients[selection]);
        Serial.print(":\t");
        Serial.print(quantities[selection]);
        Serial.println("\n");
      }
    }
    else {
      for (byte j = 0; j < 5; j++) {
        if (inputString == sheets[j] + '\r' + '\n') {
          selectedSheet = sheets[j];
          setCase = 4;
        }
      }
    }

      /* 
       for (byte i = 0; i < inputString.length(); i++) {
         Serial.println(float(inputString[i]));
       }
       Serial.println(inputString);
      clear the string:
      */

    inputString = "";
    stringComplete = false;
  }

  //check if last tare operation is complete
  if (scale.getTareStatus() == true) {
    Serial.print("Tare complete\t");
    Serial.println(oldAvgWeight);
  }

  int caseEventByte = xEventGroupGetBits(caseEventGroup); //not good, doesn't just get the bits, but clears all afterwards
  // int caseEventByte = xEventGroupWaitBits(caseEventGroup, BIT_CASE, pdTRUE, pdTRUE, 0);
  if (bitRead(caseEventByte, BIT_CASE)) {
    Serial.print("flags: "); Serial.println(caseEventByte, BIN);
    xQueueReceive(controlCase, &setCase, 10);
    xEventGroupClearBits(caseEventGroup, (1 << BIT_CASE));
  }

  switch (setCase) {
    if (setCase != memCase) Serial.printf("setCase:\t%d,\tmemCase:\t%d\n", setCase, memCase);
    case 0:
      readScale();
      operationMode = "single";
      break;

    case 1:
      scaleCount(sendState.countSet, sendState.keepCount);
      if (sendState.countSet) {
        sendState.countSet = false;
        if (!sendState.keepCount) sendState.keepCount = true;
      } 
      operationMode = "count";
      break;

    case 2:
      // set 100% by dividing current mass by selected quantity
      if (sendState.countSet && baseIngredient) {
        sendState.keepCount = true;
        baseIngredient = false;
      }
      percentageCount(sendState.countSet, sendState.keepCount, sendState.tareNow);
      if (sendState.countSet) {
        sendState.countSet = false;
        if (!sendState.keepCount) sendState.keepCount = true;
      } 
      // operationMode = "percent";
      operationMode = (inheritCase) ? "formula%":"percentage";
      break;

    case 3:
      Serial.printf("case:%d\n",memCase);
      if (wifiOnOff) {
        if (sheets[0] != "") break;
        cli();//not implemented in esp32
        scale.powerDown();             // put the ADC in sleep mode
        Serial.println("\nReading Google Spreadsheet data...");
        Serial.print("sheet names ");
        readGoogleSheetTitles();
        scale.powerUp();
        sei();//not implemented in esp32
      } else {
        Serial.println("connect to WiFi (type 'wifi' in terminal) to complete this task");
        
      }
      setCase = memCase;
      // menuPlace = memCase; 
      break;

    case 4:
      if (statusWIFI()) {
        cli(); //not implemented in esp32
        scale.powerDown();             // put the ADC in sleep mode
        Serial.println("\nReading Google Spreadsheet data...");
        Serial.print("Ingredients ");
        readGoogleSheet();
        baseIngredient = true;
        scale.powerUp();
        sei(); //not implemented in esp32
        Serial.println("pick with keyboard numbers the base ingredient upon which other ingredients' quantities will be based.");
        Serial.println("once you placed the quantity of the base ingredient, type 'set' or 'setTare'.");
        Serial.println("a list of remaining ingredients will then be displayed, choose the next ingredient, and so on.");
        inheritCase = true;
        setCase = 2;//memCase;
      } else {
        Serial.println("connect to WiFi (type 'wifi' in terminal) to complete this task");
        setCase = memCase;
      }
      
      break;

    case 5:
      if (statusWIFI()) {
        scale.powerDown();
        Serial.println("Saving measurements to Google Spreadsheet...");
        send2GoogleSheets();
        scale.powerUp();
        inheritCase = false;
        setCase = 2;
      } else {
        Serial.println("connect to WiFi (type 'wifi' in terminal) to complete this task");
        setCase = memCase;
      }
      break;

    case 7:
      // //    noInterrupts();//portDISABLE_INTERRUPTS();//detachInterrupt(doutPin);//, ISR, FALLING);
      // cli();

      // if (bleOnOff && !bleConnected) {
      // /*esp_bt_controller_enable(ESP_BT_MODE_BTDM);
      //        btStart();
      //        while (!btStarted());
      //   Start BLE server
      //  if (bleInitCounter == 0) {
      //   */
      // initBLE(true);
      // /*} else {
      //     btStart();
      //    esp_bt_controller_enable(ESP_BT_MODE_BLE);
      //    delay(100);
      //    btStart();
      //    esp_bt_controller_init();
      //  }
      // */
      // do {
      //   bleConnected = esp_bt_controller_get_status();
      // } while (!bleConnected);
      // /*
      //  bleInitCounter++;
      //  Serial.println(bleInitCounter);
      // */
      // Serial.printf("\n\ndisabling BLE will result in restarting the scale atm. Sorry!\n\n");
      // //  Serial.printf("ble stat: %d\n", bleConnected);
      // } else if (!bleOnOff && bleConnected) {
      //   ESP.restart();
      //   /*
      //     esp_bluedroid_disable();
      //     esp_bluedroid_deinit();
      //     initBLE(false);
      //     delay(100);
      //     if (esp_bt_controller_disable() != 0) {
      //       Serial.println("could not disable BLE");
      //     } else {
      //       bleConnected = false;
      //     }
      //     delay(100);
      //     esp_bt_controller_deinit();
      //     delay(100);
      //     esp_bt_controller_mem_release(ESP_BT_MODE_BLE);// uncommented in BLEDevice.cpp
      //     btStop(); // Turn off bluetooth for saving battery
      //     delay(100);
      //     bleConnected = false;
      //     do {
      //       byte getBLEStat = esp_bt_controller_get_status();//BLEDevice::getInitialized();
      //       Serial.printf("ble stat reply: %d, bleDevice lib init: %d\n", getBLEStat, BLEDevice::getInitialized());
      //       if (getBLEStat == 2) bleConnected = false;
      //       //  bleConnected = 
      //       delay(250);
      //     } while (bleConnected);
      //   */
      // }
      // Serial.printf("ble stat: %d\n", bleConnected);
      // Serial.println(ESP.getFreeHeap());
      setCase = memCase;
      // //    interrupts();//portENABLE_INTERRUPTS();//attachInterrupt(doutPin, ISR, FALLING);
      // sei();

      break;

    case 6:
      // cli();
      // detachInterrupt(doutPin);//, ISR, FALLING);
      // Serial.printf("wifiOnOff: %d, wifiConnected: %d\n", wifiOnOff, wifiConnected);
      // if (wifiOnOff && !wifiConnected) {
      //   Serial.println("turning wifi on");
      //   if (hasCredentials) {
      //     bool scanResult;
      //     byte scanTrials = 0;
      //     // Check for available AP's
      //     do {
      //       String noAPMessage = "Could not find any AP";
      //       String noAPTryAgain = ", trying again";
      //       scanResult = scanWiFi();
      //       if (!scanResult) {
      //         scanTrials++;
      //         if (scanTrials < 4) noAPMessage.concat(noAPTryAgain);
      //         Serial.println(noAPMessage);
      //       }
      //     } while(!scanResult && scanTrials < 4);
          
      //     if (scanResult) {
      //       // If AP was found, start connection
      //       connectWiFi();
      
      //       bool isConnected = statusWIFI();
      //       while (!isConnected) {
      //         isConnected = statusWIFI();
      //       }
      //       wifiConnected = statusWIFI();
      //     } else {
      //       WiFi.disconnect(true);
      //       wifiOnOff = false;
      //     }
      //   }
      // } else if (!wifiOnOff && wifiConnected) {
      //   Serial.println("turning wifi off");
      //   WiFi.disconnect(true);
      // /*
      //   esp_wifi_disconnect();
      //   delay(100);
      //   esp_wifi_stop();
      //   delay(100);
      //   esp_wifi_deinit();
      //   delay(100);
      // */
      //   do {
      //     wifiConnected = statusWIFI();
      //   } while (wifiConnected);
      // }
      // Serial.println(ESP.getFreeHeap());
      setCase = memCase;
      // sei();
      // attachInterrupt(doutPin, ISR, FALLING);
      break;
  }
  
  xQueueSend(state, &sendState, 0);

  /** WiFi connection checker (keep alive function) **/
  if (wifiOnOff) {
    if (connStatusChanged) {
      if (isConnected) {
        Serial.print("Connected to AP: ");
        Serial.print(WiFi.SSID());
        Serial.print(" with IP: ");
        Serial.print(WiFi.localIP());
        Serial.print(" RSSI: ");
        Serial.println(WiFi.RSSI());
      } else {
        if (hasCredentials) {
          Serial.println("Lost WiFi connection");
          // Received WiFi credentials
          if (!scanWiFi()) { // Check for available AP's
            Serial.println("Could not find any AP");
          } else { // If AP was found, start connection
            connectWiFi();
          }
        }
      }
      connStatusChanged = false;
    }
  }
}

// ----------------------------
// Auxiliary functions
// ----------------------------

/**
   MyServerCallbacks
   Callbacks for client connection and disconnection
*/
class MyServerCallbacks: public BLEServerCallbacks {
    // TODO this doesn't take into account several clients being connected
    void onConnect(BLEServer* pServer) {
      Serial.println("BLE client connected");
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("BLE client disconnected");
      deviceConnected = false;
      pAdvertising->start();
    }
};

/**
   MyCallbackHandler
   Callbacks for BLE client read/write requests
*/
class MyCallbackHandler: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() == 0) {
        return;
      }
      Serial.println("Received over BLE: " + String((char *)&value[0]));

      // Decode data
      int keyIndex = 0;
      for (int index = 0; index < value.length(); index ++) {
        value[index] = (char) value[index] ^ (char) apName[keyIndex];
        keyIndex++;
        if (keyIndex >= strlen(apName)) keyIndex = 0;
      }

      /** Json object for incoming data */
      JsonObject& jsonIn = jsonBuffer.parseObject((char *)&value[0]);
      if (jsonIn.success()) {
        if (jsonIn.containsKey("ssidPrim") &&
            jsonIn.containsKey("pwPrim") &&
            jsonIn.containsKey("ssidSec") &&
            jsonIn.containsKey("pwSec")) {
          ssidPrim = jsonIn["ssidPrim"].as<String>();
          pwPrim = jsonIn["pwPrim"].as<String>();
          ssidSec = jsonIn["ssidSec"].as<String>();
          pwSec = jsonIn["pwSec"].as<String>();

          Preferences preferences;
          preferences.begin("WiFiCred", false);
          preferences.putString("ssidPrim", ssidPrim);
          preferences.putString("ssidSec", ssidSec);
          preferences.putString("pwPrim", pwPrim);
          preferences.putString("pwSec", pwSec);
          preferences.putBool("valid", true);
          preferences.end();

          Serial.println("Received over bluetooth:");
          Serial.println("primary SSID: " + ssidPrim + " password: " + pwPrim);
          Serial.println("secondary SSID: " + ssidSec + " password: " + pwSec);
          connStatusChanged = true;
          hasCredentials = true;
        } else if (jsonIn.containsKey("erase")) {
          Serial.println("Received erase command");
          Preferences preferences;
          preferences.begin("WiFiCred", false);
          preferences.clear();
          preferences.end();
          connStatusChanged = true;
          hasCredentials = false;
          ssidPrim = "";
          pwPrim = "";
          ssidSec = "";
          pwSec = "";

          int err;
          err = nvs_flash_init();
          Serial.println("nvs_flash_init: " + err);
          err = nvs_flash_erase();
          Serial.println("nvs_flash_erase: " + err);
        } else if (jsonIn.containsKey("reset")) {
          WiFi.disconnect();
          esp_restart();
        } else if (jsonIn.containsKey("mode")) {
          String _mode;
          _mode = jsonIn["mode"].as<String>();
          if (_mode == "Single measurement") setCase = 0;
          else if (_mode == "Count items") setCase = 1;
          else if (_mode == "Baker's percentage") setCase = 2;
        }
      } else {
        Serial.println("Received invalid JSON");
      }
      jsonBuffer.clear();
    };

    void onRead(BLECharacteristic *pCharacteristic) {
      Serial.println("BLE onRead request");

      String wifiCredentials;

      /** Json object for outgoing data */
      JsonObject& jsonOut = jsonBuffer.createObject();
      jsonOut["ssidPrim"] = ssidPrim;
      jsonOut["pwPrim"] = pwPrim;
      jsonOut["ssidSec"] = ssidSec;
      jsonOut["pwSec"] = pwSec;
      // Convert JSON object into a string
      jsonOut.printTo(wifiCredentials);

      // encode the data
      int keyIndex = 0;
      Serial.println("Stored settings: " + wifiCredentials);
      for (int index = 0; index < wifiCredentials.length(); index ++) {
        wifiCredentials[index] = (char) wifiCredentials[index] ^ (char) apName[keyIndex];
        keyIndex++;
        if (keyIndex >= strlen(apName)) keyIndex = 0;
      }
      pCharacteristicWiFi->setValue((uint8_t*)&wifiCredentials[0], wifiCredentials.length());

      jsonBuffer.clear();
    }
};

/**
   initBLE
   Initialize BLE service and characteristic
   Start BLE server and service advertising
*/
void initBLE(bool onOff) {
  Serial.printf("initBLE core %d\n ", xPortGetCoreID());
  if (onOff) {
      if (BLEDevice::getInitialized() == 0) {
      // Initialize BLE and set output power
      BLEDevice::init(apName);
      BLEDevice::setPower(ESP_PWR_LVL_P7);
    
      // Create BLE Server
      pServer = BLEDevice::createServer();
    
      // Set server callbacks
      pServer->setCallbacks(new MyServerCallbacks());
    
      // Create BLE Service
      pService = pServer->createService(BLEUUID(SERVICE_UUID), 20);
    
      // Create BLE Characteristic for WiFi settings
      pCharacteristicWiFi = pService->createCharacteristic(
                              BLEUUID(WIFI_UUID),
                              // WIFI_UUID,
                              BLECharacteristic::PROPERTY_READ |
                              BLECharacteristic::PROPERTY_WRITE
                            );
      pCharacteristicWiFi->setCallbacks(new MyCallbackHandler());
    
      // Create BLE Characteristic for measurements
      pCharacteristicMeas = pService->createCharacteristic(
                              BLEUUID(MEASUREMENTS_UUID),
                              //    BLECharacteristic::PROPERTY_READ |
                              //    BLECharacteristic::PROPERTY_WRITE |
                              BLECharacteristic::PROPERTY_NOTIFY |
                              BLECharacteristic::PROPERTY_INDICATE
                            );
      //  pCharacteristicMeas->setCallbacks(new MyCallbackHandler());
    
      // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
      // Create a BLE Descriptor
      pCharacteristicMeas->addDescriptor(new BLE2902());
      }
  
    // Start the service
    pService->start();
  
    // Start advertising
    pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
  } else if (!onOff) {
    pAdvertising->stop();
    pService->stop();
  }
}

/** Callback for receiving IP address from AP */
void gotIP(system_event_id_t event) {
  isConnected = true;
  connStatusChanged = true;
}

/** Callback for connection loss */
void lostCon(system_event_id_t event) {
  isConnected = false;
  connStatusChanged = true;
}

void sendBLEdata(/*String _mode, */float _var1, float _var2) {
  Serial.printf("sendBLEdata core %d\n ", xPortGetCoreID());
  if (deviceConnected && ((millis() - 500) > timedSend)) {
    // Fabricate some arbitrary junk for now...
    //    txValue = analogRead(A3) / 3.456; // This could be an actual sensor reading!
    timedSend = millis();
    String jsonValue;
    float txValue = _var1;
    // Let's convert the value to a char array:
    char txString[8]; // make sure this is big enuffz
    dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer
    Serial.print("as string:\t");
    Serial.println(txString);

    //    pCharacteristic->setValue(&txValue, 1); // To send the integer value
    //    pCharacteristic->setValue("Hello!"); // Sending a test message
    pCharacteristicMeas->setValue(txString);
    std::string asSet = pCharacteristicMeas->getValue();
    String setString = "";
    Serial.print("set characteristic value as:\t");
    for (byte i = 0; i < asSet.length(); i++) {
      char stringChar = asSet[i];
      setString += stringChar;
    }
    Serial.println(setString);

    /** Json object for outgoing data */
    //    JsonObject& jsonOut = BLEjsonBuffer.createObject();
    //    jsonOut["mode"] = _mode;
    //    jsonOut["weight"] = String(_var1);
    //    jsonOut["var2"] = String(_var2);
    //    // Convert JSON object into a string
    //    jsonOut.printTo(jsonValue);

    //    pCharacteristicMeas->setValue((uint8_t*)&jsonValue[0],jsonValue.length());
    //    BLEjsonBuffer.clear();

    byte testNotify = *pCharacteristicMeas->getDescriptorByUUID((uint16_t)0x2902)->getValue();

    if (testNotify == 1) {
      pCharacteristicMeas->notify(); // Send the value to the app!
      Serial.print("*** Sent Int: /t");
      Serial.print(txValue, DEC);
      Serial.println(" ***");
      //      Serial.print("*** Sent Float: /t");
      //      Serial.print(_var2);
      //      Serial.println(" ***");
    } else {
      setCase = 0;
      Serial.print("notify failed, value of 0x2902 descriptor:\t");
      Serial.println(testNotify, HEX);//*pCharacteristicMeas->getDescriptorByUUID((uint16_t)0x2902)->getValue(), HEX);
    }
  }
}

void wifiListenerLoop(void * parameter)
{
  byte bitsForWifiOrBle = (BIT_WIFI | BIT_BLE) << 1;

  while(1) {
    int bleOrWifi = xEventGroupWaitBits(caseEventGroup, bitsForWifiOrBle, pdTRUE, pdFALSE, portMAX_DELAY);
    Serial.print("recieved:\t");Serial.println(bleOrWifi, BIN);

    if (bitRead(bleOrWifi, BIT_WIFI)) {
      // detachInterrupt(doutPin);//, ISR, FALLING);
      Serial.printf("wifiOnOff: %d, wifiConnected: %d\n", statusWIFI());
      if (!statusWIFI()) {
        Serial.println("turning wifi on");
        if (hasCredentials) {
          bool scanResult;
          byte scanTrials = 0;
          // Check for available AP's
          do {
            String noAPMessage = "Could not find any AP";
            String noAPTryAgain = ", trying again";
            detachInterrupt(doutPin);
            vTaskDelay(1);
            scanResult = scanWiFi(); 
            attachInterrupt(doutPin, ISR, FALLING);         
            if (!scanResult) {
              scanTrials++;
              if (scanTrials < 4) noAPMessage.concat(noAPTryAgain);
              Serial.println(noAPMessage);
            }
          } while(!scanResult && scanTrials < 4);
          
          if (scanResult) {
            // If AP was found, start connection
            detachInterrupt(doutPin);
            vTaskDelay(1);
            connectWiFi();
            attachInterrupt(doutPin, ISR, FALLING);
      
            bool isConnected = statusWIFI();
            while (!isConnected) {
              isConnected = statusWIFI();
            }
            // wifiConnected = statusWIFI();
          } else {
            WiFi.disconnect(true);

            // wifiOnOff = false;
          }
        }
      } else if (statusWIFI()) {
        Serial.println("turning wifi off");
        WiFi.disconnect(true);
      /*
        esp_wifi_disconnect();
        delay(100);
        esp_wifi_stop();
        delay(100);
        esp_wifi_deinit();
        delay(100);
      */
        // do {
        //   wifiConnected = statusWIFI();
        // } while (wifiConnected);
      }
      Serial.println(ESP.getFreeHeap());
      // setCase = memCase;
      // sei();
      // attachInterrupt(doutPin, ISR, FALLING);
    }
    else if (bitRead(bleOrWifi, BIT_BLE)) {
      //    noInterrupts();//portDISABLE_INTERRUPTS();//detachInterrupt(doutPin);//, ISR, FALLING);
      cli();

      if (bleOnOff && !bleConnected) {
      /*esp_bt_controller_enable(ESP_BT_MODE_BTDM);
             btStart();
             while (!btStarted());
        Start BLE server
       if (bleInitCounter == 0) {
        */
      initBLE(true);
      /*} else {
          btStart();
         esp_bt_controller_enable(ESP_BT_MODE_BLE);
         delay(100);
         btStart();
         esp_bt_controller_init();
       }
      */
      do {
        bleConnected = esp_bt_controller_get_status();
      } while (!bleConnected);
      /*
       bleInitCounter++;
       Serial.println(bleInitCounter);
      */
      Serial.printf("\n\ndisabling BLE will result in restarting the scale atm. Sorry!\n\n");
      //  Serial.printf("ble stat: %d\n", bleConnected);
      } else if (!bleOnOff && bleConnected) {
        ESP.restart();
        /*
          esp_bluedroid_disable();
          esp_bluedroid_deinit();
          initBLE(false);
          delay(100);
          if (esp_bt_controller_disable() != 0) {
            Serial.println("could not disable BLE");
          } else {
            bleConnected = false;
          }
          delay(100);
          esp_bt_controller_deinit();
          delay(100);
          esp_bt_controller_mem_release(ESP_BT_MODE_BLE);// uncommented in BLEDevice.cpp
          btStop(); // Turn off bluetooth for saving battery
          delay(100);
          bleConnected = false;
          do {
            byte getBLEStat = esp_bt_controller_get_status();//BLEDevice::getInitialized();
            Serial.printf("ble stat reply: %d, bleDevice lib init: %d\n", getBLEStat, BLEDevice::getInitialized());
            if (getBLEStat == 2) bleConnected = false;
            //  bleConnected = 
            delay(250);
          } while (bleConnected);
        */
      }
      Serial.printf("ble stat: %d\n", bleConnected);
      Serial.println(ESP.getFreeHeap());
      // setCase = memCase;
      //    interrupts();//portENABLE_INTERRUPTS();//attachInterrupt(doutPin, ISR, FALLING);
      sei();
    }
  }
}

/**
   scanWiFi
   Scans for available networks
   and decides if a switch between
   allowed networks makes sense

   @return <code>bool</code>
          True if at least one allowed network was found
*/
bool scanWiFi() {
  /** RSSI for primary network */
  int8_t rssiPrim;
  /** RSSI for secondary network */
  int8_t rssiSec;
  /** Result of this function */
  bool result = false;

  Serial.println("Start scanning for networks");

  WiFi.disconnect(true);
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);

  // Scan for AP
  int apNum = WiFi.scanNetworks(false, true, false, 1000);
  if (apNum == 0) {
    Serial.println("Found no networks?????");
    return false;
  }

  byte foundAP = 0;
  bool foundPrim = false;

  for (int index = 0; index < apNum; index++) {
    String ssid = WiFi.SSID(index);
    Serial.println("Found AP: " + ssid + " RSSI: " + WiFi.RSSI(index));
    if (!strcmp((const char*) &ssid[0], (const char*) &ssidPrim[0])) {
      Serial.println("Found primary AP");
      foundAP++;
      foundPrim = true;
      rssiPrim = WiFi.RSSI(index);
    }
    if (!strcmp((const char*) &ssid[0], (const char*) &ssidSec[0])) {
      Serial.println("Found secondary AP");
      foundAP++;
      rssiSec = WiFi.RSSI(index);
    }
  }

  switch (foundAP) {
    case 0:
      result = false;
      break;
    case 1:
      if (foundPrim) {
        usePrimAP = true;
      } else {
        usePrimAP = false;
      }
      result = true;
      break;
    default:
      Serial.printf("RSSI Prim: %d Sec: %d\n", rssiPrim, rssiSec);
      if (rssiPrim > rssiSec) {
        usePrimAP = true; // RSSI of primary network is better
      } else {
        usePrimAP = false; // RSSI of secondary network is better
      }
      result = true;
      break;
  }
  return result;
}

/**
   Start connection to AP
*/
void connectWiFi() {
  Serial.printf("connectWiFi core %d\n ", xPortGetCoreID());
  // Setup callback function for successful connection
  WiFi.onEvent(gotIP, SYSTEM_EVENT_STA_GOT_IP);
  // Setup callback function for lost connection
  WiFi.onEvent(lostCon, SYSTEM_EVENT_STA_DISCONNECTED);

  WiFi.disconnect(true);
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.print("Start connection to ");
  if (usePrimAP) {
    Serial.println(ssidPrim);
    WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
  } else {
    Serial.println(ssidSec);
    WiFi.begin(ssidSec.c_str(), pwSec.c_str());
  }
}

bool statusWIFI() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  } else {
    return true;
  }
}

//interrupt routine:
//IRAM_ATTR tells the complier, that this code Must always be in the 
// ESP32's IRAM, the limited 128k IRAM.  use it sparingly.
void IRAM_ATTR ISR() {
  scale.update();
}

// Get scale reading
void readScale() {
  // operationMode = "single";
  float newAvgWeight = scale.getData();
  if ((newAvgWeight <= oldAvgWeight - 0.25) || (newAvgWeight >= oldAvgWeight + 0.25)) {
    //    sendBLEdata(operationMode, newAvgWeight, NULL);
    Serial.print("\t\taverage:\t");
    Serial.println(newAvgWeight);
    oldAvgWeight = newAvgWeight;
  }
}

void percentageCount(bool _percentSet, bool _keepPercentage, bool _tarePercentage) {
  // operationMode = (inheritCase) ? "formula %":"percentage";
  //  float percentage = 0.0f;
  readScale();
  if (_percentSet) {
    if (_keepPercentage) {
      percentage = quantities[selection];
      keepWeight = oldAvgWeight / (percentage / 100);
      Serial.print(percentage);
      Serial.print("\t100% mass: ");
      Serial.println(keepWeight);
    } else {
      percentage = 100.0f;
      keepWeight = oldAvgWeight;
    }
    if (_tarePercentage) scale.tareNoDelay();
  } else if (!_percentSet && _keepPercentage) {
    if (keepWeight > 0.0f) percentage = oldAvgWeight / keepWeight * 100.0f;
  }
  if (percentage != keepPercent) {
    //    sendBLEdata(operationMode, oldAvgWeight, percentage);
    Serial.print("percent:\t");
    Serial.print(percentage, 2);
    Serial.print("%");
    keepPercent = percentage;
  }
}

void scaleCount(bool _countSet, bool _keepCount) {
  // operationMode = "count";
  int countItems = 0;
  readScale();
  if (_countSet == true) {
    oldCount = 0;
    countItems = 1;
    keepWeight = oldAvgWeight;
  } else if (_countSet == false && _keepCount == true) {
    if ((int)keepWeight > 0) countItems = (int)oldAvgWeight / (int)keepWeight;
  }
  if (countItems != oldCount) {
    //    sendBLEdata(operationMode, oldAvgWeight, countItems);
    Serial.print("counted items:\t");
    Serial.print(countItems);
    oldCount = countItems;
  }
}

void unitSwitch() {
  unitSelect = (unitSelect < (UNIT_STRINGS - 1)) ? (unitSelect + 1) : 0;
}

byte doGet(String _host, uint16_t _port, String _toSend) {
  toSend.remove(0);
  boolean _empty = false; // identifies if the last entry of the spreadsheet was read

  while (!_empty) {
    if (connectionCounter > 4) {
      Serial.println("more than 5 redirects, exiting");
      break;
    }
    connectionCounter++;
    //    if (client->connect(host, httpsPort))
    Serial.printf("URL: %s,\tport: %i,\nuri: %s\n", _host.c_str(), _port, _toSend.c_str());
    if (wificlient.connect(_host.c_str(), _port) == true) // Try to connect Google APIs server at port 443 (https)
    {
      //         = getSheets; // GET string to be sent (retrieve sheet titles)
      //        // Append key to be read from the spreadsheet
      //        toSend += "getScope=sheets";
      //        toSend += key; // Add API key
      _toSend += " HTTP/1.1"; // End of GET instruction

      wificlient.println(_toSend.c_str()); // Send GET to server
      String hostForSend = "Host: ";
      hostForSend += _host.c_str();
      wificlient.println(hostForSend);//"Host: sheets.googleapis.com");
      wificlient.println();

      // Check HTTP status
      // Read response status line
      // ref: https://www.tutorialspoint.com/http/http_responses.htm
      unsigned int statusCode;
      String reasonPhrase;
      String line;

      unsigned int pos = -1;
      unsigned int pos2 = -1;

      // Skip any empty lines
      do {
        line = wificlient.readStringUntil('\n');
      } while (line.length() == 0);

      Serial.println(line);

      pos = line.indexOf("HTTP/1.1 ");
      pos2 = line.indexOf(" ", 9);

      if (!pos) {
        statusCode = line.substring(9, pos2).toInt();
        reasonPhrase = line.substring(pos2 + 1, line.length() - 1);
      }
      else {
        Serial.println("Error! No valid Status Code found in HTTP Response.");
        statusCode = 0;
        reasonPhrase = "";
      }

      switch (statusCode) {
        case 200:
          recievedResponse = true;
          break;

        case 302:
          redirected = true;
          bool flag;
          // Keep reading from the input stream till we get to
          // the location field in the header
          flag = wificlient.find("Location: ");

          if (flag) {
            // Skip URI protocol (http, https, etc. till '//')
            // This assumes that the location field will be containing
            // a URL of the form: http<s>://<hostname>/<url>
            wificlient.readStringUntil('/');
            wificlient.readStringUntil('/');
            // get hostname
            String _redirHost = wificlient.readStringUntil('/');
            // get remaining url
            String _redirUrl = String('/') + wificlient.readStringUntil('\n');

            // Create a GET request for the new location
            //          wificlient.disconnect;
            //              String sendRedirected
            toSend = String ("GET ") + _redirUrl; // GET string to be sent (retrieve sheet titles)
            //          createGetRequest(_redirUrl, _redirHost.c_str());

            Serial.print("_redirHost: ");
            Serial.println(_redirHost);
            Serial.print("_redirUrl: ");
            Serial.println(_redirUrl);

            wificlient.stop();

            if (doGet(_redirHost, httpsPort, toSend) == 0) return 0;
          }
          else {
            Serial.println("No valid 'Location' field found in header!");
          }

          break;

          default:
          Serial.print(F("Unexpected response: "));
          Serial.println(line);
          wificlient.stop();
          connectionCounter = 0;
          return 1;
          break;
      }
      char status[32] = {0};

      // Skip HTTP headers
      char endOfHeaders[] = "\r\n\r\n";
      if (recievedResponse) {
        if (!wificlient.find(endOfHeaders)) {
          if (redirected) {
            redirected = false;
          } else {
            Serial.println(F("Invalid response"));
          }
          recievedResponse = false;
          wificlient.stop();
          connectionCounter = 0;
          return 2;
        } else {
          Serial.println("end of headers");
        }
        wificlient.readBytesUntil('\r', status, sizeof(status));

        // Create buffer
        //        DynamicJsonBuffer jsonBuffer(sheetBufferSize);

        // Store values and convert to float
        toSend = wificlient.readStringUntil('}');
        toSend += '}';
        toSend.remove(0, toSend.indexOf('{'));
        Serial.println(toSend);
        wificlient.stop();
        connectionCounter = 0;
        return 0;
        
            _empty = true;
          }
          else {
            _empty = true;
          }
//        }
//      }
//      spreadsheetBuffer.clear();
      wificlient.stop();  // Stop connection to the server
    }
    else
    {
      Serial.println("Connection failure");
    }
  }
  connectionCounter = 0;
  recievedResponse = false;
}

// Read Google Spreadsheet sheet titles
void readGoogleSheetTitles() {
  //    toSend = getSheets; // GET string to be sent (retrieve sheet titles)
  //    // Append key to be read from the spreadsheet
  //    toSend += "&";
  //    toSend += key;
  toSend = "GET ";
  toSend += SHEET_SCRIPT_URI;
  toSend += "getScope=sheets";
  Serial.println(toSend);
  if (doGet(host, httpsPort, toSend) == 0) {
    //    toSend = "";
    // Store values and convert to float
    JsonObject& root2 = spreadsheetBuffer.parseObject(toSend.c_str());
    if (!root2.success()) {
      Serial.println(F("Parsing failed!"));
      return;
    } else {
      root2.printTo(Serial);
      Serial.println();
      for (byte s = 1; s <= MAX_SHEETS; s++) {
        String index = "Sheet";
        index += String(s);
                 Serial.println(root2[index.c_str()].as<char*>());
        String sheetName = root2[index.c_str()];
                 Serial.printf("%s:%s\n", index.c_str(), sheetName.c_str());
        sheets[s - 1] = sheetName;
                 Serial.println(sheets[s - 1]);
      }

      /*
         "sheets": [
          {
            "properties": {
              "title": "SourdoughCiabatta"
            }
          },
          {
            "properties": {
              "title": "Sourdough20pcRye"
            }
          }
         ]
      */

      // Update display
      if (sheets[0] != "") {
        Serial.println("Response:");
        for (byte t = 0; t < MAX_SHEETS; t++) {
          if (sheets[t] != "") Serial.println(sheets[t]);
        }
        //          display.clear();
        //          display.setTextAlignment(TEXT_ALIGN_CENTER);
        //          display.setFont(ArialMT_Plain_10);
        //          display.drawString(64, 0, String(amount) + currency);
        //          display.setFont(ArialMT_Plain_16);
        //          display.drawString(64, 20, String(value,2) + "BRL");
        //          display.setFont(ArialMT_Plain_10);
        //          display.setTextAlignment(TEXT_ALIGN_CENTER);
        //          display.drawString(64, 48, String(ticker,2) + "BRL"+"/"+currency+" ("+change+")");
        //          display.display();
      }
    }
  } else {
    Serial.print("Response is (null): ");
    Serial.println(toSend);
  }
  spreadsheetBuffer.clear();
  toSend.remove(0);
}

// Read Google Spreadsheet values and update LCD display
void readGoogleSheet() {
  int row = 1; // row of the spreadsheet to be read
  toSend = "GET ";
  toSend += SHEET_SCRIPT_URI;
  toSend += "getScope=values";
  // GET string to be sent (columns from A to E)
//  toSend += "&majorDimension=ROWS";
  toSend += "&sheetID=";
  toSend += selectedSheet;
  toSend += "&startCell=";
  toSend += "B";
  toSend += row;
  toSend += "&endCell=";
  toSend += "F";
  toSend += row + 1;
  Serial.println(toSend);
  if (doGet(host, httpsPort, toSend) == 0) {
    //    toSend = "";
    // Store values and convert to float
    JsonObject& root = spreadsheetBuffer.parseObject(toSend.c_str());
    for (byte m = 0; m < MAX_INGREDIENTS; m++) {
        String _ingredient = root["values"][0][m];
        String _qtyStr = root["values"][1][m];
        ingredients[m] = _ingredient;
        quantities[m] = _qtyStr.toFloat();
        measuredQuantities[m] = 0.0f;
      }

      /*
         {
          "range": "SourdoughCiabatta!B1:F2",
          "majorDimension": "ROWS",
          "values": [
            [
              "Sourdough",
              "Flour",
              "Salt",
              "Water",
              "Olive oil"
            ],
            [
              "62.00%",
              "69.00%",
              "1.25%",
              "43.00%",
              "3.00%"
            ]
          ]
         }
      */

      // Update display
      if (ingredients[0] != "") {
        Serial.println("Response:");
        for (byte n = 0; n < MAX_INGREDIENTS; n++) {
          if (ingredients[n] != "") {
            Serial.print(n + 1);
            Serial.print(". ");
            Serial.print(ingredients[n]);
            Serial.print(":\t\t");
            Serial.println(quantities[n]);
          }
        }

        //          display.clear();
        //          display.setTextAlignment(TEXT_ALIGN_CENTER);
        //          display.setFont(ArialMT_Plain_10);
        //          display.drawString(64, 0, String(amount) + currency);
        //          display.setFont(ArialMT_Plain_16);
        //          display.drawString(64, 20, String(value,2) + "BRL");
        //          display.setFont(ArialMT_Plain_10);
        //          display.setTextAlignment(TEXT_ALIGN_CENTER);
        //          display.drawString(64, 48, String(ticker,2) + "BRL"+"/"+currency+" ("+change+")");
        //          display.display();
      
    }
  } else {
    Serial.print("Response is (null): ");
    Serial.println(toSend);
  }
  spreadsheetBuffer.clear();
  toSend.remove(0);
}

// Pseudo POST function through GET
void send2GoogleSheets() {
  toSend = "GET ";
  toSend += SHEET_SCRIPT_URI;
  toSend += "getScope=getPost";
  toSend += "&sheetID=";
  toSend += selectedSheet;
  for (byte a = 0; a < MAX_INGREDIENTS; a++) {
    if (ingredients[a] != "" && measuredQuantities[a] > 0.0f) {
      toSend += "&tag";
      toSend += a+1;
      toSend += "=";
      toSend += ingredients[a];
      toSend += "&value";
      toSend += a+1;
      toSend += "=";
      toSend += String(measuredQuantities[a], 2);
    }
  }
  Serial.println(toSend);
  if (doGet(host, httpsPort, toSend) == 0) {
    //    toSend = "";
    // Store values and convert to float
    JsonObject& root2 = spreadsheetBuffer.parseObject(toSend.c_str());
    if (!root2.success()) {
      Serial.println(F("POST failed!"));
      return;
    } else {
      root2.printTo(Serial);
      Serial.println();

      // Update display
      if (sheets[0] != "") {
        Serial.println("Response:");
        for (byte t = 0; t < MAX_SHEETS; t++) {
          if (sheets[t] != "") Serial.println(sheets[t]);
        }
      }
    }
  } else {
    Serial.print("Response is (null): ");
    Serial.println(toSend);
  }
  spreadsheetBuffer.clear();
  toSend.remove(0);
}

/**
      menu handlers
      based on u8g2.getMenuEvent()
      should copy and edit for working with touch sensors (or other)

      prototypes and functions in files:
      u8g2.h:uint8_t u8g2_UserInterfaceSelectionList(u8g2_t *u8g2, const char *title, uint8_t start_pos, const char *sl);
      u8x8.h:uint8_t u8x8_UserInterfaceSelectionList(u8x8_t *u8x8, const char *title, uint8_t start_pos, const char *sl);
      u8x8_selection_list.c:uint8_t u8x8_UserInterfaceSelectionList(u8x8_t *u8x8, const char *title, uint8_t start_pos, const char *sl)
      u8g2_selection_list.c:uint8_t u8g2_UserInterfaceSelectionList(u8g2_t *u8g2, const char *title, uint8_t start_pos, const char *sl)

      prototypes and functions for confirmation screen:
      u8g2.h:uint8_t u8g2_UserInterfaceMessage(u8g2_t *u8g2, const char *title1, const char *title2, const char *title3, const char *buttons);
      u8x8.h:uint8_t u8x8_UserInterfaceMessage(u8x8_t *u8x8, const char *title1, const char *title2, const char *title3, const char *buttons);
      u8x8_message.c:uint8_t u8x8_UserInterfaceMessage(u8x8_t *u8x8, const char *title1, const char *title2, const char *title3, const char *buttons)
      u8g2_message.c:uint8_t u8g2_UserInterfaceMessage(u8g2_t *u8g2, const char *title1, const char *title2, const char *title3, const char *buttons)

**/
void buttonCheck(void *parameter)
{
  int8_t button_event = 0;    // set this to 0, once the event has been processed

  struct stateStruct sendState;

  /** buttons **/
  pinMode(LEFT_BUT, INPUT_PULLUP);
  pinMode(RIGHT_BUT, INPUT_PULLUP);
  pinMode(UP_BUT, INPUT_PULLUP);
  pinMode(DOWN_BUT, INPUT_PULLUP);
  pinMode(SELECT_BUT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_BUT), setLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_BUT), setRight, FALLING);
  attachInterrupt(digitalPinToInterrupt(UP_BUT), setUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(DOWN_BUT), setDown, FALLING);
  attachInterrupt(digitalPinToInterrupt(SELECT_BUT), doSelect, CHANGE);

  while (1)
  {
    xQueueReceive(state, &sendState, 0);

    button_event = check_button_event(button_event);
    if (button_event == U8X8_MSG_GPIO_MENU_SELECT) {//select_set == true && (millis() - select_millis) > 10 &&
      if (!inMenu) inMenu = true;
      else {
        ;
      }
    }

    if (button_event == U8X8_MSG_GPIO_MENU_HOME) {//left_set == true && (millis() - left_millis) > 10 &&
      if (inMenu) inMenu = false;
      else {
        unitSwitch();
      }
    }

    /*
    if (button_event == U8X8_MSG_GPIO_MENU_NEXT) {//right_set == true && (millis() - right_millis) > 10 &&
        if (!inMenu) unitSwitch();
        else {
        ;
        }
    }
    */

    if (button_event == U8X8_MSG_GPIO_MENU_UP) {//up_set == true && (millis() - up_millis) > 10 &&
      if (!inMenu) scale.tareNoDelay();
      //    if (!digitalRead(UP_BUT)) {
      //      menuPlace = (menuPlace == 0) ? (NUM_MENU_ITEMS - 1) : (menuPlace - 1);
      //      up_set = false;
      //      Serial.println(menuPlace);
      //    }
    }

    if (button_event == U8X8_MSG_GPIO_MENU_DOWN) {//down_set == true && (millis() - down_millis) > 10 &&
      if (!inMenu) {
        Serial.println("set pressed");
        if (setCase == 1 || setCase == 2) {
          if (baseIngredient || !inheritCase) {
            sendState.countSet = true;
            sendState.keepCount = false;
            Serial.print("case is:\t");
            Serial.print(setCase);
            Serial.println("\tsetting unit 1 / 100%");
          }
          if (inheritCase) {
            measuredQuantities[selection] = oldAvgWeight;
            if (ingredients[selection + 1] != "") selection++;
            else selection = 0;
          }
          sendState.tareNow = true;
        } 
      }
    /*
        if (!digitalRead(DOWN_BUT)) {
          menuPlace = (menuPlace == (NUM_MENU_ITEMS - 1)) ? 0 : (menuPlace + 1);
          down_set = false;
          Serial.println(menuPlace);
        }
    */
    }

    if ( button_event > 0 )  // all known events are processed, clear event
      button_event = 0;

    xQueueSend(state, &sendState, 5);

    vTaskDelay(10); 
    // not quite sure why the delay is needed, it's only here so wdt gets something?
    // perhaps only an Arduino thing?
    // see https://github.com/espressif/arduino-esp32/issues/595
  }
}

void menuManager (void *parameter)
{

  while(1) {
    xEventGroupWaitBits(caseEventGroup, (1 << BIT_MENU), pdTRUE, pdTRUE, portMAX_DELAY);
  }
}

/* Screen display manager task
 * runs in it's own loop, pinned to core 0 in setup()
 * meant to use parameters passed by loop() in a queue
 * currently using only single queue, named queue, and containing oldAvgWeight, percentage, and setCase
 * should probably separate measurements from current working mode, menu on or off, etc.
 * should also use mutex for current working mode (setCase) as it is supposed to be written by both loops
 */
void displayManager (void * parameter)
{
  struct queueStruct receiveQueue;
  String operationMem;
  float avgWeight4Display;

  /* U8g2 Project: SSD1306 Test Board */
  pinMode(SCREEN1_RES, OUTPUT);
  pinMode(SCREEN2_RES, OUTPUT);
  digitalWrite(SCREEN1_RES, 0);
  digitalWrite(SCREEN2_RES, 0);

  u8g2.setI2CAddress(SCREEN1_ADR);
  u8g2.setBusClock(1200000);
  u8g2.begin();
  u8g2.enableUTF8Print();
  dataScreen.setI2CAddress(SCREEN2_ADR);
  dataScreen.setBusClock(1200000);
  dataScreen.begin(/*Select=*/ RIGHT_BUT, /*Right/Next=*/ U8X8_PIN_NONE, /*Left/Prev=*/ U8X8_PIN_NONE, /*Up=*/ UP_BUT, /*Down=*/ DOWN_BUT, /*Home/Cancel=*/ LEFT_BUT);//SELECT_BUT, /*Home/Cancel=*/ A6
  dataScreen.enableUTF8Print();

    /** screen display handler **/
  //  if ((millis() - displayTimer) > 250) {
  while (1) {
    xQueueReceive(queue, &receiveQueue, 100);
    byte localCase = receiveQueue.currentCase;
    if (inMenu) {// set watch point?
        // disableCore0WDT();
        localCase = drawMenu(localCase);
        Serial.printf("setCase = %d\n", localCase);
        // enableCore0WDT();
        int caseEventByte = 0x00;
        // if (localCase == 0);
        if (localCase == 6) caseEventByte |= (1 << BIT_WIFI);
        else if (localCase == 7) caseEventByte |= (1 << BIT_BLE);
        else {
          xQueueOverwrite(controlCase, &localCase);
          caseEventByte |= (1 << BIT_CASE);
        }
        // bitClear(caseEventByte, BIT_MENU);
        Serial.print("send case bits: ");Serial.println(caseEventByte, BIN);
        xEventGroupSetBits(caseEventGroup, caseEventByte);
    }
    else if (operationMem != operationMode || avgWeight4Display != receiveQueue.mainMeasurement) {
      
      // } else {
        operationMem = operationMode;
        avgWeight4Display = receiveQueue.mainMeasurement;
        String _messageInfo = "mode: ";
        _messageInfo.concat(operationMem);
        const char* message1 = (const char*)_messageInfo.c_str();
        String var1 = String(avgWeight4Display, 1);
        String param1 = unitStrings[unitSelect];
        String var2 = "";
        String param2 = "";
        String _message = "mode: ";

        switch (localCase) {
          case 0:
            var2 = var1;
            param2 = param1;
          break;

          case 1:
            var2 = oldCount;
            param2 = "pcs";
          break;

          case 2:
            var2 = percentage;
            param2 = " %";
          break;

          case 3:
          {
            // disableCore0WDT();
            int8_t selectedSheetInt = drawDBData();
            if (selectedSheetInt >= 0) {
            selectedSheet = sheets[selectedSheetInt];
            localCase = 4;
            } else {
            localCase = 0;
            }
            // enableCore0WDT();
          }
          break;
              
          case 4:
          {
            // disableCore0WDT();
            int8_t firstIngredient = drawFormula();
            if (firstIngredient >= 0) {
            selection = firstIngredient;
            mainScreenScroll = true;
            localCase = 2;
            } else {
            localCase = 0;
            } 
            // enableCore0WDT();
          }
          break;
        }
      if (inheritCase) {
          _message = ingredients[selection];
          _message.concat(" (");
          _message.concat(quantities[selection]);
          _message.concat("%)");
      } else {
          _message.concat(operationMem);
      }
      const char* message = (const char*)_message.c_str();
      mainScreenScroll = drawMainScreen(message, mainScreenScroll, var2, param2);
      // if (!inMenu) 
      drawInfoScreen(message1, var1, param1);
      // else {
          // setCase = drawMenu(setCase);
      // }
    }
    vTaskDelay(20);
    // not quite sure why the delay is needed, it's only here so wdt gets something?
    // perhaps only an Arduino thing?
    // see https://github.com/espressif/arduino-esp32/issues/595
  }
}

/*
  Draw a string with specified pixel offset.
  The offset can be negative.
  Limitation: The monochrome font with 8 pixel per glyph
*/
void drawScrollString(U8G2_SSD1306_128X64_NONAME_2_HW_I2C &screen0, int16_t offset, const char *s)
{
  static char buf[36];  // should for screen with up to 256 pixel width
  size_t len;
  size_t char_offset = 0;
  u8g2_uint_t dx = 0;
  size_t visible = 0;
  len = strlen(s);
  if ( offset < 0 )
  {
    char_offset = (-offset) / 8;
    dx = offset + char_offset * 8;
    if ( char_offset >= screen0.getDisplayWidth() / 8 )
      return;
    visible = screen0.getDisplayWidth() / 8 - char_offset + 1;
    strncpy(buf, s, visible);
    buf[visible] = '\0';
    screen0.setFont(u8g2_font_8x13_mf);
    screen0.drawStr(char_offset * 8 - dx, 13, buf);
  }
  else
  {
    char_offset = offset / 8;
    if ( char_offset >= len )
      return; // nothing visible
    dx = offset - char_offset * 8;
    visible = len - char_offset;
    if ( visible > screen0.getDisplayWidth() / 8 + 1 )
      visible = screen0.getDisplayWidth() / 8 + 1;
    strncpy(buf, s + char_offset, visible);
    buf[visible] = '\0';
    screen0.setFont(u8g2_font_8x13_mf);
    screen0.drawStr(-dx, 13, buf);
  }
}

void drawHeader(U8G2_SSD1306_128X64_NONAME_2_HW_I2C &screen1, const char *s) {
  // Serial.printf("drawHeader core %d\n ", xPortGetCoreID());
  screen1.setFont(u8g2_font_8x13_mf);
  screen1.drawStr(0, 13, s);
}

void drawIcons(U8G2_SSD1306_128X64_NONAME_2_HW_I2C &screen) {
  screen.setFont(u8g2_font_open_iconic_embedded_1x_t);
  byte iconWidth = screen.getMaxCharWidth();//8;
  bool wifiConnected = statusWIFI();
  if (wifiConnected) {
    byte wifiPlace = screen.getDisplayWidth() - iconWidth;
    screen.drawGlyph(wifiPlace, 13, 80);
  }
  if (bleConnected) {
    if (wifiConnected) iconWidth *= 2;
    byte blePlace = screen.getDisplayWidth() - iconWidth;
    screen.drawGlyph(blePlace, 13, 74);
  }
}

void drawWeather(U8G2_SSD1306_128X64_NONAME_2_HW_I2C &screen2, String symbol, String degree)
{
  //  drawWeatherSymbol(0, 48+16, symbol);
  screen2.setFont(u8g2_font_logisoso32_tf);
  screen2.setCursor(0, 42 + 16);
  screen2.print(symbol);
  screen2.setFont(u8g2_font_logisoso16_tf);
  screen2.setCursor(127 - 19, 42 + 16);
  screen2.print(degree);
  //  u8g2.print("°C");    // requires enableUTF8Print()
}

bool drawMainScreen(const char *s, bool scroll, String weight, String unit)
{
  int16_t len = strlen(s);
  int16_t offset = - ((int16_t)len * 8);//(int16_t)u8g2.getDisplayWidth();
  if ((len * 8) > (int16_t)u8g2.getDisplayWidth() && scroll) {
    for (;;)
    {
      u8g2.firstPage();
      do {
        drawScrollString(u8g2, offset, s);
        drawWeather(u8g2, weight, unit);
      } while ( u8g2.nextPage() );
      //    dataScreen.firstPage();
      //    do {
      //      drawScrollString(offset, s);
      //      drawWeather(weight, unit);
      //    } while ( dataScreen.nextPage() );
      delay(5);
      offset += 2;
      if ( offset > len + 1 )
        break;
    }
  } else {
    u8g2.firstPage();
    do {
      drawHeader(u8g2, s);
      drawWeather(u8g2, weight, unit);
    } while ( u8g2.nextPage() );
  }
  return false;
}

void drawInfoScreen(const char *s, String weight, String unit)
{
  dataScreen.firstPage();
  do {
    drawHeader(dataScreen, s);
    drawIcons(dataScreen);
    drawWeather(dataScreen, weight, unit);
  } while ( dataScreen.nextPage() );
}

/** keypad check function **/
int8_t check_button_event(int8_t button_event)
{
  if ( button_event == 0 )
    button_event = dataScreen.getMenuEvent();

  return button_event;
}

int8_t drawFormula() {
  String _menu = "";
  if (ingredients[0] != "") {
    for (byte t = 0; t < MAX_INGREDIENTS; t++) {
      if (ingredients[t] != "") {
        if (t != 0) _menu.concat("\n");
        _menu.concat(ingredients[t]);
      }
    }
    const char* menu = (const char*)_menu.c_str();
    dataScreen.setFont(u8g2_font_8x13_mf);
    dataScreen.setCursor(0, 5);
    dataScreen.firstPage();
    do {
      uint8_t menuPlace = 1;
      menuPlace = dataScreen.userInterfaceSelectionList("Choose 1st ing.:", menuPlace, menu);
      Serial.print("Menu button pressed:\t");
      Serial.println(menuPlace);
      if (menuPlace == 0) {
        inMenu = false;
        return -1;
      } else {
        inMenu = false;
        return(menuPlace - 1);
      }
    } while ( dataScreen.nextPage() );
  } else {
    Serial.println("No ingredients to choose from");
    return -2;
  }
}

int8_t drawDBData() {
  String _menu = "";
  if (sheets[0] != "") {
    for (byte t = 0; t < MAX_SHEETS; t++) {
      if (sheets[t] != "") {
        if (t != 0) _menu.concat("\n");
        _menu.concat(sheets[t]);
      }
    }
    const char* menu = (const char*)_menu.c_str();
    dataScreen.setFont(u8g2_font_8x13_mf);
    dataScreen.setCursor(0, 5);
    dataScreen.firstPage();
    do {
      uint8_t menuPlace = 1;
      menuPlace = dataScreen.userInterfaceSelectionList("Choose formula:", menuPlace, menu);
      Serial.print("Menu button pressed:\t");
      Serial.println(menuPlace);
      if (menuPlace == 0) {
        inMenu = false;
        return -1;
      } else {
        inMenu = false;
        return(menuPlace - 1);
      }
    } while ( dataScreen.nextPage() );
  } else {
    Serial.println("No formulas to choose from");
    return -2;
  }
}

byte drawMenu(byte menuPlace) {
  inheritCase = false;
  menuPlace++;
  String _menu = "";// menu_items[0];
  for (byte i = 0; i < (NUM_MENU_ITEMS); i++) {
    if (i != 0) _menu.concat("\n");
    _menu.concat(menu_items[i]);
  }
  if (!statusWIFI()) _menu.concat("\nturn WiFi on\n");
  else _menu.concat("\nturn WiFi off\n");
  if (!bleOnOff) _menu.concat("turn BLE on");
  else _menu.concat("turn BLE off (reset)");
  const char* menu = (const char*)_menu.c_str();
  dataScreen.setFont(u8g2_font_8x13_mf);
  dataScreen.setCursor(0, 5);
  dataScreen.firstPage();
  do {
    uint8_t menuMem = menuPlace;
    menuPlace = dataScreen.userInterfaceSelectionList("Mode", menuPlace, menu);
    Serial.print("Menu button pressed:\t");
    Serial.println(menuPlace);
    if (menuPlace == 0) {
      menuPlace = menuMem;
      inMenu = false;
      return(menuPlace - 1);
    } else {
      uint8_t selectionSet = dataScreen.userInterfaceMessage(
                               "Selection:",
                               u8x8_GetStringLineStart(menuPlace - 1, menu),
                               "",
                               " ok \n cancel ");
      if (selectionSet == 1) {
        // setCase = menuPlace - 1;
        inMenu = false;
        return(menuPlace - 1);
        break;
      } 
    }
  } while ( dataScreen.nextPage() );
}

/** button ISRs **/
void setLeft() {
  left_set = true;
  left_millis = millis();
}

void setRight() {
  right_set = true;
  right_millis = millis();
}

void setUp() {
  up_set = true;
  up_millis = millis();
}

void setDown() {
  down_set = true;
  down_millis = millis();
}

void doSelect() {
  select_set = true;
  select_millis = millis();
}

void helpScreen() {
  Serial.println(ESP.getFreeHeap());
  Serial.println("This is the help screen");
  Serial.println("Send these values via terminal to activate desired functions, or 'h' for this help screen:");
  Serial.println("t\t -\t tare");
  Serial.println("s\t -\t single measurement mode");
  Serial.println("c\t -\t count items");
  Serial.println("p\t -\t baker's percentage mode");
  Serial.println("r\t -\t get sheet titles from google sheets");
  Serial.println("ble\t -\t strat BLE (if already started will result in restart)");
  Serial.println("wifi\t -\t start/stop WiFi connection to local network (needed for google sheets read/write services)");
  Serial.println("or type known recipe name and send (if connected to WiFi)");
}
