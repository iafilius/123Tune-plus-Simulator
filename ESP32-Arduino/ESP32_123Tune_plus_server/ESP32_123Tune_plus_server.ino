// beginnetje gemaakt voor 123Tune port van Linux naar ESP32

#include <esp_task_wdt.h>
#include <esp_int_wdt.h>
#include <core_version.h>
#include <sys/time.h>
#include <esp_gap_ble_api.h>

#define S_SPEED 115200

#define C_nr_123MapCurve_Elements 10    // # of elements/size of arrays
#define C_nr_123AdvanceCurve_Elements 10    // # of elements/size of arrays
/*
  class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
  };
*/
/*
  static uint8_t manufacturer_data[6] = {0xE5,0x02,0x01,0x01,0x01,0x01};
  static esp_ble_adv_data_t scan_rsp_data = {
  .set_scan_rsp = true,
  .manufacturer_len = 6,
  .p_manufacturer_data = manufacturer_data,

  };
*/

/* custom uuid */
/*
  static uint8_t service_uuid128[32] = {
    / LSB <--------------------------------------------------------------------------------> MSB /
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0x00, 0x00,
  };
*/
/*
  static esp_ble_adv_data_t test_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
  };

  //this variable holds advertising parameters
  esp_ble_adv_params_t test_adv_params;
*/

// Timer code from
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Timer/RepeatTimer/RepeatTimer.ino
//
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile SemaphoreHandle_t BLE_RX2TX_Semaphore;

//
TaskHandle_t BLERX2TX_Handle = NULL;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void IRAM_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}


void BLE_timer_start(void) {

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 1000000, true);
  //timerAlarmWrite(timer, 700000, true);

  // Start an alarm
  timerAlarmEnable(timer);
}

void BLE_timer_stop(void) {
  // Stop and free timer
  timerEnd(timer);
  timer = NULL;
  // Enough disabled???
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(S_SPEED);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB.
  }

  Serial.print("Arduino IDE version: "); Serial.println( ARDUINO);
  Serial.print("ESP32-Arduino version: "); Serial.println(ARDUINO_ESP32_RELEASE);
  Serial.print("ESP-IDF Version: "); Serial.println(ESP.getSdkVersion());
  Serial.print("tskKERNEL_VERSION: "); Serial.println(tskKERNEL_VERSION_NUMBER);

  Serial.println( "Compiled: " __DATE__ ", " __TIME__ ", Compiler: " __VERSION__);
  Serial.print("Compiled for board; ");  Serial.println(ARDUINO_BOARD);


  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // semapore for starting TX triggered by RX
  BLE_RX2TX_Semaphore = xSemaphoreCreateBinary();
  // This should be right available, before BLE_RX_Responder_to_TX Task was started, or BLE process was initialized
  //if (xSemaphoreTake(BLE_RX2TX_Semaphore, portMAX_DELAY) == pdTRUE){
  //  Serial.println("BLE_RX2TX_Semaphore is set to Taken, to be released by BLE RX process");
  //}

  Serial.println("Starting BLE work!");

  Serial.print("ESP.getFreeHeap (before BLE): ");
  Serial.println(ESP.getFreeHeap());


  // Create a seperate task for handling requests received ion RX UUID to be trantmitten on TX UUID
  // Known bug in BLE library
  // https://github.com/espressif/arduino-esp32/issues/1982
  xTaskCreate(
    BLE_RX_Responder_to_TX,       /* Task function. */
    "BLERX2TX",     /* String with name of task. */
    10000,             /* Stack size in bytes (for ESP32), RTOS standard is word. */
    NULL,              /* Parameter passed as input of the task */
    0,                 /* Priority of the task. */
    &BLERX2TX_Handle);            /* Task handle. may be NULL*/


  TuneServer_init();   // Set valid values to start with in internal state
  InitBLE();
  //bpm = 1;

  Serial.print("ESP.getFreeHeap (end setup): ");
  Serial.println(ESP.getFreeHeap());

  //esp32_wdt_setup(8000);    // 4 seconds
  Serial.println("Watchdog started with timeout of 8000 ms");

  //BLE_timer_start();    // test timer
}

void loop() {
  // put your main code here, to run repeatedly:
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    // Print it
   // Serial.print("onTimer no. ");
   // Serial.print(isrCount);
   // Serial.print(" at ");
   // Serial.print(isrTime);
   // Serial.println(" ms");
    // xSemaphoreGiveFromISR(timerSemaphore, NULL); in executed in timer ISR

    // Calling BLE msrmt callback on a regular basis (engine live data)
    tune_msrmt_cb();
    //fake_resp1();
  }
  //fake_resp1();

  //delay(10000);
  //tune_msrmt_cb();
  //update_BLE_values();
  delay(100);
}
