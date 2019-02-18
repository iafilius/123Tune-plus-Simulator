// https://raw.githubusercontent.com/nkolban/ESP32_BLE_Arduino/master/examples/BLE_server/BLE_server.ino

// BLE server stuff (BLE Server example)
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <BLE2902.h>
#include <BLEAdvertising.h>

#include <sstream>
#include <string>


/* LSB <--------------------------------------------------------------------------------> MSB */
//#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
//#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
//#define TUNE123_SERVICE_UUID_c_str          "796022a0-beaf-c0bd-de48-7962f1842bda"
#define TUNE123_SERVICE_UUID_c_str            "da2b84f1-6279-48de-bdc0-afbea0226079"
 #define TUNE123_Info_Characeristic_UUID_c_str               "99564A02-DC01-4D3C-B04E-3BB1EF0571B2"
 #define TUNE123_Body_Characeristic_UUID_c_str               "A87988B9-694C-479C-900E-95DFA6C00A24"
 #define TUNE123_Ctrl_RX_Characeristic_UUID_c_str               "BF03260C-7205-4C25-AF43-93B1C299D159"
 #define TUNE123_Ctrl_TX_Characeristic_UUID_c_str               "18CDA784-4BD3-4370-85BB-BFED91EC86AF"   // TX, Notify

#define TUNE123_BATTERY_Service_UUID_uint16                       0x180F
 #define TUNE123_BATTERY_Level_Characeristic_UUID_uint16           0x2A19


#define P123TUNEPlus_Company_ID  0x8500   // 2 octets, reverse engineered like the rest, Id of BlueRadio's
//#define P123TUNEPlus_Device_ID   0xF040   // 2 octets, decimal as shown in device #61504
#define P123TUNEPlus_Device_ID   0xF03F   // 2 octets, decimal as shown in device #61503

BLEAdvertisementData advertisementData;
BLEAdvertisementData scan_resp_advertisementData;


//BLEServer* BLE_Server_p = NULL;
//BLECharacteristic* pCharacteristic = NULL;

byte flags = 0b00111110;
byte bpm;
byte heart[8] = { 0b00001110, 60, 0, 0, 0 , 0, 0, 0};
byte hrmPos[1] = {2};
byte BLE_RW_Demo = 0;
bool _BLEClientConnected = false;

//#define Tune123_Service_BLEUUID BLEUUID((uint16_t)0x180D)
//#define TuneService BLEUUID("")
// Service UUID's
static BLEUUID    Tune123_Service_BLEUUID(TUNE123_SERVICE_UUID_c_str);
static BLEUUID    Tune123_Battery_Service_BLEUUID((uint16_t)TUNE123_BATTERY_Service_UUID_uint16);

// Characteristic UUID's
static BLEUUID    TUNE123_Info_Characeristic_UUID(TUNE123_Info_Characeristic_UUID_c_str);
static BLEUUID    TUNE123_Body_Characeristic_UUID(TUNE123_Body_Characeristic_UUID_c_str);
static BLEUUID    TUNE123_Ctrl_RX_Characeristic_UUID(TUNE123_Ctrl_RX_Characeristic_UUID_c_str);
static BLEUUID    TUNE123_Ctrl_TX_Characeristic_UUID(TUNE123_Ctrl_TX_Characeristic_UUID_c_str);

BLECharacteristic heartRateMeasurementCharacteristics(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A38), BLECharacteristic::PROPERTY_READ);
BLECharacteristic variabele_demo_Characteristic(BLEUUID((uint16_t)0x2A39), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic info_Characteristic(TUNE123_Info_Characeristic_UUID, BLECharacteristic::PROPERTY_READ);
BLECharacteristic body_Characteristic(TUNE123_Body_Characeristic_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE );
BLECharacteristic ctrl_RX_Characteristic(TUNE123_Ctrl_RX_Characeristic_UUID, BLECharacteristic::PROPERTY_WRITE );
BLECharacteristic ctrl_TX_Characteristic(TUNE123_Ctrl_TX_Characeristic_UUID, BLECharacteristic::PROPERTY_NOTIFY );


BLECharacteristic TuneBattery_Level_Characteristic(BLEUUID((uint16_t)TUNE123_BATTERY_Level_Characeristic_UUID_uint16), BLECharacteristic::PROPERTY_READ);


//BLEDescriptor heartRateDescriptor(BLEUUID((uint16_t)0x2901));
//static BLEUUID serviceUUID(TUNE123_SERVICE_UUID_c_str); //128 bits

BLEDescriptor heartRateDescriptor(Tune123_Service_BLEUUID);

BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor variabele_demoDescriptior(BLEUUID((uint16_t)0x2901));
BLEDescriptor infoDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor bodyDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor ctrl_RX_Descriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor ctrl_TX_Descriptor(BLEUUID((uint16_t)0x2901));

BLEDescriptor TuneBatteryDescriptior(BLEUUID((uint16_t)0x2901));


class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* BLE_Server_p) {
      _BLEClientConnected = true;
      Serial.println("onConnect");
      BLE_timer_start();
    };

    void onDisconnect(BLEServer* BLE_Server_p) {
      _BLEClientConnected = false;
      Serial.println("onDisconnect");
      BLE_timer_stop();
    }
};

// RX&TX on same characteristic
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      Serial.print("MyCallbacks onWrite: ");Serial.println(pCharacteristic->getUUID().toString().c_str());

      if (pCharacteristic->getUUID().equals(TUNE123_Info_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onWrite info_Characteristic ");
      }

      if ( (pCharacteristic->getUUID()).equals(TUNE123_Body_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onWrite body_Characteristic: ");
        // predefined/reverse engineered answer to make client happy
        uint8_t body_mode = 0x01;
        pCharacteristic->setValue(&body_mode, sizeof(body_mode));
        return;
      }

      if ( (pCharacteristic->getUUID()).equals(TUNE123_Ctrl_RX_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onWrite ctrl_RX_Characteristic: ");
        // predefined/reverse engineered answer to make client happy
        //uint8_t body_mode = 0x01;
        //pCharacteristic->setValue(&body_mode, sizeof(body_mode));
        return;
      }

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i],HEX);

        Serial.println();
        Serial.println("*********");
      }
    }
    void onRead(BLECharacteristic *pCharacteristic) {
      struct timeval tv;
      Serial.print("MyCallbacks onRead: ");Serial.println(pCharacteristic->getUUID().toString().c_str());

      if ( (pCharacteristic->getUUID()).equals(TUNE123_Info_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onRead info_Characteristic: ");
        // predefined/reverse engineered answer to make client happy
        uint8_t body_info[] = { 0x03, 0x00, 0x02, 0x18, 0x00, 0x1a, 0x00, 0x1c, 0x00, 0x1d, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x22, 0x00};
        pCharacteristic->setValue(body_info, sizeof(body_info));
        return;
      }

      if ( (pCharacteristic->getUUID()).equals(TUNE123_Body_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onRead body_Characteristic: ");
        // predefined/reverse engineered answer to make client happy
        uint8_t body_mode = 0x01;
        pCharacteristic->setValue(&body_mode, sizeof(body_mode));
        return;
      }

      gettimeofday(&tv, NULL);
      std::ostringstream os;
      os << "Time: " << tv.tv_sec;
      Serial.println("Callback read: ");
      //Serial.println((String(os.str()));
      pCharacteristic->setValue(os.str());
    }
};



// No duplex over This specific characteristic
// RX: requests received
// TX: responses send
class MyCallbacks_CTRL_RX_TX: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      Serial.print("MyCallbacks_CTRL_RX_TX onWrite: ");Serial.println(pCharacteristic->getUUID().toString().c_str());

      if (pCharacteristic->getUUID().equals(TUNE123_Ctrl_RX_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onWrite TUNE123_Ctrl_RX_Characeristic_UUID ");
      }
      if (pCharacteristic->getUUID().equals(TUNE123_Ctrl_TX_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onWrite TUNE123_Ctrl_TX_Characeristic_UUID ");
      }


      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i],HEX);

        Serial.println();
        Serial.println("*********");
      }
    }
    void onRead(BLECharacteristic *pCharacteristic) {
      struct timeval tv;
      Serial.print("MyCallbacks_CTRL_RX_TX onRead: ");Serial.println(pCharacteristic->getUUID().toString().c_str());

      if (pCharacteristic->getUUID().equals(TUNE123_Ctrl_RX_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onRead TUNE123_Ctrl_RX_Characeristic_UUID ");
      }
      if (pCharacteristic->getUUID().equals(TUNE123_Ctrl_TX_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onRead TUNE123_Ctrl_TX_Characeristic_UUID ");
      }



      gettimeofday(&tv, NULL);
      std::ostringstream os;
      os << "Time: " << tv.tv_sec;
      Serial.println("Callback read: ");
      //Serial.println((String(os.str()));
      pCharacteristic->setValue(os.str());
    }
};


// only to be called form MyCallbacksS_CTRL_RX_TX
// reads value from BLE stack itself, to prevent unneeded copies as paramters and/or queues
// if (pCharacteristic->getUUID().equals(TUNE123_Ctrl_RX_Characeristic_UUID))
void BLE_RX_Responder_to_TX(void* parameter) {

 for (;;) {
       if (xSemaphoreTake(BLE_RX2TX_Semaphore, portMAX_DELAY) == pdTRUE){

      //std::string value_std_str = pCharacteristic->getValue();
      //size_t len=value_std_str.length();
      //uint8_t *value = (uint8_t*)value_std_str.c_str(); // to fix uint8_t vs char mismatch afer porting in one go
      //        server->ctrl_TX_Characteristic->setValue(str_resp, str_resp_len);
      //        server->ctrl_TX_Characteristic->notify();

      server_s_t* server=&server_global;
      int rx_cmd_found=0;   // counts the cases it is recognized and responded to. and if zero detect unrecognized command.

      std::string value_std_str= server->ctrl_RX_Characteristic->getValue();
      size_t len=value_std_str.length();
      uint8_t *value = (uint8_t*)value_std_str.c_str(); // to fix uint8_t vs char mismatch afer porting in one go

       Serial.print("BLE_RX_Responder_to_TX wakeup: RXUUID:"); Serial.println(server->ctrl_RX_Characteristic->getUUID().toString().c_str());
       Serial.print("BLE_RX_Responder_to_TX wakeup: TXUUID:"); Serial.println(server->ctrl_TX_Characteristic->getUUID().toString().c_str());
       Serial.print("Read: ");print_hex(value,len);
       // given in MyCallbacksS_CTRL_RX_TX
       //xSemaphoreGive(BLE_RX2TX_Semaphore);

        { // begin 123response logic


          //struct server_s_t *server = (server_s_t*)user_data;
          uint8_t ecode = 0;


          uint8_t *value_ptr;       // sometimes a 0x24 is added/prepended, need to filter out
          size_t len_cleaned;     // sometimes a 0x24 is added/prepended, need to filter out

          len_cleaned = len;
          value_ptr = value ;

          //delay(100); // bug in BLE??
          while (len_cleaned > 0 && value_ptr[0] == 0x24)
          {
            //printf("Cleaned prepending 0x24 from received command: ");
            //print_hex(value_ptr, len_cleaned);
            // No longer interested in printing these keepalives

            value_ptr++;
            len_cleaned--;
          }

          // need to clean tailing 0x24 's tooo (due to new 123 tune software??/new conditions) ???
          // only try to clean single 0x24 trailing item
          // 2019/01/19 fixup
          if(len_cleaned> 1){
            if(value_ptr[len_cleaned-1]== 0x24)len_cleaned--;
          }

          // there might be a second 0x24 in some cases (pincode change), fixup 2019/01/28
          if(len_cleaned> 1){
            if(value_ptr[len_cleaned-1]== 0x24)len_cleaned--;
          }

          if (len_cleaned == 0)
            goto done;


          //ESP_LOGI(TAG, "DBG: tune_control_point_write_cb: %lu\n",len);
          //print_hex(value,len);

          /* Accept Any length to make 123tune app happy
            if (!value || len != 20) {
            ecode = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
            goto done;
            }
          */
          // Disabled 2019/01/13 on poritn to arduino
          /*
          if (offset) {
            ecode = BT_ATT_ERROR_INVALID_OFFSET;
            goto done;
          }
          */

          if (value_ptr[0] == 1) {
            ESP_LOGI(TAG, "HR: Energy Expended value reset\n");
            server->tune_energy_expended = 0;
          }

          // send confirmation:
done:
          // 2019/01/13 commented for porting later on
          //  gatt_db_attribute_write_result(attrib, id, ecode);
          ;


          // Additional reply on the txUUID below:


          // Start notify as response
          {
            uint8_t str[] = { 0x0d };
            //uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
            uint8_t str_resp[] = { 0x0d };

            size_t str_len = sizeof(str);
            size_t str_resp_len = sizeof(str_resp);
            size_t i;
            int equal = 0;
            if (len_cleaned == str_len)
            {
              equal = 1;
              for (i = 0 ; i < len_cleaned; i++)
                if (value_ptr[i] != str[i])
                  equal = 0;
            }
            if (equal == 1)
            {
              rx_cmd_found++;
              printf("%s 1th Command Found----\n", __FUNCTION__);
              print_hex(value_ptr, len_cleaned);    // RX (Cleaned)
              print_hex(str_resp, str_resp_len);    // TX

              // produce an answer... on txUUID
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               str_resp, str_resp_len);
              */
              server->ctrl_TX_Characteristic->setValue(str_resp, str_resp_len);
              //server->ctrl_TX_Characteristic->setNotifyProperty(true);
              server->ctrl_TX_Characteristic->notify();
              //server->ctrl_TX_Characteristic->indicate();
              //server->ctrl_RX_Characteristic->write();

            }

          }

          // send resuest
          {
            //uint8_t str[] = { 0x24, 0x24, 0x0d };
//            uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };   // working 2018
            uint8_t str[] = { 0x31, 0x30, 0x40, 0x0d };   // '10@\n' command

            //uint8_t str_resp[] = { 0x0d };
            //uint8_t str_resp1[] = {0x0d, 0x31, 0x30, 0x40, 0x0d, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x30, 0x41, 0x20, 0x30, 0x30, 0x20, 0x30, 0x45, 0x20};
            //uint8_t str_resp2[] = {0x30, 0x30, 0x20, 0x31, 0x30, 0x20, 0x30, 0x30, 0x20, 0x31, 0x34, 0x20, 0x31, 0x34, 0x20, 0x31, 0x45, 0x20, 0x33, 0x43};
            //uint8_t str_resp3[] = {0x20, 0x32, 0x34, 0x20, 0x35, 0x35, 0x20, 0x33, 0x43, 0x20, 0x39, 0x36, 0x20, 0x31, 0x30, 0x33, 0x46, 0x33, 0x0d};

            /* Trying to fragmentate did not work!
              uint8_t str_resp1[] = {0x0d, 0x31, 0x30, 0x40, 0x0d, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x30, 0x41, 0x20, 0x30, 0x30, 0x20, 0x30, 0x45, 0x20,
                       0x30, 0x30, 0x20, 0x31, 0x30, 0x20, 0x30, 0x30, 0x20, 0x31, 0x34, 0x20, 0x31, 0x34, 0x20, 0x31, 0x45, 0x20, 0x33, 0x43,
                       0x20, 0x32, 0x34, 0x20, 0x35, 0x35, 0x20, 0x33, 0x43, 0x20, 0x39, 0x36, 0x20, 0x31, 0x30, 0x33, 0x46, 0x33, 0x0d};
            */

            size_t str_len = sizeof(str);
            size_t i;
            int equal = 0;

            if (len_cleaned == str_len)
            {
              equal = 1;
              for (i = 0 ; i < len_cleaned; i++)
                if (value_ptr[i] != str[i])
                  equal = 0;
            }
            if (equal == 1)
            {
              uint8_t buffer[3 * 20]; // 3x 20 bytes (1 or 2 bytes extra room);
              char csum_hex_str[5];   // 3 bytes used;
              unsigned int bytecounter;
              unsigned int local_bytecounter;
              uint8_t all_initial_values[][3] = { // the actual value without header/slot
                { 0x46, 0x46, 0x20 },   // [0]
                { 0x46, 0x46, 0x20 },   // [1]
                { 0x30, 0x41, 0x20 },   // [2]  nr1 RPM     FIXED
                { 0x30, 0x30, 0x20 },   // [3]  nr1 advance (not fixed)
                { 0x30, 0x45, 0x20 }, // [4]  nr2 RPM
                { 0x30, 0x30, 0x20 },   // [5]  nr2 advance
                { 0x31, 0x30, 0x20 },   // [6]  nr3 RPM
                { 0x30, 0x30, 0x20 },   // [7]  nr3 advance
                { 0x31, 0x34, 0x20 },   // [8]  nr4 RPM
                { 0x31, 0x34, 0x20 },   // [9]  nr4 advance
                { 0x31, 0x45, 0x20 },   // [10] nr5 RPM
                { 0x33, 0x43, 0x20 },   // [11] nr5 advance
                { 0x32, 0x34, 0x20 },   // [12] nr6 RPM
                { 0x35, 0x35, 0x20 },   // [13] nr6 advance
                { 0x33, 0x43, 0x20 },   // [14] nr7 RPM
                { 0x39, 0x36, 0x20 }  // [15] nr7 advance
              };
              int csum;   // 3 byte checksum

              rx_cmd_found++;
              printf("%s 2th \'10@\\n\' Command Found----\n", __FUNCTION__);

              // discover/test fields
              //all_initial_values[0][0]=0x31;
              //all_initial_values[0][1]=0x41;
              //all_initial_values[0][0]=0x31;
              //all_initial_values[0][1]=0x41;

              // produce an answer...

              // fill graph under cmd2 with custom values.
              // Fixed 500 RPM
              //str_resp1[11]=server->tune_AdvanceCurveRPM[0][0]; // Fixed 500 RPM
              //str_resp1[12]=server->tune_AdvanceCurveRPM[0][1];

              // overwrite captrued advance with custom value, calculate csum later.
              //all_initial_values[3][0] = server->tune_AdvanceCurveDegrees[0][0];
              //all_initial_values[3][1] = server->tune_AdvanceCurveDegrees[0][1];

              // copy internal state tables to 123Tune bloothooth protocol format
              // n=1 .. 7, array offset=2
              for(int n=1,i=2;n<=7;n++) {
                all_initial_values[i][0]=server->tune_AdvanceCurveRPM[n-1][0];
                all_initial_values[i++][1]=server->tune_AdvanceCurveRPM[n-1][1];
                all_initial_values[i][0]=server->tune_AdvanceCurveDegrees[n-1][0];
                all_initial_values[i++][1]=server->tune_AdvanceCurveDegrees[n-1][1];
              }

              // Fill complete unfragmented buffer
              bytecounter = 0;
              // copy header @ start
              for (local_bytecounter = 0; local_bytecounter < sizeof(str); local_bytecounter++, bytecounter++) {
                buffer[bytecounter] = str[local_bytecounter];
              }
              // copy values (from sniffer for now)
              for (i = 0; i < (sizeof(all_initial_values) / 3); i++ ) {
                buffer[bytecounter++] = all_initial_values[i][0]; local_bytecounter++;
                buffer[bytecounter++] = all_initial_values[i][1]; local_bytecounter++;
                buffer[bytecounter++] = all_initial_values[i][2]; local_bytecounter++;
              }

              // copy ID  from request str: 0x31 0x30+sequence 0x40=16 (data len)
              buffer[bytecounter++] = str[1]; // skip first byte in cmd2, second, third 2 bytes
              buffer[bytecounter++] = str[2];

              csum = 0;
              // calculate checkum, could be integrated in buffer copy above for efficienty, but not for readability.
              for (i = 0; i < sizeof(all_initial_values) / 3; i++) {
                char str_l[20];

                str_l[0] = all_initial_values[i][0];
                str_l[1] = all_initial_values[i][1];
                str_l[2] = '\0';
                csum += strtol(str_l, NULL, 16);
              }
              sprintf(csum_hex_str, "%03X", csum);

              // fill csum
              buffer[bytecounter++] = csum_hex_str[0];
              buffer[bytecounter++] = csum_hex_str[1];
              buffer[bytecounter++] = csum_hex_str[2];

              // add termination 0x0d
              buffer[bytecounter++] = 0x0d; // bytecounter containts the total number of bytes

              printf("%s bytecounter=%d csum=%s\n", __FUNCTION__, bytecounter, csum_hex_str);

              //printf("%s 2th Command Found----\n", __FUNCTION__);
              print_hex(value_ptr, len_cleaned);

              // size is 59, fist 20 bytes fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[0], 20);
              */
              print_hex(&buffer[0], 20);
              server->ctrl_TX_Characteristic->setValue(&buffer[0], 20);
              server->ctrl_TX_Characteristic->notify();

              bytecounter -= 20;

              // second 20 bytes fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[20], 20);
              */
              print_hex(&buffer[20], 20);
              server->ctrl_TX_Characteristic->setValue(&buffer[20], 20);
              server->ctrl_TX_Characteristic->notify();

              bytecounter -= 20;
              // last fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[40], bytecounter );
              */
              print_hex(&buffer[40], bytecounter);
              server->ctrl_TX_Characteristic->setValue(&buffer[40], bytecounter);
              server->ctrl_TX_Characteristic->notify();



            }

          }



          {
            //uint8_t str[] = { 0x24, 0x24, 0x0d };
            //uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
//            uint8_t str[] = {  0x31, 0x31, 0x40, 0x0d };    // was working in 2018
            uint8_t str[] = {  0x31, 0x31, 0x40, 0x0d  }; // '11@\n' command

            //uint8_t str_resp[] = { 0x0d };fffffffffffffff
            //uint8_t str_resp1[] = { 0x31, 0x31, 0x40, 0x0d, 0x35, 0x41, 0x20, 0x41, 0x35, 0x20, 0x41, 0x30, 0x20, 0x41, 0x35, 0x20, 0x46, 0x46, 0x20, 0x46 };
            //uint8_t str_resp2[] = { 0x46, 0x20, 0x33, 0x37, 0x20, 0x33, 0x38, 0x20, 0x33, 0x33, 0x20, 0x33, 0x32, 0x20, 0x30, 0x30, 0x20, 0x46, 0x46, 0x20 };
            //uint8_t str_resp3[] = { 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x31, 0x36, 0x20, 0x35, 0x41, 0x20, 0x31, 0x31, 0x38, 0x38, 0x33, 0x0d };
            size_t str_len = sizeof(str);
            //size_t str_resp1_len=sizeof(str_resp1);
            //size_t str_resp2_len=sizeof(str_resp2);
            //size_t str_resp3_len=sizeof(str_resp3);
            size_t i;
            int equal = 0;

            if (len_cleaned == str_len)
            {
              equal = 1;
              for (i = 0 ; i < len_cleaned; i++)
                if (value_ptr[i] != str[i])
                  equal = 0;
            }
            if (equal == 1)
            {
              uint8_t buffer[3 * 20]; // 3x 20 bytes (1 or 2 bytes extra room);
              char csum_hex_str[5];   // 3 bytes used;
              unsigned int bytecounter;
              unsigned int local_bytecounter;
              uint8_t all_initial_values[][3] = { // the actual value without header/slot
                { 0x35, 0x41, 0x20 },   // [0]  nr8 RPM
                { 0x41, 0x35, 0x20 },   // [1]  nr8 Advance
                { 0x41, 0x30, 0x20 },   // [2]  nr9 RPM
                { 0x41, 0x35, 0x20 },   // [3]  nr9 Advance
                { 0x46, 0x46, 0x20 },   // [4]  nr10 RPM
                { 0x46, 0x46, 0x20 },   // [5]  nr1f0 Advance
                { 0x33, 0x37, 0x20 },   // [6]  pincode[0] 0x33, 0x37 = "0x37" ='7' = 7
                { 0x33, 0x38, 0x20 },   // [7]  pincode[1] 0x33, 0x38 = "0x38" ='8' = 8
                { 0x33, 0x33, 0x20 },   // [8]  pincode[2] 0x33, 0x33 = "0x33" ='3' = 3
                { 0x33, 0x32, 0x20 },   // [9]  pincode[3] 0x33, 0x32 = "0x32" ='2' = 2
                { 0x30, 0x30, 0x20 },   // [10] pincode[4] = '\0'  0x30,0x30 = "0x30" = '0' = 0, So it is not wis to use a '0' as pincode... as a \0 = same as '0'
                { 0x46, 0x46, 0x20 },   // [11]
                { 0x46, 0x46, 0x20 },   // [12]
                { 0x46, 0x46, 0x20 },   // [13]
                { 0x31, 0x36, 0x20 },   // [14] Vacuum advance graph Start@RPM
                { 0x35, 0x41, 0x20 }  // [15] RPM Limit
              };
              int csum;   // 3 byte checksum
              rx_cmd_found++;

              printf("%s 3th \'11@\\n\' Command Found----\n", __FUNCTION__);
              print_hex(value_ptr, len_cleaned);

              // produce an answer...

              // fill graph under cmd2 with custom values.
              // Fixed 500 RPM
              //str_resp1[11]=server->tune_AdvanceCurveRPM[0][0]; // Fixed 500 RPM
              //str_resp1[12]=server->tune_AdvanceCurveRPM[0][1];
              // overwrite captrued advance with custom value, calculate csum later.
              //all_initial_values[3][0]=server->tune_AdvanceCurveDegrees[0][0];
              //all_initial_values[3][1]=server->tune_AdvanceCurveDegrees[0][1];

              // copy internal state tables to 123Tune bloothooth protocol format
              // n=1 .. 7, array offset=2
              for(int n=8,i=0;n<=10;n++) {
                all_initial_values[i][0]=server->tune_AdvanceCurveRPM[n-1][0];
                all_initial_values[i++][1]=server->tune_AdvanceCurveRPM[n-1][1];
                all_initial_values[i][0]=server->tune_AdvanceCurveDegrees[n-1][0];
                all_initial_values[i++][1]=server->tune_AdvanceCurveDegrees[n-1][1];
              }

          		// DEBUG
          		//printf("Default/hardcoded initial PINCODE[1] was: %c\n",TunePinCode2char(all_initial_values[6][0],all_initial_values[6][1]));
          		//printf("Default/hardcoded initial PINCODE[2] was: %c\n",TunePinCode2char(all_initial_values[7][0],all_initial_values[7][1]));
          		//printf("Default/hardcoded initial PINCODE[3] was: %c\n",TunePinCode2char(all_initial_values[8][0],all_initial_values[8][1]));
          		//printf("Default/hardcoded initial PINCODE[4] was: %c\n",TunePinCode2char(all_initial_values[9][0],all_initial_values[9][1]));

          		// Set pincode
          		// Need to dynamic read the pincode, as it gets checked after a pincode change with this cmd 3
          		all_initial_values[6][0]=server->tune_Pincode[0][0];    // Single Char!!
              all_initial_values[6][1]=server->tune_Pincode[0][1];    // Single Char!! "30"=0, "31"=1
          		all_initial_values[7][0]=server->tune_Pincode[1][0];    // Single Char!!
              all_initial_values[7][1]=server->tune_Pincode[1][1];    // Single Char!!
          		all_initial_values[8][0]=server->tune_Pincode[2][0];    // Single Char!!
              all_initial_values[8][1]=server->tune_Pincode[2][1];    // Single Char!!
          		all_initial_values[9][0]=server->tune_Pincode[3][0];    // Single Char!!
              all_initial_values[9][1]=server->tune_Pincode[3][1];    // Single Char!!

          		printf("Dynamic PINCODE[1] was: %c\n",TunePinCode2char(all_initial_values[6][0],all_initial_values[6][1]));
          		printf("Dynamic PINCODE[2] was: %c\n",TunePinCode2char(all_initial_values[7][0],all_initial_values[7][1]));
          		printf("Dynamic PINCODE[3] was: %c\n",TunePinCode2char(all_initial_values[8][0],all_initial_values[8][1]));
          		printf("Dynamic PINCODE[4] was: %c\n",TunePinCode2char(all_initial_values[9][0],all_initial_values[9][1]));


              // Vacuum advance graph Start@RPM
              all_initial_values[14][0]=server->tune_MapCurveStartRPM[0];    // Single Char!!
              all_initial_values[14][1]=server->tune_MapCurveStartRPM[1];    // Single Char!!

              // RPM Limit
              all_initial_values[15][0]=server->tune_RPMLimit[0];    // Single Char!!
              all_initial_values[15][1]=server->tune_RPMLimit[1];    // Single Char!!

              // Fill complete unfragmented buffer
              bytecounter = 0;
              // copy header @ start
              for (local_bytecounter = 0; local_bytecounter < sizeof(str); local_bytecounter++, bytecounter++) {
                buffer[bytecounter] = str[local_bytecounter];
              }
              // copy values (from sniffer for now)
              for (i = 0; i < (sizeof(all_initial_values) / 3); i++ ) {
                buffer[bytecounter++] = all_initial_values[i][0]; local_bytecounter++;
                buffer[bytecounter++] = all_initial_values[i][1]; local_bytecounter++;
                buffer[bytecounter++] = all_initial_values[i][2]; local_bytecounter++;
              }

              // copy ID  from request str: 0x31 0x30+sequence 0x40=16 (data len)
              buffer[bytecounter++] = str[0]; // first 2 butes in cmd and further
              buffer[bytecounter++] = str[1];

              csum = 0;
              // calculate checkum, could be integrated in buffer copy above for efficienty, but not for readability.
              for (i = 0; i < sizeof(all_initial_values) / 3; i++) {
                char str_l[20];

                str_l[0] = all_initial_values[i][0];
                str_l[1] = all_initial_values[i][1];
                str_l[2] = '\0';
                csum += strtol(str_l, NULL, 16);
              }
              sprintf(csum_hex_str, "%03X", csum);

              // fill csum
              buffer[bytecounter++] = csum_hex_str[0];
              buffer[bytecounter++] = csum_hex_str[1];
              buffer[bytecounter++] = csum_hex_str[2];

              // add termination 0x0d
              buffer[bytecounter++] = 0x0d; // bytecounter containts the total number of bytes

              printf("%s bytecounter=%d csum=%s\n", __FUNCTION__, bytecounter, csum_hex_str);


              // size is 59, fist 20 bytes fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[0], 20);
              */
              server->ctrl_TX_Characteristic->setValue(&buffer[0], 20);
              server->ctrl_TX_Characteristic->notify();

              bytecounter -= 20;

              // second 20 bytes fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[20], 20);
              */
              server->ctrl_TX_Characteristic->setValue(&buffer[20], 20);
              server->ctrl_TX_Characteristic->notify();

              bytecounter -= 20;
              // last fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[40], bytecounter );
              */
              server->ctrl_TX_Characteristic->setValue(&buffer[40], bytecounter);
              server->ctrl_TX_Characteristic->notify();

            }

          }


          {
            //uint8_t str[] = { 0x24, 0x24, 0x0d };
            //uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
            //uint8_t str[] = { 0x24, 0x24, 0x31, 0x31, 0x40, 0x0d };
//            uint8_t str[] = { 0x31, 0x32, 0x40, 0x0d };   // was working in 2018
            uint8_t str[] = { 0x31, 0x32, 0x40, 0x0d };   // '12@\n' command
            //uint8_t str_resp[] = { 0x0d };
            //uint8_t str_resp1[] = { 0x31, 0x32, 0x40, 0x0d, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x30, 0x30, 0x20, 0x33, 0x37, 0x20, 0x31, 0x44, 0x20, 0x33 };
            //uint8_t str_resp2[] = { 0x37, 0x20, 0x31, 0x45, 0x20, 0x33, 0x37, 0x20, 0x32, 0x38, 0x20, 0x33, 0x37, 0x20, 0x35, 0x38, 0x20, 0x30, 0x30, 0x20 };
            //uint8_t str_resp3[] = { 0x36, 0x34, 0x20, 0x30, 0x30, 0x20, 0x43, 0x38, 0x20, 0x30, 0x30, 0x20, 0x31, 0x32, 0x34, 0x43, 0x31, 0x0d };   // copy original
            //uint8_t str_resp3[] = { 0x36, 0x34, 0x20, 0x30, 0x30, 0x20, 0x43, 0x38, 0x20, 0x30, 0x30, 0x20, 0x31, 0x32, 0x34, 0x43, 0x31, 0x0d }; // search for 0x43, 0x38, meaning

            size_t str_len = sizeof(str);
            //size_t str_resp1_len=sizeof(str_resp1);
            //size_t str_resp2_len=sizeof(str_resp2);
            //size_t str_resp3_len=sizeof(str_resp3);
            size_t i;
            int equal = 0;

            if (len_cleaned == str_len)
            {
              equal = 1;
              for (i = 0 ; i < len_cleaned; i++)
                if (value_ptr[i] != str[i])
                  equal = 0;
            }
            if (equal == 1)
            {
              uint8_t buffer[3 * 20]; // 3x 20 bytes (1 or 2 bytes extra room);
              char csum_hex_str[5];   // 3 bytes used;
              unsigned int bytecounter;
              unsigned int local_bytecounter;
              uint8_t all_initial_values[][3] = { // the actual value without header/slot
                { 0x46, 0x46, 0x20 },   // [0]
                { 0x46, 0x46, 0x20 },   // [1]
                { 0x30, 0x30, 0x20 },   // [2]  nr1 Vacuum Advance graph: abs pressure (vacuum)     0
                { 0x33, 0x37, 0x20 },   // [3]  nr1 Vacuum Advance graph: advance                   11
                { 0x31, 0x44, 0x20 },   // [4]  nr2 Vacuum Advance graph: abs pressure (vacuum)     29
                { 0x33, 0x37, 0x20 },   // [5]  nr2 Vacuum Advance graph: advance                   11
                { 0x31, 0x45, 0x20 },   // [6]  nr3 Vacuum Advance graph: abs pressure (vacuum)     30
                { 0x33, 0x37, 0x20 },   // [7]  nr3 Vacuum Advance graph: advance                   11
                { 0x32, 0x38, 0x20 },   // [8]  nr4 Vacuum Advance graph: abs pressure (vacuum)     40
                { 0x33, 0x37, 0x20 },   // [9]  nr4 Vacuum Advance graph: advance                   11
                { 0x35, 0x38, 0x20 },   // [10] nr5 Vacuum Advance graph: abs pressure (vacuum)     88
                { 0x30, 0x30, 0x20 },   // [11] nr5 Vacuum Advance graph: advance                   0
                { 0x36, 0x34, 0x20 },   // [12] nr6 Vacuum Advance graph: abs pressure (vacuum)     100
                { 0x30, 0x30, 0x20 },   // [13] nr6 Vacuum Advance graph: advance                   0
                { 0x43, 0x38, 0x20 },   // [14] nr7 Vacuum Advance graph: abs pressure (vacuum)     200
                { 0x30, 0x30, 0x20 }    // [15] nr7 Vacuum Advance graph: advance                   0
              };
              int csum;   // 3 byte checksum

              rx_cmd_found++;
              printf("%s 4th \'12@\\n\' Command Found----\n", __FUNCTION__);
              print_hex(value_ptr, len_cleaned);

              // produce an answer...

              // fill graph under cmd2 with custom values.
              // Fixed 500 RPM
              //str_resp1[11]=server->tune_AdvanceCurveRPM[0][0]; // Fixed 500 RPM
              //str_resp1[12]=server->tune_AdvanceCurveRPM[0][1];

              // copy internal state tables to 123Tune bloothooth protocol format
              // n=1 .. 7, array offset=2
              for(int n=1,i=2;n<=7;n++) {
                all_initial_values[i][0]=server->tune_MapCurvePressure[n-1][0];
                all_initial_values[i++][1]=server->tune_MapCurvePressure[n-1][1];
                all_initial_values[i][0]=server->tune_MapCurveDegrees[n-1][0];
                all_initial_values[i++][1]=server->tune_MapCurveDegrees[n-1][1];
              }

/*
              // overwrite captrued advance with custom value, calculate csum later.
              // '1'
              all_initial_values[2][0]=server->tune_MapCurvePressure[0][0]; // Fixed 500 RPM
              all_initial_values[2][1]=server->tune_MapCurvePressure[0][1]; // Fixed 500 RPM
              all_initial_values[3][0]=server->tune_MapCurveDegrees[0][0];
              all_initial_values[3][1]=server->tune_MapCurveDegrees[0][1];

              // '2'
              all_initial_values[4][0]=server->tune_MapCurvePressure[1][0];
              all_initial_values[4][1]=server->tune_MapCurvePressure[1][1];
              all_initial_values[5][0]=server->tune_MapCurveDegrees[1][0];
              all_initial_values[5][1]=server->tune_MapCurveDegrees[1][1];

              // '3'
              all_initial_values[6][0]=server->tune_MapCurvePressure[2][0];
              all_initial_values[6][1]=server->tune_MapCurvePressure[2][1];
              all_initial_values[7][0]=server->tune_MapCurveDegrees[2][0];
              all_initial_values[7][1]=server->tune_MapCurveDegrees[2][1];

              // '4'
              all_initial_values[8][0]=server->tune_MapCurvePressure[3][0];
              all_initial_values[8][1]=server->tune_MapCurvePressure[3][1];
              all_initial_values[9][0]=server->tune_MapCurveDegrees[3][0];
              all_initial_values[9][1]=server->tune_MapCurveDegrees[3][1];

              // '5'
              all_initial_values[10][0]=server->tune_MapCurvePressure[4][0];
              all_initial_values[10][1]=server->tune_MapCurvePressure[4][1];
              all_initial_values[11][0]=server->tune_MapCurveDegrees[4][0];
              all_initial_values[11][1]=server->tune_MapCurveDegrees[4][1];

              // '6'
              all_initial_values[12][0]=server->tune_MapCurvePressure[5][0];
              all_initial_values[12][1]=server->tune_MapCurvePressure[5][1];
              all_initial_values[13][0]=server->tune_MapCurveDegrees[5][0];
              all_initial_values[13][1]=server->tune_MapCurveDegrees[5][1];

              // '7'
              all_initial_values[14][0]=server->tune_MapCurvePressure[6][0];
              all_initial_values[14][1]=server->tune_MapCurvePressure[6][1];
              all_initial_values[15][0]=server->tune_MapCurveDegrees[6][0];
              all_initial_values[15][1]=server->tune_MapCurveDegrees[6][1];
*/
              // Fill complete unfragmented buffer
              bytecounter = 0;
              // copy header @ start
              for (local_bytecounter = 0; local_bytecounter < sizeof(str); local_bytecounter++, bytecounter++) {
                buffer[bytecounter] = str[local_bytecounter];
              }
              // copy values (from sniffer for now)
              for (i = 0; i < (sizeof(all_initial_values) / 3); i++ ) {
                buffer[bytecounter++] = all_initial_values[i][0]; local_bytecounter++;
                buffer[bytecounter++] = all_initial_values[i][1]; local_bytecounter++;
                buffer[bytecounter++] = all_initial_values[i][2]; local_bytecounter++;
              }

              // copy ID  from request str: 0x31 0x30+sequence 0x40=16 (data len)
              buffer[bytecounter++] = str[0]; // first 2 butes in cmd and further
              buffer[bytecounter++] = str[1];

              csum = 0;
              // calculate checkum, could be integrated in buffer copy above for efficienty, but not for readability.
              for (i = 0; i < sizeof(all_initial_values) / 3; i++) {
                char str_l[20];

                str_l[0] = all_initial_values[i][0];
                str_l[1] = all_initial_values[i][1];
                str_l[2] = '\0';
                csum += strtol(str_l, NULL, 16);
              }
              sprintf(csum_hex_str, "%03X", csum);

              // fill csum
              buffer[bytecounter++] = csum_hex_str[0];
              buffer[bytecounter++] = csum_hex_str[1];
              buffer[bytecounter++] = csum_hex_str[2];

              // add termination 0x0d
              buffer[bytecounter++] = 0x0d; // bytecounter containts the total number of bytes

              printf("%s bytecounter=%d csum=%s\n", __FUNCTION__, bytecounter, csum_hex_str);


              // size is 59, fist 20 bytes fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[0], 20);
              */
              server->ctrl_TX_Characteristic->setValue(&buffer[0], 20);
              server->ctrl_TX_Characteristic->notify();

              bytecounter -= 20;

              // second 20 bytes fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[20], 20);
              */
              server->ctrl_TX_Characteristic->setValue(&buffer[20], 20);
              server->ctrl_TX_Characteristic->notify();

              bytecounter -= 20;
              // last fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[40], bytecounter );
              */
              server->ctrl_TX_Characteristic->setValue(&buffer[40], bytecounter);
              server->ctrl_TX_Characteristic->notify();

            }

          }

          { // MAP/RPM Advance curve are filled after command 5.
            //uint8_t str[] = { 0x24, 0x24, 0x0d };
            //uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
            //uint8_t str[] = { 0x24, 0x24, 0x31, 0x31, 0x40, 0x0d };
            //uint8_t str[] = { 0x24, 0x24, 0x31, 0x32, 0x40, 0x0d };
//            uint8_t str[] = { 0x31, 0x33, 0x40, 0x0d };   // was working in 2018
            uint8_t str[] = { 0x31, 0x33, 0x40, 0x0d  };    // '13@\n' command
            //uint8_t str_resp[] = { 0x0d };
            //uint8_t str_resp1[] = { 0x31, 0x33, 0x40, 0x0d, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46 };
            //uint8_t str_resp2[] = { 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20 };
            //uint8_t str_resp3[] = { 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x31, 0x33, 0x46, 0x46, 0x30, 0x0d };

            size_t str_len = sizeof(str);
            //size_t str_resp1_len=sizeof(str_resp1);
            //size_t str_resp2_len=sizeof(str_resp2);
            //size_t str_resp3_len=sizeof(str_resp3);


            size_t i;
            int equal = 0;
            if (len_cleaned == str_len)
            {
              equal = 1;
              for (i = 0 ; i < len_cleaned; i++)
                if (value_ptr[i] != str[i])
                  equal = 0;
            }
            if (equal == 1)
            {
              uint8_t buffer[3 * 20]; // 3x 20 bytes (1 or 2 bytes extra room);
              char csum_hex_str[5];   // 3 bytes used;
              unsigned int bytecounter;
              unsigned int local_bytecounter;
              uint8_t all_initial_values[][3] = { // the actual value without header/slot
                { 0x46, 0x46, 0x20 }, // [0]  nr8  Vacuum Advance graph: abs pressure (vacuum)  "0xFF"
                { 0x46, 0x46, 0x20 },   // [1]  nr8 Vacuum Advance graph: advance               "0xFF"
                { 0x46, 0x46, 0x20 }, // [2]  nr9  Vacuum Advance graph: abs pressure (vacuum)   "0xFF"
                { 0x46, 0x46, 0x20 },   // [3]  nr9 Vacuum Advance graph: advance               "0xFF"
                { 0x46, 0x46, 0x20 }, // [4]  nr10 Vacuum Advance graph: abs pressure (vacuum)   "0xFF"
                { 0x46, 0x46, 0x20 },   // [5]  nr10 Vacuum Advance graph: advance              "0xFF"
                { 0x46, 0x46, 0x20 },
                { 0x46, 0x46, 0x20 },
                { 0x46, 0x46, 0x20 },
                { 0x46, 0x46, 0x20 },
                { 0x46, 0x46, 0x20 },
                { 0x46, 0x46, 0x20 },
                { 0x46, 0x46, 0x20 },
                { 0x46, 0x46, 0x20 },
                { 0x46, 0x46, 0x20 },
                { 0x46, 0x46, 0x20 }
              };
              int csum;   // 3 byte checksum

              rx_cmd_found++;
              printf("%s 5th \'13@\\n\' Command Found----\n", __FUNCTION__);
              print_hex(value_ptr, len_cleaned);

              // produce an answer...

              // fill graph under cmd2 with custom values.
              // Fixed 500 RPM
              //str_resp1[11]=server->tune_AdvanceCurveRPM[0][0]; // Fixed 500 RPM
              //str_resp1[12]=server->tune_AdvanceCurveRPM[0][1];

              // overwrite captrued advance with custom value, calculate csum later.
              //all_initial_values[3][0]=server->tune_AdvanceCurveDegrees[0][0];
              //all_initial_values[3][1]=server->tune_AdvanceCurveDegrees[0][1];


              // copy internal state tables to 123Tune bloothooth protocol format
              // n=8 .. 10, array offset=0
              for(int n=8,i=0;n<=10;n++) {
                all_initial_values[i][0]=server->tune_MapCurvePressure[n-1][0];
                all_initial_values[i++][1]=server->tune_MapCurvePressure[n-1][1];
                all_initial_values[i][0]=server->tune_MapCurveDegrees[n-1][0];
                all_initial_values[i++][1]=server->tune_MapCurveDegrees[n-1][1];
              }
/*
              // '8'
              all_initial_values[0][0]=server->tune_MapCurvePressure[7][0];
              all_initial_values[0][1]=server->tune_MapCurvePressure[7][1];
              all_initial_values[1][0]=server->tune_MapCurveDegrees[7][0];
              all_initial_values[1][1]=server->tune_MapCurveDegrees[7][1];

              // '9'
              all_initial_values[2][0]=server->tune_MapCurvePressure[8][0];
              all_initial_values[2][1]=server->tune_MapCurvePressure[8][1];
              all_initial_values[3][0]=server->tune_MapCurveDegrees[8][0];
              all_initial_values[3][1]=server->tune_MapCurveDegrees[8][1];

              // '10'
              all_initial_values[4][0]=server->tune_MapCurvePressure[9][0];
              all_initial_values[4][1]=server->tune_MapCurvePressure[9][1];
              all_initial_values[5][0]=server->tune_MapCurveDegrees[9][0];
              all_initial_values[5][1]=server->tune_MapCurveDegrees[9][1];
*/
              // Fill complete unfragmented buffer
              bytecounter = 0;
              // copy header @ start
              for (local_bytecounter = 0; local_bytecounter < sizeof(str); local_bytecounter++, bytecounter++) {
                buffer[bytecounter] = str[local_bytecounter];
              }
              // copy values (from sniffer for now)
              for (i = 0; i < (sizeof(all_initial_values) / 3); i++ ) {
                buffer[bytecounter++] = all_initial_values[i][0]; local_bytecounter++;
                buffer[bytecounter++] = all_initial_values[i][1]; local_bytecounter++;
                buffer[bytecounter++] = all_initial_values[i][2]; local_bytecounter++;
              }

              // copy ID  from request str: 0x31 0x30+sequence 0x40=16 (data len)
              buffer[bytecounter++] = str[0]; // first 2 butes in cmd and further
              buffer[bytecounter++] = str[1];

              csum = 0;
              // calculate checkum, could be integrated in buffer copy above for efficienty, but not for readability.
              for (i = 0; i < sizeof(all_initial_values) / 3; i++) {
                char str_l[20];

                str_l[0] = all_initial_values[i][0];
                str_l[1] = all_initial_values[i][1];
                str_l[2] = '\0';
                csum += strtol(str_l, NULL, 16);
              }
              sprintf(csum_hex_str, "%03X", csum);

              // fill csum
              buffer[bytecounter++] = csum_hex_str[0];
              buffer[bytecounter++] = csum_hex_str[1];
              buffer[bytecounter++] = csum_hex_str[2];

              // add termination 0x0d
              buffer[bytecounter++] = 0x0d; // bytecounter containts the total number of bytes

              printf("%s bytecounter=%d csum=%s\n", __FUNCTION__, bytecounter, csum_hex_str);


              // size is 59, fist 20 bytes fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[0], 20);
              */
              server->ctrl_TX_Characteristic->setValue(&buffer[0], 20);
              server->ctrl_TX_Characteristic->notify();

              bytecounter -= 20;

              // second 20 bytes fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[20], 20);
              */
              server->ctrl_TX_Characteristic->setValue(&buffer[20], 20);
              server->ctrl_TX_Characteristic->notify();

              bytecounter -= 20;
              // last fragment
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               &buffer[40], bytecounter );
              */
              server->ctrl_TX_Characteristic->setValue(&buffer[40], bytecounter);
              server->ctrl_TX_Characteristic->notify();
            }

          }

          { // All Meter value_ptrs show up after command 6  !!!
            // Service command 'v@' (Version Service Command)
            // returns voltage, Temp,pessure and hardware/software version

            //static uint_8 ub = 0x37;
            //static uint_8 lb = 0x46;
            // values start showing up after this repeated exected reguest/response
            //uint8_t str[] = { 0x24, 0x24, 0x0d };
            //uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
            //uint8_t str[] = { 0x24, 0x24, 0x31, 0x31, 0x40, 0x0d };
            //uint8_t str[] = { 0x24, 0x24, 0x31, 0x32, 0x40, 0x0d };
            //uint8_t str[] = { 0x24, 0x24, 0x31, 0x33, 0x40, 0x0d };
//            uint8_t str[] = {  0x76, 0x40, 0x0d };      // was working in 2018
            uint8_t str[] = {  0x76, 0x40, 0x0d };    // 'v@\n'
            //uint8_t str_resp[] = { 0x0d };

            // orig command 6:
            //uint8_t str_resp1[] = { 0x76, 0x40, 0x0d, 0x34, 0x30, 0x33, 0x41, 0x36, 0x34, 0x34, 0x31, 0x2d, 0x31, 0x30, 0x2d, 0x34, 0x35, 0x20 };

            // playground to divercover encoding:
            //uint8_t str_resp1[] =   { 0x76, 0x40, 0x0d, 0x34, 0x33, 0x34, 0x39, 0x38, 0x46, 0x34, 0x39, 0x2d, 0x31, 0x30, 0x2d, 0x34, 0x35, 0x20 };
            //                        [request 3Bytes]   [Volt 2Byt],[Temp 2Byt], [BAR 2Byte]  '4'    '1'   '-'    1'   '0'   '-'   '4'   '5'   ' '
            //uint8_t str_resp2[] = {  };
            //uint8_t str_resp3[] = {  };

            size_t str_len = sizeof(str);
            //size_t str_resp2_len=sizeof(str_resp2);
            //size_t str_resp3_len=sizeof(str_resp3);

            size_t i;
            int equal = 0;

            if (len_cleaned == str_len)
            {
              equal = 1;
              for (i = 0 ; i < len_cleaned; i++)
                if (value_ptr[i] != str[i])
                  equal = 0;
            }
            if (equal == 1)
            {
              unsigned int bytecounter;

              uint8_t str_resp1[] =   {
                0x76, 0x40, 0x0d, // 'v@\n'
                0x34, 0x33,     // [3][4] Voltage
                0x34, 0x39,     // [5][6] Temperature
                0x37, 0x34,     // [7][8] Pressure
                0x34, 0x31, 0x2d, 0x31, 0x30, 0x2d, 0x34, 0x35, 0x20  // Version: '41-10-45 '
              };
              size_t str_resp1_len = sizeof(str_resp1);

              bytecounter=str_len;//len_cleaned == str_len;

              //decimal2TuneVoltage(14.0, &server->tune_Voltage[0],&server->tune_Voltage[1]);

              str_resp1[bytecounter++] = server->tune_Voltage[0];                 // read actual value MSB from internal state
              str_resp1[bytecounter++] = server->tune_Voltage[1];     // read actual value LSB from internal state

              //decimal2TuneTemperature(35 ,&server->tune_Temperature[0],&server->tune_Temperature[1]);

              str_resp1[bytecounter++] = server->tune_Temperature[0];             // read actual value MSB from internal state
              str_resp1[bytecounter++] = server->tune_Temperature[1];   // read actual value LSB from internal state

              //decimal2TunePressure(150, &server->tune_Pressure[0],&server->tune_Pressure[1]);

              str_resp1[bytecounter++] = server->tune_Pressure[0];              // read actual value MSB from internal state
              str_resp1[bytecounter++] = server->tune_Pressure[1];    // read actual value LSB from internal state

              // Nice, version command (displayed in 123Tuneapp debug log) is coded with Volt/Tempererature/Pressure+real versionstring.
              // Decoded format:
              // is Stripped/shown in app as "v3.4-7", so last digit of 78  is not used/shown, and  '56' is not shown, or something else?
              // #define P123TUNE_Device_Version "78-34-56 "
              // format "XX-XX-XX "
              #define P123TUNE_Device_Version "41-10-45 "

              strncpy((char*)&str_resp1[bytecounter],P123TUNE_Device_Version,sizeof(P123TUNE_Device_Version));

              //printf("%s Temperature %d Celcius\n", __FUNCTION__, TuneTemperature2decimal(server->tune_Temperature[0], server->tune_Temperature[1])); // test reverse function  (and forward)

              rx_cmd_found++;
              printf("%s 6th \'v@\\n\' Command Found----\n", __FUNCTION__);
              //print_hex(value_ptr, len_cleaned);
              print_hex(value_ptr, len_cleaned);    // RX (Cleaned)
              print_hex(str_resp1, str_resp1_len);    // TX

              // produce an answer...
              /*
                bt_gatt_server_send_notification(server->gatt,
                                               server->tune_msrmt_handle,
                                               str_resp1, str_resp1_len);
              */
              server->ctrl_TX_Characteristic->setValue(str_resp1, str_resp1_len);
              server->ctrl_TX_Characteristic->notify();

            }

          }


          // Tuning implementation UUID: 59:d1:99:c2:b1:93:43:af:25:4c:05:72:0c:26:03:bf
          // 74 is Toggle Tuning mode (Start/Stop)
          // 61 == +
          // 62 == -
          // 02 == exit 123tune programm -> UUID 24:0a:c0:a6:df:95:0e:90:9c:47:4c:69:b9:88:79:a8 (not here)
          {
            if (len_cleaned == 1)
            {
              //static uint8_t tune_Advance_bck[2]; // TODO, needs to based on graph
              switch (value_ptr[0]) {
                case 0x61:  // + in Tuning mode (Advance)
                  rx_cmd_found++;
                  printf("%s Tuning mode + Found\n", __FUNCTION__);
                  server->tune_TuningMode_Advance++;

                  // Direct modify advance, need to changed later, based on curve [TODO]
                  // decimal2TuneAdvance( TuneAdvance2decimal(server->tune_Advance[0],server->tune_Advance[1])   + 1.0  ,&server->tune_Advance[0],&server->tune_Advance[1]);

                  // Directly modify tune_TuningMode_Advance, and keep away from actual server->tune_Advance[] value
                  //decimal2TuneAdvance( TuneAdvance2decimal(server->tune_TuningMode_Advance[0],server->tune_TuningMode_Advance[1])   + 1.0  ,&server->tune_TuningMode_Advance[0],&server->tune_TuningMode_Advance[1]);

                  break;
                case 0x72:  // - in Tuning mode (Advance)
                  rx_cmd_found++;
                  printf("%s Tuning mode - Found\n", __FUNCTION__);
                  server->tune_TuningMode_Advance--;

                  // Direct modify advance, need to changed later, based on curve [TODO]
                  //decimal2TuneAdvance( TuneAdvance2decimal(server->tune_Advance[0],server->tune_Advance[1])   - 1.0  ,&server->tune_Advance[0],&server->tune_Advance[1]);

                  // Directly modify tune_TuningMode_Advance, and keep away from actual server->tune_Advance[] value
                  //decimal2TuneAdvance( TuneAdvance2decimal(server->tune_TuningMode_Advance[0],server->tune_TuningMode_Advance[1])   - 1.0  ,&server->tune_TuningMode_Advance[0],&server->tune_TuningMode_Advance[1]);


                  break;
                case 0x74:  // Toggle Tuning mode
                  // TODO needs to be modyfied for using graphs
                  rx_cmd_found++;
                  printf("%s Tuning mode Toggle Found\n", __FUNCTION__);
                  if (!server->tune_TuningMode_enabled) {
                    // Enable
                    // Start always at 0
                    //decimal2TuneAdvance(0 ,&server->tune_TuningMode_Advance[0],&server->tune_TuningMode_Advance[1]);
                    server->tune_TuningMode_Advance = 0;

                  } else {  // restore state
                    // Disable
                    // setting to 0 is attractive, but it's simply a don't care when it's not enabled.
                    // other code shoud honor 'server->tune_TuningMode_enabled'
                    // decimal2TuneAdvance(0 ,&server->tune_TuningMode_Advance[0],&server->tune_TuningMode_Advance[1]);
                  }

                  server->tune_TuningMode_enabled = !server->tune_TuningMode_enabled; //Toggle status boolean
                  break;
                default:  // do not touch this
                  break;

              }
              // no response other then acknowledgement
            }
          }




          // write curve/map implementation 2017/02/07
          {
            //uint8_t str[] = {  0x31, 0x30, 0x2D, 0x30, 0x30, 0x34, 0x0D };
            //uint8_t str_resp1[] =   { 0x31, 0x30, 0x2D, 0x30, 0x30, 0x34, 0x0D };
            //const uint8_t *value_ptr; // sometimes a 0x24 is added/prepended, need to filter out
            //size_t len_cleaned; // sometimes a 0x24 is added/prepended, need to filter out

            //len_cleaned=len;
            //value_ptr=value;

            //size_t str_len=sizeof(str);
            //size_t str_resp1_len=sizeof(str_resp1);

/*      // No need to clean up as it is done at beginning already, remoteved 2019/01/28
            // prepending cleanup is done in the beginning of this function
            if ( len == 8 && value_ptr[0] == 0x24) { // skip first 0x24 when strings still meets expectations
              value_ptr++;
              len_cleaned--;
            }
            // postpending cleanup is _not_ done at begin of this function
            if ( len == 8 && value_ptr[7] == 0x24) { // skip last 0x24 when strings still meets expectations
              //value_ptr++;
              len_cleaned--;
            }
*/
            //size_t i;
            //int equal=0;
            if (len_cleaned == 7) // 7 chars, or more chars (uncleaned)
            {
              int nr;   // graph index number 1 .. 10
              if ( value_ptr[2] == 0x2D && value_ptr[3] == 0x30 && value_ptr[6] == 0x0D ) //
              {
                switch (value_ptr[4]) {
                  case 0x30:  // Advance curve Remove Command ????
                    printf("%02X Advance Curve Command Found: ", value_ptr[4]);

                    switch (value_ptr[5]) {
                      // can't adjust no 1 RPM
                      case 0x34:  // Advance graps entries: No 2
                      case 0x36:  // Advance graps entries: No 3
                      case 0x38:  // Advance graps entries: No 4
                      case 0x41:  // Advance graps entries: No 5
                      case 0x43:      // Advance graps entries: No 6
                      case 0x45:      // Advance graps entries: No 7
                        rx_cmd_found++;
                        nr = TuneGraphNo2decimal(value_ptr[4], value_ptr[5]) / 2;
                        printf("%02X (ADD)RPM value %04d rpm at curve postion No: %d\n", value_ptr[5], TuneRPM2decimal(value_ptr[0], value_ptr[1]), nr );
                        // Store new value in internal server state: range checking should be done here
                        server->tune_AdvanceCurveRPM[nr - 1][0] = value_ptr[0];
                        server->tune_AdvanceCurveRPM[nr - 1][1] = value_ptr[1];

                        // fill rest with N/A values
                        if(TuneRPM2decimal(value_ptr[0], value_ptr[1]) == 8000) {
                          // the number of degrees is variable, do not fixate this
                          //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr - 1][0],&server->tune_MapCurveDegrees[nr - 1][1]);

                          // re-use n variabele (as it has exactly the right value to start with) to finish aand fill the rest with default valuses
                          for(; nr < C_nr_123MapCurve_Elements;nr++) {
                            // "0xFF" = 0x46,0x46
                            server->tune_AdvanceCurveRPM[nr][0] = 0x46;  // "0xG " == N/A
                            server->tune_AdvanceCurveRPM[nr][1] = 0x46; // "0x G" == N/A
                            server->tune_AdvanceCurveDegrees[nr][0] = 0x46;  // "0xG " == N/A
                            server->tune_AdvanceCurveDegrees[nr][1] = 0x46; // "0x G"  == N/A
                            //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr][0],&server->tune_MapCurveDegrees[nr][1]);
                          }
                        }


                        break;
                      case 0x33:      // Degrees: no 1
                      case 0x35:  // Degrees: no 2
                      case 0x37:      // Degrees: no 3
                      case 0x39:      // Degrees: no 4
                      case 0x42:      // Degrees: no 5
                      case 0x44:      // Degrees: no 6
                      case 0x46:      // Degrees: no 7
                        rx_cmd_found++;
                        nr = (TuneGraphNo2decimal(value_ptr[4], value_ptr[5]) - 1) / 2;
                        printf("%02X (ADD)Advance value %02.2f degrees at curve postion No: %d\n", value_ptr[5], TuneAdvance2decimal(value_ptr[0], value_ptr[1]), nr );
                        // Store new value in internal server state: range checking should be done here
                        server->tune_AdvanceCurveDegrees[nr - 1][0] = value_ptr[0];
                        server->tune_AdvanceCurveDegrees[nr - 1][1] = value_ptr[1];
                        break;
                      default:
                        printf("UNKNOWN Advance CMD found\n");
                        break;
                    }

                    print_hex(value_ptr, len_cleaned);
                    /*
                      bt_gatt_server_send_notification(server->gatt,
                                                     server->tune_msrmt_handle,
                                                     value_ptr, len_cleaned);
                    */
                    server->ctrl_TX_Characteristic->setValue(value_ptr, len_cleaned);
                    server->ctrl_TX_Characteristic->notify();

                    break;
                  case 0x31:  // Advance Curve ???
                    // 35 35 2D 30 31 41 0D Immobilizer ON '55-01A\n'
                    // 41 41 2D 30 31 41 0D Immobilizer Off 'AA-01A\n'

                    printf("%02X Advance Curve Command Found: ", value_ptr[4]);
                    switch (value_ptr[5]) {
                      case 0x30:  // Advance graps entries: No 8
                      case 0x32:  // Advance graps entries: No 9
                      case 0x34:  // Advance graps entries: No 10, is created automatically by app when enough points are inserted
                        rx_cmd_found++;
                        nr = TuneGraphNo2decimal(value_ptr[4], value_ptr[5]) / 2 ;
                        printf("%02X (ADD)RPM value %04d rpm at curve postion No: %d\n", value_ptr[5], TuneRPM2decimal(value_ptr[0], value_ptr[1]), nr );
                        // Store new value in internal server state: range checking should be done here
                        server->tune_AdvanceCurveRPM[nr - 1][0] = value_ptr[0];
                        server->tune_AdvanceCurveRPM[nr - 1][1] = value_ptr[1];

                        // fill rest with N/A values
                        if(TuneRPM2decimal(value_ptr[0], value_ptr[1]) == 8000) {
                          // the number of degrees is variable, do not fixate this
                          //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr - 1][0],&server->tune_MapCurveDegrees[nr - 1][1]);

                          // re-use n variabele (as it has exactly the right value to start with) to finish aand fill the rest with default valuses
                          for(; nr < C_nr_123MapCurve_Elements;nr++) {
                            // "0xFF" = 0x46,0x46
                            server->tune_AdvanceCurveRPM[nr][0] = 0x46;  // "0xG " == N/A
                            server->tune_AdvanceCurveRPM[nr][1] = 0x46; // "0x G" == N/A
                            server->tune_AdvanceCurveDegrees[nr][0] = 0x46;  // "0xG " == N/A
                            server->tune_AdvanceCurveDegrees[nr][1] = 0x46; // "0x G"  == N/A
                            //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr][0],&server->tune_MapCurveDegrees[nr][1]);
                          }
                        }

                        break;
                      case 0x31:      // Degrees: no 8
                      case 0x33:  // Degrees: no 9
                      case 0x35:      // Degrees: no 10
                        rx_cmd_found++;
                        nr = (TuneGraphNo2decimal(value_ptr[4], value_ptr[5]) - 1) / 2;
                        printf("%02X (ADD)Advance value %02.2f degrees at curve postion No: %d\n", value_ptr[5], TuneAdvance2decimal(value_ptr[0], value_ptr[1]), nr );
                        // Store new value in internal server state: range checking should be done here
                        server->tune_AdvanceCurveDegrees[nr - 1][0] = value_ptr[0];
                        server->tune_AdvanceCurveDegrees[nr - 1][1] = value_ptr[1];
                        break;
                      //case 0x34:
                      //  printf("%02X (ADD)RPM value %04d rpm\n",value_ptr[5],TuneRPM2decimal(value_ptr[0],value_ptr[1]));
                      //  break;

                      // 33 30 2D 30 31 36 0D set PIN :: [value,value][30,31][pos-0x36]
                      // Example pincode '8888'
                      //command: '38-016'; (1), value 0x38='8'
                      //command: '38-017'; (2), value 0x38='8'
                      //command: '38-018'; (3), value 0x38='8'
                      //command: '38-019'; (4), value 0x38='8'

                      case 0x36:
                      case 0x37:
                      case 0x38:
                      case 0x39:  // and 0x40 perhaps ???? no more then 4 char's are allowed in 123-app... 0x40 may be used for something else.
                        rx_cmd_found++;
                        printf("%02X PINSET[%d]=\'%c\'\n", value_ptr[5], value_ptr[5] - 0x36 , TunePinCode2char(value_ptr[0], value_ptr[1]) );
                        // store value in internal server
                        server->tune_Pincode[value_ptr[5] - 0x36][0] = value_ptr[0];
                        server->tune_Pincode[value_ptr[5] - 0x36][1] = value_ptr[1];
                        break;

                      case 0x41:
                        // 35 35 2D 30 31 41 0D
                        // 35 35 2D 30 31 41 0D Immobilizer ON , Command: '55-01A\n'
                        if ( (value_ptr[0] == 0x35) && (value_ptr[1] == 0x35) ) {
                        rx_cmd_found++;
                          printf("%02X IMMOBILIZER: ON\n", value_ptr[5] );
                          server->tune_IMMOBILIZED = true;
                        }
                        // 41 41 2D 30 31 41 0D
                        // 41 41 2D 30 31 41 0D Immobilizer Off, Command: 'AA-01A\n'
                        if ( (value_ptr[0] == 0x41) && (value_ptr[1] == 0x41) ) {
                        rx_cmd_found++;
                          printf("%02X IMMOBILIZER: OFF \n", value_ptr[5] );
                          server->tune_IMMOBILIZED = false;
                        }
                        break;
                      case 0x45:
                        rx_cmd_found++;
                        printf("%02X Starts@RPM value %04d rpm\n", value_ptr[5], TuneRPM2decimal(value_ptr[0], value_ptr[1]) );
                        server->tune_MapCurveStartRPM[0] = value_ptr[0];
                        server->tune_MapCurveStartRPM[1] = value_ptr[1];
                        break;
                      case 0x46:
                        rx_cmd_found++;
                        printf("%02X MAX RPM value %04d rpm\n", value_ptr[5], TuneRPM2decimal(value_ptr[0], value_ptr[1]) );
                        server->tune_RPMLimit[0] = value_ptr[0];
                        server->tune_RPMLimit[1] = value_ptr[1];
                        break;
                      default:
                        printf("UNKNOWN Advance Curve Command\n");
                        break;
                    }

                    print_hex(value_ptr, len_cleaned);
                    /*
                      bt_gatt_server_send_notification(server->gatt,
                                                     server->tune_msrmt_handle,
                                                     value_ptr, len_cleaned);
                    */
                    server->ctrl_TX_Characteristic->setValue(value_ptr, len_cleaned);
                    server->ctrl_TX_Characteristic->notify();

                    break;
                  case 0x32:  // Vacuum map
                    printf("%s Vacuum MAP Curve Command Found----\n", __FUNCTION__);
                    // Pos 1 -> 35, 2 -> 37 ...

                    switch (value_ptr[5]) {
                      // can't adjust no 1
                      case 0x34:  // Advance graps entries: No 2
                      case 0x36:  // Advance graps entries: No 3
                      case 0x38:  // Advance graps entries: No 4
                      case 0x41:  // Advance graps entries: No 5
                      case 0x43:      // Advance graps entries: No 6
                      case 0x45:      // Advance graps entries: No 7
                        rx_cmd_found++;
                        nr = TuneGraphNo2decimal(value_ptr[4] - 2, value_ptr[5]) / 2; // graph starts @ ofsset of 2, uneven numbers
                        printf("%02X (ADD)pressure value %04d kP at curve postion No: %d\n", value_ptr[5], TunePressure2decimal(value_ptr[0], value_ptr[1]), nr  );
                        // Store new value in internal server state: range checking should be done here
                        server->tune_MapCurvePressure[nr - 1][0] = value_ptr[0];
                        server->tune_MapCurvePressure[nr - 1][1] = value_ptr[1];

                        // 100, has always 0 Degrees
                        // 200, has always 0 degrees, and is the last one. next array elements should be filled with "0xFF";

                        if(TunePressure2decimal(value_ptr[0], value_ptr[1]) == 100) {
                          // No need to enforce the 0.0, client software does
                          //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr - 1][0],&server->tune_MapCurveDegrees[nr - 1][1]);
                        }
                        if(TunePressure2decimal(value_ptr[0], value_ptr[1]) == 200) {
                          // the number of degrees is variable, do not fixate this
                          //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr - 1][0],&server->tune_MapCurveDegrees[nr - 1][1]);

                          // re-use n variabele (as it has exactly the right value to start with) to finish aand fill the rest with default valuses
                          for(; nr < C_nr_123MapCurve_Elements;nr++) {
                            // "0xFF" = 0x46,0x46
                            server->tune_MapCurvePressure[nr][0] = 0x46;  // "0xG " == N/A
                            server->tune_MapCurvePressure[nr][1] = 0x46; // "0x G" == N/A
                            server->tune_MapCurveDegrees[nr][0] = 0x46;  // "0xG " == N/A
                            server->tune_MapCurveDegrees[nr][1] = 0x46; // "0x G"  == N/A
                            //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr][0],&server->tune_MapCurveDegrees[nr][1]);
                          }
                        }

                        break;
                      case 0x33:      // Degrees: no 1
                      case 0x35:  // Degrees: no 2
                      case 0x37:      // Degrees: no 3
                      case 0x39:      // Degrees: no 4
                      case 0x42:      // Degrees: no 5
                      case 0x44:      // Degrees: no 6
                      case 0x46:      // Degrees: no 7
                        rx_cmd_found++;
                        nr = (TuneGraphNo2decimal(value_ptr[4] - 2, value_ptr[5]) - 1) / 2; // graph starts @ ofsset of 2, uneven numbers
                        printf("%02X (ADD)Crankshaft value %02.2f degrees at curve postion No: %d\n", value_ptr[5], TuneAdvance2decimal(value_ptr[0], value_ptr[1]), nr  );
                        // Store new value in internal server state: range checking should be done here
                        server->tune_MapCurveDegrees[nr - 1][0] = value_ptr[0];
                        server->tune_MapCurveDegrees[nr - 1][1] = value_ptr[1];
                        break;
                      default:
                        printf("UNKNOWN Vacuum MAP Curve sub-Command\n");
                        break;
                    }




                    print_hex(value_ptr, len_cleaned);
                    /*
                      bt_gatt_server_send_notification(server->gatt,
                                                     server->tune_msrmt_handle,
                                                     value_ptr, len_cleaned);
                    */
                    server->ctrl_TX_Characteristic->setValue(value_ptr, len_cleaned);
                    server->ctrl_TX_Characteristic->notify();

                    break;
                  case 0x33:  // Save command????
                    // 30:30:2d:30:32:44:0d
                    // 43:38:2d:30:32:45:0d
                    printf("%s Vacuum MAP Curve Command Found----\n", __FUNCTION__);

                    switch (value_ptr[5]) {
                      case 0x30:  // Advance graps entries: No 8
                      case 0x32:  // Advance graps entries: No 9
                      case 0x34:  // Advance graps entries: No 10, is created automatically by app when enough points are inserted
                        rx_cmd_found++;
                        nr = TuneGraphNo2decimal(value_ptr[4] - 2, value_ptr[5]) / 2; // graph starts @ ofsset of 2, uneven numbers
                        printf("%02X (ADD)pressure value %04d kP at curve postion No: %d\n", value_ptr[5], TunePressure2decimal(value_ptr[0], value_ptr[1]), nr  );
                        // Store new value in internal server state: range checking should be done here
                        server->tune_MapCurvePressure[nr - 1][0] = value_ptr[0];
                        server->tune_MapCurvePressure[nr - 1][1] = value_ptr[1];


                        // 100, has always 0 Degrees
                        // 200, has always 0 degrees, and is the last one. next array elements should be filled with "0xFF";

                        if(TunePressure2decimal(value_ptr[0], value_ptr[1]) == 100) {
                          // No need to enforce the 0.0, client software does
                          //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr - 1][0],&server->tune_MapCurveDegrees[nr - 1][1]);
                        }
                        if(TunePressure2decimal(value_ptr[0], value_ptr[1]) == 200) {
                          // the number of degrees is variable, do not fixate this
                          //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr - 1][0],&server->tune_MapCurveDegrees[nr - 1][1]);

                          // re-use n variabele (as it has exactly the right value to start with) to finish aand fill the rest with default valuses
                          for(; nr < C_nr_123MapCurve_Elements;nr++) {
                            // "0xFF" = 0x46,0x46
                            server->tune_MapCurvePressure[nr][0] = 0x46;  // "0xG " == N/A
                            server->tune_MapCurvePressure[nr][1] = 0x46; // "0x G" == N/A
                            server->tune_MapCurveDegrees[nr][0] = 0x46;  // "0xG " == N/A
                            server->tune_MapCurveDegrees[nr][1] = 0x46; // "0x G"  == N/A
                            //decimal2TuneAdvance(0.0,&server->tune_MapCurveDegrees[nr][0],&server->tune_MapCurveDegrees[nr][1]);
                          }
                        }

                        break;
                      case 0x31:      // Degrees: no 8
                      case 0x33:  // Degrees: no 9
                      case 0x35:      // Degrees: no 10
                        rx_cmd_found++;
                        nr = (TuneGraphNo2decimal(value_ptr[4] - 2, value_ptr[5]) - 1) / 2 ; // graph starts @ ofsset of 2, uneven numbers
                        printf("%02X (ADD)Crankshaft value %02.2f degrees at curve postion No: %d\n", value_ptr[5], TuneAdvance2decimal(value_ptr[0], value_ptr[1]), nr );
                        // Store new value in internal server state: range checking should be done here
                        server->tune_MapCurveDegrees[nr - 1][0] = value_ptr[0];
                        server->tune_MapCurveDegrees[nr - 1][1] = value_ptr[1];
                        break;
                      default:
                        printf("UNKNOWN Vacuum MAP Curve sub-Command\n");
                        break;
                    }





                    print_hex(value_ptr, len_cleaned);
                    /*
                      bt_gatt_server_send_notification(server->gatt,
                                                     server->tune_msrmt_handle,
                                                     value_ptr, len_cleaned);
                    */
                    server->ctrl_TX_Characteristic->setValue(value_ptr, len_cleaned);
                    server->ctrl_TX_Characteristic->notify();


                    break;

                  default:
                    printf("UNKNOWN write curve/map sub cmd received!!!\n");
                    print_hex(value_ptr, len_cleaned);

                    break;
                } // switch (value_ptr[4])

              } // if ( value_ptr[2] == 0x2D && value_ptr[3] == 0x30 && value_ptr[6] == 0x0D )
            } // if (len_cleaned == 7)
          } // write curve/map implementation 2017/02/07


          // check if we missed a command
          if(len_cleaned>0) {
            if(rx_cmd_found==0){
                        printf("UNKNOWN cmd received!!!\n");
                        print_hex(value_ptr, len_cleaned);
                        delay(4000);
            }
            if(rx_cmd_found==0){
                        printf("cmd received and it was decode\n");
                        print_hex(value_ptr, len_cleaned);
            }
            if(rx_cmd_found>1){
                        printf("cmd is decoded by multiple (%d) code blocks, this MUST be wrong!!!\n",rx_cmd_found);
                        print_hex(value_ptr, len_cleaned);
                        delay(4000);
            }
          } else {
              printf("Decoded as keepalive/empty command\n");
          }

        } // end begin 123response logic



      } //if (xSemaphoreTake(BLE_RX2TX_Semaphore, portMAX_DELAY) == pdTRUE){
  } // for (;;)
}

// ESP32 BLE library has a known/unsolved quirck, it can't NOTIFY from onRead or onWrite and visa versa
// https://github.com/espressif/arduino-esp32/issues/1982

// No duplex over This specific characteristic
// RX: requests received
// TX: responses send
class MyCallbacksS_CTRL_RX_TX: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      //std::string value_std_str = pCharacteristic->getValue();
      //size_t len=value_std_str.length();
      //uint8_t *value = (uint8_t*)value_std_str.c_str(); // to fix uint8_t vs char mismatch afer porting in one go

      Serial.print("MyCallbacksS_CTRL_RX_TX onWrite: "); Serial.println(pCharacteristic->getUUID().toString().c_str());

      if (pCharacteristic->getUUID().equals(TUNE123_Ctrl_TX_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onWrite TUNE123_Ctrl_TX_Characeristic_UUID ");
        return;
      }
      if (pCharacteristic->getUUID().equals(TUNE123_Ctrl_RX_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onWrite TUNE123_Ctrl_RX_Characeristic_UUID ");
          // Release and get BLE_RX_Responder_to_TX to work

          xSemaphoreGive(BLE_RX2TX_Semaphore);
          return;


      }

/*
      if (value_std_str.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value_std_str.length(); i++)
          Serial.print(value_std_str[i], HEX);

        Serial.println();
        Serial.println("*********");
      }
*/
    }
    void onRead(BLECharacteristic *pCharacteristic) {
      struct timeval tv;
      Serial.print("MyCallbacksS_CTRL_RX_TX onRead: "); Serial.println(pCharacteristic->getUUID().toString().c_str());

      if (pCharacteristic->getUUID().equals(TUNE123_Ctrl_RX_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onRead TUNE123_Ctrl_RX_Characeristic_UUID ");

      }
      if (pCharacteristic->getUUID().equals(TUNE123_Ctrl_TX_Characeristic_UUID)) {
        Serial.println("BLECharacteristicCallbacks onRead TUNE123_Ctrl_TX_Characeristic_UUID ");
      }



      gettimeofday(&tv, NULL);
      std::ostringstream os;
      os << "Time: " << tv.tv_sec;
      Serial.println("Callback read: ");
      //Serial.println((String(os.str()));
      pCharacteristic->setValue(os.str());
    }
};


















void InitBLE() {
  BLEDevice::init("123\\TUNE+");
  // Create the BLE Server
  BLEServer *BLE_Server_p = BLEDevice::createServer();

  // Set Server callback
  BLE_Server_p->setCallbacks(new MyServerCallbacks());

  // Create primary BLE Service
  BLEService *BLE_TUNE123_Service_p = BLE_Server_p->createService(Tune123_Service_BLEUUID);

  // Create the additional Battery Service
  BLEService *BLE_TUNE123_Battery_Service_p = BLE_Server_p->createService(Tune123_Battery_Service_BLEUUID);

  // Characteristics defined globally

  // Create a BLE Descriptor
  //heartRateMeasurementCharacteristics.addDescriptor(new BLE2902());
  // Descriptior are defined globally

  // Hmm, is this the way to do that?
  //TuneBattery_Level_Characteristic.addDescriptor(new BLE2902());

  BLE_TUNE123_Service_p->addCharacteristic(&heartRateMeasurementCharacteristics);
  heartRateDescriptor.setValue("Rate from 0 to 200");
  heartRateMeasurementCharacteristics.addDescriptor(&heartRateDescriptor);


  BLE_TUNE123_Service_p->addCharacteristic(&sensorPositionCharacteristic);
  sensorPositionDescriptor.setValue("Position 0 - 6");
  sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);

  BLE_TUNE123_Service_p->addCharacteristic(&variabele_demo_Characteristic);
  variabele_demoDescriptior.setValue("RW testing callback");
  //variabele_demo_Characteristic.setValue("Hello World");
  variabele_demo_Characteristic.addDescriptor(&variabele_demoDescriptior);
  // assign event handlers for characteristic
  variabele_demo_Characteristic.setCallbacks(new MyCallbacks());



  BLE_TUNE123_Service_p->addCharacteristic(&info_Characteristic);
  infoDescriptor.setValue("TUNE123 Info");
  info_Characteristic.addDescriptor(&infoDescriptor);
  // assign event handlers for characteristic
  info_Characteristic.setCallbacks(new MyCallbacks());

  BLE_TUNE123_Service_p->addCharacteristic(&body_Characteristic);
  bodyDescriptor.setValue("TUNE123 Body");
  body_Characteristic.addDescriptor(&bodyDescriptor);
  // assign event handlers for characteristic
  body_Characteristic.setCallbacks(new MyCallbacks());

  BLE_TUNE123_Service_p->addCharacteristic(&ctrl_RX_Characteristic);
  ctrl_RX_Descriptor.setValue("TUNE123 CTRL RX");
  ctrl_RX_Characteristic.addDescriptor(&ctrl_RX_Descriptor);
  // assign event handlers for characteristic
  ctrl_RX_Characteristic.setCallbacks(new MyCallbacksS_CTRL_RX_TX());   // need own callback due to huge size/logic


  BLE_TUNE123_Service_p->addCharacteristic(&ctrl_TX_Characteristic);
  ctrl_TX_Descriptor.setValue("TUNE123 CTRL TX");
  ctrl_TX_Characteristic.addDescriptor(&ctrl_TX_Descriptor);
  // assign event handlers for characteristic
  // This is required for 2902 controll
  ctrl_TX_Characteristic.setCallbacks(new MyCallbacksS_CTRL_RX_TX());   // need own callback due to huge size/logic
  // Hmm, is this the way to do that?
  // See BLE_uart.ino where the 2902 is configured on TX!
  BLE2902* ble2902_p=new BLE2902();
  ble2902_p->setNotifications(true);
  ctrl_TX_Characteristic.addDescriptor(ble2902_p);      // switching on/of TX channel, but how does this exactly work????






  // make it global available for state engine and callback functions (RX/TX)
  //
  server_global.ctrl_TX_Characteristic=&ctrl_TX_Characteristic;
  server_global.ctrl_RX_Characteristic=&ctrl_RX_Characteristic;

  // Tune battery service/characteristic
  BLE_TUNE123_Battery_Service_p->addCharacteristic(&TuneBattery_Level_Characteristic);
  TuneBatteryDescriptior.setValue("Rate from 0 to 200");
  TuneBattery_Level_Characteristic.addDescriptor(&TuneBatteryDescriptior);
  // assign event handlers for characteristic
  TuneBattery_Level_Characteristic.setCallbacks(new MyCallbacks());


  // The heartRate service is already advertized, now twice, disabled:
  //BLE_Server_p->getAdvertising()->addServiceUUID(Tune123_Service_BLEUUID);
  //BLE_Server_p->getAdvertising()->addServiceUUID(Tune123_Service_BLEUUID);
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(Tune123_Service_BLEUUID);
  pAdvertising->addServiceUUID(Tune123_Battery_Service_BLEUUID);

  pAdvertising->setScanResponse(true);
  //pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  //pAdvertising->setMinPreferred(0x12);

  //pAdvertising->setManufacturerData("1234Tune++");


  //advertisementData.setShortName("123\\TUNE+");
  Serial.print("advertisementData.getPayload().length(): ");Serial.println(advertisementData.getPayload().length());
  advertisementData.setFlags(0x1a); // 0x01

  // TX power 4   // 0x0A (ESP_BLE_AD_TYPE_TX_PWR)
  Serial.print("advertisementData.getPayload().length(): ");Serial.println(advertisementData.getPayload().length());
  //advertisementData.setTXPower(0x04);

  Serial.print("Tune123_Service_BLEUUID.bitSize(): ");Serial.println(Tune123_Service_BLEUUID.bitSize());
  advertisementData.setCompleteServices(Tune123_Service_BLEUUID); // 0x07

  Serial.print("advertisementData.getPayload().length(): ");Serial.println(advertisementData.getPayload().length());

  // Active custom payload BLE advertizing
  pAdvertising->setAdvertisementData(advertisementData);


  // Custom BLE scan response payload
  //  Format: MSB_Company_ID , LSB_Company_ID, 0xFF, 0x01, MSB_Device_ID, LSB_Device_ID
  //std::string manufacturerData = {(P123TUNEPlus_Company_ID>>8)&0xFF, (P123TUNEPlus_Company_ID & 0xFF ), 0xFF, 0x01, 0xF0, 0x40 }; // working #61504
  //std::string manufacturerData = {(P123TUNEPlus_Company_ID>>8)&0xFF, (P123TUNEPlus_Company_ID & 0xFF ), 0xFF, 0x01, 0xF0, 0x41 }; // modified #61505
  //std::string manufacturerData = {(P123TUNEPlus_Company_ID>>8)&0xFF, (P123TUNEPlus_Company_ID & 0xFF ), 0xFF, 0x01, 0xF1, 0x41 }; // modified #61761
  //std::string manufacturerData = {(P123TUNEPlus_Company_ID>>8)&0xFF, (P123TUNEPlus_Company_ID & 0xFF ), 0xF0, 0x01, 0xF1, 0x41 }; // modified not recognized
  //std::string manufacturerData = {(P123TUNEPlus_Company_ID>>8)&0xFF, (P123TUNEPlus_Company_ID & 0xFF ), 0xFF, 0x00, 0xF1, 0x41 }; // modified #61761
  //std::string manufacturerData = {(P123TUNEPlus_Company_ID>>8)&0xFF, (P123TUNEPlus_Company_ID & 0xFF ), 0xFF, 0x0F, 0xF1, 0x41 }; // modified : crashes 123Tune+ app!!!
  //std::string manufacturerData = {(P123TUNEPlus_Company_ID>>8)&0xFF, (P123TUNEPlus_Company_ID & 0xFF ), 0xFF, 0x02, 0xF1, 0x41 }; // modified : not recognized
  //std::string manufacturerData = {(P123TUNEPlus_Company_ID>>8)&0xFF, (P123TUNEPlus_Company_ID & 0xFF ), 0xFF, 0x00, 0xF1, 0x41 }; // modified : #61761 (repeated test)
  // 0xF141 = 61761 decimal (so last 2 digits represent the "unique ID/serial")
  std::string manufacturerData = {(P123TUNEPlus_Company_ID>>8)&0xFF, (P123TUNEPlus_Company_ID & 0xFF ), 0xFF, 0x01, (P123TUNEPlus_Device_ID>>8)&0xFF, (P123TUNEPlus_Device_ID & 0xFF ) };

  Serial.print("advertisementData.getPayload().length(): ");Serial.println(advertisementData.getPayload().length());
  scan_resp_advertisementData.setManufacturerData(manufacturerData );
  //advertisementData.getPayload().data();
  Serial.print("advertisementData.getPayload().length(): ");Serial.println(advertisementData.getPayload().length());

  // Activating custom BLE Scan response payload
  pAdvertising->setScanResponseData(scan_resp_advertisementData);



  // Start primary Service
  BLE_TUNE123_Service_p->start();

  // Start next BLE Service
  BLE_TUNE123_Battery_Service_p->start();

  // Start advertising
  //BLE_Server_p->getAdvertising()->start();
  BLEDevice::startAdvertising();


  bpm = 1;    // copied freom init loop as bpm is unknown there.... but what does it.
}


void  update_BLE_values() {
  static uint32_t previous_update_time = millis();
  uint32_t current_time = millis();

  current_time = millis();

  // TODO unlink from display update time
  // NOTE to frequent updates crashes device!!!! (for exampe each milli second)
  if ( ( current_time - previous_update_time ) >= 500 ) {
    previous_update_time = current_time;

    // BLE update stuff
    heart[1] = (byte)bpm;
    int energyUsed = 3000;
    heart[3] = energyUsed / 256;
    heart[2] = energyUsed - (heart[2] * 256);
    //Serial.println(bpm);

    heartRateMeasurementCharacteristics.setValue(heart, 8);
    heartRateMeasurementCharacteristics.notify();

    sensorPositionCharacteristic.setValue(hrmPos, 1);
    bpm++;
    // END BLE stuff

    variabele_demo_Characteristic.getValue();


    // 123tuNE updates.....
    // Simululation folow itś own path, but anyway as it does not work, intnegrate it here
    //tune_msrmt_cb();

  } else {
    // Serial.println("No time yet for BLE value updates");
  }
}


void fake_resp1(void){
    uint8_t str_resp[] = { 0x0d };
    size_t str_resp_len = sizeof(str_resp);
    server_s_t* server=&server_global;
    server->ctrl_TX_Characteristic->setValue(str_resp, str_resp_len);
    server->ctrl_TX_Characteristic->notify();

}
