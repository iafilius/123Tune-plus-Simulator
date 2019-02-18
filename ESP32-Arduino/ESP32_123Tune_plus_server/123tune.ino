#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <BLE2902.h>
#include <BLEAdvertising.h>

// still to be ported to Arduino/ESP32/BLE
// http://gbrault.github.io/gattclient/att-types_8h.html
#define   BT_ATT_ERROR_INVALID_OFFSET   0x07
#define   BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN   0x0D


#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

struct server_s_t {
  int fd;
  /*
  struct bt_att *att;
  struct gatt_db *db;
  struct bt_gatt_server *gatt;
  */
  // Port Arduino
  BLEServer *BLE_Server_p;
  BLEService *BLE_TUNE123_Service_p;
  BLEService *BLE_TUNE123_Battery_Service_p;
  BLECharacteristic *ctrl_RX_Characteristic;
  BLECharacteristic *ctrl_TX_Characteristic;    // tune_msrmt


  // End Port Arduino

  uint8_t *device_name;
  size_t name_len;

  uint16_t gatt_svc_chngd_handle;
  bool svc_chngd_enabled;

  uint16_t hr_handle;
  uint16_t hr_msrmt_handle;
  uint16_t hr_energy_expended;

  uint16_t tune_handle;
  uint16_t tune_msrmt_handle;
  uint16_t tune_energy_expended;

  bool hr_visible;
  bool hr_msrmt_enabled;
  bool tune_visible;
  bool tune_msrmt_enabled;

  bool tune_TuningMode_enabled; // In Tuning mode, or not
  int tune_TuningMode_Advance;  // degrees Advance shift in Tuning mode, positive or negative, max value ???
  bool tune_IMMOBILIZED;    // immobilized? or not

  // Internal values only, desired state is not same as actual state, perhaps statore desired state as well??
  uint8_t tune_RPM[2];    // Actual RPM
  uint8_t tune_Advance[2];  // Actual Advance based on curve and Tuning
  uint8_t tune_Pressure[2]; // Actual measured pressure
  uint8_t tune_Temperature[2];  // Actual Temperature
  uint8_t tune_Voltage[2];  // Actual Voltage
  uint8_t tune_Ampere[2];   // Actual Ampere
  uint8_t tune_UNDISCOVERED[2]; // Unknown until now, let's see later


  // 2 arrays make one graphs, arrays are bluetooth command organized.
  uint8_t tune_AdvanceCurveRPM[C_nr_123AdvanceCurve_Elements][2];    // Pos No  1 ... 10  , Value[2]
  uint8_t tune_AdvanceCurveDegrees[C_nr_123AdvanceCurve_Elements][2];  // Pos No  1 ... 10  , Value[2]
  uint8_t tune_RPMLimit[2];     // Above this...

  // MAP curve
  uint8_t tune_MapCurvePressure[C_nr_123MapCurve_Elements][2];   // Pos No  1 ... 10  , Value[2]
  uint8_t tune_MapCurveDegrees[C_nr_123MapCurve_Elements][2];    // Pos No  1 ... 10  , Value[2]
  uint8_t tune_MapCurveStartRPM[2];   // below this value no MapCurve Active.

 // 4 elements should be enough?
  uint8_t tune_Pincode[10][2];      // Strings stored in transport format, as is the other variables

  int hr_ee_count;
  int tune_ee_count;
  unsigned int hr_timeout_id;
  unsigned int tune_timeout_id;
} server_global;


// declaration
void print_hex(const uint8_t* p, size_t len);
uint32_t TuneRPM2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneRPM(uint32_t RPM, uint8_t *MSB, uint8_t *LSB); // inverse from TuneRPM2decimal

double TuneAdvance2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneAdvance(double Advance, uint8_t *MSB, uint8_t *LSB); // inverse TuneAdvance2decimal


uint32_t TuneGraphNo2decimal(uint8_t MSB, uint8_t LSB);

uint32_t TunePressure2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TunePressure(uint32_t Pressure, uint8_t *MSB, uint8_t *LSB);   // Inverse TunePressure2decimal

int32_t  TuneTemperature2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneTemperature(double Temperature, uint8_t *MSB, uint8_t *LSB);   // inverse

double   TuneAmpere2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneAmpere(double Ampere, uint8_t *MSB, uint8_t *LSB);   //inverse

double TuneVoltage2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneVoltage(double Voltage, uint8_t *MSB, uint8_t *LSB);   // inverse


char TunePinCode2char(uint8_t MSB, uint8_t LSB);
uint32_t char2TunePinCode(char PinChar, uint8_t *MSB, uint8_t *LSB);    // inverse


// Calcualte Advance based on current RPM and RPM/Advance Graph
double CalculateAdvanceByRPM(void *user_data);

// Calcualte Advance based on current Pressure and Pressure(Vacuum)/Advance Graph
double CalculateAdvanceByPressure(void *user_data);


// update Actual calculated Advance
void UpdateRealtimeTuneAdvance(void *user_data);



// calculated and write byte number 3
uint32_t TuneSetnewChecksum(const uint8_t* str,uint8_t *csum);
#define ROUND_2_INT(f) ((int)(f >= 0.0 ? (f + 0.5) : (f - 0.5)))  // Avoid chopping when float to int, use rounding when a bit matters..



// definition
void print_hex(const uint8_t* p, size_t len)
{
size_t i;

printf("len=%02u: ",len);
for (i=0; i< len;i++)
  {
  printf("%02X ", p[i]);
  }

printf("\n");
}


//class MyCallbacks: public BLECharacteristic_Callback_tune_msrmt_cb
static bool tune_msrmt_cb(void)
//
{
  //struct server_s_t *server = (server_s_t *)user_data;
  server_s_t* server=&server_global;

  // bool expended_present = !(server->tune_ee_count % 10);
  uint16_t len = 1;
  uint8_t pdu[100];
  uint32_t cur_ee;
  static int iteration=-10;


  pdu[0] = 0x0d;
  //pdu[1] = 90 + (rand() % 40);

  //printf("SIMULATE: %s %d\n",__FUNCTION__,iteration);

  switch (iteration++) {
    case -20:
    case -19:
    case -18:
    case -17:
    case -16:
    case -15:
    case -14:
    case -13:
    case -12:
    case -11:
    case -10:
    case -9:
    case -8:
    case -7:
    case -6:
    case -5:
    case -4:
    case -3:
    case -2:
    case -1:
    case 0:
      pdu[0]=0x0d;
      len=1;
      //if (iteration==0) iteration+=20;  // jump to default
      return true;
      break;
/*    case 1:
      { //30:42:45:67:20 87.86 Sec
      uint8_t str[] = { 0x30, 0x42, 0x45, 0x67, 0x20 } ;
      len=sizeof(str);
      memcpy(pdu,str,len);
      }
      break;


    case 2:
      { // 31:30:30:41:20 88.65Sec
      uint8_t str[] = { 0x31, 0x30, 0x30, 0x41, 0x20 };
      len=sizeof(str);
      memcpy(pdu,str,len);
      }
      break;
    case 3:
      { // 32:36:34:4c:20 89.139 Sec
      uint8_t str[] = { 0x32, 0x36, 0x34, 0x4c, 0x20 };
      len=sizeof(str);
      memcpy(pdu,str,len);
      }
      break;
    case 4:
      { // 33:33:30:46:20 89,499 Sec
      uint8_t str[] = { 0x33, 0x33, 0x30, 0x46, 0x20 };
      len=sizeof(str);
      memcpy(pdu,str,len);
      }
      break;
    case 5:
      { // 30:30:41:51:20 89.799 sec
      uint8_t str[] = { 0x30, 0x30, 0x41, 0x51, 0x20 };
      len=sizeof(str);
      memcpy(pdu,str,len);
      }
      break;
    case 6:
      { // 31:30:30:41:20 90.159 sec
      uint8_t str[] = { 0x31, 0x30, 0x30, 0x41, 0x20 };
      len=sizeof(str);
      memcpy(pdu,str,len);
      }
      break;
    case 7:
      { // 32:36:34:4c:20 90.369
      uint8_t str[] = { 0x32, 0x36, 0x34, 0x4c, 0x20 };
      //return true;
      len=sizeof(str);
      memcpy(pdu,str,len);
      }
      break;
    case 8:
      { // 34:31:30:45:20 90.669
      uint8_t str[] = { 0x34, 0x31, 0x30, 0x45, 0x20 };
      //return true;
      len=sizeof(str);
      memcpy(pdu,str,len);
      }
      break;
    case 9:
      { // 30:30:41:51:20 90.88
      uint8_t str[] = { 0x30, 0x30, 0x41, 0x51, 0x20 };
      //return true;
      len=sizeof(str);
      memcpy(pdu,str,len);
      }
      break;
*/
    case 1:
      { // RPM initial special value
      //uint8_t str[] = { 0x30, 0x42, 0x45, 0x67, 0x20 }; // 8000 RPM (eerste waarde die word doorgestuurd (is om max schaal aan te geven wellicht?
      uint8_t str[] = { 0x30, 0x30, 0x41, 0x51, 0x20 }; // eerste waarde die word doorgestuurd (is om m

      //decimal2TuneRPM(1000,&str[1],&str[2]);      // set RPM by decimal value, TODO checksum??
      //TuneSetnewChecksum(str,&str[3]);      // set str[3]

      printf("\n\nSIMULATE: %s Special Start RPM set to: %d RPM\n",__FUNCTION__,TuneRPM2decimal(str[1],str[2]));     // test reverse function  (and forward)

      // Prooved Values (initial
      //  uint8_t str[] = { 0x30, 0x42, 0x45, 0x67, 0x20 }; // 8000 RPM (eerste waarde die word doorgestuurd (is om max schaal aan te geven wellicht?

      len=sizeof(str);
      memcpy(pdu,str,len);
      print_hex(pdu,len);
      /*bt_gatt_server_send_notification(server->gatt,
            server->tune_msrmt_handle,
            pdu, len);*/
      server->ctrl_TX_Characteristic->setValue(pdu,len);
      server->ctrl_TX_Characteristic->notify();
      }


      // set new Temperature in server internal
      decimal2TuneTemperature(60 ,&server->tune_Temperature[0],&server->tune_Temperature[1]);

      // set new Voltage in server Internal
      decimal2TuneVoltage(14.2 ,&server->tune_Voltage[0],&server->tune_Voltage[1]);

      // set new pressure in server Internal
      decimal2TunePressure(20 ,&server->tune_Pressure[0],&server->tune_Pressure[1]);


      // set new Advance in server Internal <static superceded by dynamic graph based code>
      // Should by dynamic base don Curve/RPM/Vacumm <TODO>
      decimal2TuneAdvance(10.0 ,&server->tune_Advance[0],&server->tune_Advance[1]);

      // set Temperature in server internal
      decimal2TuneAmpere(2.8 ,&server->tune_Ampere[0],&server->tune_Ampere[1]);

      // Set new RPM in internal system
      decimal2TuneRPM(1500,&server->tune_RPM[0],&server->tune_RPM[1]);

      break;







    default:
      { // RPM
      //uint8_t str[] = { 0x30, 0x35, 0x30, 0x45, 0x20 };
      //uint8_t str[] = { 0x30, 0x30, 0x43, 0x53, 0x20 };
      //uint8_t str[] = { 0x30, 0x36, 0x45, 0x5b, 0x20 };   // 5500 RPM uit MAP
      uint8_t str[] = { 0x30, 0x00, 0x00, 0x00, 0x20 }; // MAX RPM 4500????? nee ~5000 RPM



      str[1]=server->tune_RPM[0];     // read actual values from internal state
      str[2]=server->tune_RPM[1];
      TuneSetnewChecksum(str,&str[3]);    // set str[3] checksum

      printf("\n\nSIMULATE: %s RPM was set to: %d RPM\n",__FUNCTION__,TuneRPM2decimal(str[1],str[2])  );     // test reverse function  (and forward)


      // Prooved Values
      //  uint8_t str[] = { 0x30, 0x30, 0x30, 0x40, 0x20 }; // 0 (meter slat niet uit, maar past perfect in rei, en opvolgens past ook

      //  uint8_t str[] = { 0x30, 0x30, 0x31, 0x41, 0x20 }; // 50 RPM
      //  uint8_t str[] = { 0x30, 0x30, 0x43, 0x53, 0x20 }; // 500 RPM
      //  uint8_t str[] = { 0x30, 0x30, 0x41, 0x51, 0x20 }; // 500+ RPM
      //  uint8_t str[] = { 0x30, 0x30, 0x43, 0x53, 0x20 }; // 600 RPM
      //  uint8_t str[] = { 0x30, 0x30, 0x45, 0x55, 0x20 }; // 700 RPM
      //  uint8_t str[] = { 0x30, 0x30, 0x46, 0x56, 0x20 }; // 800 RPM


      //  uint8_t str[] = { 0x30, 0x31, 0x30, 0x41, 0x20 }; // 800 RPM
      //  uint8_t str[] = { 0x30, 0x31, 0x31, 0x42, 0x20 }; // 900+ RPM
      //  uint8_t str[] = { 0x30, 0x31, 0x33, 0x44, 0x20 }; // 1000-RPM
      //  uint8_t str[] = { 0x30, 0x31, 0x34, 0x45, 0x20 }; // 1000 RPM
      //  uint8_t str[] = { 0x30, 0x31, 0x35, 0x46, 0x20 }; // 1000+ RPM
      //  uint8_t str[] = { 0x30, 0x31, 0x38, 0x49, 0x20 }; // 1200
      //  uit capture nog waarde paren als 42,53  44,55 45,56

      //  uint8_t str[] = { 0x30, 0x31, 0x46, 0x57, 0x20 }; // 1600- RPM

      //  uint8_t str[] = { 0x30, 0x32, 0x30, 0x42, 0x20 }; // 1600 RPM
      //  uint8_t str[] = { 0x30, 0x32, 0x32, 0x44, 0x20 }; // 1700 RPM
      //  uint8_t str[] = { 0x30, 0x32, 0x33, 0x45, 0x20 }; // 1800- RPM
      //  uint8_t str[] = { 0x30, 0x32, 0x34, 0x46, 0x20 }; // 1800 RPM
      //  uint8_t str[] = { 0x30, 0x32, 0x46, 0x58, 0x20 }; // 2400 RPM

      //  uint8_t str[] = { 0x30, 0x33, 0x30, 0x43, 0x20 }; // 2400 RPM
      //  uint8_t str[] = { 0x30, 0x33, 0x31, 0x44, 0x20 }; // 2400+ RPM
      //  uint8_t str[] = { 0x30, 0x33, 0x44, 0x57, 0x20 }; // 3000 RPM

      //  uint8_t str[] = { 0x30, 0x34, 0x30, 0x44, 0x20 }; // 3200 RPM
      //  uint8_t str[] = { 0x30, 0x34, 0x38, 0x4c, 0x20 }; // 3600- RPM

      //  uint8_t str[] = { 0x30, 0x35, 0x30, 0x45, 0x20 }; // 4000 RPM
      //  uint8_t str[] = { 0x30, 0x35, 0x31, 0x46, 0x20 }; // 4000+ RPM
      //  uint8_t str[] = { 0x30, 0x35, 0x32, 0x47, 0x20 }; // 4050 RPM

      //  uint8_t str[] = { 0x30, 0x36, 0x30, 0x46, 0x20 }; // 4800 RPM

      //uint8_t str[] = { 0x30, 0x36, 0x45, 0x5b, 0x20 };   // 5500 RPM uit MAP
      //  uint8_t str[] = { 0x30, 0x37, 0x30, 0x47, 0x20 }; // 5600 RPM

      //  uint8_t str[] = { 0x30, 0x38, 0x30, 0x48, 0x20 }; // 6400 RPM
      //  uint8_t str[] = { 0x30, 0x39, 0x30, 0x49, 0x20 }; // 7200 RPM

      //  uint8_t str[] = { 0x30, 0x40, 0x30, 0x50, 0x20 }; // deze laat meter niet meer uitslaan...


      //  uint8_t str[] = { 0x30, 0x42, 0x45, 0x67, 0x20 }; // 8000 RPM (eerste waarde die word doorgestuurd (is om max schaal aan te geven wellicht?

      len=sizeof(str);
      memcpy(pdu,str,len);
      print_hex(pdu,len);
      /*
      bt_gatt_server_send_notification(server->gatt,
            server->tune_msrmt_handle,
            pdu, len);*/
      server->ctrl_TX_Characteristic->setValue(pdu,len);
      server->ctrl_TX_Characteristic->notify();
      }


      { // Advance
      static int i=0x00;
      static int j=0x40;
      //uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };
      //uint8_t str[] = { 0x31, 0x46, 0x30, 0x57, 0x20 };   // 48,50
      //uint8_t str[] = { 0x31, 0x40, 0x30, 0x51, 0x20 };         //
      // uint8_t str[] = { 0x31, 0x30, 0x46, 0x57, 0x20 };    //  3 degrees
      //uint8_t str[] = { 0x31, 0x31, 0x30, 0x42, 0x20 };         // 3,25 decgrees
      //uint8_t str[] = { 0x31, 0x45, 0x31, 0x57, 0x20 };    // 45, 31 == 45 from map command
      uint8_t str[] = { 0x31, 0x00, 0x00, 0x00, 0x20 };    // 11 degrees from map/vacuum


      // JIT update the actual (calcualated) Advance values, before use
      UpdateRealtimeTuneAdvance(server);

      str[1]=server->tune_Advance[0];     // read actual values from internal state
      str[2]=server->tune_Advance[1];
      TuneSetnewChecksum(str,&str[3]);    // set str[3]

      printf("SIMULATE: %s Advance was set to: %2.2f degrees\n",__FUNCTION__,TuneAdvance2decimal(str[1],str[2]));     // test reverse function  (and forward)


      // prooved collection
      // uint8_t str[] = { 0x31, 0x30, 0x30, 0x41, 0x20 };        //  0 from captrue
      // uint8_t str[] = { 0x31, 0x30, 0x46, 0x57, 0x20 };    //  3 degrees

      // uint8_t str[] = { 0x31, 0x31, 0x30, 0x42, 0x20 };        // 3,25 decgrees
      // uint8_t str[] = { 0x31, 0x32, 0x30, 0x43, 0x20 };        // 6,5 decgrees
      // uint8_t str[] = { 0x31, 0x33, 0x30, 0x44, 0x20 };        // 10- decgrees
      // uint8_t str[] = { 0x31, 0x34, 0x30, 0x45, 0x20 };        // 12,5 decgrees
      // uint8_t str[] = { 0x31, 0x35, 0x30, 0x46, 0x20 };        // 16 decgrees
      // uint8_t str[] = { 0x31, 0x36, 0x30, 0x47, 0x20 };        // 19 decgrees
      // uint8_t str[] = { 0x31, 0x37, 0x30, 0x48, 0x20 };        // 22 decgrees
      // uint8_t str[] = { 0x31, 0x38, 0x30, 0x49, 0x20 };        // 25+ decgrees
      // uint8_t str[] = { 0x31, 0x39, 0x30, 0x4a, 0x20 };        // 28,5 decgrees
      // uint8_t str[] = { 0x31, 0x3a, 0x30, 0x4b, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3b, 0x30, 0x4c, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3c, 0x30, 0x4d, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3d, 0x30, 0x4e, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3e, 0x30, 0x4f, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3f, 0x30, 0x50, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x40, 0x30, 0x51, 0x20 };        // NA!!!!

      // uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };    // 31,8 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x31, 0x53, 0x20 };    // 32.0 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x32, 0x54, 0x20 };    // 32,2 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x33, 0x55, 0x20 };    // 32,4 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x34, 0x56, 0x20 };    // 32,6 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x35, 0x57, 0x20 };    // 32,8 degrees Advance

      //
      // uint8_t str[] = { 0x31, 0x41, 0x46, 0x68, 0x20 };    // 35.0 degrees Advance
      // 69 niets...
      // uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };       // 32--
      // uint8_t str[] = { 0x31, 0x42, 0x30, 0x53, 0x20 };  // 35.0
      // uint8_t str[] = { 0x31, 0x43, 0x30, 0x54, 0x20 };  // 38,50
      // uint8_t str[] = { 0x31, 0x44, 0x30, 0x55, 0x20 };  // 41,5
      // uint8_t str[] = { 0x31, 0x45, 0x30, 0x56, 0x20 };  // 45
      // uint8_t str[] = { 0x31, 0x46, 0x30, 0x57, 0x20 };  // 48,50



      len=sizeof(str);
      //for (i=0;i<256;i++)
        {
        //str[2]=i;
        //str[3]=j;
        printf("SIMULATE: uint8_t[2][3]=%02X  %02X------------------------------\n",i,j);

        memcpy(pdu,str,len);
        print_hex(pdu,len);

        /*
        bt_gatt_server_send_notification(server->gatt,
            server->tune_msrmt_handle,
            pdu, len); */
        server->ctrl_TX_Characteristic->setValue(pdu,len);
        server->ctrl_TX_Characteristic->notify();
        }
      //j++;
      //i++;
      }

      { // BAR
      //static int i=0x0,j;

      // uint8_t str[] = { 0x32, 0x36,0x34, 0x4c, 0x20 };  //  0 BAR
      //uint8_t str[] = { 0x32, 0x37, 0x30, 0x49, 0x20 }; //  0.1 BAR
      //uint8_t str[] = { 0x32, 0x38, 0x30, 0x4a, 0x20 }; //  0.295(-) BAR
      //uint8_t str[] = { 0x32, 0x39, 0x30, 0x4b, 0x20 }; //  0.45 BAR
      //uint8_t str[] = { 0x32, 0x3a, 0x30, 0x4c, 0x20 }; //  0,45 BAR
      //uint8_t str[] = { 0x32, 0x3b, 0x30, 0x4d, 0x20 }; // geen response
      //uint8_t str[] = { 0x32, 0x3c, 0x30, 0x4e, 0x20 }; //  geen response
      //uint8_t str[] = { 0x32, 0x35, 0x35, 0x4c, 0x20 }; //  - 0.04 BAR ???
      //uint8_t str[] = { 0x32, 0x35, 0x39, 0x50, 0x20 }; //  -0.11
      //uint8_t str[] = { 0x32, 0x35, 0x3a, 0x51, 0x20 }; //  opvolger van 0x35, 0x39 , maar doet het niet meer....
      //uint8_t str[] = { 0x32, 0x35, 0x45, 0x5c, 0x20 }; //  uit MAP edit... 94kP (-0.4BAR) == 1 strepje, 0.06, maar meter is blijkbaar niet erg naukeurig af te lezen
      //uint8_t str[] = { 0x32, 0x36, 0x30, 0x48, 0x20 }; //  - 0,04 BAR of zo (minder dan 1 streekpje, streepje is 0.6BAR
      uint8_t str[] = { 0x32, 0x00, 0x00, 0x00, 0x20 }; //  empty pressure Frame


      //decimal2TunePressure(100 ,&str[1],&str[2]);   // set pressure in kP, 100= 0BAR, 0 = -1 BAR

      //decimal2TunePressure(100 ,&server->tune_Pressure[0],&server->tune_Pressure[1]);   // set Pressure in server internal



      str[1]=server->tune_Pressure[0];      // read actual values from internal state
      str[2]=server->tune_Pressure[1];

      TuneSetnewChecksum(str,&str[3]);      // set str[3]

      printf("SIMULATE: %s presure was set to: %d kP\n",__FUNCTION__,TunePressure2decimal(str[1],str[2]));     // test reverse function  (and forward)

      // proved collection
      //uint8_t str[] = { 0x32, 0x30, 0x30, 0x42, 0x20 }; // -1 BAR
      // uint8_t str[] = { 0x32, 0x31, 0x30, 0x43, 0x20 };  // -0,8 BAR
      // uint8_t str[] = { 0x32, 0x32, 0x30, 0x44, 0x20 };  // -0,6 BAR
      // uint8_t str[] = { 0x32, 0x33, 0x30, 0x45, 0x20 };  // -0,45 BAR
      // uint8_t str[] = { 0x32, 0x34, 0x30, 0x46, 0x20 };  // -0,3 BAR
      // uint8_t str[] = { 0x32, 0x35, 0x30, 0x47, 0x20 };  // -0,2 BAR
      // uint8_t str[] = { 0x32, 0x35, 0x39, 0x50, 0x20 };  //  -0.11 // hoogste waarde, opvolger 0x3a werkt niet meer.
      //uint8_t str[] = { 0x32, 0x35, 0x3a, 0x51, 0x20 }; //  opvolger van 0x35, 0x39 , maar doet het niet meer....
      // uint8_t str[] = { 0x32, 0x35, 0x45, 0x5c, 0x20 };  //  uit MAP edit... 94kP (-0.4BAR) == 1 strepje, 0.06, maar meter is blijkbaar niet erg naukeurig af te lezen



      // prooved collection
      // uint8_t str[] = { 0x31, 0x30, 0x30, 0x41, 0x20 };        //  0 from captrue
      // uint8_t str[] = { 0x31, 0x30, 0x46, 0x57, 0x20 };    //  3 degrees

      // uint8_t str[] = { 0x31, 0x31, 0x30, 0x42, 0x20 };        // 3,25 decgrees
      // uint8_t str[] = { 0x31, 0x32, 0x30, 0x43, 0x20 };        // 6,5 decgrees
      // uint8_t str[] = { 0x31, 0x33, 0x30, 0x44, 0x20 };        // 10- decgrees
      // uint8_t str[] = { 0x31, 0x34, 0x30, 0x45, 0x20 };        // 12,5 decgrees
      // uint8_t str[] = { 0x31, 0x35, 0x30, 0x46, 0x20 };        // 16 decgrees
      // uint8_t str[] = { 0x31, 0x36, 0x30, 0x47, 0x20 };        // 19 decgrees
      // uint8_t str[] = { 0x31, 0x37, 0x30, 0x48, 0x20 };        // 22 decgrees
      // uint8_t str[] = { 0x31, 0x38, 0x30, 0x49, 0x20 };        // 25+ decgrees
      // uint8_t str[] = { 0x31, 0x39, 0x30, 0x4a, 0x20 };        // 28,5 decgrees
      // uint8_t str[] = { 0x31, 0x3a, 0x30, 0x4b, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3b, 0x30, 0x4c, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3c, 0x30, 0x4d, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3d, 0x30, 0x4e, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3e, 0x30, 0x4f, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x3f, 0x30, 0x50, 0x20 };        // NA!!!!
      // uint8_t str[] = { 0x31, 0x40, 0x30, 0x51, 0x20 };        // NA!!!!

      // uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };    // 31,8 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x31, 0x53, 0x20 };    // 32.0 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x32, 0x54, 0x20 };    // 32,2 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x33, 0x55, 0x20 };    // 32,4 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x34, 0x56, 0x20 };    // 32,6 degrees Advance
      // uint8_t str[] = { 0x31, 0x41, 0x35, 0x57, 0x20 };    // 32,8 degrees Advance

      //
      // uint8_t str[] = { 0x31, 0x41, 0x46, 0x68, 0x20 };    // 35.0 degrees Advance
      // 69 niets...
      // uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };       // 32--
      // uint8_t str[] = { 0x31, 0x42, 0x30, 0x53, 0x20 };  // 35.0
      // uint8_t str[] = { 0x31, 0x43, 0x30, 0x54, 0x20 };  // 38,50
      // uint8_t str[] = { 0x31, 0x44, 0x30, 0x55, 0x20 };  // 41,5
      // uint8_t str[] = { 0x31, 0x45, 0x30, 0x56, 0x20 };  // 45
      // uint8_t str[] = { 0x31, 0x46, 0x30, 0x57, 0x20 };  // 48,50



      len=sizeof(str);
      //for (i=0;i<256;i++)
        {
        //str[2]=i;
        //str[3]=j;
        //printf("uint8_t[2][3]=%02X  %02X------------------------------\n",i,j);

        memcpy(pdu,str,len);
        print_hex(pdu,len);
       /*
        bt_gatt_server_send_notification(server->gatt,
            server->tune_msrmt_handle,
            pdu, len); */
        server->ctrl_TX_Characteristic->setValue(pdu,len);
        server->ctrl_TX_Characteristic->notify();

        }
      //j++;
      //i++;
      }



      { // 0x33 Temperature
      // uint8_t str[] = { 0x33, 0x30, 0x30, 0x43, 0x20 };  // -20 (tegen aanslag)
      // uint8_t str[] = { 0x33, 0x31, 0x30, 0x44, 0x20 };  // -17 graden
      // uint8_t str[] = { 0x33, 0x32, 0x30, 0x45, 0x20 };  // 1 graad boven 0
      // uint8_t str[] = { 0x33, 0x33, 0x30, 0x46, 0x20 };  // from capture, 19 graden
      // uint8_t str[] = { 0x33, 0x34, 0x30, 0x47, 0x20 };  // 37 graden
      // uint8_t str[] = { 0x33, 0x35, 0x30, 0x48, 0x20 };  // 50  graden
      // uint8_t str[] = { 0x33, 0x36, 0x30, 0x49, 0x20 };  // 66  graden
      //uint8_t str[] = { 0x33, 0x37, 0x30, 0x4a, 0x20 }; // 82  graden
      //uint8_t str[] = { 0x33, 0x38, 0x30, 0x4b, 0x20 }; // 99  graden
      uint8_t str[] = { 0x33, 0x00, 0x00, 0x00, 0x20 };   // Empty Temperature string


      //decimal2TuneTemperature(60 ,&str[1],&str[2]);   // celcius
      //decimal2TuneTemperature(60 ,&server->tune_Temperature[0],&server->tune_Temperature[1]);   // set Temperature in server internal


      str[1]=server->tune_Temperature[0];     // read actual values from internal state
      str[2]=server->tune_Temperature[1];

      TuneSetnewChecksum(str,&str[3]);      // set str[3]

      printf("SIMULATE: %s Temperature was set to: %d Celcius\n",__FUNCTION__,TuneTemperature2decimal(str[1],str[2]));  // test reverse function  (and forward)

      // Prooved Values
      // uint8_t str[] = { 0x33, 0x30, 0x30, 0x43, 0x20 };  // -20 (tegen aanslag)
      // uint8_t str[] = { 0x33, 0x31, 0x30, 0x44, 0x20 };  // -17 graden
      // uint8_t str[] = { 0x33, 0x32, 0x30, 0x45, 0x20 };  // 1 graad boven 0
      // uint8_t str[] = { 0x33, 0x33, 0x30, 0x46, 0x20 };  // from capture, 19 graden
      // uint8_t str[] = { 0x33, 0x34, 0x30, 0x47, 0x20 };  // 37 graden
      // uint8_t str[] = { 0x33, 0x35, 0x30, 0x48, 0x20 };  // 50  graden
      // uint8_t str[] = { 0x33, 0x36, 0x30, 0x49, 0x20 };  // 66  graden
      // uint8_t str[] = { 0x33, 0x37, 0x30, 0x4a, 0x20 };  // 82  graden
      // uint8_t str[] = { 0x33, 0x38, 0x30, 0x4b, 0x20 };  // 99  graden

      len=sizeof(str);
      memcpy(pdu,str,len);
      print_hex(pdu,len);

      /*
      bt_gatt_server_send_notification(server->gatt,
            server->tune_msrmt_handle,
            pdu, len);*/
      server->ctrl_TX_Characteristic->setValue(pdu,len);
      server->ctrl_TX_Characteristic->notify();
      }


      { // 0x34 Tuning mode!!!!
        // 34:31:30:45:20 [geen Tuning mode]
        // 34:31:31:46:20 [IN Tuning mode] (na 0x74 toggle)
      // uint8_t str[] = { 0x34, 0x30, 0x30, 0x44, 0x20 };  //  niets
      uint8_t str[] = { 0x34, 0x31, 0x30, 0x45, 0x20 }; // uit capture/geen tuning
      // uint8_t str[] = { 0x34, 0x32, 0x30, 0x46, 0x20 };  // niets
      // uint8_t str[] = { 0x34, 0x33, 0x30, 0x47, 0x20 };  // niets
      //uint8_t str[] = { 0x34, 0x34, 0x30, 0x48, 0x20 }; // niets


      // Prooved Values
      //uint8_t str[] = { 0x34, 0x31, 0x30, 0x45, 0x20 }; // uit capture

      if (server->tune_TuningMode_enabled) {
        str[2]=0x31;  // Enabled Tuning message
        // str[3]=0x46; // Enabled Tuning message   CSUM
      }
      TuneSetnewChecksum(str,&str[3]);
      printf("SIMULATE: %s Tuning mode was set to: '%c%c'\n",__FUNCTION__,str[1],str[2]);

      len=sizeof(str);
      memcpy(pdu,str,len);
      print_hex(pdu,len);

      /*
      bt_gatt_server_send_notification(server->gatt,
            server->tune_msrmt_handle,
            pdu, len);*/
      server->ctrl_TX_Characteristic->setValue(pdu,len);
      server->ctrl_TX_Characteristic->notify();

      }



      { // 0x35 Ampere
      //uint8_t str[] = { 0x35, 0x30, 0x30, 0x45, 0x20 }; // 0 Ampere
      //uint8_t str[] = { 0x35, 0x31, 0x30, 0x46, 0x20 }; // 1,85 Ampere
      //uint8_t str[] = { 0x35, 0x32, 0x30, 0x47, 0x20 }; // 2,75 Ampere
      //uint8_t str[] = { 0x35, 0x33, 0x30, 0x48, 0x20 }; // 4 Ampere (meter in hoek)
      //uint8_t str[] = { 0x35, 0x34, 0x30, 0x49, 0x20 }; // 4 Ampere (meter in hoek)

      //uint8_t str[] = { 0x35, 0x31, 0x30, 0x47, 0x20 }; // 1,85 Ampere
      //uint8_t str[] = { 0x35, 0x32, 0x30, 0x47, 0x20 }; // 2,75
      uint8_t str[] = { 0x35, 0x00, 0x00, 0x20, 0x20 };


      str[1]=server->tune_Ampere[0];      // read actual values from internal state
      str[2]=server->tune_Ampere[1];


      TuneSetnewChecksum(str,&str[3]);      // set str[3]

      printf("SIMULATE: %s Ampere was set to: %1.2f Ampere\n",__FUNCTION__,TuneAmpere2decimal(str[1],str[2]));     // test reverse function  (and forward)


      // Prooved Values
      //uint8_t str[] = { 0x35, 0x30, 0x30, 0x45, 0x20 }; // 0 Ampere
      //uint8_t str[] = { 0x35, 0x31, 0x30, 0x46, 0x20 }; // 1,85 Ampere
      //uint8_t str[] = { 0x35, 0x32, 0x30, 0x47, 0x20 }; // 2,75 Ampere
      //uint8_t str[] = { 0x35, 0x33, 0x30, 0x48, 0x20 }; // 4 Ampere (meter in hoek)
      //uint8_t str[] = { 0x35, 0x34, 0x30, 0x49, 0x20 }; // 4 Ampere (meter in hoek)

      len=sizeof(str);
      memcpy(pdu,str,len);
      print_hex(pdu,len);

      /*
      bt_gatt_server_send_notification(server->gatt,
            server->tune_msrmt_handle,
            pdu, len); */
      server->ctrl_TX_Characteristic->setValue(pdu,len);
      server->ctrl_TX_Characteristic->notify();
      }


      { // 0x41 Volt

      //uint8_t str[] = { 0x41, 0x34, 0x30, 0x55, 0x20 }; // 14,1 V
      uint8_t str[] = { 0x41, 0x00, 0x00, 0x00, 0x20 }; // empty Temperature Frame

      // Prooved Values
      // uint8_t str[] = { 0x41, 0x33, 0x30, 0x54, 0x20 };  // 11 V (tegen aanslag)
      // uint8_t str[] = { 0x41, 0x33, 0x32, 0x56, 0x20 };  // 11.0V (net tegen aanslag)
      // uint8_t str[] = { 0x41, 0x33, 0x33, 0x57, 0x20 };  // 11,2V
      // uint8_t str[] = { 0x41, 0x33, 0x35, 0x59, 0x20 };  // 11,6V
      // uint8_t str[] = { 0x41, 0x33, 0x46, 0x55, 0x20 };  // 13,9 V
      // uint8_t str[] = { 0x41, 0x33, 0x46, 0x55, 0x20 };  // no response  (out hex scale)

      // uint8_t str[] = { 0x41, 0x34, 0x30, 0x55, 0x20 };  // 14,1 V
      // uint8_t str[] = { 0x41, 0x34, 0x31, 0x56, 0x20 };  // uit capture, 14,3 V

      //decimal2TuneVoltage(14.0 ,&str[1],&str[2]);     // set Voltage

      //uint8_t str[] = { 0x41, 0x34, 0x30, 0x55, 0x20 }; // 14,1V  -> 0x40 ~= 14,1
      //decimal2TuneVoltage(14.0 ,&server->tune_Voltage[0],&server->tune_Voltage[1]);   // set Temperature in server internal


      str[1]=server->tune_Voltage[0];     // read actual values from internal state
      str[2]=server->tune_Voltage[1];

      TuneSetnewChecksum(str,&str[3]);      // set str[3]


      printf("SIMULATE: %s Voltage set to: %1.2f Volt\n",__FUNCTION__,TuneVoltage2decimal(str[1],str[2]));     // test reverse function  (and forward)



      len=sizeof(str);
      memcpy(pdu,str,len);
      print_hex(pdu,len);
      /*
      bt_gatt_server_send_notification(server->gatt,
            server->tune_msrmt_handle,
            pdu, len); */
      server->ctrl_TX_Characteristic->setValue(pdu,len);
      server->ctrl_TX_Characteristic->notify();
      }



      { // 0x42 ??? Tune mode iets ?, this is required to get 123 logged its datasets
        // Shift Light Active, GPS ms ????
      static int i=0,j=0,k=0,l=0;
      //uint8_t str[] = { 0x42, 0x34, 0x35, 0x5b, 0x0d }; // in capture always the same, even with running motor on drilling machine
      //uint8_t str[] = { 0x42, 0x35, 0x35, 0x5c, 0x20 }; // niets, maar foutief??
      //uint8_t str[] = { 0x42, 0x34, 0x35, 0x5b, 0x0d }; // niets (maar wel correct
      uint8_t str[] = { 0x42, 0x34, 0x35, 0x5b, 0x0d }; // Deze logged als B56[  0x42='B' 0x35='5'5 0x36='6' 0x5b='['


      //decimal2TuneUNDISCOVERED(14.0 ,&server->tune_UNDISCOVERED[0],&server->tune_UNDISCOVERED[1]);    // set UNDISCOVERED in server internal
      //misuse anotehr function to trough some random numbers;
      //decimal2TunePressure(i++ ,&server->tune_UNDISCOVERED[0],&server->tune_UNDISCOVERED[1]);
      //server->tune_UNDISCOVERED[0]=str[1];  // Set internal values
      //server->tune_UNDISCOVERED[1]=str[2];
      //str[2]=0x41;



      //str[1]=server->tune_UNDISCOVERED[0];      // read actual values from internal state
      //str[2]=server->tune_UNDISCOVERED[1];
      //str[0]=i++; // emperical test


      // This triggers GPSms column
      //str[1]=i; // emperical test
      //str[2]=i++; // emperical test
      //if (i>0x40)i=30;

/*
      str[0]=i;   // emperical test
      str[1]=j; // emperical test
      str[2]=k; // emperical test
      str[3]=l++; // emperical test
      if (l>=0x7F) {
        l=' ';
        k++;
      }
      if (k>=0x7F) {
        k=' ';
        j++;
      }
      if (j>=0x7F) {
        j=' ';
        i++;
      }
      if (i>=0x7F) {
        i=' ';
      }
*/
/*
    //char possible_chars[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    char possible_chars[]="0123456789AB";
    str[0]=possible_chars[i];   // emperical test
    str[1]=possible_chars[j]; // emperical test
    str[2]=possible_chars[k]; // emperical test
    str[3]=possible_chars[l++]; // emperical test
    if (l>=sizeof(possible_chars)) {
      l=0;
      k++;
    }
    if (k>=sizeof(possible_chars)) {
      k=0;
      j++;
    }
    if (j>=sizeof(possible_chars)) {
      j=0;
      i++;
    }
    if (i>=sizeof(possible_chars)) {
      i=0;
    }

      // end empirical testing


      // in analogy with other datset's assuming chsum is identical , reused concept here
      //TuneSetnewChecksum(str,&str[3]);      // set str[3]
*/
      printf("SIMULATE: %s UNDISCOVERED(ShiftLightActive?/GPSms?) %02X [%d,%d,%d,%d]\n",__FUNCTION__,str[0],i,j,k,l);

      // Prooved Values

      len=sizeof(str);
      memcpy(pdu,str,len);
      print_hex(pdu,len);

      /*
      bt_gatt_server_send_notification(server->gatt,
            server->tune_msrmt_handle,
            pdu, len); */
      server->ctrl_TX_Characteristic->setValue(pdu,len);
      server->ctrl_TX_Characteristic->notify();
      }




      break;
  }

  //print_hex(pdu,len);

  /*
  if (expended_present) {
    pdu[0] |= 0x08;
    put_le16(server->tune_energy_expended, pdu + 2);
    len += 2;
  }
  */

  //bt_gatt_server_send_notification(server->gatt,
  //          server->tune_msrmt_handle,
  //          pdu, len);


  cur_ee = server->tune_energy_expended;
  server->tune_energy_expended = MIN(UINT16_MAX, cur_ee + 10);
  server->tune_ee_count++;

  return true;
}


// simulate: notified RPM 2017/02/01 every seconds .. (from start)
static void update_tune_msrmt_simulation(struct server_s_t *server)
{
  if (!server->tune_msrmt_enabled || !server->tune_visible) {
    // 2019/01/13 temporary disabled for porting later time
    //timeout_remove(server->tune_timeout_id);
    BLE_timer_stop();
    return;
  }

    // 2019/01/13 temporary disabled for porting later time
    //server->tune_timeout_id = timeout_add(1000, tune_msrmt_cb, server, NULL);
    BLE_timer_start();
}


static void tune_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
          unsigned int id, uint16_t offset,
          const uint8_t *value, size_t len,
          uint8_t opcode, struct bt_att *att,
          void *user_data)
{
  struct server_s_t *server = (server_s_t*)user_data;
  uint8_t ecode = 0;

  ESP_LOGI(TAG, "DBG: tune_msrmt_ccc_write_cb %lu\n", len);
  print_hex(value,len);

  if (!value || len != 2) {
    ecode = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
    goto done;
  }

  if (offset) {
    ecode = BT_ATT_ERROR_INVALID_OFFSET;
    goto done;
  }

  if (value[0] == 0x00)
    server->tune_msrmt_enabled = false;
  else if (value[0] == 0x01) {
    if (server->tune_msrmt_enabled) {
      ESP_LOGI(TAG, "tune Measurement Already Enabled\n");
      goto done;
    }

    server->tune_msrmt_enabled = true;
  } else
    ecode = 0x80;

  ESP_LOGI(TAG, "TUNE: Measurement Enabled: %s\n",
        server->hr_msrmt_enabled ? "true" : "false");

  update_tune_msrmt_simulation(server);

done:
  // 2019/01/13 Disable for porting
  // gatt_db_attribute_write_result(attrib, id, ecode);
  ;
}


// Initialize the 123 tune server structure with valid values
void TuneServer_init(void) {
 int graph_index;
 server_s_t* server=&server_global;

  decimal2TuneVoltage(13.0, &server->tune_Voltage[0],&server->tune_Voltage[1]);
  decimal2TuneTemperature(70 ,&server->tune_Temperature[0],&server->tune_Temperature[1]);
  decimal2TunePressure(50, &server->tune_Pressure[0],&server->tune_Pressure[1]);

  // set initial internal pincode
  char2TunePinCode('7', &server->tune_Pincode[0][0],&server->tune_Pincode[0][1]);
  char2TunePinCode('8', &server->tune_Pincode[1][0],&server->tune_Pincode[1][1]);
  char2TunePinCode('3', &server->tune_Pincode[2][0],&server->tune_Pincode[2][1]);
  char2TunePinCode('2', &server->tune_Pincode[3][0],&server->tune_Pincode[3][1]);
  char2TunePinCode('\0', &server->tune_Pincode[4][0],&server->tune_Pincode[4][1]);

  printf("Set 123Tune initial pincode to 7832\n");

  printf("AdvanceCurve RPM/Advance set to:\n");

  // set initial internal Advance Curve (RPM/Advance), max 10 entries!!
  graph_index=0;
  decimal2TuneRPM(500, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);  // 500 is fixed
  decimal2TuneAdvance(0, &server->tune_AdvanceCurveDegrees[graph_index][0],&server->tune_AdvanceCurveDegrees[graph_index][1]);
  print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
  print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TuneRPM(700, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);
  decimal2TuneAdvance(0, &server->tune_AdvanceCurveDegrees[graph_index][0],&server->tune_AdvanceCurveDegrees[graph_index][1]);
  print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
  print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TuneRPM(800, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);
  decimal2TuneAdvance(0, &server->tune_AdvanceCurveDegrees[graph_index][0],&server->tune_AdvanceCurveDegrees[graph_index][1]);
  print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
  print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TuneRPM(1000, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);
  decimal2TuneAdvance(4.0, &server->tune_AdvanceCurveDegrees[graph_index][0],&server->tune_AdvanceCurveDegrees[graph_index][1]);
  print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
  print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TuneRPM(1500, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);
  decimal2TuneAdvance(12.0, &server->tune_AdvanceCurveDegrees[graph_index][0],&server->tune_AdvanceCurveDegrees[graph_index][1]);
  print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
  print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TuneRPM(1800, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);
  decimal2TuneAdvance(17.0, &server->tune_AdvanceCurveDegrees[graph_index][0],&server->tune_AdvanceCurveDegrees[graph_index][1]);
  print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
  print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TuneRPM(3000, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);
  decimal2TuneAdvance(30.0, &server->tune_AdvanceCurveDegrees[graph_index][0],&server->tune_AdvanceCurveDegrees[graph_index][1]);
  print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
  print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TuneRPM(4500, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);
  decimal2TuneAdvance(33.0, &server->tune_AdvanceCurveDegrees[graph_index][0],&server->tune_AdvanceCurveDegrees[graph_index][1]);
  print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
  print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TuneRPM(8000, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]); // 8000 is fixed
  decimal2TuneAdvance(33.0, &server->tune_AdvanceCurveDegrees[graph_index][0],&server->tune_AdvanceCurveDegrees[graph_index][1]);
  print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
  print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);

  graph_index++;
    // re-use grapth_index variabele (as it has exactly the right value to start with) to finish aand fill the rest with default valuses
    for(;graph_index < C_nr_123AdvanceCurve_Elements;graph_index++) {
      // "0xFF" = 0x46,0x46
      server->tune_AdvanceCurveRPM[graph_index][0] = 0x46;  // "0xG " == N/A
      server->tune_AdvanceCurveRPM[graph_index][1] = 0x46; // "0x G" == N/A
      server->tune_AdvanceCurveDegrees[graph_index][0] = 0x46;  // "0xG " == N/A
      server->tune_AdvanceCurveDegrees[graph_index][1] = 0x46; // "0x G" == N/A

      print_hex(server->tune_AdvanceCurveRPM[graph_index],2);
      print_hex(server->tune_AdvanceCurveDegrees[graph_index],2);
    }


  printf("Predefined MAP table:\n");
  // set initial internal MAP Curve (Vacuum/Advance), max 10 entries!!
  graph_index=0;
  decimal2TunePressure(0, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]);   // 0 is fixed
  decimal2TuneAdvance(11.0, &server->tune_MapCurveDegrees[graph_index][0],&server->tune_MapCurveDegrees[graph_index][1]);
  print_hex(server->tune_MapCurvePressure[graph_index],2);
  print_hex(server->tune_MapCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TunePressure(29, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]);
  decimal2TuneAdvance(11.0, &server->tune_MapCurveDegrees[graph_index][0],&server->tune_MapCurveDegrees[graph_index][1]);
  print_hex(server->tune_MapCurvePressure[graph_index],2);
  print_hex(server->tune_MapCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TunePressure(30, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]);
  decimal2TuneAdvance(11.0, &server->tune_MapCurveDegrees[graph_index][0],&server->tune_MapCurveDegrees[graph_index][1]);
  print_hex(server->tune_MapCurvePressure[graph_index],2);
  print_hex(server->tune_MapCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TunePressure(40, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]);
  decimal2TuneAdvance(11.0, &server->tune_MapCurveDegrees[graph_index][0],&server->tune_MapCurveDegrees[graph_index][1]);
  print_hex(server->tune_MapCurvePressure[graph_index],2);
  print_hex(server->tune_MapCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TunePressure(88, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]);
  decimal2TuneAdvance(0, &server->tune_MapCurveDegrees[graph_index][0],&server->tune_MapCurveDegrees[graph_index][1]);
  print_hex(server->tune_MapCurvePressure[graph_index],2);
  print_hex(server->tune_MapCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TunePressure(100, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]); // 100 is fixed
  decimal2TuneAdvance(0, &server->tune_MapCurveDegrees[graph_index][0],&server->tune_MapCurveDegrees[graph_index][1]);      // 0 seems fixed as well
  print_hex(server->tune_MapCurvePressure[graph_index],2);
  print_hex(server->tune_MapCurveDegrees[graph_index],2);

  graph_index++;
  decimal2TunePressure(200, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]); // 200 is fixed/last item
  decimal2TuneAdvance(0, &server->tune_MapCurveDegrees[graph_index][0],&server->tune_MapCurveDegrees[graph_index][1]);      // degrees is variabel, but negative only
  print_hex(server->tune_MapCurvePressure[graph_index],2);
  print_hex(server->tune_MapCurveDegrees[graph_index],2);

  graph_index++;
    // re-use grapth_index variabele (as it has exactly the right value to start with) to finish aand fill the rest with default valuses
    for(;graph_index < C_nr_123MapCurve_Elements;graph_index++) {
      // "0xFF" = 0x46,0x46
      server->tune_MapCurvePressure[graph_index][0] = 0x46;  // "0xG " == N/A
      server->tune_MapCurvePressure[graph_index][1] = 0x46; // "0x G" == N/A
      server->tune_MapCurveDegrees[graph_index][0] = 0x46;  // "0xG " == N/A
      server->tune_MapCurveDegrees[graph_index][1] = 0x46; // "0x G" == N/A

      print_hex(server->tune_MapCurvePressure[graph_index],2);
      print_hex(server->tune_MapCurveDegrees[graph_index],2);
    }


  // set initial internal MAP Curve start RPM (below MAP not active)
  decimal2TuneRPM(1200, &server->tune_MapCurveStartRPM[0],&server->tune_MapCurveStartRPM[1]);

  // Set initial/internal RPM Limit
  decimal2TuneRPM(4650, &server->tune_RPMLimit[0],&server->tune_RPMLimit[1]);


  //
  server->tune_TuningMode_enabled=false;
  server->tune_TuningMode_Advance=0;
  server->tune_IMMOBILIZED=false;


  // ToDO
  // Device 61504

  // End set internal Tune values



}
