/*
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2014  Google Inc.
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>
#include <getopt.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>

#include "lib/bluetooth.h"
#include "lib/hci.h"
#include "lib/hci_lib.h"
#include "lib/l2cap.h"
#include "lib/uuid.h"

#include "src/shared/mainloop.h"
#include "src/shared/mainloop-glib.c"
#include "src/shared/util.h"
#include "src/shared/att.h"
#include "src/shared/queue.h"
#include "src/shared/timeout.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-server.h"

#define UUID_GAP			0x1800
#define UUID_GATT			0x1801
#define UUID_HEART_RATE			0x180d
#define UUID_HEART_RATE_MSRMT		0x2a37
#define UUID_HEART_RATE_BODY		0x2a38
#define UUID_HEART_RATE_CTRL		0x2a39
#define UUID_123TUNE_UNKNOWN_STR 	"0xDA2B84F1627948DEBDC0AFBEA0226079"

#define C_nr_123MapCurve_Elements 10    // # of elements/size of arrays
#define C_nr_123AdvanceCurve_Elements 10    // # of elements/size of arrays

#define ATT_CID 4

#define PRLOG(...) \
	do { \
		printf(__VA_ARGS__); \
		print_prompt(); \
	} while (0)

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define COLOR_OFF	"\x1B[0m"
#define COLOR_RED	"\x1B[0;91m"
#define COLOR_GREEN	"\x1B[0;92m"
#define COLOR_YELLOW	"\x1B[0;93m"
#define COLOR_BLUE	"\x1B[0;94m"
#define COLOR_MAGENTA	"\x1B[0;95m"
#define COLOR_BOLDGRAY	"\x1B[1;30m"
#define COLOR_BOLDWHITE	"\x1B[1;37m"

static const char test_device_name[] = "123\\TUNE+";
static bool verbose = false;

struct server {
	int fd;
	struct bt_att *att;
	struct gatt_db *db;
	struct bt_gatt_server *gatt;

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

	bool tune_TuningMode_enabled;	// In Tuning mode, or not
	int tune_TuningMode_Advance;	// degrees Advance shift in Tuning mode, positive or negative, max value ???
	bool tune_IMMOBILIZED;		// immobilized? or not

	// Internal values only, desired state is not same as actual state, perhaps statore desired state as well??
	uint8_t tune_RPM[2];		// Actual RPM
	uint8_t tune_Advance[2];	// Actual Advance based on curve and Tuning
	uint8_t tune_Pressure[2];	// Actual measured pressure
	uint8_t tune_Temperature[2];	// Actual Temperature
	uint8_t tune_Voltage[2];	// Actual Voltage
	uint8_t tune_Ampere[2];		// Actual Ampere
	uint8_t tune_UNDISCOVERED[2];	// Unknown until now, let's see later


	// 2 arrays make one graphs, arrays are bluetooth command organized.
	uint8_t tune_AdvanceCurveRPM[10][2];	 	// Pos No  1 ... 10  , Value[2]
	uint8_t tune_AdvanceCurveDegrees[10][2];	// Pos No  1 ... 10  , Value[2]
	uint8_t tune_RPMLimit[2];			// Above this...

	// MAP curve
	uint8_t tune_MapCurvePressure[10][2];		// Pos No  1 ... 10  , Value[2]
	uint8_t tune_MapCurveDegrees[10][2];		// Pos No  1 ... 10  , Value[2]
	uint8_t tune_MapCurveStartRPM[2];		// below this value no MapCurve Active.

	uint8_t tune_Pincode[10][2];			// Strings stored in transport format, as is the other variables

	int hr_ee_count;
	int tune_ee_count;
	unsigned int hr_timeout_id;
	unsigned int tune_timeout_id;
};


// declaration
void print_hex(const uint8_t* p, size_t len);
uint32_t TuneRPM2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneRPM(uint32_t RPM, uint8_t *MSB, uint8_t *LSB);	// inverse from TuneRPM2decimal

double TuneAdvance2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneAdvance(double Advance, uint8_t *MSB, uint8_t *LSB);	// inverse TuneAdvance2decimal


uint32_t TuneGraphNo2decimal(uint8_t MSB, uint8_t LSB);

uint32_t TunePressure2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TunePressure(uint32_t Pressure, uint8_t *MSB, uint8_t *LSB); 	// Inverse TunePressure2decimal

int32_t  TuneTemperature2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneTemperature(double Temperature, uint8_t *MSB, uint8_t *LSB); 	// inverse

double   TuneAmpere2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneAmpere(double Ampere, uint8_t *MSB, uint8_t *LSB);  	//inverse

double TuneVoltage2decimal(uint8_t MSB, uint8_t LSB);
uint32_t decimal2TuneVoltage(double Voltage, uint8_t *MSB, uint8_t *LSB); 	// inverse


char TunePinCode2char(uint8_t MSB, uint8_t LSB);
uint32_t char2TunePinCode(char PinChar, uint8_t *MSB, uint8_t *LSB);		// inverse


// Calcualte Advance based on current RPM and RPM/Advance Graph
double CalculateAdvanceByRPM(void *user_data);

// Calcualte Advance based on current Pressure and Pressure(Vacuum)/Advance Graph
double CalculateAdvanceByPressure(void *user_data);


// update Actual calculated Advance
void UpdateRealtimeTuneAdvance(void *user_data);



// calculated and write byte number 3
uint32_t TuneSetnewChecksum(const uint8_t* str,uint8_t *csum);
#define ROUND_2_INT(f) ((int)(f >= 0.0 ? (f + 0.5) : (f - 0.5)))	// Avoid chopping when float to int, use rounding when a bit matters..



// definition
void print_hex(const uint8_t* p, size_t len)
{
size_t i;

printf("len=%02lu: ",len);
for (i=0; i< len;i++)
	{
	printf("%02X ", p[i]);
	}

printf("\n");
}


static void print_prompt(void)
{
	printf(COLOR_BLUE "[GATT server]" COLOR_OFF "# ");
	fflush(stdout);
}

static void att_disconnect_cb(int err, void *user_data)
{
	printf("Device disconnected: %s\n", strerror(err));

	mainloop_quit();
}

static void att_debug_cb(const char *str, void *user_data)
{
	const char *prefix = user_data;

	PRLOG(COLOR_BOLDGRAY "%s" COLOR_BOLDWHITE "%s\n" COLOR_OFF, prefix,
									str);
}

static void gatt_debug_cb(const char *str, void *user_data)
{
	const char *prefix = user_data;

	PRLOG(COLOR_GREEN "%s%s\n" COLOR_OFF, prefix, str);
}

static void gap_device_name_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t error = 0;
	size_t len = 0;
	const uint8_t *value = NULL;

	PRLOG("GAP Device Name Read called\n");

	len = server->name_len;

	if (offset > len) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	len -= offset;
	value = len ? &server->device_name[offset] : NULL;

done:
	gatt_db_attribute_read_result(attrib, id, error, value, len);
}

static void gap_device_name_write_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					const uint8_t *value, size_t len,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t error = 0;

	PRLOG("GAP Device Name Write called\n");

	/* If the value is being completely truncated, clean up and return */
	if (!(offset + len)) {
		free(server->device_name);
		server->device_name = NULL;
		server->name_len = 0;
		goto done;
	}

	/* Implement this as a variable length attribute value. */
	if (offset > server->name_len) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	if (offset + len != server->name_len) {
		uint8_t *name;

		name = realloc(server->device_name, offset + len);
		if (!name) {
			error = BT_ATT_ERROR_INSUFFICIENT_RESOURCES;
			goto done;
		}

		server->device_name = name;
		server->name_len = offset + len;
	}

	if (value)
		memcpy(server->device_name + offset, value, len);

done:
	gatt_db_attribute_write_result(attrib, id, error);
}

static void gap_device_name_ext_prop_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	uint8_t value[2];

	PRLOG("Device Name Extended Properties Read called\n");

	value[0] = BT_GATT_CHRC_EXT_PROP_RELIABLE_WRITE;
	value[1] = 0;

	gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

static void gatt_service_changed_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	PRLOG("Service Changed Read called\n");

	gatt_db_attribute_read_result(attrib, id, 0, NULL, 0);
}

static void gatt_svc_chngd_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t value[2];

	PRLOG("Service Changed CCC Read called\n");

	value[0] = server->svc_chngd_enabled ? 0x02 : 0x00;
	value[1] = 0x00;

	gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

static void gatt_svc_chngd_ccc_write_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					const uint8_t *value, size_t len,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t ecode = 0;

	PRLOG("Service Changed CCC Write called\n");

	if (!value || len != 2) {
		ecode = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		ecode = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	if (value[0] == 0x00)
		server->svc_chngd_enabled = false;
	else if (value[0] == 0x02)
		server->svc_chngd_enabled = true;
	else
		ecode = 0x80;

	PRLOG("Service Changed Enabled: %s\n",
				server->svc_chngd_enabled ? "true" : "false");

done:
	gatt_db_attribute_write_result(attrib, id, ecode);
}

static void hr_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t value[2];

	value[0] = server->hr_msrmt_enabled ? 0x01 : 0x00;
	value[1] = 0x00;

	PRLOG("DBG: hr_msrmt_ccc_read_cb\n");

	gatt_db_attribute_read_result(attrib, id, 0, value, 2);
}

static void tune_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t value[2];

	value[0] = server->tune_msrmt_enabled ? 0x01 : 0x00;
	value[1] = 0x00;

	PRLOG("DBG: tune_msrmt_ccc_read_cb\n");

	gatt_db_attribute_read_result(attrib, id, 0, value, 2);
}

static bool hr_msrmt_cb(void *user_data)
{
	struct server *server = user_data;
	bool expended_present = !(server->hr_ee_count % 10);
	uint16_t len = 2;
	uint8_t pdu[4];
	uint32_t cur_ee;

	pdu[0] = 0x06;
	pdu[1] = 90 + (rand() % 40);

	if (expended_present) {
		pdu[0] |= 0x08;
		put_le16(server->hr_energy_expended, pdu + 2);
		len += 2;
	}

	bt_gatt_server_send_notification(server->gatt,
						server->hr_msrmt_handle,
						pdu, len);


	cur_ee = server->hr_energy_expended;
	server->hr_energy_expended = MIN(UINT16_MAX, cur_ee + 10);
	server->hr_ee_count++;

	return true;
}


static bool tune_msrmt_cb(void *user_data)
{
	struct server *server = user_data;
	// bool expended_present = !(server->tune_ee_count % 10);
	uint16_t len = 1;
	uint8_t pdu[100];
	uint32_t cur_ee;
	static int iteration=-20;


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
			//if (iteration==0) iteration+=20;	// jump to default
			return true;
			break;
/*		case 1:
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
			//uint8_t str[] = { 0x30, 0x42, 0x45, 0x67, 0x20 };	// 8000 RPM (eerste waarde die word doorgestuurd (is om max schaal aan te geven wellicht?
			uint8_t str[] = { 0x30, 0x30, 0x41, 0x51, 0x20 };	// eerste waarde die word doorgestuurd (is om m

			//decimal2TuneRPM(1000,&str[1],&str[2]);			// set RPM by decimal value, TODO checksum??
			//TuneSetnewChecksum(str,&str[3]);			// set str[3]

			printf("\n\nSIMULATE: %s Special Start RPM set to: %d RPM\n",__FUNCTION__,TuneRPM2decimal(str[1],str[2]));     // test reverse function  (and forward)

			// Prooved Values (initial
			//	uint8_t str[] = { 0x30, 0x42, 0x45, 0x67, 0x20 };	// 8000 RPM (eerste waarde die word doorgestuurd (is om max schaal aan te geven wellicht?

			len=sizeof(str);
			memcpy(pdu,str,len);
			print_hex(pdu,len);
			bt_gatt_server_send_notification(server->gatt,
						server->tune_msrmt_handle,
						pdu, len);
			}


			// set new Temperature in server internal
			decimal2TuneTemperature(60 ,&server->tune_Temperature[0],&server->tune_Temperature[1]);

			// set new Voltage in server Internal
			decimal2TuneVoltage(14.2 ,&server->tune_Voltage[0],&server->tune_Voltage[1]);

			// set new pressure in server Internal
			decimal2TunePressure(20 ,&server->tune_Pressure[0],&server->tune_Pressure[1]);


			// set new Advance in server Internal <static superceded by dynamic graph based code>
			//decimal2TuneAdvance(10.0 ,&server->tune_Advance[0],&server->tune_Advance[1]);

			// set Temperature in server internal
			decimal2TuneAmpere(2.8 ,&server->tune_Ampere[0],&server->tune_Ampere[1]);

			// Set new RPM in internal system
			decimal2TuneRPM(1500,&server->tune_RPM[0],&server->tune_RPM[1]);

			break;







		default:
			{ // RPM
			//uint8_t str[] = { 0x30, 0x35, 0x30, 0x45, 0x20 };
			//uint8_t str[] = { 0x30, 0x30, 0x43, 0x53, 0x20 };
			//uint8_t str[] = { 0x30, 0x36, 0x45, 0x5b, 0x20 };		// 5500 RPM uit MAP
			uint8_t str[] = { 0x30, 0x00, 0x00, 0x00, 0x20 };	// MAX RPM 4500????? nee ~5000 RPM



			str[1]=server->tune_RPM[0];			// read actual values from internal state
			str[2]=server->tune_RPM[1];
			TuneSetnewChecksum(str,&str[3]);		// set str[3] checksum

			printf("\n\nSIMULATE: %s RPM was set to: %d RPM\n",__FUNCTION__,TuneRPM2decimal(str[1],str[2])  );     // test reverse function  (and forward)


			// Prooved Values
			//	uint8_t str[] = { 0x30, 0x30, 0x30, 0x40, 0x20 };	// 0 (meter slat niet uit, maar past perfect in rei, en opvolgens past ook

			//	uint8_t str[] = { 0x30, 0x30, 0x31, 0x41, 0x20 };	// 50 RPM
			//	uint8_t str[] = { 0x30, 0x30, 0x43, 0x53, 0x20 };	// 500 RPM
			//	uint8_t str[] = { 0x30, 0x30, 0x41, 0x51, 0x20 };	// 500+ RPM
			//	uint8_t str[] = { 0x30, 0x30, 0x43, 0x53, 0x20 };	// 600 RPM
			//	uint8_t str[] = { 0x30, 0x30, 0x45, 0x55, 0x20 };	// 700 RPM
			//	uint8_t str[] = { 0x30, 0x30, 0x46, 0x56, 0x20 };	// 800 RPM


			//	uint8_t str[] = { 0x30, 0x31, 0x30, 0x41, 0x20 };	// 800 RPM
			//	uint8_t str[] = { 0x30, 0x31, 0x31, 0x42, 0x20 };	// 900+ RPM
			//	uint8_t str[] = { 0x30, 0x31, 0x33, 0x44, 0x20 };	// 1000-RPM
			//	uint8_t str[] = { 0x30, 0x31, 0x34, 0x45, 0x20 };	// 1000 RPM
			//	uint8_t str[] = { 0x30, 0x31, 0x35, 0x46, 0x20 };	// 1000+ RPM
			//	uint8_t str[] = { 0x30, 0x31, 0x38, 0x49, 0x20 };	// 1200
			// 	uit capture nog waarde paren als 42,53  44,55 45,56

			//	uint8_t str[] = { 0x30, 0x31, 0x46, 0x57, 0x20 };	// 1600- RPM

			//	uint8_t str[] = { 0x30, 0x32, 0x30, 0x42, 0x20 };	// 1600 RPM
			//	uint8_t str[] = { 0x30, 0x32, 0x32, 0x44, 0x20 };	// 1700 RPM
			//	uint8_t str[] = { 0x30, 0x32, 0x33, 0x45, 0x20 };	// 1800- RPM
			//	uint8_t str[] = { 0x30, 0x32, 0x34, 0x46, 0x20 };	// 1800 RPM
			//	uint8_t str[] = { 0x30, 0x32, 0x46, 0x58, 0x20 };	// 2400 RPM

			//	uint8_t str[] = { 0x30, 0x33, 0x30, 0x43, 0x20 };	// 2400 RPM
			//	uint8_t str[] = { 0x30, 0x33, 0x31, 0x44, 0x20 };	// 2400+ RPM
			//	uint8_t str[] = { 0x30, 0x33, 0x44, 0x57, 0x20 };	// 3000 RPM

			//	uint8_t str[] = { 0x30, 0x34, 0x30, 0x44, 0x20 };	// 3200 RPM
			//	uint8_t str[] = { 0x30, 0x34, 0x38, 0x4c, 0x20 };	// 3600- RPM

			// 	uint8_t str[] = { 0x30, 0x35, 0x30, 0x45, 0x20 };	// 4000 RPM
			//	uint8_t str[] = { 0x30, 0x35, 0x31, 0x46, 0x20 };	// 4000+ RPM
			//	uint8_t str[] = { 0x30, 0x35, 0x32, 0x47, 0x20 };	// 4050 RPM

			//	uint8_t str[] = { 0x30, 0x36, 0x30, 0x46, 0x20 };	// 4800 RPM

			//uint8_t str[] = { 0x30, 0x36, 0x45, 0x5b, 0x20 };		// 5500 RPM uit MAP
			//	uint8_t str[] = { 0x30, 0x37, 0x30, 0x47, 0x20 };	// 5600 RPM

			//	uint8_t str[] = { 0x30, 0x38, 0x30, 0x48, 0x20 };	// 6400 RPM
			//	uint8_t str[] = { 0x30, 0x39, 0x30, 0x49, 0x20 };	// 7200 RPM

			//	uint8_t str[] = { 0x30, 0x40, 0x30, 0x50, 0x20 };	// deze laat meter niet meer uitslaan...


			//	uint8_t str[] = { 0x30, 0x42, 0x45, 0x67, 0x20 };	// 8000 RPM (eerste waarde die word doorgestuurd (is om max schaal aan te geven wellicht?

			len=sizeof(str);
			memcpy(pdu,str,len);
			print_hex(pdu,len);
			bt_gatt_server_send_notification(server->gatt,
						server->tune_msrmt_handle,
						pdu, len);
			}


			{	// Advance
			static int i=0x00;
			static int j=0x40;
			//uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };
			//uint8_t str[] = { 0x31, 0x46, 0x30, 0x57, 0x20 };		// 48,50
			//uint8_t str[] = { 0x31, 0x40, 0x30, 0x51, 0x20 };       	//
			// uint8_t str[] = { 0x31, 0x30, 0x46, 0x57, 0x20 };		//  3 degrees
			//uint8_t str[] = { 0x31, 0x31, 0x30, 0x42, 0x20 };       	// 3,25 decgrees
			//uint8_t str[] = { 0x31, 0x45, 0x31, 0x57, 0x20 };    // 45, 31 == 45 from map command
			uint8_t str[] = { 0x31, 0x00, 0x00, 0x00, 0x20 };    // 11 degrees from map/vacuum


			// JIT update the actual (calcualated) Advance values, before use
			UpdateRealtimeTuneAdvance(server);

			str[1]=server->tune_Advance[0];			// read actual values from internal state
			str[2]=server->tune_Advance[1];
			TuneSetnewChecksum(str,&str[3]);		// set str[3]

			printf("SIMULATE: %s Advance was set to: %2.2f degrees\n",__FUNCTION__,TuneAdvance2decimal(str[1],str[2]));     // test reverse function  (and forward)


			// prooved collection
			// uint8_t str[] = { 0x31, 0x30, 0x30, 0x41, 0x20 };       	//  0 from captrue
			// uint8_t str[] = { 0x31, 0x30, 0x46, 0x57, 0x20 };		//  3 degrees

			// uint8_t str[] = { 0x31, 0x31, 0x30, 0x42, 0x20 };       	// 3,25 decgrees
			// uint8_t str[] = { 0x31, 0x32, 0x30, 0x43, 0x20 };       	// 6,5 decgrees
			// uint8_t str[] = { 0x31, 0x33, 0x30, 0x44, 0x20 };       	// 10- decgrees
			// uint8_t str[] = { 0x31, 0x34, 0x30, 0x45, 0x20 };       	// 12,5 decgrees
			// uint8_t str[] = { 0x31, 0x35, 0x30, 0x46, 0x20 };       	// 16 decgrees
			// uint8_t str[] = { 0x31, 0x36, 0x30, 0x47, 0x20 };       	// 19 decgrees
			// uint8_t str[] = { 0x31, 0x37, 0x30, 0x48, 0x20 };       	// 22 decgrees
			// uint8_t str[] = { 0x31, 0x38, 0x30, 0x49, 0x20 };       	// 25+ decgrees
			// uint8_t str[] = { 0x31, 0x39, 0x30, 0x4a, 0x20 };       	// 28,5 decgrees
			// uint8_t str[] = { 0x31, 0x3a, 0x30, 0x4b, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3b, 0x30, 0x4c, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3c, 0x30, 0x4d, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3d, 0x30, 0x4e, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3e, 0x30, 0x4f, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3f, 0x30, 0x50, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x40, 0x30, 0x51, 0x20 };       	// NA!!!!

			// uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };		// 31,8 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x31, 0x53, 0x20 };		// 32.0 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x32, 0x54, 0x20 };		// 32,2 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x33, 0x55, 0x20 };		// 32,4 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x34, 0x56, 0x20 };		// 32,6 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x35, 0x57, 0x20 };		// 32,8 degrees Advance

			//
			// uint8_t str[] = { 0x31, 0x41, 0x46, 0x68, 0x20 };		// 35.0 degrees Advance
			// 69 niets...
			// uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };       // 32--
			// uint8_t str[] = { 0x31, 0x42, 0x30, 0x53, 0x20 };	// 35.0
			// uint8_t str[] = { 0x31, 0x43, 0x30, 0x54, 0x20 };	// 38,50
			// uint8_t str[] = { 0x31, 0x44, 0x30, 0x55, 0x20 };	// 41,5
			// uint8_t str[] = { 0x31, 0x45, 0x30, 0x56, 0x20 };	// 45
			// uint8_t str[] = { 0x31, 0x46, 0x30, 0x57, 0x20 };	// 48,50



			len=sizeof(str);
			//for (i=0;i<256;i++)
				{
				//str[2]=i;
				//str[3]=j;
				printf("SIMULATE: uint8_t[2][3]=%02X  %02X------------------------------\n",i,j);

				memcpy(pdu,str,len);
				print_hex(pdu,len);

				bt_gatt_server_send_notification(server->gatt,
						server->tune_msrmt_handle,
						pdu, len);
				}
			//j++;
			//i++;
			}

			{ // BAR
			//static int i=0x0,j;

			// uint8_t str[] = { 0x32, 0x36,0x34, 0x4c, 0x20 };  //  0 BAR
			//uint8_t str[] = { 0x32, 0x37, 0x30, 0x49, 0x20 };	//  0.1 BAR
			//uint8_t str[] = { 0x32, 0x38, 0x30, 0x4a, 0x20 };	//  0.295(-) BAR
			//uint8_t str[] = { 0x32, 0x39, 0x30, 0x4b, 0x20 };	//  0.45 BAR
			//uint8_t str[] = { 0x32, 0x3a, 0x30, 0x4c, 0x20 };	//  0,45 BAR
			//uint8_t str[] = { 0x32, 0x3b, 0x30, 0x4d, 0x20 };	// geen response
			//uint8_t str[] = { 0x32, 0x3c, 0x30, 0x4e, 0x20 };	//  geen response
			//uint8_t str[] = { 0x32, 0x35, 0x35, 0x4c, 0x20 };	//  - 0.04 BAR ???
			//uint8_t str[] = { 0x32, 0x35, 0x39, 0x50, 0x20 };	//  -0.11
			//uint8_t str[] = { 0x32, 0x35, 0x3a, 0x51, 0x20 };	//  opvolger van 0x35, 0x39 , maar doet het niet meer....
			//uint8_t str[] = { 0x32, 0x35, 0x45, 0x5c, 0x20 };	//  uit MAP edit... 94kP (-0.4BAR) == 1 strepje, 0.06, maar meter is blijkbaar niet erg naukeurig af te lezen
			//uint8_t str[] = { 0x32, 0x36, 0x30, 0x48, 0x20 };	//  - 0,04 BAR of zo (minder dan 1 streekpje, streepje is 0.6BAR
			uint8_t str[] = { 0x32, 0x00, 0x00, 0x00, 0x20 };	//  empty pressure Frame


			//decimal2TunePressure(100 ,&str[1],&str[2]);		// set pressure in kP, 100= 0BAR, 0 = -1 BAR

			//decimal2TunePressure(100 ,&server->tune_Pressure[0],&server->tune_Pressure[1]);		// set Pressure in server internal



			str[1]=server->tune_Pressure[0];			// read actual values from internal state
			str[2]=server->tune_Pressure[1];

			TuneSetnewChecksum(str,&str[3]);			// set str[3]

			printf("SIMULATE: %s presure was set to: %d kP\n",__FUNCTION__,TunePressure2decimal(str[1],str[2]));     // test reverse function  (and forward)

			// proved collection
			//uint8_t str[] = { 0x32, 0x30, 0x30, 0x42, 0x20 };	// -1 BAR
			// uint8_t str[] = { 0x32, 0x31, 0x30, 0x43, 0x20 };	// -0,8 BAR
			// uint8_t str[] = { 0x32, 0x32, 0x30, 0x44, 0x20 };	// -0,6 BAR
			// uint8_t str[] = { 0x32, 0x33, 0x30, 0x45, 0x20 };	// -0,45 BAR
			// uint8_t str[] = { 0x32, 0x34, 0x30, 0x46, 0x20 };	// -0,3 BAR
			// uint8_t str[] = { 0x32, 0x35, 0x30, 0x47, 0x20 };	// -0,2 BAR
			// uint8_t str[] = { 0x32, 0x35, 0x39, 0x50, 0x20 };	//  -0.11	// hoogste waarde, opvolger 0x3a werkt niet meer.
			//uint8_t str[] = { 0x32, 0x35, 0x3a, 0x51, 0x20 };	//  opvolger van 0x35, 0x39 , maar doet het niet meer....
			// uint8_t str[] = { 0x32, 0x35, 0x45, 0x5c, 0x20 };	//  uit MAP edit... 94kP (-0.4BAR) == 1 strepje, 0.06, maar meter is blijkbaar niet erg naukeurig af te lezen



			// prooved collection
			// uint8_t str[] = { 0x31, 0x30, 0x30, 0x41, 0x20 };       	//  0 from captrue
			// uint8_t str[] = { 0x31, 0x30, 0x46, 0x57, 0x20 };		//  3 degrees

			// uint8_t str[] = { 0x31, 0x31, 0x30, 0x42, 0x20 };       	// 3,25 decgrees
			// uint8_t str[] = { 0x31, 0x32, 0x30, 0x43, 0x20 };       	// 6,5 decgrees
			// uint8_t str[] = { 0x31, 0x33, 0x30, 0x44, 0x20 };       	// 10- decgrees
			// uint8_t str[] = { 0x31, 0x34, 0x30, 0x45, 0x20 };       	// 12,5 decgrees
			// uint8_t str[] = { 0x31, 0x35, 0x30, 0x46, 0x20 };       	// 16 decgrees
			// uint8_t str[] = { 0x31, 0x36, 0x30, 0x47, 0x20 };       	// 19 decgrees
			// uint8_t str[] = { 0x31, 0x37, 0x30, 0x48, 0x20 };       	// 22 decgrees
			// uint8_t str[] = { 0x31, 0x38, 0x30, 0x49, 0x20 };       	// 25+ decgrees
			// uint8_t str[] = { 0x31, 0x39, 0x30, 0x4a, 0x20 };       	// 28,5 decgrees
			// uint8_t str[] = { 0x31, 0x3a, 0x30, 0x4b, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3b, 0x30, 0x4c, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3c, 0x30, 0x4d, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3d, 0x30, 0x4e, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3e, 0x30, 0x4f, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x3f, 0x30, 0x50, 0x20 };       	// NA!!!!
			// uint8_t str[] = { 0x31, 0x40, 0x30, 0x51, 0x20 };       	// NA!!!!

			// uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };		// 31,8 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x31, 0x53, 0x20 };		// 32.0 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x32, 0x54, 0x20 };		// 32,2 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x33, 0x55, 0x20 };		// 32,4 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x34, 0x56, 0x20 };		// 32,6 degrees Advance
			// uint8_t str[] = { 0x31, 0x41, 0x35, 0x57, 0x20 };		// 32,8 degrees Advance

			//
			// uint8_t str[] = { 0x31, 0x41, 0x46, 0x68, 0x20 };		// 35.0 degrees Advance
			// 69 niets...
			// uint8_t str[] = { 0x31, 0x41, 0x30, 0x52, 0x20 };       // 32--
			// uint8_t str[] = { 0x31, 0x42, 0x30, 0x53, 0x20 };	// 35.0
			// uint8_t str[] = { 0x31, 0x43, 0x30, 0x54, 0x20 };	// 38,50
			// uint8_t str[] = { 0x31, 0x44, 0x30, 0x55, 0x20 };	// 41,5
			// uint8_t str[] = { 0x31, 0x45, 0x30, 0x56, 0x20 };	// 45
			// uint8_t str[] = { 0x31, 0x46, 0x30, 0x57, 0x20 };	// 48,50



			len=sizeof(str);
			//for (i=0;i<256;i++)
				{
				//str[2]=i;
				//str[3]=j;
				//printf("uint8_t[2][3]=%02X  %02X------------------------------\n",i,j);

				memcpy(pdu,str,len);
				print_hex(pdu,len);

				bt_gatt_server_send_notification(server->gatt,
						server->tune_msrmt_handle,
						pdu, len);
				}
			//j++;
			//i++;
			}



			{ // 0x33 Temperature
			// uint8_t str[] = { 0x33, 0x30, 0x30, 0x43, 0x20 };	// -20 (tegen aanslag)
			// uint8_t str[] = { 0x33, 0x31, 0x30, 0x44, 0x20 };	// -17 graden
			// uint8_t str[] = { 0x33, 0x32, 0x30, 0x45, 0x20 };	// 1 graad boven 0
			// uint8_t str[] = { 0x33, 0x33, 0x30, 0x46, 0x20 };	// from capture, 19 graden
			// uint8_t str[] = { 0x33, 0x34, 0x30, 0x47, 0x20 };	// 37 graden
			// uint8_t str[] = { 0x33, 0x35, 0x30, 0x48, 0x20 };	// 50  graden
			// uint8_t str[] = { 0x33, 0x36, 0x30, 0x49, 0x20 };	// 66  graden
			//uint8_t str[] = { 0x33, 0x37, 0x30, 0x4a, 0x20 };	// 82  graden
			//uint8_t str[] = { 0x33, 0x38, 0x30, 0x4b, 0x20 };	// 99  graden
			uint8_t str[] = { 0x33, 0x00, 0x00, 0x00, 0x20 };		// Empty Temperature string


			//decimal2TuneTemperature(60 ,&str[1],&str[2]);		// celcius
			//decimal2TuneTemperature(60 ,&server->tune_Temperature[0],&server->tune_Temperature[1]);		// set Temperature in server internal


			str[1]=server->tune_Temperature[0];			// read actual values from internal state
			str[2]=server->tune_Temperature[1];

			TuneSetnewChecksum(str,&str[3]);			// set str[3]

			printf("SIMULATE: %s Temperature was set to: %d Celcius\n",__FUNCTION__,TuneTemperature2decimal(str[1],str[2]));	// test reverse function  (and forward)

			// Prooved Values
			// uint8_t str[] = { 0x33, 0x30, 0x30, 0x43, 0x20 };	// -20 (tegen aanslag)
			// uint8_t str[] = { 0x33, 0x31, 0x30, 0x44, 0x20 };	// -17 graden
			// uint8_t str[] = { 0x33, 0x32, 0x30, 0x45, 0x20 };	// 1 graad boven 0
			// uint8_t str[] = { 0x33, 0x33, 0x30, 0x46, 0x20 };	// from capture, 19 graden
			// uint8_t str[] = { 0x33, 0x34, 0x30, 0x47, 0x20 };	// 37 graden
			// uint8_t str[] = { 0x33, 0x35, 0x30, 0x48, 0x20 };	// 50  graden
			// uint8_t str[] = { 0x33, 0x36, 0x30, 0x49, 0x20 };	// 66  graden
			// uint8_t str[] = { 0x33, 0x37, 0x30, 0x4a, 0x20 };	// 82  graden
			// uint8_t str[] = { 0x33, 0x38, 0x30, 0x4b, 0x20 };	// 99  graden

			len=sizeof(str);
			memcpy(pdu,str,len);
			print_hex(pdu,len);

			bt_gatt_server_send_notification(server->gatt,
						server->tune_msrmt_handle,
						pdu, len);
			}


			{ // 0x34 Tuning mode!!!!
			  // 34:31:30:45:20 [geen Tuning mode]
			  // 34:31:31:46:20 [IN Tuning mode] (na 0x74 toggle)
			// uint8_t str[] = { 0x34, 0x30, 0x30, 0x44, 0x20 };	//  niets
			uint8_t str[] = { 0x34, 0x31, 0x30, 0x45, 0x20 };	// uit capture/geen tuning
			// uint8_t str[] = { 0x34, 0x32, 0x30, 0x46, 0x20 };	// niets
			// uint8_t str[] = { 0x34, 0x33, 0x30, 0x47, 0x20 };	// niets
			//uint8_t str[] = { 0x34, 0x34, 0x30, 0x48, 0x20 };	// niets


			// Prooved Values
			//uint8_t str[] = { 0x34, 0x31, 0x30, 0x45, 0x20 };	// uit capture

			if (server->tune_TuningMode_enabled) {
				str[2]=0x31;	// Enabled Tuning message
				// str[3]=0x46;	// Enabled Tuning message   CSUM
			}
			TuneSetnewChecksum(str,&str[3]);
			printf("SIMULATE: %s Tuning mode was set to: '%c%c'\n",__FUNCTION__,str[1],str[2]);

			len=sizeof(str);
			memcpy(pdu,str,len);
			print_hex(pdu,len);

			bt_gatt_server_send_notification(server->gatt,
						server->tune_msrmt_handle,
						pdu, len);
			}



			{ // 0x35 Ampere
			//uint8_t str[] = { 0x35, 0x30, 0x30, 0x45, 0x20 };	// 0 Ampere
			//uint8_t str[] = { 0x35, 0x31, 0x30, 0x46, 0x20 };	// 1,85 Ampere
			//uint8_t str[] = { 0x35, 0x32, 0x30, 0x47, 0x20 };	// 2,75 Ampere
			//uint8_t str[] = { 0x35, 0x33, 0x30, 0x48, 0x20 };	// 4 Ampere (meter in hoek)
			//uint8_t str[] = { 0x35, 0x34, 0x30, 0x49, 0x20 };	// 4 Ampere (meter in hoek)

			//uint8_t str[] = { 0x35, 0x31, 0x30, 0x47, 0x20 };	// 1,85 Ampere
			//uint8_t str[] = { 0x35, 0x32, 0x30, 0x47, 0x20 };	// 2,75
			uint8_t str[] = { 0x35, 0x00, 0x00, 0x20, 0x20 };


			str[1]=server->tune_Ampere[0];			// read actual values from internal state
			str[2]=server->tune_Ampere[1];


			TuneSetnewChecksum(str,&str[3]);			// set str[3]

			printf("SIMULATE: %s Ampere was set to: %1.2f Ampere\n",__FUNCTION__,TuneAmpere2decimal(str[1],str[2]));     // test reverse function  (and forward)


			// Prooved Values
			//uint8_t str[] = { 0x35, 0x30, 0x30, 0x45, 0x20 };	// 0 Ampere
			//uint8_t str[] = { 0x35, 0x31, 0x30, 0x46, 0x20 };	// 1,85 Ampere
			//uint8_t str[] = { 0x35, 0x32, 0x30, 0x47, 0x20 };	// 2,75 Ampere
			//uint8_t str[] = { 0x35, 0x33, 0x30, 0x48, 0x20 };	// 4 Ampere (meter in hoek)
			//uint8_t str[] = { 0x35, 0x34, 0x30, 0x49, 0x20 };	// 4 Ampere (meter in hoek)

			len=sizeof(str);
			memcpy(pdu,str,len);
			print_hex(pdu,len);

			bt_gatt_server_send_notification(server->gatt,
						server->tune_msrmt_handle,
						pdu, len);
			}


			{ // 0x41 Volt

			//uint8_t str[] = { 0x41, 0x34, 0x30, 0x55, 0x20 };	// 14,1 V
			uint8_t str[] = { 0x41, 0x00, 0x00, 0x00, 0x20 };	// empty Temperature Frame

			// Prooved Values
			// uint8_t str[] = { 0x41, 0x33, 0x30, 0x54, 0x20 };	// 11 V (tegen aanslag)
			// uint8_t str[] = { 0x41, 0x33, 0x32, 0x56, 0x20 };	// 11.0V (net tegen aanslag)
			// uint8_t str[] = { 0x41, 0x33, 0x33, 0x57, 0x20 };	// 11,2V
			// uint8_t str[] = { 0x41, 0x33, 0x35, 0x59, 0x20 };	// 11,6V
			// uint8_t str[] = { 0x41, 0x33, 0x46, 0x55, 0x20 };	// 13,9 V
			// uint8_t str[] = { 0x41, 0x33, 0x46, 0x55, 0x20 };	// no response  (out hex scale)

			// uint8_t str[] = { 0x41, 0x34, 0x30, 0x55, 0x20 };	// 14,1 V
			// uint8_t str[] = { 0x41, 0x34, 0x31, 0x56, 0x20 };	// uit capture, 14,3 V

			//decimal2TuneVoltage(14.0 ,&str[1],&str[2]);			// set Voltage

			//uint8_t str[] = { 0x41, 0x34, 0x30, 0x55, 0x20 };	// 14,1V  -> 0x40 ~= 14,1
			//decimal2TuneVoltage(14.0 ,&server->tune_Voltage[0],&server->tune_Voltage[1]);		// set Temperature in server internal


			str[1]=server->tune_Voltage[0];			// read actual values from internal state
			str[2]=server->tune_Voltage[1];

			TuneSetnewChecksum(str,&str[3]);			// set str[3]


			printf("SIMULATE: %s Voltage set to: %1.2f Volt\n",__FUNCTION__,TuneVoltage2decimal(str[1],str[2]));     // test reverse function  (and forward)



			len=sizeof(str);
			memcpy(pdu,str,len);
			print_hex(pdu,len);

			bt_gatt_server_send_notification(server->gatt,
						server->tune_msrmt_handle,
						pdu, len);
			}



			{ // 0x42 ??? Tune mode iets ?????
			static int i=0;
			//uint8_t str[] = { 0x42, 0x34, 0x35, 0x5b, 0x20 };	// uit capture
			//uint8_t str[] = { 0x42, 0x35, 0x35, 0x5c, 0x20 };	// niets, maar foutief??
			uint8_t str[] = { 0x42, 0x34, 0x35, 0x5b, 0x0d };	// niets (maar wel correct


			//decimal2TuneUNDISCOVERED(14.0 ,&server->tune_UNDISCOVERED[0],&server->tune_UNDISCOVERED[1]);		// set UNDISCOVERED in server internal
			//misuse anotehr function to trough some random numbers;
			decimal2TunePressure(i++ ,&server->tune_UNDISCOVERED[0],&server->tune_UNDISCOVERED[1]);
			//server->tune_UNDISCOVERED[0]=str[1];	// Set internal values
			//server->tune_UNDISCOVERED[1]=str[2];
			//str[2]=0x41;



			str[1]=server->tune_UNDISCOVERED[0];			// read actual values from internal state
			str[2]=server->tune_UNDISCOVERED[1];
			TuneSetnewChecksum(str,&str[3]);			// set str[3]




			printf("SIMULATE: %s UNDISCOVERED %02X\n",__FUNCTION__,str[0]);

			// Prooved Values

			len=sizeof(str);
			memcpy(pdu,str,len);
			print_hex(pdu,len);

			bt_gatt_server_send_notification(server->gatt,
						server->tune_msrmt_handle,
						pdu, len);
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
	//					server->tune_msrmt_handle,
	//					pdu, len);


	cur_ee = server->tune_energy_expended;
	server->tune_energy_expended = MIN(UINT16_MAX, cur_ee + 10);
	server->tune_ee_count++;

	return true;
}


static void update_hr_msrmt_simulation(struct server *server)
{
	if (!server->hr_msrmt_enabled || !server->hr_visible) {
		timeout_remove(server->hr_timeout_id);
		return;
	}

	server->hr_timeout_id = timeout_add(1000, hr_msrmt_cb, server, NULL);
}

// simulate: notified RPM 2017/02/01 every seconds .. (from start)
static void update_tune_msrmt_simulation(struct server *server)
{
	if (!server->tune_msrmt_enabled || !server->tune_visible) {
		timeout_remove(server->tune_timeout_id);
		return;
	}

	server->tune_timeout_id = timeout_add(1000, tune_msrmt_cb, server, NULL);
}

static void hr_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					const uint8_t *value, size_t len,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t ecode = 0;

	PRLOG("DBG: hr_msrmt_ccc_write_cb %lu\n", len);
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
		server->hr_msrmt_enabled = false;
	else if (value[0] == 0x01) {
		if (server->hr_msrmt_enabled) {
			PRLOG("HR Measurement Already Enabled\n");
			goto done;
		}

		server->hr_msrmt_enabled = true;
	} else
		ecode = 0x80;

	PRLOG("HR: Measurement Enabled: %s\n",
				server->hr_msrmt_enabled ? "true" : "false");

	update_hr_msrmt_simulation(server);

done:
	gatt_db_attribute_write_result(attrib, id, ecode);
}

static void tune_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					const uint8_t *value, size_t len,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t ecode = 0;

	PRLOG("DBG: tune_msrmt_ccc_write_cb %lu\n", len);
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
			PRLOG("tune Measurement Already Enabled\n");
			goto done;
		}

		server->tune_msrmt_enabled = true;
	} else
		ecode = 0x80;

	PRLOG("TUNE: Measurement Enabled: %s\n",
				server->hr_msrmt_enabled ? "true" : "false");

	update_tune_msrmt_simulation(server);

done:
	gatt_db_attribute_write_result(attrib, id, ecode);
}

static void hr_control_point_write_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					const uint8_t *value, size_t len,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t ecode = 0;

	if (!value || len != 1) {
		ecode = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		ecode = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	if (value[0] == 1) {
		PRLOG("HR: Energy Expended value reset\n");
		server->hr_energy_expended = 0;
	}

done:
	gatt_db_attribute_write_result(attrib, id, ecode);
}

// Turns bluetooth binary into decimal RPM
uint32_t TuneRPM2decimal(uint8_t MSB, uint8_t LSB)
{
	uint32_t MSB_dec;
	uint32_t LSB_dec;
	uint32_t value;
	double MSB_weight=800/1;	// From readout digital advance curve Scaling MSB != scaling LSB
	double LSB_Weight=50/1;		// From roudaout advance curve, is not exact due to jump in readout table.
	char str[2];			// char string for hex number

	str[0]=MSB;			// fill string, to convert a single hex char
	str[1]='\0';
	MSB_dec=strtol(str,NULL,16);

	str[0]=LSB;                     // fill string, to convert a single hex char
	str[1]='\0';
	LSB_dec=strtol(str,NULL,16);

	value=MSB_dec*MSB_weight + LSB_dec*LSB_Weight;
return(value);
}


// Turns decimal bluetooth binary bluetooth binary
// returns values in value pointers MSB, LSB
uint32_t decimal2TuneRPM(uint32_t RPM, uint8_t *MSB, uint8_t *LSB)
{
	uint32_t MSB_dec;
	uint32_t LSB_dec;
	uint32_t MSB_weight=800/1;	// From readout digital advance curve Scaling MSB != scaling LSB
	uint32_t LSB_Weight=50/1;		// From roudaout advance curve, is not exact due to jump in readout table.
	char str_m[20];			// char string for hex number
	char str_l[20];			// char string for hex number

	MSB_dec=RPM/MSB_weight;		// div/800
	LSB_dec=(RPM%MSB_weight)/LSB_Weight;	// rest div/50 (no rounding required, accuracy is 50RPM, rounded to below.

	sprintf(str_m,"%X",MSB_dec);
	MSB[0]=str_m[0];


	sprintf(str_l,"%X",LSB_dec);
	LSB[0]=str_l[0];

	//printf("%s MSB_dec=%d LSB_dec=%d   MSB=%c LSB=%c str_m=%s str_l=%s\n",__FUNCTION__,  MSB_dec,LSB_dec,  MSB[0],LSB[0],  str_m,str_l);

return(0);
}



// converst hex value as string to integer (0-0xFF)
// Same logic as TunePinCode2char
uint32_t TunePressure2decimal(uint8_t MSB, uint8_t LSB)
{
	uint32_t value;
	char str[3];			// char string for hex number

	str[0]=MSB;			// fill string, to convert a single hex char
	str[1]=LSB;
	str[2]='\0';
	value=strtol(str,NULL,16);


return(value);
}

// in kP / absolute
// values returned in param 2 and 3
// same logic as char2TunePinCode()
uint32_t decimal2TunePressure(uint32_t Pressure, uint8_t *MSB, uint8_t *LSB)
{
	char str_m[20];			// char string for hex number


	sprintf(str_m,"%02X",Pressure);		// No rounding, exact
	MSB[0]=str_m[0];
	LSB[0]=str_m[1];

	//printf("%s pressure=%d kP   MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, Pressure,  MSB[0],LSB[0],  str_m);

return(0);
}

// converts a single Tune Char into Pincode Char ('0'-'9')
// Logic same as TunePressure2decimal(), except output is presented as char
char TunePinCode2char(uint8_t MSB, uint8_t LSB)
{
        uint32_t value;
        char str[3];                    // char string for hex number

        str[0]=MSB;                     // fill string, to convert a single hex char
        str[1]=LSB;
        str[2]='\0';
        value=strtol(str,NULL,16);

        if(!isdigit(value))  {
        	printf("%s Conversion Error, result is not a char: %02X\n",__FUNCTION__, value);
        	return(-1);
	}

return(value);
}

// Logic same as decimal2TunePressure(), except input is presented as char
uint32_t char2TunePinCode(char PinChar, uint8_t *MSB, uint8_t *LSB)
{
        char str_m[20];                 // char string for hex number


        sprintf(str_m,"%02X",PinChar);         // No rounding, exact
        MSB[0]=str_m[0];
        LSB[0]=str_m[1];

        //printf("%s pressure=%d kP   MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, Pressure,  MSB[0],LSB[0],  str_m

return(0);

}






// returns sigend int (-20 -100) in celcius
int32_t TuneTemperature2decimal(uint8_t MSB, uint8_t LSB)
{
	uint32_t value;
	char str[3];			// char string for hex number

	str[0]=MSB;			// fill string, to convert a single hex char
	str[1]=LSB;
	str[2]='\0';
	value=strtol(str,NULL,16);

	value-=30;			// emperisch found offset correction value 0 is; -25 on gauge 10 -> -20

return(value);
}


// -20 ... 100 degress celcius
// offset of 30, in celsius, and linear in celsius.
uint32_t decimal2TuneTemperature(double Temperature, uint8_t *MSB, uint8_t *LSB)
{
	char str_m[20];			// char string for hex number

	uint32_t T=ROUND_2_INT(Temperature+30);	// emperisch found offset correction value 0 is; -25 on gauge 10 -> -20. use rounding instead chopping (but more accuracy then 1 degree is not usefull)

	sprintf(str_m,"%02X",T);
	MSB[0]=str_m[0];
	LSB[0]=str_m[1];

	//printf("%s Temperature=%d    MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, T,  MSB[0],LSB[0],  str_m);

return(0);

}

double TuneAmpere2decimal(uint8_t MSB, uint8_t LSB)
{
	double value;
	double WeightFactor=16/1.85;    // op basis van afgelezen waardes
	char str[3];			// char string for hex number

	str[0]=MSB;			// fill string, to convert a single hex char
	str[1]=LSB;
	str[2]='\0';
	value=strtol(str,NULL,16);

	value/=WeightFactor;			// scaling factor, emperisch

return(value);
}


uint32_t decimal2TuneAmpere(double Ampere, uint8_t *MSB, uint8_t *LSB)
{
	char str_m[20];			// char string for hex number

	double WeightFactor=16/1.85;	// op basis van afgelezen waardes
	uint32_t A=ROUND_2_INT(Ampere*WeightFactor);		// round instead chop

	sprintf(str_m,"%02X",A);
	MSB[0]=str_m[0];
	LSB[0]=str_m[1];

	//printf("%s Ampere=%d A   MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, A,  MSB[0],LSB[0],  str_m);

return(0);
}


double TuneVoltage2decimal(uint8_t MSB, uint8_t LSB)
{
	double value;
	double WeightFactor=0x40/14.1;    // op basis van afgelezen waardes
	char str[3];			// char string for hex number

	str[0]=MSB;			// fill string, to convert a single hex char
	str[1]=LSB;
	str[2]='\0';
	value=strtol(str,NULL,16);

	value/=WeightFactor;			// scaling factor, emperisch

return(value);
}



// Volt graphs is very course, and not recommended for anything more percise then 0.5V!!!!
// This is due to non optimal value/scale is used, and one bit ~= 0.3V !!!!
uint32_t decimal2TuneVoltage(double Voltage, uint8_t *MSB, uint8_t *LSB)
{
	char str_m[20];			// char string for hex number

	//double WeightFactor=4*14.0/12.28;		// op basis van afgelezen waardes
	double WeightFactor=0x40/14.1;			// op basis van 0x24. 00 ~= 14,1V sample ((maar is aan lage kant afgerond door bit granulariteit
	uint32_t Volt=ROUND_2_INT(Voltage*WeightFactor);

	sprintf(str_m,"%02X",Volt);
	MSB[0]=str_m[0];
	LSB[0]=str_m[1];

	//printf("%s Volt=%d V (123Tune)  MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, Volt,  MSB[0],LSB[0],  str_m);

return(0);
}






// converts bluetooth binary to advance in degrees
double TuneAdvance2decimal(uint8_t MSB, uint8_t LSB)
{
	uint32_t MSB_dec;
	uint32_t LSB_dec;

	double value;
	double MSB_weight=3.2/1;        // From readout digital advance curve Scaling MSB != scaling LSB
	double LSB_Weight=0.2/1;         // From roudaout advance curve, is not exact due to jump in readout table. 0,2 degrees == 1 bit
	char str[2];                    // char string for hex number

	str[0]=MSB;                     // fill string, to convert a single hex char
	str[1]='\0';
	MSB_dec=strtol(str,NULL,16);

	str[0]=LSB;                     // fill string, to convert a single hex char
	str[1]='\0';
	LSB_dec=strtol(str,NULL,16);

	value=MSB_dec*MSB_weight + LSB_dec*LSB_Weight;

return(value);
}


// float like 20.0 may be actually 19.999999999, and therefore the actual caclulated value may get chopped off to 19.8 (due to bit persision)
// the persision is 0.2 degrees, so adding 0.1 gets rounding fixed for positive numbers.
uint32_t decimal2TuneAdvance(double Advance, uint8_t *MSB, uint8_t *LSB)
{
	uint32_t MSB_dec;
	uint32_t LSB_dec;
	double MSB_weight=3.2/1;	// From readout digital advance curve Scaling MSB != scaling LSB
	double LSB_Weight=0.2/1;		// From roudaout advance curve, is not exact due to jump in readout table.
	char str_m[20];			// char string for hex number
	char str_l[20];			// char string for hex number

	Advance+=0.001;	//helps natural rounding for positive numbers
	MSB_dec=Advance/MSB_weight;		// div/800
	LSB_dec=(Advance-MSB_dec*MSB_weight)/LSB_Weight;	// rest div/50

	sprintf(str_m,"%X",MSB_dec);
	MSB[0]=str_m[0];


	sprintf(str_l,"%X",LSB_dec);
	LSB[0]=str_l[0];

	//printf("%s MSB_dec=%d LSB_dec=%d   MSB=%c LSB=%c str_m=%s str_l=%s\n",__FUNCTION__,  MSB_dec,LSB_dec,  MSB[0],LSB[0],  str_m,str_l);

return(0);

}


// Calcualte Advance based on current RPM and RPM/Advance Graph
double CalculateAdvanceByRPM(void *user_data)
{
	struct server *server = user_data;
	uint32_t RPM=TuneRPM2decimal(server->tune_RPM[0],server->tune_RPM[1]);
	uint32_t Lower_RPM=0, Upper_RPM;	// Graph points
	int i;


	// calculate number of elements first!!!! <TODO>
	for(i=0;i<(10-1);i++) {	// 10-1 due to number of zones insted of boundaries
		Upper_RPM=TuneRPM2decimal(server->tune_AdvanceCurveRPM[i][0],server->tune_AdvanceCurveRPM[i][1]);

		if(i==0 && RPM < Upper_RPM) { // TODO <= ??
			printf("%s AdvanceCurve before first point adding 0 degrees\n",__FUNCTION__);
			return(0);	// 0 degrees or inactive until first point on the map
		}

		// between these points?
		if((Lower_RPM <= RPM) && (RPM <= Upper_RPM) ) {
			double Lower_Advance, Upper_Advance, Advance;

			// linear interpolation
			Upper_Advance=TuneAdvance2decimal(server->tune_AdvanceCurveDegrees[i][0],  server->tune_AdvanceCurveDegrees[i][1]);
			if (i==0) {     // prevent i-1=-1 situation while startup for out of range array.
				// i=0, i-1 <0
                                Lower_Advance = Upper_Advance;
			} else {
				Lower_Advance=TuneAdvance2decimal(server->tune_AdvanceCurveDegrees[i-1][0],server->tune_AdvanceCurveDegrees[i-1][1]);
			}

			if ( (Upper_RPM - Lower_RPM) == 0) {
				// equal, prevend deviding by zero
				Advance = Lower_Advance; 	// or Upper_Advance, which is the same
			} else {
				// cast to double to force franctional devision
				Advance = ((double)(RPM-Lower_RPM)/(Upper_RPM-Lower_RPM)) * (Upper_Advance - Lower_Advance) + Lower_Advance;
			}



			// wrong, but fot testing simpel just send UPP boundary value.... not the linear calculated value. <TODO>
			//return(TuneAdvance2decimal(server->tune_AdvanceCurveDegrees[i][0],server->tune_AdvanceCurveDegrees[i][1]));

			printf("%s AdvanceCurve is active adding %f degrees\n",__FUNCTION__, Advance);

			return(Advance);
			}

		// prepare for next round
		Lower_RPM=Upper_RPM;
		}



// dummy to build framework <TODO>
// 0 when not found, or unforseed/undefined
printf("%s RPM %d not found in graph\n",__FUNCTION__,RPM);
return(0);
}



// Calcualte Advance based on current Pressure and Pressure(Vacuum)/Advance Graph

double CalculateAdvanceByPressure(void *user_data)
{
	struct server *server = user_data;
	uint32_t Pressure=TunePressure2decimal(server->tune_Pressure[0],server->tune_Pressure[1]);
        uint32_t Lower_Pressure=0, Upper_Pressure;	// Graphs points
        int i;
        uint32_t RPM;
        uint32_t MapCurveStartRPM;

        RPM=TuneRPM2decimal(server->tune_RPM[0],server->tune_RPM[1]);
        MapCurveStartRPM=TuneRPM2decimal(server->tune_MapCurveStartRPM[0],server->tune_MapCurveStartRPM[1]);

        if(RPM < MapCurveStartRPM) {
        	printf("%s MAPCurve not active RPM<MapCurveStartRPM (%d<%d) adding 0 degrees\n",__FUNCTION__,RPM,MapCurveStartRPM);

        	return(0);
        }






	// calculate number of elements first!!!! <TODO>
	for(i=0;i<(10-1);i++) {	// 10-1 due to number of zones insted of boundaries
		Upper_Pressure=TunePressure2decimal(server->tune_MapCurvePressure[i][0],server->tune_MapCurvePressure[i][1]);

		if(i==0 && Pressure < Upper_Pressure) { // TODO <= ??
			printf("%s MAPCurve before first point adding 0 degrees\n",__FUNCTION__);
			return(0);	// 0 degrees or inactive until first point on the map
		}

		// between these points?
		if((Lower_Pressure <= Pressure) && (Pressure <= Upper_Pressure) ) {
			double Lower_Advance, Upper_Advance, Advance;

			// linear interpolation MAP graph
			Upper_Advance=TuneAdvance2decimal(server->tune_MapCurveDegrees[i][0],  server->tune_MapCurveDegrees[i][1]);

			if (i==0) {	// prevent i-1=-1 situation while startup for out of range array.
				// i=0, i-1 <0
				Lower_Advance = Upper_Advance;
			} else {
				//  i-1>=0
				Lower_Advance=TuneAdvance2decimal(server->tune_MapCurveDegrees[i-1][0],server->tune_MapCurveDegrees[i-1][1]);
			}

			// cast to double to force franctional devision
			// Prevent devision by zero if Upper_Pressure-Lower_Pressure==0
			if( (Upper_Pressure - Lower_Pressure) ==0) {
				// Upper_Pressure == Lower_Pressure
				Advance =  Lower_Advance; 	// or Upper_Advance, which is the same in this case.
			} else {
				// Upper_Pressure != Lower_Pressure
				Advance = ((double)(Pressure - Lower_Pressure)/(Upper_Pressure - Lower_Pressure)) * (Upper_Advance - Lower_Advance) + Lower_Advance;
			}

			// wrong, but fot testing simpel just send UPPER boundary value.... not the linear calculated value. <TODO>
			//return(TuneAdvance2decimal(server->tune_MapCurveDegrees[i][0],server->tune_MapCurveDegrees[i][1]));

			printf("%s MAPCurve is active RPM>=MapCurveStartRPM (%d>=%d) adding %f degrees\n",__FUNCTION__,RPM,MapCurveStartRPM, Advance);

			return(Advance);
			}

		// prepare for next round
		Lower_Pressure=Upper_Pressure;
		}




// dummy to build framework <TODO>
// 0 when not found, or unforseed/undefined
printf("%s pressure %d not found in graph\n",__FUNCTION__,Pressure);
return(0);
}


// retuns the actual value of Advance based on a number of paramters
// MUST call this function before reading/transmitting/displaying values
 //server->tune_Pressure
 //server->tune_RPM
 //server->tune_AdvanceCurveRPM[]
 //server->tune_MapCurveDegrees[]
 //server->tune_TuningMode_Advance
 //server->tune_MapCurveStartRPM
 //server->tune_RPMLimit
 //
void UpdateRealtimeTuneAdvance(void *user_data)
{
	struct server *server = user_data;
	double Advance;


	// Calculate base Advance based on RPM/Advance graph
	Advance=CalculateAdvanceByRPM(server);

	// Most simplistic for now
	// read current value
	//Advance=TuneAdvance2decimal(server->tune_Advance[0],server->tune_Advance[1]);

	// Add the Advance based on Vacuum/Advance Graph
	Advance+=CalculateAdvanceByPressure(server);

	// Tuning offset, (when in Tuning mode only!)
	if(server->tune_TuningMode_enabled) {
		Advance+=server->tune_TuningMode_Advance;
	}

	// write back updated value
	decimal2TuneAdvance(Advance, &server->tune_Advance[0], &server->tune_Advance[1]);
}



// Turns bluetooth binary into decimal position No 1 -- 10
// even numbers are RPM Curve positions times 2
// uneven numbers are Degrees Curve position times 2 +1
// Curve No 1: RPM 	-> 2 (*2)	// First RPM  fixated on 500, so 2 is illegal
// Curve No 1: Degrees 	-> 3
// Curve No 2: RPM 	-> 4 (*2)
// Curve No 2: Degrees	-> 5 (*2+1)
// Curve No 3: RPM      -> 6 (*2)
// Curve No 3: Degrees  -> 7 (*2+1)
// Curve No 4: RPM 	-> 8 (*2)
// Curve No 4: Degrees	-> 9 (*2+1)
// Curve No 5: RPM      -> 10 (*2)
// Curve No 5: Degrees  -> 11 (*2+1)
// Curve No 6: RPM 	-> 12 (*2)
// Curve No 6: Degrees	-> 13 (*2+1)
// Curve No 7: RPM      -> 14 (*2)
// Curve No 7: Degrees  -> 15 (*2+1)
// Curve No 8: RPM 	-> 16 (*2)
// Curve No 8: Degrees	-> 17 (*2+1)
// Curve No 9: RPM      -> 18 (*2)
// Curve No 9: Degrees  -> 19 (*2+1)
// Curve No 10: RPM      -> 20 (*2)	// Last RPM (numer depending on number of points) is fixated on 8000 RPM
// Curve No 10: Degrees  -> 21 (*2+1)
uint32_t TuneGraphNo2decimal(uint8_t MSB, uint8_t LSB)
{
	uint32_t value;
	char str[3];			// char string for hex number

	str[0]=MSB;			// fill string, to convert a single hex char
	str[1]=LSB;
	str[2]='\0';
	value=strtol(str,NULL,16);


return(value);
}









// Calculates and sets checksum byte [3]
// see checksum.txt for finding/logic deduction
// csum should point to byte number 3. this construciton is to prevent writing other bytes
uint32_t TuneSetnewChecksum(const uint8_t* str,uint8_t *csum)
{

//CSUM= <ID>+0x10 + <MSB-0x30) + (LSB-0x30)
// write byte 3 in str
csum[0]=str[0]+0x10 + str[1]-'0' + str[2]-'0';


return(0);
}




// process rxUUID
static void tune_control_point_write_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					const uint8_t *value, size_t len,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	struct server *server = user_data;
	uint8_t ecode = 0;

	const uint8_t *value_ptr;       // sometimes a 0x24 is added/prepended, need to filter out
        size_t len_cleaned;     // sometimes a 0x24 is added/prepended, need to filter out

 			{ // begin 123response logic
        len_cleaned=len;
        value_ptr=value;

        int rx_cmd_found=0;   // counts the cases it is recognized and responded to. and if zero detect unrecognized command.


        while (len_cleaned>0 && value_ptr[0]==0x24)
        	{
        	//printf("Cleaned prepending 0x24 from received command: ");
        	//print_hex(value_ptr, len_cleaned);
        	// No longer interested in printing these keepalives

        	value_ptr++;
                len_cleaned--;
        	}

          // need to clean tailing 0x24 's tooo (due to new 123 tune software??/new conditions) ???
          // only clean single 0x24 trailing item
          // 2019/01/19 fixup
          if(len_cleaned>0) {
          	if(value_ptr[len_cleaned-1]== 0x24)len_cleaned--;
          }

					// there might be a second 0x24 in some cases (pincode change), fixup 2019/01/28
          if(len_cleaned> 1){
            if(value_ptr[len_cleaned-1]== 0x24)len_cleaned--;
          }

        if(len_cleaned==0)
        	goto done;


	//PRLOG("DBG: tune_control_point_write_cb: %lu\n",len);
	//print_hex(value,len);

	/* Accept Any length to make 123tune app happy
	if (!value || len != 20) {
		ecode = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}
	*/

	if (offset) {
		ecode = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	if (value_ptr[0] == 1) {
		PRLOG("HR: Energy Expended value reset\n");
		server->tune_energy_expended = 0;
	}

// send confirmation:
done:
	gatt_db_attribute_write_result(attrib, id, ecode);



// Additional reply on the txUUID below:


	// Start notify as response
	{
	uint8_t str[] = { 0x0d };
	//uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
	uint8_t str_resp[] = { 0x0d };

	size_t str_len=sizeof(str);
	size_t str_resp_len=sizeof(str_resp);
	size_t i;
	int equal=0;
	if (len_cleaned == str_len)
		{
		equal=1;
		for (i =0 ; i< len_cleaned; i++)
			if (value_ptr[i] != str[i])
				equal=0;
		}
	if(equal == 1)
		{
		rx_cmd_found++;
		printf("%s 1th Command Found----\n",__FUNCTION__);
		print_hex(value_ptr, len_cleaned);

		// produce an answer... on txUUID
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                str_resp, str_resp_len);


		}

	}

	// send resuest
	{
	//uint8_t str[] = { 0x24, 0x24, 0x0d };
	//uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
	//uint8_t str[] = { 0x31, 0x30, 0x40, 0x0d, 0x24 };
	uint8_t str[] = { 0x31, 0x30, 0x40, 0x0d };
	//uint8_t str_resp[] = { 0x0d };
	//uint8_t str_resp1[] = {0x0d, 0x31, 0x30, 0x40, 0x0d, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x30, 0x41, 0x20, 0x30, 0x30, 0x20, 0x30, 0x45, 0x20};
	//uint8_t str_resp2[] = {0x30, 0x30, 0x20, 0x31, 0x30, 0x20, 0x30, 0x30, 0x20, 0x31, 0x34, 0x20, 0x31, 0x34, 0x20, 0x31, 0x45, 0x20, 0x33, 0x43};
	//uint8_t str_resp3[] = {0x20, 0x32, 0x34, 0x20, 0x35, 0x35, 0x20, 0x33, 0x43, 0x20, 0x39, 0x36, 0x20, 0x31, 0x30, 0x33, 0x46, 0x33, 0x0d};

	/* Trying to fragmentate did not work!
	uint8_t str_resp1[] = {0x0d, 0x31, 0x30, 0x40, 0x0d, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x30, 0x41, 0x20, 0x30, 0x30, 0x20, 0x30, 0x45, 0x20,
			       0x30, 0x30, 0x20, 0x31, 0x30, 0x20, 0x30, 0x30, 0x20, 0x31, 0x34, 0x20, 0x31, 0x34, 0x20, 0x31, 0x45, 0x20, 0x33, 0x43,
			       0x20, 0x32, 0x34, 0x20, 0x35, 0x35, 0x20, 0x33, 0x43, 0x20, 0x39, 0x36, 0x20, 0x31, 0x30, 0x33, 0x46, 0x33, 0x0d};
	*/

	size_t str_len=sizeof(str);
	size_t i;
	int equal=0;

	if (len_cleaned == str_len)
		{
		equal=1;
		for (i =0 ; i< len_cleaned; i++)
			if (value_ptr[i] != str[i])
				equal=0;
		}
	if(equal == 1)
		{
		uint8_t buffer[3*20];		// 3x 20 bytes (1 or 2 bytes extra room);
		char csum_hex_str[5];		// 3 bytes used;
		unsigned int bytecounter;
		unsigned int local_bytecounter;
		uint8_t all_initial_values[][3]={	// the actual value without header/slot
			{ 0x46, 0x46, 0x20 }, 	// [0]
			{ 0x46, 0x46, 0x20 }, 	// [1]
			{ 0x30, 0x41, 0x20 }, 	// [2]  nr1 RPM
			{ 0x30, 0x30, 0x20 }, 	// [3]  nr1 advance
			{ 0x30, 0x45, 0x20 },	// [4]  nr2 RPM
			{ 0x30, 0x30, 0x20 }, 	// [5]  nr2 advance
			{ 0x31, 0x30, 0x20 }, 	// [6]  nr3 RPM
			{ 0x30, 0x30, 0x20 }, 	// [7]  nr3 advance
			{ 0x31, 0x34, 0x20 },  	// [8]  nr4 RPM
			{ 0x31, 0x34, 0x20 }, 	// [9]  nr4 advance
			{ 0x31, 0x45, 0x20 }, 	// [10] nr5 RPM
			{ 0x33, 0x43, 0x20 }, 	// [11] nr5 advance
			{ 0x32, 0x34, 0x20 }, 	// [12] nr6 RPM
			{ 0x35, 0x35, 0x20 }, 	// [13] nr6 advance
			{ 0x33, 0x43, 0x20 }, 	// [14] nr7 RPM
			{ 0x39, 0x36, 0x20 }	// [15] nr7 advance
			};
		int csum;		// 3 byte checksum

		rx_cmd_found++;
		printf("%s 2th Command Found----\n",__FUNCTION__);
		print_hex(value_ptr, len_cleaned);

		// produce an answer...

		// fill graph under cmd2 with custom values.
		// Fixed 500 RPM
		//str_resp1[11]=server->tune_AdvanceCurveRPM[0][0];	// Fixed 500 RPM
		//str_resp1[12]=server->tune_AdvanceCurveRPM[0][1];

		// overwrite captrued advance with custom value, calculate csum later.
		all_initial_values[3][0]=server->tune_AdvanceCurveDegrees[0][0];
		all_initial_values[3][1]=server->tune_AdvanceCurveDegrees[0][1];


		// Fill complete unfragmented buffer
		bytecounter=0;
		// copy header @ start
		for (local_bytecounter=0;local_bytecounter<sizeof(str);local_bytecounter++,bytecounter++) {
			buffer[bytecounter]=str[local_bytecounter];
		}
		// copy values (from sniffer for now)
		for(i=0;i < (sizeof(all_initial_values)/3); i++ ) {
			buffer[bytecounter++]=all_initial_values[i][0]; local_bytecounter++;
			buffer[bytecounter++]=all_initial_values[i][1]; local_bytecounter++;
			buffer[bytecounter++]=all_initial_values[i][2]; local_bytecounter++;
		}

		// copy ID  from request str: 0x31 0x30+sequence 0x40=16 (data len)
		buffer[bytecounter++]=str[1];	// skip first byte in cmd2, second, third 2 bytes
		buffer[bytecounter++]=str[2];

		csum=0;
		// calculate checkum, could be integrated in buffer copy above for efficienty, but not for readability.
		for (i=0;i< sizeof(all_initial_values)/3;i++) {
			char str_l[20];

			str_l[0]=all_initial_values[i][0];
			str_l[1]=all_initial_values[i][1];
			str_l[2]='\0';
			csum+=strtol(str_l,NULL,16);
		}
		sprintf(csum_hex_str,"%03X",csum);

		// fill csum
		buffer[bytecounter++]=csum_hex_str[0];
		buffer[bytecounter++]=csum_hex_str[1];
		buffer[bytecounter++]=csum_hex_str[2];

		// add termination 0x0d
		buffer[bytecounter++]=0x0d;	// bytecounter containts the total number of bytes

		printf("%s bytecounter=%d csum=%s\n",__FUNCTION__,bytecounter,csum_hex_str);


		// size is 59, fist 20 bytes fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[0], 20);
	       bytecounter-=20;

		// second 20 bytes fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[20], 20);
               bytecounter-=20;
		// last fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[40],bytecounter );



		}

	}



	{
	//uint8_t str[] = { 0x24, 0x24, 0x0d };
	//uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
	uint8_t str[] = {  0x31, 0x31, 0x40, 0x0d };
	//uint8_t str_resp[] = { 0x0d };
	//uint8_t str_resp1[] = { 0x31, 0x31, 0x40, 0x0d, 0x35, 0x41, 0x20, 0x41, 0x35, 0x20, 0x41, 0x30, 0x20, 0x41, 0x35, 0x20, 0x46, 0x46, 0x20, 0x46 };
	//uint8_t str_resp2[] = { 0x46, 0x20, 0x33, 0x37, 0x20, 0x33, 0x38, 0x20, 0x33, 0x33, 0x20, 0x33, 0x32, 0x20, 0x30, 0x30, 0x20, 0x46, 0x46, 0x20 };
	//uint8_t str_resp3[] = { 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x31, 0x36, 0x20, 0x35, 0x41, 0x20, 0x31, 0x31, 0x38, 0x38, 0x33, 0x0d };
	size_t str_len=sizeof(str);
	//size_t str_resp1_len=sizeof(str_resp1);
	//size_t str_resp2_len=sizeof(str_resp2);
	//size_t str_resp3_len=sizeof(str_resp3);
	size_t i;
	int equal=0;

	if (len_cleaned == str_len)
		{
		equal=1;
		for (i =0 ; i< len_cleaned; i++)
			if (value_ptr[i] != str[i])
				equal=0;
		}
	if(equal == 1)
		{
		uint8_t buffer[3*20];		// 3x 20 bytes (1 or 2 bytes extra room);
		char csum_hex_str[5];		// 3 bytes used;
		unsigned int bytecounter;
		unsigned int local_bytecounter;
		uint8_t all_initial_values[][3]={	// the actual value without header/slot
			{ 0x35, 0x41, 0x20 }, 	// [0]	nr8 RPM
			{ 0x41, 0x35, 0x20 }, 	// [1]  nr8 Advance
			{ 0x41, 0x30, 0x20 }, 	// [2]  nr9 RPM
			{ 0x41, 0x35, 0x20 },  	// [3]  nr9 Advance
			{ 0x46, 0x46, 0x20 },   // [4]  nr10 RPM
			{ 0x46, 0x46, 0x20 }, 	// [5]  nr10 Advance
			{ 0x33, 0x37, 0x20 }, 	// [6]  pincode[0]
			{ 0x33, 0x38, 0x20 }, 	// [7]  pincode[1]
			{ 0x33, 0x33, 0x20 }, 	// [8]  pincode[2]
			{ 0x33, 0x32, 0x20 }, 	// [9]  pincode[3]
			{ 0x30, 0x30, 0x20 }, 	// [10] pincode[4] = '\0'
			{ 0x46, 0x46, 0x20 }, 	// [11]
			{ 0x46, 0x46, 0x20 }, 	// [12]
			{ 0x46, 0x46, 0x20 }, 	// [13]
			{ 0x31, 0x36, 0x20 }, 	// [14] Vacuum advance graph Start@RPM
			{ 0x35, 0x41, 0x20 }	// [15] RPM Limit
			};
		int csum;		// 3 byte checksum

		rx_cmd_found++;
		printf("%s 3th Command Found----\n",__FUNCTION__);
		print_hex(value_ptr, len_cleaned);

		// produce an answer...

		// fill graph under cmd2 with custom values.
		// Fixed 500 RPM
		//str_resp1[11]=server->tune_AdvanceCurveRPM[0][0];	// Fixed 500 RPM
		//str_resp1[12]=server->tune_AdvanceCurveRPM[0][1];

		// overwrite captrued advance with custom value, calculate csum later.
		//all_initial_values[3][0]=server->tune_AdvanceCurveDegrees[0][0];
		//all_initial_values[3][1]=server->tune_AdvanceCurveDegrees[0][1];

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
		bytecounter=0;
		// copy header @ start
		for (local_bytecounter=0;local_bytecounter<sizeof(str);local_bytecounter++,bytecounter++) {
			buffer[bytecounter]=str[local_bytecounter];
		}
		// copy values (from sniffer for now)
		for(i=0;i < (sizeof(all_initial_values)/3); i++ ) {
			buffer[bytecounter++]=all_initial_values[i][0]; local_bytecounter++;
			buffer[bytecounter++]=all_initial_values[i][1]; local_bytecounter++;
			buffer[bytecounter++]=all_initial_values[i][2]; local_bytecounter++;
		}

		// copy ID  from request str: 0x31 0x30+sequence 0x40=16 (data len)
		buffer[bytecounter++]=str[0];	// first 2 butes in cmd and further
		buffer[bytecounter++]=str[1];

		csum=0;
		// calculate checkum, could be integrated in buffer copy above for efficienty, but not for readability.
		for (i=0;i< sizeof(all_initial_values)/3;i++) {
			char str_l[20];

			str_l[0]=all_initial_values[i][0];
			str_l[1]=all_initial_values[i][1];
			str_l[2]='\0';
			csum+=strtol(str_l,NULL,16);
		}
		sprintf(csum_hex_str,"%03X",csum);

		// fill csum
		buffer[bytecounter++]=csum_hex_str[0];
		buffer[bytecounter++]=csum_hex_str[1];
		buffer[bytecounter++]=csum_hex_str[2];

		// add termination 0x0d
		buffer[bytecounter++]=0x0d;	// bytecounter containts the total number of bytes

		printf("%s bytecounter=%d csum=%s\n",__FUNCTION__,bytecounter,csum_hex_str);


		// size is 59, fist 20 bytes fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[0], 20);
	       bytecounter-=20;

		// second 20 bytes fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[20], 20);
               bytecounter-=20;
		// last fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[40],bytecounter );

		}

	}


	{
	//uint8_t str[] = { 0x24, 0x24, 0x0d };
	//uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
	//uint8_t str[] = { 0x24, 0x24, 0x31, 0x31, 0x40, 0x0d };
	uint8_t str[] = { 0x31, 0x32, 0x40, 0x0d  };		// working in 2018
	//uint8_t str[] = { 0x31, 0x32, 0x40, 0x0d, 0x24 };
	//uint8_t str_resp[] = { 0x0d };
	//uint8_t str_resp1[] = { 0x31, 0x32, 0x40, 0x0d, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x30, 0x30, 0x20, 0x33, 0x37, 0x20, 0x31, 0x44, 0x20, 0x33 };
	//uint8_t str_resp2[] = { 0x37, 0x20, 0x31, 0x45, 0x20, 0x33, 0x37, 0x20, 0x32, 0x38, 0x20, 0x33, 0x37, 0x20, 0x35, 0x38, 0x20, 0x30, 0x30, 0x20 };
	//uint8_t str_resp3[] = { 0x36, 0x34, 0x20, 0x30, 0x30, 0x20, 0x43, 0x38, 0x20, 0x30, 0x30, 0x20, 0x31, 0x32, 0x34, 0x43, 0x31, 0x0d };		// copy original
	//uint8_t str_resp3[] = { 0x36, 0x34, 0x20, 0x30, 0x30, 0x20, 0x43, 0x38, 0x20, 0x30, 0x30, 0x20, 0x31, 0x32, 0x34, 0x43, 0x31, 0x0d };	// search for 0x43, 0x38, meaning

	size_t str_len=sizeof(str);
	//size_t str_resp1_len=sizeof(str_resp1);
	//size_t str_resp2_len=sizeof(str_resp2);
	//size_t str_resp3_len=sizeof(str_resp3);
	size_t i;
	int equal=0;

	if (len_cleaned == str_len)
		{
		equal=1;
		for (i =0 ; i< len_cleaned; i++)
			if (value_ptr[i] != str[i])
				equal=0;
		}
	if(equal == 1)
		{
		uint8_t buffer[3*20];		// 3x 20 bytes (1 or 2 bytes extra room);
		char csum_hex_str[5];		// 3 bytes used;
		unsigned int bytecounter;
		unsigned int local_bytecounter;
		uint8_t all_initial_values[][3]={	// the actual value without header/slot
			{ 0x46, 0x46, 0x20 }, 	// [0]
			{ 0x46, 0x46, 0x20 }, 	// [1]
			{ 0x30, 0x30, 0x20 }, 	// [2]	nr1 Vacuum Advance graph: abs pressure (vacuum)
			{ 0x33, 0x37, 0x20 }, 	// [3] 	nr1 Vacuum Advance graph: advance
			{ 0x31, 0x44, 0x20 }, 	// [4]	nr2 Vacuum Advance graph: abs pressure (vacuum)
			{ 0x33, 0x37, 0x20 }, 	// [5] 	nr2 Vacuum Advance graph: advance
			{ 0x31, 0x45, 0x20 }, 	// [6]	nr3 Vacuum Advance graph: abs pressure (vacuum)
			{ 0x33, 0x37, 0x20 }, 	// [7] 	nr3 Vacuum Advance graph: advance
			{ 0x32, 0x38, 0x20 }, 	// [8]	nr4 Vacuum Advance graph: abs pressure (vacuum)
			{ 0x33, 0x37, 0x20 }, 	// [9] 	nr4 Vacuum Advance graph: advance
			{ 0x35, 0x38, 0x20 }, 	// [10]	nr5 Vacuum Advance graph: abs pressure (vacuum)
			{ 0x30, 0x30, 0x20 }, 	// [11] nr5 Vacuum Advance graph: advance
			{ 0x36, 0x34, 0x20 }, 	// [12]	nr6 Vacuum Advance graph: abs pressure (vacuum)
			{ 0x30, 0x30, 0x20 }, 	// [13] nr6 Vacuum Advance graph: advance
			{ 0x43, 0x38, 0x20 }, 	// [14]	nr7 Vacuum Advance graph: abs pressure (vacuum)
			{ 0x30, 0x30, 0x20 }  	// [15] nr7 Vacuum Advance graph: advance
			};
		int csum;		// 3 byte checksum

		rx_cmd_found++;
		printf("%s 4th Command Found----\n",__FUNCTION__);
		print_hex(value_ptr, len_cleaned);

		// produce an answer...

		// fill graph under cmd2 with custom values.
		// Fixed 500 RPM
		//str_resp1[11]=server->tune_AdvanceCurveRPM[0][0];	// Fixed 500 RPM
		//str_resp1[12]=server->tune_AdvanceCurveRPM[0][1];

		// overwrite captrued advance with custom value, calculate csum later.
		//all_initial_values[3][0]=server->tune_AdvanceCurveDegrees[0][0];
		//all_initial_values[3][1]=server->tune_AdvanceCurveDegrees[0][1];

		// copy internal state tables to 123Tune bloothooth protocol format
		// n=1 .. 7, array offset=2
		for(int n=1,i=2;n<=7;n++) {
			all_initial_values[i][0]=server->tune_MapCurvePressure[n-1][0];
			all_initial_values[i++][1]=server->tune_MapCurvePressure[n-1][1];
			all_initial_values[i][0]=server->tune_MapCurveDegrees[n-1][0];
			all_initial_values[i++][1]=server->tune_MapCurveDegrees[n-1][1];
		}

		// Fill complete unfragmented buffer
		bytecounter=0;
		// copy header @ start
		for (local_bytecounter=0;local_bytecounter<sizeof(str);local_bytecounter++,bytecounter++) {
			buffer[bytecounter]=str[local_bytecounter];
		}
		// copy values (from sniffer for now)
		for(i=0;i < (sizeof(all_initial_values)/3); i++ ) {
			buffer[bytecounter++]=all_initial_values[i][0]; local_bytecounter++;
			buffer[bytecounter++]=all_initial_values[i][1]; local_bytecounter++;
			buffer[bytecounter++]=all_initial_values[i][2]; local_bytecounter++;
		}

		// copy ID  from request str: 0x31 0x30+sequence 0x40=16 (data len)
		buffer[bytecounter++]=str[0];	// first 2 butes in cmd and further
		buffer[bytecounter++]=str[1];

		csum=0;
		// calculate checkum, could be integrated in buffer copy above for efficienty, but not for readability.
		for (i=0;i< sizeof(all_initial_values)/3;i++) {
			char str_l[20];

			str_l[0]=all_initial_values[i][0];
			str_l[1]=all_initial_values[i][1];
			str_l[2]='\0';
			csum+=strtol(str_l,NULL,16);
		}
		sprintf(csum_hex_str,"%03X",csum);

		// fill csum
		buffer[bytecounter++]=csum_hex_str[0];
		buffer[bytecounter++]=csum_hex_str[1];
		buffer[bytecounter++]=csum_hex_str[2];

		// add termination 0x0d
		buffer[bytecounter++]=0x0d;	// bytecounter containts the total number of bytes

		printf("%s bytecounter=%d csum=%s\n",__FUNCTION__,bytecounter,csum_hex_str);


		// size is 59, fist 20 bytes fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[0], 20);
	       bytecounter-=20;

		// second 20 bytes fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[20], 20);
               bytecounter-=20;
		// last fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[40],bytecounter );
		}

	}

	{ // MAP/RPM Advance curve are filled after command 5.
	//uint8_t str[] = { 0x24, 0x24, 0x0d };
	//uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
	//uint8_t str[] = { 0x24, 0x24, 0x31, 0x31, 0x40, 0x0d };
	//uint8_t str[] = { 0x24, 0x24, 0x31, 0x32, 0x40, 0x0d };
	uint8_t str[] = { 0x31, 0x33, 0x40, 0x0d };	// was working in 2018
	//uint8_t str[] = { 0x31, 0x33, 0x40, 0x0d, 0x24 };
	//uint8_t str_resp[] = { 0x0d };
	//uint8_t str_resp1[] = { 0x31, 0x33, 0x40, 0x0d, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46 };
	//uint8_t str_resp2[] = { 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20 };
	//uint8_t str_resp3[] = { 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x46, 0x46, 0x20, 0x31, 0x33, 0x46, 0x46, 0x30, 0x0d };

	size_t str_len=sizeof(str);
	//size_t str_resp1_len=sizeof(str_resp1);
	//size_t str_resp2_len=sizeof(str_resp2);
	//size_t str_resp3_len=sizeof(str_resp3);


	size_t i;
	int equal=0;
	if (len_cleaned == str_len)
		{
		equal=1;
		for (i =0 ; i< len_cleaned; i++)
			if (value_ptr[i] != str[i])
				equal=0;
		}
	if(equal == 1)
		{
		uint8_t buffer[3*20];		// 3x 20 bytes (1 or 2 bytes extra room);
		char csum_hex_str[5];		// 3 bytes used;
		unsigned int bytecounter;
		unsigned int local_bytecounter;
		uint8_t all_initial_values[][3]={	// the actual value without header/slot
			{ 0x46, 0x46, 0x20 },	// [0]	nr8  Vacuum Advance graph: abs pressure (vacuum)
			{ 0x46, 0x46, 0x20 }, 	// [1]  nr8 Vacuum Advance graph: advance
			{ 0x46, 0x46, 0x20 },	// [2]  nr9  Vacuum Advance graph: abs pressure (vacuum)
			{ 0x46, 0x46, 0x20 }, 	// [3]  nr9 Vacuum Advance graph: advance
			{ 0x46, 0x46, 0x20 },	// [4]  nr10 Vacuum Advance graph: abs pressure (vacuum)
			{ 0x46, 0x46, 0x20 }, 	// [5]  nr10 Vacuum Advance graph: advance
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
		int csum;		// 3 byte checksum

		rx_cmd_found++;
		printf("%s 5th Command Found----\n",__FUNCTION__);
		print_hex(value_ptr, len_cleaned);

		// produce an answer...

		// fill graph under cmd2 with custom values.
		// Fixed 500 RPM
		//str_resp1[11]=server->tune_AdvanceCurveRPM[0][0];	// Fixed 500 RPM
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

		// Fill complete unfragmented buffer
		bytecounter=0;
		// copy header @ start
		for (local_bytecounter=0;local_bytecounter<sizeof(str);local_bytecounter++,bytecounter++) {
			buffer[bytecounter]=str[local_bytecounter];
		}
		// copy values (from sniffer for now)
		for(i=0;i < (sizeof(all_initial_values)/3); i++ ) {
			buffer[bytecounter++]=all_initial_values[i][0]; local_bytecounter++;
			buffer[bytecounter++]=all_initial_values[i][1]; local_bytecounter++;
			buffer[bytecounter++]=all_initial_values[i][2]; local_bytecounter++;
		}

		// copy ID  from request str: 0x31 0x30+sequence 0x40=16 (data len)
		buffer[bytecounter++]=str[0];	// first 2 butes in cmd and further
		buffer[bytecounter++]=str[1];

		csum=0;
		// calculate checkum, could be integrated in buffer copy above for efficienty, but not for readability.
		for (i=0;i< sizeof(all_initial_values)/3;i++) {
			char str_l[20];

			str_l[0]=all_initial_values[i][0];
			str_l[1]=all_initial_values[i][1];
			str_l[2]='\0';
			csum+=strtol(str_l,NULL,16);
		}
		sprintf(csum_hex_str,"%03X",csum);

		// fill csum
		buffer[bytecounter++]=csum_hex_str[0];
		buffer[bytecounter++]=csum_hex_str[1];
		buffer[bytecounter++]=csum_hex_str[2];

		// add termination 0x0d
		buffer[bytecounter++]=0x0d;	// bytecounter containts the total number of bytes

		printf("%s bytecounter=%d csum=%s\n",__FUNCTION__,bytecounter,csum_hex_str);


		// size is 59, fist 20 bytes fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[0], 20);
	       bytecounter-=20;

		// second 20 bytes fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[20], 20);
               bytecounter-=20;
		// last fragment
	       bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                &buffer[40],bytecounter );
		}

	}

	{ // All Meter value_ptrs show up after command 6  !!!

	//static uint_8 ub = 0x37;
	//static uint_8 lb = 0x46;
	// values start showing up after this repeated exected reguest/response
	//uint8_t str[] = { 0x24, 0x24, 0x0d };
	//uint8_t str[] = { 0x0d, 0x31, 0x30, 0x40, 0x0d };
	//uint8_t str[] = { 0x24, 0x24, 0x31, 0x31, 0x40, 0x0d };
	//uint8_t str[] = { 0x24, 0x24, 0x31, 0x32, 0x40, 0x0d };
	//uint8_t str[] = { 0x24, 0x24, 0x31, 0x33, 0x40, 0x0d };
	uint8_t str[] = {  0x76, 0x40, 0x0d };	// was working in 2018
	//uint8_t str[] = {  0x76, 0x40, 0x0d, 0x24 };
	//uint8_t str_resp[] = { 0x0d };

	// orig command 6:
	//uint8_t str_resp1[] = { 0x76, 0x40, 0x0d, 0x34, 0x30, 0x33, 0x41, 0x36, 0x34, 0x34, 0x31, 0x2d, 0x31, 0x30, 0x2d, 0x34, 0x35, 0x20 };

	// playground to divercover encoding:
	//uint8_t str_resp1[] =   { 0x76, 0x40, 0x0d, 0x34, 0x33, 0x34, 0x39, 0x38, 0x46, 0x34, 0x39, 0x2d, 0x31, 0x30, 0x2d, 0x34, 0x35, 0x20 };
        //                        [request 3Bytes]   [Volt 2Byt],[Temp 2Byt], [BAR 2Byte] BAR?  BAR?   ??     '1'   '0'   ??    '4'   '5'     ??
	//uint8_t str_resp2[] = {  };
	//uint8_t str_resp3[] = {  };

	size_t str_len=sizeof(str);
	//size_t str_resp2_len=sizeof(str_resp2);
	//size_t str_resp3_len=sizeof(str_resp3);

	size_t i;
	int equal=0;

	if (len_cleaned == str_len)
		{
		equal=1;
		for (i =0 ; i< len_cleaned; i++)
			if (value_ptr[i] != str[i])
				equal=0;
		}
	if(equal == 1)
		{
		unsigned int bytecounter;

		uint8_t str_resp1[] =   {
			0x76, 0x40, 0x0d,	// the optional trailing 0x24 must NOT be added....
			0x34, 0x33, 		// [3][4] Voltage
			0x34, 0x39, 		// [5][6] Temperature
			0x37, 0x34, 		// [7][8] Pressure
			0x34, 0x31, 0x2d, 0x31, 0x30, 0x2d, 0x34, 0x35, 0x20  // Version: '41-10-45 '
			};
		size_t str_resp1_len=sizeof(str_resp1);

		bytecounter=str_len;//len_cleaned == str_len;

		//
		//decimal2TuneVoltage(13.0, &server->tune_Voltage[0],&server->tune_Voltage[1]);

		str_resp1[bytecounter++] = server->tune_Voltage[0];                 // read actual value MSB from internal state
		str_resp1[bytecounter++] = server->tune_Voltage[1];			// read actual value LSB from internal state

		//decimal2TuneTemperature(70 ,&server->tune_Temperature[0],&server->tune_Temperature[1]);

		str_resp1[bytecounter++] = server->tune_Temperature[0];             // read actual value MSB from internal state
		str_resp1[bytecounter++] = server->tune_Temperature[1];		// read actual value LSB from internal state

		//decimal2TunePressure(200, &server->tune_Pressure[0],&server->tune_Pressure[1]);
		str_resp1[bytecounter++] = server->tune_Pressure[0];             	// read actual value MSB from internal state
		str_resp1[bytecounter++] = server->tune_Pressure[1];		// read actual value LSB from internal state

		// Test these byte location.. failed.
		//str_resp1[bytecounter++] = server->tune_Temperature[0];             // read actual value MSB from internal state
		//str_resp1[bytecounter++] = server->tune_Temperature[1];		// read actual value LSB from internal state

		#define P123TUNE_Device_Version "41-10-45 "

		strncpy((char*)&str_resp1[bytecounter],P123TUNE_Device_Version,sizeof(P123TUNE_Device_Version));

		printf("%s Temperature %d Celcius\n",__FUNCTION__,TuneTemperature2decimal(server->tune_Temperature[0],server->tune_Temperature[1]));	// test reverse function  (and forward)

		rx_cmd_found++;
		printf("%s 6th Command Found----\n",__FUNCTION__);
		print_hex(value_ptr, len_cleaned);

		// produce an answer...
	        bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                str_resp1, str_resp1_len);
		}

	}


	// Tuning implementation UUID: 59:d1:99:c2:b1:93:43:af:25:4c:05:72:0c:26:03:bf
	// 74 is Toggle Tuning mode (Start/Stop)
	// 61 == +
	// 62 == -
	// 02 == exit 123tune programm -> UUID 24:0a:c0:a6:df:95:0e:90:9c:47:4c:69:b9:88:79:a8 (not here)
	{
	if(len_cleaned==1)
		{
		//static uint8_t tune_Advance_bck[2];	// TODO, needs to based on graph
		switch(value_ptr[0]) {
			case 0x61:	// + in Tuning mode (Advance)
				rx_cmd_found++;
				printf("%s Tuning mode + Found\n",__FUNCTION__);
				server->tune_TuningMode_Advance++;

				// Direct modify advance, need to changed later, based on curve [TODO]
				// decimal2TuneAdvance( TuneAdvance2decimal(server->tune_Advance[0],server->tune_Advance[1])   + 1.0  ,&server->tune_Advance[0],&server->tune_Advance[1]);

				// Directly modify tune_TuningMode_Advance, and keep away from actual server->tune_Advance[] value
				//decimal2TuneAdvance( TuneAdvance2decimal(server->tune_TuningMode_Advance[0],server->tune_TuningMode_Advance[1])   + 1.0  ,&server->tune_TuningMode_Advance[0],&server->tune_TuningMode_Advance[1]);

				break;
			case 0x72:	// - in Tuning mode (Advance)
				rx_cmd_found++;
				printf("%s Tuning mode - Found\n",__FUNCTION__);
				server->tune_TuningMode_Advance--;

				// Direct modify advance, need to changed later, based on curve [TODO]
				//decimal2TuneAdvance( TuneAdvance2decimal(server->tune_Advance[0],server->tune_Advance[1])   - 1.0  ,&server->tune_Advance[0],&server->tune_Advance[1]);

				// Directly modify tune_TuningMode_Advance, and keep away from actual server->tune_Advance[] value
				//decimal2TuneAdvance( TuneAdvance2decimal(server->tune_TuningMode_Advance[0],server->tune_TuningMode_Advance[1])   - 1.0  ,&server->tune_TuningMode_Advance[0],&server->tune_TuningMode_Advance[1]);


				break;
			case 0x74: 	// Toggle Tuning mode
				// TODO needs to be modyfied for using graphs
				rx_cmd_found++;
				printf("%s Tuning mode Toggle Found\n",__FUNCTION__);
				if(!server->tune_TuningMode_enabled) {
					// Enable
					// Start always at 0
					//decimal2TuneAdvance(0 ,&server->tune_TuningMode_Advance[0],&server->tune_TuningMode_Advance[1]);
					server->tune_TuningMode_Advance=0;

				} else { 	// restore state
					// Disable
					// setting to 0 is attractive, but it's simply a don't care when it's not enabled.
					// other code shoud honor 'server->tune_TuningMode_enabled'
					// decimal2TuneAdvance(0 ,&server->tune_TuningMode_Advance[0],&server->tune_TuningMode_Advance[1]);
				}

				server->tune_TuningMode_enabled = !server->tune_TuningMode_enabled;	//Toggle status boolean
				break;
			default:	// do not touch this
				break;

			}
		// no response other then acknowledgement
		}
	}




	// write curve/map implementation 2017/02/07
	{
	//uint8_t str[] = {  0x31, 0x30, 0x2D, 0x30, 0x30, 0x34, 0x0D };
	//uint8_t str_resp1[] =   { 0x31, 0x30, 0x2D, 0x30, 0x30, 0x34, 0x0D };
	//const uint8_t *value_ptr;	// sometimes a 0x24 is added/prepended, need to filter out
	//size_t len_cleaned;	// sometimes a 0x24 is added/prepended, need to filter out

	//len_cleaned=len;
	//value_ptr=value;

	//size_t str_len=sizeof(str);
	//size_t str_resp1_len=sizeof(str_resp1);

	// prepending cleanup is done in the beginning of this function
	if( len == 8 && value_ptr[0] == 0x24) {	// skip first 0x24 when strings still meets expectations
		value_ptr++;
		len_cleaned--;
	}
	// postpending cleanup is _not_ done at begin of this function
	if( len == 8 && value_ptr[7] == 0x24) {	// skip last 0x24 when strings still meets expectations
		//value_ptr++;
		len_cleaned--;
	}

	//size_t i;
	//int equal=0;
	if (len_cleaned == 7)	// 7 chars, or more chars (uncleaned)
		{
		int nr;		// graph index number 1 .. 10
		if ( value_ptr[2] == 0x2D && value_ptr[3] == 0x30 && value_ptr[6] == 0x0D )	//
		 {
		 switch (value_ptr[4]) {
			case 0x30:	// Advance curve Remove Command ????
		 		printf("%02X Advance Curve Command Found: ",value_ptr[4]);

		 		switch (value_ptr[5]) {
		 					// can't adjust no 1 RPM
		 			case 0x34:	// Advance graps entries: No 2
		 			case 0x36:	// Advance graps entries: No 3
		 			case 0x38:	// Advance graps entries: No 4
		 			case 0x41:	// Advance graps entries: No 5
		 			case 0x43:      // Advance graps entries: No 6
		 			case 0x45:      // Advance graps entries: No 7
						rx_cmd_found++;
		 				nr=TuneGraphNo2decimal(value_ptr[4],value_ptr[5])/2;
		 				printf("%02X (ADD)RPM value %04d rpm at curve postion No: %d\n",value_ptr[5],TuneRPM2decimal(value_ptr[0],value_ptr[1]), nr );
						// Store new value in internal server state: range checking should be done here
						server->tune_AdvanceCurveRPM[nr -1][0]=value_ptr[0];
						server->tune_AdvanceCurveRPM[nr -1][1]=value_ptr[1];

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
		 			case 0x35:	// Degrees: no 2
		 			case 0x37:      // Degrees: no 3
		 			case 0x39:      // Degrees: no 4
		 			case 0x42:      // Degrees: no 5
		 			case 0x44:      // Degrees: no 6
		 			case 0x46:      // Degrees: no 7
						rx_cmd_found++;
		 				nr=(TuneGraphNo2decimal(value_ptr[4],value_ptr[5])-1)/2;
						printf("%02X (ADD)Advance value %02.2f degrees at curve postion No: %d\n",value_ptr[5],TuneAdvance2decimal(value_ptr[0],value_ptr[1]), nr );
						// Store new value in internal server state: range checking should be done here
						server->tune_AdvanceCurveDegrees[nr -1][0]=value_ptr[0];
						server->tune_AdvanceCurveDegrees[nr -1][1]=value_ptr[1];
		 				break;
					default:
						printf("UNKNOWN\n");
						break;
		 		}

				print_hex(value_ptr, len_cleaned);
				bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                value_ptr, len_cleaned);

				break;
		 	case 0x31:	// Advance Curve ???
		 		// 35 35 2D 30 31 41 0D Immobilizer ON
		 		// 41 41 2D 30 31 41 0D Immobilizer Off

		 		printf("%02X Advance Curve Command Found: ",value_ptr[4]);
		 		switch (value_ptr[5]) {
		 			case 0x30:	// Advance graps entries: No 8
		 			case 0x32:	// Advance graps entries: No 9
		 			case 0x34:	// Advance graps entries: No 10, is created automatically by app when enough points are inserted
						rx_cmd_found++;
		 				nr=TuneGraphNo2decimal(value_ptr[4],value_ptr[5])/2 ;
		 				printf("%02X (ADD)RPM value %04d rpm at curve postion No: %d\n",value_ptr[5],TuneRPM2decimal(value_ptr[0],value_ptr[1]), nr );
						// Store new value in internal server state: range checking should be done here
						server->tune_AdvanceCurveRPM[nr -1][0]=value_ptr[0];
						server->tune_AdvanceCurveRPM[nr -1][1]=value_ptr[1];

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
					case 0x33:	// Degrees: no 9
					case 0x35:      // Degrees: no 10
						rx_cmd_found++;
						nr=(TuneGraphNo2decimal(value_ptr[4],value_ptr[5])-1)/2;
						printf("%02X (ADD)Advance value %02.2f degrees at curve postion No: %d\n",value_ptr[5],TuneAdvance2decimal(value_ptr[0],value_ptr[1]), nr );
						// Store new value in internal server state: range checking should be done here
						server->tune_AdvanceCurveDegrees[nr -1][0]=value_ptr[0];
						server->tune_AdvanceCurveDegrees[nr -1][1]=value_ptr[1];
						break;
		 			//case 0x34:
		 			//	printf("%02X (ADD)RPM value %04d rpm\n",value_ptr[5],TuneRPM2decimal(value_ptr[0],value_ptr[1]));
		 			//	break;

		 				// 33 30 2D 30 31 36 0D set PIN :: [value,value][30,31][pos-0x36]
					case 0x36:
					case 0x37:
					case 0x38:
					case 0x39:	// and 0x40 perhaps ???? no more then 4 char's are allowed in 123-app... 0x40 may be used for something else.
						rx_cmd_found++;
						printf("%02X PINSET[%d]=\'%c\'\n",value_ptr[5], value_ptr[5]-0x36 ,TunePinCode2char(value_ptr[0],value_ptr[1]) );
						// store value in internal server
						server->tune_Pincode[value_ptr[5]-0x36][0]=value_ptr[0];
						server->tune_Pincode[value_ptr[5]-0x36][1]=value_ptr[1];
						break;

		 			case 0x41:
		 				// 35 35 2D 30 31 41 0D
		 				if ( (value_ptr[0]==0x35) && (value_ptr[1]==0x35) ) {
							rx_cmd_found++;
			 				printf("%02X IMMOBILIZER: ON\n",value_ptr[5] );
			 				server->tune_IMMOBILIZED = true;
			 				}
						// 41 41 2D 30 31 41 0D
		 				if ( (value_ptr[0]==0x41) && (value_ptr[1]==0x41) ) {
							rx_cmd_found++;
			 				printf("%02X IMMOBILIZER: OFF \n",value_ptr[5] );
			 				server->tune_IMMOBILIZED = false;
			 				}
						break;
		 			case 0x45:
						rx_cmd_found++;
		 				printf("%02X Starts@RPM value %04d rpm\n",value_ptr[5],TuneRPM2decimal(value_ptr[0],value_ptr[1]) );
		 				server->tune_MapCurveStartRPM[0]=value_ptr[0];
		 				server->tune_MapCurveStartRPM[1]=value_ptr[1];
		 				break;
		 			case 0x46:
						rx_cmd_found++;
		 				printf("%02X MAX RPM value %04d rpm\n",value_ptr[5],TuneRPM2decimal(value_ptr[0],value_ptr[1]) );
		 				server->tune_RPMLimit[0]=value_ptr[0];
		 				server->tune_RPMLimit[1]=value_ptr[1];
		 				break;
					default:
						printf("UNKNOWN\n");
						break;
		 		}

				print_hex(value_ptr, len_cleaned);
				bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                value_ptr, len_cleaned);


		 		break;
			case 0x32:	// Vacuum map
		 		printf("%s Vacuum MAP Curve Command Found----\n",__FUNCTION__);
		 		// Pos 1 -> 35, 2 -> 37 ...

		 		switch (value_ptr[5]) {
		 					// can't adjust no 1
		 			case 0x34:	// Advance graps entries: No 2
		 			case 0x36:	// Advance graps entries: No 3
		 			case 0x38:	// Advance graps entries: No 4
		 			case 0x41:	// Advance graps entries: No 5
		 			case 0x43:      // Advance graps entries: No 6
		 			case 0x45:      // Advance graps entries: No 7
						rx_cmd_found++;
		 				nr=TuneGraphNo2decimal(value_ptr[4]-2,value_ptr[5])/2; // graph starts @ ofsset of 2, uneven numbers
		 				printf("%02X (ADD)pressure value %04d kP at curve postion No: %d\n",value_ptr[5],TunePressure2decimal(value_ptr[0],value_ptr[1]), nr  );
						// Store new value in internal server state: range checking should be done here
						server->tune_MapCurvePressure[nr -1][0]=value_ptr[0];
						server->tune_MapCurvePressure[nr -1][1]=value_ptr[1];


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
		 			case 0x35:	// Degrees: no 2
		 			case 0x37:      // Degrees: no 3
		 			case 0x39:      // Degrees: no 4
		 			case 0x42:      // Degrees: no 5
		 			case 0x44:      // Degrees: no 6
		 			case 0x46:      // Degrees: no 7
						rx_cmd_found++;
		 				nr=(TuneGraphNo2decimal(value_ptr[4]-2,value_ptr[5])-1)/2; // graph starts @ ofsset of 2, uneven numbers
		 				printf("%02X (ADD)Crankshaft value %02.2f degrees at curve postion No: %d\n",value_ptr[5],TuneAdvance2decimal(value_ptr[0],value_ptr[1]), nr  );
						// Store new value in internal server state: range checking should be done here
						server->tune_MapCurveDegrees[nr -1][0]=value_ptr[0];
						server->tune_MapCurveDegrees[nr -1][1]=value_ptr[1];
		 				break;
					default:
						printf("UNKNOWN\n");
						break;
		 		}




				print_hex(value_ptr, len_cleaned);
				bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                value_ptr, len_cleaned);

				break;
			case 0x33:	// Save command????
				// 30:30:2d:30:32:44:0d
				// 43:38:2d:30:32:45:0d
		 		printf("%s Vacuum MAP Curve Command Found----\n",__FUNCTION__);

		 		switch (value_ptr[5]) {
		 			case 0x30:	// Advance graps entries: No 8
		 			case 0x32:	// Advance graps entries: No 9
		 			case 0x34:	// Advance graps entries: No 10, is created automatically by app when enough points are inserted
						rx_cmd_found++;
		 				nr=TuneGraphNo2decimal(value_ptr[4]-2,value_ptr[5])/2;// graph starts @ ofsset of 2, uneven numbers
		 				printf("%02X (ADD)pressure value %04d kP at curve postion No: %d\n",value_ptr[5],TunePressure2decimal(value_ptr[0],value_ptr[1]), nr  );
						// Store new value in internal server state: range checking should be done here
						server->tune_MapCurvePressure[nr -1][0]=value_ptr[0];
						server->tune_MapCurvePressure[nr -1][1]=value_ptr[1];


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
					case 0x33:	// Degrees: no 9
					case 0x35:      // Degrees: no 10
						rx_cmd_found++;
						nr=(TuneGraphNo2decimal(value_ptr[4]-2,value_ptr[5])-1)/2 ;  // graph starts @ ofsset of 2, uneven numbers
		 				printf("%02X (ADD)Crankshaft value %02.2f degrees at curve postion No: %d\n",value_ptr[5],TuneAdvance2decimal(value_ptr[0],value_ptr[1]), nr );
						// Store new value in internal server state: range checking should be done here
						server->tune_MapCurveDegrees[nr -1][0]=value_ptr[0];
						server->tune_MapCurveDegrees[nr -1][1]=value_ptr[1];
		 				break;
					default:
						printf("UNKNOWN\n");
						break;
		 		}





				print_hex(value_ptr, len_cleaned);
				bt_gatt_server_send_notification(server->gatt,
                                                server->tune_msrmt_handle,
                                                value_ptr, len_cleaned);

				break;

			default:
				printf("UNKNOWN\n");
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
                        //delay(4000);
                        sleep(4);
            }
            if(rx_cmd_found==0){
                        printf("cmd received and it was decode\n");
                        print_hex(value_ptr, len_cleaned);
            }
            if(rx_cmd_found>1){
                        printf("cmd is decoded by multiple (%d) code blocks, this MUST be wrong!!!\n",rx_cmd_found);
                        print_hex(value_ptr, len_cleaned);
                        //delay(4000);
                        sleep(4);
            }
          } else {
              printf("Decoded as keepalive/empty command\n");
          }

				} // end begin 123response logic
}

static void confirm_write(struct gatt_db_attribute *attr, int err,
							void *user_data)
{
	if (!err)
		return;

	fprintf(stderr, "Error caching attribute %p - err: %d\n", attr, err);
	exit(1);
}

static void populate_gap_service(struct server *server)
{
	bt_uuid_t uuid;
	struct gatt_db_attribute *service, *tmp;
	uint16_t appearance;

	/* Add the GAP service */
	bt_uuid16_create(&uuid, UUID_GAP);
	service = gatt_db_add_service(server->db, &uuid, true, 6);

	/*
	 * Device Name characteristic. Make the value dynamically read and
	 * written via callbacks.
	 */
	bt_uuid16_create(&uuid, GATT_CHARAC_DEVICE_NAME);
	gatt_db_service_add_characteristic(service, &uuid,
					BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
					BT_GATT_CHRC_PROP_READ |
					BT_GATT_CHRC_PROP_EXT_PROP,
					gap_device_name_read_cb,
					gap_device_name_write_cb,
					server);

	bt_uuid16_create(&uuid, GATT_CHARAC_EXT_PROPER_UUID);
	gatt_db_service_add_descriptor(service, &uuid, BT_ATT_PERM_READ,
					gap_device_name_ext_prop_read_cb,
					NULL, server);

	/*
	 * Appearance characteristic. Reads and writes should obtain the value
	 * from the database.
	 */
	bt_uuid16_create(&uuid, GATT_CHARAC_APPEARANCE);
	tmp = gatt_db_service_add_characteristic(service, &uuid,
							BT_ATT_PERM_READ,
							BT_GATT_CHRC_PROP_READ,
							NULL, NULL, server);

	/*
	 * Write the appearance value to the database, since we're not using a
	 * callback.
	 */
	put_le16(128, &appearance);
	gatt_db_attribute_write(tmp, 0, (void *) &appearance,
							sizeof(appearance),
							BT_ATT_OP_WRITE_REQ,
							NULL, confirm_write,
							NULL);

	gatt_db_service_set_active(service, true);
}

static void populate_gatt_service(struct server *server)
{
	bt_uuid_t uuid;
	struct gatt_db_attribute *service, *svc_chngd;

	/* Add the GATT service */
	bt_uuid16_create(&uuid, UUID_GATT);
	service = gatt_db_add_service(server->db, &uuid, true, 4);

	bt_uuid16_create(&uuid, GATT_CHARAC_SERVICE_CHANGED);
	svc_chngd = gatt_db_service_add_characteristic(service, &uuid,
			BT_ATT_PERM_READ,
			BT_GATT_CHRC_PROP_READ | BT_GATT_CHRC_PROP_INDICATE,
			gatt_service_changed_cb,
			NULL, server);
	server->gatt_svc_chngd_handle = gatt_db_attribute_get_handle(svc_chngd);

	bt_uuid16_create(&uuid, GATT_CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				gatt_svc_chngd_ccc_read_cb,
				gatt_svc_chngd_ccc_write_cb, server);

	gatt_db_service_set_active(service, true);
}

static void populate_hr_service(struct server *server)
{
	bt_uuid_t uuid;
	struct gatt_db_attribute *service, *hr_msrmt, *body;
	uint8_t body_loc = 1;  /* "Chest" */

	/* Add Heart Rate Service */
	bt_uuid16_create(&uuid, UUID_HEART_RATE);
	service = gatt_db_add_service(server->db, &uuid, true, 8);
	server->hr_handle = gatt_db_attribute_get_handle(service);

	/* HR Measurement Characteristic */
	bt_uuid16_create(&uuid, UUID_HEART_RATE_MSRMT);
	hr_msrmt = gatt_db_service_add_characteristic(service, &uuid,
						BT_ATT_PERM_NONE,
						BT_GATT_CHRC_PROP_NOTIFY,
						NULL, NULL, NULL);
	server->hr_msrmt_handle = gatt_db_attribute_get_handle(hr_msrmt);

	bt_uuid16_create(&uuid, GATT_CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
					BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
					hr_msrmt_ccc_read_cb,
					hr_msrmt_ccc_write_cb, server);

	/*
	 * Body Sensor Location Characteristic. Make reads obtain the value from
	 * the database.
	 */
	bt_uuid16_create(&uuid, UUID_HEART_RATE_BODY);
	body = gatt_db_service_add_characteristic(service, &uuid,
						BT_ATT_PERM_READ,
						BT_GATT_CHRC_PROP_READ,
						NULL, NULL, server);
	gatt_db_attribute_write(body, 0, (void *) &body_loc, sizeof(body_loc),
							BT_ATT_OP_WRITE_REQ,
							NULL, confirm_write,
							NULL);

	/* HR Control Point Characteristic */
	bt_uuid16_create(&uuid, UUID_HEART_RATE_CTRL);
	gatt_db_service_add_characteristic(service, &uuid,
						BT_ATT_PERM_WRITE,
						BT_GATT_CHRC_PROP_WRITE,
						NULL, hr_control_point_write_cb,
						server);

	if (server->hr_visible)
		gatt_db_service_set_active(service, true);
}


// 123 tune populate
static void populate_tune_service(struct server *server)
{
	bt_uuid_t uuid;
	struct gatt_db_attribute *service, *body, *tune_msrmt;
	//struct gatt_db_attribute *hr_msrmt;

	// See http://www.byteworks.us/Byte_Works/Blog/Entries/2012/12/28_Build_Your_Own_Bluetooth_low_energy_based_circuits_using_the_Blue_Radios_BR-XB-LE4.0-S2.html
	// which probably has taken as literal example by 123Tune developer

	// 0xDA2B84F1627948DEBDC0AFBEA0226079
	// DA2B84F1-6279-48DE-BDC0-AFBEA0226079
	// service uuid
	// from byteworks example: blueRadiosUUID$ = "DA2B84F1-6279-48DE-BDC0-AFBEA0226079"
	// BR-XB-LE4.0-S2 service BRSP "Blue Radio Serial Port"
	const uint128_t u128_BRSP_Service = {
		.data = { 0xDA, 0x2B, 0x84, 0xF1, 0x62, 0x79, 0x48, 0xDE, 0xBD, 0xC0, 0xAF, 0xBE, 0xA0, 0x22, 0x60, 0x79} };

	// 99564a02-dc01-4d3c-b04e-3bb1ef0571b2
	// read
	// from byteworks example: infoUUID$ = "99564A02-DC01-4D3C-B04E-3BB1EF0571B2"
	const uint128_t u128_info = {
		.data = { 0x99, 0x56, 0x4A, 0x02, 0xDC, 0x01, 0x4D, 0x3C, 0xB0, 0x4E, 0x3B, 0xB1, 0xEF, 0x05, 0x71, 0xB2} };

	// 03:00:02:18:00:1a:00:1c:00:1d:00:1f:00:21:00:22:00 (for easy finding it back from wireshark)
	uint8_t body_info[] = { 0x03, 0x00, 0x02, 0x18, 0x00, 0x1a, 0x00, 0x1c, 0x00, 0x1d, 0x00, 0x1f, 0x00, 0x21, 0x00, 0x22, 0x00};
	//uint8_t body_loc_b271[] = { 0x00};

	// a87988b9-694c-479c-900e-95dfa6c00a24
	// read/write
	// from byteworks example: modeUUID$ = "A87988B9-694C-479C-900E-95DFA6C00A24"
	const uint128_t u128_modeUUID = {
		.data = { 0xA8, 0x79, 0x88, 0xB9, 0x69, 0x4C, 0x47, 0x9C, 0x90, 0x0E, 0x95, 0xDF, 0xA6, 0xC0, 0x0A, 0x24} };

	uint8_t body_mode = 1;  /* "Chest" */  //only used  at startign 123tune and exitting

	// bf03260c-7205-4c25-af43-93b1c299d159
	// Write
	// from byteworks example: rxUUID$ = "BF03260C-7205-4C25-AF43-93B1C299D159"
	const uint128_t u128_rxUUID = {
		.data = { 0xBF, 0x03, 0x26, 0x0C, 0x72, 0x05, 0x4C, 0x25, 0xAF, 0x43, 0x93, 0xB1, 0xC2, 0x99, 0xD1, 0x59} };

	// 18cda784-4bd3-4370-85bb-bfed91ec86af
	// Notify
	// from byteworks example: txUUID$ = "18CDA784-4BD3-4370-85BB-BFED91EC86AF"
	const uint128_t u128_txUUID = {
		.data = { 0x18, 0xCD, 0xA7, 0x84, 0x4B, 0xD3, 0x43, 0x70, 0x85, 0xBB, 0xBF, 0xED, 0x91, 0xEC, 0x86, 0xAF} };

	/* Add Heart Rate Service */
	//bt_uuid16_create(&uuid, UUID_GATT);
	//bt_uuid16_create(&uuid, UUID_123TUNE_UNKNOWN);
	//bt_string_to_uuid(&uuid,UUID_123TUNE_UNKNOWN_STR);
	//bt_uuid128_create(uuid, UUID_123TUNE_UNKNOWN);

	// 0xDA2B84F1627948DEBDC0AFBEA0226079
	//u128 = {
	//	.data { 0xDA, 0x2B, 0x84, 0xF1, 0x62, 0x79, 0x48, 0xDE, 0xBD, 0xC0, 0xAF, 0xBE, 0xA0, 0x22, 0x60, 0x79} };
	// BLE Service blueRadiosUUID: "DA2B84F1-6279-48DE-BDC0-AFBEA0226079"
	bt_uuid128_create(&uuid,u128_BRSP_Service);

	service = gatt_db_add_service(server->db, &uuid, true, 32);
	server->tune_handle = gatt_db_attribute_get_handle(service);


	/* READ from remote and write it locally */
	// b271
	// Characteristic: infoUUID$ = "99564A02-DC01-4D3C-B04E-3BB1EF0571B2"
	bt_uuid128_create(&uuid, u128_info);

	body = gatt_db_service_add_characteristic(service, &uuid,
						BT_ATT_PERM_READ,
						BT_GATT_CHRC_PROP_READ,
						NULL, NULL, server);
	gatt_db_attribute_write(body, 0, (void *) &body_info, sizeof(body_info),
							BT_ATT_OP_WRITE_REQ,
							NULL, confirm_write,
							NULL);



	// READ transformed into READ + WRITE , may(probably) not complete

	// this uuid received write request with value 1, and returns value 1. in case of exit for the 123tune progamm the command 02 send by 123tune program.
	// Characteristic: modeUUID$ = "A87988B9-694C-479C-900E-95DFA6C00A24"
	// Set the mode to remote command mode (1) or 2??? exit command mode?
	bt_uuid128_create(&uuid, u128_modeUUID);

	body = gatt_db_service_add_characteristic(service, &uuid,
						BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
						BT_GATT_CHRC_PROP_READ | BT_GATT_CHRC_PROP_WRITE,
						NULL, NULL, server);
	gatt_db_attribute_write(body, 0, (void *) &body_mode, sizeof(body_mode),
							BT_ATT_OP_WRITE_REQ,
							NULL, confirm_write,
							NULL);

	/* WRITE Characteristic */
	//* gets written into first time:
	// Value: 24:24:0d:24:24:24:24:24:24:24:24:24:24:24:24:24:0d:31:30:40
	// lenth: 20 bytes
	// Characteristic rxUUID$ = "BF03260C-7205-4C25-AF43-93B1C299D159"
	bt_uuid128_create(&uuid, u128_rxUUID);
	gatt_db_service_add_characteristic(service, &uuid,
						BT_ATT_PERM_WRITE,
						BT_GATT_CHRC_PROP_WRITE,
						NULL, tune_control_point_write_cb,
						server);



	// NOTIFY transformed into INDICATE ... not finished, reverted to NOTIFY
	// Characteristic txUUID$ = "18CDA784-4BD3-4370-85BB-BFED91EC86AF"
	bt_uuid128_create(&uuid, u128_txUUID);

	tune_msrmt = gatt_db_service_add_characteristic(service, &uuid,
						BT_ATT_PERM_NONE,
						BT_GATT_CHRC_PROP_NOTIFY,
						NULL, NULL, NULL);
	server->tune_msrmt_handle = gatt_db_attribute_get_handle(tune_msrmt);


        // required Item??? 0x2902 Client Characteristic Configuration) attributes on the device.
	bt_uuid16_create(&uuid, GATT_CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
					BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
					tune_msrmt_ccc_read_cb,
					tune_msrmt_ccc_write_cb, server);



	if (server->tune_visible)
		gatt_db_service_set_active(service, true);
}


// 123 tune Battery populate
static void populate_tune_battery_service(struct server *server)
{
	bt_uuid_t uuid;
	struct gatt_db_attribute *service, *body;
	uint8_t body_loc = 0x64;  /* "Chest" */

	/* Add Battery Service Service */
	bt_uuid16_create(&uuid, 0x180F);

	service = gatt_db_add_service(server->db, &uuid, true, 32);
	server->tune_handle = gatt_db_attribute_get_handle(service);

	/*
	 *  Battery Level
	 *  Make reads obtain the value from
	 * the database. (old stuff)
	 */
	bt_uuid16_create(&uuid, 0x2a19 );
	body = gatt_db_service_add_characteristic(service, &uuid,
						BT_ATT_PERM_READ,
						BT_GATT_CHRC_PROP_READ,
						NULL, NULL, server);
	gatt_db_attribute_write(body, 0, (void *) &body_loc, sizeof(body_loc),
							BT_ATT_OP_WRITE_REQ,
							NULL, confirm_write,
							NULL);

	if (server->tune_visible)
		gatt_db_service_set_active(service, true);
}





static void populate_db(struct server *server)
{
	populate_gap_service(server);
	populate_gatt_service(server);
	populate_hr_service(server);
	populate_tune_service(server);
	populate_tune_battery_service(server);
}

static struct server *server_create(int fd, uint16_t mtu, bool hr_visible, bool tune_visible)
{
	struct server *server;
	size_t name_len = strlen(test_device_name);
	int graph_index;

	server = new0(struct server, 1);
	if (!server) {
		fprintf(stderr, "Failed to allocate memory for server\n");
		return NULL;
	}

	server->att = bt_att_new(fd, false);
	if (!server->att) {
		fprintf(stderr, "Failed to initialze ATT transport layer\n");
		goto fail;
	}

	if (!bt_att_set_close_on_unref(server->att, true)) {
		fprintf(stderr, "Failed to set up ATT transport layer\n");
		goto fail;
	}

	if (!bt_att_register_disconnect(server->att, att_disconnect_cb, NULL,
									NULL)) {
		fprintf(stderr, "Failed to set ATT disconnect handler\n");
		goto fail;
	}

	server->name_len = name_len + 1;
	server->device_name = malloc(name_len + 1);
	if (!server->device_name) {
		fprintf(stderr, "Failed to allocate memory for device name\n");
		goto fail;
	}

	memcpy(server->device_name, test_device_name, name_len);
	server->device_name[name_len] = '\0';

	server->fd = fd;
	server->db = gatt_db_new();
	if (!server->db) {
		fprintf(stderr, "Failed to create GATT database\n");
		goto fail;
	}

	server->gatt = bt_gatt_server_new(server->db, server->att, mtu, 0);
	if (!server->gatt) {
		fprintf(stderr, "Failed to create GATT server\n");
		goto fail;
	}

	server->hr_visible = hr_visible;
	server->tune_visible = tune_visible;

	if (verbose) {
		bt_att_set_debug(server->att, att_debug_cb, "att: ", NULL);
		bt_gatt_server_set_debug(server->gatt, gatt_debug_cb,
							"server: ", NULL);
	}

	/* Random seed for generating fake Heart Rate measurements */
	srand(time(NULL));


	// Set internal state initial Tune internal values
	decimal2TuneTemperature(20 ,&server->tune_Temperature[0],&server->tune_Temperature[1]);

	// set internal state initial Voltage
	decimal2TuneVoltage(12.6, &server->tune_Voltage[0],&server->tune_Voltage[1]);

	// set internal state initial Pressure
	decimal2TunePressure(100, &server->tune_Pressure[0],&server->tune_Pressure[1]);

	// set new Advance in server Internal <static superceded by dynamic graph based code>
	decimal2TuneAdvance(10.0 ,&server->tune_Advance[0],&server->tune_Advance[1]);


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
	decimal2TuneRPM(500, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);	// 500 is fixed
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
	decimal2TuneRPM(8000, &server->tune_AdvanceCurveRPM[graph_index][0],&server->tune_AdvanceCurveRPM[graph_index][1]);	// 8000 is fixed
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
	decimal2TunePressure(0, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]);		// 0 is fixed
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
	decimal2TunePressure(100, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]);	// 100 is fixed
	decimal2TuneAdvance(0, &server->tune_MapCurveDegrees[graph_index][0],&server->tune_MapCurveDegrees[graph_index][1]);
	print_hex(server->tune_MapCurvePressure[graph_index],2);
  print_hex(server->tune_MapCurveDegrees[graph_index],2);

	graph_index++;
	decimal2TunePressure(200, &server->tune_MapCurvePressure[graph_index][0],&server->tune_MapCurvePressure[graph_index][1]);	// 200 is fixed
	decimal2TuneAdvance(0, &server->tune_MapCurveDegrees[graph_index][0],&server->tune_MapCurveDegrees[graph_index][1]);
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
	decimal2TuneRPM(1100, &server->tune_MapCurveStartRPM[0],&server->tune_MapCurveStartRPM[1]);

		// set initial internal MAX RPM
	decimal2TuneRPM(4500, &server->tune_RPMLimit[0],&server->tune_RPMLimit[1]);

	//
	server->tune_TuningMode_enabled=false;
	server->tune_TuningMode_Advance=0;
	server->tune_IMMOBILIZED=false;


	// End set internal Tune values


	/* bt_gatt_server already holds a reference */
	populate_db(server);

	return server;

fail:
	gatt_db_unref(server->db);
	free(server->device_name);
	bt_att_unref(server->att);
	free(server);

	return NULL;
}

static void server_destroy(struct server *server)
{
	timeout_remove(server->hr_timeout_id);
	bt_gatt_server_unref(server->gatt);
	gatt_db_unref(server->db);
}

static void usage(void)
{
	printf("btgatt-server\n");
	printf("Usage:\n\tbtgatt-server [options]\n");

	printf("Options:\n"
		"\t-i, --index <id>\t\tSpecify adapter index, e.g. hci0\n"
		"\t-m, --mtu <mtu>\t\t\tThe ATT MTU to use\n"
		"\t-s, --security-level <sec>\tSet security level (low|"
								"medium|high)\n"
		"\t-t, --type [random|public] \t The source address type\n"
		"\t-v, --verbose\t\t\tEnable extra logging\n"
		"\t-r, --heart-rate\t\tEnable Heart Rate service\n"
		"\t-1, --123-tune\t\tEnable 123Tune service\n"
		"\t-h, --help\t\t\tDisplay help\n");
}

static struct option main_options[] = {
	{ "index",		1, 0, 'i' },
	{ "mtu",		1, 0, 'm' },
	{ "security-level",	1, 0, 's' },
	{ "type",		1, 0, 't' },
	{ "verbose",		0, 0, 'v' },
	{ "heart-rate",		0, 0, 'r' },
	{ "123-tune",		0, 0, '1' },
	{ "help",		0, 0, 'h' },
	{ }
};

static int l2cap_le_att_listen_and_accept(bdaddr_t *src, int sec,
							uint8_t src_type)
{
	int sk, nsk;
	struct sockaddr_l2 srcaddr, addr;
	socklen_t optlen;
	struct bt_security btsec;
	char ba[18];

	sk = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
	if (sk < 0) {
		perror("Failed to create L2CAP socket");
		return -1;
	}

	/* Set up source address */
	memset(&srcaddr, 0, sizeof(srcaddr));
	srcaddr.l2_family = AF_BLUETOOTH;
	srcaddr.l2_cid = htobs(ATT_CID);
	srcaddr.l2_bdaddr_type = src_type;
	bacpy(&srcaddr.l2_bdaddr, src);

	if (bind(sk, (struct sockaddr *) &srcaddr, sizeof(srcaddr)) < 0) {
		perror("Failed to bind L2CAP socket");
		goto fail;
	}

	/* Set the security level */
	memset(&btsec, 0, sizeof(btsec));
	btsec.level = sec;
	if (setsockopt(sk, SOL_BLUETOOTH, BT_SECURITY, &btsec,
							sizeof(btsec)) != 0) {
		fprintf(stderr, "Failed to set L2CAP security level\n");
		goto fail;
	}

	if (listen(sk, 10) < 0) {
		perror("Listening on socket failed");
		goto fail;
	}

	printf("Started listening on ATT channel. Waiting for connections\n");

	memset(&addr, 0, sizeof(addr));
	optlen = sizeof(addr);
	nsk = accept(sk, (struct sockaddr *) &addr, &optlen);
	if (nsk < 0) {
		perror("Accept failed");
		goto fail;
	}

	ba2str(&addr.l2_bdaddr, ba);
	printf("Connect from %s\n", ba);
	close(sk);

	return nsk;

fail:
	close(sk);
	return -1;
}

static void notify_usage(void)
{
	printf("Usage: notify [options] <value_handle> <value>\n"
					"Options:\n"
					"\t -i, --indicate\tSend indication\n"
					"e.g.:\n"
					"\tnotify 0x0001 00 01 00\n");
}

static struct option notify_options[] = {
	{ "indicate",	0, 0, 'i' },
	{ }
};

static bool parse_args(char *str, int expected_argc,  char **argv, int *argc)
{
	char **ap;

	for (ap = argv; (*ap = strsep(&str, " \t")) != NULL;) {
		if (**ap == '\0')
			continue;

		(*argc)++;
		ap++;

		if (*argc > expected_argc)
			return false;
	}

	return true;
}

static void conf_cb(void *user_data)
{
	PRLOG("Received confirmation\n");
}

static void cmd_notify(struct server *server, char *cmd_str)
{
	int opt, i;
	char *argvbuf[516];
	char **argv = argvbuf;
	int argc = 1;
	uint16_t handle;
	char *endptr = NULL;
	int length;
	uint8_t *value = NULL;
	bool indicate = false;

	if (!parse_args(cmd_str, 514, argv + 1, &argc)) {
		printf("Too many arguments\n");
		notify_usage();
		return;
	}

	optind = 0;
	argv[0] = "notify";
	while ((opt = getopt_long(argc, argv, "+i", notify_options,
								NULL)) != -1) {
		switch (opt) {
		case 'i':
			indicate = true;
			break;
		default:
			notify_usage();
			return;
		}
	}

	argc -= optind;
	argv += optind;

	if (argc < 1) {
		notify_usage();
		return;
	}

	handle = strtol(argv[0], &endptr, 16);
	if (!endptr || *endptr != '\0' || !handle) {
		printf("Invalid handle: %s\n", argv[0]);
		return;
	}

	length = argc - 1;

	if (length > 0) {
		if (length > UINT16_MAX) {
			printf("Value too long\n");
			return;
		}

		value = malloc(length);
		if (!value) {
			printf("Failed to construct value\n");
			return;
		}

		for (i = 1; i < argc; i++) {
			if (strlen(argv[i]) != 2) {
				printf("Invalid value byte: %s\n",
								argv[i]);
				goto done;
			}

			value[i-1] = strtol(argv[i], &endptr, 16);
			if (endptr == argv[i] || *endptr != '\0'
							|| errno == ERANGE) {
				printf("Invalid value byte: %s\n",
								argv[i]);
				goto done;
			}
		}
	}

	if (indicate) {
		if (!bt_gatt_server_send_indication(server->gatt, handle,
							value, length,
							conf_cb, NULL, NULL))
			printf("Failed to initiate indication\n");
	} else if (!bt_gatt_server_send_notification(server->gatt, handle,
								value, length))
		printf("Failed to initiate notification\n");

done:
	free(value);
}

static void heart_rate_usage(void)
{
	printf("Usage: heart-rate on|off\n");
}

static void cmd_heart_rate(struct server *server, char *cmd_str)
{
	bool enable;
	uint8_t pdu[4];
	struct gatt_db_attribute *attr;

	if (!cmd_str) {
		heart_rate_usage();
		return;
	}

	if (strcmp(cmd_str, "on") == 0)
		enable = true;
	else if (strcmp(cmd_str, "off") == 0)
		enable = false;
	else {
		heart_rate_usage();
		return;
	}

	if (enable == server->hr_visible) {
		printf("Heart Rate Service already %s\n",
						enable ? "visible" : "hidden");
		return;
	}

	server->hr_visible = enable;
	attr = gatt_db_get_attribute(server->db, server->hr_handle);
	gatt_db_service_set_active(attr, server->hr_visible);
	update_hr_msrmt_simulation(server);

	if (!server->svc_chngd_enabled)
		return;

	put_le16(server->hr_handle, pdu);
	put_le16(server->hr_handle + 7, pdu + 2);

	server->hr_msrmt_enabled = false;
	update_hr_msrmt_simulation(server);

	bt_gatt_server_send_indication(server->gatt,
						server->gatt_svc_chngd_handle,
						pdu, 4, conf_cb, NULL, NULL);
}

static void print_uuid(const bt_uuid_t *uuid)
{
	char uuid_str[MAX_LEN_UUID_STR];
	bt_uuid_t uuid128;

	bt_uuid_to_uuid128(uuid, &uuid128);
	bt_uuid_to_string(&uuid128, uuid_str, sizeof(uuid_str));

	printf("%s\n", uuid_str);
}

static void print_incl(struct gatt_db_attribute *attr, void *user_data)
{
	struct server *server = user_data;
	uint16_t handle, start, end;
	struct gatt_db_attribute *service;
	bt_uuid_t uuid;

	if (!gatt_db_attribute_get_incl_data(attr, &handle, &start, &end))
		return;

	service = gatt_db_get_attribute(server->db, start);
	if (!service)
		return;

	gatt_db_attribute_get_service_uuid(service, &uuid);

	printf("\t  " COLOR_GREEN "include" COLOR_OFF " - handle: "
					"0x%04x, - start: 0x%04x, end: 0x%04x,"
					"uuid: ", handle, start, end);
	print_uuid(&uuid);
}

static void print_desc(struct gatt_db_attribute *attr, void *user_data)
{
	printf("\t\t  " COLOR_MAGENTA "descr" COLOR_OFF
					" - handle: 0x%04x, uuid: ",
					gatt_db_attribute_get_handle(attr));
	print_uuid(gatt_db_attribute_get_type(attr));
}

static void print_chrc(struct gatt_db_attribute *attr, void *user_data)
{
	uint16_t handle, value_handle;
	uint8_t properties;
	uint16_t ext_prop;
	bt_uuid_t uuid;

	if (!gatt_db_attribute_get_char_data(attr, &handle,
								&value_handle,
								&properties,
								&ext_prop,
								&uuid))
		return;

	printf("\t  " COLOR_YELLOW "charac" COLOR_OFF
				" - start: 0x%04x, value: 0x%04x, "
				"props: 0x%02x, ext_prop: 0x%04x, uuid: ",
				handle, value_handle, properties, ext_prop);
	print_uuid(&uuid);

	gatt_db_service_foreach_desc(attr, print_desc, NULL);
}

static void print_service(struct gatt_db_attribute *attr, void *user_data)
{
	struct server *server = user_data;
	uint16_t start, end;
	bool primary;
	bt_uuid_t uuid;

	if (!gatt_db_attribute_get_service_data(attr, &start, &end, &primary,
									&uuid))
		return;

	printf(COLOR_RED "service" COLOR_OFF " - start: 0x%04x, "
				"end: 0x%04x, type: %s, uuid: ",
				start, end, primary ? "primary" : "secondary");
	print_uuid(&uuid);

	gatt_db_service_foreach_incl(attr, print_incl, server);
	gatt_db_service_foreach_char(attr, print_chrc, NULL);

	printf("\n");
}

static void cmd_services(struct server *server, char *cmd_str)
{
	gatt_db_foreach_service(server->db, NULL, print_service, server);
}


static void simulation_usage(void)
{
        printf("Usage: simulation [options]\nOptions:\n"
                "\t -r, --rpm <rpm>\tNew RPM\n"
                "e.g.:\n"
                "\tsimulation -r 1500\n");
}




static void cmd_simulation(struct server *server, char *cmd_str)
{
        char *argv[11];
        int argc = 0;
        int rpm;
        int pressure;
        //int i;



        // getopt_long example: http://www.ibm.com/developerworks/aix/library/au-unix-getopt.html
        int opt = 0;
	int longIndex = 0;

        static const char *optString = "r:p:Il:o:vh?";

        static const struct option longOpts[] = {
		{ "rpm", required_argument, NULL, 'r' },
		{ "pressure", required_argument, NULL, 'p' },
        	{ "no-index", no_argument, NULL, 'I' },
		{ "language", required_argument, NULL, 'l' },
		{ "output", required_argument, NULL, 'o' },
		{ "verbose", no_argument, NULL, 'v' },
		{ "randomize", no_argument, NULL, 0 },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, no_argument, NULL, 0 }
	};



	// sanitize paramters to well known stuctures and methods. (argv/argc)
	// adapt for mismatch (+1 arg for programm name with getopt_*()
        if (!parse_args(cmd_str, 10, &argv[1], &argc)) {
                simulation_usage();
                return;
        }
        argc++;	// getopt_*() / parse_args() result  mismatch fix
        argv[0]="__FUNCTION__";	// getopt_*() mismatch fix, fill argv[0]
        optind=0;	// reset global getopt*() to force to process from the beginning

/*
printf("opt=%d argc=%d\n",opt, argc);
for(i=0;i<argc;i++) {
	printf(" argc=%d argv[%i]=%s\n",argc,i,argv[i]);
}
*/

opt = getopt_long( argc, argv, optString, longOpts, &longIndex );
//opt = getopt( argc, argv, optString );
printf("opt=%d\n",opt);


    while( opt != -1 ) {
        switch( opt ) {
            case 'r':
	                rpm=strtol(optarg,NULL,10);
        	        printf("RPM is now set to %d\n",rpm);
        	        decimal2TuneRPM(rpm,&server->tune_RPM[0],&server->tune_RPM[1]);
                break;

            case 'p':
 	               pressure=strtol(optarg,NULL,10);
 	               printf("Pressure is now set to %d\n",pressure);
 	               decimal2TunePressure(pressure ,&server->tune_Pressure[0],&server->tune_Pressure[1]);
                break;

            case 'I':
                //globalArgs.noIndex = 1; /* true */
                break;

            case 'l':
                //globalArgs.langCode = optarg;
                break;

            case 'o':
                //globalArgs.outFileName = optarg;
                break;

            case 'v':
                //globalArgs.verbosity++;
                break;

            case 'h':   /* fall-through is intentional */
            case '?':
                //display_usage();
                simulation_usage();
                break;

            case 0:     /* long option without a short arg */
                //if( strcmp( "randomize", longOpts[longIndex].name ) == 0 ) {
                //    globalArgs.randomized = 1;
                //}
                break;

            default:
                /* You won't actually get here. */
                break;
        }

        opt = getopt_long( argc, argv, optString, longOpts, &longIndex );
        //opt = getopt( argc, argv, optString );
    }


//sleep (5);
return;


        if (!strcmp(argv[0], "-r") || !strcmp(argv[0], "--rpm")) {
                rpm=strtol(argv[1],NULL,10);
                printf("RPM is now set to %d\n",rpm);
		decimal2TuneRPM(rpm,&server->tune_RPM[0],&server->tune_RPM[1]);
		return;
	}

        if (!strcmp(argv[0], "-p") || !strcmp(argv[0], "--pressure")) {
                pressure=strtol(argv[1],NULL,10);
                printf("Pressure is now set to %d\n",pressure);
		decimal2TunePressure(pressure ,&server->tune_Pressure[0],&server->tune_Pressure[1]);
		return;
	}



        if (server->tune_msrmt_enabled && server->tune_visible) {
                timeout_remove(server->tune_timeout_id);
                server->tune_msrmt_enabled=0;
        }

        //server->tune_timeout_id = timeout_add(1000, tune_msrmt_cb, server, NULL);

	printf("Disable running simulation. TODO:  stop start and custom dynamic setttings\n");
}


static bool convert_sign_key(char *optarg, uint8_t key[16])
{
	int i;

	if (strlen(optarg) != 32) {
		printf("sign-key length is invalid\n");
		return false;
	}

	for (i = 0; i < 16; i++) {
		if (sscanf(optarg + (i * 2), "%2hhx", &key[i]) != 1)
			return false;
	}

	return true;
}

static void set_sign_key_usage(void)
{
	printf("Usage: set-sign-key [options]\nOptions:\n"
		"\t -c, --sign-key <remote csrk>\tRemote CSRK\n"
		"e.g.:\n"
		"\tset-sign-key -c D8515948451FEA320DC05A2E88308188\n");
}

static bool remote_counter(uint32_t *sign_cnt, void *user_data)
{
	static uint32_t cnt = 0;

	if (*sign_cnt < cnt)
		return false;

	cnt = *sign_cnt;

	return true;
}

static void cmd_set_sign_key(struct server *server, char *cmd_str)
{
	char *argv[3];
	int argc = 0;
	uint8_t key[16];

	memset(key, 0, 16);

	if (!parse_args(cmd_str, 2, argv, &argc)) {
		set_sign_key_usage();
		return;
	}

	if (argc != 2) {
		set_sign_key_usage();
		return;
	}

	if (!strcmp(argv[0], "-c") || !strcmp(argv[0], "--sign-key")) {
		if (convert_sign_key(argv[1], key))
			bt_att_set_remote_key(server->att, key, remote_counter,
									server);
	} else
		set_sign_key_usage();
}

static void cmd_help(struct server *server, char *cmd_str);

typedef void (*command_func_t)(struct server *server, char *cmd_str);

static struct {
	char *cmd;
	command_func_t func;
	char *doc;
} command[] = {
	{ "help", cmd_help, "\tDisplay help message" },
	{ "notify", cmd_notify, "\tSend handle-value notification" },
	{ "heart-rate", cmd_heart_rate, "\tHide/Unhide Heart Rate Service" },
	{ "services", cmd_services, "\tEnumerate all services" },
	{ "simulation", cmd_simulation, "\tStop or start simulation and adjust RPM and presure" },
	{ "set-sign-key", cmd_set_sign_key,
			"\tSet remote signing key for signed write command"},
	{ }
};

static void cmd_help(struct server *server, char *cmd_str)
{
	int i;

	printf("Commands:\n");
	for (i = 0; command[i].cmd; i++)
		printf("\t%-15s\t%s\n", command[i].cmd, command[i].doc);
}

static void prompt_read_cb(int fd, uint32_t events, void *user_data)
{
	ssize_t read;
	size_t len = 0;
	char *line = NULL;
	char *cmd = NULL, *args;
	struct server *server = user_data;
	int i;

	if (events & (EPOLLRDHUP | EPOLLHUP | EPOLLERR)) {
		mainloop_quit();
		return;
	}

	read = getline(&line, &len, stdin);
	if (read < 0)
		return;

	if (read <= 1) {
		cmd_help(server, NULL);
		print_prompt();
		return;
	}

	line[read-1] = '\0';
	args = line;

	while ((cmd = strsep(&args, " \t")))
		if (*cmd != '\0')
			break;

	if (!cmd)
		goto failed;

	for (i = 0; command[i].cmd; i++) {
		if (strcmp(command[i].cmd, cmd) == 0)
			break;
	}

	if (command[i].cmd)
		command[i].func(server, args);
	else
		fprintf(stderr, "Unknown command: %s\n", line);

failed:
	print_prompt();

	free(line);
}

static void signal_cb(int signum, void *user_data)
{
	switch (signum) {
	case SIGINT:
	case SIGTERM:
		mainloop_quit();
		break;
	default:
		break;
	}
}

int main(int argc, char *argv[])
{
	int opt;
	bdaddr_t src_addr;
	int dev_id = -1;
	int fd;
	int sec = BT_SECURITY_LOW;
	uint8_t src_type = BDADDR_LE_PUBLIC;
	uint16_t mtu = 0;
	sigset_t mask;
	bool hr_visible = false;
	bool tune_visible = false;

	struct server *server;

	while ((opt = getopt_long(argc, argv, "+hvr1s:t:m:i:",
						main_options, NULL)) != -1) {
		switch (opt) {
		case 'h':
			usage();
			return EXIT_SUCCESS;
		case 'v':
			verbose = true;
			break;
		case 'r':
			hr_visible = true;
			break;
		case '1':	// 123 TUne
			tune_visible = true;
			//tune_visible =true ;
			break;
		case 's':
			if (strcmp(optarg, "low") == 0)
				sec = BT_SECURITY_LOW;
			else if (strcmp(optarg, "medium") == 0)
				sec = BT_SECURITY_MEDIUM;
			else if (strcmp(optarg, "high") == 0)
				sec = BT_SECURITY_HIGH;
			else {
				fprintf(stderr, "Invalid security level\n");
				return EXIT_FAILURE;
			}
			break;
		case 't':
			if (strcmp(optarg, "random") == 0)
				src_type = BDADDR_LE_RANDOM;
			else if (strcmp(optarg, "public") == 0)
				src_type = BDADDR_LE_PUBLIC;
			else {
				fprintf(stderr,
					"Allowed types: random, public\n");
				return EXIT_FAILURE;
			}
			break;
		case 'm': {
			int arg;

			arg = atoi(optarg);
			if (arg <= 0) {
				fprintf(stderr, "Invalid MTU: %d\n", arg);
				return EXIT_FAILURE;
			}

			if (arg > UINT16_MAX) {
				fprintf(stderr, "MTU too large: %d\n", arg);
				return EXIT_FAILURE;
			}

			mtu = (uint16_t) arg;
			break;
		}
		case 'i':
			dev_id = hci_devid(optarg);
			if (dev_id < 0) {
				perror("Invalid adapter");
				return EXIT_FAILURE;
			}

			break;
		default:
			fprintf(stderr, "Invalid option: %c\n", opt);
			return EXIT_FAILURE;
		}
	}

	argc -= optind;
	argv -= optind;
	optind = 0;

	if (argc) {
		usage();
		return EXIT_SUCCESS;
	}

	if (dev_id == -1)
		bacpy(&src_addr, BDADDR_ANY);
	else if (hci_devba(dev_id, &src_addr) < 0) {
		perror("Adapter not available");
		return EXIT_FAILURE;
	}

	fd = l2cap_le_att_listen_and_accept(&src_addr, sec, src_type);
	if (fd < 0) {
		fprintf(stderr, "Failed to accept L2CAP ATT connection\n");
		return EXIT_FAILURE;
	}

	mainloop_init();

	server = server_create(fd, mtu, hr_visible, tune_visible);

	if (!server) {
		close(fd);
		return EXIT_FAILURE;
	}

	if (mainloop_add_fd(fileno(stdin),
				EPOLLIN | EPOLLRDHUP | EPOLLHUP | EPOLLERR,
				prompt_read_cb, server, NULL) < 0) {
		fprintf(stderr, "Failed to initialize console\n");
		server_destroy(server);

		return EXIT_FAILURE;
	}

	printf("Running GATT server\n");

	sigemptyset(&mask);
	sigaddset(&mask, SIGINT);
	sigaddset(&mask, SIGTERM);

	mainloop_set_signal(&mask, signal_cb, NULL, NULL);

	print_prompt();

	mainloop_run();

	printf("\n\nShutting down...\n");

	server_destroy(server);

	return EXIT_SUCCESS;
}
