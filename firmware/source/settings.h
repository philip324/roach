/*
 * Settings
 *
 * created on 2012-5-14 by fgb (derived from or_const.h by apullin)
 */

#ifndef __SETTINGS_H
#define __SETTINGS_H

/////// Radio settings ///////
#define RADIO_CHANNEL		0x19
#define RADIO_SRC_ADDR 		0x2052
#define RADIO_PAN_ID        0x2050
//Hard-coded destination address, must match basestation or XBee addr
#define RADIO_DST_ADDR		0x2051


#define RADIO_TXPQ_MAX_SIZE   10
#define RADIO_RXPQ_MAX_SIZE   10

// Telemetry type setup
#define TELEM_TYPE vrTelemStruct_t
#define TELEM_INCLUDE "vr_telem.h"
#define TELEMPACKFUNC(x) vrTelemGetData(x)

// Encoders Setup
//Left legs
#define LEFT_LEGS_PID_NUM       0       //PID module index is 0-3
#define LEFT_LEGS_ENC_NUM       1       //amsEnc module index is 0-3
#define AMS_ENC_OFFSET_1        7706
#define LEFT_LEG_FLIP           1       //"forward" reversed for left
#define LEFT_LEGS_TIH_CHAN      1       //tiH module index is 1-4
//Right legs
#define RIGHT_LEGS_PID_NUM      1       //PID module index is 0-3
#define RIGHT_LEGS_ENC_NUM      0       //amsEnc module index is 0-3
#define AMS_ENC_OFFSET_0        5758
#define RIGHT_LEG_FLIP          0       //"forward" regular for right
#define RIGHT_LEGS_TIH_CHAN     2       //tiH module index is 1-4

#endif //_SETTINGS_H