/**
 * @file wisblock_cayenne.h
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Extend CayenneLPP class with custom channels
 * @version 0.1
 * @date 2022-04-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef WISBLOCK_CAYENNE_H
#define WISBLOCK_CAYENNE_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <CayenneLPP.h>

#define LPP_GPS4 136  // 3 byte lon/lat 0.0001 °, 3 bytes alt 0.01 meter (Cayenne LPP default)
#define LPP_GPS6 137  // 4 byte lon/lat 0.000001 °, 3 bytes alt 0.01 meter (Customized Cayenne LPP, higher precision)
#define LPP_VOC 138	  // 2 byte VOC index
#define WB_DEV_ID 255 // 4 byte device ID (used for P2P communication)

// Only Data Size
#define LPP_GPS4_SIZE 9
#define LPP_GPS6_SIZE 11
#define LPP_GPSH_SIZE 14
#define LPP_VOC_SIZE 2
#define WB_DEV_ID_SIZE 4

// Cayenne LPP Channel numbers per sensor value used in WisBlock API examples
#define LPP_CHANNEL_DEVID 0 // Device ID, only used in LoRa P2P

class WisCayenne : public CayenneLPP
{
public:
	WisCayenne(uint8_t size) : CayenneLPP(size) {}

	uint8_t addGNSS_4(uint8_t channel, int32_t latitude, int32_t longitude, int32_t altitude);
	uint8_t addGNSS_6(uint8_t channel, int32_t latitude, int32_t longitude, int32_t altitude);
	uint8_t addGNSS_H(int32_t latitude, int32_t longitude, int16_t altitude, int16_t accuracy, int16_t battery);
	uint8_t addVoc_index(uint8_t channel, uint32_t voc_index);
	uint8_t addDevID(uint8_t channel, uint8_t *dev_id);

private:
};
#endif