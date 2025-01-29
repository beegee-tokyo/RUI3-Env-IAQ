/**
 * @file RUI3-EPD-Clock.ino
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief RUI3 IAQ sensor based on Bosch BME680
 * @version 0.1
 * @date 2025-01-29
 *
 * @copyright Copyright (c) 2024
 *
 */

/*******************************************************************************************************************/
/** License for the BME680 IAQ algorithm                                                                           */
/** Origin from https://github.com/G6EJD/BME680-Example                                                            */
/*******************************************************************************************************************/
/*
 This software, the ideas and concepts is Copyright (c) David Bird 2018. All rights to this software are reserved.

 Any redistribution or reproduction of any part or all of the contents in any form is prohibited other than the following:
 1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
 2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird as the source of the material.
 3. You may not, except with my express written permission, distribute or commercially exploit the content.
 4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.

 The above copyright ('as annotated') notice and this permission notice shall be included in all copies or substantial portions of the Software and where the
 software use is visible to an end-user.

 THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY
 OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 See more at http://www.dsbird.org.uk
*/

#include "app.h"

/** Packet is confirmed/unconfirmed (Set with AT commands) */
bool g_confirmed_mode = false;
/** If confirmed packet, number or retries (Set with AT commands) */
uint8_t g_confirmed_retry = 0;
/** Data rate  (Set with AT commands) */
uint8_t g_data_rate = 3;
/** Enable/disable CAD (Set with AT commands) */
bool g_use_cad = false;

/** Time interval to send packets in milliseconds */
uint32_t g_send_repeat_time = 60000;

/** Flag if transmit is active, used by some sensors */
volatile bool tx_active = false;

/** fPort to send packages */
uint8_t set_fPort = 2;

/** LoRaWAN packet */
WisCayenne g_solution_data(255);

uint8_t sync_time_status = 0;

/**
 * @brief Callback after join request cycle
 *
 * @param status Join result
 */
void joinCallback(int32_t status)
{
	MYLOG("JOIN-CB", "Join result %d", status);
	if (status != 0)
	{
		// MYLOG("JOIN-CB", "LoRaWan OTAA - join fail! \r\n");
	}
	else
	{
		// MYLOG("JOIN-CB", "LoRaWan OTAA - joined! \r\n");
		digitalWrite(LED_BLUE, LOW);
	}
}

/**
 * @brief LoRaWAN callback after packet was received
 *
 * @param data pointer to structure with the received data
 */
void receiveCallback(SERVICE_LORA_RECEIVE_T *data)
{
// 	MYLOG("RX-CB", "RX, port %d, DR %d, RSSI %d, SNR %d", data->Port, data->RxDatarate, data->Rssi, data->Snr);
// #if MY_DEBUG > 0
// 	for (int i = 0; i < data->BufferSize; i++)
// 	{
// 		Serial.printf("%02X", data->Buffer[i]);
// 	}
// 	Serial.print("\r\n");
// #endif
	tx_active = false;
}

/**
 * @brief Callback for LinkCheck result
 *
 * @param data pointer to structure with the linkcheck result
 */
void linkcheckCallback(SERVICE_LORA_LINKCHECK_T *data)
{
	// MYLOG("LC_CB", "%s Margin %d GW# %d RSSI%d SNR %d", data->State == 0 ? "OK" : "NOK",
	// 	  data->DemodMargin, data->NbGateways,
	// 	  data->Rssi, data->Snr);
}

/**
 * @brief Request network time
 *
 * @param status 0 = got time, otherwise failed
 */
void timeReqCallback(int32_t status)
{
	// MYLOG("TREQ", "Time request status %d", status);
	if (sync_time_status == 0)
	{
		sync_time_status = 1;
	}
}

/**
 * @brief LoRaWAN callback after TX is finished
 *
 * @param status TX status
 */
void sendCallback(int32_t status)
{
	// MYLOG("TX-CB", "TX status %d", status);
	digitalWrite(LED_BLUE, LOW);
	tx_active = false;
}

/**
 * @brief Arduino setup, called once after reboot/power-up
 *
 */
void setup()
{
	// Setup callbacks
	g_confirmed_mode = api.lorawan.cfm.get();

	g_confirmed_retry = api.lorawan.rety.get();

	g_data_rate = api.lorawan.dr.get();

	// Setup the callbacks for joined and send finished
	api.lorawan.registerRecvCallback(receiveCallback);
	api.lorawan.registerSendCallback(sendCallback);
	api.lorawan.registerJoinCallback(joinCallback);
	api.lorawan.registerLinkCheckCallback(linkcheckCallback);
	api.lorawan.registerTimereqCallback(timeReqCallback);

	pinMode(LED_GREEN, OUTPUT);
	digitalWrite(LED_GREEN, HIGH);
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, HIGH);

	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, LOW);

	// Start Serial
	Serial.begin(115200);

	// Delay for 5 seconds to give the chance for AT+BOOT
	delay(5000);

	api.system.firmwareVersion.set("RUI3-IAQ-V1.0.0");

	Serial.println("RAKwireless RUI3 Node");
	Serial.println("------------------------------------------------------");
	Serial.println("Setup the device with WisToolBox or AT commands before using it");
	Serial.printf("Version %s\n", api.system.firmwareVersion.get().c_str());
	Serial.println("------------------------------------------------------");

	// Initialize module
	delay(250);
	Wire.begin();

	// Register the custom AT command to get device status
	delay(250);
	if (!init_status_at())
	{
		MYLOG("SETUP", "Add custom AT command STATUS fail");
	}

	// Register the custom AT command to set the send interval
	delay(250);
	if (!init_interval_at())
	{
		MYLOG("SETUP", "Add custom AT command Send Interval fail");
	}

	// Initialize BME680
	has_rak1906 = init_rak1906();

	if (has_rak1906)
	{
		// Register the custom AT command to set the IAQ readings
		MYLOG("SETUP", "Init custom AT command IAQ");
		delay(250);
		if (!init_iaq_interval_at())
		{
			MYLOG("SETUP", "Add custom AT command IAQ fail");
		}
	}

	// Get saved sending interval from flash
	get_at_setting();

	digitalWrite(LED_GREEN, LOW);

	api.lorawan.join(1, 1, 30, 10);

	// Initialize timers
	if (has_rak1906)
	{
		api.system.timer.create(RAK_TIMER_0, read_bme680, RAK_TIMER_PERIODIC);
		if (custom_parameters.iaq_interval != 0)
		{
			// Start a timer.
			api.system.timer.start(RAK_TIMER_0, custom_parameters.iaq_interval, NULL);
		}
	}

	api.system.timer.create(RAK_TIMER_1, sensor_handler, RAK_TIMER_PERIODIC);
	if (custom_parameters.send_interval != 0)
	{
		// Start a timer.
		api.system.timer.start(RAK_TIMER_1, custom_parameters.send_interval, NULL);
	}

	// Enable Timerequest
	api.lorawan.timereq.set(1);

	// Enable low power mode
	api.system.lpm.set(1);

#if defined(_VARIANT_RAK3172_) || defined(_VARIANT_RAK3172_SIP_)
// No BLE
#else
	Serial6.begin(115200, RAK_AT_MODE);
	api.ble.advertise.start(30);
#endif

	// If already joined, send a first packet
	if (api.lorawan.njs.get())
	{
		// Enable Timerequest
		// api.lorawan.timereq.set(1);

		// Clear payload
		g_solution_data.reset();

		// Create payload
		// Add battery voltage
		g_solution_data.addVoltage(LPP_CHANNEL_BATT, api.system.bat.get());

		send_packet();
	}
}

/**
 * @brief sensor_handler is a timer function called every
 * g_send_repeat_time milliseconds.
 *
 */
void sensor_handler(void *)
{
	// MYLOG("UPLINK", "Start");
	digitalWrite(LED_BLUE, HIGH);

	if (api.lorawan.nwm.get() == 1)
	{
		// Check if the node has joined the network
		if (!api.lorawan.njs.get())
		{
			// MYLOG("UPLINK", "Skip");
			return;
		}
	}

	// Clear payload
	g_solution_data.reset();

	// Create payload
	// Add battery voltage
	g_solution_data.addVoltage(LPP_CHANNEL_BATT, api.system.bat.get());
	// Add BME680 data
	g_solution_data.addTemperature(LPP_CHANNEL_TEMP_2, g_last_temperature);
	g_solution_data.addRelativeHumidity(LPP_CHANNEL_HUMID_2, g_last_humidity);
	g_solution_data.addBarometricPressure(LPP_CHANNEL_PRESS_2, g_last_pressure);
	g_solution_data.addVoc_index(LPP_CHANNEL_GAS_2, (uint32_t)g_last_iaq);

	// Send the packet
	send_packet();
}

/**
 * @brief This example is complete timer
 * driven. The loop() does nothing than
 * sleep.
 *
 */
void loop()
{
	// Sleep always
	api.system.sleep.all();
}

/**
 * @brief Send the data packet that was prepared in
 * Cayenne LPP format by the different sensor and location
 * aqcuision functions
 *
 */
void send_packet(void)
{
	// MYLOG("UPLINK", "Sending %d bytes", g_solution_data.getSize());

#if MY_DEBUG > 0
	uint8_t *packet_buffer = g_solution_data.getBuffer();

	for (int i = 0; i < g_solution_data.getSize(); i++)
	{
		Serial.printf("%02X", packet_buffer[i]);
	}
	Serial.print("\r\n");
#endif

	// Enable Timerequest
	api.lorawan.timereq.set(1);

	// Send the packet
	if (api.lorawan.send(g_solution_data.getSize(), g_solution_data.getBuffer(), set_fPort, g_confirmed_mode, g_confirmed_retry))
	{
		// MYLOG("UPLINK", "LPW Packet enqueued");
		tx_active = true;
	}
	else
	{
		// MYLOG("UPLINK", "Send failed");
		tx_active = false;
	}
}