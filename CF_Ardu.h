#ifndef __CF_ARDU_H__
#define __CF_ARDU_H__

/***
The MIT License (MIT)
Copyright (c) <2016> <Adeola Bannis>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation 
files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, 
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished 
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

***/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


#ifdef CF_DEBUG
#define debug(x) Serial.print(x)
#define debug_hex(x) Serial.print(x, HEX)
#define debugln(x) Serial.println(x)
#define debugln_hex(x) Serial.println(x, HEX)
#else
#define debug(x)  /* pass */
#define debugln(x) /* pass*/
#define debug_hex(x)  /* pass */
#define debugln_hex(x) /* pass */
#endif

#include "CF_Ardu_Packets.h"

//#define USE_EXT_RADIO

// TODO: convert to enums
// --- commands ---
// toc
#define CMD_GET_ITEM 0
#define CMD_GET_INFO 1

// settings
#define CONTROL_CREATE_BLOCK 0
#define CONTROL_APPEND_BLOCK 1
#define CONTROL_DELETE_BLOCK 2
#define CONTROL_START_BLOCK 3
#define CONTROL_STOP_BLOCK 4
#define CONTROL_RESET 5

// --- LOG VARIABLE TYPES ----
// TODO

// --- PORTS ---
#define PORT_LOGGING 5    // Port to send log requests
#define PORT_COMMANDER 3 // port for flight commands
#define PORT_PARAMETER 2 // for changing parameters

// for log only TODO: other ports
#define CHANNEL_TOC 0
#define CHANNEL_SETTINGS 1
#define CHANNEL_DATA 2

typedef enum {
	DUMMY = 0,
	BUFFER,
} cf_send_state;

class CF_Ardu {
public:
	//TODO: change to bool to indicate busyness
	CF_Ardu(uint8_t cePin, uint8_t csPin, long long radioAddr = 0xE7E7E7E7E7LL, uint8_t radioChannel = 0x50, rf24_datarate_e rfDataRate = RF24_250KBPS);
	void startRadio();
	void stopRadio();
	void setCommanderInterval(uint8_t _msInt);
	void setCommanderSetpoint(float pitch, float roll, float yaw, uint16_t thrust);
	void initLogSystem();
	void sendAndReceive(long timeout=50);
	void printOutgoingPacket();
	void printIncomingPacket();
#ifdef USE_EXT_RADIO
	RF24 *extRadio;
#endif
private:

	cf_send_state send_state;
#ifndef USE_EXT_RADIO
	RF24 radio; // internal radio object
#endif

	void dispatchPacket(uint8_t len); // parse packets after receiving
	void requestNextTOCItem();

	// for radio sending
	uint8_t _outPacketLen;
	cl_packet outgoing;
	uint8_t _inPacketLen;
	cf_packet incoming;

	// we will ignore other requests while we fetch the log info
	bool _logBusy;

	// have we gotten the TOC and the full list?
	bool _logReady; 
	uint8_t _itemToFetch;
	cf_toc_crc_pkt _logInfo; // keep this packet around

	//not sure I need to save these:
	uint8_t _cePin;
	uint8_t _csPin;
	uint8_t _addrShort;
	long long _addrLong;
	uint8_t _channel;
	rf24_datarate_e _dataRate;

	// for keeping communication alive
	uint8_t _commanderInterval;
	uint8_t _setThrust;
	uint8_t _setPitch;
	uint8_t _setRoll;
	uint8_t _setYaw;

};
#endif