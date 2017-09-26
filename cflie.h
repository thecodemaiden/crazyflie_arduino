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
#include <RF24/RF24.h>

//#define CF_DEBUG 

#ifdef CF_DEBUG
    #define debug(...) printf(__VA_ARGS__)
#else
#define debug(...)  /* pass */
#endif

#include "cflie_packets.h"
class LogStorage;
struct LogVariable;

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

//#define USE_EXT_RADIO

typedef enum {
	DUMMY = 0,
	LOG_TOC,
	LOG_DATA,
	PARAM_SET,
	COMMANDER
} cf_send_state;

// assume all CFs have the same log params...
struct toc_item {
	uint8_t type;
	char name[28];
	uint32_t value; // 400 extra bytes won't kill me
};

// Busy reasons
#define BUSY_LOG_TOC (1<<0)
#define BUSY_LOG_DATA (1<<1)
#define BUSY_COMMANDER (1<<2)
#define BUSY_PARAMETER (1<<3)


class Crazyflie {
public:
	Crazyflie(RF24 *radio, uint64_t radioAddr=0xE7E7E7E7E7L, uint8_t pipeNum=1);

	void setCommanderInterval(uint16_t msInt); // clamps to <= 500ms
	void setCommanderSetpoint(float pitch, float roll, float yaw, uint16_t thrust);
	
	void initLogSystem();
    void requestRSSILog();
    void startRSSILog();
    void stopRSSILog();

	void sendAndReceive(uint32_t timeout=50);

    uint8_t getLastRSSI();

	// for debugging
	void printOutgoingPacket();
	void printIncomingPacket();

	void startCommander(); // sends commander packets at intervals to keep connection alive
	void stopCommander(); // stop sending commander packets

	bool isBusy();
	bool hasLogInfo();

    unsigned int getLogTocSize();
    const LogVariable *getLogVariable(unsigned int varID);
    void setMotorFrequency(uint8_t mNum, uint16_t freq);

    ~Crazyflie();

private:

	cf_send_state send_state;
	RF24 *radio; // internal radio object

	void dispatchPacket(); // parse packets after receiving
    void handleTocPacket();
    void handleLogBlockPacket();
    void handleLogDataPacket();
	void requestNextTOCItem();

	void prepareCommanderPacket();
    void prepareDummyPacket();

	// for keeping communication alive
	uint16_t _commanderInterval;
    unsigned long _lastCommanderTime;

	// for radio sending
	uint8_t _outPacketLen;
	uint8_t _outgoing[32];
	uint8_t _inPacketLen;
	uint8_t _incoming[32];

	// we will ignore other requests while we fetch the log info
	uint8_t _busy;

	// have we gotten the TOC and the full list?
	bool _logReady; 
    bool _logBlockReady;
	uint8_t _itemToFetch;
	cf_toc_crc_pkt _logInfo; // keep this packet around
    LogStorage *_logStorage;

    // from raw ack packet
    uint8_t _lastRSSI;

	uint64_t _addrLong;
    uint8_t _pipeNum;

	uint16_t _setThrust;
	uint8_t _setPitch;
	uint8_t _setRoll;
	uint8_t _setYaw;
	bool _keepAlive;

};
#endif
