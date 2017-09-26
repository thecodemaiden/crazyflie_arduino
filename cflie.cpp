#include "cflie.h"
#include "cflie_log.h"
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

#include <cstdio>

Crazyflie::Crazyflie(RF24 *extRadio, uint64_t radioAddr, uint8_t pipeNum)
	: radio(extRadio), _busy(0), _logReady(false), _logBlockReady(false),
      _logInfo(), _addrLong(radioAddr), _pipeNum(pipeNum),  _keepAlive(false)
{
    _setThrust= _setPitch = _setYaw = _setRoll = 0;

    _logStorage = new LogStorage();
}

Crazyflie::~Crazyflie()
{
    delete _logStorage;
}

void Crazyflie::setCommanderInterval(uint16_t msInt)
{
    if (msInt < 50) msInt = 50;
    if (msInt > 500) msInt = 500;
	_commanderInterval = msInt;
}

void Crazyflie::setCommanderSetpoint(float pitch, float roll, float yaw, uint16_t thrust)
{
	_setThrust = thrust;
	_setPitch = pitch;
	_setRoll = roll;
	_setYaw = yaw;
	//_lastCommanderTime = millis();
}

void Crazyflie::prepareCommanderPacket()
{
	if (_busy) return; // TODO: rly?
	_busy |= BUSY_COMMANDER;
    cl_commander c = {0};
	memset(_outgoing, 0, 32);

	_outgoing[0]= (PORT_COMMANDER & 0xF) << 4 | 0x3 << 2;

	c.roll = _setRoll;
	c.pitch = _setPitch;
	c.yaw = _setYaw;
	c.thrust = _setThrust;
	_outPacketLen = 15;
    c.pack(_outgoing+1);
	send_state = COMMANDER;
}

void Crazyflie::initLogSystem()
{
	if (_busy) return;
	_busy |= BUSY_LOG_TOC;
	// prepare the toc info packet
    cl_toc_pkt p = {0};
	memset(_outgoing, 0, 32);
	_outgoing[0] = (PORT_LOGGING & 0xF) << 4 | 0x1 << 2 | (CHANNEL_TOC & 0x3);

	p.command = CMD_GET_INFO;
	_outPacketLen = 2;
    p.pack(_outgoing+1);
	send_state = LOG_TOC;
}

void Crazyflie::requestRSSILog()
{
    if (_busy) return;
    _busy = _busy | BUSY_LOG_DATA;
    cl_log_settings pkt = {0};
    pkt.command = CONTROL_CREATE_BLOCK;
    pkt.blockID=1;
    pkt.variables[0].log_type = LOG_UINT8;
    pkt.variables[0].varID = 51;

    _outgoing[0] = (PORT_LOGGING & 0xF) << 4 | 0x3 << 2| (CHANNEL_SETTINGS & 0x3);
    pkt.pack(_outgoing+1);

    send_state = LOG_DATA;
    _outPacketLen = 5;
}

void Crazyflie::stopRSSILog()
{
    if (_busy) return;
    _busy = _busy | BUSY_LOG_DATA;
    cl_log_settings pkt = {0};
    pkt.command = CONTROL_STOP_BLOCK;
    pkt.blockID=1;
    _outgoing[0] = (PORT_LOGGING & 0xF) << 4 | 0x3 << 2| (CHANNEL_SETTINGS & 0x3);
    pkt.pack(_outgoing+1);
    send_state = LOG_DATA;
    _outPacketLen = 3;
}

void Crazyflie::startRSSILog()
{
    if (_busy) return;
    _busy = _busy | BUSY_LOG_DATA;
    cl_log_settings pkt = {0};
    pkt.command = CONTROL_START_BLOCK;
    pkt.blockID=1;
    pkt.period = 7;

    _outgoing[0] = (PORT_LOGGING & 0xF) << 4 | 0x3 << 2| (CHANNEL_SETTINGS & 0x3);
    pkt.pack(_outgoing+1);

    send_state = LOG_DATA;
    _outPacketLen = 4;
}

void Crazyflie::prepareDummyPacket()
{
    if (_busy) return;
   memset(_outgoing, 0, 32);
   _outgoing[0] = 0xFF;
   send_state = DUMMY;
   _outPacketLen = 1;
}


bool Crazyflie::hasLogInfo()
{
	return _logReady;
}

bool Crazyflie::isBusy()
{
	return (bool)_busy;
}

void Crazyflie::sendAndReceive(uint32_t timeout)
{

	// is the commander on?
    unsigned long now = millis();
    if (_keepAlive && (_lastCommanderTime + _commanderInterval < now)){
	prepareCommanderPacket();
	_lastCommanderTime = now;
    } else if (!_busy) {
        prepareDummyPacket();
    }

	radio->stopListening();

    radio->openWritingPipe(_addrLong);
	// payload should already be set up
	// send the packet. Blocks until sent
    radio->write(_outgoing, _outPacketLen);
	// unset commander flag if set - will be reset next loop if keepalive is on
	if ((_busy & BUSY_COMMANDER)) {
		_busy &= ~BUSY_COMMANDER;
	}

    //radio->openReadingPipe(_pipeNum, _addrLong);
	// start listening for an ACK
	radio->startListening();
	// Wait here until we get all responses, or timeout
	bool didTimeout = false;
	unsigned long start = millis();
	while (!radio->available() && !didTimeout)
	{
		if (millis() - start > timeout)
			didTimeout = true;
	}

	if (didTimeout)
	{
		debug("response timed out\n");
	}
	else
	{
		// clear _incoming packet
		memset(_incoming, 0, 32);

		// read response
		_inPacketLen = radio->getDynamicPayloadSize();
		radio->read(_incoming, _inPacketLen);

		dispatchPacket();
	}

}

void Crazyflie::handleTocPacket()
{
    // first assume it's a crc packet
    uint8_t command = _incoming[1];
    switch (command) {
        case CMD_GET_INFO:
            {
                // it is CRC
                cf_toc_crc_pkt p;

                p.unpack(_incoming+1);

                if (p.crc != _logInfo.crc || p.num != _logInfo.num) {
                    _logInfo.unpack(_incoming+1);
                    send_state = LOG_TOC;
                    _itemToFetch = 0;
                    requestNextTOCItem();
                    printf("[%d] TOC size: %d\n", _pipeNum, _logInfo.num); 
                }   

                break;
            }
        case CMD_GET_ITEM:
            // actually it was a toc item request
            {
                if (_logInfo.num == 0)
                    break;

                cf_toc_item_pkt p;
                p.unpack(_incoming+1);

                uint8_t fetchedItem = p.index;
                send_state = LOG_TOC;
                if (fetchedItem < _itemToFetch) {
                    send_state = DUMMY;
                }
                else
                    if (fetchedItem == _itemToFetch) {
                        uint8_t groupEnd = 0;
                        while (p.varName[groupEnd] != '\0') groupEnd++;
                        p.varName[groupEnd] = '.';
                        debug("Got variable %s, type %x \n", p.varName, p.varType);
                        _logStorage->setVariable(fetchedItem, (LogVarType)p.varType, p.varName);
                        _itemToFetch += 1;
			//printf("Copter %d on log item %d\n",_pipeNum,_itemToFetch);
                        send_state = LOG_TOC;
                    }
                if (_itemToFetch >= _logInfo.num) {
                    _busy &= ~BUSY_LOG_TOC;
                    _logReady = true;
                    send_state = DUMMY;
                }
                else {
                    requestNextTOCItem();
                    send_state = LOG_TOC;
                }
                uint8_t groupEnd = 0;
                while (p.varName[groupEnd] != '\0') groupEnd++;

                break;
            }
        default:
            break;
    }
}

void Crazyflie::handleLogBlockPacket()
{
    uint8_t command = _incoming[1];
    uint8_t retStatus = _incoming[3];
    _busy &= ~BUSY_LOG_DATA;   
    send_state = DUMMY;
    switch (command) {
        case CONTROL_CREATE_BLOCK:
            {
                // ignoring block id for now
                if (retStatus == 0 || retStatus == 0x11) {
                    // created or existing
                    _logBlockReady = true;
                    startRSSILog();
                } break;
            }
        case CONTROL_START_BLOCK:
            {
                break;
            }
        default:
            printf("Unknown log settings command: %x\n", command);
            break;
    }
    if (retStatus != 0) {
        printf("Error starting log block: %x\n", retStatus);
    }

}

void Crazyflie::handleLogDataPacket()
{
    cf_log_data pkt = {0};
    pkt.unpack(_incoming+1);
    printf("[%08d] %x\n", pkt.timestamp, pkt.values[0]);
}

void Crazyflie::dispatchPacket()
{
	uint8_t port = (_incoming[0] >> 4);
	uint8_t channel = (_incoming[0] & 0x3);
    bool handled = false;
	if (port == PORT_LOGGING) {
        if  (channel == CHANNEL_TOC) {
            handleTocPacket();
            handled = true;
        }
        if (channel == CHANNEL_SETTINGS) {
            handleLogBlockPacket();
            handled = true;
        }
        if (channel == CHANNEL_DATA) {
            handleLogDataPacket();
            handled = true;
        }
    } else if (port == 0xf && channel == 0x3) {
        _lastRSSI = _incoming[2];
        //printf("[%02d] RSSI: %02X\n", _pipeNum, _lastRSSI);
        handled = true;
    } else if (port == 0x2 && channel == 0x3) {
		handled = true;
		printIncomingPacket();
		_busy = _busy & (~BUSY_PARAMETER);
		// RESPONSE OF PARAM SET
		char *gname = (char *)(_incoming+2);
		char *pname = (char *)(_incoming+9);
		uint8_t errCode = (uint8_t)(_incoming[15]);
		printf("PARAM %s.%s: %d\n", gname, pname, errCode);
	}

    if (!handled) printf("Unknown packet type %02x%02x\n", port, channel);
}

void Crazyflie::requestNextTOCItem()
{
	// prepare the packet
	debug("Preparing TOC item request: %d", _itemToFetch);

	memset(_outgoing, 0, 32);
	_outgoing[0] = (PORT_LOGGING & 0xF) << 4 | 3 << 2 | (CHANNEL_TOC & 0x3);
    cl_toc_pkt p = {0};
	p.command = CMD_GET_ITEM;
	p.index = _itemToFetch;
    p.pack(_outgoing+1);
	_outPacketLen = 3;
}

const LogVariable *Crazyflie::getLogVariable(unsigned int varID)
{
    return _logStorage->getVariable(varID);
}

unsigned int Crazyflie::getLogTocSize()
{
    return _logStorage->getSize();
}

void Crazyflie::startCommander()
{
	_keepAlive = true;
}

void Crazyflie::stopCommander()
{
	_keepAlive = false;
}

void Crazyflie::setMotorFrequency(uint8_t mNum, uint16_t freq)
{
// use the set-by-name parameter function for now
if (!_busy) {
    _busy |= BUSY_PARAMETER;
	memset(_outgoing,0,32);
	// PORT 2, channel 3
	_outgoing[0] = (0x2<<4)|(0x3);
	_outgoing[1] = 0x00;
	strncpy((char *)(_outgoing+2), "mtrsnd", 6);
	strncpy((char *)(_outgoing+9), "freq", 4);
	char lastChar = '0' + mNum;
	_outgoing[13] = lastChar;
	_outgoing[15] = 0x09; //uint16_t...
	// finally the value
    memcpy(_outgoing+16, &freq, 2);
    _outPacketLen = 18;
    printOutgoingPacket();
}

}


void Crazyflie::printOutgoingPacket()
{
    printf("Outgoing length: %d\n", _outPacketLen);    
    for (int i = 0; i < _outPacketLen; i++) {
        printf("%02x ", _outgoing[i]);
    }
    printf("\n");
}

void Crazyflie::printIncomingPacket()
{
    printf("Incoming length: %d\n", _inPacketLen);    

    for (int i = 0; i < _inPacketLen; i++) {
        printf("%02x ", _incoming[i]);
    }
    printf("\n");
}
