#include "CF_Ardu.h"
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

CF_Ardu::CF_Ardu(uint8_t cePin, uint8_t csPin, uint64_t radioAddr, uint8_t radioChannel, rf24_datarate_e rfDataRate)
	:_cePin(cePin), _csPin(csPin), _addrLong(radioAddr), _channel(radioChannel), _dataRate(rfDataRate)
#ifndef USE_EXT_RADIO
	, radio(cePin, csPin)
#endif
{
	_busy = 0;
	_keepAlive = false;
	_logReady = false;
	_logInfo.num = 0;
	_addrShort = 0x00; // what's this??

}
void CF_Ardu::startRadio()
{
#ifdef USE_EXT_RADIO
	RF24 radio = *extRadio;
	printf("Using external radio\n");
#endif
	// Init nRRF24L01
	radio.begin();

	// enable dynamic payloads, channel, data rate 250K
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.setPALevel(RF24_PA_LOW);
	radio.setChannel(_channel);
	radio.setDataRate(_dataRate);
	radio.setRetries(15, 3);
	radio.setCRCLength(RF24_CRC_16);
	radio.openWritingPipe(_addrLong);
	radio.openReadingPipe(1, _addrLong);

	//runTime = millis();

	// Start listening
	radio.startListening();
}

void CF_Ardu::printRadioInfo()
{
#ifdef USE_EXT_RADIO
	RF24 radio = *extRadio;
#endif
	radio.printDetails();
}

void CF_Ardu::stopRadio()
{
#ifdef USE_EXT_RADIO
	RF24 radio = *extRadio;
#endif
	// nothing here yet
	radio.stopListening();
}

void CF_Ardu::setCommanderInterval(uint8_t msInt)
{
	_commanderInterval = msInt;
}

void CF_Ardu::setCommanderSetpoint(float pitch, float roll, float yaw, uint16_t thrust)
{
	_setThrust = thrust;
	_setPitch = pitch;
	_setRoll = roll;
	_setYaw = yaw;
}

void CF_Ardu::prepareCommanderPacket()
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


void CF_Ardu::initLogSystem()
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

bool CF_Ardu::hasLogInfo()
{
	return _logReady;
}

bool CF_Ardu::isBusy()
{
	return (bool)_busy;
}

void CF_Ardu::sendAndReceive(uint32_t timeout)
{
#ifdef USE_EXT_RADIO
	RF24 radio = *extRadio;
#endif

	// is the commander on?
	if (_keepAlive) {
		prepareCommanderPacket();
	}

	// if we are not busy, no need to be here
	if (!_busy) {
		debugln("Nothing to do");
		return;
	}
	// First, stop listening so we can talk.
	radio.stopListening();

	// payload should already be set up
	
	// send the packet. Blocks until sent
    //printOutgoingPacket();
    if (send_state == DUMMY) {
        uint8_t dummyVal = 0xff;
        radio.write(&dummyVal, 1);
    } else {
	    radio.write(_outgoing, _outPacketLen);
    }
	// unset commander flag if set - will be reset next loop if keepalive is on
	if ((_busy & BUSY_COMMANDER)) {
		_busy &= ~BUSY_COMMANDER;
	}

	// start listening for an ACK
	radio.startListening();
	// Wait here until we get all responses, or timeout
	bool didTimeout = false;
	unsigned long start = millis();
	while (!radio.available() && !didTimeout)
	{
		if (millis() - start > timeout)
			didTimeout = true;
	}

	if (didTimeout)
	{
		debugln("response timed out");
	}
	else
	{
		// clear _incoming packet
		memset(_incoming, 0, 32);

		// read response
		_inPacketLen = radio.getDynamicPayloadSize();
		radio.read(_incoming, _inPacketLen);

		dispatchPacket();
	}

}

void CF_Ardu::dispatchPacket()
{
	uint8_t port = (_incoming[0] >> 4);
	uint8_t channel = (_incoming[0] & 0x3);
    //debugln("Port: %x Channel: %x\n", port, channel);
	if (port == PORT_LOGGING && channel == CHANNEL_TOC) {
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
                printf("TOC size: %d\n", _logInfo.num); 
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
                    printf("Got variable %s.%s\n", p.varName, &p.varName[groupEnd+1]);
					_itemToFetch += 1;
					send_state = LOG_TOC;
				}
			if (_itemToFetch >= _logInfo.num) {
				_busy &= ~BUSY_LOG_TOC;
				_logReady = true;
				send_state = DUMMY;

				debugln("LOG COMPLETE");
			}
			else {
				requestNextTOCItem();
				send_state = LOG_TOC;
			}
			uint8_t groupEnd = 0;
			while (p.varName[groupEnd] != '\0') groupEnd++;

			// let's print what was in here
			debug("Got variable ");
			debug(fetchedItem);
			debug(": ");
			char groupName[32] = { 0 };
			char varName[32] = { 0 };

			memcpy(groupName, p.varName, groupEnd);
			memcpy(varName, p.varName + groupEnd + 1, 28 - groupEnd - 1);
			debug(groupName);
			debug(".");
			debugln(varName);

			break;
		}
		default:
			debugln("Unknown packet");
			break;
		}
	}
}

void CF_Ardu::requestNextTOCItem()
{
	// prepare the packet
	debug("Preparing TOC item request: ");
	debugln(_itemToFetch);

	memset(_outgoing, 0, 32);
	_outgoing[0] = (PORT_LOGGING & 0xF) << 4 | 3 << 2 | (CHANNEL_TOC & 0x3);
    cl_toc_pkt p = {0};
	p.command = CMD_GET_ITEM;
	p.index = _itemToFetch;
    p.pack(_outgoing+1);
	_outPacketLen = 3;
}


void CF_Ardu::startCommander()
{
	_keepAlive = true;
}

void CF_Ardu::stopCommander()
{
	_keepAlive = false;
}


void CF_Ardu::printOutgoingPacket()
{
    printf("Outgoing length: %d\n", _outPacketLen);    
    for (int i = 0; i < 32; i++) {
        printf("%02x ", _outgoing[i]);
    }
    printf("\n");
}

void CF_Ardu::printIncomingPacket()
{
    printf("Incoming length: %d\n", _inPacketLen);    

    for (int i = 0; i < 32; i++) {
        printf("%02x ", _incoming[i]);
    }
    printf("\n");
}
