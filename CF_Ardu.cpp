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

CF_Ardu::CF_Ardu(uint8_t cePin, uint8_t csPin, long long radioAddr, uint8_t radioChannel, rf24_datarate_e rfDataRate)
	:_cePin(cePin), _csPin(csPin), _addrLong(radioAddr), _channel(radioChannel), _dataRate(rfDataRate)
#ifndef USE_EXT_RADIO
	, radio(cePin, csPin)
#endif
{
	_busy = 0;
	_keepAlive = false;
	_logReady = false;
	_logInfo.num = -1;
	_addrShort = 0x00; // what's this??

}
void CF_Ardu::startRadio()
{
#ifdef USE_EXT_RADIO
	RF24 radio = *extRadio;
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

	delay(100);
	//runTime = millis();

	// Start listening
	radio.startListening();
}

void CF_Ardu::printRadioInfo()
{
#ifdef USE_EXT_RADIO
	RF24 radio = *extRadio;
#endif
	Serial.print("Channel: 0x");
	int chan = radio.getChannel();
	Serial.println(chan, HEX);
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
	memset(&outgoing, 0, 32);
	outgoing.header = (PORT_COMMANDER & 0xF) << 4 | 0x3 << 2;
	outgoing.setpoint.roll = _setRoll;
	outgoing.setpoint.pitch = _setPitch;
	outgoing.setpoint.yaw = _setYaw;
	outgoing.setpoint.thrust = _setThrust;
	_outPacketLen = 15;
	send_state = COMMANDER;
	int i;
	Serial.print(outgoing.header, HEX);
	for (i = 0; i < 14; i++){
		Serial.print(" ");
		Serial.print(outgoing.payload[i], HEX);
	}
	Serial.println();
}


void CF_Ardu::initLogSystem()
{
	if (_busy) return;
	_busy |= BUSY_LOG_TOC;
	// prepare the toc info packet
	memset(&outgoing, 0, 32);
	outgoing.header = (PORT_LOGGING & 0xF) << 4 | 0x3 << 2 | (CHANNEL_TOC & 0x3);
	outgoing.toc.command = CMD_GET_INFO;
	_outPacketLen = 2;
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

void CF_Ardu::sendAndReceive(long timeout)
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
	byte raw_packet[32] = { 0 };
	uint8_t packetLen = _outPacketLen;
	memcpy(raw_packet, &outgoing, 32);

	// send the packet. Blocks until sent
	radio.write(raw_packet, packetLen);

	// unset commander flag if set - will be reset next loop if keepalive is on
	if ((_busy & BUSY_COMMANDER)) {
		_busy &= ~BUSY_COMMANDER;
	}

	// start listening for an ACK
	radio.startListening();
	// Wait here until we get all responses, or timeout
	boolean didTimeout = false;
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
		// clear incoming packet
		memset(&incoming, 0, 32);

		// read response
		uint8_t len = radio.getDynamicPayloadSize();
		radio.read(raw_packet, len);

		memcpy(&incoming, raw_packet, len);

		dispatchPacket(len);

	}

}

void CF_Ardu::dispatchPacket(uint8_t len)
{
	byte port = (incoming.header >> 4);
	byte channel = (incoming.header & 0x3);
	//printIncomingPacket();
	if (port == PORT_LOGGING && channel == CHANNEL_TOC) {
		// first assume it's a crc packet
		byte command = incoming.crc_info.command;
		switch (command) {
		case CMD_GET_INFO:
		{
			// it is CRC
			memcpy(&_logInfo, &incoming.crc_info, 8);
			send_state = LOG_TOC;
			_itemToFetch = 0;
			requestNextTOCItem();
			Serial.print("TOC size: "); Serial.println(_logInfo.num);

			break;
		}
		case CMD_GET_ITEM:
			// actually it was a toc item request
		{
			if (_logInfo.num < 0)
				break;

			byte fetchedItem = incoming.item_info.index;
			send_state = LOG_TOC;
			if (fetchedItem < _itemToFetch) {
				send_state = DUMMY;
			}
			else
				if (fetchedItem == _itemToFetch) {
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
				//Serial.print("*");
				requestNextTOCItem();
				send_state = LOG_TOC;
			}
			uint8_t groupEnd = 0;
			while (incoming.item_info.varName[groupEnd] != '\0') groupEnd++;

			// let's print what was in here
			debug("Got variable ");
			debug(fetchedItem);
			debug(": ");
			char groupName[32] = { 0 };
			char varName[32] = { 0 };

			memcpy(groupName, incoming.item_info.varName, groupEnd);
			memcpy(varName, incoming.item_info.varName + groupEnd + 1, 28 - groupEnd - 1);
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
	_inPacketLen = len;
}

void CF_Ardu::requestNextTOCItem()
{
	// prepare the packet
	debug("Preparing TOC item request: ");
	debugln(_itemToFetch);

	memset(&outgoing, 0, 32);
	outgoing.header = (PORT_LOGGING & 0xF) << 4 | 3 << 2 | (CHANNEL_TOC & 0x3);
	outgoing.toc.command = CMD_GET_ITEM;
	outgoing.toc.index = _itemToFetch;
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
	debug("Outgoing length: ");
	debugln(_outPacketLen);
	debug("HEADER: 0x");
	debugln_hex(outgoing.header);
	debug("Payload: ");
	for (int i = 0; i < _outPacketLen - 1; i++) {
		debug("0x");
		debug_hex(outgoing.payload[i]);
		debug(" ");
	}
	debugln();
}

void CF_Ardu::printIncomingPacket()
{
	debug("Incoming length: ");
	debugln(_inPacketLen);
	debug("HEADER: 0x");
	debugln_hex(incoming.header);
	debug("Payload: ");
	for (int i = 0; i < _inPacketLen - 1; i++) {
		debug("0x");
		debug_hex(incoming.payload[i]);
		debug(" ");
	}
	debugln();
}
