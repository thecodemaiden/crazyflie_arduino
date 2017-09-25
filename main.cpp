#include <cstdlib>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include "cflie.h"
#include "cflie_log.h"

using namespace std;

Crazyflie *cf;

#define FULL_ADDR(X) (0xE7E7E7E7 ## X)


void initRadio(RF24 &radio)
{
	radio.begin();

	// enable dynamic payloads, channel, data rate 250K
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.setPALevel(RF24_PA_LOW);
	radio.setChannel(0x50);
	radio.setDataRate(RF24_250KBPS);
	radio.setRetries(15, 1);
	radio.setCRCLength(RF24_CRC_16);
	radio.openWritingPipe(FULL_ADDR(E7));
	radio.openReadingPipe(1, FULL_ADDR(E7));

	// Start listening
	radio.startListening();
}

int main(int argc, char** argv){

    RF24 radio(26,10);
    initRadio(radio);
    unsigned int addrIn=0;
    printf("Enter copter address suffix: ");
    scanf("%2xhh", &addrIn);


    uint64_t address = FULL_ADDR(00)+addrIn;
    // Setup Crazyflie

	cf = new Crazyflie(&radio, address, 0);
	radio.stopListening();
	radio.openReadingPipe(0, address);
	radio.startListening();

    radio.printDetails();

//#define LOAD_LOG
#ifdef LOAD_LOG
    // Loading logs in parallel takes too GD long
        cf->initLogSystem();
        while(!cf->hasLogInfo()) {
            cf->sendAndReceive(50);
        }
        printf("Crazyflie  is ready\n");
#ifdef PRINT_TOC
    unsigned int logSize = cf->getLogTocSize();
    for (unsigned int i =0; i!=logSize; i++) {
        const LogVariable *v = cf->getLogVariable(i);
        printf("%d:\t%s\n", i, v->name);
    }
#endif
#endif
	cf->setCommanderInterval(20);
	cf->setCommanderSetpoint(0,0,0,0);
	cf->startCommander();

	long nextTime;
	
	// pulse for 1.5s, rest for 1.5s, repeat 3x
	for (int l=0; l<3; l++) {
		nextTime = millis() + 1500;
		cf->setCommanderSetpoint(0,0,0,0);
		while (millis() < nextTime) {
			cf->sendAndReceive(50);
		}
		nextTime = millis() + 1000;
		cf->setCommanderSetpoint(0,0,0,10000);
		while (millis() < nextTime) {
			cf->sendAndReceive(50);
		}
		printf("Pulse 1\n");
	}

	cf->setCommanderSetpoint(0,0,0,0);
	cf->sendAndReceive(50);
	cf->sendAndReceive(50);
    delete cf;

    return 0;
}

