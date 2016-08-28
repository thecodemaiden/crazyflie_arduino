#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include "cflie.h"
#include "cflie_log.h"

using namespace std;

struct Setpoint
{
    unsigned long afterTime;
    float pitch;
    float roll;
    float yaw;
    int16_t thrust;
    Setpoint():afterTime(0), pitch(0), roll(0), yaw(0), thrust(0) {}
};


void initRadio(RF24 &radio)
{
	radio.begin();

	// enable dynamic payloads, channel, data rate 250K
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.setPALevel(RF24_PA_LOW);
	radio.setChannel(0x50);
	radio.setDataRate(RF24_250KBPS);
	radio.setRetries(15, 3);
	radio.setCRCLength(RF24_CRC_16);
	radio.openWritingPipe(0xE7E7E7E7E7L);
	radio.openReadingPipe(1, 0xE7E7E7E7E7L);

	//runTime = millis();

	// Start listening
	radio.startListening();
}

#define itinLen 5

Setpoint itinerary[itinLen];
unsigned long startTime;

void runItinerary(Crazyflie *copter)
{
    unsigned i=0;
    Setpoint *next = &itinerary[i];

    while (i<itinLen) {
        unsigned long now = millis();
        if (startTime + next->afterTime <= now) {
            printf("Executing step %d\n", i);
            copter->setCommanderSetpoint(next->pitch, next->roll, next->yaw, next->thrust);
            ++i;
            next = &itinerary[i];
        }
        copter->sendAndReceive(100);
    }   
}

#define HOVER_THRUST 41000

void setupItinerary()
{
    itinerary[0].afterTime = 2000;
    itinerary[0].thrust = 44000;

    itinerary[1].afterTime = 3500;
    itinerary[1].thrust = HOVER_THRUST;

    itinerary[2].afterTime = 7000;
    itinerary[2].roll = -5.0;
    itinerary[2].thrust = HOVER_THRUST;

    itinerary[3].afterTime = 9000;
    itinerary[3].roll = 10.0;
    itinerary[3].thrust = HOVER_THRUST;

    itinerary[4].afterTime = 11000;
    itinerary[4].thrust = 37500;

}


#define RSSI_IDX 51
int main(int argc, char** argv){
    RF24 radio(26,10);
    initRadio(radio);

    radio.printDetails();
    Crazyflie cf(&radio);

    setupItinerary();
    cf.initLogSystem();
    bool readyToFly = false;
    while (!readyToFly) {
        readyToFly = cf.hasLogInfo();
        cf.sendAndReceive(50);
    }
    printf("Log TOC downloaded.\n");

#ifdef PRINT_TOC
    unsigned int logSize = cf.getLogTocSize();
    for (unsigned int i =0; i!=logSize; i++) {
        const LogVariable *v = cf.getLogVariable(i);
        printf("%d:\t%s\n", i, v->name);
    }
#endif

//    cf.startCommander();
#define DOFLY 0
#if DOFLY

    startTime = millis();

    runItinerary(&cf);

    printf("Itinerary complete.\n");

    cf.setCommanderSetpoint(0,0,0,35000);
#endif

   cf.requestRSSILog();
  //  cf.stopRSSILog();
    while (1) {
        cf.sendAndReceive(100);
    }

    return 0;
}

