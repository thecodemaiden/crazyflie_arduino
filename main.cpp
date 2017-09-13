#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include "cflie.h"
#include "cflie_log.h"
#include <vector>
#include <memory>

using namespace std;

struct Setpoint
{
    unsigned long afterTime;
    float pitch;
    float roll;
    float yaw;
    int16_t thrust;
    Setpoint(unsigned long afterTime=0, int16_t thrust=0,
        float pitch=0, float roll=0, float yaw=0)
    :afterTime(afterTime), pitch(pitch), roll(roll), yaw(yaw), thrust(thrust) {}
};

class Itinerary
{
    std::vector<shared_ptr<Setpoint> > setpoints;
    size_t nextIdx;
    bool started;
    unsigned long startTime;
    Crazyflie *copter;

public:
    void addSetpoint(shared_ptr<Setpoint> sp)
    {
        setpoints.push_back(sp);
    }

    Itinerary(Crazyflie *cf)
        :nextIdx(0), started(false), startTime(0) 
    {
        copter = cf;
    }

    bool tick() {
        // return true when done
        unsigned long now = millis();
        if (!started) {
            startTime = now;
            started = true;
        }
        if (nextIdx == setpoints.size()) return true;
        shared_ptr<Setpoint> next = setpoints.at(nextIdx);

        if (startTime + next->afterTime <= now) {
            printf("Executing step %d\n", nextIdx);
            copter->setCommanderSetpoint(next->pitch, next->roll, next->yaw, next->thrust);
            ++nextIdx;
        }
        return false;
    }

};

Crazyflie *flies[6];
int nReady = 0;
int nFlies = 0;

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

void cleanup()
{
    for (int i=0; i<nFlies; i++) {
        flies[i]->setCommanderSetpoint(0,0,0,0);
        flies[i]->sendAndReceive(50);
        delete flies[i];
    }
}

void addCrazyflie(RF24 *radio, uint64_t address, uint8_t pipeNum)
{
    if (nFlies < 6) {
        Crazyflie *cf = new Crazyflie(radio, address, pipeNum);
        flies[nFlies++] = cf;
        radio->stopListening();
        radio->openReadingPipe(nFlies, address);
        radio->startListening();
    }
}

#define HOVER_THRUST 41000

void setupItinerary(Itinerary *itin, int offset)
{

    shared_ptr<Setpoint> sp4(new Setpoint(2000+offset,10000,0,10.0,0));
    shared_ptr<Setpoint> sp5(new Setpoint(4000+offset,8000,0,0,0));

    itin->addSetpoint(sp4);
    itin->addSetpoint(sp5);

}


int main(int argc, char** argv){


    RF24 radio(26,10);
    initRadio(radio);

    addCrazyflie(&radio, FULL_ADDR(E7), 1);
    addCrazyflie(&radio, FULL_ADDR(E6), 2);

    Itinerary it = Itinerary(flies[0]);
    setupItinerary(&it, 0);
    Itinerary it2 = Itinerary(flies[1]);
    setupItinerary(&it2, 1500);

    radio.printDetails();

    for (int j=0; j<nFlies; j++) {
        flies[j]->initLogSystem();
    }
     
    bool allReady = false;
    while (!allReady) {
        int nReady = 0;
        for (int j=0; j<nFlies; j++) {
            Crazyflie *cf = flies[j];
            cf->sendAndReceive(50);
            if (cf->hasLogInfo()) {
                ++nReady;
                //we can do other things...
            }
            allReady = (nReady == nFlies);
        }
    }
    printf("All TOCs downloaded.\n");

#ifdef PRINT_TOC
    unsigned int logSize = cf->getLogTocSize();
    for (unsigned int i =0; i!=logSize; i++) {
        const LogVariable *v = cf->getLogVariable(i);
        printf("%d:\t%s\n", i, v->name);
    }
#endif
    for (int j=0; j<nFlies; j++) {
        flies[j]->setCommanderInterval(200);
        flies[j]->startCommander();
    }

    bool itin1Complete = false;
    bool itin2Complete = false;
    while (!itin1Complete || !itin2Complete) {
        for (int i=0; i<nFlies; i++) {
            flies[i]->sendAndReceive(50);
            itin1Complete = it.tick();
            itin2Complete = it2.tick();
        }
    }

    cleanup();

    return 0;
}

