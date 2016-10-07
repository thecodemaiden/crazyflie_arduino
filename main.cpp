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
	radio.setRetries(15, 0);
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

void setupItinerary(Itinerary *itin)
{

    shared_ptr<Setpoint> sp1(new Setpoint(2000,44000,0,0,0));
    shared_ptr<Setpoint> sp2(new Setpoint(3500,HOVER_THRUST,0,0,0));
    shared_ptr<Setpoint> sp3(new Setpoint(7000,HOVER_THRUST, 0,-5.0,0));
    shared_ptr<Setpoint> sp4(new Setpoint(9000,HOVER_THRUST,0,10.0,0));
    shared_ptr<Setpoint> sp5(new Setpoint(11000,37500,0,0,0));

    itin->addSetpoint(sp1);
    itin->addSetpoint(sp2);
    itin->addSetpoint(sp3);
    itin->addSetpoint(sp4);
    itin->addSetpoint(sp5);

}


int main(int argc, char** argv){


    RF24 radio(26,10);
    initRadio(radio);

    addCrazyflie(&radio, FULL_ADDR(E6), 1);
    addCrazyflie(&radio, FULL_ADDR(E7), 2);
    addCrazyflie(&radio, FULL_ADDR(E5), 3);
    addCrazyflie(&radio, FULL_ADDR(E4), 4);
    addCrazyflie(&radio, FULL_ADDR(E3), 5);

    Itinerary it = Itinerary(flies[1]);
    setupItinerary(&it);

    radio.printDetails();

/*
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
*/
#ifdef PRINT_TOC
    unsigned int logSize = cf->getLogTocSize();
    for (unsigned int i =0; i!=logSize; i++) {
        const LogVariable *v = cf->getLogVariable(i);
        printf("%d:\t%s\n", i, v->name);
    }
#endif
    for (int j=0; j<nFlies; j++) {
        flies[j]->requestRSSILog();
        flies[j]->setCommanderInterval(200);
        flies[j]->startCommander();
    }
    bool itinComplete = false;
    int sendNum = 0;
    while (!itinComplete) {
        for (int i=0; i<nFlies; i++) {
            flies[i]->sendAndReceive(50);
            sendNum += 1;
            printf("send[%02d][%lu]%d\n", i+1, millis(), sendNum);
#ifdef DOFLY
            itinComplete = it.tick();
#else
            delay(100);
#endif
        }
    }
    cleanup();

    return 0;
}

