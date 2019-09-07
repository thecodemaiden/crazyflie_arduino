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
#define GET_LOGS 1
#define PRINT_TOC 1


void initRadio(RF24 &radio)
{
	radio.begin();

	// enable dynamic payloads, channel, data rate 250K
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.setPALevel(RF24_PA_LOW);
	radio.setChannel(125);
	radio.setDataRate(RF24_2MBPS);
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
        flies[i]->send();
    }
    for (int i=0; i<nFlies; i++) {
	flies[i]->receive();
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

    addCrazyflie(&radio, FULL_ADDR(E7), 1);

    Itinerary it = Itinerary(flies[0]);
    setupItinerary(&it);

    radio.printDetails();

#ifdef GET_LOGS

    for (int j=0; j<nFlies; j++) {
        flies[j]->initLogSystem();
    }
     
    bool allReady = false;
    while (!allReady) {
        int nReady = 0;
        for (int j=0; j<nFlies; j++) {
            flies[j]->send();
	}
	for (int j=0; j<nFlies; j++) {
	    flies[j]->receive(50);
            if (flies[j]->hasLogInfo()) {
                ++nReady;
                //we can do other things...
            }
            allReady = (nReady == nFlies);
        }
    }
    printf("All TOCs downloaded.\n");
#ifdef PRINT_TOC
    unsigned int logSize = flies[0]->getLogTocSize();
    for (unsigned int i =0; i!=logSize; i++) {
        const LogVariable *v = flies[0]->getLogVariable(i);
        printf("%d:\t%s\n", i, v->name);
    }
#endif
#endif
    for (int j=0; j<nFlies; j++) {
        flies[j]->requestRSSILog();
        flies[j]->setCommanderInterval(200);
        flies[j]->startCommander();
    }
    bool itinComplete = false;
    int sendNum = 0;
    bool sendResult = false;
    while (!itinComplete) {
        for (int i=0; i<nFlies; i++) {
		
            sendResult = flies[i]->send();
            sendNum += 1;
            printf("send[%02d][%lu] ", i+1, millis());
	    if (!sendResult)
                printf("failed \n");
	    else
	        printf("succeeded \n");
#ifdef DOFLY
            itinComplete = it.tick();
#else
            delay(50);
#endif
	    flies[i]->receive(100);
        }
    }
    cleanup();

    return 0;
}

