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

#define MAX_FLIES 6

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

    Crazyflie* getCopter() { return copter;}

    Itinerary(Crazyflie *cf)
        :nextIdx(0), started(false), startTime(0), copter(cf){}

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

Crazyflie *flies[MAX_FLIES];
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

uint8_t addCrazyflie(RF24 *radio, uint64_t address)
{
    if (nFlies < MAX_FLIES) {
        Crazyflie *cf = new Crazyflie(radio, address, nFlies);
        flies[nFlies] = cf;
        radio->stopListening();
        radio->openReadingPipe(nFlies, address);
        radio->startListening();
	nFlies +=1;
    }
    return (uint8_t)nFlies-1;
}

#define HOVER_THRUST 41000

void setupItinerary(Itinerary *itin, int offset)
{
    // each drone buzzes for 1 second, offset 2s each
    // then they all buzz together at 13s for 1.5s   

    shared_ptr<Setpoint> sp1(new Setpoint(1000+offset,10000,0,10.0,0));
    shared_ptr<Setpoint> sp2(new Setpoint(2000+offset,0,0,10.0,0));
    shared_ptr<Setpoint> sp3(new Setpoint(14000,10000,0,10.0,0));
    shared_ptr<Setpoint> sp4(new Setpoint(15500,0,0,10.0,0));
    shared_ptr<Setpoint> sp5(new Setpoint(16000,0,0,10.0,0));

    itin->addSetpoint(sp1);
    itin->addSetpoint(sp2);
    itin->addSetpoint(sp3);
    itin->addSetpoint(sp4);
    itin->addSetpoint(sp5);

}


int main(int argc, char** argv){
    Itinerary *plans[MAX_FLIES]={NULL};

    RF24 radio(26,10);
    initRadio(radio);

    addCrazyflie(&radio, FULL_ADDR(E4));
    addCrazyflie(&radio, FULL_ADDR(E7));
    addCrazyflie(&radio, FULL_ADDR(E6));
    addCrazyflie(&radio, FULL_ADDR(E5));
    addCrazyflie(&radio, FULL_ADDR(E3));
    addCrazyflie(&radio, FULL_ADDR(E2));

    radio.printDetails();
    for (int i=0; i<nFlies; i++) {
	plans[i] = new Itinerary(flies[i]);
	setupItinerary(plans[i], i*2000);
    }
//#define LOAD_LOG
#ifdef LOAD_LOG
    // Loading logs in parallel takes too GD long
    for (int j=0; j<nFlies; j++) {
        Crazyflie *cf = flies[j];
        cf->initLogSystem();
        while(!cf->hasLogInfo()) {
            cf->sendAndReceive(50);
        }
        printf("Crazyflie %d is ready\n", j+1);
    }


#ifdef PRINT_TOC
    unsigned int logSize = cf->getLogTocSize();
    for (unsigned int i =0; i!=logSize; i++) {
        const LogVariable *v = cf->getLogVariable(i);
        printf("%d:\t%s\n", i, v->name);
    }
#endif
#endif
    for (int j=0; j<nFlies; j++) {
        flies[j]->setCommanderInterval(20);
        flies[j]->startCommander();
	flies[j]->sendAndReceive(50);
    }

    int nDone;
    do { 
	nDone = 0;
	// execute the itineraries
	for (int i=0; i<nFlies; i++) {
	    plans[i]->getCopter()->sendAndReceive(50);
	    if (plans[i]->tick()) nDone+=1;
	}
    } while (nDone < nFlies);
	

    for (int j=0; j<nFlies; j++) {
	delete plans[j];
    }

    cleanup();

    return 0;
}

