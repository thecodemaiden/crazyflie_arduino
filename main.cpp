#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include "CF_Ardu.h"

using namespace std;
#ifdef USE_EXT_RADIO
RF24 radio(26,10);
#endif
CF_Ardu cf(26,10);

struct Setpoint
{
    unsigned long afterTime;
    float pitch;
    float roll;
    float yaw;
    int16_t thrust;
    Setpoint():afterTime(0), pitch(0), roll(0), yaw(0), thrust(0) {}
};

#define itinLen 5

Setpoint itinerary[itinLen];
unsigned long startTime;

void runItinerary(CF_Ardu *copter)
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

    /*

    itinerary[2].afterTime = 9000;
    itinerary[2].pitch = 0;
    itinerary[2].roll = 0;
    itinerary[2].yaw = 0;
    itinerary[2].thrust = 35000;
    */
}

int main(int argc, char** argv){
#ifdef USE_EXT_RADIO
    cf.extRadio = &radio;
    //radio.begin();
#endif
    cf.startRadio();
    cf.printRadioInfo();

#ifdef USE_EXT_RADIO
    radio.printDetails();
#endif

    setupItinerary();
    cf.initLogSystem();
    bool readyToFly = false;
    while (!readyToFly) {
        readyToFly = cf.hasLogInfo();
        cf.sendAndReceive(100);
    }
    cf.startCommander();

    startTime = millis();
    printf("Log TOC downloaded.\n");

    runItinerary(&cf);

    printf("Itinerary complete.\n");

    cf.setCommanderSetpoint(0,0,0,35000);
    while (1) {
        cf.sendAndReceive(100);
    }

    return 0;
}

