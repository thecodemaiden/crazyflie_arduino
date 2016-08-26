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


void mainLoop()
{   
    cf.sendAndReceive(100);  
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

  cf.initLogSystem();
    printf("%lu\n", millis());
  while(1) {
     mainLoop();
    }

  return 0;
}

