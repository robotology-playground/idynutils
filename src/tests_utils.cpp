#include <drc_shared/tests_utils.h>
#include <time.h>
#include <iostream>

double tests_utils::getRandomAngle()
{
    static bool initialized = false;
    if(!initialized) {
        time_t seed = time(NULL);
        seed48((unsigned short*)(&seed));
        initialized = true;
    }
    return drand48()*2.0*M_PI-M_PI;
}

