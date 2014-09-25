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

double tests_utils::getRandomAngle(const double min, const double max)
{
    if(min < -M_PI || max > M_PI)
        return getRandomAngle();

    static bool initialized = false;
    if(!initialized) {
        srand(time(NULL));
        initialized = true;
    }
    return (double)rand()/RAND_MAX * (max-min) + min;
}

