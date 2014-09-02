#include <drc_shared/tests_utils.h>

double tests_utils::getRandomAngle()
{
    static unsigned short seed = (unsigned short)time(NULL);
    seed48(&seed);
    return drand48()*2.0*M_PI-M_PI;
}

