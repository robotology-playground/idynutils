#ifndef _TESTS_UTILS_H_
#define _TESTS_UTILS_H_

#include <math.h>
#include <stdlib.h>
#include <time.h>

class tests_utils
{
public:
    /**
     * @brief getRandomAngle return a random angle in [-M_PI, M_PI)
     * @return random angle between [-M_PI, M_PI)
     */
    static double getRandomAngle();

};

#endif
