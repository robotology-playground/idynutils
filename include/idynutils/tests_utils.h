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

    /**
     * @brief getRandomAngle return a random angle in [min, max]
     * @param min >= -M_PI
     * @param max <= M_PI
     * @return random angle in [min, max]
     */
    static double getRandomAngle(const double min, const double max);

};

#endif
