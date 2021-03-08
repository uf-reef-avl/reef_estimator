#ifndef REEF_ESTIMATOR_Z_ESTIMATOR_H
#define REEF_ESTIMATOR_Z_ESTIMATOR_H

#include "estimator.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

namespace reef_estimator
{
    class ZEstimator: public Estimator
    {
        public:          
            ZEstimator();
            ~ZEstimator();
            void resetLandingState();
            void setTakeoffState(bool takeoff);
            /*This class is inherited from Estimator, thus
            it has update and propagate methods already.
             */
        void updateLinearModel();

    };
}

#endif //REEF_ESTIMATOR_Z_ESTIMATOR_H
