#include "ros/ros.h"
#include "sensor_manager.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "reef_estimator");
    reef_estimator::SensorManager estimator_object;
    ros::spin();
    return 0;
}
