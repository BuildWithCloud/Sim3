//
// Created by joebishop on 15/09/2025.
//

#include "SensorData.h"

SensorData::SensorData(btVector3* imuLinearAccels, btVector3* imuAngularAccels, std::optional<float> lidarAltitude, float thrust){
    IMULinearAccels = imuLinearAccels;
    IMUAngularAccels = imuAngularAccels;
    LIDARAltitude = lidarAltitude;
    Thrust = thrust;
}