//
// Created by joebishop on 15/09/2025.
//

#ifndef SIM3_SENSORDATA_H
#define SIM3_SENSORDATA_H
#include <LinearMath/btVector3.h>
#include <optional>


class SensorData {
public:
    btVector3* IMULinearAccels; // (m/s^2) acceleration measured by the IMU
    btVector3* IMUAngularAccels; // rad/s^2 anticlockwise around the axis
    std::optional<float> LIDARAltitude; // (m) Distance from ground to lidar (see config for further details)
    float Thrust; // (N) magnitude of thrust currently being produced
    SensorData(btVector3* imuLinearAccels, btVector3* imuAngularAccels, std::optional<float> lidarAltitude, float thrust);
};


#endif //SIM3_SENSORDATA_H