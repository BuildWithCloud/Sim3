//
// Created by joebishop on 08/09/2025.
//

#ifndef SIM3_SIMDATA_H
#define SIM3_SIMDATA_H
#include <string>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>

class SimData {
public:
    float Time;
    btVector3 position;
    btVector3 velocity;
    btQuaternion orientation;
    float Throttle;
    float FuelMass;
    btVector3 EngineForce;
    SimData(const float time, const btVector3 pos, const btQuaternion orient, const float throttle,
            const float fuelMass, const btVector3 engineForce);
    SimData();
    std::string GetString() const;
};
#endif //SIM3_SIMDATA_H
