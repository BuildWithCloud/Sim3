//
// Created by joebishop on 07/09/2025.
//

#ifndef SIM3_SIM3_H
#define SIM3_SIM3_H
#include <iosfwd>
#include <bits/stl_vector.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "Config.h"
#include "SimData.h"


class Sim3 {
public:
    static void simulate(btDynamicsWorld *dynamics_world, const Config *config);
    static btVector3 EngineForceFromControlInputs(float throttle, float TVCAngleX, float TVCAngleY, const Config* config);
    static float CalculateTrueThrottlePosition(float desiredThrottle, float currentThrottle, const Config* config);

    static bool AllowEngine(float throttle, float fuelMass, const Config *config);

    static void SaveLog(SimData *logs[], int iterations);
};


#endif //SIM3_SIM3_H