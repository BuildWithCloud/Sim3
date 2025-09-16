//
// Created by joebishop on 07/09/2025.
//

#ifndef SIM3_SIM3_H
#define SIM3_SIM3_H
#include <iosfwd>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "Config.h"
#include "SimData.h"
#include <vector>


class Sim3 {
public:
    Sim3(Config* config);
    void simulate(btDynamicsWorld *dynamics_world);
    btVector3 EngineForceFromControlInputs(float throttle, float TVCAngleX, float TVCAngleY);
    float CalculateTrueThrottlePosition(float desiredThrottle, float currentThrottle);

    bool AllowEngine(float throttle, float fuelMass);

    void SaveLog();

private:
    btVector3 EngineForce;
    float throttle = 0;
    float FuelMass = 0;
    Config* SimulationConfig;
    btRigidBody* Rocket;
    std::vector<SimData*> Logs;


    btVector3 CalculateIMULinearAccels();
};


#endif //SIM3_SIM3_H