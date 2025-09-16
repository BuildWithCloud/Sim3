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


    void SaveLog();

private:
    Config* SimulationConfig;
    btRigidBody* Rocket;
    std::vector<SimData*> Logs;

    // Rocket present state
    float FuelMass = 0;
    float throttle = 0;
    btVector3 EngineForce;
    btVector3 COMPosition;

    // Engine
    btVector3 EngineForceFromControlInputs(float throttle, float TVCAngleX, float TVCAngleY);
    bool AllowEngine(float throttle, float fuelMass);
    float CalculateTrueThrottlePosition(float desiredThrottle, float currentThrottle);

    // Sensors
    btVector3 CalculateIMULinearAccels();

    // Simulation
    btVector3 CalculateCOMPosition();

    // Logging
};


#endif //SIM3_SIM3_H