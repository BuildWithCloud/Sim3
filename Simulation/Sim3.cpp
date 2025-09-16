//
// Created by joebishop on 07/09/2025.
//

#include "Sim3.h"
#include <algorithm>
#include "Config.h"
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include "SimData.h"
#include <fstream>
#include <vector>

Sim3::Sim3(Config* config) {
    SimulationConfig= config;
}

void Sim3::simulate(btDynamicsWorld* dynamics_world) {
    Rocket = btRigidBody::upcast(dynamics_world->getCollisionObjectArray()[0]);
    const int iterations = int(SimulationConfig->SimulationTime / SimulationConfig->TimeStep);
    throttle = 0.0f;
    FuelMass = SimulationConfig->InitialPropVolume * SimulationConfig->PropDensity;
    EngineForce = btVector3(0, 0, 0);
    for (int i = 0; i < iterations; i++) {
        // Generate Sensor Data

        // Run Control
        float DesiredThrottleInput = 0.5f;
        float DesiredTVCX = 0.f;
        float DesiredTVCY = 0.f;
        // Change Vehicle State
        throttle = CalculateTrueThrottlePosition(DesiredThrottleInput, throttle);
        FuelMass = FuelMass - throttle * SimulationConfig->MaxFuelFlowRate * SimulationConfig->TimeStep;
        if (FuelMass < 0.f) FuelMass = 0.f;
        // Figure out forces from control output
        EngineForce = EngineForceFromControlInputs(throttle, 0.0f, 0.0f);

        //Apply engine force
        EngineForce = EngineForce.rotate(Rocket->getOrientation().getAxis(), Rocket->getOrientation().getAngle());
        if (AllowEngine(throttle, FuelMass)) Rocket->applyForce(EngineForce, SimulationConfig->EnginePosition);

        // Save Data
        btVector3 currentRocketPosition = Rocket->getCenterOfMassPosition() - SimulationConfig->COMPosition;
        Logs.push_back( new SimData(i * SimulationConfig->TimeStep, currentRocketPosition, Rocket->getOrientation(),
                              throttle, FuelMass, EngineForce));
        // Simulate next step
        dynamics_world->stepSimulation(SimulationConfig->TimeStep);
    }
    SaveLog();
}

// Engine Things
btVector3 Sim3::EngineForceFromControlInputs(float throttle, float TVCAngleX, float TVCAngleY) {
    // Clamp throttle
    if (throttle < SimulationConfig->MinThrottle) {
        throttle = 0.f; // Engine flames out
    }
    if (throttle > 1.f) {
        throttle = 1.f;
    }
    // Clamp TVC angles
    if (TVCAngleX < SimulationConfig->MinAngleX) {
        TVCAngleX = SimulationConfig->MinAngleX;
    }
    if (TVCAngleX > SimulationConfig->MaxAngleX) {
        TVCAngleX = SimulationConfig->MaxAngleX;
    }
    if (TVCAngleY < SimulationConfig->MinAngleY) {
        TVCAngleY = SimulationConfig->MinAngleY;
    }
    if (TVCAngleY > SimulationConfig->MaxAngleY) {
        TVCAngleY = SimulationConfig->MaxAngleY;
    }
    // Calculate thrust
    float thrust = throttle * SimulationConfig->Isp * 9.81f * SimulationConfig->MaxFuelFlowRate; // Thrust in Newtons
    // Calculate force vector
    btVector3 force(0, 0, thrust);
    // Apply gimbal angles
    force.rotate(btVector3(1, 0, 0), TVCAngleX);
    force.rotate(btVector3(0, 1, 0), TVCAngleY);
    return force;
}

float Sim3::CalculateTrueThrottlePosition(float desiredThrottle, float currentThrottle) {
    float maxChange = SimulationConfig->MaxRateOfChangeOfThrottle * SimulationConfig->TimeStep;
    float throttleDifference = desiredThrottle - currentThrottle;
    return std::ranges::clamp(throttleDifference, throttleDifference-maxChange, throttleDifference+maxChange);
}

bool Sim3::AllowEngine(float throttle, float fuelMass) {
    if (throttle < SimulationConfig->MinThrottle) {
        return false;
    }
    if (fuelMass <= 0.f) {
        return false;
    }
    return true;
}

// Sensors:
//IMU
btVector3 Sim3::CalculateIMULinearAccels() {
    // true value
    SimData* PreviousSimData = Logs.back();
    btVector3 PreviousLinVel = PreviousSimData->velocity;
    btVector3 CurrentLinVel = Rocket->getLinearVelocity();
    btVector3 CurrentLinAcc = (CurrentLinVel - PreviousLinVel) / SimulationConfig->TimeStep;
    // add noise

    // return
    return CurrentLinAcc;
}

//Logging
void Sim3::SaveLog() {
    std::string output = "Time,posX,posY,posZ,velX,velY,velZ,orientationX,orientationY,orientationZ,orientationW,Throttle,FuelMass,EngineForceX,EngineForceY,EngineForceZ\n";
    for (auto log : Logs) {
        output += log->GetString() + "\n";
    }
    std::ifstream name("../NextLog.txt");
    std::string input;
    name >> input;
    int nextNumber = std::stoi(input);

    std::string pathToWriteLogs = "../Logs/log" + std::to_string(nextNumber) + ".csv";

    std::ofstream out(pathToWriteLogs);
    out << output;
    out.close();

    nextNumber ++;
    std::ofstream out2("../NextLog.txt");
    out2 << nextNumber;
    out2.close();
}
