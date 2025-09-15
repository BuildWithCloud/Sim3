//
// Created by joebishop on 07/09/2025.
//

#include "Sim3.h"
#include <algorithm>
#include <iostream>
#include "Config.h"
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include "SimData.h"
#include <fstream>

void Sim3::simulate(btDynamicsWorld* dynamics_world, const Config* config) {
    btRigidBody* rocket = btRigidBody::upcast(dynamics_world->getCollisionObjectArray()[0]);
    const int iterations = static_cast<int>(config->SimulationTime / config->TimeStep);
    SimData* logs[iterations] = {};
    float throttle = 0.0f;
    float FuelMass = config->InitialPropVolume * config->PropDensity;
    for (int i = 0; i < iterations; i++) {
        // Generate Sensor Data

        // Run Control
        float DesiredThrottleInput = 0.5f;
        float DesiredTVCX = 0.f;
        float DesiredTVCY = 0.f;
        // Change Vehicle State
        throttle = CalculateTrueThrottlePosition(DesiredThrottleInput, throttle, config);
        FuelMass = FuelMass - throttle * config->MaxFuelFlowRate * config->TimeStep;
        if (FuelMass < 0.f) FuelMass = 0.f;
        // Figure out forces from control output
        btVector3 EngineForce = EngineForceFromControlInputs(throttle, 0.0f, 0.0f, config);

        //Apply Forces
        EngineForce = EngineForce.rotate(rocket->getOrientation().getAxis(), rocket->getOrientation().getAngle());
        // rotate to match rocket orientation
        if (AllowEngine(throttle, FuelMass, config)) rocket->applyForce(EngineForce, config->EnginePosition);

        // Save Data
        logs[i] = new SimData(i * config->TimeStep, rocket->getCenterOfMassPosition(), rocket->getOrientation(),
                              throttle, FuelMass, EngineForce);
        // Simulate next step
        dynamics_world->stepSimulation(config->TimeStep);
    }
    SaveLog(logs, iterations);
    for (int i = 0; i < iterations; i++) {
        delete logs[i];
    }

}

btVector3 Sim3::EngineForceFromControlInputs(float throttle, float TVCAngleX, float TVCAngleY, const Config* config) {
    // Clamp throttle
    if (throttle < config->MinThrottle) {
        throttle = 0.f; // Engine flames out
    }
    if (throttle > 1.f) {
        throttle = 1.f;
    }
    // Clamp TVC angles
    if (TVCAngleX < config->MinAngleX) {
        TVCAngleX = config->MinAngleX;
    }
    if (TVCAngleX > config->MaxAngleX) {
        TVCAngleX = config->MaxAngleX;
    }
    if (TVCAngleY < config->MinAngleY) {
        TVCAngleY = config->MinAngleY;
    }
    if (TVCAngleY > config->MaxAngleY) {
        TVCAngleY = config->MaxAngleY;
    }
    // Calculate thrust
    float thrust = throttle * config->Isp * 9.81f * config->MaxFuelFlowRate; // Thrust in Newtons
    // Calculate force vector
    btVector3 force(0, 0, thrust);
    // Apply gimbal angles
    force.rotate(btVector3(1, 0, 0), TVCAngleX);
    force.rotate(btVector3(0, 1, 0), TVCAngleY);
    return force;
}

float Sim3::CalculateTrueThrottlePosition(float desiredThrottle, float currentThrottle, const Config* config) {
    float maxChange = config->MaxRateOfChangeOfThrottle * config->TimeStep;
    float throttleDifference = desiredThrottle - currentThrottle;
    return std::ranges::clamp(throttleDifference, throttleDifference-maxChange, throttleDifference+maxChange);
}

bool Sim3::AllowEngine(float throttle, float fuelMass, const Config* config) {
    if (throttle < config->MinThrottle) {
        return false;
    }
    if (fuelMass <= 0.f) {
        return false;
    }
    return true;
}

void Sim3::SaveLog( SimData* logs[], int iterations ) {
    std::string output = "Time,posX,posY,posZ,orientationX,orientationY,orientationZ,orientationW,Throttle,FuelMass,EngineForceX,EngineForceY,EngineForceZ\n";
    for (int i = 0; i < iterations ; i++) {
        output += logs[i]->GetString() + "\n";
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