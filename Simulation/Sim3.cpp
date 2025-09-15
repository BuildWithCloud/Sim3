//
// Created by joebishop on 07/09/2025.
//

#include "Sim3.h"
#include <algorithm>
#include "Config.h"
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include "SimData.h"
#include <fstream>

Sim3::Sim3(Config* config) {
    SimulationConfig= config;
}

void Sim3::simulate(btDynamicsWorld* dynamics_world) {
    btRigidBody* rocket = btRigidBody::upcast(dynamics_world->getCollisionObjectArray()[0]);
    const int iterations = static_cast<int>(SimulationConfig->SimulationTime / SimulationConfig->TimeStep);
    SimData* logs[iterations] = {};
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
        EngineForce = EngineForce.rotate(rocket->getOrientation().getAxis(), rocket->getOrientation().getAngle());
        if (AllowEngine(throttle, FuelMass)) rocket->applyForce(EngineForce, SimulationConfig->EnginePosition);

        // Save Data
        logs[i] = new SimData(i * SimulationConfig->TimeStep, rocket->getCenterOfMassPosition(), rocket->getOrientation(),
                              throttle, FuelMass, EngineForce);
        // Simulate next step
        dynamics_world->stepSimulation(SimulationConfig->TimeStep);
    }
    SaveLog(logs, iterations);
    for (int i = 0; i < iterations; i++) {
        delete logs[i];
    }

}

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

btVector3 Sim3::CalculateIMULinearAccels() {
    // true value
    btVector3 IMULinearAccels = EngineForce;
    // add noise

    // return
    return IMULinearAccels;
}