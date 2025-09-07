//
// Created by joebishop on 07/09/2025.
//

#ifndef SIM3_CONFIG_H
#define SIM3_CONFIG_H
#include <numbers>
#include <btBulletDynamicsCommon.h>

class config {
private:
    static constexpr float pi = static_cast<float>(std::numbers::pi);
public:
    // LiDAR
    float MaxLidarDistance = 10.f; // (m) beyond this distance, the LIDAR will be unable to detect the ground
    btVector3 LidarPosition = btVector3(0, 0, -1); // (m) position of the LIDAR relative to the rocket's
                                                        //origin

    // Propellant config
    float InitialPropVolume = 0.5f; // (m^3) volume of propelant at the start of the simulation
    float PropDensity = 600.f; // (kg/m^3) density of the propellant

    // Engine Config
    float Isp = 300.f; // (s) specific impulse of the engine
    float MaxFuelFlowRate = 0.5f; // (kg/s) maximum rate at which fuel can be expelled
    float MinThrottle = 0.4f; // below this value, the engine will flame out and produce no thrust
    float MaxRateOfChangeOfThrottle = 0.5f; // (1/s) maximum rate at which the throttle can change
    btVector3 EnginePosition = btVector3(0, 0, -1); // (m) position of the engine relative to the rocket's
                                                          // origin

    //TVC Config
    // The below are the minimum and maximum angles that the engine can gimbal to
    float MinAngleX = -10 * pi / 180; // (rad)
    float MaxAngleX = 10 * pi / 180; // (rad)
    float MinAngleY = -10 * pi / 180; // (rad)
    float MaxAngleY = 10 * pi / 180; // (rad)
    // The below are the maximum rates at which the engine can gimbal
    float MaxRateOfChangeOfAngleX = 20 * pi / 180; // (rad/s)
    float MaxRateOfChangeOfAngleY = 20 * pi / 180; // (rad/s)

    // Rocket Config
    float RocketDryMass = 30.f; // (kg) mass of the rocket without propellant
    btVector3 COMPosition = btVector3(0, 0, -0.2);
    float RocketHeight = 2.f; // (m) height of the rocket
    float RocketRadius = 0.5f; // (m) radius of the rocket
};


#endif //SIM3_CONFIG_H