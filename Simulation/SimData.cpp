//
// Created by joebishop on 08/09/2025.
//
#include "SimData.h"

#include <stdexcept>
#include <string>

SimData::SimData(const float time, const btVector3 pos, const btQuaternion orient, const float throttle,
                 const float fuelMass, const btVector3 engineForce) {
    Time = time;
    position = pos;
    orientation = orient;
    Throttle = throttle;
    FuelMass = fuelMass;
    EngineForce = engineForce;
}
SimData::SimData() = default;

std::string SimData::GetString() const {
    std::string output = "";
    output += std::to_string(Time) + ",";
    output += std::to_string(position.getX()) + ",";
    output += std::to_string(position.getY()) + ",";
    output += std::to_string(position.getZ()) + ",";
    output += std::to_string(orientation.getX()) + ",";
    output += std::to_string(orientation.getY()) + ",";
    output += std::to_string(orientation.getZ()) + ",";
    output += std::to_string(orientation.getW()) + ",";
    output += std::to_string(Throttle) + ",";
    output += std::to_string(FuelMass) + ",";
    output += std::to_string(EngineForce.getX()) + ",";
    output += std::to_string(EngineForce.getY()) + ",";
    output += std::to_string(EngineForce.getZ());
    return output;
}
