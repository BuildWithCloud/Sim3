//
// Created by joebishop on 08/09/2025.
//
#include "SimData.h"

#include <stdexcept>
#include <string>

SimData::SimData(const float time, const btVector3 pos, const btQuaternion orient, const float throttle,
                 const float fuelMass, const btVector3 engineForce, const btVector3 comPosition) {
    Time = time;
    position = pos;
    orientation = orient;
    Throttle = throttle;
    FuelMass = fuelMass;
    EngineForce = engineForce;
    COMPosition = comPosition;
}
SimData::SimData() = default;

std::string SimData::GetString() const {
    std::string output = "";
    output += std::to_string(Time) + ",";
    output += std::to_string(position.getX()) + ",";
    output += std::to_string(position.getY()) + ",";
    output += std::to_string(position.getZ()) + ",";
    output += std::to_string(velocity.getX()) + ",";
    output += std::to_string(velocity.getY()) + ",";
    output += std::to_string(velocity.getZ()) + ",";
    output += std::to_string(orientation.getX()) + ",";
    output += std::to_string(orientation.getY()) + ",";
    output += std::to_string(orientation.getZ()) + ",";
    output += std::to_string(orientation.getW()) + ",";
    output += std::to_string(Throttle) + ",";
    output += std::to_string(FuelMass) + ",";
    output += std::to_string(EngineForce.getX()) + ",";
    output += std::to_string(EngineForce.getY()) + ",";
    output += std::to_string(EngineForce.getZ()) + ",";
    output += std::to_string(COMPosition.getX()) + ",";
    output += std::to_string(COMPosition.getY()) + ",";
    output += std::to_string(COMPosition.getZ());
    return output;
}

std::string SimData::GetHeaderString(){
    return "Time,posX,posY,posZ,velX,velY,velZ,orientationX,orientationY,orientationZ,orientationW,Throttle,"
           "FuelMass,EngineForceX,EngineForceY,EngineForceZ,COMPosX,COMPosY,COMPosZ\n";
}

