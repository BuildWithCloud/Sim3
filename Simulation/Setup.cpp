//
// Created by joebishop on 07/09/2025.
//

#include "Setup.h"
#include <btBulletDynamicsCommon.h>
#include "Config.h"
#include "Sim3.h"

void Setup::Main() {
    // Initialise Bullet Physics
    auto* config = new Config();
    auto* collisionConfiguration = new btDefaultCollisionConfiguration();
    auto* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    auto* overlappingPairCache = new btDbvtBroadphase();
    auto* solver = new btSequentialImpulseConstraintSolver;
    auto* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, 0, -9.81));
    btRigidBody rocket = InitialiseRocket(dynamicsWorld, config);
    btRigidBody floor = InitialiseFloor(dynamicsWorld, config);
    // End Initialisation
    // Start Simulation
    Sim3 sim;
    sim.simulate(dynamicsWorld, config);
    // End Simulation
    // Cleanup and deletion
    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;
    // cleanup end
}

btRigidBody Setup::InitialiseRocket(btDynamicsWorld* dynamics_world, Config* config) {
    btCollisionShape* rocketCollisionShape = new btCylinderShapeZ(btVector3(btScalar(config->RocketRadius),
        btScalar(config->RocketRadius), btScalar(config->RocketHeight / 2)));
    btTransform startTransform = config->RocketStartPosition;

    btScalar mass(config->RocketDryMass + config->InitialPropVolume * config->PropDensity);
    btVector3 localInertia(0, 0, 0);
    rocketCollisionShape->calculateLocalInertia(mass, localInertia);
    auto* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, rocketCollisionShape, localInertia);
    auto* body = new btRigidBody(rbInfo);
    btTransform COMTransform = btTransform::getIdentity();
    COMTransform.setOrigin(config->COMPosition);
    body->setCenterOfMassTransform(COMTransform);
    dynamics_world->addRigidBody(body);
    return *body;
}

btRigidBody Setup::InitialiseFloor(btDynamicsWorld* dynamics_world, Config* config) {
    btCollisionShape* floorCollisionShape = new btBoxShape(btVector3(100, 100, 2));
    btTransform startTransform = btTransform::getIdentity();
    startTransform.setOrigin(btVector3(0, 0, -1));
    btScalar mass(0.f);
    btVector3 localInertia(0, 0, 0);
    auto* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, floorCollisionShape, localInertia);
    auto* body = new btRigidBody(rbInfo);
    dynamics_world->addRigidBody(body);
    return *body;
}