//
// Created by joebishop on 07/09/2025.
//

#ifndef SIM3_SETUP_H
#define SIM3_SETUP_H
#include <btBulletDynamicsCommon.h>
#include "Config.h"

class Setup {
public:
    static void Main();

    static btRigidBody InitialiseRocket(btDynamicsWorld* dynamics_world, Config* config);

    static btRigidBody InitialiseFloor(btDynamicsWorld* dynamics_world, Config* config);
};


#endif //SIM3_SETUP_H