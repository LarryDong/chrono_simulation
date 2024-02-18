	
#pragma once

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_models/vehicle/gator/Gator.h"


class MyPlatform {

public:
    MyPlatform(void) {

        using namespace chrono;
        using namespace chrono::vehicle;
        using namespace chrono::irrlicht;

        ChVector<> initLoc(0, 0, 0.5);
        ChQuaternion<> initRot(1, 0, 0, 0);

        platform_.SetContactMethod(ChContactMethod::NSC);
        platform_.SetChassisCollisionType(CollisionType::NONE);
        platform_.SetChassisFixed(false);
        platform_.SetInitPosition(ChCoordsys<>(initLoc, initRot));
        platform_.SetTireType(TireModelType::TMEASY);
        platform_.SetTireStepSize(1e-3);
        platform_.SetAerodynamicDrag(0.5, 5.0, 1.2);
        platform_.Initialize();
        platform_.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
        platform_.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        platform_.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
        platform_.SetWheelVisualizationType(VisualizationType::PRIMITIVES);
        platform_.SetTireVisualizationType(VisualizationType::PRIMITIVES);
        platform_.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    }

public:
	chrono::vehicle::gator::Gator platform_;
};
