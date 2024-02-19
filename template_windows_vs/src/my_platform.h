	
#pragma once

#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"       // keyboard control
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"         // file input
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_thirdparty/filesystem/path.h"

class MyPlatform {

public:
    MyPlatform(void) {

        using namespace chrono;
        using namespace chrono::vehicle;
        using namespace chrono::irrlicht;


        // Load path file;
        bool is_close_loop = true;
        path_ = ChBezierCurve::read("E:/codeGit/chrono_simulation/template_windows_vs/matlab_scripts/trajectory_control_points.txt", is_close_loop);     //
        chrono::ChVector<> point0 = path_->getPoint(0);
        chrono::ChVector<> point1 = path_->getPoint(1);
        start_point_ = point0;
        end_point_ = path_->getPoint(path_->getNumPoints() - 1);
        ChVector<> initLoc = point0;
        initLoc.z() = 0.5;
        ChQuaternion<> initRot = Q_from_AngZ(std::atan2(point1.y() - point0.y(), point1.x() - point0.x()));

        // create car;
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
    std::shared_ptr<chrono::ChBezierCurve> path_;
    chrono::ChVector<> start_point_, end_point_;

};
