// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Demonstration of a steering path-follower PID controller with two alternatives.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;


// Contact method type
ChContactMethod contact_method = ChContactMethod::SMC;

// Type of tire model (RIGID, FIALA, PAC89, PAC02, or TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Type of engine model (SHAFTS, SIMPLE, SIMPLE_MAP)
EngineModelType engine_model = EngineModelType::SHAFTS;

// Type of transmission model (SHAFTS, SIMPLE_MAP)
TransmissionModelType transmission_model = TransmissionModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::RWD;

// Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
// Note: Compliant steering requires higher PID gains.
SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Input file names for the path-follower driver model
std::string path_file("paths/my_path.txt");

// Set to true for a closed-loop path and false for an open-loop
bool closed_loop = false;

// Desired vehicle speed (m/s)
double target_speed = 12;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 300.0;  // size in X direction
double terrainWidth = 300.0;   // size in Y direction

// Point on chassis tracked by the chase camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step size
double step_size = 2e-3;
double tire_step_size = 1e-3;

// Simulation end time
double t_end = 100;

// Render FPS
double fps = 60;

// POV-Ray output
bool povray_output = false;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = true;
int filter_window_size = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    chrono::SetChronoDataPath("E:/codeGit/chrono/chrono/build/data/");              // change the default data loading path.
    chrono::vehicle::SetDataPath("E:/codeGit/chrono/chrono/build/data/vehicle/");              // change the vehicle data path

    // ----------------------
    // Create the Bezier path
    // ----------------------
    // From data file
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file), closed_loop);
    // Bezier曲线文件：x行 y列，y=3/9. 3：节点坐标；9：节点，incoming控制点，outcoming控制点
    // read是一个静态公有成员函数。即使没有创建类的对象，也可以调用这些函数。

    auto point0 = path->getPoint(0);
    auto point1 = path->getPoint(1);

    ChVector<> initLoc = point0;
    initLoc.z() = 0.5;
    ChQuaternion<> initRot = Q_from_AngZ(std::atan2(point1.y() - point0.y(), point1.x() - point0.x()));


    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------
    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(contact_method);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    hmmwv.SetEngineType(engine_model);
    hmmwv.SetTransmissionType(transmission_model);
    hmmwv.SetDriveType(drive_type);
    hmmwv.SetSteeringType(steering_type);
    hmmwv.SetTireType(tire_model);
    hmmwv.SetTireStepSize(tire_step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(chassis_vis_type);
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    hmmwv.SetSteeringVisualizationType(steering_vis_type);
    hmmwv.SetWheelVisualizationType(wheel_vis_type);
    hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(hmmwv.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 0.8f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);    // thinkness默认1.0
    patch->SetColor(ChColor(1, 0.5, 0.5));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // ------------------------
    // Create the driver system
    // ------------------------
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "my_path", target_speed);     // ChDriver->ChClosedLoopDriver->ChPathFollowerDriver
    driver.SetColor(ChColor(0.0f, 0.0f, 0.8f));
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.8, 0, 0);     // SetGains (double Kp, double Ki, double Kd)
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();


    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetLogLevel(irr::ELL_NONE);
    vis->AttachVehicle(&hmmwv.GetVehicle());
    vis->SetWindowTitle("Steering PID Controller Demo");
    vis->SetHUDLocation(500, 20);       // HUD: 平视显示系统
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AddLight(ChVector<>(-150, -150, 200), 300, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(-150, +150, 200), 300, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(+150, -150, 200), 300, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(+150, +150, 200), 300, ChColor(0.7f, 0.7f, 0.7f));

    // Visualization of controller points (sentinel & target)
    // Sentinel（哨兵）: 通常用来监视或标记某个特定区域、路径或条件的存在。在仿真或游戏环境中，一个哨兵点可能用来指示玩家或系统需要特别注意的地方，或者作为触发某些事件的标记。
    // Target（目标）: 通常指一个要达到或影响的点。在仿真中，这可能是需要导航到的位置，或者是需要与之交互的对象。
    // The base class implements the basic functionality to control the error between the location of a sentinel point (a point at a look-ahead distance in front of the vehicle) and the current target point.

    auto ballS = chrono_types::make_shared<ChVisualShapeSphere>(0.1);           // sentinel 哨兵
    auto ballT = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    ballS->SetColor(ChColor(1, 0, 0));
    ballT->SetColor(ChColor(0, 1, 0));
    int iballS = vis->AddVisualModel(ballS, ChFrame<>());
    int iballT = vis->AddVisualModel(ballT, ChFrame<>());


    // GC: gravity center，质心/底盘的加速度；  driver：驾驶员位置的加速度
    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);      // fwd: forward,
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);      // lat: latitute
    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

    // ---------------
    // Simulation loop
    // ---------------
    // Driver location in vehicle local frame
    ChVector<> driver_pos = hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;

    // Number of simulation steps between miscellaneous events
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int sim_frame = 0;
    int render_frame = 0;

    hmmwv.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        // Extract system state
        double time = hmmwv.GetSystem()->GetChTime();
        ChVector<> acc_CG = hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();         // 获取车体地盘质心的加速度
        ChVector<> acc_driver = hmmwv.GetVehicle().GetPointAcceleration(driver_pos);    // 获取驾驶员所在位置点的加速度
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());          // 这是一步滤波，获取窗口范围内的平均加速度
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());

        // End simulation
        if (time >= t_end)
            vis->Quit();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update sentinel and target location markers for the path-follower controller.
        vis->UpdateVisualModel(iballS, ChFrame<>(driver.GetSteeringController().GetSentinelLocation()));
        vis->UpdateVisualModel(iballT, ChFrame<>(driver.GetSteeringController().GetTargetLocation()));

        vis->BeginScene();
        vis->Render();
        vis->EndScene();


        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Increment simulation frame number
        sim_frame++;
    }

    return 0;
}
