
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors//ChSensorBuffer.h"
#include <iostream>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::gator;
using namespace chrono::sensor;


// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::PRIMITIVES;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Collision type for chassis (PRIMITIVES, HULLS, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// SENSOR PARAMETERS
// Save sensor data
bool sensor_save = false;

// Visualize sensor data
bool sensor_vis = true;


// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    chrono::SetChronoDataPath("E:/codeGit/chrono/chrono/build/data/");              // change the default data loading path.
    chrono::vehicle::SetDataPath("E:/codeGit/chrono/chrono/build/data/vehicle/");              // change the vehicle data path

    // --------------
    // Create vehicle
    // --------------
    Gator gator;
    gator.SetContactMethod(contact_method);
    gator.SetChassisCollisionType(chassis_collision_type);
    gator.SetChassisFixed(false);
    gator.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    gator.SetTireType(tire_model);
    gator.SetTireStepSize(tire_step_size);
    gator.SetAerodynamicDrag(0.5, 5.0, 1.2);
    gator.Initialize();
    gator.SetChassisVisualizationType(chassis_vis_type);
    gator.SetSuspensionVisualizationType(suspension_vis_type);
    gator.SetSteeringVisualizationType(steering_vis_type);
    gator.SetWheelVisualizationType(wheel_vis_type);
    gator.SetTireVisualizationType(tire_vis_type);
    // Associate a collision system
    gator.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(gator.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    std::shared_ptr<RigidTerrain::Patch> patch;
    patch = terrain.AddPatch(patch_mat, CSYSNORM, 100.0, 100.0);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    terrain.Initialize();

    auto ground_body = patch->GetGroundBody();      // ground_body: ChBody 是地形的ChBody

    auto vis_mat1 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat1->SetKdTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"));         // Kd, Ks, Ke：一些材质的反射、发光等设定。
    vis_mat1->SetWeightTexture(GetChronoDataFile("sensor/textures/weight1.png"));       // 两种地形材料的混合权重。weight1和2是互补的。
    vis_mat1->SetSpecularColor({ .0f, .0f, .0f });
    vis_mat1->SetRoughness(1.f);
    vis_mat1->SetUseSpecularWorkflow(false);

    auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat2->SetKdTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"));
    vis_mat2->SetWeightTexture(GetChronoDataFile("sensor/textures/weight2.png"));
    vis_mat2->SetSpecularColor({ .0f, .0f, .0f });
    vis_mat2->SetRoughness(1.f);
    vis_mat2->SetUseSpecularWorkflow(false);

    auto visual_shape = ground_body->GetVisualModel()->GetShape(0);
    visual_shape->SetMaterial(0, vis_mat1);         // 将第i个material设置为某个特定的material
    visual_shape->AddMaterial(vis_mat2);

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // -------------------------------------
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Gator Demo");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 2.0), 5.0, 0);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&gator.GetVehicle());

    // ------------------------
    // Create the driver system
    // ------------------------
    // Create the interactive driver system
    ChInteractiveDriverIRR driver(*vis);
    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);
    driver.Initialize();


    // 传感器部分
    auto manager = chrono_types::make_shared<ChSensorManager>(gator.GetSystem());
    manager->scene->AddPointLight({ 100, 100, 100 }, { 2, 2, 2 }, 5000);
    auto lidar_extrinsics = chrono::ChFrame<double>({ 0, 0, 1 }, Q_from_AngAxis(0, { 1, 0, 0 }));
    // Lidar from JSON file - Velodyne VLP-16
    auto vlp16 = chrono::sensor::Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/Velodyne/HDL-32E.json"), gator.GetChassisBody(), lidar_extrinsics);
    vlp16->PushFilter(chrono_types::make_shared<ChFilterSavePtCloud>("C:/Users/larrydong/Desktop/lidar_output/"));  // 包含这个filter，在每次Apply对应的filter时(update中)，会执行保存到文件，并输出Beam count
    manager->AddSensor(vlp16);

    // ---------------
    // Simulation loop
    // ---------------
    // output vehicle mass
    std::cout << "VEHICLE MASS: " << gator.GetVehicle().GetMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    float orbit_radius = 10.f;
    float orbit_rate = 1;

    ChRealtimeStepTimer realtime_timer;         // Ch
    while (vis->Run()) {
        double time = gator.GetSystem()->GetChTime();

        chrono::sensor::UserXYZIBufferPtr data_ptr;
        data_ptr = vlp16->GetMostRecentBuffer< chrono::sensor::UserXYZIBufferPtr>();
        if (data_ptr->Buffer) {
            using namespace std;
            cout << "==>    Width: " << data_ptr->Width << ", height: " << data_ptr->Height << ",  ts: " << data_ptr->TimeStamp << ", launched count: " << data_ptr->LaunchedCount << endl;
        }

        manager->Update();

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();              // 这一步更新的lidar？
            vis->EndScene();
            render_frame++;
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        gator.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        gator.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
