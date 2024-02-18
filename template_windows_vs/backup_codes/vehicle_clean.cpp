
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include <iostream>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50



int main(int argc, char* argv[]) {

    chrono::SetChronoDataPath("E:/codeGit/chrono/chrono/build/data/");              // change the default data loading path.
    chrono::vehicle::SetDataPath("E:/codeGit/chrono/chrono/build/data/vehicle/");              // change the vehicle data path

    ChContactMethod contact_method = ChContactMethod::SMC;
    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(contact_method);
    hmmwv.SetChassisCollisionType(CollisionType::NONE);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>({ 0, 0, 0.5 }, { 1, 0, 0, 0 }));
    hmmwv.SetEngineType(EngineModelType::SHAFTS);
    hmmwv.SetTransmissionType(TransmissionModelType::SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.UseTierodBodies(true);
    hmmwv.SetSteeringType(SteeringTypeWV::PITMAN_ARM);
    hmmwv.SetBrakeType(BrakeType::SHAFTS);
    hmmwv.SetTireType(TireModelType::PAC02);
    hmmwv.SetTireStepSize(tire_step_size);
    hmmwv.Initialize();

    // Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
    VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
    VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
    VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
    VisualizationType wheel_vis_type = VisualizationType::PRIMITIVES;
    VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;
    hmmwv.SetChassisVisualizationType(chassis_vis_type);
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    hmmwv.SetSteeringVisualizationType(steering_vis_type);
    hmmwv.SetWheelVisualizationType(wheel_vis_type);
    hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(hmmwv.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    // Rigid terrain
    double terrainHeight = 0;      // terrain height (FLAT terrain only)
    double terrainLength = 200.0;  // size in X direction
    double terrainWidth = 200.0;   // size in Y direction
    std::shared_ptr<RigidTerrain::Patch> patch;
    patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 200, 200);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    terrain.Initialize();


    // ------------------------------------------------------------------------------
    // Create the vehicle run-time visualization interface and the interactive driver
    // ------------------------------------------------------------------------------
    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1

    std::shared_ptr<ChVehicleVisualSystem> vis;
    std::shared_ptr<ChDriver> driver;
    // Create the vehicle Irrlicht interface
    auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();       //~ ChWheeled这个类继承了可视化的基类
    vis_irr->SetWindowTitle("HMMWV Demo");
    vis_irr->SetChaseCamera({ 0.0, 0.0, 1.75 }, 6.0, 0.5);
    vis_irr->Initialize();
    vis_irr->AddLightDirectional();
    vis_irr->AddSkyBox();
    vis_irr->AddLogo();
    vis_irr->AttachVehicle(&hmmwv.GetVehicle());

    // Create the interactive Irrlicht driver system
    auto driver_irr = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
    driver_irr->SetSteeringDelta(render_step_size / steering_time);
    driver_irr->SetThrottleDelta(render_step_size / throttle_time);
    driver_irr->SetBrakingDelta(render_step_size / braking_time);
    driver_irr->Initialize();
    vis = vis_irr;
    driver = driver_irr;

    // ---------------
    // Simulation loop
    // ---------------
    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    hmmwv.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        double time = hmmwv.GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output post-processing data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver->GetInputs();

        // Update modules (process inputs from other modules)
        driver->Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);            //~ 更新vis的trackpoint等。

        // Increment frame number
        step_number++;
        std::cout << "Step: " << step_number << std::endl;
    }
    return 0;
}
