

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

//#define USE_JSON

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::irrlicht;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

// =============================================================================

int main(int argc, char* argv[]) {

    chrono::SetChronoDataPath("E:/codeGit/chrono/chrono/build/data/");              // change the default data loading path.
    chrono::vehicle::SetDataPath("E:/codeGit/chrono/chrono/build/data/vehicle/");              // change the vehicle data path

    // Simulation step sizes
    double step_size = 3e-3;
    double tire_step_size = 1e-3;

    // --------------
    // Create systems
    // --------------
    // Create the HMMWV vehicle, set parameters, and initialize
    // 设置vehicle的物理模型、与可视化模型。
    HMMWV_Reduced hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(ChContactMethod::SMC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetChassisCollisionType(CollisionType::NONE);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(-10, 0, 1), ChQuaternion<>(1, 0, 0, 0)));
    hmmwv.SetEngineType(EngineModelType::SIMPLE);
    hmmwv.SetTransmissionType(TransmissionModelType::SIMPLE_MAP);
    hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv.SetBrakeType(BrakeType::SHAFTS);
    hmmwv.SetTireType(TireModelType::TMEASY);
    hmmwv.SetTireStepSize(tire_step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);


#ifdef USE_JSON
    // Create the terrain from JSON specification file
    RigidTerrain terrain(hmmwv.GetSystem(), "C:/Users/larrydong/Desktop/my_terrain.json");      // TODO: HHHHHHHHHHHHHHHHHHHHHHHHHHHH
#else
    // Create the terrain patches programatically
    RigidTerrain terrain(hmmwv.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);       // 土壤杨氏模量：100-5000
    auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector<>(20, 0, 0), QUNIT), "C:/Users/larrydong/Desktop/my_terrain.bmp", 64.0, 64.0, 0.0, 0.1);   // 64x64，纯黑像素0，纯白1
    //auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector<>(20, 0, 0), QUNIT), vehicle::GetDataFile("terrain/meshes/test.obj"));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 6.0f, 6.0f);
    terrain.Initialize();
#endif


    // Set the time response for steering and throttle keyboard inputs.
    double render_step_size = 1.0 / 50;  // FPS = 50
    double steering_time = 1.0;          // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;          // time to go from 0 to +1
    double braking_time = 0.3;           // time to go from 0 to +1

    // Create the vehicle run-time visualization interface and the interactive driver
    std::shared_ptr<ChVehicleVisualSystem> vis;
    std::shared_ptr<ChDriver> driver;

    // Create the vehicle Irrlicht interface
    auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis_irr->SetWindowTitle("Rigid Terrain Demo");
    vis_irr->SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.75);      // 追踪 wheel vehicle 的相机，分别是：旋转，
    vis_irr->Initialize();
    vis_irr->AddLightDirectional();
    vis_irr->AddSkyBox();
    vis_irr->AddLogo();
    vis_irr->AttachVehicle(&hmmwv.GetVehicle());

    auto driver_irr = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
    driver_irr->SetSteeringDelta(render_step_size / steering_time);
    driver_irr->SetThrottleDelta(render_step_size / throttle_time);
    driver_irr->SetBrakingDelta(render_step_size / braking_time);
    driver_irr->Initialize();
    vis = vis_irr;
    driver = driver_irr;


    hmmwv.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        double time = hmmwv.GetSystem()->GetChTime();

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Get driver inputs
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
        vis->Advance(step_size);
    }

    return 0;
}
