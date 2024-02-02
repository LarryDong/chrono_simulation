
// This code load a 3D map from a .obj file, which is generated by Meshlab from real Lidar's Point cloud;
// 2024.02.02, last update by Larry Dong.


#include <cmath>
#include <cstdio>
#include <iomanip>

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"


#include <chrono>           // c++'s std chrono. For time control
#include <thread>


using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;


// Output directories
const std::string out_dir = "SENSOR_OUTPUT/LIDAR_DEMO/";

int main(int argc, char* argv[]) {

    chrono::SetChronoDataPath("E:/codeGit/chrono/chrono/build/data/");              // change the default data loading path.
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;


    // --------------------------------------------
    // add a few box bodies to be sensed by a lidar
    // --------------------------------------------
    // Vis the body.
    //auto box_body = chrono_types::make_shared<ChBodyEasyBox>(50, 50, 1, 1000, true, false);
    //box_body->SetPos({ 0, 0, -1 });
    //box_body->SetBodyFixed(true);
    //sys.Add(box_body);

    //auto b1_vis = chrono_types::make_shared<chrono::ChVisualShapeBox>(50, 50, 1);
    //b1_vis->SetColor(chrono::ChColor(0.0f, 1.0f, 0));
    //box_body->AddVisualShape(b1_vis, ChFrame<>({ 0,0,0 }, QUNIT));
    //b1_vis->SetVisible(true);


    // -----------------------
    // add the environment 3D map from lidar-scan
    // -----------------------
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile("C:/Users/larrydong/Desktop/scene.obj", false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));                         // scale to a different size
    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("3D Scene");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({ 0, 0, 0 });
    mesh_body->AddVisualShape(trimesh_shape, ChFrame<>());
    mesh_body->SetBodyFixed(true);
    sys.Add(mesh_body);


    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->SetVerbose(false);

    // -----------------------------------------------
    // Create a lidar and add it to the sensor manager
    // -----------------------------------------------
    float lidar_size_x = 1;
    float lidar_size_y = 1;
    float lidar_size_z = 1;
    auto lidar_body = chrono_types::make_shared<ChBodyEasyBox>(lidar_size_x, lidar_size_y, lidar_size_z, 1, true, false);
    auto lidar_body_vis = chrono_types::make_shared<chrono::ChVisualShapeBox>(lidar_size_x, lidar_size_y, lidar_size_z);
    lidar_body_vis->SetColor(chrono::ChColor(1.0f, 0.0f, 0.0f));
    lidar_body_vis->SetVisible(true);               // this may block lidar's scan, set to false for final output.
    lidar_body->AddVisualShape(lidar_body_vis, ChFrame<>({ 0,0,0 }, QUNIT));
    lidar_body->SetBodyFixed(false);        // if the lidar need to move, cannot be set true;
    sys.Add(lidar_body);
    // attach a lidar to lidar_body;
    auto offset_pose = chrono::ChFrame<double>({ 0,0,5 }, Q_from_AngAxis(0, { 1, 0, 0 }));
    auto hdl32e = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/Velodyne/HDL-32E.json"), lidar_body, offset_pose);
    manager->AddSensor(hdl32e);

    // ---------------
    // Simulate system
    // ---------------
    float ch_time = 0.0, end_time = 20.0;
    float step_size = 0.005;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    // ---------------
    // Add the vis module
    // ---------------
    auto vis = chrono_types::make_shared<chrono::irrlicht::ChVisualSystemIrrlicht>();  // visual system has many `vis` engine, irrlicht is one.
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Chrono::Irrlicht visualization");
    vis->Initialize();
    vis->AddCamera({ 0,0,50 }, { 0,0,-1 });
    vis->AddTypicalLights();
    vis->AddSkyBox();
    

    //-------------------------
    // Real-time control. To control the simulation time is same as world time.
    //-------------------------
    float ch_time_1s = 0.0;
    std::chrono::milliseconds world_time_1ms((int)(step_size * 1000));
    std::chrono::steady_clock::time_point world_time_next = std::chrono::steady_clock::now() + world_time_1ms;



    while (ch_time < end_time && vis->Run()) {

        // move the "lidar" based on time
        if (ch_time_1s <= 4) {
            lidar_body->SetPos_dt({ 0.5,0,0 });
        }
        else if (ch_time_1s > 4 && ch_time_1s <= 9) {
            lidar_body->SetPos_dt({ 0, 0.5, 0 });
        }
        else if (ch_time_1s > 9 && ch_time_1s <= 12) {
            lidar_body->SetPos_dt({ 0,0,0.8 });
        }
        else {
            lidar_body->SetPos_dt({ -0.5,0,-0.5 });
            return 0;
        }

        // Update sensor manager
        manager->Update();

        // Update Vis
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Perform step of dynamics
        sys.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();

		// real-time control
		std::this_thread::sleep_until(world_time_next);
		world_time_next += world_time_1ms;
        if (ch_time - ch_time_1s > 1) {
            ch_time_1s = ch_time;
            std::cout << "Current Simulation time: " << int(ch_time) << " s." << std::endl;
        }

    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}
