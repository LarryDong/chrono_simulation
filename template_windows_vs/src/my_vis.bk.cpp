// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// Chrono demonstration of a lidar sensor
// Simple demonstration of certain filters and the visualization of a static mesh
//
// =============================================================================

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

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// Lidar parameters
// -----------------------------------------------------------------------------

// Noise model attached to the sensor
enum NoiseModel {
    CONST_NORMAL_XYZI,  // Gaussian noise with constant mean and standard deviation
    NONE                // No noise model
};
NoiseModel noise_model = CONST_NORMAL_XYZI;

// Lidar return mode
// Either STRONGEST_RETURN, MEAN_RETURN, FIRST_RETURN, LAST_RETURN
LidarReturnMode return_mode = LidarReturnMode::STRONGEST_RETURN;

// Update rate in Hz
float update_rate = 5.f;

// Number of horizontal and vertical samples
unsigned int horizontal_samples = 4500;
unsigned int vertical_samples = 32;

// Horizontal and vertical field of view (radians)
float horizontal_fov = (float)(2 * CH_C_PI);  // 360 degree scan
float max_vert_angle = (float)CH_C_PI / 12;   // 15 degrees up
float min_vert_angle = (float)-CH_C_PI / 6;   // 30 degrees down

// Lag time
float lag = 0.f;

// Collection window for the lidar
float collection_time = 1 / update_rate;  // typically 1/update rate

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-3;

// Simulation end time
float end_time = 2000.0f;

// Save lidar point clouds
bool save = false;

// Render lidar point clouds
bool vis = true;

// Output directories
const std::string out_dir = "SENSOR_OUTPUT/LIDAR_DEMO/";

int main(int argc, char* argv[]) {

    chrono::SetChronoDataPath("E:/codeGit/chrono/chrono/build/data/");
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    // ----------------------------------
    // add a mesh to be sensed by a lidar
    // ----------------------------------
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("HMMWV Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({ 0, 0, 0 });
    mesh_body->AddVisualShape(trimesh_shape, ChFrame<>());
    mesh_body->SetBodyFixed(true);
    sys.Add(mesh_body);

    // --------------------------------------------
    // add a few box bodies to be sensed by a lidar
    // --------------------------------------------

     //~ Add the vis the same time;
    auto vis = chrono_types::make_shared<chrono::irrlicht::ChVisualSystemIrrlicht>();  // visual system has many `vis` engine, irrlicht is one.
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Chrono::Irrlicht visualization");
    vis->Initialize();
    vis->AddCamera({ 0,0,50 }, { 0,0,-1 });
    vis->AddTypicalLights();
    vis->AddGrid(0.5, 0.5, 12, 12, ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(CH_C_PI_2)),
        ChColor(0.31f, 0.43f, 0.43f));


    // Vis the body.
    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(100, 100, 1, 1000, true, false);
    box_body->SetPos({ 0, 0, -1 });
    box_body->SetBodyFixed(true);
    sys.Add(box_body);

    auto b1_vis = chrono_types::make_shared<chrono::ChVisualShapeBox>(100, 100, 1);
    b1_vis->SetColor(chrono::ChColor(0.2f, 0.5f, 0));
    box_body->AddVisualShape(b1_vis, ChFrame<>({ 0,0,0 }, QUNIT));
    b1_vis->SetVisible(true);


    auto box_body_1 = chrono_types::make_shared<ChBodyEasyBox>(100, 1, 100, 1000, true, false);
    box_body_1->SetPos({ 0, -10, -3 });
    box_body_1->SetBodyFixed(true);
    sys.Add(box_body_1);

    auto box_body_2 = chrono_types::make_shared<ChBodyEasyBox>(100, 1, 100, 1000, true, false);
    box_body_2->SetPos({ 0, 10, -3 });
    box_body_2->SetBodyFixed(true);
    sys.Add(box_body_2);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->SetVerbose(false);

    // -----------------------------------------------
    // Create a lidar and add it to the sensor manager
    // -----------------------------------------------
    auto offset_pose = chrono::ChFrame<double>({ -4, 0, 1 }, Q_from_AngAxis(0, { 0, 1, 0 }));


    auto hdl32e = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/Velodyne/HDL-32E.json"), box_body, offset_pose);
    manager->AddSensor(hdl32e);

    // ---------------
    // Simulate system
    // ---------------
    float orbit_rate = 2.5;
    float ch_time = 0.0;

    UserDIBufferPtr di_ideal_ptr;

    UserXYZIBufferPtr xyzi_buf;
    UserXYZIBufferPtr xyzi_buf_old;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time && vis->Run()) {
        mesh_body->SetRot(Q_from_AngAxis(ch_time * orbit_rate, { 0, 0, 1 }));

        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Update Vis
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Perform step of dynamics
        sys.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();
    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}
