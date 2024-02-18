

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
#include "chrono/assets/ChVisualShapeSurface.h"
#include "chrono/assets/ChVisualShapeCone.h"


#include <iostream>
#include <chrono>
#include <thread>   // for simulation time control;


using namespace chrono;
//using namespace chrono::geometry;
//using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// Lidar parameters
// -----------------------------------------------------------------------------

// Noise model attached to the sensor
enum NoiseModel {
    CONST_NORMAL_XYZI,  // Gaussian noise with constant mean and standard deviation
    NONE                // No noise model
};
NoiseModel noise_model = CONST_NORMAL_XYZI;


// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------




// Output directories
const std::string out_dir = "SENSOR_OUTPUT/LIDAR_DEMO/";


int main(int argc, char* argv[]) {

    chrono::SetChronoDataPath("E:/codeGit/chrono/chrono/build/data/");


    // -----------------
    // Create the system
    // -----------------
    chrono::ChSystemNSC sys;
    

    // add mesh

    // Create the rigid body (this won't move, it is only for visualization tests)
    auto body = chrono_types::make_shared<ChBody>();
    body->SetBodyFixed(true);
    sys.Add(body);
    auto objmesh = chrono_types::make_shared<ChVisualShapeModelFile>();
    //objmesh->SetFilename(GetChronoDataFile("C:/Users/larrydong/Desktop/scene.obj"));
    objmesh->SetFilename(GetChronoDataFile("models/forklift/body.obj"));
    
    body->AddVisualShape(objmesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));

    // ----------------------------------
    // add a mesh to be sensed by a lidar
    // ----------------------------------
    //mmesh->Transform(chrono::ChVector<>(0, 0, -2), chrono::ChMatrix33<>(1));  // scale to a different size
    //auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    //trimesh_shape->SetMesh(mmesh);
    //trimesh_shape->SetName("HMMWV Chassis Mesh");
    //trimesh_shape->SetMutable(false);
    //auto mesh_body = chrono_types::make_shared<ChBody>();
    //mesh_body->SetPos({ 0, 0, -2 });
    //mesh_body->AddVisualShape(trimesh_shape, ChFrame<>());
    //mesh_body->SetBodyFixed(true);
    //sys.Add(mesh_body);


    // --------------------------------------------
    // add a few box bodies to be sensed by a lidar
    // --------------------------------------------

    // TODO: box的配置不正确。显示的不合理。
    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, false);
    box_body->SetPos({ 0, 0, -100 });             // "ground"
    box_body->SetBodyFixed(true);
    sys.Add(box_body);


    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<sensor::ChSensorManager>(&sys);
    manager->SetVerbose(false);

    // -----------------------------------------------
    // Create a lidar and add it to the sensor manager
    // -----------------------------------------------
    auto lidar_initial_pose = chrono::ChFrame<double>({ 0, 0, 1 }, Q_from_AngAxis(0, { 1, 0, 0 }));

    // Lidar from JSON file - Velodyne VLP-16
    auto vlp16 = chrono::sensor::Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/Velodyne/HDL-32E.json"), box_body, lidar_initial_pose);       // Why need to  attach to a parent?
    manager->AddSensor(vlp16);




    // lidar move control
    
    bool b_need_change_lidar_pose= false;
    float lidar_last_update_time = 0;           // last time that update lidar pose;
    float lidar_x_speed = 2;                 // m/s;
    chrono::ChVector<> lidar_position(0, 0, 0);
    float lidar_angle_y(0.0f);

    float vibration_frequency = 10;         // Hz.
    float vibration_linear_acc_z = 0;       // z-direction acc, m/s2
    float vibration_angular_vel_y = 10*chrono::CH_C_DEG_TO_RAD;          // y-axis angular velocity, rad/s
    
    //float r = 0, r_old = 0;                 // a random value;

    // time control;
    float simulation_step_size = 1e-3;      // Simulation step size
    float simulation_end_time = 20.0f;
    float ch_time = 0.0;                    // "current" simulation time
    float ch_time_1s = 0.0;

    // use std::chrono to precisely control the simulation time;
    std::chrono::milliseconds world_time_1ms((int)(simulation_step_size * 1000));
    std::chrono::steady_clock::time_point world_time_next = std::chrono::steady_clock::now() + world_time_1ms;
    

    // visualization
    auto vis = chrono_types::make_shared<chrono::irrlicht::ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Chrono::Irrlicht visualization");
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(-2, 3, -4));
    vis->AddTypicalLights();
    //vis->AddGrid(0.5, 0.5, 12, 12, ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(CH_C_PI_2)),ChColor(0.31f, 0.43f, 0.43f));


    while (ch_time < simulation_end_time && vis->Run()) {
        //manager->Update();
        vis->BeginScene();
        vis->Render();

   //     // check if need to change lidar pose;
   //     if (ch_time - lidar_last_update_time > 1.0f / vibration_frequency) {
   //         b_need_change_lidar_pose= true;
   //         lidar_last_update_time = ch_time;
   //     }
   //     if (b_need_change_lidar_pose){
   //         float r = 2*((double)std::rand() / (RAND_MAX)) - 1;        // r: (-1,1); generate a random variable.
   //         // update position. Add moving speed;
   //         lidar_position += chrono::ChVector<>(lidar_x_speed * simulation_step_size, 0, 0);
   //         // update position. Add z-axis vibration;
			//chrono::ChVector<> d_pos_z(0, 0, r * (0.5f * vibration_linear_acc_z * simulation_step_size * simulation_step_size));        // 1/2*a*t^2
   //         lidar_position += d_pos_z;
   //         // update rotation. Add y-axis angular shake;
   //         float d_ang_y = r * (vibration_angular_vel_y * simulation_step_size);
   //         lidar_angle_y += d_ang_y;
   //         // update lidar pose.
   //         chrono::ChQuaternion lidar_q = chrono::Q_from_AngAxis(lidar_angle_y, chrono::ChVector(0, 1, 0));
   //         auto lidar_frame = chrono::ChFrame(lidar_position, lidar_q);
   //         vlp16->SetOffsetPose(lidar_frame);
   //     }

        // Perform step of dynamics
        sys.DoStepDynamics(simulation_step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();


   //     // real-time control
   //     std::this_thread::sleep_until(world_time_next);
   //     world_time_next += world_time_1ms;

        if (ch_time - ch_time_1s > 1) {
            ch_time_1s = ch_time;
            std::cout << "Current Simulation time: " << int(ch_time) << " s." << std::endl;
        }
        

        vis->EndScene();
    }

    return 0;
}
