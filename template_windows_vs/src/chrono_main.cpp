
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"

#include "my_platform.h"
#include "my_environment.h"
#include "my_sensors.h"

#include <iostream>
#include <vector>

using namespace chrono;
using namespace chrono::vehicle;
using std::cout;
using std::endl;
using std::vector;

using namespace chrono::geometry;


class SimConfig {
public:
    double step_size = 1e-3;
    double render_step_size = 1.0 / 50.0;
    double init_wait_time = 10.0;                // wait 10.0s to drive the car;
}sim_config;




int main(int argc, char* argv[]) {
    
    chrono::SetChronoDataPath("E:/codeGit/chrono/chrono/build/data/");              // change the default data loading path.
    chrono::vehicle::SetDataPath("E:/codeGit/chrono/chrono/build/data/vehicle/");              // change the vehicle data path

    // 1. Create the gator
    MyPlatform gator;
    std::cout << "==> Init gator. " << std::endl;

    // 2. Create terrain and surroundings.
    MyEnvironment env(gator.platform_.GetSystem());
    std::cout << "==> Init Env. " << std::endl;


    // 3. Create Vehicle Irrlicht interface.
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("MyGator");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 2.0), 5, 0);
    //vis->SetChaseCamera(ChVector<>(0.0, 0.0, 5.0), 20.0, 0);
    vis->Initialize();
    //vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&gator.platform_.GetVehicle());
    // Add path-follower
    auto ballT = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    ballT->SetColor(ChColor(0, 1, 0));
    int iballT = vis->AddVisualModel(ballT, ChFrame<>());

    double light_intensity = 0.5f;
    // ������Դ��Kd��Ks����Ӱ�졣
    vis->AddLight(ChVector<>(0, 0, 200), 300, ChColor(light_intensity, light_intensity, light_intensity));/*
    vis->AddLight(ChVector<>(-50, +50, 200), 300, ChColor(light_intensity, light_intensity, light_intensity));
    vis->AddLight(ChVector<>(+50, -50, 200), 300, ChColor(light_intensity, light_intensity, light_intensity));
    vis->AddLight(ChVector<>(+50, +50, 200), 300, ChColor(light_intensity, light_intensity, light_intensity));*/
 // 
    // ���û����⣬KaӰ����ʾ��
    double ambient_light_intensity = 128;
    vis->GetSceneManager()->setAmbientLight(irr::video::SColor(0, ambient_light_intensity, ambient_light_intensity, ambient_light_intensity));



    std::cout << "==> Init vehicle irr. " << std::endl;

    // 4. Create vehicle driver system
#ifdef DRIVER_INTERACTIVE
    ChInteractiveDriverIRR driver(*vis);
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(sim_config.render_step_size / steering_time);
    driver.SetThrottleDelta(sim_config.render_step_size / throttle_time);
    driver.SetBrakingDelta(sim_config.render_step_size / braking_time);
    driver.Initialize();
#else------------------------
    float target_speed = VEHICLE_SPEED;
    ChPathFollowerDriver driver(gator.platform_.GetVehicle(), gator.path_, "my_path", target_speed);
    driver.SetColor(ChColor(0.0f, 0.0f, 0.8f));
    driver.GetSteeringController().SetLookAheadDistance(2);
    driver.GetSteeringController().SetGains(0.8, 0, 0);     // SetGains (double Kp, double Ki, double Kd)
    driver.GetSpeedController().SetGains(0.4, 0.1, 0);
    driver.Initialize();
#endif
    std::cout << "==> Init driver. " << std::endl;

    // 5. Create Sensors.
    std::string lidar_output_folder = OUTPUT_FOLDER + "lidar/";
    MySensors sensors(gator.platform_, lidar_output_folder);
    std::cout << "==> Init sensors: lidar & imu. " << std::endl;

    // 6. Config the output
    std::string imu_filename = OUTPUT_FOLDER + "imu.csv";
    std::string gt_filename = OUTPUT_FOLDER + "gt.csv";
    std::ofstream imu_output(imu_filename);
    std::ofstream gt_output(gt_filename);
    if (!imu_output) {
        std::cout << "[Error]. Cannot save IMU into: " << imu_filename << std::endl;
        while (1) { ; }
    }
    if (!gt_output) {
        std::cout << "[Error]. Cannot save GT into: " << gt_filename << std::endl;
        while (1) { ; }
    }
    imu_output << "// Timestamp, acc-x, y, z, gyro-x, y, z(rad/s)" << std::endl;
    gt_output << "// Timestamp, pos-x, y, z, qw, qx, qy, qz" << std::endl;


    // Simulation control;
    int step_number = 0;
    bool is_first_loop = true;          // first loop, wait for some second without any input
    bool is_awayfrom_startopint = false;       // when move away from the start point, stop the vehicle when come back to the end point

    while (vis->Run()) {

        double time = gator.platform_.GetSystem()->GetChTime();
        auto chassis = gator.platform_.GetChassisBody();

        // 1. Update sensors.
        sensors.manager_.Update();
        // Get driver input. From Irrlicht-interface or Path PID controller. 
        DriverInputs driver_inputs = driver.GetInputs();
        if (time < sim_config.init_wait_time) {
            driver.SetDesiredSpeed(0.0f);           // keep speed 0 when waiting for stablizing
            driver_inputs.m_steering = 0.0f;
            driver_inputs.m_throttle = 0.0f;
            driver_inputs.m_braking = 0.0f;
        }
        else {
            if (is_first_loop) {
                driver.SetDesiredSpeed(target_speed);
                is_first_loop = false;
            }
        }

        // automatically stop the simulation if to the endpoint.
        // if moives away from the start point, stopped when back to the start point;
        if ((chassis->GetPos() - gator.start_point_).Length() > 10.0) {      // move away from the start point;
            is_awayfrom_startopint = true;
            //std::cout << "--> Moved away. " << std::endl;
        }
        if (is_awayfrom_startopint) {           // can be stopped
            auto dis = (chassis->GetPos() - gator.end_point_).Length();
            if (dis < 2.0) {
                std::cout << "--> To end point. The program stopped." << std::endl;
                break;
            }
        }


        // Output data to folder;
        // 1. Lidar output: already done by `gator.advance()`;
        // 2. IMU output to one file;
        if (time > sim_config.init_wait_time) {
            std::vector<double> acc, gyro;
            if (sensors.getIMU(acc, gyro)) {        // if IMU is updated
                imu_output << time << ", " << acc[0] << ", " << acc[1] << ", " << acc[2] << ", " << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << std::endl;
            }
            // 3. Ground-truth to a new file;
            chrono::ChVector<double> pos = gator.platform_.GetChassisBody()->GetPos();
            chrono::Quaternion rot = gator.platform_.GetChassisBody()->GetRot();
            gt_output << time << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
                << rot.e0() << ", " << rot.e1() << ", " << rot.e2() << ", " << rot.e3() << std::endl;
            // TODO: ISSUE. pos��rot��vehicle�ģ���SLAM������Lidar�ġ�����������
        }

        // step=1ms
        // Terminal output.
        if (step_number % 100 == 0) {                   // update vis 10Hz
            vis->UpdateVisualModel(iballT, ChFrame<>(driver.GetSteeringController().GetTargetLocation()));
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
        
        if (step_number % 1000 == 0)
            std::cout << "Real-world time: " << step_number << "ms." << std::endl;
        if (step_number % 1000 == 0) {                  // ouptut IMU/pos to terminal 1Hz
            vector<double> acc, gyro;
            if (sensors.getIMU(acc, gyro)) {
                cout << "Imu: Acc : " << acc[0] << ", " << acc[1] << ", " << acc[2] << endl;
                cout << "   : gyro: " << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << endl;
            }
            cout << chassis->GetPos() << ", " << chassis->GetRot() << endl;
        }


        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        env.terrain_.Synchronize(time);
        gator.platform_.Synchronize(time, driver_inputs, env.terrain_);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(sim_config.step_size);
        env.terrain_.Advance(sim_config.step_size);
        gator.platform_.Advance(sim_config.step_size);
        vis->Advance(sim_config.step_size);

        step_number++;
    }

    // close files;
    imu_output.close();
    gt_output.close();

    return 0;
}




//