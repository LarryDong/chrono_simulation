
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

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


class SimConfig {
public:
    double step_size = 1e-3;
    double render_step_size = 1.0 / 50.0;
}sim_config;

// Output: lidar, IMU, gt.
const std::string output_folder_base = "C:/Users/larrydong/Desktop/chrono_output/";


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
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 2.0), 5.0, 0);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&gator.platform_.GetVehicle());
    std::cout << "==> Init vehicle irr. " << std::endl;

    // 4. Create vehicle driver system
    ChInteractiveDriverIRR driver(*vis);
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(sim_config.render_step_size / steering_time);
    driver.SetThrottleDelta(sim_config.render_step_size / throttle_time);
    driver.SetBrakingDelta(sim_config.render_step_size / braking_time);
    driver.Initialize();
    std::cout << "==> Init driver. " << std::endl;

    // 5. Create Sensors.
    std::string lidar_output_folder = output_folder_base + "lidar/";
    MySensors sensors(gator.platform_, lidar_output_folder);
    sensors.li
    std::cout << "==> Init sensors: lidar & imu. " << std::endl;

    // 6. Config the output
    std::string imu_filename = output_folder_base + "imu.csv";
    std::string gt_filename = output_folder_base + "gt.csv";
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

    int step_number = 0;
    while (vis->Run()){

        double time = gator.platform_.GetSystem()->GetChTime();

        // 1. Update sensors.
        sensors.manager_.Update();

        // Get driver input
        DriverInputs driver_inputs = driver.GetInputs();

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



        // Output data to folder;
        // 1. Lidar output: already done by `gator.advance()`;
        // 2. IMU output to one file;
        std::vector<double> acc, gyro;
        if (sensors.getIMU(acc, gyro)) {        // if IMU is updated
            imu_output << time << ", " << acc[0] << ", " << acc[1] << ", " << acc[2] << ", " << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << std::endl;
        }
        // 3. Ground-truth to a new file;
        chrono::ChVector<double> pos = gator.platform_.GetChassisBody()->GetPos();
        chrono::Quaternion rot = gator.platform_.GetChassisBody()->GetRot();
        gt_output << time << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
            << rot.e0() << ", " << rot.e1() << ", " << rot.e2() << ", " << rot.e3() << std::endl;


        // some visualization and terminal output;
        if (step_number % 10 == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
        if (step_number % 1000 == 0) {
            vector<double> acc, gyro;
            if (sensors.getIMU(acc, gyro)) {
                cout << "Imu: Acc : " << acc[0] << ", " << acc[1] << ", " << acc[2] << endl;
                cout << "   : gyro: " << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << endl;
            }
            auto chassis = gator.platform_.GetChassisBody();
            cout << chassis->GetPos() << ", " << chassis->GetRot() << endl;
        }
    }

    // close files;
    imu_output.close();
    gt_output.close();


    return 0;
}