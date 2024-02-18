#pragma once
//
//auto extrinsics_lidar = chrono::ChFrame<double>({ 0, 0, 1 }, Q_from_AngAxis(0, { 1, 0, 0 }));
//auto extrinsics_imu = chrono::ChFrame<double>({ 0, 0, 1 }, Q_from_AngAxis(0, { 1, 0, 0 }));


#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"
#include "chrono_models/vehicle/gator/Gator.h"

#include <iostream>
#include <vector>

class MySensors {

public:
	MySensors(chrono::vehicle::gator::Gator& platform) : manager_(platform.GetSystem()){
	
		using namespace chrono;
		using namespace chrono::sensor;
		//manager_.scene->AddPointLight({ 100, 100, 100 }, { 2, 2, 2 }, 5000);


		//// Create Lidar. From JSON
		auto lidar_ext = chrono::ChFrame<double>({ 0, 0, 1 }, Q_from_AngAxis(0, { 1, 0, 0 }));
		lidar_ = chrono::sensor::Sensor::CreateFromJSON("E:/codeGit/chrono_simulation/template_windows_vs/configs/Lidar.json", platform.GetChassisBody(), lidar_ext);
		// TODO: save lidar's data;
		manager_.AddSensor(lidar_);
		
		// Create IMU;
		auto imu_ext = chrono::ChFrame<double>({ 0, 0, 1 }, Q_from_AngAxis(0, { 1, 0, 0 }));
		const float imu_update_rate = 100.0f;

		std::shared_ptr<ChNoiseModel> acc_noise_model, gyro_noise_model;
		acc_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,
							ChVector<double>({ 0., 0., 0. }),           // mean,
							ChVector<double>({ 0.001, 0.001, 0.001 }),  // stdev,
							.0001,                                    // bias_drift,
							.1);                                      // tau_drift,
		gyro_noise_model = chrono_types::make_shared<ChNoiseNormalDrift>(imu_update_rate,
							ChVector<double>({ 0., 0., 0. }),  // float mean,
							ChVector<double>({ 0.001, 0.001, 0.001 }),  // float
							.001,  // double bias_drift,
							.1);   // double tau_drift,

		acc_ = chrono_types::make_shared<ChAccelerometerSensor>(platform.GetChassisBody(),    // body to which the IMU is attached
					imu_update_rate,   // update rate
					imu_ext,			// offset pose from body
					acc_noise_model);  // IMU noise model
		acc_->SetName("IMU - Accelerometer");
		acc_->SetLag(0.0f);
		acc_->SetCollectionWindow(0);
		acc_->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());  // Add a filter to access the imu data
		manager_.AddSensor(acc_);                                            // Add the IMU sensor to the sensor manager

		gyro_ = chrono_types::make_shared<ChGyroscopeSensor>(platform.GetChassisBody(),     // body to which the IMU is attached
					imu_update_rate,    // update rate
					imu_ext,    // offset pose from body
					gyro_noise_model);  // IMU noise model
		gyro_->SetName("IMU - Accelerometer");
		gyro_->SetLag(0);
		gyro_->SetCollectionWindow(0);
		gyro_->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());  // Add a filter to access the imu data
		manager_.AddSensor(gyro_);                                           // Add the IMU sensor to the sensor manager
	}


	void printIMU(void);
	bool getIMU(std::vector<double>& accs, std::vector<double>& gyros);

public:
	chrono::sensor::ChSensorManager manager_;

private:
	//const chrono::ChFrame<> lidar_ext_, imu_ext_;
	std::shared_ptr<chrono::sensor::ChSensor> lidar_;
	std::shared_ptr< chrono::sensor::ChAccelerometerSensor> acc_;
	std::shared_ptr< chrono::sensor::ChGyroscopeSensor> gyro_;

	int lidar_last_count_, imu_last_count_;

};


