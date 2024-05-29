#pragma once
//
//auto extrinsics_lidar = chrono::ChFrame<double>({ 0, 0, 1 }, Q_from_AngAxis(0, { 1, 0, 0 }));
//auto extrinsics_imu = chrono::ChFrame<double>({ 0, 0, 1 }, Q_from_AngAxis(0, { 1, 0, 0 }));


#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"
#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"

#include <iostream>
#include <vector>


#define LIDAR_CONFIG_JSON "D:/qsq/project/src/Lidar.json"


class MySensors {

public:
	MySensors(chrono::vehicle::sedan::Sedan& platform, std::string lidar_output_folder = "empty") : manager_(platform.GetSystem()) {
	//MySensors(chrono::vehicle::gator::Gator& platform, std::string lidar_output_folder="empty") : manager_(platform.GetSystem()) {
	
		using namespace chrono;
		using namespace chrono::sensor;

		//// Create Lidar. From JSON
		auto lidar_ext = chrono::ChFrame<double>({ 0, 0, 3 }, Q_from_AngAxis(0, { 1, 0, 0 }));
		lidar_ = chrono::sensor::Sensor::CreateFromJSON(LIDAR_CONFIG_JSON, platform.GetChassisBody(), lidar_ext);
		if (lidar_output_folder == "empty") {
			std::cout << "[Error]. Lidar output folder not provided. " << std::endl;
			while (1) { ; }
		}
		else {
			// save lidar config;
			std::ofstream out(lidar_output_folder + "lidar_config.txt");
			if (!out) {
				std::cout << "[Error]. Unabled to save lidar config" << std::endl;
			}
			out << "Lidar frequency: " << lidar_->GetUpdateRate() << std::endl;
			out << "Lidar extrinsics: " << lidar_ext << std::endl;
			out.close();
		}
		lidar_->PushFilter(chrono_types::make_shared<ChFilterSavePtCloud>(lidar_output_folder));
		manager_.AddSensor(lidar_);
		lidar_last_count_ = 0;
		std::cout << "--> Lidar inited" << std::endl;
		
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

		imu_last_count_ = 0;
		std::cout << "--> IMU inited" << std::endl;

		//// Create a camera sensor for 3rd-person-view visualization
		//auto cam_body = chrono_types::make_shared<ChBody>();
		//cam_body->SetBodyFixed(true);
		//cam_body->SetPos(ChVector<>(0, 0, 100));			// 500m height in map center;
		//platform.GetSystem()->AddBody(cam_body);
		//auto camera = chrono_types::make_shared<ChCameraSensor>(
		//	cam_body,                          // �󶨵���ChBody����
		//	20.0f,                         // �����ʣ�Hz��
		//	chrono::ChFrame<double>({ 0, 0, -5 }, Q_from_AngAxis(0, { 1, 0, 0 })), // �����ChBody��λ�úͷ���
		//	1280,                          // ͼ����
		//	720,                           // ͼ��߶�
		//	CH_C_PI                    // ����ͷ�ӳ��Ƕ�
		//);
		//camera->SetName("3rd-view");
		//camera->SetLag(0);
		//double exposure_time = 0.02f;
		//camera->SetCollectionWindow(exposure_time);
		//camera->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 480));
		//manager_.AddSensor(camera);
		//manager_.scene->AddPointLight({ 100, 100, 100 }, { 2, 2, 2 }, 5000);
		//manager_.scene->SetAmbientLight({ 0, 0, 0 });
		//manager_.scene->SetFogScatteringFromDistance(200.0);
		//// Set environment map
		//Background b;
		//b.mode = BackgroundMode::GRADIENT;
		//b.color_horizon = { 0.6f, light_intensity, 0.8f };
		//b.color_zenith = { 0.4f, 0.5f, 0.6f };
		//manager_.scene->SetBackground(b);
		//std::cout << "--> 3rd view camera inited" << std::endl;
	}




	//auto cam2 = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
	//	my_hmmwv.GetChassisBody(),                                            // body camera is attached to
	//	cam_update_rate,                                                      // update rate in Hz
	//	chrono::ChFrame<double>({ 1, 0, .875 }, Q_from_AngAxis(0, { 1, 0, 0 })),  // offset pose
	//	image_width,                                                          // image width
	//	image_height,                                                         // image height
	//	cam_fov,
	//	super_samples);  // fov, lag, exposure
	//cam2->SetName("Camera Sensor");

	void printIMU(void);
	bool getIMU(std::vector<double>& accs, std::vector<double>& gyros);

public:
	chrono::sensor::ChSensorManager manager_;

private:
	std::shared_ptr<chrono::sensor::ChSensor> lidar_;
	std::shared_ptr< chrono::sensor::ChAccelerometerSensor> acc_;
	std::shared_ptr< chrono::sensor::ChGyroscopeSensor> gyro_;

	size_t lidar_last_count_, imu_last_count_;

};


