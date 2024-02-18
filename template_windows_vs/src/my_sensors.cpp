
#include "my_sensors.h"

using std::vector;
using std::cout;
using std::endl;

using namespace chrono;
using namespace chrono::sensor;

void MySensors::printIMU(void) {
	//cout <<"IMU: " << acc_.
}

bool MySensors::getIMU(vector<double>& accs, vector<double>& gyros) {
	auto bufferAcc = acc_->GetMostRecentBuffer<UserAccelBufferPtr>();
	auto bufferGyro = gyro_->GetMostRecentBuffer<UserGyroBufferPtr>();

    if (bufferAcc->Buffer && bufferGyro->Buffer) {
        AccelData acc_data = bufferAcc->Buffer[0];
        GyroData gyro_data = bufferGyro->Buffer[0];

        accs.resize(3);
        gyros.resize(3);
        accs[0] = acc_data.X;
        accs[1] = acc_data.Y;
        accs[2] = acc_data.Z;
        gyros[0] = gyro_data.Roll;
        gyros[1] = gyro_data.Pitch;
        gyros[2] = gyro_data.Yaw;

        return true;
        //std::cout << "IMU acc : " << acc_data.X << ", " << acc_data.Y << ", " << acc_data.Z << std::endl;
        //std::cout << "IMU gryo: " << gyro_data.Roll << ", " << gyro_data.Pitch << ", " << gyro_data.Yaw << std::endl;
    }
    else
        return false;
}

