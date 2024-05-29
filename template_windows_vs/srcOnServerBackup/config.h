#pragma once

#include <iostream>

// 以下内容不修改
const std::string OUTPUT_FOLDER = "D:/DongYan/chrono_simulation/chrono_output/";		// 输出路径，一般不修改。创建lidar/路径
const double VEHICLE_SPEED = 5.0f;		// 车速，暂定5m不修改
const double INIT_WAIT_TIME = 5.0f;	// 初始等待时间，3s，不修改。

// 以下内容同时修改
const std::string SCENE_OBJ = "D:/DongYan/chrono_simulation/scene/outdoor_simple.obj";	// 场景文件，outdoor_simple, city-clean
const std::string TRAJECTORY_TXT = "D:/DongYan/chrono_simulation/trajectory/trajectory_wild.txt";	// trajectory_wild,trajectory_city，轨迹

// 地形每次更改
const std::string TERRAIN_OBJ = "D:/DongYan/chrono_simulation/terrain/step_three_30cm.obj";		// 地形文件


//

