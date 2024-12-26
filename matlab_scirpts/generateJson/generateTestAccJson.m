clc;clear;close all;

function generateJson(outputPath, params)
    % 创建结构体
    data.physicalSystem.gravity = params.gravity;
    data.physicalSystem.base_x_initial_velocity = params.base_x_initial_velocity;
    data.physicalSystem.base_y_initial_velocity = params.base_y_initial_velocity;
    data.physicalSystem.base_z_initial_velocity = params.base_z_initial_velocity;
    data.physicalSystem.IMU_x_initial_velocity = params.IMU_x_initial_velocity;
    data.physicalSystem.IMU_y_initial_velocity = params.IMU_y_initial_velocity;
    data.physicalSystem.IMU_z_initial_velocity = params.IMU_z_initial_velocity;
    data.physicalSystem.spring_down_rest_L = params.spring_down_rest_L;
    data.physicalSystem.delta_chassis_height = params.delta_chassis_height;
    data.physicalSystem.spring_down_K = params.spring_down_K;
    data.physicalSystem.spring_around_K = params.spring_around_K;
    data.physicalSystem.spring_down_C = params.spring_down_C;
    data.physicalSystem.spring_around_C = params.spring_around_C;
    data.physicalSystem.chassis_M = params.chassis_M;
    data.physicalSystem.base_M = params.base_M;
    data.physicalSystem.corner_M = params.corner_M;
    data.physicalSystem.chtime_end = params.chtime_end;
    data.physicalSystem.IMU_force_phase = params.IMU_force_phase;
    data.physicalSystem.IMU_force_freq = params.IMU_force_freq;
    data.physicalSystem.IMU_force_amp = params.IMU_force_amp;
    data.physicalSystem.IMU_torque_phase = params.IMU_torque_phase;
    data.physicalSystem.IMU_torque_freq = params.IMU_torque_freq;
    data.physicalSystem.IMU_torque_amp = params.IMU_torque_amp;
    data.physicalSystem.t_gap = params.t_gap;
    data.IMUConfig.imu_update_rate = params.imu_update_rate;

    % 将结构体转换为 JSON 字符串
    jsonString = jsonencode(data);

    
    % 格式化 JSON 字符串
    jsonString = formatJson(jsonString); 

    % 动态生成文件名
    fileName = sprintf('force-A%.1fF%dfV2-2024.12.23.json', params.IMU_force_amp, params.IMU_force_freq);

    % 生成完整的文件路径
    fullFileName = fullfile(outputPath, fileName);

    % 将 JSON 字符串写入文件
    fid = fopen(fullFileName, 'w');
    if fid == -1
        error('无法创建文件：%s', fullFileName);
    end
    fwrite(fid, jsonString, 'char');
    fclose(fid);
end

function formattedJson = formatJson(jsonString)
    % 格式化 JSON 字符串
    indent = '    ';
    level = 0;
    formattedJson = '';
    inString = false;

    for i = 1:length(jsonString)
        char = jsonString(i);
        if char == '"' && (i == 1 || jsonString(i-1) ~= '\')
            inString = ~inString;
        end
        if ~inString
            if char == '{' || char == '['
                level = level + 1;
                formattedJson = [formattedJson, char, newline, repmat(indent, 1, level)];
            elseif char == '}' || char == ']'
                level = level - 1;
                formattedJson = [formattedJson, newline, repmat(indent, 1, level), char];
            elseif char == ','
                formattedJson = [formattedJson, char, newline, repmat(indent, 1, level)];
            else
                formattedJson = [formattedJson, char];
            end
        else
            formattedJson = [formattedJson, char];
        end
    end
end

outputPath = 'D:\lab\项目\振动对slam影响\matlab\json\json_output\accY_2024.12.23\accY_differentA_F100'; 

% 检查目录是否存在，如果不存在则创建
if ~exist(outputPath, 'dir')
    mkdir(outputPath);
end


params.gravity = 0;
params.base_x_initial_velocity = 2;
params.base_y_initial_velocity = 0;
params.base_z_initial_velocity = 0;
params.IMU_x_initial_velocity = 2;
params.IMU_y_initial_velocity = 0;
params.IMU_z_initial_velocity = 0;
params.spring_down_rest_L = 2.0;
params.delta_chassis_height = 0;
params.spring_down_K = 1;
params.spring_around_K = 1;
params.spring_down_C = 10;
params.spring_around_C = 10;
params.chassis_M = 1;
params.base_M = 10;
params.corner_M = 10;
params.chtime_end = 5;

params.IMU_force_phase = 0;

params.IMU_torque_amp = 0;    
params.IMU_torque_freq = 0;
params.IMU_torque_phase = 0;

params.t_gap = 5;
params.imu_update_rate = 200.0;

params.IMU_force_freq = 100;

for a = 0:0.2:22   
    params.IMU_force_amp = a;    
    generateJson(outputPath, params);
end

