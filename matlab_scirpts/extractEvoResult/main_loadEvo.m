
clc;clear;close all;

% 文件名格式：
% res_rpe-rot-10m-pointlio_1-A1F100V2-2024.12.23
% res_rpe-{A}-{B}-{method}_{D}-{E}-{F}.csv

method_list = {'loam', 'liomapping', 'liosam', 'fastlio', 'pointlio', 'CT-ICP'};

for idx_method = 1:length(method_list)
    
    method = method_list{idx_method};
    A = 'rot';      % 输入A值
    B = '10m';      % 输入B值
    F = '2024.12.23';
    type_list = ["force", "torque"];
    amp_list_force = [1, 20];       % 设定force的A的数值
    amp_list_torque = [5, 100];     % 设定torque的A的数值
    fre_list = [1, 10, 100];       % 示例的fre_list
    folder = './'; % 文件夹路径
    
    % 初始化存储矩阵。D从1~6表示力和力矩。
    matrices = cell(6, 1); % 每个method_A_D对应一个矩阵，共6个
    for D = 1:6
        matrices{D} = NaN(2, 3);  % 每个矩阵是amp_list行，fre_list列
    end
    
    
    % 获取所有符合条件的csv文件
    method_folder = fullfile(folder, [method, '_evo']);
    csv_files = dir(fullfile(method_folder, strcat('res_rpe-', A, '-', B, '-', method, '*.csv')));
    
    
    %% 遍历两种加速度
    for idx_type = 1:length(type_list)
        
        type = type_list(idx_type);
        if strcmp(type, "force")==0
            amp_list = amp_list_force;
        elseif strcmp(type, "torque")==0
            amp_list = amp_list_torque;
        end
        
        % 遍历每个evo的结果文件，读取需要的数据
        for i = 1:length(csv_files)
            file_name = csv_files(i).name;
            % 提取 D, E 和 F
            tokens = regexp(file_name, ['res_rpe-', A, '-', B, '-', method, '_(\d+)-A(\d+)F(\d+)V2-', F, '.csv'], 'tokens');
            if ~isempty(tokens)
                D_value = str2double(tokens{1}{1});
                amp = str2double(tokens{1}{2});
                fre = str2double(tokens{1}{3});
        
                % 查找amp和fre在列表中的索引
                [~, amp_idx] = ismember(amp, amp_list);
                [~, fre_idx] = ismember(fre, fre_list);
        
                % disp([D_value, amp, fre]);
                
                if amp_idx > 0 && fre_idx > 0 && D_value >= 1 && D_value <= 6
                    % 读取CSV文件
                    file_path = fullfile(method_folder, file_name);
                    data = readmatrix(file_path);
                    
                    % 保存到相应的矩阵
                    rmse = data(1, 6);              % 读取rmse值
                    matrices{D_value}(amp_idx, fre_idx) = rmse;
                end
            end
        end
    end
    
    % 输出结果
    fprintf("--> Method: %s \n", string(method));
    for D = 1:6
        fprintf("Matrix for D = %d:\n", D);
        disp(matrices{D});
    end
    
    %% output 
    acc_x = matrices{1};
    acc_y = matrices{2};
    acc_z = matrices{3};
    gyro_x = matrices{4};
    gyro_y = matrices{5};
    gyro_z = matrices{6};
    
    
    method = string(method);
    rpetype = string(A);
    
    output_filename = method + "-" +  A + ".mat";
    save(output_filename, 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z');

end
