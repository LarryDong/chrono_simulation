name_acc = readtable('D:\lab\项目\振动对slam影响\matlab\json\name_excel\Acc_vibration_12.24.xlsx');
name_gyro = readtable('D:\lab\项目\振动对slam影响\matlab\json\name_excel\Gyro_vibration_12.24.xlsx');
path = 'E:\36组测试数据比较20241225\liomapping_output\liomapping_evo';
%% 1   
pattern_rot_2m_1 = 'res_rpe-rot-2m-*_1-A%dF%dV2*.csv';
png_name_rot_2m_1 = '1_rot_2m';
generateErrorHeatMap(name_acc, path, pattern_rot_2m_1, png_name_rot_2m_1);

pattern_rot_5m_1 = 'res_rpe-rot-5m-*_1-A%dF%dV2*.csv';
png_name_rot_5m_1 = '1_rot_5m';
generateErrorHeatMap(name_acc, path, pattern_rot_5m_1, png_name_rot_5m_1);

pattern_rot_10m_1 = 'res_rpe-rot-10m-*_1-A%dF%dV2*.csv';
png_name_rot_10m_1 = '1_rot_10m';
generateErrorHeatMap(name_acc, path, pattern_rot_10m_1, png_name_rot_10m_1);

pattern_trans_2m_1 = 'res_rpe-trans-2m-*_1-A%dF%dV2*.csv';
png_name_trans_2m_1 = '1_trans_2m';
generateErrorHeatMap(name_acc, path, pattern_trans_2m_1, png_name_trans_2m_1);

pattern_trans_5m_1 = 'res_rpe-trans-5m-*_1-A%dF%dV2*.csv';
png_name_trans_5m_1 = '1_trans_5m';
generateErrorHeatMap(name_acc, path, pattern_trans_5m_1, png_name_trans_5m_1);

pattern_trans_10m_1 = 'res_rpe-trans-10m-*_1-A%dF%dV2*.csv';
png_name_trans_10m_1 = '1_trans_10m';
generateErrorHeatMap(name_acc, path, pattern_trans_10m_1, png_name_trans_10m_1);
%% 2
pattern_rot_2m_2 = 'res_rpe-rot-2m-*_2-A%dF%dV2*.csv';
png_name_rot_2m_2 = '2_rot_2m';
generateErrorHeatMap(name_acc, path, pattern_rot_2m_2, png_name_rot_2m_2);

pattern_rot_5m_2 = 'res_rpe-rot-5m-*_2-A%dF%dV2*.csv';
png_name_rot_5m_2 = '2_rot_5m';
generateErrorHeatMap(name_acc, path, pattern_rot_5m_2, png_name_rot_5m_2);

pattern_rot_10m_2 = 'res_rpe-rot-10m-*_2-A%dF%dV2*.csv';
png_name_rot_10m_2 = '2_rot_10m';
generateErrorHeatMap(name_acc, path, pattern_rot_10m_2, png_name_rot_10m_2);

pattern_trans_2m_2 = 'res_rpe-trans-2m-*_2-A%dF%dV2*.csv';
png_name_trans_2m_2 = '2_trans_2m';
generateErrorHeatMap(name_acc, path, pattern_trans_2m_2, png_name_trans_2m_2);

pattern_trans_5m_2 = 'res_rpe-trans-5m-*_2-A%dF%dV2*.csv';
png_name_trans_5m_2 = '2_trans_5m';
generateErrorHeatMap(name_acc, path, pattern_trans_5m_2, png_name_trans_5m_2);

pattern_trans_10m_2 = 'res_rpe-trans-10m-*_2-A%dF%dV2*.csv';
png_name_trans_10m_2 = '2_trans_10m';
generateErrorHeatMap(name_acc, path, pattern_trans_10m_2, png_name_trans_10m_2);

%% 3
pattern_rot_2m_3 = 'res_rpe-rot-2m-*_3-A%dF%dV2*.csv';
png_name_rot_2m_3 = '3_rot_2m';
generateErrorHeatMap(name_acc, path, pattern_rot_2m_3, png_name_rot_2m_3);

pattern_rot_5m_3 = 'res_rpe-rot-5m-*_3-A%dF%dV2*.csv';
png_name_rot_5m_3 = '3_rot_5m';
generateErrorHeatMap(name_acc, path, pattern_rot_5m_3, png_name_rot_5m_3);

pattern_rot_10m_3 = 'res_rpe-rot-10m-*_3-A%dF%dV2*.csv';
png_name_rot_10m_3 = '3_rot_10m';
generateErrorHeatMap(name_acc, path, pattern_rot_10m_3, png_name_rot_10m_3);

pattern_trans_2m_3 = 'res_rpe-trans-2m-*_3-A%dF%dV2*.csv';
png_name_trans_2m_3 = '3_trans_2m';
generateErrorHeatMap(name_acc, path, pattern_trans_2m_3, png_name_trans_2m_3);

pattern_trans_5m_3 = 'res_rpe-trans-5m-*_3-A%dF%dV2*.csv';
png_name_trans_5m_3 = '3_trans_5m';
generateErrorHeatMap(name_acc, path, pattern_trans_5m_3, png_name_trans_5m_3);

pattern_trans_10m_3 = 'res_rpe-trans-10m-*_3-A%dF%dV2*.csv';
png_name_trans_10m_3 = '3_trans_10m';
generateErrorHeatMap(name_acc, path, pattern_trans_10m_3, png_name_trans_10m_3);

%% 4
pattern_rot_2m_4 = 'res_rpe-rot-2m-*_4-A%dF%dV2*.csv';
png_name_rot_2m_4 = '4_rot_2m';
generateErrorHeatMap(name_gyro, path, pattern_rot_2m_4, png_name_rot_2m_4);

pattern_rot_5m_4 = 'res_rpe-rot-5m-*_4-A%dF%dV2*.csv';
png_name_rot_5m_4 = '4_rot_5m';
generateErrorHeatMap(name_gyro, path, pattern_rot_5m_4, png_name_rot_5m_4);

pattern_rot_10m_4 = 'res_rpe-rot-10m-*_4-A%dF%dV2*.csv';
png_name_rot_10m_4 = '4_rot_10m';
generateErrorHeatMap(name_gyro, path, pattern_rot_10m_4, png_name_rot_10m_4);

pattern_trans_2m_4 = 'res_rpe-trans-2m-*_4-A%dF%dV2*.csv';
png_name_trans_2m_4 = '4_trans_2m';
generateErrorHeatMap(name_gyro, path, pattern_trans_2m_4, png_name_trans_2m_4);

pattern_trans_5m_4 = 'res_rpe-trans-5m-*_4-A%dF%dV2*.csv';
png_name_trans_5m_4 = '4_trans_5m';
generateErrorHeatMap(name_gyro, path, pattern_trans_5m_4, png_name_trans_5m_4);

pattern_trans_10m_4 = 'res_rpe-trans-10m-*_4-A%dF%dV2*.csv';
png_name_trans_10m_4 = '4_trans_10m';
generateErrorHeatMap(name_gyro, path, pattern_trans_10m_4, png_name_trans_10m_4);

%% 5
pattern_rot_2m_5 = 'res_rpe-rot-2m-*_5-A%dF%dV2*.csv';
png_name_rot_2m_5 = '5_rot_2m';
generateErrorHeatMap(name_gyro, path, pattern_rot_2m_5, png_name_rot_2m_5);

pattern_rot_5m_5 = 'res_rpe-rot-5m-*_5-A%dF%dV2*.csv';
png_name_rot_5m_5 = '5_rot_5m';
generateErrorHeatMap(name_gyro, path, pattern_rot_5m_5, png_name_rot_5m_5);

pattern_rot_10m_5 = 'res_rpe-rot-10m-*_5-A%dF%dV2*.csv';
png_name_rot_10m_5 = '5_rot_10m';
generateErrorHeatMap(name_gyro, path, pattern_rot_10m_5, png_name_rot_10m_5);

pattern_trans_2m_5 = 'res_rpe-trans-2m-*_5-A%dF%dV2*.csv';
png_name_trans_2m_5 = '5_trans_2m';
generateErrorHeatMap(name_gyro, path, pattern_trans_2m_5, png_name_trans_2m_5);

pattern_trans_5m_5 = 'res_rpe-trans-5m-*_5-A%dF%dV2*.csv';
png_name_trans_5m_5 = '5_trans_5m';
generateErrorHeatMap(name_gyro, path, pattern_trans_5m_5, png_name_trans_5m_5);

pattern_trans_10m_5 = 'res_rpe-trans-10m-*_5-A%dF%dV2*.csv';
png_name_trans_10m_5 = '5_trans_10m';
generateErrorHeatMap(name_gyro, path, pattern_trans_10m_5, png_name_trans_10m_5);

%% 6
pattern_rot_2m_6 = 'res_rpe-rot-2m-*_6-A%dF%dV2*.csv';
png_name_rot_2m_6 = '6_rot_2m';
generateErrorHeatMap(name_gyro, path, pattern_rot_2m_6, png_name_rot_2m_6);

pattern_rot_5m_6 = 'res_rpe-rot-5m-*_6-A%dF%dV2*.csv';
png_name_rot_5m_6 = '6_rot_5m';
generateErrorHeatMap(name_gyro, path, pattern_rot_5m_6, png_name_rot_5m_6);

pattern_rot_10m_6 = 'res_rpe-rot-10m-*_6-A%dF%dV2*.csv';
png_name_rot_10m_6 = '6_rot_10m';
generateErrorHeatMap(name_gyro, path, pattern_rot_10m_6, png_name_rot_10m_6);

pattern_trans_2m_6 = 'res_rpe-trans-2m-*_6-A%dF%dV2*.csv';
png_name_trans_2m_6 = '6_trans_2m';
generateErrorHeatMap(name_gyro, path, pattern_trans_2m_6, png_name_trans_2m_6);

pattern_trans_5m_6 = 'res_rpe-trans-5m-*_6-A%dF%dV2*.csv';
png_name_trans_5m_6 = '6_trans_5m';
generateErrorHeatMap(name_gyro, path, pattern_trans_5m_6, png_name_trans_5m_6);

pattern_trans_10m_6 = 'res_rpe-trans-10m-*_6-A%dF%dV2*.csv';
png_name_trans_10m_6 = '6_trans_10m';
generateErrorHeatMap(name_gyro, path, pattern_trans_10m_6, png_name_trans_10m_6);


function generateErrorHeatMap(name, path, pattern, png_name)
    % 读取频率和振幅列表
    freq_list = name{1, 2:end};
    amp_list = name{2:end, 1};
    f_l = length(freq_list);
    a_l = length(amp_list);
    
    % 初始化误差矩阵
    error_matrix = zeros(a_l, f_l);

    % 遍历 freq_list 和 amp_list 填充误差矩阵
    for f = 1:f_l
        for a = 1:a_l
            freq = freq_list(f);
            amp = amp_list(a);
            error = loadError(path, freq, amp, pattern, 'rmse'); % 方法可以是'mean','mse','std'
            error_matrix(a, f) = error;
        end
    end

    % 绘制误差热图
    drawErrorHeatMap(error_matrix, freq_list, amp_list, png_name,path);

    % 保存误差矩阵和频率、振幅列表为 MAT 文件
    mat_full_path = fullfile(path, [png_name, '.mat']);
    %save(mat_full_path, 'error_matrix', 'freq_list', 'amp_list', '-v7.3'); % 保存 .mat 文件

    % % 扩展误差矩阵：添加 freq_list 和 amp_list
    % error_matrix_with_axes = [[0, freq_list]; [amp_list, error_matrix]];
    
    % % 绘制误差热图并保存为 PNG 文件
    % drawErrorHeatMap(error_matrix, freq_list, amp_list, path, png_name);
    % 
    % % 保存扩展后的误差矩阵为 MAT 文件
    % mat_full_path = fullfile(path, [png_name, '.mat']);
    % save(mat_full_path, 'error_matrix_with_axes', '-v7.3'); % 保存 .mat 文件
end

% 加载误差函数
function error = loadError(path, f, a, pattern, method)
    % 替换模式中的占位符
    formattedPattern = sprintf(pattern, a, f);
    files = dir(fullfile(path, formattedPattern));
    
    if isempty(files)
        warning('未找到匹配的文件：A%dF%d', a, f);
        error = NaN;
        return;
    elseif length(files) > 1
        warning('找到多个匹配的文件：A%dF%d，使用第一个匹配的文件。', a, f);
    end
    
    % 使用找到的第一个文件
    filename = files(1).name;
    fullPath = fullfile(path, filename);
    data = readmatrix(fullPath);
    switch method
        case 'max'
            error = data(1,2);
        case 'mean'
            error = data(1,3);
        case 'median'
            error = data(1,4);
        case 'min'
            error = data(1,5);
        case 'rmse'
            error = data(1,6);
        case 'sse'
            error = data(1,7);
        case 'std'
            error = data(1,8);
        otherwise
            error = NaN;
    end
end

% 绘制热图函数
function drawErrorHeatMap(error_matrix, freq_list, amp_list, png_name,path)
    % 将 amp_list 和 error_matrix 倒序
    amp_list_reversed = flipud(amp_list);       % 倒序 amp_list
    error_matrix_reversed = flipud(error_matrix); % 倒序 error_matrix 的行

    % 创建热图
    figure;
    h = heatmap(freq_list, amp_list_reversed, error_matrix_reversed);
    
    % 设置热图属性
    h.Title = 'Error Heat Map';
    h.XLabel = 'Frequency';
    h.YLabel = 'Amplitude';

   % 导出热图为 PNG 文件
    % 构造完整的保存路径
    png_full_path = fullfile(path, [png_name, '.png']);
    saveas(gcf, png_full_path);
    close(gcf); % 关闭图窗
end