% 设置文件夹路径
originalFolder = 'D:\lab\项目\振动对slam影响\data\raw\differentG_1\torque-A0F10V1-2024.10.25\Lidar'; % 原始点云数据文件夹
noisyFolder = 'D:\lab\项目\振动对slam影响\data\raw\differentG_1_noise\torque-A0F10V1-2024.10.25\Lidar';       % 加噪声点云数据文件夹

% 获取文件夹内所有csv文件
originalFiles = dir(fullfile(originalFolder, '*.csv'));
noisyFiles = dir(fullfile(noisyFolder, '*.csv'));

% 检查两文件夹中帧数是否一致
if length(originalFiles) ~= length(noisyFiles)
    error('原始数据和加噪声数据的帧数不一致');
end

% 读取每个frame并进行对比绘制
for i = 1000:length(originalFiles)
    % 读取当前frame的原始点云和加噪声点云
    originalData = readmatrix(fullfile(originalFolder, originalFiles(i).name));
    noisyData = readmatrix(fullfile(noisyFolder, noisyFiles(i).name));
    
    % 提取 xyz 列 (前三列)
    originalXYZ = originalData(:, 1:3);
    noisyXYZ = noisyData(:, 1:3);
    
    % 创建 pointCloud 对象
    pcOriginal = pointCloud(originalXYZ);
    pcNoisy = pointCloud(noisyXYZ);
    
    % 在同一坐标系中显示原始与加噪声点云
    figure;
    pcshowpair(pcOriginal, pcNoisy);
    title(['Frame ', num2str(i), ' Comparison of Original and Noisy Point Clouds']);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    legend('Original', 'Noisy');
    pause(0.5); % 暂停以便观察每帧的图像
end
