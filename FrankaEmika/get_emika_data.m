% 预分配结果 cell
num = 40;
pos = zeros(8,num);
tau = zeros(8,num);
rootDir= "/home/sia/Franka_Emika_data/exp1_calibration/";
% 生成 01~40 的文件夹名字
for idx = 1:num
    folderName = sprintf('xinhui%02d.bag_csv', idx);
    folderPath = fullfile(rootDir, folderName);

    if ~isfolder(folderPath)
        fprintf('⚠️ 文件夹不存在：%s\n', folderName);
        continue;
    end

    % 寻找 joint_states 文件
    jointFile = fullfile(folderPath, '_joint_states.txt');
    
    if ~isfile(jointFile)
        fprintf('⚠️ 未找到文件：%s\n', jointFile);
        continue;
    end

    % 读取 CSV 数据（包括 header）
    opts = detectImportOptions(jointFile, 'Delimiter', ',');
    T = readtable(jointFile, opts);
    q1 = mean(T.field_position0);
    q2 = mean(T.field_position1);
    q3 = mean(T.field_position2);
    q4 = mean(T.field_position3);
    q5 = mean(T.field_position4);
    q6 = mean(T.field_position5);
    q7 = mean(T.field_position6);

    dq1 = mean(T.field_velocity0);
    dq2 = mean(T.field_velocity1);
    dq3 = mean(T.field_velocity2);
    dq4 = mean(T.field_velocity3);
    dq5 = mean(T.field_velocity4);
    dq6 = mean(T.field_velocity5);
    dq7 = mean(T.field_velocity6);

    t1 = mean(T.field_effort0);
    t2 = mean(T.field_effort1);
    t3 = mean(T.field_effort2);
    t4 = mean(T.field_effort3);
    t5 = mean(T.field_effort4);
    t6 = mean(T.field_effort5);
    t7 = mean(T.field_effort6);

    pos(1:7,idx) = [q1,q2,q3,q4,q5,q6,q7]';
    tau(1:7,idx) = [t1,t2,t3,t4,t5,t6,t7]';  
end