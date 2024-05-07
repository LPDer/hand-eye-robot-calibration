clear;
clc;

% add path
currentFolder = pwd;
parentFolder = fileparts(currentFolder);
foldersToAdd = genpath(parentFolder);
addpath(foldersToAdd);

% load data
load("./data/sim_data_POE_noise_0.mat");

% divide data into calibration set and verification set
num_q = size(q_1,2);
indice = randperm(num_q);
calibration_index = indice(1:0.8*num_q);
verification_index = setdiff(indice,calibration_index);
q_1_cali = q_1(:,calibration_index);
q_2_cali = q_2(:,calibration_index);
q_3_cali = q_3(:,calibration_index);
q_1_ver = q_1(:,verification_index);
q_2_ver = q_2(:,verification_index);
q_3_ver = q_3(:,verification_index);
p_S_1_cali = p_S_1(:,calibration_index);
p_S_2_cali = p_S_2(:,calibration_index);
p_S_3_cali = p_S_3(:,calibration_index);
p_S_1_ver = p_S_1(:,verification_index);
p_S_2_ver = p_S_2(:,verification_index);
p_S_3_ver = p_S_3(:,verification_index);

% start iteration, define parameters to be updated
iter_step = 20;
ksi_cur = ksi_n; T_BS_0_cur = T_BS_0_n;
for iter = 1:iter_step
    J_set = []; b_set = [];
    for i = 1:0.8*num_q
        T1_cur = fkine_POE(ksi_cur,q_1_cali(:,i),T_BS_0_cur);
        J1 = cal_J_POE(T1_cur,G,p_B_1);
        temp = T1_cur(:,:,num_links+1)*[p_S_1_cali(:,i);1]-[p_B_1;1];
        b1 = temp(1:3);
        T2_cur = fkine_POE(ksi_cur,q_2_cali(:,i),T_BS_0_cur);
        J2 = cal_J_POE(T2_cur,G,p_B_2);
        temp = T2_cur(:,:,num_links+1)*[p_S_2_cali(:,i);1]-[p_B_2;1];
        b2 = temp(1:3);
        T3_cur = fkine_POE(ksi_cur,q_3_cali(:,i),T_BS_0_cur);
        J3 = cal_J_POE(T3_cur,G,p_B_3);
        temp = T3_cur(:,:,num_links+1)*[p_S_3_cali(:,i);1]-[p_B_3;1];
        b3 = temp(1:3);
        J_set = [J_set;J1;J2;J3];
        b_set = [b_set;b1;b2;b3];
    end
    x = inv(J_set'*J_set)*J_set'*b_set;
    % update parameters
    yita_c = G*x;
    for i = 1:num_links
        ksi_cur(:,i) = Adjoint(expm(hat(yita_c(6*i-5:6*i))))*ksi_cur(:,i);
    end
    T_BS_0_cur = expm(hat(yita_c(6*num_links+1:6*num_links+6)))*T_BS_0_cur;
end

% calculate residual error of verification set
residual_err_1 = zeros(0.2*num_q,1);
residual_err_2 = zeros(0.2*num_q,1);
residual_err_3 = zeros(0.2*num_q,1);
for i = 1:0.2*num_q
    T1_cur = fkine_POE(ksi_cur,q_1_ver(:,i),T_BS_0_cur);
    residual_err_1(i) = norm(T1_cur(:,:,num_links+1)*[p_S_1_ver(:,i);1]...
        -[p_B_1;1]);
    T2_cur = fkine_POE(ksi_cur,q_2_ver(:,i),T_BS_0_cur);
    residual_err_2(i) = norm(T2_cur(:,:,num_links+1)*[p_S_2_ver(:,i);1]...
        -[p_B_2;1]);
    T3_cur = fkine_POE(ksi_cur,q_3_ver(:,i),T_BS_0_cur);
    residual_err_3(i) = norm(T3_cur(:,:,num_links+1)*[p_S_3_ver(:,i);1]...
        -[p_B_3;1]);
end
