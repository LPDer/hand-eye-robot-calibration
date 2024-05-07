% generate data for calibration using POE model

% add path
currentFolder = pwd;
parentFolder = fileparts(currentFolder);
foldersToAdd = genpath(parentFolder);
addpath(foldersToAdd);

% nominal parameter
ksi_1 = [0 0 0 0 0 1]'; ksi_2 = [89.2 0 0 0 -1 0]';
ksi_3 = [514.2 0 0 0 -1 0]'; ksi_4 = [906.2 0 0 0 -1 0]';
ksi_5 = [-109.3 0 0 0 0 1]'; ksi_6 = [1000.95 0 0 0 -1 0]';
ksi_n = [ksi_1,ksi_2,ksi_3,ksi_4,ksi_5,ksi_6];
T_BS_0_n = [eye(3),[0 -109.3 906.2]';0 0 0 1];

p_B_1 = [300 500 0]';
p_B_2 = [320 500 0]';
p_B_3 = [300 520 0]';

% actual parameter
d_k = [-0.0047  0.0018  0.0001 -0.0003...
        0.0023 -0.0034  0.0002  0.0001...
         0.007 -0.0205  0.0003 -0.0001...
       -0.0158  0.0051  0.0002  0.0001...
       -0.0133  0.0113  0.0003  0.0002...
         0.003 -0.0026 -0.0002  0.0002...
       -0.0117  -0.024 -0.0033  0.0002 -0.0001 0.0003]';

g_vx = [1 0 0 0 0 0]'; g_vy = [0 1 0 0 0 0]';
g_vz = [0 0 1 0 0 0]'; g_wx = [0 0 0 1 0 0]';
g_wy = [0 0 0 0 1 0]'; g_wz = [0 0 0 0 0 1]';
G1 = [g_vx,g_vy,g_wx,g_wy]; G2 = [g_vx,g_vz,g_wx,g_wz]; G3 = G2;
G4 = [g_vx,g_vz,g_wx,g_wz]; G5 = G1;
G6 = [g_vx,g_vz,g_wx,g_wz]; G_BS = eye(6);
G = blkdiag(G1,G2,G3,G4,G5,G6,G_BS);

num_links = size(ksi_n,2);

yita = G*d_k;
ksi_a = zeros(6,6);
for i = 1:6
    ksi_a(:,i) = Adjoint(expm(hat(yita(6*i-5:6*i))))*ksi_n(:,i);
end
T_BS_0_a = expm(hat(yita(37:42)))*T_BS_0_n;

% joint angle
num_q = 100;
q_1 = zeros(6,num_q);
q_1(1,:) = deg2rad((rand(1,num_q)*320-160));
q_1(2,:) = deg2rad((rand(1,num_q)*250-180));
q_1(3,:) = deg2rad((rand(1,num_q)*265-45));
q_1(4,:) = deg2rad((rand(1,num_q)*300-150));
q_1(5,:) = deg2rad((rand(1,num_q)*200-100));
q_1(6,:) = deg2rad((rand(1,num_q)*532-266));
q_2 = zeros(6,num_q);
q_2(1,:) = deg2rad((rand(1,num_q)*320-160));
q_2(2,:) = deg2rad((rand(1,num_q)*250-180));
q_2(3,:) = deg2rad((rand(1,num_q)*265-45));
q_2(4,:) = deg2rad((rand(1,num_q)*300-150));
q_2(5,:) = deg2rad((rand(1,num_q)*200-100));
q_2(6,:) = deg2rad((rand(1,num_q)*532-266));
q_3 = zeros(6,num_q);
q_3(1,:) = deg2rad((rand(1,num_q)*320-160));
q_3(2,:) = deg2rad((rand(1,num_q)*250-180));
q_3(3,:) = deg2rad((rand(1,num_q)*265-45));
q_3(4,:) = deg2rad((rand(1,num_q)*300-150));
q_3(5,:) = deg2rad((rand(1,num_q)*200-100));
q_3(6,:) = deg2rad((rand(1,num_q)*532-266));

% calculate position collected by sensor
noise = 0;
p_S_1 = zeros(3,num_q);
p_S_2 = zeros(3,num_q);
p_S_3 = zeros(3,num_q);
for i = 1:num_q
    T1 = fkine_POE(ksi_a,q_1(:,i),T_BS_0_a);
    temp = T1(:,:,num_links+1)\[p_B_1;1];
    p_S_1(:,i) = temp(1:3)+(randn(3,1)*noise);
    T2 = fkine_POE(ksi_a,q_2(:,i),T_BS_0_a);
    temp = T2(:,:,num_links+1)\[p_B_2;1];
    p_S_2(:,i) = temp(1:3)+(randn(3,1)*noise);
    T3 = fkine_POE(ksi_a,q_3(:,i),T_BS_0_a);
    temp = T3(:,:,num_links+1)\[p_B_3;1];
    p_S_3(:,i) = temp(1:3)+(randn(3,1)*noise);
end

% save data
save("./data/sim_data_POE_noise_0.mat");