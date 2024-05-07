clear;
clc;

% generate data

% nominal parameter
EMDH_n = [0,      0,   0, -150, 0;
         0,   89.2,  90,    0, 0;
        90,      0,   0,  425, 0;
         0,      0,   0,  392, 0;
       -90,  109.3, -90,    0, 0;
         0,  94.75,  90,    0, 0;
         0,      0, -90,    0, 0;
         0, -94.75,   0,    0, 0];

% actual parameter
d_theta = rad2deg([0.0001 -0.0002 0.0003 -0.0001 -0.0004 0.0002 -0.0001 ...
    -0.0002]');
d_alpha = rad2deg([-0.0001 -0.0004 0.0004 0.0002 -0.0003 -0.0002 ...
    0.0001 0]');
d_a = [-0.0047 0.007 -0.0158 -0.0133 0.003 -0.0117 -0.024 0]';
d_d = [0 -0.0205 0 0 0.0051 -0.0026 0.0122 0.0151]';
d_beta = rad2deg([-0.0003 0 0.0001 0.0002 0 0 0 0]');
EMDH_a = EMDH_n+[d_theta d_d d_alpha d_a d_beta];

p_B_1 = [0 0 100]';
p_B_2 = [200 0 0]';
p_B_3 = [0 300 0]';

N = size(EMDH_n,1);

% joint angle 
num_q = 100;
q_1 = zeros(6,num_q);
q_1(1,:) = (rand(1,num_q)*320-160);
q_1(2,:) = (rand(1,num_q)*250-180);
q_1(3,:) = (rand(1,num_q)*265-45);
q_1(4,:) = (rand(1,num_q)*300-150);
q_1(5,:) = (rand(1,num_q)*200-100);
q_1(6,:) = (rand(1,num_q)*532-266);
q_2 = zeros(6,num_q);
q_2(1,:) = (rand(1,num_q)*320-160);
q_2(2,:) = (rand(1,num_q)*250-180);
q_2(3,:) = (rand(1,num_q)*265-45);
q_2(4,:) = (rand(1,num_q)*300-150);
q_2(5,:) = (rand(1,num_q)*200-100);
q_2(6,:) = (rand(1,num_q)*532-266);
q_3 = zeros(6,num_q);
q_3(1,:) = (rand(1,num_q)*320-160);
q_3(2,:) = (rand(1,num_q)*250-180);
q_3(3,:) = (rand(1,num_q)*265-45);
q_3(4,:) = (rand(1,num_q)*300-150);
q_3(5,:) = (rand(1,num_q)*200-100);
q_3(6,:) = (rand(1,num_q)*532-266);

% calculate position collected by sensor
noise = 0;
p_S_1 = zeros(3,num_q);
p_S_2 = zeros(3,num_q);
p_S_3 = zeros(3,num_q);
for i = 1:num_q
    T1 = fkine_EMDH(EMDH_a,q_1(:,i));
    temp = T1(:,:,N)\[p_B_1;1];
    p_S_1(:,i) = temp(1:3)+(randn(3,1)*noise);
    T2 = fkine_EMDH(EMDH_a,q_2(:,i));
    temp = T2(:,:,N)\[p_B_2;1];
    p_S_2(:,i) = temp(1:3)+(randn(3,1)*noise);
    T3 = fkine_EMDH(EMDH_a,q_3(:,i));
    temp = T3(:,:,N)\[p_B_3;1];
    p_S_3(:,i) = temp(1:3)+(randn(3,1)*noise);
end

% save data
save('./data/sim_data_EMDH_noise_0.mat')
