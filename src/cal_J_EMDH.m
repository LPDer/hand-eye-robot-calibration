function J_MDH = cal_J_EMDH(T,MDH_cur,p_B)
M_theta = zeros(6,8); M_d = zeros(6,5);
M_alpha = zeros(6,7); M_a = zeros(6,7);
M_beta = zeros(6,3);
% calculate M_theta
for j = 1:8
    alpha = MDH_cur(j,3); a = MDH_cur(j,4);
    beta = MDH_cur(j,5);
    delta_theta = [hat_om([-cosd(alpha)*sind(beta), sind(alpha),...
        cosd(alpha)*cosd(beta)]'), [a*sind(alpha)*sind(beta),...
        a*cosd(alpha), -a*sind(alpha)*cosd(beta)]'; 0 0 0 0];
    M_theta(:,j) = vee_ksi(T(:,:,j)*delta_theta...
        /T(:,:,j));
end
% calculate M_d
count = 1;
for j = [2 5 6 7 8]
    alpha = MDH_cur(j,3); beta = MDH_cur(j,5);
    delta_d = zeros(4,4);
    delta_d(1:3,4) = [-cosd(alpha)*sind(beta), sind(alpha), ...
        cosd(alpha)*cosd(beta)]';
    M_d(:,count) = inv_cross4(T(:,:,j)*delta_d...
        /T(:,:,j));
    count = count+1;
end
% calculate M_alpha and M_a
for i = 1:7
    beta = MDH_cur(i,5);
    delta_alpha = [hat_om([cosd(beta) 0 sind(beta)]'),zeros(3,1);zeros(1,4)];
    delta_a = zeros(4,4); delta_a(1,4) = cosd(beta);
    M_alpha(:,i) = inv_cross4(T(:,:,i)*delta_alpha/T(:,:,i));
    M_a(:,i) = inv_cross4(T(:,:,i)*delta_a/T(:,:,i));
end
% calculate M_beta
count = 1;
for i = [1, 3, 4]
    delta_beta = zeros(4,4); delta_beta(1,3) = 1; delta_beta(3,1) = -1;
    M_beta(:,count) = inv_cross4(T(:,:,i)*delta_beta/T(:,:,i));
    count = count+1;
end
% construct Jacobian matrix
M = [M_theta, M_d, M_alpha, M_a, M_beta];
J_MDH = [-eye(3), hat_om(p_B)]*M;
end