function J = cal_J_POE(T,G,p)

Ad = zeros(6,6,size(T,3));
Ad(:,:,1) = Adjoint(eye(4));

for i = 2:size(T,3)
    Ad(:,:,i) = Adjoint(T(:,:,i-1));
end
F = [];
for i = 1:size(T,3)-1
    F = [F,Ad(:,:,i)-Ad(:,:,i+1)];
end
F = [F,Ad(:,:,size(T,3))];
J = -[eye(3),-hat(p)]*F*G;
end