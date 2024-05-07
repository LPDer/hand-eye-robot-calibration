function T = fkine_POE(ksi, q, T_0)
    num_links = size(ksi,2);
    T = zeros(4,4,num_links+1);
    temp = eye(4);
    for i = 1:num_links
        temp = temp*expm(hat(ksi(:,i))*q(i));
        T(:,:,i) = temp;
    end
    T(:,:,num_links+1) = T(:,:,num_links)*T_0;
end