function T = fkine_EMDH(mdh_par, q)
    N = size(mdh_par,1);
    temp = eye(4);
    T = zeros(4,4,N);
    q = [0;q;0];
    for i = 1:N
        temp = temp*fkine_single_MDH(mdh_par(i,:),q(i));
        T(:,:,i) = temp;
    end
end