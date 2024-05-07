function ksi = vee(T)
% inverse operation of hat()

if size(T,1) == 4
    ksi = zeros(6,1);
    ksi(1:3) = T(1:3,4);
    ksi(4) = -T(2,3);
    ksi(5) = T(1,3);
    ksi(6) = -T(1,2);
else
    ksi = zeros(3,1);
    ksi(1) = -T(2,3);
    ksi(2) = T(1,3);
    ksi(3) = -T(1,2);
end
end