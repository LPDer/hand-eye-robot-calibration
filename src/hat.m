function T = hat(ksi)
% cross operation
% R3 or R6 to so(3) or se(3)

    if size(ksi,1) == 6
        T = [0     -ksi(6) ksi(5) ksi(1);
             ksi(6) 0     -ksi(4) ksi(2);
            -ksi(5) ksi(4) 0      ksi(3);
             0      0      0      0];
    else
        T = [0     -ksi(3) ksi(2);
             ksi(3) 0     -ksi(1); 
            -ksi(2) ksi(1) 0   ];
    end
end