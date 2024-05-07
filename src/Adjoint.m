function Ad=Adjoint(g)
%Çó¾ØÕógµÄAdjoint¾ØÕó
Ad_11=g([1,2,3],[1,2,3]);
Ad_12=hat(g(1:3,4))*g([1,2,3],[1,2,3]);
Ad_21=zeros(3);
Ad_22=g([1,2,3],[1,2,3]);
Ad=[Ad_11,Ad_12;Ad_21,Ad_22];
