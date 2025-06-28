
m = 0.035;

g = 9.81;
I = [1.4, 0, 0;
     0, 1.4, 0;
     0, 0, 2.17]*1e-5;
invI = inv(I);

L = 0.048;

save('sysParam.mat','m','g','I','invI','L');

