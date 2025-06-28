function [SP,tVec] = SPBuilding(Ts)


figure6 = importfile_figureZero('/figure6');
timeVecs = figure6.duration;
tVec = linspace(0,sum(timeVecs),sum(timeVecs)/Ts+2);

totTime = 0;
for i=1:length(timeVecs)
    if i~=1
        totTime = totTime+timeVecs(i-1);
    else
        totTime = 0;
    end
    for j=(0:Ts:timeVecs(i)/Ts)*Ts
        % SP.x(round((j+totTime)/Ts+1)) = figure6(i,:).x7*j^7+figure6(i,:).x6*j^6+figure6(i,:).x5*j^5+figure6(i,:).x4*j^4+figure6(i,:).x3*j^3+figure6(i,:).x2*j^2+figure6(i,:).x1*j+figure6(i,:).x0;
        % SP.y(round((j+totTime)/Ts+1)) = figure6(i,:).y7*j^7+figure6(i,:).y6*j^6+figure6(i,:).y5*j^5+figure6(i,:).y4*j^4+figure6(i,:).y3*j^3+figure6(i,:).y2*j^2+figure6(i,:).y1*j+figure6(i,:).y0;
        % SP.z(round((j+totTime)/Ts+1)) = figure6(i,:).z7*j^7+figure6(i,:).z6*j^6+figure6(i,:).z5*j^5+figure6(i,:).z4*j^4+figure6(i,:).z3*j^3+figure6(i,:).z2*j^2+figure6(i,:).z1*j+figure6(i,:).z0;
        coeffs_x = [figure6(i,:).x7 figure6(i,:).x6 figure6(i,:).x5 figure6(i,:).x4 figure6(i,:).x3 figure6(i,:).x2 figure6(i,:).x1 figure6(i,:).x0];
        SP.x(round((j+totTime)/Ts+1)) = polyval(coeffs_x,j);
        coeffs_y = [figure6(i,:).y7 figure6(i,:).y6 figure6(i,:).y5 figure6(i,:).y4 figure6(i,:).y3 figure6(i,:).y2 figure6(i,:).y1 figure6(i,:).y0];
        SP.y(round((j+totTime)/Ts+1)) = polyval(coeffs_y,j);
        coeffs_z = [figure6(i,:).z7 figure6(i,:).z6 figure6(i,:).z5 figure6(i,:).z4 figure6(i,:).z3 figure6(i,:).z2 figure6(i,:).z1 figure6(i,:).z0];
        SP.z(round((j+totTime)/Ts+1)) = polyval(coeffs_z,j);
    end
end

SP.x = [SP.x fliplr(SP.x)];
SP.y = [SP.y fliplr(SP.y)];
SP.z = [SP.z fliplr(SP.z)];
tVec = [tVec tVec+tVec(end)];
%%
% figure(1);clf;plot3(SPx(45:end),SPy(45:end),SPz(45:end))
% 
% figure(2);clf;
%     subplot(311)
%         plot(tVec,SPx)
%     subplot(312)
%         plot(tVec,SPy)
%     subplot(313)
%         plot(tVec,SPz)

end






