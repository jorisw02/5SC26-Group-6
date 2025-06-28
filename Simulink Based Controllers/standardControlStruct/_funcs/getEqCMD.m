function cmd_eq = getEqCMD(m,g)

syms cmdT cmdRoll cmdPitch cmdYaw  

% Define the cmd values from the control inputs
cmdi = [1 -0.5  0.5  1;...
        1 -0.5 -0.5 -1;...
        1  0.5 -0.5  1;...
        1  0.5  0.5 -1]*[cmdT;cmdRoll;cmdPitch;cmdYaw];

% Define the parameters for quadratic relation of cmd and Force
Kf2 = 2.130295e-11;
Kf1 = 1.032633e-6;
Kf0 = 5.484560e-4;

% Define the parameters for afine relation of Force and Moment
Km1 = 0.005964552;
Km0 = 1.563383;

% Make the Forces and the Moments
Fi = Kf2*cmdi.^2+Kf1*cmdi+Kf0;
Mi = Km1*Fi+Km0;

% Defnie the total Thrust Force of the drone
F = sum(Fi);

% Fill in zero for all rotations since we want to have cmd at hover
F = subs(F,{'cmdRoll','cmdPitch','cmdYaw'},[0,0,0]);

% Get the coefficients of the cmd to Force relation
F_coeffs = fliplr(coeffs(F,'cmdT'));

% Subtract m*g to get an equation that is =0
F_coeffs(3) = F_coeffs(3)-m*g;

% Get the roots of that equation to get the possible cmd's
cmd_eq = roots(F_coeffs);

% Only the positive cmd is possible, select that one
cmd_eq = (cmd_eq(cmd_eq>0));

end