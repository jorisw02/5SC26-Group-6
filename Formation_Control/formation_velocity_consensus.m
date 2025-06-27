function dx = formation_velocity_consensus(t,x,L,P_desired)
    dx = zeros(18,1); % helps avoid undefined warnings
    % p = x(1:9);
    % v = x(10:18);
    % the gain to the velocity consensus part
    k_v = 2;
    % flocking dynamics
    dx(1:9) = x(10:18);
    dx(10:18) = - k_v*kron(L,eye(3))*x(10:18) -kron(L,eye(3))*(x(1:9) - P_desired);
end