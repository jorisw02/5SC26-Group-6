function dx = quadcopter_nonlinear_dynamics(t, x, u, m, g, I, inv_I)
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);

    R = rotation_matrix(phi, theta, psi);

    dx(1:3,1) = [x(4); x(5); x(6)];
    dx(4:6,1) = 1/m*([0;0;-m*g]+R*[0;0;u(1)]);    

    dx(7:9,1) = [1, 0,        -sin(theta); 
                 0, cos(phi),  sin(phi)*cos(theta);
                 0, -sin(phi), cos(phi)*cos(theta)]* [p;q;r];
    
    cross_term = cross([p;q;r], I*[p;q;r]); 

    dx(10:12,1) = inv_I* ([u(2); u(3); u(4)] - cross_term);
end