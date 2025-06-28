function makeStandardPID()

    % Position Controller (inside a loop to reduce needed scrolling :D)
    for i=1
        C.pos.x.Kp = 2;
        C.pos.x.Ki = 0;
        C.pos.x.Kd = 0;
        
        C.pos.y.Kp = 2;
        C.pos.y.Ki = 0;
        C.pos.y.Kd = 0;
        
        C.pos.z.Kp = 2;
        C.pos.z.Ki = 0.5;
        C.pos.z.Kd = 0;
    end
    
    % Velocity Controller (inside a loop to reduce needed scrolling :D)
    for i=1
        C.vel.xdot.Kp = 25;
        C.vel.xdot.Ki = 1;
        C.vel.xdot.Kd = 0;
        
        C.vel.ydot.Kp = 25;
        C.vel.ydot.Ki = 1;
        C.vel.ydot.Kd = 0;
        
        C.vel.zdot.Kp = 25;
        C.vel.zdot.Ki = 15;
        C.vel.zdot.Kd = 0;
    end
    
    % Attitude Controller (inside a loop to reduce needed scrolling :D)
    for i=1
        C.att.phi.Kp = 6;
        C.att.phi.Ki = 3;
        C.att.phi.Kd = 0;
        
        C.att.theta.Kp = 6;
        C.att.theta.Ki = 3;
        C.att.theta.Kd = 0;
        
        C.att.psi.Kp = 6;
        C.att.psi.Ki = 1;
        C.att.psi.Kd = 0;
    end
    
    % Attitude Rate Controller (inside a loop to reduce needed scrolling :D)
    for i=1
        C.attRate.phidot.Kp = 250;
        C.attRate.phidot.Ki = 500;
        C.attRate.phidot.Kd = 2.5;
        
        C.attRate.thetadot.Kp = 250;
        C.attRate.thetadot.Ki = 500;
        C.attRate.thetadot.Kd = 2.5;
        
        C.attRate.psidot.Kp = 120;
        C.attRate.psidot.Ki = 16.7;
        C.attRate.psidot.Kd = 0;
    end
    save('./_controllers/standardPID.mat','C')
   
end




