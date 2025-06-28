function [sim_ref_x,sim_ref_y,sim_ref_z,sim_ref_yaw] = makeSetpoint(SPsel,tVec)

switch SPsel
    case -1
        %% Define setpoint1 - Ramp + Hover in z
        
        ref0.x = zeros(size(tVec));
        ref0.y = zeros(size(tVec));
        ref0.z = [zeros(1,100) ones(1,length(tVec)-100)];
        ref0.yaw = zeros(size(tVec));
        
        sim_ref_x   = timetable(seconds(tVec)',ref0.x');
        sim_ref_y   = timetable(seconds(tVec)',ref0.y');
        sim_ref_z   = timetable(seconds(tVec)',ref0.z');
        sim_ref_yaw = timetable(seconds(tVec)',ref0.yaw');
    case 0
        %% Define setpoint1 - Ramp + Hover in z
        
        ref0.x = zeros(size(tVec));
        ref0.y = zeros(size(tVec));
        ref0.z = zeros(size(tVec));
        ref0.yaw = zeros(size(tVec));
        
        sim_ref_x   = timetable(seconds(tVec)',ref0.x');
        sim_ref_y   = timetable(seconds(tVec)',ref0.y');
        sim_ref_z   = timetable(seconds(tVec)',ref0.z');
        sim_ref_yaw = timetable(seconds(tVec)',ref0.yaw');
    case 1
        %% Define setpoint1 - Ramp + Hover in z
        rampUntil = 3; % [s]
        
        ref1.x = 0.01*(tVec.*(tVec<rampUntil)+rampUntil*(tVec>=rampUntil));
        ref1.y = 0.01*(tVec.*(tVec<rampUntil)+rampUntil*(tVec>=rampUntil));
        ref1.z = 0.01*(tVec.*(tVec<rampUntil)+rampUntil*(tVec>=rampUntil));
        ref1.yaw = zeros(size(tVec));
        
        sim_ref_x   = timetable(seconds(tVec)',ref1.x');
        sim_ref_y   = timetable(seconds(tVec)',ref1.y');
        sim_ref_z   = timetable(seconds(tVec)',ref1.z');
        sim_ref_yaw = timetable(seconds(tVec)',ref1.yaw');

    case 2
        %% Define setpoint2 - cricle in x and y + Hover in z
        rampUntil = 3; % [s]
        
        ref2.x = 0.1.*cos(0.5.*tVec);
        ref2.y = 0.1.*sin(0.5.*tVec);
        ref2.z = 0.1*(tVec.*(tVec<rampUntil)+rampUntil*(tVec>=rampUntil));
        % ref2.z = zeros(size(tVec));
        ref2.yaw = zeros(size(tVec));
        
        sim_ref_x   = timetable(seconds(tVec)',ref2.x');
        sim_ref_y   = timetable(seconds(tVec)',ref2.y');
        sim_ref_z   = timetable(seconds(tVec)',ref2.z');
        sim_ref_yaw = timetable(seconds(tVec)',ref2.yaw');

    case 3
        %% Define setpoint3 - cricle in x, y and z
        ref2.x = 0.1.*cos(0.5.*tVec);
        ref2.y = 0.1.*sin(0.5.*tVec);
        ref2.z = 0.2.*sin(0.3.*tVec);
        ref2.yaw = zeros(size(tVec));
        
        sim_ref_x   = timetable(seconds(tVec)',ref2.x');
        sim_ref_y   = timetable(seconds(tVec)',ref2.y');
        sim_ref_z   = timetable(seconds(tVec)',ref2.z');
        sim_ref_yaw = timetable(seconds(tVec)',ref2.yaw');

    case 4
        %% Define setpoint4 - cricle in x and y + Hover in z
        rampUntil = 3; % [s]
        
        ref2.x = 0.1.*cos(0.5.*tVec);
        ref2.y = 0.1.*sin(0.5.*tVec);
        % ref2.z = 0.1*(tVec.*(tVec<rampUntil)+rampUntil*(tVec>=rampUntil));
        ref2.z = zeros(size(tVec));
        ref2.yaw = zeros(size(tVec));
        
        sim_ref_x   = timetable(seconds(tVec)',ref2.x');
        sim_ref_y   = timetable(seconds(tVec)',ref2.y');
        sim_ref_z   = timetable(seconds(tVec)',ref2.z');
        sim_ref_yaw = timetable(seconds(tVec)',ref2.yaw');

    case 5
        %% Define setpoint5 - ramp in z only
        rampUntil = 3; % [s]
        
        ref2.x = zeros(size(tVec));
        ref2.y = zeros(size(tVec));
        ref2.z = 0.1*(tVec.*(tVec<rampUntil)+rampUntil*(tVec>=rampUntil));
        % ref2.z = zeros(size(tVec));
        ref2.yaw = zeros(size(tVec));
        
        sim_ref_x   = timetable(seconds(tVec)',ref2.x');
        sim_ref_y   = timetable(seconds(tVec)',ref2.y');
        sim_ref_z   = timetable(seconds(tVec)',ref2.z');
        sim_ref_yaw = timetable(seconds(tVec)',ref2.yaw');

    case 6
        %% Define setpoint6 - Figure 8 tilted in Rx
        Ts = tVec(2)-tVec(1);
        [~,x_vec,y_vec,z_vec] = Figure8_Ref3D_v2(tVec(end), Ts, 0.5, 'rolled', 0);
        
        ref2.x = x_vec;
        ref2.y = y_vec;
        ref2.z = z_vec;
        ref2.yaw = zeros(size(tVec));
        
        sim_ref_x   = timetable(seconds(tVec)',ref2.x');
        sim_ref_y   = timetable(seconds(tVec)',ref2.y');
        sim_ref_z   = timetable(seconds(tVec)',ref2.z');
        sim_ref_yaw = timetable(seconds(tVec)',ref2.yaw');

    case 7
        %% Define setpoint7 - Figure 8 tilted in Ry
        Ts = tVec(2)-tVec(1);
        [~,x_vec,y_vec,z_vec] = Figure8_Ref3D_v2(tVec(end), Ts, 0.2, 'pitched', 0);
        
        ref2.x = x_vec;
        ref2.y = y_vec;
        ref2.z = z_vec;
        ref2.yaw = zeros(size(tVec));
        
        sim_ref_x   = timetable(seconds(tVec)',ref2.x');
        sim_ref_y   = timetable(seconds(tVec)',ref2.y');
        sim_ref_z   = timetable(seconds(tVec)',ref2.z');
        sim_ref_yaw = timetable(seconds(tVec)',ref2.yaw');
end

end