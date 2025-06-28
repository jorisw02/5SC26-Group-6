function noise4Sim = makeNoise_V2(noiseOn,tVec)
    load('sys_param2.mat');
    Ts = 0.01;
    Tend = tVec(end);
    tVec_Ts = linspace(0,Tend,Tend/Ts+1);
    idxEnd = Tend/Ts+1;

    % Posibble noise signals for realism
    % !!Variances are semi randomly chosen!!

    % Plant output noise (Like OE model)
    noise4Sim.noNoise = zeros(size(input_noise(1,1:idxEnd)));

    if noiseOn
        noise4Sim.Thrust = timetable(seconds(tVec_Ts)',input_noise(1,1:idxEnd)');
        noise4Sim.Mx     = timetable(seconds(tVec_Ts)',input_noise(2,1:idxEnd)');
        noise4Sim.My     = timetable(seconds(tVec_Ts)',input_noise(3,1:idxEnd)');
        noise4Sim.Mz     = timetable(seconds(tVec_Ts)',input_noise(4,1:idxEnd)');

        noise4Sim.x      = timetable(seconds(tVec_Ts)',noise(1,1:idxEnd)');
        noise4Sim.y      = timetable(seconds(tVec_Ts)',noise(2,1:idxEnd)');
        noise4Sim.z      = timetable(seconds(tVec_Ts)',noise(3,1:idxEnd)');
        noise4Sim.phi    = timetable(seconds(tVec_Ts)',noise(7,1:idxEnd)');
        noise4Sim.theta  = timetable(seconds(tVec_Ts)',noise(8,1:idxEnd)');
        noise4Sim.psi    = timetable(seconds(tVec_Ts)',noise(9,1:idxEnd)');
    else
        noise4Sim.Thrust = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');
        noise4Sim.Mx     = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');
        noise4Sim.My     = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');
        noise4Sim.Mz     = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');

        noise4Sim.x      = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');
        noise4Sim.y      = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');
        noise4Sim.z      = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');
        noise4Sim.phi    = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');
        noise4Sim.theta  = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');
        noise4Sim.psi    = timetable(seconds(tVec_Ts)',noise4Sim.noNoise');
    end


end