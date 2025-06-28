function sysID = addSysID(sysID_on,fs,Tend,tVec,type)
% Requires the System Identification toolbox
    % Posibble noise signals for realism
    % !!Variances are semi randomly chosen!!
    
    % sysID.band = [0.1 0.8]; % Sampling frequency
    sysID.band = [0 1]; % Sampling frequency
    sysID.timeLength = Tend*fs;
    
    sysID.Thrust.max = 1000; % Maximum signal amplitude [N]
    sysID.M1.max = 500; % Maximum signal amplitude [Nm]
    sysID.M2.max = 500; % Maximum signal amplitude [Nm]
    sysID.M3.max = 500; % Maximum signal amplitude [Nm]

    sysID.N_period = 1;

    if sysID_on    
        sysID.Thrust.PRBS = idinput(sysID.timeLength/sysID.N_period,'prbs',sysID.band,[-sysID.Thrust.max sysID.Thrust.max]);
        sysID.M1.PRBS     = idinput(sysID.timeLength/sysID.N_period,'prbs',sysID.band,[-sysID.M1.max sysID.M1.max]);
        sysID.M2.PRBS     = idinput(sysID.timeLength/sysID.N_period,'prbs',sysID.band,[-sysID.M2.max sysID.M2.max]);
        sysID.M3.PRBS     = idinput(sysID.timeLength/sysID.N_period,'prbs',sysID.band,[-sysID.M3.max sysID.M3.max]);

        sysID.Thrust.PRBS = [repmat(sysID.Thrust.PRBS,sysID.N_period,1);0];
        sysID.M1.PRBS     = [repmat(sysID.M1.PRBS,sysID.N_period,1);0];
        sysID.M2.PRBS     = [repmat(sysID.M2.PRBS,sysID.N_period,1);0];
        sysID.M3.PRBS     = [repmat(sysID.M3.PRBS,sysID.N_period,1);0];

        switch type
            case 'all'
                sysID.type = 99;
            case 'F'
                sysID.type = 1;
            case 'Mx'
                sysID.type = 2;
            case 'My'
                sysID.type = 3;
            case 'Mz'
                sysID.type = 4;
        end
      
    else
        sysID.type = 0;
        sysID.Thrust.PRBS = zeros(sysID.timeLength+1,1);
        sysID.M1.PRBS     = zeros(sysID.timeLength+1,1);
        sysID.M2.PRBS     = zeros(sysID.timeLength+1,1);
        sysID.M3.PRBS     = zeros(sysID.timeLength+1,1);
    end
    sysID.Thrust_PRBS = timetable(seconds(tVec)',sysID.Thrust.PRBS);
    sysID.M1_PRBS     = timetable(seconds(tVec)',sysID.M1.PRBS);
    sysID.M2_PRBS     = timetable(seconds(tVec)',sysID.M2.PRBS);
    sysID.M3_PRBS     = timetable(seconds(tVec)',sysID.M3.PRBS);

    if sysID.type~=0
        sysID.CL = 0;
    else
        sysID.CL = 1;
    end

end