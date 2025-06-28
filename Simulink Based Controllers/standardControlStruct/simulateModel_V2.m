function simOut = simulateModel_V2(modelType)
    mdl = 'nonLin_Model_std_V2';
    load_system(mdl)
        path = [mdl,'/Controllers/PID controller (drone)'];
        set_param(path,'Commented','on');

        path = [mdl,'/Controllers/SS controller (H-inf)'];
        set_param(path,'Commented','on');

        path = [mdl,'/Controllers/SS controller (other)'];
        set_param(path,'Commented','on');

        path = [mdl,'/Controllers/MPC controller'];
        set_param(path,'Commented','on');

    if strcmp(modelType,'casPID')
        path = [mdl,'/Controllers/PID controller (drone)'];
        set_param(path,'Commented','off');

    elseif strcmp(modelType,'Hinf')
        path = [mdl,'/Controllers/SS controller (H-inf)'];
        set_param(path,'Commented','off');

    elseif strcmp(modelType,'SS')
        path = [mdl,'/Controllers/SS controller (other)'];
        set_param(path,'Commented','off');

    elseif strcmp(modelType,'MPC')
        path = [mdl,'/Controllers/MPC controller'];
        set_param(path,'Commented','off');
    end


    rawSimOut = sim('nonLin_Model_std_V2.slx');
    simOut.tVec = squeeze(rawSimOut.tout);

    simOut.pos.all = squeeze(rawSimOut.pos.Data)';
        simOut.pos.x        = simOut.pos.all(:,1);
        simOut.pos.y        = simOut.pos.all(:,2);
        simOut.pos.z        = simOut.pos.all(:,3);
        simOut.pos.xdot     = simOut.pos.all(:,4);
        simOut.pos.ydot     = simOut.pos.all(:,5);
        simOut.pos.zdot     = simOut.pos.all(:,6);
        simOut.pos.phi      = simOut.pos.all(:,7);
        simOut.pos.theta    = simOut.pos.all(:,8);
        simOut.pos.psi      = simOut.pos.all(:,9);
        simOut.pos.phidot   = simOut.pos.all(:,10);
        simOut.pos.thetadot = simOut.pos.all(:,11);
        simOut.pos.psidot   = simOut.pos.all(:,12);

    simOut.controllerOutput.all = squeeze(rawSimOut.controllerOutput.Data);
        simOut.controllerOutput.F   = simOut.controllerOutput.all(:,1);
        simOut.controllerOutput.M1  = simOut.controllerOutput.all(:,2);
        simOut.controllerOutput.M2  = simOut.controllerOutput.all(:,3);
        simOut.controllerOutput.M3  = simOut.controllerOutput.all(:,4);

    simOut.plantInput.all = squeeze(rawSimOut.plantInput.Data);
        simOut.plantInput.F   = simOut.plantInput.all(:,1);
        simOut.plantInput.M1  = simOut.plantInput.all(:,2);
        simOut.plantInput.M2  = simOut.plantInput.all(:,3);
        simOut.plantInput.M3  = simOut.plantInput.all(:,4);

    simOut.ref.all = [squeeze(rawSimOut.posRef.Data) squeeze(rawSimOut.yawRef.Data)];
        simOut.ref.x = simOut.ref.all(:,1);
        simOut.ref.y = simOut.ref.all(:,2);
        simOut.ref.z = simOut.ref.all(:,3);
        simOut.ref.yaw = simOut.ref.all(:,4);

    simOut.stateVec.all = squeeze(rawSimOut.stateVec.Data)';
        simOut.stateVec.x        = simOut.stateVec.all(:,1);
        simOut.stateVec.y        = simOut.stateVec.all(:,2);
        simOut.stateVec.z        = simOut.stateVec.all(:,3);
        simOut.stateVec.xdot     = simOut.stateVec.all(:,4);
        simOut.stateVec.ydot     = simOut.stateVec.all(:,5);
        simOut.stateVec.zdot     = simOut.stateVec.all(:,6);
        simOut.stateVec.phi      = simOut.stateVec.all(:,7);
        simOut.stateVec.theta    = simOut.stateVec.all(:,8);
        simOut.stateVec.psi      = simOut.stateVec.all(:,9);
        simOut.stateVec.phidot   = simOut.stateVec.all(:,10);
        simOut.stateVec.thetadot = simOut.stateVec.all(:,11);
        simOut.stateVec.psidot   = simOut.stateVec.all(:,12);

    simOut.error.all = squeeze(rawSimOut.error.Data)';
        simOut.error.x   = simOut.error.all(:,1);
        simOut.error.y   = simOut.error.all(:,2);
        simOut.error.z   = simOut.error.all(:,3);
        simOut.error.yaw = simOut.error.all(:,4);
end

