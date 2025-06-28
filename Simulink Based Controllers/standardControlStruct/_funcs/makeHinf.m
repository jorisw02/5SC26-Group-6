function makeHinf(linSys,useWeights,varargin)
    bestWeightSet = 1;
    addpath('./_weights/')
    if useWeights
        if nargin==3
            weightSel = varargin{1};
        else
            weightSel = bestWeightSet;
        end
        disp(['Using best weighting filter set (',num2str(weightSel),')']);
        loadWeightFile = ['./_weights/Wfilters',num2str(weightSel),'.mat']
        load(loadWeightFile);
        [K,CL,gamma] = hinfsyn(Wy_tot*linSys*Wu_tot,6,4);
        %%%
        stab_Ptilde = Wy_tot*linSys*Wu_tot;
        stab_P = linSys;
        stab_Wy = Wy_tot;
        stab_Wu = Wu_tot;
        %%%
        stab_CL_weighted = Wy_tot(1:13,1:13)*CL*Wu_tot(1:7,1:7);
        stab_C = CL;
        stab_Wy_short = Wy_tot(1:13,1:13);
        stab_Wu_short = Wu_tot(1:7,1:7);
        %%%
    else
        weightSel = 0;
        [K,CL,gamma] = hinfsyn(linSys,6,4);
        % [K,gamma] = sdhinfsyn(linSys,6,4,'Ts',0.001);
        % CL = lft(linSys,K);
        stabCheck = 0;
    end

    if gamma>1
        warning(['Weighting filters (performance specs) are not satisfied, gamma: ',num2str(gamma)])
    end
    K.inputName = {'err_x','err_y','err_z','phi','theta','psi'};
    K.outputName = {'Thrust','M1','M2','M3'};
    % save(['./_controllers/Hinf_W',num2str(weightSel),'.mat'],'K','CL','gamma','stab_Ptilde','stab_Wy','stab_Wu','stab_P','stab_CL_weighted','stab_C','stab_Wy_short','stab_Wu_short');
    save(['./_controllers/Hinf_W',num2str(weightSel),'.mat'],'K','CL','gamma');

end