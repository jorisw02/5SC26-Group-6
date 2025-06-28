function noise = makeNoise(noiseOn)
    % Posibble noise signals for realism
    % !!Variances are semi randomly chosen!!

    % Plant output noise (Like OE model)
    noise.out.mean = zeros(1,12);
    
    % Plant input noise
    noise.Thrust.mean = 0;
    noise.M1.mean = 0;
    noise.M2.mean = 0;
    noise.M3.mean = 0;

    if noiseOn
        noise.M3.var  = 0.000000001;   %10mNm variance
        noise.M2.var  = 0.000000001;   %10mNm variance
        noise.M1.var  = 0.000000001;   %10mNm variance
        noise.Thrust.var = 0.0000001; %100mN variance
        noise.out.var = [0.000000005.*ones(1,3) 0.00000001.*ones(1,3) 0.0000000005.*ones(1,3) 0.000000001.*ones(1,3)];  %2mm var
    else
        noise.M3.var  = 0;
        noise.M2.var  = 0;
        noise.M1.var  = 0;
        noise.Thrust.var = 0;
        noise.out.var = zeros(1,12);
    end
    
    % randomly chosen noise seed to make sure the noise signals do not have the
    % same properties
    noise.out.seed    = [83391 8311 6520 45252 34547 46971 9273 4008 88711 81544 28636 64095];
    noise.Thrust.seed = 937546;
    noise.M1.seed     = 29574;
    noise.M2.seed     = 620374;
    noise.M3.seed     = 982647;
end