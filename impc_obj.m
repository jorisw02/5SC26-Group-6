function [Phi_I, Gamma_I, Omega, Psi] = impc_obj(A, B, C, Q, P, R, N)
    [ny, nx] = size(C);  % Output and state dimensions

    Phi_I = zeros(ny*N, nx+ny);
    Gamma_I = kron(eye(N),C*B); 

    temp_CA = 0;
    temp_CAB = C*B;
    for i = 1:N
        temp_CA = temp_CA + C*A^i;
        Phi_I((i-1)*ny+1:i*ny,1:nx) = temp_CA;
        Phi_I((i-1)*ny+1:i*ny,nx+1:end) = eye(ny);
        
        if i ~= 1
            temp_CAB = temp_CAB + C*A^(i-1)*B;
            Gamma_I = Gamma_I + kron(diag(ones(1,N-i+1),-i+1),temp_CAB);
        end
    end
    Omega = blkdiag(kron(eye(N-1),Q),P);
    Psi = kron(eye(N),R);
end
