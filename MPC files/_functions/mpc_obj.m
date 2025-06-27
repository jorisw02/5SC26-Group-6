function [Phi, Gamma, Omega,Psi] = mpc_obj(A,B,C,Q,R,P,N)
    Phi = [];
    Gamma = kron(eye(N),C*B);
    for k = 0:N-1
        if k ~= 0
            Phi = [Phi;C*A^(k+1)];
            Gamma = Gamma + kron(diag(ones(1,N-k),-k),C*A^k*B);
        else
            Phi = [Phi;C*A^(k+1)]; 
        end
    end
    Omega = blkdiag(kron(eye(N-1),Q),P);
    Psi = kron(eye(N),R);
end
