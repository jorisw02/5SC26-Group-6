function dx = formation_control_displacement(t, x, L, P_desired)
    dx = -kron(L,eye(3))*(x-P_desired);
end

