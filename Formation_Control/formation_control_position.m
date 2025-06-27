function dx = formation_control_position(t, x, A, P_desired)
    dx = A*(x-P_desired);
end