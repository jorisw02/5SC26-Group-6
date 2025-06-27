function ui = controlLaw(i, positions, edges, d_star, kP)
    ui = zeros(3,1);
    pi = positions(:, i); 

    for k = 1:size(edges,1)
        edge = edges(k,:);

        if edge(1) == i
            j = edge(2);
        elseif edge(2) == i
            j = edge(1);
        else
            continue;
        end

        pj = positions(:, j);
        dij_star = d_star(i, j);
        diff = pj - pi;

        error = norm(diff)^2 - dij_star^2;

        ui = ui + kP * error * diff;
    end
end
