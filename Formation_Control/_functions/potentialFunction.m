function V_i = potentialFunction(i, positions, edges, d_star, kP) 
    V_i = 0;
    pi = positions(:, i);   % positions is a 2xN matrix containing   
                            % the x-y position of each drone
    
    for k = 1:size(edges,1) % Iterate over the number of row
        edge = edges(k,:);  % Select a certain edge, for example
                            % edges(1,:) -> [1,2]
                            
        if edge(1) == i     % Check if the first node is actually
            j = edge(2);    % the i drone   
        elseif edge(2) == i
            j = edge(1);
        else
            continue;  
        end
        
        pj = positions(:, j);
        dij_star = d_star(i, j); 
        r = norm(pj - pi);        
        gamma = (r^2 - dij_star^2)^2;
        
        V_i = V_i + gamma;
    end
    
    V_i = (kP / 4) * V_i;
end