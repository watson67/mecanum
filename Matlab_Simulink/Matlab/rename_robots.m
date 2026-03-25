function new_name = rename_robot(robot_name)

    robots = {'Athos', 'Porthos', 'Dartagnan', 'Edmon','Mercedes'};
    
    % Forcer en char
    if isstring(robot_name)
        robot_name = char(robot_name);
    elseif iscell(robot_name)
        robot_name = robot_name{1};
    end

    % Comparaison insensible à la casse
    idx = find(strcmpi(robot_name, robots), 1);

    if isempty(idx)
        new_name = robot_name; % fallback
    else
        new_name = sprintf('Robot_%d', idx);
    end
end