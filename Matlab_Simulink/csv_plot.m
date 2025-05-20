% filepath: /home/eswarm/mecanum/Matlab_Simulink/csv_plot.m
% Script pour afficher les commandes reçues par robot à partir d'un CSV

% --- Paramètres ---
csv_file = '../csv/cmd_vel_log_20250520_153042.csv'; % Adapter le chemin si besoin

% --- Lecture du CSV ---
T = readtable(csv_file, 'Delimiter', ',', 'ReadVariableNames', true);

robot_names = T.Properties.VariableNames;
n_robots = numel(robot_names);

figure;
for i = 1:n_robots
    % Récupère les timestamps non vides pour ce robot, conversion robuste
    timestamps = T.(robot_names{i});
    % Si ce n'est pas un cell array, convertit en cellstr
    if ~iscell(timestamps)
        timestamps = cellstr(string(timestamps));
    end
    idx = ~cellfun(@isempty, timestamps);
    times = timestamps(idx);
    % Convertit en valeurs numériques (secondes depuis le début)
    if isempty(times)
        t_sec = [];
    else
        t0 = datetime(times{1}, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSSSSS');
        t_dt = datetime(times, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSSSSS');
        t_sec = seconds(t_dt - t0);
    end

    subplot(n_robots,1,i);
    plot(t_sec, ones(size(t_sec)), 'k.', 'MarkerSize', 12); % Un point par commande
    ylabel(robot_names{i});
    xlim([0 inf]);
    ylim([0 2]);
    set(gca, 'YTick', []);
    title(sprintf('%s (%d commandes)', robot_names{i}, numel(t_sec)));
end
xlabel('Temps (s)');
sgtitle('Commandes reçues par robot');

% Légende globale
legend(arrayfun(@(i) sprintf('%s: %d pts', robot_names{i}, sum(~cellfun(@isempty, cellstr(string(T.(robot_names{i})))))), ...
    1:n_robots, 'UniformOutput', false), 'Location', 'bestoutside');