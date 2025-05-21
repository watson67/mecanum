% === Trajectoire théorique ===
a = 1.0;
b = 0.23;
num_points = 16;
start_x = -0.64;
start_y = -0.77;

X = zeros(num_points + 2, 1);
Y = zeros(num_points + 2, 1);

X(1) = start_x;
Y(1) = start_y;

for i = 1:num_points-1
    t = 2 * pi * (i-1) / num_points;
    X(i+1) = a * sin(t);
    Y(i+1) = b * sin(2 * t);
end

X(end) = start_x;
Y(end) = start_y;

% === Trajectoire réelle depuis CSV ===
% Remplace le nom du fichier par le tien si nécessaire
data = readtable("~/mecanum/csv/barycenter_logger.csv");

% Extraire x et y
x_real = data.x;
y_real = data.y;

% === Tracé ===
figure;
hold on;
grid on;
axis equal;

% Trajectoire théorique (points cibles)
plot(-Y, X, '-o', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', 'Trajectoire théorique');

% Numérotation des points
for i = 1:length(X)
    text(-Y(i)+0.01, X(i)+0.01, num2str(i-1), 'FontSize', 8);
end

% Trajectoire réelle (essaim)
plot(-y_real, x_real, '-', 'Color', [0.85 0.1 0.1], 'LineWidth', 2, 'DisplayName', 'Trajectoire réelle');

xlabel('y (positif vers la gauche)');
ylabel('x (positif vers le haut)');
title('Trajectoire théorique vs. réelle (repère personnalisé)');
legend;
