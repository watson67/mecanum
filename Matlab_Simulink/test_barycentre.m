close all

% Coordonnées des robots
robot1 = [1, 1];
robot2 = [3, 3];
robot3 = [4, 0];

% Calcul du barycentre
barycentre = (robot1 + robot2 + robot3) / 3;

% Affichage des points
figure;
hold on;
grid on;
axis equal;

% Tracer les robots
plot(robot1(1), robot1(2), 'ro', 'MarkerSize', 10, 'DisplayName', 'Robot 1');
plot(robot2(1), robot2(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Robot 2');
plot(robot3(1), robot3(2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Robot 3');

% Tracer le barycentre
plot(barycentre(1), barycentre(2), 'kx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Barycentre');

% Annotation
legend;
title('Position des robots et de leur barycentre');
xlabel('X');
ylabel('Y');
