% Simulation numérique d'un essaim de 3 robots holonomes avec loi de consensus
clear all; close all; clc;

% Paramètres des robots
R = 0.05;          % Rayon des roues (m)
a = 0.15;          % Distance du centre aux roues selon x (m)
b = 0.15;          % Distance du centre aux roues selon y (m)
alpha = pi/4;      % Angle d'inclinaison des rouleaux (45°)

% Paramètres de simulation
dt = 0.05;         % Pas de temps (s)
t_end = 20;        % Temps de fin de simulation (s)
t = 0:dt:t_end;    % Vecteur temps
N = length(t);     % Nombre d'itérations

% Paramètres de formation et consensus
d12 = 1.0;         % Distance désirée entre robots 1 et 2 (m)
d23 = 1.0;         % Distance désirée entre robots 2 et 3 (m)
d31 = 1.0;         % Distance désirée entre robots 3 et 1 (m)
k_consensus = 1.5; % Gain pour la loi de consensus

% État initial des robots (positions et orientations)
% On place les robots en triangle équilatéral
X = zeros(9, N);   % Matrice d'état [x1;y1;theta1;x2;y2;theta2;x3;y3;theta3]
X(:,1) = [0; 0; 0;                         % Robot 1: à l'origine
          d12; 0; 0;                       % Robot 2: à droite de robot 1
          d12/2; sqrt(3)*d12/2; 0];        % Robot 3: pour former un triangle équilatéral

% Matrice de conversion pour vitesses linéaires/angulaires vers vitesses de roues
A_inv = [1, -1/tan(alpha), (b+a*tan(alpha))/tan(alpha);
         1, 1/tan(alpha), (b+a*tan(alpha))/tan(alpha);
         1, -1/tan(alpha), (b+a*tan(alpha))/tan(alpha);
         1, 1/tan(alpha), (b+a*tan(alpha))/tan(alpha)];

% Matrice de conversion inverse (vitesses des roues vers vitesses du robot)
A_dir = (R/4) * [1, 1, 1, 1;
                -1, 1, -1, 1;
                -1/(a+b), -1/(a+b), 1/(a+b), 1/(a+b)];

% Historique des positions du barycentre et des commandes
barycentre = zeros(2, N);
commandes_barycentre = zeros(3, N);
commandes_robots = zeros(9, N);
wheel_speeds = zeros(12, N);  % 4 roues x 3 robots

% Calcul du barycentre initial
barycentre(:,1) = [mean(X([1 4 7],1)); mean(X([2 5 8],1))];

% Définition de la trajectoire du barycentre (cercle)
rayon_cercle = 2.0;  % Rayon du cercle (m)
omega_cercle = 0.2;  % Vitesse angulaire (rad/s)
centre_cercle = [0; 0];  % Centre du cercle (m)

% Matrice d'adjacence du graphe de communication (topologie complète)
Adj = [0 1 1;   % Robot 1 est connecté à 2 et 3
       1 0 1;   % Robot 2 est connecté à 1 et 3
       1 1 0];  % Robot 3 est connecté à 1 et 2

% Matrice laplacienne du graphe
L = diag(sum(Adj,2)) - Adj;

% Position relative désirée des robots par rapport au barycentre (formation triangle équilatéral)
delta_des = [0, -1/sqrt(3)*d12;     % Robot 1: en bas
             d12/2, 1/(2*sqrt(3))*d12;   % Robot 2: en haut à droite
            -d12/2, 1/(2*sqrt(3))*d12];  % Robot 3: en haut à gauche

% Boucle de simulation
for k = 1:N-1
    % Calcul des commandes pour le barycentre (suivre un cercle)
    t_current = t(k);
    
    % Position désirée du barycentre sur le cercle
    pos_des_x = centre_cercle(1) + rayon_cercle * cos(omega_cercle * t_current);
    pos_des_y = centre_cercle(2) + rayon_cercle * sin(omega_cercle * t_current);
    
    % Vitesse désirée du barycentre (tangente au cercle)
    vx_c = -rayon_cercle * omega_cercle * sin(omega_cercle * t_current);
    vy_c = rayon_cercle * omega_cercle * cos(omega_cercle * t_current);
    omega_c = 0;  % Le barycentre ne tourne pas sur lui-même
    
    % Stockage des commandes du barycentre
    commandes_barycentre(:,k) = [vx_c; vy_c; omega_c];
    
    % État actuel des robots
    x1 = X(1,k); y1 = X(2,k); theta1 = X(3,k);
    x2 = X(4,k); y2 = X(5,k); theta2 = X(6,k);
    x3 = X(7,k); y3 = X(8,k); theta3 = X(9,k);
    
    % Calcul du barycentre actuel
    xc = (x1 + x2 + x3)/3;
    yc = (y1 + y2 + y3)/3;
    
    % Positions relatives actuelles par rapport au barycentre
    delta_act = [x1-xc, y1-yc;
                 x2-xc, y2-yc;
                 x3-xc, y3-yc];
    
    % Erreurs de position relative pour la formation
    delta_err = delta_act - delta_des;
    
    % Positions des robots
    pos = [x1, y1; x2, y2; x3, y3];
    
    % Application de la loi de consensus
    u_consensus = zeros(3, 2);  % Initialisation des commandes de consensus
    
    % Pour chaque robot i
    for i = 1:3
        % Erreur de formation (pour maintenir la position relative au barycentre)
        e_formation = delta_err(i, :);
        
        % Consensus sur les positions relatives 
        % (le robot i ajuste sa position en fonction des positions des autres robots)
        sum_consensus = zeros(1, 2);
        for j = 1:3
            if Adj(i, j) == 1  % Si robots i et j sont connectés
                % Calcul de la différence entre les positions actuelles
                diff_pos = pos(j, :) - pos(i, :);
                
                % Calcul de la distance actuelle
                d_ij = norm(diff_pos);
                
                % Détermination de la distance désirée
                if (i==1 && j==2) || (i==2 && j==1)
                    d_des = d12;
                elseif (i==2 && j==3) || (i==3 && j==2)
                    d_des = d23;
                else
                    d_des = d31;
                end
                
                % Calcul de l'erreur de distance
                err_dist = d_ij - d_des;
                
                % Contribution du consensus (pondérée par l'erreur de distance)
                if d_ij > 0  % Éviter la division par zéro
                    sum_consensus = sum_consensus + err_dist * diff_pos / d_ij;
                end
            end
        end
        
        % Commande finale pour le robot i: consensus + correction de formation
        u_consensus(i, :) = k_consensus * sum_consensus - 0.5 * e_formation;
    end
    
    % Distribution de la commande du barycentre aux robots individuels
    % avec ajout du terme de consensus
    vx1 = vx_c + u_consensus(1, 1);
    vy1 = vy_c + u_consensus(1, 2);
    vx2 = vx_c + u_consensus(2, 1);
    vy2 = vy_c + u_consensus(2, 2);
    vx3 = vx_c + u_consensus(3, 1);
    vy3 = vy_c + u_consensus(3, 2);
    
    % Rotation des vitesses dans le repère du robot (global -> local)
    vx1_local = vx1*cos(theta1) + vy1*sin(theta1);
    vy1_local = -vx1*sin(theta1) + vy1*cos(theta1);
    
    vx2_local = vx2*cos(theta2) + vy2*sin(theta2);
    vy2_local = -vx2*sin(theta2) + vy2*cos(theta2);
    
    vx3_local = vx3*cos(theta3) + vy3*sin(theta3);
    vy3_local = -vx3*sin(theta3) + vy3*cos(theta3);
    
    % Commandes finales pour les robots (dans le repère local)
    U1 = [vx1_local; vy1_local; omega_c];
    U2 = [vx2_local; vy2_local; omega_c];
    U3 = [vx3_local; vy3_local; omega_c];
    
    % Stockage des commandes des robots
    commandes_robots(:,k) = [vx1_local; vy1_local; omega_c;
                            vx2_local; vy2_local; omega_c;
                            vx3_local; vy3_local; omega_c];
    
    % Calcul des vitesses des roues pour chaque robot
    wheel_speeds1 = (1/R) * A_inv * U1;
    wheel_speeds2 = (1/R) * A_inv * U2;
    wheel_speeds3 = (1/R) * A_inv * U3;
    
    % Stockage des vitesses des roues
    wheel_speeds(:,k) = [wheel_speeds1; wheel_speeds2; wheel_speeds3];
    
    % Mise à jour de l'état des robots (intégration Euler)
    % Robot 1
    dx1 = vx1_local*cos(theta1) - vy1_local*sin(theta1);
    dy1 = vx1_local*sin(theta1) + vy1_local*cos(theta1);
    dtheta1 = omega_c;
    
    % Robot 2
    dx2 = vx2_local*cos(theta2) - vy2_local*sin(theta2);
    dy2 = vx2_local*sin(theta2) + vy2_local*cos(theta2);
    dtheta2 = omega_c;
    
    % Robot 3
    dx3 = vx3_local*cos(theta3) - vy3_local*sin(theta3);
    dy3 = vx3_local*sin(theta3) + vy3_local*cos(theta3);
    dtheta3 = omega_c;
    
    % Mise à jour de l'état
    X(:,k+1) = X(:,k) + dt * [dx1; dy1; dtheta1; dx2; dy2; dtheta2; dx3; dy3; dtheta3];
    
    % Calcul du nouveau barycentre
    barycentre(:,k+1) = [mean(X([1 4 7],k+1)); mean(X([2 5 8],k+1))];
end

%% Visualisation des résultats
figure(1);
set(gcf, 'Position', [100, 100, 1000, 800]);

% Subplot 1: Trajectoire des robots et barycentre
subplot(2, 2, [1,3]);
hold on;
plot(X(1,:), X(2,:), 'r-', 'LineWidth', 1.5);  % Robot 1
plot(X(4,:), X(5,:), 'g-', 'LineWidth', 1.5);  % Robot 2
plot(X(7,:), X(8,:), 'b-', 'LineWidth', 1.5);  % Robot 3
plot(barycentre(1,:), barycentre(2,:), 'k--', 'LineWidth', 2);  % Barycentre

% Dessin du cercle de référence
theta_circle = linspace(0, 2*pi, 100);
x_circle = centre_cercle(1) + rayon_cercle * cos(theta_circle);
y_circle = centre_cercle(2) + rayon_cercle * sin(theta_circle);
plot(x_circle, y_circle, 'k:', 'LineWidth', 1);

% Points pour quelques positions clés (début, milieu, fin)
indices = [1, round(N/3), round(2*N/3), N];
colors = {'r', 'g', 'b'};
markers = {'o', 's', 'd'};

for idx = indices
    % Dessiner la formation à cet instant
    for r = 1:3
        robot_x = X(3*r-2, idx);
        robot_y = X(3*r-1, idx);
        theta = X(3*r, idx);
        
        % Position du robot
        plot(robot_x, robot_y, [colors{r}, markers{1}], 'MarkerSize', 8, 'LineWidth', 2);
        
        % Direction du robot (petite flèche)
        quiver(robot_x, robot_y, 0.2*cos(theta), 0.2*sin(theta), 0, colors{r}, 'LineWidth', 2);
    end
    
    % Dessiner le barycentre
    bary_x = barycentre(1, idx);
    bary_y = barycentre(2, idx);
    plot(bary_x, bary_y, 'ko', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Dessiner les lignes de la formation
    plot([X(1,idx), X(4,idx)], [X(2,idx), X(5,idx)], 'k-', 'LineWidth', 1.5);
    plot([X(4,idx), X(7,idx)], [X(5,idx), X(8,idx)], 'k-', 'LineWidth', 1.5);
    plot([X(7,idx), X(1,idx)], [X(8,idx), X(2,idx)], 'k-', 'LineWidth', 1.5);
end

grid on;
xlabel('Position X (m)');
ylabel('Position Y (m)');
title('Trajectoires des robots et du barycentre');
legend('Robot 1', 'Robot 2', 'Robot 3', 'Barycentre', 'Trajectoire de référence');
axis equal;

% Subplot 2: Évolution des distances entre robots
subplot(2, 2, 2);
hold on;

% Calcul des distances pour chaque pas de temps
distances = zeros(3, N);
for k = 1:N
    distances(1,k) = sqrt((X(1,k)-X(4,k))^2 + (X(2,k)-X(5,k))^2);  % d12
    distances(2,k) = sqrt((X(4,k)-X(7,k))^2 + (X(5,k)-X(8,k))^2);  % d23
    distances(3,k) = sqrt((X(7,k)-X(1,k))^2 + (X(8,k)-X(2,k))^2);  % d31
end

plot(t, distances(1,:), 'r-', 'LineWidth', 1.5);
plot(t, distances(2,:), 'g-', 'LineWidth', 1.5);
plot(t, distances(3,:), 'b-', 'LineWidth', 1.5);

% Lignes horizontales pour les distances désirées
plot(t, d12*ones(size(t)), 'r--', 'LineWidth', 1);
plot(t, d23*ones(size(t)), 'g--', 'LineWidth', 1);
plot(t, d31*ones(size(t)), 'b--', 'LineWidth', 1);

grid on;
xlabel('Temps (s)');
ylabel('Distance (m)');
title('Distances entre robots');
legend('Distance 1-2', 'Distance 2-3', 'Distance 3-1', 'Désirée 1-2', 'Désirée 2-3', 'Désirée 3-1');

% Subplot 3: Commandes de consensus
subplot(2, 2, 4);
hold on;

% Création d'un tableau pour stocker les termes de consensus
consensus_terms = zeros(6, N-1);
for k = 1:N-1
    consensus_terms(:,k) = [u_consensus(1,1); u_consensus(1,2);
                          u_consensus(2,1); u_consensus(2,2);
                          u_consensus(3,1); u_consensus(3,2)];
end

% Affichage des termes de consensus
plot(t(1:N-1), consensus_terms(1,1:N-1), 'r-', 'LineWidth', 1.5);
plot(t(1:N-1), consensus_terms(2,1:N-1), 'r--', 'LineWidth', 1.5);
plot(t(1:N-1), consensus_terms(3,1:N-1), 'g-', 'LineWidth', 1.5);
plot(t(1:N-1), consensus_terms(4,1:N-1), 'g--', 'LineWidth', 1.5);
plot(t(1:N-1), consensus_terms(5,1:N-1), 'b-', 'LineWidth', 1.5);
plot(t(1:N-1), consensus_terms(6,1:N-1), 'b--', 'LineWidth', 1.5);

grid on;
xlabel('Temps (s)');
ylabel('Amplitude');
title('Termes de consensus');
legend('u_{cons,x} Robot 1', 'u_{cons,y} Robot 1', 'u_{cons,x} Robot 2', 'u_{cons,y} Robot 2', 'u_{cons,x} Robot 3', 'u_{cons,y} Robot 3');

% Animation (optionnelle - décommenter pour l'activer)
figure(2);
set(gcf, 'Position', [200, 200, 800, 600]);
title('Animation de l''essaim de robots avec loi de consensus');
xlabel('Position X (m)');
ylabel('Position Y (m)');
axis equal;
grid on;

% Limites pour l'animation
x_min = min([min(X(1,:)), min(X(4,:)), min(X(7,:))]) - 0.5;
x_max = max([max(X(1,:)), max(X(4,:)), max(X(7,:))]) + 0.5;
y_min = min([min(X(2,:)), min(X(5,:)), min(X(8,:))]) - 0.5;
y_max = max([max(X(2,:)), max(X(5,:)), max(X(8,:))]) + 0.5;
axis([x_min x_max y_min y_max]);

% Animation step by step
robot_size = 0.2;  % Taille pour la représentation des robots
for k = 1:10:N  % Animation plus rapide en prenant un pas de 10
    cla;  % Effacer les dessins précédents
    hold on;
    
    % Dessiner la trajectoire de référence (cercle)
    plot(x_circle, y_circle, 'k:', 'LineWidth', 1);
    
    % Dessiner les trajectoires jusqu'à l'instant actuel
    plot(X(1,1:k), X(2,1:k), 'r-', 'LineWidth', 1);
    plot(X(4,1:k), X(5,1:k), 'g-', 'LineWidth', 1);
    plot(X(7,1:k), X(8,1:k), 'b-', 'LineWidth', 1);
    plot(barycentre(1,1:k), barycentre(2,1:k), 'k--', 'LineWidth', 1.5);
    
    % Dessiner les robots à l'instant k
    for r = 1:3
        robot_x = X(3*r-2, k);
        robot_y = X(3*r-1, k);
        theta = X(3*r, k);
        
        % Couleur du robot
        if r == 1
            color = 'r';
        elseif r == 2
            color = 'g';
        else
            color = 'b';
        end
        
        % Dessiner le corps du robot (cercle)
        rectangle('Position', [robot_x-robot_size/2, robot_y-robot_size/2, robot_size, robot_size], ...
                 'Curvature', [1 1], 'FaceColor', color, 'EdgeColor', 'k');
        
        % Dessiner la direction du robot (ligne)
        quiver(robot_x, robot_y, 0.3*cos(theta), 0.3*sin(theta), 0, 'k', 'LineWidth', 2);
    end
    
    % Dessiner le barycentre
    bary_x = barycentre(1, k);
    bary_y = barycentre(2, k);
    plot(bary_x, bary_y, 'ko', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'k');
    
    % Dessiner les lignes de la formation
    plot([X(1,k), X(4,k)], [X(2,k), X(5,k)], 'k-', 'LineWidth', 1.5);
    plot([X(4,k), X(7,k)], [X(5,k), X(8,k)], 'k-', 'LineWidth', 1.5);
    plot([X(7,k), X(1,k)], [X(8,k), X(2,k)], 'k-', 'LineWidth', 1.5);
    
    % Informations sur l'instant actuel
    text(x_min + 0.5, y_max - 0.5, ['Temps: ' num2str(t(k), '%.2f') ' s'], 'FontSize', 12);
    
    grid on;
    title('Animation de l''essaim de robots avec loi de consensus');
    xlabel('Position X (m)');
    ylabel('Position Y (m)');
    legend('Trajectoire référence', 'Robot 1', 'Robot 2', 'Robot 3', 'Barycentre');
    
    drawnow;
    pause(0.01);  % Pause pour visualiser l'animation
end

% Affichage des erreurs de formation moyennes
err_mean = mean(abs(distances - [d12; d23; d31]), 2);
fprintf('Erreurs de formation moyennes :\n');
fprintf('Distance 1-2 : %.4f m\n', err_mean(1));
fprintf('Distance 2-3 : %.4f m\n', err_mean(2));
fprintf('Distance 3-1 : %.4f m\n', err_mean(3));