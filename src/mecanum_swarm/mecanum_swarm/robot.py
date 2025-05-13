import numpy as np

params = {
    'rho0': 1.0,    # Borne initiale de performance
    'rho_inf': 0.03, # Borne finale
    'a': 0.5,       # Taux de convergence
    'k': 0.3,       # Gain de contrôle
    'safety_radius': 0.2,
    'comm_radius': 5.0
}

class Robot:
    def __init__(self, robot_name, robot_id, initial_pos, neighbors, desired_distances, params):
        """
        Initialise un robot avec ses paramètres.
        
        Args:
            robot_name (str): Nom du robot.
            robot_id (int): Identifiant unique du robot.
            initial_pos (list/np.ndarray): Position initiale [x, y].
            neighbors (list): Liste des noms des voisins.
            desired_distances (dict): {voisin_name: distance_désirée}.
            params (dict): Paramètres de contrôle et de performance.
        """
        self.robot_name = robot_name
        self.id = robot_id
        self.position = np.array(initial_pos, dtype=float)
        self.neighbors = neighbors
        self.d_ij = desired_distances

        # Dictionnaire partagé pour stocker les positions des voisins
        self.neighbors_positions = {n: np.array([0.0, 0.0]) for n in neighbors}

        # Paramètres de performance (PPC)
        self.rho_ij0 = params.get('rho0', 1.0)  # Borne initiale
        self.rho_ij_inf = params.get('rho_inf', 0.05)  # Borne finale
        self.a_ij = params.get('a', 0.5)  # Taux de convergence
        
        # Contraintes physiques
        self.r_s = params.get('safety_radius', 0.2)  # Rayon de sécurité (collisions)
        self.r_c = params.get('comm_radius', 5.0)  # Rayon de communication
        
        # Gains de contrôle
        self.k_ij = params.get('k', 0.3)  # Gain proportionnel
        
        # Initialisation des bornes b_ij (Algorithme 1 simplifié)
        self.b_ij = {j: self._compute_b_ij(j) for j in self.neighbors}
    
    def set_pose(self, x, y):
        self.position = np.array([x, y], dtype=float)

    def update_neighbor_position(self, neighbor_name, pos):
        """Met à jour la position d'un voisin."""
        if neighbor_name in self.neighbors_positions:
            self.neighbors_positions[neighbor_name] = np.array(pos, dtype=float)

    def _get_neighbor_pos(self, neighbor_name):
        """Retourne la position courante connue du voisin."""
        return self.neighbors_positions.get(neighbor_name, np.array([0.0, 0.0]))

    def _compute_b_ij(self, neighbor_name):
        """
        Calcule les bornes b_ij et b_ij pour l'erreur (Algorithme 1 de l'article).
        """
        q_j = self._get_neighbor_pos(neighbor_name)
        e_ij_0 = np.linalg.norm(self.position - q_j) - self.d_ij[neighbor_name]
        b_upper = min(abs(e_ij_0) + 0.1, self.r_c - self.d_ij[neighbor_name])  # Borne supérieure
        b_lower = min(abs(e_ij_0) + 0.1, self.d_ij[neighbor_name] - self.r_s)  # Borne inférieure
        return {'b_upper': b_upper, 'b_lower': b_lower}
    

    def _compute_control_input(self, t):
        """
        Calcule la commande de contrôle u_i à l'instant t.
        """
        u_i = np.zeros_like(self.position)
        
        for j in self.neighbors:
            q_j = self._get_neighbor_pos(j)
            q_ij = self.position - q_j  # Vecteur relatif
            distance = np.linalg.norm(q_ij)
            
            # Éviter les divisions par zéro
            if distance < 1e-6:
                continue
            
            # Erreur de distance et η_ij
            e_ij = distance - self.d_ij[j]
            eta_ij = distance**2 - self.d_ij[j]**2
            
            # Fonction de performance ρ_ij(t)
            rho_ij = (self.rho_ij0 - self.rho_ij_inf) * np.exp(-self.a_ij * t) + self.rho_ij_inf
            
            # Erreur normalisée et transformation σ_ij
            eta_hat_ij = eta_ij / (rho_ij + 1e-10)
            b = self.b_ij[j]
            b_upper = b['b_upper']
            b_lower = b['b_lower']
            # Utilisation d'une borne moyenne pour la transformation (simplifié)
            b_val = 0.5 * (b_upper + b_lower)
            # Transformation sigma (évite singularités)
            denom = (b_val * b_val - b_val * eta_hat_ij + 1e-10)
            if denom <= 0:
                continue
            sigma_ij = 0.5 * np.log(
                (b_val * eta_hat_ij + b_val * b_val) / denom
            )
            xi_ij = (1 / (rho_ij + 1e-10)) * (
                1 / (eta_hat_ij + b_val + 1e-10) - 
                1 / (eta_hat_ij - b_val + 1e-10)
            )
            u_i += -self.k_ij * xi_ij * sigma_ij * q_ij
        
        return u_i
    

    def update_position(self, t, dt):
        """
        Met à jour la position du robot en intégrant la commande de contrôle.
        """
        u_i = self._compute_control_input(t)
        self.position += u_i * dt
        self._enforce_safety()  # Appliquer les contraintes
    

    def _enforce_safety(self):
        """
        Vérifie les contraintes de sécurité (collisions/connectivité).
        """
        for j in self.neighbors:
            q_j = self._get_neighbor_pos(j)
            distance = np.linalg.norm(self.position - q_j)
            
            # Évitement de collisions
            if distance < self.r_s:
                direction = (self.position - q_j) / (distance + 1e-10)
                self.position += direction * (self.r_s - distance) * 0.5
            
            # Maintien de la connectivité
            if distance > self.r_c:
                direction = (q_j - self.position) / (distance + 1e-10)
                self.position += direction * (distance - self.r_c) * 0.5