#!/usr/bin/env python3

import numpy as np
import math

'''
Ce programme contient des fonctions pour réaliser les calculs du papier suivant :
"Consensus-based formation control and obstacle avoidance for nonholonomic 
multi-robot system"  (Daravuth Koung; Isabelle Fantoni; Olivier Kermorgant; 
Lamia Belouaer )
'''

"""
# Valeurs par défaut
h = 0.2
c1_gamma = 0.25 # navigation gain (position)
c2_gamma = 0.25 # navigation gain (vitesse)
c1_beta = 0.3
c = 5           # interaction range between robots
epsilon = 0.1
a = 1
b = 1
e = abs(a-b)/math.sqrt(4*a*b)
Kp = 0.2
Ki = 0.05  
x_min = 0.3
x_max = 2

"""

# Paramètres du contrôle
h = 0.2
c1_gamma = 0.2 # navigation gain (position)
c2_gamma = 0.05 # navigation gain (vitesse)
c1_beta = 0.3
c = 5         
epsilon = 0.1
a = 1
b = 1
e = abs(a-b)/math.sqrt(4*a*b)

# Coefficients PID
Kp = 0.2
Ki = 0.05
Kd = 0.0

# Saturation des commandes
x_min = -0.04
x_max = 0.04

#rotation gain
c1_rotation = 0.2

# Paramètre de lissage pour ui_gamma
gamma_smoothing_factor = 0.8  # Entre 0 et 1, plus proche de 1 = plus lisse

def sigma_norm(z):
    """
    Formule 4 (norme sigma)

    ||z||_sigma = 1 / epsilon * (sqrt( 1 + epsilon * ||z||²) - 1)

    """
    return 1 / epsilon * ( math.sqrt(1 + epsilon * np.dot(z, z)  ) -1 )

def sigma_epsilon(z):
    """
    Formule 5 (gradient de z)

    z / (1 + epsilon*sigma_norm(epsilon,z))

    """
    return z / (1 + epsilon * sigma_norm(z))

def rho_h(s):
    """
    Formule 7 (Fonction rho)

    Pour s dans [0, h], retourne 1.
    Pour s dans [h, 1], retourne 1/2 * (1 + cos(pi * (s - h) / (1 - h))).
    Sinon, retourne 0.
    """

    if 0 <= s <= h:
        return 1
    elif h < s <= 1:
        return 0.5 * (1 + math.cos(math.pi * (s - h) / (1 - h)))
    else:
        return 0
    
def sigma_1(s):
    return s / math.sqrt( 1 + s**2)

def phi_alpha(s,d):
    """
    d : desired distance
    """

    return 1/2 * rho_h( s / sigma_norm(c) ) * ((a+b) * sigma_1(s - sigma_norm(d) + e) + ( a - b ))
    
def nij(pj,pi):
    return sigma_epsilon(pj-pi)

def n_ik(p_ik,pi):
    return sigma_epsilon(p_ik-pi)

def b_ik(p_ik,pi,d_bet):
    """
    Formule 12 
    d_prime : distance entre le robot et l'obstacle

    """
    return rho_h(sigma_norm(p_ik - pi) / d_bet)

def d_beta(d):
    return sigma_norm(d)

def phi_beta(s,d_bet):
    """
    Formule 13 (Fonction phi_beta)
    """
    return rho_h(s/d_bet) * ( sigma_1(s - d_bet ) -1)

def calculate_angle_to_x_axis(vector):
    """
    Calcule l'angle entre un vecteur et l'axe X du repère mocap.
    
    Parameters:
    - vector: vecteur [x, y]
    
    Returns:
    - angle en radians entre -π et π
    """
    return math.atan2(vector[1], vector[0])

def calculate_dynamic_c1_gamma(pj_array, pi, dij_list, is_rotating=False, initial_angles=None, rotation_count=0):
    """
    Calcule c1_gamma dynamiquement en fonction de la proximité des robots voisins
    par rapport aux distances désirées ET des angles de formation.
    
    Parameters:
    - pj_array: positions des robots voisins
    - pi: position du robot i
    - dij_list: distances désirées aux voisins
    - is_rotating: True si une rotation est en cours
    - initial_angles: dictionnaire des angles initiaux {index_voisin: angle_initial}
    - rotation_count: nombre de rotations de 90° effectuées
    
    Returns:
    - c1_gamma ajusté dynamiquement
    """
    base_c1_gamma = c1_rotation if is_rotating else c1_gamma
    
    if not pj_array or not dij_list:
        return base_c1_gamma
    
    # RÉDUCTION BASÉE SUR LES DISTANCES (existante)
    min_ratio = float('inf')
    max_ratio = 0.0
    
    for idx, pj in enumerate(pj_array):
        if idx < len(dij_list):
            distance_actuelle = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
            distance_desiree = dij_list[idx]
            
            if distance_desiree > 0:
                ratio = distance_actuelle / distance_desiree
                min_ratio = min(min_ratio, ratio)
                max_ratio = max(max_ratio, ratio)
    
    # Seuils basés sur les ratios des distances désirées
    if is_rotating:
        min_ratio_threshold = 0.95
        max_ratio_threshold = 1.05
        k_min = 1
        k_max = 1
        additional_reduction = 0.1
    else:
        min_ratio_threshold = 0.9
        max_ratio_threshold = 1.1
        k_min = 0.7
        k_max = 0.5
        additional_reduction = 0.4
    
    distance_reduction_factor = 1.0
    
    # Réduction basée sur les distances
    if min_ratio < min_ratio_threshold:
        deviation = (min_ratio_threshold - min_ratio) / min_ratio_threshold
        proximity_reduction = math.exp(-k_min * deviation)
        distance_reduction_factor *= proximity_reduction
    
    if max_ratio > max_ratio_threshold:
        deviation = min((max_ratio - max_ratio_threshold) / max_ratio_threshold, 1.0)
        separation_reduction = math.exp(-k_max * deviation)
        distance_reduction_factor *= separation_reduction
    
    if min_ratio < min_ratio_threshold or max_ratio > max_ratio_threshold:
        distance_reduction_factor *= additional_reduction
    
    # RÉDUCTION BASÉE SUR LES ANGLES (même logique que les distances)
    angle_reduction_factor = 1.0
    
    if initial_angles and pj_array:
        # Calculer les ratios d'angles (erreur normalisée)
        angle_ratios = []
        
        for idx, pj in enumerate(pj_array):
            if idx in initial_angles:
                # Vecteur actuel vers le voisin
                current_vector = pj - pi
                if np.linalg.norm(current_vector) > 0.01:
                    # Angle actuel
                    current_angle = calculate_angle_to_x_axis(current_vector)
                    
                    # Angle cible (ajusté selon les rotations)
                    target_angle = initial_angles[idx] + (rotation_count * math.pi / 2)
                    target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))
                    
                    # Calculer l'erreur angulaire normalisée (0 = parfait, 1 = erreur maximale de π)
                    angle_error = abs(current_angle - target_angle)
                    angle_error = min(angle_error, 2 * math.pi - angle_error)
                    angle_ratio = angle_error / math.pi  # Normaliser entre 0 et 1
                    angle_ratios.append(angle_ratio)
        
        if angle_ratios:
            min_angle_ratio = min(angle_ratios)  # Meilleur angle (plus proche de 0)
            max_angle_ratio = max(angle_ratios)  # Pire angle (plus proche de 1)
            
            # Même logique que pour les distances
            if is_rotating:
                # En rotation: plus strict sur les angles
                min_angle_threshold = 0.05  # 5% d'erreur max (≈ 9°)
                max_angle_threshold = 0.15  # 15% d'erreur max (≈ 27°)
                k_angle_min = 1
                k_angle_max = 1
                angle_additional_reduction = 0.1
            else:
                # Mode normal: plus tolérant
                min_angle_threshold = 0.095  # 8% d'erreur max (≈ 14°)
                max_angle_threshold = 0.098  # 20% d'erreur max (≈ 36°)
                k_angle_min = 0.6
                k_angle_max = 0.4
                angle_additional_reduction = 0.4
            
            # Réduction si erreur angulaire trop importante
            if max_angle_ratio > max_angle_threshold:
                deviation = min((max_angle_ratio - max_angle_threshold) / (1.0 - max_angle_threshold), 1.0)
                angle_error_reduction = math.exp(-k_angle_max * deviation)
                angle_reduction_factor *= angle_error_reduction
            
            # Réduction supplémentaire si les angles dépassent les seuils
            if max_angle_ratio > max_angle_threshold:
                angle_reduction_factor *= angle_additional_reduction
    
    # Combiner les deux réductions (prendre le minimum = plus restrictif)
    combined_reduction_factor = min(distance_reduction_factor, angle_reduction_factor)
    
    adjusted_c1_gamma = base_c1_gamma * combined_reduction_factor
    return adjusted_c1_gamma

def smooth_gamma_transition(current_gamma, previous_gamma, smoothing_factor=gamma_smoothing_factor):
    """
    Applique un lissage exponentiel à ui_gamma pour éviter les transitions brusques.
    
    Parameters:
    - current_gamma: nouveau ui_gamma calculé
    - previous_gamma: ui_gamma précédent
    - smoothing_factor: facteur de lissage (0 = pas de lissage, 1 = pas de changement)
    
    Returns:
    - ui_gamma lissé
    """
    if previous_gamma is None:
        return current_gamma
    
    # Lissage exponentiel: nouveau = alpha * ancien + (1-alpha) * nouveau
    smoothed_gamma = smoothing_factor * previous_gamma + (1 - smoothing_factor) * current_gamma
    return smoothed_gamma

def control(pj_array=None, pi=None, dij_list=None, pr=None, dt=0.1, integral_term=None, is_rotating=False, logger=None, previous_gamma=None):
    """
    Formule 18 , sans ui_beta
    
    Parameters:
    - pj_array: positions des robots voisins
    - pi: position du robot i
    - dij_list: distances désirées aux voisins
    - pr: position de référence (point but ou centre de l'essaim)
    - dt: pas de temps pour l'intégration
    - integral_term: valeur accumulée de l'intégrale (None pour initialiser)
    - is_rotating: True si une rotation est en cours, False sinon
    - logger: logger ROS2 pour afficher les messages
    - previous_gamma: ui_gamma précédent pour le lissage
    
    Returns:
    - Le vecteur de contrôle
    - La nouvelle valeur de l'intégrale pour l'appel suivant
    - ui_gamma lissé pour la prochaine itération
    """
    ui_alpha = np.zeros(2)
    ui_gamma = np.zeros(2)
    
    # Adapter c1_gamma dynamiquement selon les distances désirées
    current_c1_gamma = calculate_dynamic_c1_gamma(pj_array, pi, dij_list, is_rotating)
    
    # Initialiser l'intégrale si elle n'existe pas
    if integral_term is None:
        integral_term = np.zeros((len(pj_array), 2))
    
    # Pour tous les voisins j de i
    for idx in range(len(pj_array)):
        pj = pj_array[idx]
        dij = dij_list[idx]
        
        current_term = phi_alpha(sigma_norm(pj-pi), dij) * nij(pj, pi)
        
        # Mettre à jour l'intégrale 
        integral_term[idx] += current_term * dt
        
        # Appliquer l'intégrale au contrôle
        ui_alpha += Kp * phi_alpha(sigma_norm(pj-pi),dij) * nij(pj,pi) + Ki * integral_term[idx]

    ui_gamma_raw = -((current_c1_gamma * (pi - pr)))
    
    # Appliquer le lissage à ui_gamma
    ui_gamma = smooth_gamma_transition(ui_gamma_raw, previous_gamma)
    
    if logger:
        min_ratio = float('inf')
        max_ratio = 0.0
        if pj_array and dij_list:
            for idx, pj in enumerate(pj_array):
                if idx < len(dij_list):
                    dist_actuelle = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                    dist_desiree = dij_list[idx]
                    if dist_desiree > 0:
                        ratio = dist_actuelle / dist_desiree
                        min_ratio = min(min_ratio, ratio)
                        max_ratio = max(max_ratio, ratio)
        mode_str = "ROTATION" if is_rotating else "NORMAL"
        logger.info(f"[{mode_str}] c1_gamma: {current_c1_gamma:.4f} (min_ratio: {min_ratio:.2f}, max_ratio: {max_ratio:.2f})")
        logger.info(f"ui_alpha: {ui_alpha}, ui_gamma (smooth): {ui_gamma}")

    return (ui_alpha) + ui_gamma, integral_term, ui_gamma

def control_with_components(pj_array=None, pi=None, dij_list=None, pr=None, dt=0.1, integral_term=None, derivative_term=None, is_rotating=False, logger=None, previous_gamma=None, initial_angles=None, rotation_count=0):
    """
    Version de control() qui retourne aussi les composantes ui_alpha et ui_gamma séparément
    Avec action dérivée (Kd) sur ui_alpha et lissage de ui_gamma
    
    Parameters:
    - derivative_term: dictionnaire contenant les erreurs précédentes pour chaque voisin
    - is_rotating: True si une rotation est en cours
    - previous_gamma: ui_gamma précédent pour le lissage (peut être None)
    - initial_angles: dictionnaire des angles initiaux {index_voisin: angle_initial}
    - rotation_count: nombre de rotations de 90° effectuées
    
    Returns:
    - Le vecteur de contrôle total
    - La nouvelle valeur de l'intégrale pour l'appel suivant
    - La nouvelle valeur du terme dérivé pour l'appel suivant
    - ui_alpha (numpy array)
    - ui_gamma lissé (numpy array)
    - ui_gamma lissé pour la prochaine itération
    """
    ui_alpha = np.zeros(2)
    ui_gamma = np.zeros(2)
    
    # Adapter c1_gamma dynamiquement selon les distances désirées ET les angles
    current_c1_gamma = calculate_dynamic_c1_gamma(pj_array, pi, dij_list, is_rotating, initial_angles, rotation_count)
    
    # Initialiser l'intégrale si elle n'existe pas
    if integral_term is None:
        integral_term = np.zeros((len(pj_array), 2))
    
    # Initialiser le terme dérivé si il n'existe pas
    if derivative_term is None:
        derivative_term = {'previous_errors': np.zeros((len(pj_array), 2))}
    
    # Stocker les erreurs actuelles pour le calcul de la dérivée
    current_errors = np.zeros((len(pj_array), 2))
    
    # Pour tous les voisins j de i
    for idx in range(len(pj_array)):
        pj = pj_array[idx]
        dij = dij_list[idx]
        
        # Calcul de l'erreur actuelle
        error_vector = phi_alpha(sigma_norm(pj-pi), dij) * nij(pj, pi)
        current_errors[idx] = error_vector
        
        # Terme proportionnel
        proportional_term = Kp * error_vector
        
        # Terme intégral
        integral_term[idx] += error_vector * dt
        integral_action = Ki * integral_term[idx]
        
        # Terme dérivé
        if 'previous_errors' in derivative_term and idx < len(derivative_term['previous_errors']):
            derivative_action = Kd * (error_vector - derivative_term['previous_errors'][idx]) / dt
        else:
            derivative_action = np.zeros(2)  # Pas de dérivée à la première itération
        
        # Somme PID sur ui_alpha
        ui_alpha += proportional_term + integral_action + derivative_action

    # Mettre à jour les erreurs précédentes pour la prochaine itération
    derivative_term['previous_errors'] = current_errors.copy()

    ui_gamma_raw = -((current_c1_gamma * (pi - pr)))
    
    # Appliquer le lissage à ui_gamma si previous_gamma est fourni
    ui_gamma = smooth_gamma_transition(ui_gamma_raw, previous_gamma)
    
    if logger:
        min_ratio = float('inf')
        max_ratio = 0.0
        min_angle_ratio = float('inf')
        max_angle_ratio = 0.0
        
        if pj_array and dij_list:
            for idx, pj in enumerate(pj_array):
                if idx < len(dij_list):
                    dist_actuelle = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                    dist_desiree = dij_list[idx]
                    if dist_desiree > 0:
                        ratio = dist_actuelle / dist_desiree
                        min_ratio = min(min_ratio, ratio)
                        max_ratio = max(max_ratio, ratio)
                
                # Calculer les ratios d'angles pour les logs
                if initial_angles and idx in initial_angles:
                    current_vector = pj - pi
                    if np.linalg.norm(current_vector) > 0.01:
                        current_angle = calculate_angle_to_x_axis(current_vector)
                        target_angle = initial_angles[idx] + (rotation_count * math.pi / 2)
                        target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))
                        angle_error = abs(current_angle - target_angle)
                        angle_error = min(angle_error, 2 * math.pi - angle_error)
                        angle_ratio = angle_error / math.pi
                        min_angle_ratio = min(min_angle_ratio, angle_ratio)
                        max_angle_ratio = max(max_angle_ratio, angle_ratio)
        
        # Affichage unifié des ratios
        if min_angle_ratio == float('inf'):
            min_angle_ratio = 0.0
            max_angle_ratio = 0.0
        
        mode_str = "ROTATION" if is_rotating else "NORMAL"
        logger.info(f"[{mode_str}] c1_gamma: {current_c1_gamma:.4f} (dist_ratio: {min_ratio:.2f}-{max_ratio:.2f}, angle_ratio: {min_angle_ratio:.2f}-{max_angle_ratio:.2f})")
        logger.info(f"ui_alpha: {ui_alpha}, ui_gamma (smooth): {ui_gamma}")

    ui = 1 * (ui_alpha) + ui_gamma  
    
    return ui, integral_term, derivative_term, ui_alpha, ui_gamma, ui_gamma

def control_obstacle(pj_array=None, pi=None, dij_list=None,
                     pk_array=None, pi_array=None, d_bet=None, 
                     pr=None, dt=0.1, integral_term=None, logger=None):
    """
    Formule 18 avec ui_beta
    
    Parameters:
    - pj_array: positions des robots voisins
    - pi: position du robot i
    - dij_list: distances désirées aux voisins
    - pr: position de référence (point but ou centre de l'essaim)
    - dt: pas de temps pour l'intégration
    - integral_term: valeur accumulée de l'intégrale (None pour initialiser)
    - logger: logger ROS2 pour afficher les messages
    
    Returns:
    - Le vecteur de contrôle
    - La nouvelle valeur de l'intégrale pour l'appel suivant
    """
    ui_alpha = np.zeros(2)
    ui_beta = np.zeros(2)
    ui_gamma = np.zeros(2)
    
    # Initialiser l'intégrale si elle n'existe pas
    if integral_term is None:
        integral_term = np.zeros((len(pj_array), 2))
    
    # Pour tous les voisins j de i
    for idx in range(len(pj_array)):
        pj = pj_array[idx]
        dij = dij_list[idx]
        
        # Calculer le terme à intégrer
        current_term = phi_alpha(sigma_norm(pj-pi), dij) * nij(pj, pi)
        
        # Mettre à jour l'intégrale (méthode d'Euler)
        integral_term[idx] += current_term * dt
        
        # Appliquer l'intégrale au contrôle
        ui_alpha += Kp * phi_alpha(sigma_norm(pj-pi),dij) * nij(pj,pi) + Ki * integral_term[idx]

    # Pour tous les obstacles k de i
    for idx in range(len(pk_array)):
        pk = pk_array[idx]
        d_bet = d_bet[idx]
        
        # Appliquer l'intégrale au contrôle
        ui_beta += phi_beta(sigma_norm(pk-pi),d_bet) * n_ik(pk,pi)
    ui_beta *= c1_beta
    ui_gamma = -(c1_gamma * (pi - pr))
    if logger:
        
        logger.info(f"sigma_norm: {phi_alpha(sigma_norm(pj-pi),dij)}")
        logger.info(f"d_bet: {d_bet} ; pk: {pk}")
        logger.info(f"ui_alpha: {ui_alpha}, ui_beta: {ui_beta}, ui_gamma: {ui_gamma}")
    return ui_alpha + ui_beta + ui_gamma, integral_term


def sat(x):
    """
    Formule 19 (saturation)
    Saturation de x entre x_min et x_max.
    """
    for i in range(len(x)):
        if x[i] < x_min:
            x[i] = x_min
        elif x[i] > x_max:
            x[i] = x_max
        else:
            x[i] = x[i]
    return x
