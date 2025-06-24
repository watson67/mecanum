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
c1_gamma = 0.15 # navigation gain (position)
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

def control(pj_array=None, pi=None, dij_list=None, pr=None, dt=0.1, integral_term=None, logger=None):
    """
    Formule 18 , sans ui_beta
    
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
    ui_gamma = np.zeros(2)
    
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
        #if abs(sigma_norm(pj-pi)) < 1e-1:
        #    ui_alpha = np.zeros(2)
        #else:
        #    ui_alpha += Kp * phi_alpha(sigma_norm(pj-pi),dij) * nij(pj,pi) + Ki * integral_term[idx]
        ui_alpha += Kp * phi_alpha(sigma_norm(pj-pi),dij) * nij(pj,pi) + Ki * integral_term[idx]

    #ui_gamma = -(c1_gamma * (pi - pr))
    ui_gamma = -((c1_gamma * (pi - pr)))

    if logger:
        logger.info(f"sigma_norm: {(sigma_norm(pj-pi))}")
        logger.info(f"ui_gamma_sans-saturation: {c1_gamma * (pi - pr)}")
        logger.info(f"ui_alpha: {ui_alpha}, ui_gamma: {ui_gamma}")
    

    return (ui_alpha) + ui_gamma, integral_term

def control_with_components(pj_array=None, pi=None, dij_list=None, pr=None, dt=0.1, integral_term=None, derivative_term=None, logger=None):
    """
    Version de control() qui retourne aussi les composantes ui_alpha et ui_gamma séparément
    Avec action dérivée (Kd) sur ui_alpha
    
    Parameters:
    - derivative_term: dictionnaire contenant les erreurs précédentes pour chaque voisin
    
    Returns:
    - Le vecteur de contrôle total
    - La nouvelle valeur de l'intégrale pour l'appel suivant
    - La nouvelle valeur du terme dérivé pour l'appel suivant
    - ui_alpha (numpy array)
    - ui_gamma (numpy array)
    """
    ui_alpha = np.zeros(2)
    ui_gamma = np.zeros(2)
    
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

    ui_gamma = -((c1_gamma * (pi - pr)))

    if logger:
        logger.info(f"ui_alpha (PID): {ui_alpha}, ui_gamma: {ui_gamma}")

    ui = 1 * (ui_alpha) + 1 * ui_gamma
    
    return  ui, integral_term, derivative_term, ui_alpha, ui_gamma

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
