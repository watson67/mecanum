# Packages

Chaque package contient son propre README pour plus d'informations.

## Principaux packages

### `teleop_mecanum`
Ce package contient des nœuds pour la téléopération des robots Mecanum à l'aide d'un clavier ou d'une manette. Il propose plusieurs modes de contrôle, notamment pour un robot unique ou plusieurs robots simultanément.

- **Principaux nœuds :**
  - `teleop_mecanum`: Contrôle d'un robot via un clavier.
  - `teleop_mecanum_all`: Contrôle simultané de plusieurs robots.
  - `teleop_dualshock`: Contrôle d'un robot via une manette DualShock.
  - `tf2_teleop`: Téléopération avec transformations TF2. Les commandes de vitesses sont envoyées depuis le repère global, puis transformées dans le repère body du robot.

### `swarm_manager`
Ce package contient les gestionnaires principaux pour le contrôle d'essaim, incluant la gestion des transformations TF2 et la coordination des robots.

- **Principaux nœuds :**
  - `tf2_manager`: Gestionnaire des transformations TF2 pour tous les robots.
  - `tf2_obstacle_manager`: Gestionnaire des obstacles avec TF2.
  - `tf2_visu`: Visualisation des transformations.
  - `swarm_master`: Contrôleur principal de l'essaim avec interface clavier.
  - `distributed_tf2_manager`: Gestionnaire distribué des transformations.
  - `distributed_manager`: Gestionnaire distribué pour la coordination.
  - `goal_point_sender`: Envoi automatique des points objectifs selon des trajectoires prédéfinies.

- **Fonctionnalités :**
  - Gestion centralisée des transformations TF2.
  - Interface de contrôle maître pour démarrer/arrêter l'essaim.
  - Envoi de trajectoires (rectangle, cercle, triangle, huit).
  - Logging automatique des distances entre robots.

---

### `distributed_swarm`
Ce package implémente des algorithmes de contrôle d'essaim distribués où chaque robot calcule sa propre commande de contrôle.

- **Principaux nœuds :**
  - `distributed_swarm`: Contrôle distribué standard de l'essaim.
  - `distributed_obstacle_swarm`: Contrôle distribué avec évitement d'obstacles.
  - `distributed_event_swarm`: Contrôle distribué basé sur les événements (réduction de la fréquence de calcul).

- **Fonctionnalités :**
  - Chaque robot détermine automatiquement son nom via le hostname.
  - Contrôle de consensus distribué avec maintien de formation.
  - Évitement d'obstacles intégré.
  - Commande événementielle pour optimiser les ressources.
  - Communication inter-robots via topics ROS2.

---

### `centralized_swarm`
Ce package implémente des algorithmes de contrôle d'essaim centralisés où un nœud unique calcule les commandes pour tous les robots.

- **Principaux nœuds :**
  - `swarm`: Contrôle centralisé moderne de l'essaim avec formation individuelle.
  - `old_swarm`: Ancienne version du contrôle centralisé avec formation commune.

- **Fonctionnalités :**
  - Calcul centralisé de toutes les commandes de contrôle.
  - Maintien de formation basé sur les positions initiales.
  - Algorithmes de consensus avec termes PID.
  - Support des relations de voisinage configurables.
  - Transformation automatique des vitesses dans le repère de chaque robot.

---

### `mecanum_swarm`
Ce package contient les premières version du contrôle d'essaim. Il contient tous les nœuds nécessaires à la mise en place et au contrôle de l'essaim.

- **Fonctionnalités :**
  - Contrôle de l'essaim.
  - Enregistrement des données.
  - Algorithmes de coordination.

---

## Packages utilitaires

### `data_visualizer`
Ce package est dédié à la visualisation des données des robots, telles que leurs positions, trajectoires ou états. Il est utile pour le débogage et l'analyse des comportements des robots.

- **Fonctionnalités :**
  - Visualisation des trajectoires.
  - Affichage des données en temps réel.

### `turtle_swarm`
Ce package permet de simuler le système sur turtlesim. Les tortues et topics utilisés auront le même nom que les robots réels, ce qui permet de tester sans modification particulière chaque programme sur turtlesim.

- **Fonctionnalités :**
  - Simulation du système sur turtlesim.
  - Topics utilisés sur turtlesim ont les mêmes noms que ceux pour les turtlebots réels.

**Note :** Si les robots réels sont allumés et que vous souhaitez utiliser turtlesim pour tester un programme en même temps, il est nécessaire de changer le **ROS_DOMAIN_ID** de chaque terminal. Cela permet d'éviter les conflits entre les topics des robots réels et ceux des tortues simulées sur turtlesim.

Pour changer le **ROS_DOMAIN_ID**, utiliser la commande suivante : 
```bash
export ROS_DOMAIN_ID=10
```
Mettre un nombre différent du ROS_DOMAIN_ID des terminaux communiquant avec les robots réels.

---

### `logger`
Ce package est dédié à l'enregistrement et au logging des données des robots pendant leur fonctionnement.

- **Principaux nœuds :**
  - `cmd_vel_rate_logger`: Enregistrement du taux des commandes de vitesse.
  - `barycenter_logger`: Enregistrement des données du barycentre.
  - `distances_logger`: Enregistrement des distances entre robots.
  - `goal_point_logger`: Enregistrement des points objectifs.
  - `event_logger`: Enregistrement des événements système.
---

## Notes
- Tous les packages nécessitent ROS2 (testé avec Humble).
- Pour compiler et sourcer l'espace de travail :
  ```bash
  colcon build
  source install/setup.bash
  ```