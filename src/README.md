## Packages

### 1. `teleop_mecanum`
Ce package contient des nœuds pour la téléopération des robots Mecanum à l'aide d'un clavier ou d'une manette. Il propose plusieurs modes de contrôle, notamment pour un robot unique ou plusieurs robots simultanément.

- **Principaux nœuds :**
  - `teleop_mecanum`: Contrôle d'un robot via un clavier.
  - `teleop_mecanum_all`: Contrôle simultané de plusieurs robots.
  - `teleop_dualshock`: Contrôle d'un robot via une manette DualShock.
  - `tf2_teleop`: Téléopération avec transformations TF2.

---

### 2. `leader_follower`
Ce package implémente des algorithmes de type *leader-follower*, où un ou plusieurs robots suivent un leader en maintenant une distance ou une formation spécifique.

- **Principaux nœuds :**
  - `leader_follower_v1`: Un robot suit un leader.


---

### 3. `formation_initialize`
Ce package est utilisé pour initialiser et gérer des formations.

- **Principaux nœuds :**
  - `formation_initializer`: Initialise une formation en triangle.
  - `trajectory_visualizer`: Visualise les trajectoires des robots.

---

### 4. `consensus`
Ce package contient des algorithmes basés sur la loi de consensus pour coordonner les mouvements d'un essaim de robots. 

- **Statut :** Work in progress.

---

### 5. `data_visualizer`
Ce package est dédié à la visualisation des données des robots, telles que leurs positions, trajectoires ou états. Il est utile pour le débogage et l'analyse des comportements des robots.

- **Fonctionnalités :**
  - Visualisation des trajectoires.
  - Affichage des données en temps réel.

---
### 6. `turtle_swarm`
Ce package permet de simuler sur turtlesim le système. Les tortues et topics utilisés auront le même nom que les robots réels, ce qui permet de tester sans modification particulière chaque programme sur turtlesim.

- **Fonctionnalités :**
  - Simulation du système sur turtlesim
  - Topics utilisés sur turtlesim ont les mêmes noms que ceux pour les turtlebots réels

**Note :** Si les robots réels sont allumés et que vous souhaitez utiliser turtlesim pour tester un programme en même temps, il est nécessaire de changer le **ROS_DOMAIN_ID** de chaque terminal. Cela permet d'éviter les conflits entre les topics des robots réels et ceux des tortues simulées sur turtlesim, car ROS2 utilise le **ROS_DOMAIN_ID** pour isoler les communications entre différents systèmes. En attribuant un **ROS_DOMAIN_ID** différent à chaque environnement (robots réels et turtlesim), vous vous assurez que les messages publiés et souscrits ne se mélangent pas.

Pour changer le **ROS_DOMAIN_ID**, utiliser la commande suivante : 
  ```bash
  export ROS_DOMAIN_ID=10
  ```
Mettre un nombre différent du ROS_DOMAIN_ID des terminaux communiquant avec les robots réels

---
## Notes
- Tous les packages nécessitent ROS2 (testé avec Humble).
- Certains packages, comme `leader_follower` et `consensus`, nécessitent un système de Motion Capture pour fonctionner correctement.
- Pour compiler et sourcer l'espace de travail :
  ```bash
  colcon build
  source install/setup.bash