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

## Notes
- Tous les packages nécessitent ROS2 (testé avec Humble).
- Certains packages, comme `leader_follower` et `consensus`, nécessitent un système de Motion Capture pour fonctionner correctement.
- Pour compiler et sourcer l'espace de travail :
  ```bash
  colcon build
  source install/setup.bash