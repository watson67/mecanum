# turtle_swarm

Le package `turtle_swarm` permet de simuler et contrôler un essaim de tortues dans turtlesim.

## Programmes Python

- **spawn_3turtles.py**  
  Ce script supprime la tortue par défaut (`turtle1`) et fait apparaître trois tortues nommées `Athos`, `Porthos` et `Aramis` aux sommets d'un triangle équilatéral, avec des orientations aléatoires.  
  Il utilise les services `/spawn` et `/kill` de turtlesim.

- **pose_conversion2.py**  
  Ce script convertit les messages de position (`Pose`) de chaque tortue (`Athos`, `Porthos`, `Aramis`) en messages `PoseStamped` (format ROS standard), publiés sur les topics `/vrpn_mocap/<nom>/pose`, afin de correspondre au système réel.

## Utilisation

Il est recommandé d'utiliser un fichier launch pour démarrer l'ensemble des noeuds nécessaires à la simulation, afin de garantir le bon ordre de lancement et la configuration correcte.

### Lancement via launch (recommandé)

```bash
ros2 launch turtle_swarm swarm_launch.py
```

### Lancement manuel (non recommandé)

Vous pouvez lancer chaque script individuellement :

```bash
ros2 run turtle_swarm spawn_3turtles
ros2 run turtle_swarm pose_conversion2
```

Cependant, cette méthode nécessite de gérer manuellement l'ordre de lancement et les dépendances.

## Conseils

- Utilisez le fichier launch pour éviter les erreurs de configuration.


