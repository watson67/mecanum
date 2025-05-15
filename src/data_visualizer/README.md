# turtle_swarm

Le package `data_visualizer` permet de visualiser les données des topics ROS2 via une interface tkinter.

## Programmes Python

- **app.py**  
  Permet de tracer les trajectoires des robots

## Utilisation

Il est recommandé d'utiliser un fichier launch pour démarrer l'ensemble des noeuds nécessaires à la simulation, afin de garantir le bon ordre de lancement et la configuration correcte.

### Lancement 

```bash
ros2 run data_visualizer visualizer
```