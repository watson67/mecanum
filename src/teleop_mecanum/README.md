# Téléopération pour Robots à Roues Mécanum

Ce package ROS2 contient plusieurs nœuds pour contrôler des robots à roues mécanum dans différentes configurations.

## Programmes 

### 1. teleop_mecanum

Contrôle basique d'un seul robot à roues mécanum avec un clavier AZERTY.

**Commande pour lancer :**
```bash
ros2 run teleop_mecanum teleop_mecanum [topic]
```
*Où `[topic]` est optionnel et par défaut `/cmd_vel`*

Possibilité de choisir le topic sur lequel le programme va publier. Par exemple, pour le topic /Aramis/cmd_vel, utiliser la commande : 
```bash
ros2 run teleop_mecanum teleop_mecanum Aramis/cmd_vel
```

### 2. teleop_mecanum_all

Contrôle simultané de tous les robots (Aramis, Athos, Porthos) avec un seul clavier AZERTY.

**Commande pour lancer :**
```bash
ros2 run teleop_mecanum teleop_mecanum_all
```

### 3. tf2_teleop

Téléopération d'un robot dans le repère global. Les commandes sont transformées du repère global vers le repère du robot en utilisant TF2 basé sur les données VRPN.

**Commande pour lancer :**
```bash
ros2 run teleop_mecanum tf2_teleop [robot_name]
```
*Où `[robot_name]` est l'un des robots : "Aramis", "Athos", "Porthos"*

### 4. tf2_teleop_all

Téléopération de tous les robots dans le repère global. Utilise des threads séparés pour chaque robot.

**Commande pour lancer :**
```bash
ros2 run teleop_mecanum tf2_teleop_all
```

### 6. teleop_dualshock

Contrôle d'un robot en utilisant une manette DualShock 4.

**Commande pour lancer :**
```bash
ros2 run teleop_mecanum teleop_dualshock [topic]
```
*Où `[topic]` est optionnel et par défaut `/cmd_vel`*

Note : programme soumis à des problèmes de délais

## Mappage des touches (clavier AZERTY)

| **Touche** | **Action**                |
|------------|---------------------------|
| `z`        | Avancer                   |
| `s`        | Reculer                   |
| `q`        | Translation gauche        |
| `d`        | Translation droite        |
| `a`        | Diagonale avant-gauche    |
| `e`        | Diagonale avant-droite    |
| `w`        | Diagonale arrière-gauche  |
| `x`        | Diagonale arrière-droite  |
| `r`        | Rotation gauche           |
| `t`        | Rotation droite           |
| `f`        | Virage arrière-droit      |
| `g`        | Virage arrière-gauche     |
| `c`        | Virage avant-gauche       |
| `v`        | Virage avant-droit        |
| Autre      | Stop                      |

## Prérequis

- ROS2 (testé avec Humble)
- Package VRPN client configuré pour les robots nommés "Aramis", "Athos", "Porthos"
- Pour teleop_dualshock : package Python `inputs` installé (`pip install inputs`)

## Notes

- Les programmes tf2_teleop et tf2_teleop_all nécessitent que les transformations TF2 soient publiées. Si TF2 échoue, un système de fallback manuel basé sur les données VRPN sera utilisé.

