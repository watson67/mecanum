## mecanum_swarm

Permet de mettre en place un essaim de robot, selon la méthode décrite dans le document : "Consensus-based formation control and obstacle avoidance for nonholonomic 
multi-robot system"  (Daravuth Koung; Isabelle Fantoni; Olivier Kermorgant; 
Lamia Belouaer )

### Description des principaux programmes

#### Programme de contrôle d'essaim

- `swarm_master.py` : Interface clavier pour démarrer/arrêter le contrôle de l'essaim et envoyer des commandes globales. Ce programme doit être utilisé pour lancer tous les autres ; il faut l'exécuter dans un terminal différent de celui du fichier launch lancé. Ce programme permet de publier sur un topic `/master`, auquel tous les autres noeuds du package souscrivent. De cette manière, il est possible de lancer, ou non les différents noeuds.

- `swarm.py` : Contrôleur centralisé de l'essaim. Calcule et publie les commandes de vitesse pour chaque robot afin de maintenir la formation et suivre un point cible. Nécessite de recevoir un "1" sur le topic /master pour s'exécuter (voir fichier `swarm_master.py`).

- `distributed_swarm.py` : Contrôleur distribué à lancer indépendamment sur chaque robot. Chaque robot calcule alors sa propre commande en fonction de ses voisins et du point cible. Il faut s'assurer que le hostname Ubuntu de l'appareil sur lequel tourne le programme est bien sous la forme {nomdurobot}-desktop, et que le nom est bien inclus dans le fichier `robots.yaml` dans config. Nécessite de recevoir un "1" sur le topic /master pour s'exécuter (voir fichier `swarm_master.py`). L'algorithme est identique à `swarm.py`. 

- `distributed_event_swarm.py` : Variante de `distributed_swarm.py` avec contrôle événementiel. Les commandes ne sont recalculées que lors d'événements.

- `formation_init_spat.py` : Permet de déterminer les voisins de chaque robots. Les résultats obtenues se trouvent dans `config/robots.yaml`. C'est également dans ce fichier que l'ensemble des robots sont déclarés.    

```bash
ros2 run mecanum_swarm formation_init_spat --ros-args -p enable_plotting:=true
```

- `formules.py` : Contient les fonctions mathématiques et de contrôle, selon le document "Consensus-based formation control and obstacle avoidance for nonholonomic multi-robot system" (Daravuth Koung; Isabelle Fantoni; Olivier Kermorgant; Lamia Belouaer)

#### Gestion des positions

- `tf2_manager.py` : Transforme les positions reçues via motion capture (vrpn, topics /vrpn_mocap/{nomdurobot}) au format TF2, et calcule le barycentre de l'essaim.

- `tf2_visu.py` : Affiche périodiquement la position de chaque robot dans le repère global à partir des TF2.

#### Gestion de trajectoire

- `goal_point_sender.py` : Permet d'envoyer des points cibles à l'essaim selon une trajectoire prédéfinie dans le fichier `trajectory.py`. Le fonctionnement est le suivant : on publie un premier point cible sur le topic /goal_point, puis lorsque l'essaim est arrivé à destination, un "1" est publié sur /target_reached (voir noeud `distributed_manager.py` ou `swarm.py`), on publie alors le point cible suivant.

- `distributed_manager.py` : Dans le cadre d'essaim distribué, ce programme permet de s'assurer que tous les robots ont atteint leur objectif, et publie un "1" sur le topic /target_reached cela fait.

Les trois programmes ci-dessous sont d'anciennes versions de `goal_point_sender.py`

- `circle.py` : Génère une trajectoire circulaire pour le point cible de l'essaim.

- `rectangle.py` : Génère une trajectoire rectangulaire pour le point cible de l'essaim.

- `eight.py` : Génère une trajectoire en forme de "8" pour le point cible de l'essaim.

#### Logger

Ces programmes permettent d'enregistrer des données au format csv. L'enregistrement commence lorsqu'un "1" est reçu sur le topic /master, et s'arrête lorsqu'un "0" est reçu.
- `distances_logger.py` : Enregistre les distances inter-robots.
- `goal_point_logger.py` : Enregistre les points cibles envoyés à l'essaim.
- `cmd_vel_rate_logger.py` : Enregistre la fréquence de publication des commandes de vitesse.
- `barycenter_logger.py` : Enregistre la position du barycentre de l'essaim.


#### Autres

- `config.py` : Charge la configuration des robots et des voisins à partir d'un fichier YAML.


### Lancement

Des fichiers de lancement (`launch/*.py`) sont fournis pour démarrer les différents scénarios (centralisé, distribué, événementiel, etc.) avec tous les noeuds nécessaires.


#### Fichiers de lancement principaux

- `distributed_swarm.py` : Lance le mode distribué classique (tf2_manager, distributed_manager, goal_point_sender, loggers).    
Chaque robot doit executer le noeud `distributed_swarm.py` (ou `distributed_event_swarm.py`) en local.
Les noms fichiers csv des loggers peuvent être modifiés au sein du fichier launch. L'argument (ici "classic"), correspond au dossier dans lequel seront enregistrés les csv. Cela permet de différencier les csv obtenus par commande classique de ceux obtenus par commande évennementielle.

Pour lancer ce launch file :
```bash
ros2 launch mecanum_swarm distributed_swarm.py
```

- `distributed_event_swarm.py` : Lance le mode distribué événementiel (tf2_manager, distributed_manager, goal_point_sender, loggers), de la même manière que `distributed_swarm.py`.  

Pour lancer ce launch file :
```bash
ros2 launch mecanum_swarm distributed_event_swarm.py
```

`Note :` Pour que ces programmes fonctionnent, il faut lancer le noeud de motion capture afin d'obtenir les topics /vrpn_mocap/{nomdurobot}, ainsi que le noeud `swarm_master.py` dans un autre terminal.

#### Anciens programmes 

Ces programmes correspondent aux premiers essais et ne sont plus à jour :
- `swarm_v1.py` : Lance le mode centralisé classique (tf2_manager, swarm, rviz).
- `swarm_square.py` : Lance le mode centralisé avec une trajectoire rectangulaire et les loggers.
- `swarm_circle.py` : Lance le mode centralisé avec une trajectoire circulaire.
- `swarm_eight.py` : Lance le mode centralisé avec une trajectoire en huit.
- `tf2_visualisation.py` : Lance uniquement la visualisation des TF2 (tf2_manager, tf2_visu, rviz).