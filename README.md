# Mecanum

## Téléchargement du package

L'ensemble des programmes pour utiliser les robots Mecanum se trouvent sur Github. Pour les utiliser, cloner le repository suivant :

```bash
git clone https://github.com/watson/mecanum.git
```

Puis, se placer dans le dossier cloné, et *build* :

```bash
cd mecanum
colcon build
source install/setup.bash
```

Pour ne pas avoir à sourcer le fichier `setup.bash` à chaque ouverture de terminal, utiliser la commande suivante :

```bash
echo "source ~/mecanum/install/setup.bash" >> ~/.bashrc
```

## Installation du package `vrpn_mocap`

La section suivante décrit comment récupérer, sur un topic ROS2, la position des *Rigid Bodies* détectés par le système de Motion Capture Motive.

Commencer par créer un nouvel espace de travail ROS2. Ouvrir un terminal et entrer les commandes suivantes pour créer le répertoire `mocap_ws` ainsi que son sous-répertoire `src` :

```bash
mkdir -p ~/mocap_ws/src
cd ~/mocap_ws
```

Cloner ensuite le dépôt GitHub du package `vrpn_mocap` dans le répertoire `src` de l’espace de travail :

```bash
cd ~/mocap_ws/src
git clone https://github.com/alvinsunyixiao/vrpn_mocap.git
```

Une fois le dépôt cloné, installer les dépendances nécessaires à la compilation en exécutant la commande suivante :

```bash
rosdep install --from-paths src -y --ignore-src
```

Procéder ensuite à la compilation du package :

```bash
colcon build
```

Enfin, sourcer l’espace de travail :

```bash
source install/setup.bash
```

Pour éviter d’avoir à sourcer l’environnement à chaque nouvelle session, ajouter cette ligne au fichier `.bashrc` :

```bash
echo "source ~/mocap_ws/install/setup.bash" >> ~/.bashrc
```

Il est possible de personnaliser le comportement du package en modifiant le fichier `client.launch.yaml`. Les principaux paramètres configurables sont les suivants :

- **`server`** : adresse IP ou nom de domaine du serveur VRPN (par défaut : `localhost`).
- **`port`** : port utilisé par le serveur VRPN (par défaut : `3883`).
- **`frame_id`** : nom de la frame de référence fixe (par défaut : `world`).
- **`update_freq`** : fréquence de publication des données de capture de mouvement (par défaut : `100.0 Hz`).

Pour tester le package, utiliser la commande suivante (modifier l'adresse IP et le port pour correspondre au système de Motion Capture) :

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.103 port:=3883
```

Il est possible de modifier le fichier `client.launch.yaml`, afin de ne pas avoir à spécifier l'IP du serveur et le port dans la commande. Pour cela, modifier le fichier `client.launch.yaml` se trouvant dans `/mocap/src/vrpn_mocap/launch` pour qu'il corresponde à :

```yaml
launch:

- arg:
    name: "server"
    default: "192.168.0.103"
- arg:
    name: "port"
    default: "3883"

- node:
    pkg: "vrpn_mocap"
    namespace: "vrpn_mocap"
    exec: "client_node"
    name: "vrpn_mocap_client_node"
    param:
    -
      from: "$(find-pkg-share vrpn_mocap)/config/client.yaml"
    -
      name: "server"
      value: "$(var server)"
    -
      name: "port"
      value: "$(var port)"
```

La nouvelle commande à utiliser pour lancer le noeud sera alors :

```bash
ros2 launch vrpn_mocap client.launch.yaml
```

## Utilisation du package

### Sous dossier `Matlab_Simulink`

Ce dossier contient différents fichiers Simulink permettant de réaliser un essaim *Leader-Follower*. Ces programmes utilisent les données du système de Motion Capture, il faut donc lancer le noeud suivant au préalable :

```bash
ros2 launch vrpn_mocap client.launch.yaml
```

Pour piloter le robot *Leader*, utiliser un programme mentionné dans la sous-section [teleop_mecanum](#teleop_mecanum).

### Sous dossier `src`

Ce dossier contient l'ensemble des packages ROS2. 

### Sous dossier `holonome_mecanum_ros2`

Ce dossier contient les codes Arduino à téléverser dans l'OpenCR.