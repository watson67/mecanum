# Initialisation de l'IDE

Pour programmer la carte OpenCR, l'IDE Arduino sera utilisé.  
Il faut donc le télécharger ici :  
https://www.arduino.cc/en/software/

Une fois l'IDE installé et ouvert, allez dans **Fichier > Préférences**, et ajoutez le lien suivant dans la case *URL de gestionnaire de cartes supplémentaires* :

https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json

Plus de détails sont donnés ici :  
https://emanual.robotis.com/docs/en/parts/controller/opencr10/

# Programmation de la carte

Ouvrir le le programme `holonome_mecanum_ros2.ino` dans l'IDE Arduino puis téléverser le programme. 

**Note : ** `holonome_mecanum_ros2.ino` doit se trouver dans un dossier contenant également : 
- `turtlebot3_mecanum_motor_driver.cpp`
- `turtlebot3_mecanum_motor_driver.h`
- `turtlebot3_mecanum.h`