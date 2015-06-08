# Installation #

## Installation des dépendances ##
```
#!sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full ros-indigo-freenect-stack ros-indigo-ardrone-autonomy libncursesw5-dev
```
## Utilisation du dépôt ##

### Création d’un espace de travail catkin ###
```
#!sh
source /opt/ros/indigo/setup.bash
mkdir -p ~/hand_control_ws/src
cd ~/hand_control_ws/src
catkin_init_workspace
```
### Clonage du dépôt ###

Clonage de telle sorte que le dossier `hand_control` se situe dans le dossier `~/hand_control_ws/src/`, par exemple :

```
#!sh
cd ~/hand_control_ws/src
git clone git@bitbucket.org:_Luc_/hand_control.git
# ou bien : git clone https://username@bitbucket.org/_Luc_/hand_control.git # (changer username)
```
Le contenu du dépôt se trouve alors dans `~/hand_control_ws/src`.

## Compilation ##

Il est ensuite possible de compiler :

```
#!sh
cd ~/hand_control_ws
catkin_make
```

Puis pour pouvoir utiliser les commoandes ROS : 
```
#!sh
source /opt/ros/indigo/setup.bash
source ~/hand_control_ws/devel/setup.bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo "source ~/hand_control_ws/devel/setup.bash" >> ~/.bashrc
```

# Utilisation #

## Branchement de la Kinect et paramétrage ##

1. Brancher la Kinect (sous tension) à l’ordinateur par USB ;
2. Lancer le "launchfile" `kinect_commander.launch`: `roslaunch hand_control kinect_commander.launch` ;
3. Vérifier les paramètres du filtre :
    - lancer rviz : `rosrun rqt_rviz rqt_rviz`;
    - visualiser la sortie du filtrage (topic : `/filter/output` ; frame : `/camera_depth_optical_frame`) et repérer la main ;
    - lancer `rqt_reconfigure` : `rosrun rqt_reconfigure rqt_reconfigure` pour :
      - modifier les paramètres du filtre jusqu’à ne voir que les points de la main/gant/pancarte sur rviz (voir ci-dessous).
      - modifier le paramètre `neutral_alt` du nœud `commander` à la hauteur souhaitée (en mètres). C’est la hauteur de la main qui correspondra à l’immobilité de l’altitude.
    
### Paramètres du filtre ###

Les paramètres du filtre (modifiables avec `dynamic_reconfigure` et en particulier `rqt_reconfigure` sont :

* `z_max` : en mètres, altitude maximale de la main, doit être inférieure à la hauteur du plafond.

* pour un gant ou un carton *coloré* (vert, bleu etc.), on a généralement :
  - `hue` : par exemple 220 (bleu ciel) ou 150 (vert) ou 0 (rose/rouge) ;
  - `delta_hue` : 10-20 ;
  - `sat/val_min` : 0.0 ;
  - `sat/val_max` : 1.0 ;

* pour un gant *noir* :
  - `hue` : 0 ;
  - `delta_hue` : 180 ;
  - `sat_min` : 0.0 ;
  - `sat_max` : 1.0 ;
  - `val_min` : 0.0 ;
  - `val_max` : 0.3 (à modifier à votre convenance);

## Connexion au drone et pilotage ##
