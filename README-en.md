# Installation #

This package was developped with the Indigo version of ROS.

## Dependencies installation ##
```
#!sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full ros-indigo-freenect-stack ros-indigo-ardrone-autonomy libncursesw5-dev
```
## Package installation ##

### Catkin workspace creation ###

For instance :

```
#!sh
source /opt/ros/indigo/setup.bash
mkdir -p ~/hand_control_ws/src
cd ~/hand_control_ws/src
catkin_init_workspace
```

### Code location ###

If necessary, rename the folder with the file named `hand_control`, and move it in `~/hand_control_ws/src/` or in the subfolder `src` of your catkin workspace.

## Compilation ##

You're now able to compile :

```
#!sh
cd ~/hand_control_ws # or your catkin workspace
catkin_make
```

Then you can run the following commands to be able to use the ROS commands. If necessary replace "hand_control_ws" by the name of your catkin workspace.

```
#!sh
source /opt/ros/indigo/setup.bash
source ~/hand_control_ws/devel/setup.bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo "source ~/hand_control_ws/devel/setup.bash" >> ~/.bashrc
```

# Use #

## Connection and setting up of the Kinect ##

1. Connect the Kinect (under voltage) to the computer via USB ;
2. Put the Kinect on the ground, pointed toward the roof ; be aware that your arm must be perpendicular to the Kinect in order to control the drone properly ;
2. Launch the "launchfile" kinect_commander.launch : `roslaunch hand_control kinect_commander.launch` ;
3. Check the drone parameters :
    - launch rviz :  `rosrun rqt_rviz rqt_rviz`
    - display the output of the filtering (topic : `/filter/output` ; frame : `/camera_depth_optical_frame`) and locate the hand ;
    - launch rqt_reconfigure : `rosrun rqt_reconfigure rqt_reconfigure` in order to :
      - change the filter parameters until you only see the points of the hand/glove/panel on rviz (see above).
      - change the parameter `neutral_alt` of the node `commander` to the wanted height (in meters), correponding to the height of the hand for which the height of the drone will be stable.
    
### Parameters of the filter ###

The parameters of the filter (that can be changed thanks to `dynamic_reconfigure` and in particular `rqt_reconfigure`) are :

* `z_max` : in meters, maximal height of the hand. It must be lower than the height of the roof.
* for a glove or a *colored$* panel (green, blue, etc.), we generaly have :
    - `hue` : for example 220 (sky blue) or 150 (green) or 0 (pink/red) ;
    - `delta_hue` : between 10 and 20 ;
    - `sat/val_min` : 0.0 ;
    - `sat/val_max` : 1.0 ;
* pour un gant *noir* :
    - `hue` : 0 ;
    - `delta_hue` : 180 ;
    - `sat_min` : 0.0 ;
    - `sat_max` : 1.0 ;
    - `val_min` : 0.0 ;
    - `val_max` : 0.3 (at your convenience);

### Other parameters ###

Always with `rqt_reconfigure`, but with the `estimator` node :
- `reverse` : swap x and y, the axes of the Kinect (default : false, ie. unchecked)
- `reverse_angle` : change the angle choosen for the compute of the angle of the hand (default : false, ie. unchecked)

## Connection to the drone and steering ##

* Connect the computer to the wifi network of the drone ;
* Lancer le "launchfile" ardrone.launch : `roslaunch hand_control ardrone.launch` ;
* Pour décoller : 
    - soit `rostopic pub /ardrone/takeoff std_msgs/Empty` ;
    - soit lancer le nœud keyboard_cmd : `rosrun hand_control keyboard_cmd` et utiliser la touche *t* du clavier.
* Pour atterrir :
    - soit `rostopic pub /ardrone/land std_msgs/Empty` ;
    - soit, avec keyboard_cmd, utiliser la touche *b* du clavier.
* Arrêt d’urgence :
    - soit `rostopic pub /ardrone/reset std_msgs/Empty` ;
    - soit, avec keyboard_cmd, utiliser la touche *g* du clavier.

### Commande à la main ###

* Avancer/reculer & translations latérales : inclinaison de la main ;
* Tourner (rotation autours de l’axe z) : angle de l’axe de la main avec l’axe parallèle au sol et perpendiculaire à la Kinect ;
* Monter/descendre : altitude de la main.

### Options et paramètres de la commande ###

Pour éditer les options de la commande, lancer si ce n’est déjà fait `rosrun rqt_reconfigure rqt_reconfigure` :

- `max_curvature` : non utilisé pour l’instantt ;
- `x/y/z/theta_minimal_deviation` : seuils à partir desquels le mouvement de la main est pris en compte. Tout mettre à 0.0 rend le comportement complétement linéaire.
    * x, y : entre 0. et 1. (il s’agit des composantes x et y de la normale au plan);
    * z : en mètre ;
    * theta : en degrés.
- `neutral_alt` : hauteur de la main qui correspond à l’immobilité de l’altitude du drone ;
- `min_points_number` : nombre minimal de points (du nuage de points qui a servi à régresser le plan reçu) nécessaire pour qu’une commande soit envoyé au drone.
- `angle/x/y/z_vel` : coefficients de proportionnalité à appliquer aux données en entrée pour établir la commande envoyée au drone. Les augmenter augmentera la vitesse du drone.
- `up_fact` : coefficient de proportionnalité à appliquer à la commande augmentant l’altitude du drone, par rapport à la commande équivalente la diminuant (pour corriger l’effet de la gravité).

### Notes sur `keyboard_cmd` ###

Il permet de publier des commandes sur le topic `cmd_vel` et ainsi de piloter le drone. Il est prévu pour les claviers azerty. Pour le lancer :

```
#!sh
rosrun hand_control keyboard_cmd
```

Pour augmenter/diminuer les vitesses (expliqué sur l’affichage du programme) : touches a,z,e,r et w,x,c,v

L’affichage des informations reçues du drone n’est mise à jour qu’à l’occasion de l’appui sur une touche. 

Pour quitter : Ctr+C, et appui sur "Entrée" pour retrouver l’affichage de la console.

# Problème(s) rencontré(s) — Améliorations souhaitables #

- Si des commandes sont publiées sur `cmd_vel` (depuis la Kinect par exemple) après le lancement du fichier `ardrone.launch` et avant le décollage du drone, alors, après le décollage, de drone semble obéir aux commandes publiées avant le décollage.

- Comme écrit plus haut, l’affichage des informations reçues du drone sur `keyboard_cmd` n’est mise à jour qu’à l’occasion de l’appui sur une touche, et peut donc rester fixe quand on n’utilise pas la commande au clavier.

- Le décollage/atterrissage n’est pas possible à commander avec la main. Il faut utiliser le clavier (`keyboard_cmd` ou `rostopic pub`) à la place. Il est possible de de remédier à cela en créant deux nouveaux seuils, minimaux et maximaux, pour la hauteur de la main : une main très basse ferait atterrir le drone, une main très haute le ferait décoller.
