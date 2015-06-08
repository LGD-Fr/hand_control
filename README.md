# Installation #

## Installation des dépendances ##
```
#!sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full ros-indigo-freenect-stack ros-indigo-ardrone-autonomy libncursesw5-dev
```
## Installation du paquet ##

### Création d’un espace de travail catkin ###

Par exemple :

```
#!sh
source /opt/ros/indigo/setup.bash
mkdir -p ~/hand_control_ws/src
cd ~/hand_control_ws/src
catkin_init_workspace
```

### Déplacement du code ###

Renommer si besoin est le dossier qui contient ce ficher en `hand_control`, et le déplacer dans `~/hand_control_ws/src/`, ou dans le sous-dossier `src` de votre espace de travail catkin.

## Compilation ##

Il est ensuite possible de compiler :

```
#!sh
cd ~/hand_control_ws # ou votre espace de travail catkin
catkin_make
```

Puis pour pouvoir utiliser les commoandes ROS, en remplaçant si besoin "hand_control_ws" par votre espace de travail catkin :

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
2. Poser la Kinect sur le sol, pointant le plafond, votre bras devra être perpendiculaire à la Kinect pour pouvoir bien piloter le drone ;
2. Lancer le "launchfile" kinect_commander.launch : `roslaunch hand_control kinect_commander.launch` ;
3. Vérifier les paramètres du filtre :
    - lancer rviz :  `rosrun rqt_rviz rqt_rviz`
    - visualiser la sortie du filtrage (topic : `/filter/output` ; frame : `/camera_depth_optical_frame`) et repérer la main ;
    - lancer rqt_reconfigure : `rosrun rqt_reconfigure rqt_reconfigure` pour :
      - modifier les paramètres du filtre jusqu’à ne voir que les points de la main/gant/pancarte sur rviz (voir ci-dessous).
      - modifier le paramètre `neutral_alt` du nœud `commander` à la hauteur souhaitée (en mètres). C’est la hauteur de la main qui correspondra à l’immobilité de l’altitude.
    
### Paramètres du filtre ###

Les paramètres du filtre (modifiables avec `dynamic_reconfigure` et en particulier `rqt_reconfigure`) sont :

* `z_max` : en mètres, altitude maximale de la main, doit être inférieure à la hauteur du plafond.
* pour un gant ou un carton *coloré* (vert, bleu etc.), on a généralement :
    - `hue` : par exemple 220 (bleu ciel) ou 150 (vert) ou 0 (rose/rouge) ;
    - `delta_hue` : entre 10 et 20 ;
    - `sat/val_min` : 0.0 ;
    - `sat/val_max` : 1.0 ;
* pour un gant *noir* :
    - `hue` : 0 ;
    - `delta_hue` : 180 ;
    - `sat_min` : 0.0 ;
    - `sat_max` : 1.0 ;
    - `val_min` : 0.0 ;
    - `val_max` : 0.3 (à modifier à votre convenance);

### Autres paramètres ###

Toujours avec `rqt_reconfigure`, cette fois pour le nœud `estimator` :
- `reverse` : échange x et y (axes de la Kinect) (valeur par défaut pour une utilisation normale : faux [décoché])
- `reverse_angle` : modifie l’axe choisi pour calculer l’angle de la main (valeur par défaut pour un utilisation normale : faux [décoché])

## Connexion au drone et pilotage ##

* Connecter l’ordinateur au réseau wifi du drone ;
* Lancer le "launchfile" ardrone.launch : `roslaunch hand_control ardrone.launch` ;
* Pour décoller : 
    - soit `rostopic pub /ardrone/takeoff std_msgs/Empty` ;
    - soit lancer le nœud keyboard_cmd : `rosrun hand_control keyboard_cmd` et utiliser la touche *t* du clavier.
* Pour atterir :
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

- Comme écrit plus haut, l’affichage des informations reçues du drone sur `keyboard_cmd` n’est mise à jour qu’à l’occasion de l’appui sur une touche, et peut donc resté fixe quand on n’utilise pas la commande au clavier.
