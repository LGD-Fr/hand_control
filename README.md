# Contrôle par geste d'un drone #

**Extrait de la présentation du projet**

« On s'intéresse dans ce projet à contrôler un drone à l'aide la main. On utilisera pour ce faire une kinect, placée à l'horizontal, au dessus de laquelle on placera la main du contrôleur. La kinect fournit des informations sur la profondeur des objets placés en face d'elle. On peut alors régresser un plan sur les échantillons et utiliser deux inclinaisons et la distance pour contrôler le roulis, le tangage et l'altitude d'un drone. »

## Utilisation du dépôt ##

Après avoir créé un espace de travail catkin :

```
#!sh
mkdir -p ~/hand_control_ws/src
cd ~/hand_control_ws/src
catkin_init_workspace

```
vous devez cloner le dépôt de telle sorte que le dossier «hand_control» se situe dans le dossier «~/hand_control_ws/src/», par exemple :

```
#!sh
git clone git@bitbucket.org:_Luc_/handcontrol.git
mv -t ~/hand_control_ws/src handcontrol/* handcontrol/.git
rmdir handcontrol
```
Le contenu du dépôt se trouve alors dans «~/hand_control_ws/src». Il est ensuite possible de compiler :

```
#!sh
cd ~/hand_control_ws
catkin_make
```

Puis pour faciliter le développement : 
```
#!sh
source ~/hand_control_ws/devel/setup.bash
echo "source ~/hand_control_ws/devel/setup.bash" >> ~/.bashrc
```

Cf. le Wiki pour l’installation de ROS, PCL et des pilotes pour la Kinect et le drone.
