# Contrôle par geste d'un drone #

**Extrait de la présentation du projet**

« On s'intéresse dans ce projet à contrôler un drone à l'aide la main. On utilisera pour ce faire une kinect, placée à l'horizontal, au dessus de laquelle on placera la main du contrôleur. La kinect fournit des informations sur la profondeur des objets placés en face d'elle. On peut alors régresser un plan sur les échantillons et utiliser deux inclinaisons et la distance pour contrôler le roulis, le tangage et l'altitude d'un drone. »

## Recherche documentaire ##
Consulter le dossier /rechercheDoc

Un etherpad est disponible à l'url suivante : [http://etherpad.rezometz.org/p/handcontrol](http://etherpad.rezometz.org/p/handcontrol)

### Conclusion de la recherche ###

Bibliothèques à regarder :


- [PCL (PointCloud)](http://www.pointclouds.org/) : beaucoup, beaucoup de choses...
    * [les tutoriels](http://www.pointclouds.org/documentation/tutorials/)
    * [pcl::NormalEstimation](http://docs.pointclouds.org/trunk/classpcl_1_1_normal_estimation.html) & [pcl::NormalEstimationOMP](http://docs.pointclouds.org/trunk/classpcl_1_1_normal_estimation_o_m_p.html) pour régresser un plan
         - en particulier [computePointNormal](http://docs.pointclouds.org/trunk/classpcl_1_1_normal_estimation.html#afa0dd0bf400977f40eb91f08750bfa17) qui sert l’équation du plan sur un plateau
         - explication [ici](http://www.pointclouds.org/documentation/tutorials/normal_estimation.php#normal-estimation)
    * [pcl::ConditionalRemoval](http://docs.pointclouds.org/trunk/classpcl_1_1_conditional_removal.html) pour filtrer les points (critère de profondeur (plus simple a priori) ou couleur)
- AR Drone :
    * [ardrone_autonomy](https://github.com/AutonomyLab/ardrone_autonomy/tree/master#sending-commands-to-ar-drone)

Pour l’interface Kinect on a le choix entre :

- l’interface fournie par PCL, qui récupère directement un PointCloud utilisable par PCL à partir de la Kinect :
    * [openni_wrapper::DeviceKinect](http://docs.pointclouds.org/trunk/classopenni__wrapper_1_1_device_kinect.html) & [openni_wrapper::OpenNIDevice](http://docs.pointclouds.org/trunk/classopenni__wrapper_1_1_open_n_i_device.html) pour parler à la Kinect
    * openni_grabber : [tuto](http://www.pointclouds.org/documentation/tutorials/openni_grabber.php#openni-grabber), [doc](http://docs.pointclouds.org/trunk/classpcl_1_1_open_n_i_grabber.html)
- l’interface fournie par les packages ROS :
    * [OpenKinect / libfreenect](https://github.com/OpenKinect) avec [freenect_stack](http://wiki.ros.org/freenect_stack) qui publie des sensor_msgs::Image
    * puis conversion des sensor_msgs::Image en pcl::PCLPointCloud2 avec [pcl_conversions::moveToPCL](http://docs.ros.org/indigo/api/pcl_conversions/html/namespacepcl__conversions.html#a40366a910d7ce4ae63b121150381098d) du package [perception_pcl](http://wiki.ros.org/perception_pcl?distro=indigo), cf. [pcl_ros](http://wiki.ros.org/pcl_ros?distro=indigo)
    * puis utilisation avec [pcl_ros](http://wiki.ros.org/pcl_ros?distro=indigo) (conversion automatique std_msgs/PointCloud2 => PointCloud), sinon conversion manuelle avec [pcl::fromPCLPointCloud2](http://docs.pointclouds.org/trunk/namespacepcl.html#a89aca82e188e18a7c9a71324e9610ec9) ou [pcl::fromROSMsg](http://docs.pointclouds.org/trunk/namespacepcl.html#a22b3a98851964a79fafa7b9e543b3c75)

Publication & Souscription à des topics de PointCloud avec [pcl_ros](http://wiki.ros.org/pcl_ros?distro=indigo)

### Important ###
Avant de coder, regarder :

- [les conventions d’écriture du code du projet ROS](http://wiki.ros.org/CppStyleGuide)
- [le guide du développeur](http://wiki.ros.org/DevelopersGuide)

## Mail du responsable de projet (extrait) ##

Donc sur le principe du projet :

- on place une kinect pointant vers le haut de laquelle on va extraire un nuage de points (PointCloud) correspondant à une main placée au dessus. D'ailleurs, je pense qu'on prendre un gant de couleur unie bien criarde pour facilement filtrer les points d'intérêt et ne s'intéresse qu'à ceux de la main.
- on régresse un plan sur le point cloud
- on utilise les paramètres du plan pour définir le roll, pitch, altitude d'un drone;

Il vous faut donc dans un premier temps :
 
- faire les tutoriels de ROS (jusqu'au tutoriel sur les publisher et subscribers en C++/Python); Pour installer ROS sur votre machine, c'est beaucoup plus simple à ma connaissance d'installer ubuntu (http://wiki.ros.org/)
- regarder comment on définit/utilise des launch files sous ROS, ce qui facilite grandement le lancement de plusieurs noeuds (http://wiki.ros.org/roslaunch)
- regarder et utiliser la kinect sous ROS en utilisant freenect;  Notamment regarder les nœuds et ce qu'ils publient sur leurs topics (http://wiki.ros.org/freenect_camera), il y a notamment une image de profondeur et une image RGB;
- regarder et utiliser le drone sous ROS : (par exemple : http://wiki.ros.org/ardrone_driver)

J'ai sorti une kinect, elle est posée sur un des bureaux de la smartroom.

Pour rappel, à la fin du projet, je vous demanderais de me remettre un rapport et de faire une soutenance.