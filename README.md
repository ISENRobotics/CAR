# CAR

## Créer un catkin workspace:
   1. Prérequis 
      * avoir installé ros indigo
   2. Créer le dossier et l'initialiser
      * mkdir -p ~/catkin_ws/src
      * cd ~/catkin_ws/src
      * catkin_init_workspace
   3. Build 
      * cd ~/catkin_ws/
      * catkin_make
   4. Ajout setup.*sh en fonction de votre terminal
      * si vous utilisez bash
      * ajouter les lignes suivantes dans votre fichier .bashrc : 
         * #ros
         * source /opt/ros/indigo/setup.bash
         * #catkin workspace
         * source ~/catkin_ws/devel/setup.bash
 
## Récupérer le git 
   1. Clonner le git
      * git clone https://github.com/greatforce/CAR.git ~/catkin_ws/src
 
## Initialisation des PWM
   1. Ajout de l'execution automatique de script 
      1. copier le fichier initpwm dans /etc/init.d
         * cp ~/catkin_ws/src/script/initpwm /etc/init.d/
         * cd /etc/init.d
         * sudo chmod +x initpwm
         * sudo chown root:root initpwm
         * sudo update-rc.d initpwm defaults
 
## Configuration réseaux
   1. Modifier le fichier /etc/natwork/interfaces
      1. verifier quelle est l'interface wifi
         * ifcongif -a
      2. ajouter les lignes suivantes au fichier interfaces modifier wlan0 si vous n'avez pas le même nom d'interface (ex : wlan1 )
         * auto wlan0
         * iface wlan0 inet dhcp
         *      wpa-ssid "votre ssid"
         *      wpa-psk "votre mdp"

## Configuration GPS
   1. Installer les paquets nécessaires
      * sudo apt-get install gpsd gpsd-clients python-gps libgps-dev
   2. (optionnel) Afficher directement les données issues du GPS
      * sudo gpscat -s 4800 /dev/tty"votre USB"
   3. (optionnel) Passer le GPS en NMEA
      * gpsctl -n /dev/tty"votre USB"
   4. (optionnel) Modifier le port GPS
      * gpsd -S "votre port" /dev/tty"votre USB"

## Lancer le programme ROS
   1. lancer le .launch
      * roslaunch rc_car car.launch
  



sudo apt-get install ros-indigo-geographic-msgs
poiur robotlocalisation
