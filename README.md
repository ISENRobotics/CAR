# CAR

## créer un catkin workspace:
1. prerequit 
 * avoir installer ros indigo
2. Créer le dossieret l'initialiser
 * mkdir -p ~/catkin_ws/src
 * cd ~/catkin_ws/src
 * catkin_init_workspace
3. Build 
 * cd ~/catkin_ws/
 * catkin_make
4. ajout setup.*sh en fonction de votre terminal
 * si vous utiliser bash
 * ajouter les lignes suivantes dans votre fichier .bashrc : 
    * #ros
    * source /opt/ros/indigo/setup.bash
    * #catkin workspace
    * source ~/catkin_ws/devel/setup.bash
 
## recuperer le git 
 1. clonner le git
  * git clone https://github.com/greatforce/CAR.git ~/catkin_ws/src
 
## initialisation des pwms
   1. ajout de l'execution automatique de script 
      1. copier le fichier initpwm dans /etc/init.d
      * cp ~/catkin_ws/src/script/initpwm /etc/init.d/
      * cd /etc/init.d
      * sudo chmod +x initpwm
      * sudo chown root:root initpwm
      * sudo update-rc.d initpwm defaults
 
## configuration réseaux
   1. modifier le fichier /etc/natwork/interfaces
      1. verifier quelle est l'interface wifi
         * ifcongif -a
      2. ajouter les lignes suivante au fichier interfaces modifier wlan0 si vous n'avez pas le même nom d'interface (ex : wlan1 )
         * auto wlan0
         * iface wlan0 inet dhcp
         *      wpa-ssid "votre ssid"
         *      wpa-psk "votre mdp"
  
