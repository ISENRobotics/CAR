xdel(winsid())
clear;
funcprot(0);//supprime un avertissement génant à l'execution
clf();

// Fonction de dessin de l'objet simulé
function draw_car(x)
    // Dessin de l'objet simulé
    car=[-1,2, 2.5,2.5 ,2 ,-1,-1; // Corps principal de la voiture
           1,1,0.5,-0.5,-1,-1, 1;
           0,0,0,0,0,0,0;
           1,1,1,1,1,1,1];
    carWheel=[-0.5,0.5; // Base pour les roues
               0,0;
               0,0;
               1,1];
    carLeftFW=[cos(x(4)) -sin(x(4)) 0  1.5; // Transformation de la roue avant gauche
               sin(x(4))  cos(x(4)) 0  1.1;
                   0         0       1   0;
                   0         0       0   1];
    carRightFW=[cos(x(4)) -sin(x(4)) 0  1.5; // Transformation de la roue avant droite
                sin(x(4))  cos(x(4)) 0 -1.1;
                    0         0       1   0;
                    0         0       0   1];
    // Matrice de transformation homogène, rotation selon z et translation dans le plan (x,y)
    matRot=[cos(x(3)) -sin(x(3)) 0 x(1);
            sin(x(3))  cos(x(3)) 0 x(2);
               0         0       1   0;
               0         0       0   1];
    // Application de la transformation homogène
    C=matRot*car;
    LFW=matRot*carLeftFW*carWheel;
    RFW=matRot*carRightFW*carWheel;
    // Dessin du motif transformé
    plot([C(1,:)], [C(2,:)],'black','LineWidth',2);
    plot([LFW(1,:)], [LFW(2,:)],'black','LineWidth',2);
    plot([RFW(1,:)], [RFW(2,:)],'black','LineWidth',2);
endfunction

// Fonction de dessin de la liste des waypoints avec les rayons de précision
function plotWP(listeWP, R_MAX)
    a=linspace(0,2*%pi,100);
    b=linspace(-10,10,100);
    s=size(listeWP);
    plot(listeWP(1,1), listeWP(2,1),'o'); // Premier waypoint
    
    for i=2:s(2) // Reste des waypoints
        AB = listeWP(:,i) - listeWP(:,i-1); // Vecteur AB
        
        // équation de la droite perpendiculaire à AB passant par B
        if AB(2)==0 then
            x = zeros(1,100) + listeWP(1,i);
            y = b + listeWP(2,i);
        else
            x = b + listeWP(1,i);
            y = - AB(1)/AB(2)*b + listeWP(2,i);
        end
        
        plot(x, y, 'Line Style', '--'); // Segment perpendiculaire
        plot(listeWP(1,i), listeWP(2,i),'x'); // Centre
        plot(listeWP(1,i)+R_MAX*cos(a), listeWP(2,i)+R_MAX*sin(a)); // Cercle
    end
endfunction

// Fonction d'évolution du pendule, c'est le dX/dt, l'équation d'évolution de la représentation d'état
function y=fctEvolution(x,u)
    A=[0,0,0,0,cos(x(4))*cos(x(3));
       0,0,0,0,cos(x(4))*sin(x(3));
       0,0,0,0,1/l*sin(x(4));
       0,0,0,0,0;
       0,0,0,0,0];
    
    B=[0,0;
       0,0;
       0,0;
       1,0;
       0,1];
    
    y=A*x+B*u;
endfunction

// Fonction de calcul de l'orientation souhaitée de la voiture pour rejoindre la droite (AB)
function [theta_x, theta_y]=orientationSouhaitee(OM, OA, OB, R_MAX)
    // Calcul du vecteur directeur de la droite (AB) passant par les waypoints A et B
    AB = sqrt((OB(1) - OA(1))^2 + (OB(2) - OA(2))^2); // Norme de AB
    u = (OB - OA)/AB; // Vecteur directeur unitaire de (AB)
    AM = OM - OA; // Vecteur position du mobile par rapport au point A
    e = det([AM, u]); // Distance algébrique entre le point M et la droite (AB)
    
    a = atan(e,R_MAX) + atan(u(2),u(1)); // Angle de l'orientation souhaitée
                                         // Somme de l'angle souhaitée par rapport à la droite
                                         // et de l'angle de la droite par rapport à l'angle zéro
    theta_x = cos(a);
    theta_y = sin(a);
endfunction

// Fonction de calcul de la commande de l'angle des roues avant
function delta=angleRoues(theta, theta_des, angle_braq_max)
    // Angle désiré des roues avant entre -pi et pi
    delta_des = pmodulo(theta_des - theta + %pi, 2*%pi) - %pi;
    
    // Seuillage de l'angle des roues avant pour limiter à l'angle de braquage maximum
    if delta_des<(-angle_braq_max) then
        delta=-angle_braq_max;
    elseif delta_des>(angle_braq_max)
        delta=angle_braq_max;
    else
        delta=delta_des;
    end
endfunction

// Fonction de calcul du critère de perpendicularité, produit scalaire de AB avec BM
function res=criterePerp(OM, OA, OB)
    AB = OB - OA; // Vecteur AB
    BM = OM - OB; // Vecteur BM
    
    res = AB(1)*BM(1)+AB(2)*BM(2); // Produit scalaire de AB avec BM
endfunction

// Fonction de calcul du critère de distance
function res=critereDist(OM, OB, R_MAX)
    res = (OM(1) - OB(1))^2 + (OM(2) - OB(2))^2 - R_MAX^2;
endfunction

//Corps principal du programme : Simulation
l=2; // Distance entre les roues arrières et les roues avant
R_MAX = 2.5; // Rayon du "couloir de la droite", paramètre qui influe sur l'orientation souhaitée
listeWP = [ 2,-12, 4, 4;  // Liste des waypoints
           -4, 4, 4, 16];
iWP = 1; // Indice de sélection des waypoints précédent et suivant
dt=0.01; // Pas de temps
x=[-10;9;0;0;35];// On fixe les conditions initiales
u=[0;0];// On fixe les entrées
angle_braq_max = %pi/4; // Angle de braquage maximum en degrée
//OA = x(1:2); // Premier waypoint à la position de départ
//OB = listeWP(:,1); // Initialisation du point B
OA = listeWP(:,iWP); // Initialisation du point A
OB = listeWP(:,iWP+1); // Initialisation du point B

// Création de la figure
fSimulator=scf(0);
clf(0)
set(gca(),"auto_clear","off") // hold on; en matlab, superposition des courbes
mtlb_axis('off'); // Pour effacer les axes
mtlb_axis([-1.5,1.5,-1.5,1.5]);   // Repère

for t=0:dt:30 // Pour t de 0 à 30 par pas de dt
    scf(fSimulator);
    drawlater();
    clf();   // efface la figure courante 
    set(gca(),"auto_clear","off") //hold on; en matlab // superposition des courbes
    mtlb_axis('off'); // pour effacer les axes
    mtlb_axis([-30,30,-30,30]);   // repère
    plot([[OA(1), OB(1)]], [[OA(2), OB(2)]], 'red','LineWidth',2 );
    plotWP(listeWP, R_MAX);
    plot(x(1), x(2), '*', 'Color', 'green');
    draw_car(x); // On dessine la position actuelle de la voiture sur la figure fSimulator
    drawnow(); //pairé avec drawlater(), permet d'éviter l'effet de clignotement désagréable

    x=x+fctEvolution(x,u).*dt // On évalue la prochaine position (Méthode d'Euler)
    
    if critereDist(x(1:2), OB, R_MAX)<=0 then
        iWP = iWP + 1;
        OA = listeWP(:, iWP);
        OB = listeWP(:, iWP+1);
    elseif criterePerp(x(1:2), OA, OB)>=0
        OA = x(1:2);
    end
    
    // On évalue l'orientation souhaitée de la voiture en fonction de sa position actuelle
    [theta_x, theta_y] = orientationSouhaitee(x(1:2), OA, OB, R_MAX)
    theta_des = atan(theta_y, theta_x);
    
    // On modifie l'angle des roues avant en fonction de l'orientation souhaitée et de l'orientation actuelle
    x(4) = angleRoues(x(3), theta_des, angle_braq_max);
    
    //xpause(100*dt); // On met le système en pause
end
