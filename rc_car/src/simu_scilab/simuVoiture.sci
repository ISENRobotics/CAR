xdel(winsid())
clear;
funcprot(0);//supprime un avertissement génant à l'execution
clf();

// Fonction de dessin de l'objet simulé
function draw_car(x)
    drawlater();
    // 1) Configuration du graphique
    clf();   // efface la figure courante 
    set(gca(),"auto_clear","off") //hold on; en matlab // superposition des courbes
    mtlb_axis('off'); // pour effacer les axes
    mtlb_axis([-10,10,-10,10]);   // repère
    // 2) Dessin de l'objet simulé
    car=[-1,2, 2.5,2.5 ,2 ,-1,-1; // Corps principal de la voiture
           1,1,0.5,-0.5,-1,-1, 1;
           0,0,0,0,0,0,0;
           1,1,1,1,1,1,1];
    carWheel=[-0.5,0.5; // Base pour les roues
               0,0;
               0,0;
               1,1];
    carLeftFW=[cos(x(4)) -sin(x(4)) 0  1.5; // Transformation de la roue avant gauche
               sin(x(4))  cos(x(4)) 0    1.1;
                   0         0       1   0;
                   0         0       0   1];
    carRightFW=[cos(x(4)) -sin(x(4)) 0  1.5; // Transformation de la roue avant droite
                sin(x(4))  cos(x(4)) 0   -1.1;
                    0         0       1   0;
                    0         0       0   1];
    // 3) Matrice de transformation homogène, rotation selon z et translation dans le plan (x,y)
    matRot=[cos(x(3)) -sin(x(3)) 0 x(1);
            sin(x(3))  cos(x(3)) 0 x(2);
               0         0       1   0;
               0         0       0   1];
    // 4) Application de la transformation homogène
    C=matRot*car;
    LFW=matRot*carLeftFW*carWheel;
    RFW=matRot*carRightFW*carWheel;
    // 5) Dessin du motif transformé
    plot([C(1,:)], [C(2,:)],'black','LineWidth',2);
    plot([LFW(1,:)], [LFW(2,:)],'black','LineWidth',2);
    plot([RFW(1,:)], [RFW(2,:)],'black','LineWidth',2);
    drawnow(); //pairé avec drawlater(), permet d'éviter l'effet de clignotement désagréable
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

// Fonction de dessin du champ de vecteur de l'orientation souhaitée
function y=plotChpVect(OA,OB,R_MAX)
    // Définition d'un espace de travail
    Mx = -10:1:10; My = -10:1:10;
    [X1,X2] = meshgrid(Mx,My);
    
    theta_x = zeros(size(X1,1), size(X1,2));
    theta_y = zeros(size(X1,1), size(X1,2));
    
    // Extrémités du segment représentant la droite à rejoindre
    ligne_c1 = [-10 ; (OB(2)-OA(2))/(OB(1)-OA(1))*(-10-OA(1))+OA(2)];
    ligne_c2 = [10 ; (OB(2)-OA(2))/(OB(1)-OA(1))*(10-OA(1))+OA(2)];
    
    alpha = atan(OB(2)-OA(2), OB(1)-OA(1)); // Angle du vecteur AB
    // Extrémités des segments représentant le couloir de rayon R_MAX autour de la droite à rejoindre
    ligne_d1 = [ligne_c1(1)+R_MAX.*sin(alpha) ; ligne_c1(2)-R_MAX.*cos(alpha)];
    ligne_d2 = [ligne_c2(1)+R_MAX.*sin(alpha) ; ligne_c2(2)-R_MAX.*cos(alpha)];
    ligne_g1 = [ligne_c1(1)-R_MAX.*sin(alpha) ; ligne_c1(2)+R_MAX.*cos(alpha)];
    ligne_g2 = [ligne_c2(1)-R_MAX.*sin(alpha) ; ligne_c2(2)+R_MAX.*cos(alpha)];
    
    // Boucle de calcul du champ de vecteur aux points considérés
    for i=1:size(X1,1)
        for j=1:size(X1,2)
            [theta_x(i,j), theta_y(i,j)] = orientationSouhaitee([X1(i,j) ; X2(i,j)], OA, OB, R_MAX);
        end
    end

    champ(Mx,My,theta_x',theta_y','blue'); // Dessin du champ de vecteur
    // Dessin des éléments associés au vecteur AB
    plot([[ligne_c1(1), ligne_c2(1)]], [[ligne_c1(2), ligne_c2(2)]],'red','LineWidth',2);
    plot([[ligne_d1(1), ligne_d2(1)]], [[ligne_d1(2), ligne_d2(2)]],'blue','LineWidth',2);
    plot([[ligne_g1(1), ligne_g2(1)]], [[ligne_g1(2), ligne_g2(2)]],'blue','LineWidth',2);
    plot(OA(1), OA(2), 'black','o');
    plot(OB(1), OB(2), 'black','x');
    //plot([[OA(1), OB(1)]], [[OA(2), OB(2)]],'red','LineWidth',2);
    y=0;
endfunction

//Corps principal du programme : Simulation
l=2; // Distance entre les roues arrières et les roues avant
R_MAX = 2.5; // Rayon du "couloir de la droite", paramètre qui influe sur l'orientation souhaitée
OA = [1 ; 1]; // Coordonnées du points A
OB = [-2 ; 2]; // Coordonnées du points B
dt=0.01; // Pas de temps
x=[0;0;0;0;20];// On fixe les conditions initiales
u=[0;0];// On fixe les entrées
angle_braq_max = %pi/4; // Angle de braquage maximum en degrée

// Création de deux figures, une pour la simulation, l'autre pour le champ de vecteur
fSimulator=scf(0);
clf(0)
set(gca(),"auto_clear","off") // hold on; en matlab, superposition des courbes
mtlb_axis('off'); // Pour effacer les axes
mtlb_axis([-1.5,1.5,-1.5,1.5]);   // Repère
fChpVect=scf(1);
clf(1)
set(gca(),"auto_clear","off") // hold on; en matlab, superposition des courbes
mtlb_axis('off'); // Pour effacer les axes
mtlb_axis([-7,7,-7,7]);   // Repère
// Champ de vecteur
plotChpVect(OA, OB, R_MAX);

for t=0:dt:30 // Pour t de 0 à 30 par pas de dt
    scf(fSimulator);
    draw_car(x); // On dessine la position actuelle de la voiture sur la figure fSimulator
    scf(fChpVect);
    plot(x(1),x(2), '+'); // On dessine la position actuelle de la voiture sur la figure fChpVect

    x=x+fctEvolution(x,u).*dt // On évalue la prochaine position (Méthode d'Euler)
    
    // On évalue l'orientation souhaitée de la voiture en fonction de sa position actuelle
    [theta_x, theta_y] = orientationSouhaitee(x(1:2), OA, OB, R_MAX)
    theta_des = atan(theta_y, theta_x);
    
    // On modifie l'angle des roues avant en fonction de l'orientation souhaitée et de l'orientation actuelle
    x(4) = angleRoues(x(3), theta_des, angle_braq_max);
    
    //xpause(100*dt); // On met le système en pause
end
