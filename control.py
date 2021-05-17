#
# Fortune Maeva et Tissot Priscilla
#

import numpy as np
import math
if __package__ is None or __package__ == '':
    import interpolation
else:
    from . import interpolation

def sandbox(t):
    """
    python simulator.py -m sandbox

    Un premier bac à sable pour faire des expériences

    La fonction reçoit le temps écoulé depuis le début (t) et retourne une position cible
    pour les angles des 12 moteurs

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    joints = [0]*12
    
    # position debout immobile
    for joint in [2,5,8,11]:
        joints[joint] = math.pi/2
    
    t = t%5

    if(t>4 and t<5) :# pates 2 et 4 posées et 1 et 3 en arrière posées
        joints[0]= -math.pi/6
        joints[6] = math.pi/6
    elif(t>3 and t<4) : # leve pate 2 et 4 et les pates 1 et 3 sont mises en arrière
        joints[4] = math.pi/6
        joints[10] = math.pi/6
        joints[0]= -math.pi/6
        joints[6] = math.pi/6
    elif(t>2 and t<3):#pates 1 et 3 en avant et posées
        joints[0]= math.pi/6
        joints[6] = -math.pi/6
    elif(t>1 and t<2) : #Avance les pates 1 et 3 (qui sont toujours levées)
        joints[1]= math.pi/6
        joints[7] = math.pi/6
        joints[0]= math.pi/6
        joints[6] = -math.pi/6
    elif(t>0 and t<1) : # leve les pates 1 et 3
        joints[1] = math.pi/6
        joints[7] = math.pi/6
    
    return joints

def inverse(x, y, z):
    """
    python simulator.py -m inverse

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Sliders: La position cible x, y, z du bout de la patte
    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    l1 = 0.045
    l2 = 0.065
    l3 = 0.087

    alpha = np.arctan2(y,x)
    xA = l1*np.cos(alpha)
    yA = l1*np.sin(alpha)
    zA = 0

    AC = np.sqrt((x-xA)*(x-xA) + (y-yA)*(y-yA) + (z-zA)*(z-zA))

    cos_beta = (AC*AC + l2*l2 - l3*l3) / (2*AC*l2)
    if cos_beta < -1: cos_beta = -1
    if cos_beta > 1:  cos_beta = 1
    beta = np.arccos(cos_beta)

    diff = np.arcsin((z-zA)/AC)
    beta = beta+diff

    cos_gamma = (l2*l2 + l3*l3 - AC*AC) / (2*l2*l3)
    if cos_gamma < -1: cos_gamma = -1
    if cos_gamma > 1:  cos_gamma = 1
    gamma = np.arccos(cos_gamma)
    gamma = np.pi - gamma

    return [alpha, beta, gamma]

def draw(t):
    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    traj = interpolation.LinearSpline3D()
    traj.add_entry(0, 0.17, 0.08, -0.04)
    traj.add_entry(2, 0.17, -0.08, -0.04)
    traj.add_entry(4, 0.17, 0.0, 0.04)
    traj.add_entry(6, 0.17, 0.08, -0.04)

    M = traj.interpolate(math.fmod(t,6))

    return inverse(M[0], M[1], M[2])

def legs(leg1, leg2, leg3, leg4):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    targets = [0]*12
    
    # translation
    ecart = np.sqrt(0.04*0.04/2)

    # initialisation de x et y de chaque pattes
    x1 = calculX(-3*np.pi/4,0, leg1)-0.04
    y1 = calculY(-3*np.pi/4,0, leg1)
    x2 = calculX(3*np.pi/4,-ecart, leg2)
    y2 = calculY(3*np.pi/4,-ecart, leg2)
    x3 = calculX(np.pi/4,0, leg3) -0.04
    y3 = calculY(np.pi/4,0, leg3)
    x4 = calculX(-np.pi/4,ecart, leg4)
    y4 = calculY(-np.pi/4,ecart, leg4)
    
    # calcule des angles de chaque patte
    anglepate1 = inverse(x1,y1,leg1[2])
    anglepate2 = inverse(x2,y2,leg2[2])
    anglepate3 = inverse(x3,y3,leg3[2])
    anglepate4 = inverse(x4,y4,leg4[2])

    # initialisation des coordonnées dans targets
    targets[2] = anglepate1[2]
    targets[1] = anglepate1[1]
    targets[0] = anglepate1[0]
    targets[5] = anglepate2[2]
    targets[4] = anglepate2[1]
    targets[3] = anglepate2[0]
    targets[8] = anglepate3[2]
    targets[7] = anglepate3[1]
    targets[6] = anglepate3[0]
    targets[11] = anglepate4[2]
    targets[10] = anglepate4[1]
    targets[9] = anglepate4[0]

    return targets

# calcule les coordonées de X après translation et rotation
def calculX(radian, ecart, leg) :
    return (leg[0]-ecart)*np.cos(radian) - (leg[1]-ecart)*np.sin(radian)

# calcule les coordonées de Y après translation et rotation
def calculY(radian, ecart, leg) :
    return (leg[0]-ecart)*np.sin(radian) + (leg[1]-ecart)*np.cos(radian)

def walk(t, speed_x, speed_y, speed_rotation):
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    # taille des 2 premieres parties de bras
    bras = 0.114

    #pate 1
    trajp1 = interpolation.LinearSpline3D()
    trajp1.add_entry(0, -bras,bras,-0.087) # bras droit et posé
    trajp1.add_entry(1, -bras,bras,-0.06)# bras droit et levé
    trajp1.add_entry(2, calculX(speed_y-speed_x+speed_rotation,0,(0,np.sqrt(2*bras*bras))), calculY(speed_y-speed_x+speed_rotation,0,(0,np.sqrt(2*bras*bras))),-0.06)# bras en avant et levé + prise en compte des speed
    trajp1.add_entry(3, calculX(speed_y-speed_x+speed_rotation,0,(0,np.sqrt(2*bras*bras))), calculY(speed_y-speed_x+speed_rotation,0,(0,np.sqrt(2*bras*bras))),-0.087)# bras en avant et posé + prise en compte des speed
    trajp1.add_entry(4, calculX(speed_x-speed_y+speed_rotation,0,(-np.sqrt(2*bras*bras),0)), calculY(speed_x-speed_y+speed_rotation,0,(-np.sqrt(2*bras*bras),0)),-0.087)# (traction) bras en arrière et posé + prise en compte des speed
    trajp1.add_entry(5, calculX(speed_x-speed_y+speed_rotation,0,(-bras,bras)), calculY(speed_x-speed_y+speed_rotation,0,(-bras,bras)),-0.06)# bras en arrière et levé + prise en compte des speed

    M1 = trajp1.interpolate(math.fmod(t,5))

    #pate 2
    trajp2 = interpolation.LinearSpline3D()
    trajp2.add_entry(0, calculX(0,0,(-bras,-bras)), calculY(0,0,(-bras,-bras)),-0.087)# bras droit et posé
    trajp2.add_entry(2.9, calculX(0,0,(-bras,-bras)), calculY(0,0,(-bras,-bras)),-0.087)# bras droit et posé
    trajp2.add_entry(3, calculX(speed_rotation*2,0,(-bras,-bras)), calculY(speed_rotation,0,(-bras,-bras)),-0.06)# bras levé 
    trajp2.add_entry(4, calculX(speed_rotation*2,0,(-bras,-bras)), calculY(speed_rotation,0,(-bras,-bras)),-0.06)# bras levé
    trajp2.add_entry(4.9, calculX(speed_rotation*2,0,(-bras,-bras)), calculY(speed_rotation,0,(-bras,-bras)),-0.087)# bras posé

    M2 = trajp2.interpolate(math.fmod(t,5))

    #pate3
    trajp3 = interpolation.LinearSpline3D()
    trajp3.add_entry(0, bras,-bras,-0.087)# bras droit et posé
    trajp3.add_entry(1, bras,-bras,-0.06)# bras droit et levé
    trajp3.add_entry(2, calculX(speed_y-speed_x+speed_rotation,0,(np.sqrt(2*bras*bras),0)),calculY(speed_y-speed_x+speed_rotation,0,(np.sqrt(2*bras*bras),0)),-0.06)# bras en avant et levé + prise en compte des speed
    trajp3.add_entry(3, calculX(speed_y-speed_x+speed_rotation,0,(np.sqrt(2*bras*bras),0)),calculY(speed_y-speed_x+speed_rotation,0,(np.sqrt(2*bras*bras),0)),-0.087)# bras en avant et posé + prise en compte des speed
    trajp3.add_entry(4, calculX(speed_x-speed_y+speed_rotation,0,(0,-np.sqrt(2*bras*bras))),calculY(speed_x-speed_y+speed_rotation,0,(0,-np.sqrt(2*bras*bras))),-0.087)# (traction) bras en arrière et posé + prise en compte des speed
    trajp3.add_entry(5, calculX(speed_x-speed_y+speed_rotation,0,(bras,-bras)),calculY(speed_x-speed_y+speed_rotation,0,(bras,-bras)),-0.06)# bras en arrière et levé + prise en compte des speed

    M3 = trajp3.interpolate(math.fmod(t,5))

    #pate 4
    trajp4 = interpolation.LinearSpline3D()
    trajp4.add_entry(0, calculX(0,0,(bras,bras)), calculY(0,0,(bras,bras)),-0.087)# bras droit et posé
    trajp4.add_entry(2.9, calculX(0,0,(bras,bras)), calculY(0,0,(bras,bras)),-0.087)# bras droit et posé
    trajp4.add_entry(3, calculX(speed_rotation*2,0,(bras,bras)), calculY(speed_rotation,0,(bras,bras)),-0.06)# bras levé 
    trajp4.add_entry(4, calculX(speed_rotation*2,0,(bras,bras)), calculY(speed_rotation,0,(bras,bras)),-0.06)# bras levé 
    trajp4.add_entry(4.9, calculX(speed_rotation*2,0,(bras,bras)), calculY(speed_rotation,0,(bras,bras)),-0.087)# bras posé

    M4 = trajp4.interpolate(math.fmod(t,5))


    return legs(M1, M2, M3, M4)


if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")