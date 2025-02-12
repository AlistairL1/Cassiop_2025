# Objectif
Ce document servira à synthétiser les réunions avec notre tuteur

## Réunion du 17 Décembre
- On a notre position, une trajectoire de reference que le drone doit suivre on veut aller de a vers b => suivre une trajectoire
- On a deux points a et b et il faut le rejoindre , si trajectoire constante (voir lien chat) - graphe ligne pointillée qui est la référence , en utilisant le régulateur - refefrence se deplace vers le point B - si on deplace la reference on ralenti et si on ne l'atteint pas on accelère
- Le truc le plus compliqué : comment choisir A et B de façon à ce que ça suivre notre trajectoire
- Se baser sur des techniques qui sont basées sur des slams
- Plutôt position de référence ou vitesse de référence ? si on connaît la dérivée, c'est plus précis

## Réunion du 3 février
- Lorsqu'on a une personne attachée au drone, la direction latérale est désactivée
- Code le parcours d'un chemin (suivre trajectoire) :
    - Aller d'un point A, à un point B, à un point C, ...
    - Voir le chemin d'une façon plus "mathématique"
    - Pour simplifier, lorsque le drone se déplace sans personne attachée, on peut admettre qu'on atteint un point A, B, C, ... lorsqu'on arrive à un certain _range_ dudit point.
    - Autre solution, utiliser un simple contrôleur proportionnel


- Planification trajectoire (créer le chemin pour aller de A vers B) avec des obstacles :
    - Créer une zone d'exclusion avec une suite de points (regarder OMPL)


- Problèmes de hauts-niveaux :
    - Prendre/Établir des décisions
    - Établir la carte sur laquelle le drone se déplace (_Stable Machine_)
    - Si le GPS ou le LIDAR est bruité, est-il possible de fournir le signal tel que fourni ou faut-il le lisser ?
    - Pour la communication, si on a le temps, trouver une implémentation de SLAM qui ne repose pas sur ROS (_Robotic Operating System_)


- Pour la semaine prochaine :
    - Contrôleur avec x et y ET avec x, y et Theta

## Réunion du 10 février
- Principe fonctionnement du filtre passe-bas
- Proposition d'autres filtres numériques (moyenne glissante)

- Filtre passe-bas bien car 1 seule multiplication MAIS déphasage important et chute importante de l'amplitude.

- Configuration A avec (x et y)

- Pour fix mon code (converger pour un chemin de A vers B), connaissant \theta orientation et theta cible, pondéré l'accélération (V = V *(1_(theta_or - theta_c)))

- On a le droit d'import des bibliothèques supplémentaires