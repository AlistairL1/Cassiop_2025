# Objectif
Ce document servira à synthétiser les réunions avec notre tuteur

## Réunion du 17 Décembre
- On a notre position, une trajectoire de reference que le drone doit suivre on veut aller de a vers b => suivre une trajectoire
- On a deux points a et b et il faut le rejoindre , si trajectoire constante (voir lien chat) - graphe ligne pointillée qui est la référence , en utilisant le régulateur - refefrence se deplace vers le point B - si on deplace la reference on ralenti et si on ne l'atteint pas on accelère
- Le truc le plus compliqué : comment choisir A et B de façon à ce que ça suivre notre trajectoire
- Se baser sur des techniques qui sont basées sur des slams
- Plutôt position de référence ou vitesse de référence ? si on connaît la dérivée, c'est plus précis