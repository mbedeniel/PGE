# Yaskawa réels : points à vérifier

- **Objet :** confirmer que la simulation (`yaskawa/`, paquets `mfja_yaskawa_*`) correspond aux deux Yaskawa réels de la salle 315, et récupérer ce qu'il manque pour la terminer.
- **Avec :** Diane BURY (MFJA), qui a passé les Yaskawa sous ROS 2.
- **Version :** 18/11/2026

Ce qui vient déjà des fichiers utilisés sur les robots réels (dépôt `ros_yaskawa_hc10`) : dimensions, axes et limites des articulations, noms des articulations, modèle de chaque côté. Tout le reste est à vérifier ci-dessous.

## Points à vérifier

| N° | Point | Comment le vérifier | Valeur dans la simulation | Résultat |
|---|---|---|---|---|
| 1 | Interface ROS 2 réelle | Où est la configuration (dépôt, branche, PC) ? Quel nœud ROS 2 (MotoROS2 ? version) ? Robot connecté, relever `ros2 topic list -t`, `ros2 service list -t`, `ros2 action list -t`. | Action `<ns>/arm_controller/follow_joint_trajectory`, `<ns>/joint_states` | |
| 2 | Mise en mouvement | Séquence pour autoriser un mouvement (activation des servos, mode « trajectoire », remise à zéro d'une alarme) et mode du pupitre requis (distant, apprentissage). | Aucune (mouvement immédiat) | |
| 3 | Noms | Espaces de noms, noms des articulations, préfixe éventuel des repères TF. | `yaskawa_LEFT`, `yaskawa_RIGHT` ; `joint_1_s` … `joint_6_t` | |
| 4 | Modèle de chaque côté | Plaque signalétique de chaque robot. | Gauche : HC10DT (capots blancs, `10.100.12.134`) ; droite : HC10 (capots bleus, `.133`) | |
| 5 | Zéro de l'articulation 1 du HC10DT | Mettre l'articulation 1 (S) à 0, noter la direction du bras par rapport à la table et au rail. Comparer avec `show.launch.py robot_model:=hc10dt` à `joint_1_s = 0`. | Décalage de +90° sur l'origine de `joint_1_s` (repris du dépôt réel, 0° dans le modèle ROS-Industrial d'origine) | |
| 6 | Géométrie (3 ou 4 poses par robot) | Au pupitre : angles des 6 articulations (°) et position de la bride en coordonnées ROBOT, outil 0 (X, Y, Z en mm). Mêmes angles en simulation, lire `base` → `tool0` (commande ci-dessous). Écart attendu < 5 mm. | Modèles `hc10_robot` et `hc10dt_robot` | |
| 7 | Limites articulaires | Limites logicielles du contrôleur pour chaque axe. | HC10 : U −5°…355°, autres ±180° ; HC10DT : U −95°…265°, autres ±180° | |
| 8 | Vitesses | Vitesses maximales autorisées, en particulier en mode collaboratif. | 130, 130, 180, 180, 250, 250 °/s | |
| 9 | Même trajectoire réel / simulation | Envoyer la même trajectoire lente aux deux, enregistrer `joint_states` (`ros2 bag record`), comparer durée et écarts. | Suivi par `joint_trajectory_controller` | |
| 10 | Implantation | Mesurer la position et l'orientation de chaque base par rapport à son rail et à sa table, et la hauteur de table. | `mfja_yaskawa_gazebo/config/yaskawa_cell.yaml` (poses reprises du jumeau) | |
| 11 | Pinces | Référence de chaque pince, course, effort, commande (sorties du contrôleur ? service ROS 2 ?), capteurs ouvert/fermé. | Non simulées ; courses relevées dans l'ancien modèle : gauche 10,8 mm par mors, droite 35 mm | |
| 12 | Outil et charge | Numéro d'outil, TCP et masse déclarés au contrôleur. | Aucun | |
| 13 | Conditions d'essai | Qui autorise les essais, zone libre, vitesse réduite, arrêt d'urgence, créneaux. | — | |

## Commandes

Afficher un robot avec une glissière par articulation (points 5 et 6) :

```bash
ros2 launch mfja_yaskawa_description show.launch.py robot_model:=hc10dt
```

Lire la position de la bride (point 6), dans un second terminal, après avoir réglé les glissières aux angles du pupitre convertis en radians :

```bash
ros2 run tf2_ros tf2_echo base tool0
```

Dans les modèles ROS-Industrial, `base` correspond au repère ROBOT du contrôleur et `tool0` à la bride (outil 0). Une inversion ou un échange de X et Y signale un décalage de 90° sur l'articulation 1.

Enregistrer une trajectoire (point 9) :

```bash
ros2 bag record -o essai_yaskawa_reel /yaskawa_LEFT/joint_states
```
