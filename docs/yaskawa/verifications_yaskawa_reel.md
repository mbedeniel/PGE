# Yaskawa réels : points à vérifier

- **Objet :** confirmer que la simulation (`yaskawa/`, paquets `mfja_yaskawa_*`) correspond aux deux Yaskawa réels de la salle 315, et terminer ce qui manque (pinces).
- **Avec :** Diane BURY (MFJA). Séance prévue le lundi 23/11/2026 à 14 h.
- **Version :** 19/11/2026, après la réponse de Mme Bury et la lecture du dépôt [mfja_ros2_yaskawa](https://github.com/aip-primeca-occitanie/mfja_ros2_yaskawa).

## Ce qui est connu au 19/11

- Pilote : MotoROS2 0.2.1 (commit `41ac074`) sur les contrôleurs, agent micro-ROS (docker) sur le PC de la salle. Clé du pupitre en « remote », portes de sécurité fermées.
- Interface, dans `/yaskawa_LEFT/` (HC10DT) et `/yaskawa_RIGHT/` (HC10) : action `follow_joint_trajectory`, `joint_states`, `robot_status`, services `start_traj_mode`, `stop_traj_mode`, `reset_error`, `read_single_io`, `write_single_io`. Trajectoires de 200 points au plus.
- Descriptions des bras : paquet `motoman_hc10_support` de Yaskawa ([motoman_ros2_support_packages](https://github.com/Yaskawa-Global/motoman_ros2_support_packages)) ; articulations `joint_1` … `joint_6` ; pas de décalage de 90° sur l'articulation 1 du HC10DT. La simulation utilise désormais exactement ces modèles.
- Pinces : pas encore commandées par ROS 2. Une tâche de fond sur chaque contrôleur lance l'initialisation, l'ouverture ou la fermeture quand OT#100, OT#101 ou OT#102 est activée (boutons du pupitre).

## Points à vérifier

| N° | Point | Comment le vérifier | État au 19/11 | Résultat |
|---|---|---|---|---|
| 1 | Configuration MotoROS2 de chaque contrôleur | Récupérer `motoros2_config.yaml` des deux robots : espace de noms, noms des articulations, `publish_tf`, `namespace_tf`, `tf_frame_prefix`. Version de `motoros2_interfaces` sur le PC de la salle. | Espaces de noms et noms des articulations déduits du dépôt, fichiers non vus | |
| 2 | Pince : adresses | Au pupitre, adresse logique de OT#100, OT#101, OT#102. | Calcul : 10133, 10134, 10135 (numérotation Yaskawa) | |
| 3 | Pince : commande par ROS 2 | Robot à l'arrêt, pince vide, une personne à l'arrêt d'urgence. Lire puis écrire OT#101 avec `write_single_io` (commandes ci-dessous) et observer. Faut-il une impulsion ou un niveau ? Qui remet la sortie à 0 ? Délai d'exécution ? | À tester | |
| 4 | Pince : état | Existe-t-il une entrée qui indique pince ouverte, fermée, initialisée, en défaut ? Adresse ? | Inconnu | |
| 5 | Pince : chemin de référence | Le script d'étudiants `supervision_IHM.py` commande une pince Zimmer par une passerelle IO-Link Turck (`10.100.12.115`, Modbus TCP). Quel chemin retenir : sorties du contrôleur ou passerelle ? | À décider | |
| 6 | Zéro de l'articulation 1 du HC10DT | Articulation 1 à 0 : noter la direction du bras par rapport à la table, comparer avec `show.launch.py robot_model:=hc10dt`. | Le modèle ROS 2 n'a pas de décalage ; vérification visuelle | |
| 7 | Géométrie (3 ou 4 poses par robot) | Au pupitre : angles des 6 articulations (°) et position de la bride en coordonnées ROBOT, outil 0. Mêmes angles en simulation, lire `base` → `flange`. Écart attendu < 5 mm. | À faire | |
| 8 | Limites et vitesses | Limites logicielles du contrôleur et vitesses maximales, en particulier en mode collaboratif. | Modèle : HC10 U −5°…355°, HC10DT U −95°…265°, autres ±180° ; 130, 130, 180, 180, 250, 250 °/s | |
| 9 | Même trajectoire réel / simulation | Lancer `execute_trajectory.py` (mode réel) sur le robot puis sur la simulation, avec le même fichier ; enregistrer `joint_states` (`ros2 bag record`) ; comparer durée et écarts. | La simulation accepte désormais les mêmes commandes | |
| 10 | Implantation | Position et orientation de chaque base par rapport à son rail et à sa table, hauteur de table. | Poses reprises du jumeau | |
| 11 | Outil et charge | Numéro d'outil, TCP et masse déclarés au contrôleur. | Inconnu | |
| 12 | Erreurs en fin de trajectoire | Codes -500302 et -100303 parfois renvoyés alors que le mouvement est correct : cause connue ? | Signalé dans le README du dépôt | |

## Commandes

Afficher un robot avec une glissière par articulation (points 6 et 7) :

```bash
ros2 launch mfja_yaskawa_description show.launch.py robot_model:=hc10dt
ros2 run tf2_ros tf2_echo base flange
```

Dans les modèles Yaskawa, `base` correspond au repère ROBOT du contrôleur et `flange` à la bride (outil 0).

Pince (point 3), sur le robot gauche ; adresses à confirmer au point 2 :

```bash
ros2 service call /yaskawa_LEFT/read_single_io motoros2_interfaces/srv/ReadSingleIO "{address: 10134}"
ros2 service call /yaskawa_LEFT/write_single_io motoros2_interfaces/srv/WriteSingleIO "{address: 10134, value: 1}"
ros2 service call /yaskawa_LEFT/write_single_io motoros2_interfaces/srv/WriteSingleIO "{address: 10134, value: 0}"
```

Enregistrer une trajectoire (point 9) :

```bash
ros2 bag record -o essai_yaskawa_reel /yaskawa_LEFT/joint_states /yaskawa_LEFT/robot_status
```
