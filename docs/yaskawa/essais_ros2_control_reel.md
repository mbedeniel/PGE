# Essais : Yaskawa réels sur ros2_control (motoros2_hw_interfaces)

Objectif : commander les bras réels avec le même `joint_trajectory_controller` qu'en simulation, au moyen
de l'interface matérielle [`motoros2_hw_interfaces`](https://github.com/b-robotized/motoros2_hw_interfaces),
qui s'appuie sur le mode « point queue » de MotoROS2 (rien à installer sur les contrôleurs). Choix validé par
Mme Bury et M. Stasse le 20/11/2026 ; il lève aussi la limite de 200 points par trajectoire.

Procédure complète : `yaskawa/mfja_yaskawa/README.md`, section « Real robots on ros2_control ».

## Installation sur le PC des robots (MFJA, salle 315)

- Machine virtuelle VirtualBox « YaskawaROS2 » (Ubuntu 24.04, ROS 2 Jazzy) sur un hôte Windows 11.
  Carte `enp0s3` : réseau de la salle, adresse fixe 10.100.12.171/24 (adresse de l'agent micro-ROS configurée
  dans MotoROS2 sur les deux contrôleurs). Carte `enp0s8` : NAT, accès internet.
- Espace de travail séparé `~/pge_ws`, construit à partir de `pge_real_robots.repos`, dans un shell
  `bash --norc` qui ne charge que `/opt/ros/jazzy` : l'espace de travail existant du PC n'est ni modifié
  ni utilisé.
- Agent micro-ROS : celui du PC (`runmicrorosdocker`), inchangé.

Incident réseau corrigé le 24/11/2026 : la connexion de `enp0s3` demandait aussi une adresse par DHCP. Sans
serveur DHCP sur le réseau de la salle, l'activation échouait toutes les 45 s et l'adresse 10.100.12.171
disparaissait : les robots perdaient l'agent par intermittence. La connexion est passée en adressage manuel.

## Résultats du 24/11/2026, bras droit (HC10, `yaskawa_RIGHT`)

| Étape | Résultat |
|---|---|
| a. Script `move_joint.py` sur le bras simulé (`ROS_DOMAIN_ID=42`) | Réussi |
| b. Lancement réel, `arm_controller` inactif | Interface matérielle active (asynchrone, 50 Hz), positions lues identiques à celles de MotoROS2, services `start_point_queue_mode` et `queue_traj_point` présents |
| c. `start_point_queue_mode` | READY (1), servos sous tension |
| d. Activation de `arm_controller` | Le bras ne bouge pas ; écart consigne/mesure ≈ 1e-5 rad |
| e. Premier mouvement (articulation 6, 0,1 rad aller-retour) | À faire |

Points envoyés à MotoROS2 en 61 s de maintien : 3050, dont 87 en échec (2,85 %). Tous les échecs sont des
réponses arrivées après le délai de 30 ms ; MotoROS2 n'a refusé aucun point. Aller-retour moyen d'un appel :
16,9 ms. La fréquence de 50 Hz est donc adaptée ; 100 Hz échouerait le plus souvent.

Bras gauche (HC10DT, 10.100.12.134) : contrôleur injoignable ce jour-là.

## Suite

1. Étape e, puis les autres articulations par petits angles, une trajectoire de plus de 200 points
   (`--points 300`) et la reprise après un arrêt d'urgence.
2. Les mêmes essais sur le bras gauche.
3. Suivre le taux d'échec des appels pendant les mouvements.
4. Simulation au plus près du réel : mêmes interfaces de commande (position + vitesse) qu'en réel, et
   imitation du mode « point queue » dans `mfja_yaskawa_motoros2_sim`.
5. Pince : interface `GPIOSystem` du même paquet (sorties OT#100 à OT#102).
