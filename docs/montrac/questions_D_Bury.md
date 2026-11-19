# Montrac : questions pour Diane Bury (MFJA)

- **Objet :** réunir les informations nécessaires pour piloter le Montrac réel depuis ROS 2 et pour en faire une simulation fidèle.
- **Lot :** L4 Intralogistique (Montrac), responsable Niel MBEDE
- **Interlocutrice :** Diane BURY, MFJA
- **Version :** 19/11/2026 (pistes ajoutées après la réponse de Mme Bury du 19/11)
- **Entretien prévu :** lundi 23/11/2026, 14 h

## Contexte

Plan décidé avec O. Stasse le 13/11/2026 :

1. écrire un programme simple, sans ROS, qui accède à une moitié du Montrac ;
2. en faire une interface matérielle ros2_control (`hardware_interface::SystemInterface`) ;
3. créer un paquet `montrac_description` dont l'argument xacro `gz_sim` charge soit l'interface matérielle réelle, soit le plugin Gazebo. Ce paquet remplacera le contrôleur cinématique actuel.

La simulation devra donc offrir exactement les commandes et les retours d'état du système réel. Le jumeau actuel (dépôt `mfja_3rd_floor_gz`, branche `INTERNSHIP-ALI-2026`) repose sur des hypothèses qui n'ont pas été vérifiées sur le matériel (section 12). Cette fiche sert à les confirmer ou à les corriger.

**Utilisation :** si le temps est court, commencer par la section 1. Les réponses sont notées dans la colonne « Réponse », puis la fiche complétée est versionnée.

## Pistes au 19/11/2026 (à confirmer le 23/11)

Mme Bury n'a pas encore répondu sur le Montrac ; elle propose d'en parler le 23/11. Les pistes ci-dessous viennent du dépôt [mfja_ros2_yaskawa](https://github.com/aip-primeca-occitanie/mfja_ros2_yaskawa) (script `supervision_IHM.py`, interface d'étudiants GEII marquée « non testée ») et de son message. Elle rappelle que les informations, même officielles, peuvent être incomplètes.

| Piste | Source | Lien avec les questions |
|---|---|---|
| Un « automate maître » répond en **Modbus TCP** à l'adresse `10.100.12.121`, port 502 ; le script lit une bobine (coil 0) comme ordre de départ et écrit une bobine (coil 1) pour signaler la fin de cycle. Est-ce l'automate qui pilote le Montrac ? | `supervision_IHM.py` | 1.1, 1.2, 3.1, 3.3, 3.4 |
| Une **passerelle IO-Link Turck** répond en Modbus TCP à `10.100.12.115` (registres 2049 en écriture, 1 en lecture) ; elle pilote une pince Zimmer. Sert-elle aussi au Montrac (modules IO-Link) ? | `supervision_IHM.py` | 1.2, 8.2 |
| Le réseau des équipements de la salle 315 semble être `10.100.12.x` (Yaskawa `.133` et `.134`, passerelle `.115`, automate `.121`). Masque et plan d'adressage à confirmer. | Dépôts `ros_yaskawa_hc10` et `mfja_ros2_yaskawa` | 3.5 |
| La configuration réseau locale est décrite sur le Moodle de la plateforme (wiki salle 315). Accès accordé par Mme Bury après une première connexion de chaque étudiant. | Message du 19/11, README de `mfja_ros2_yaskawa` | Section 2 |

## 1. Questions prioritaires

Les réponses à 1.3 et 1.4 décident de l'interface ros2_control. Le jumeau actuel commande chaque navette individuellement et ses capteurs indiquent quelle navette passe. Si le système réel ne le permet pas, la simulation devra être modifiée en conséquence.

| N° | Question | Réponse |
|---|---|---|
| 1.1 | Qu'est-ce qui pilote le Montrac aujourd'hui : un automate (marque, modèle, CPU) ? Un automate par cellule ou un seul pour les deux ? Y a-t-il un pupitre ou une IHM ? | Piste : « automate maître » en `10.100.12.121` (voir pistes) |
| 1.2 | Par quel moyen un PC peut-il lire les capteurs et commander les actionneurs : Ethernet (Modbus TCP, OPC UA, S7, Profinet), maître IO-Link, entrées/sorties TOR libres, autre ? Lequel est autorisé à la MFJA ? | Piste : Modbus TCP, déjà utilisé par un script d'étudiants vers l'automate maître |
| 1.3 | Peut-on arrêter ou redémarrer une navette individuellement (commande embarquée, module infrarouge, alimentation par tronçon) ? Ou seulement avec les stoppeurs, à des points fixes ? | |
| 1.4 | Les capteurs identifient-ils la navette (RFID, code, couleur) ou détectent-ils seulement une présence ? | |
| 1.5 | Un programme automate est-il en service ? Peut-on lire et écrire ses variables sans le modifier, ou faut-il lui ajouter un bloc d'échange ? Qui a le droit de le modifier, et avec quel logiciel ? | |
| 1.6 | Que couvre « une moitié du Montrac » pour la première étape : une cellule entière (gauche ou droite) ou une partie de cellule ? Laquelle est la plus disponible ? | |
| 1.7 | Conditions des essais : présence d'une personne habilitée, créneaux, consignes de sécurité, remise en état avant les TP ? | |

## 2. Documents à récupérer

- [ ] Projet automate (export du programme), table des mnémoniques et table des entrées/sorties
- [ ] Schémas électriques de l'armoire et du câblage des cellules
- [ ] Documentation constructeur (montratec) : navettes, aiguillages, stoppeurs, unités d'indexation, capteurs, modules de commande éventuels
- [ ] Plan coté ou CAO des deux circuits, dont le fichier SolidWorks source utilisé pour le jumeau (avec sa version)
- [ ] Plan d'adressage réseau de la cellule : adresses IP de l'automate et de l'IHM, sous-réseau
- [ ] Sujets et corrigés des TP qui utilisent le Montrac
- [ ] Analyse de risques, consignes de sécurité et notice de mise en route de la cellule
- [ ] Traces de tout essai antérieur de connexion d'un PC (serveur OPC UA, Node-RED, script, supervision)
- [ ] Table des adresses Modbus de l'automate maître (`10.100.12.121`) et rôle de ses bobines et registres
- [ ] Accès au Moodle de la plateforme (wiki salle 315) pour toute l'équipe

## 3. Automate et communication

| N° | Question | Réponse |
|---|---|---|
| 3.1 | Référence exacte de l'automate (modèle, CPU, version de firmware) et de ses modules d'E/S ou de communication. | |
| 3.2 | Logiciel et version nécessaires pour ouvrir le programme (TIA Portal, EcoStruxure, Codesys…). Une licence est-elle disponible pour l'équipe ? | |
| 3.3 | Un serveur OPC UA ou Modbus TCP est-il déjà actif sur l'automate ? Sinon, peut-on l'activer ? | |
| 3.4 | Quelles variables contiennent l'état des capteurs, des aiguillages et des stoppeurs ? Lesquelles peut-on écrire pour commander ? | |
| 3.5 | Le réseau de la cellule est-il isolé de celui de l'université ? Peut-on y brancher un PC portable, et sur quel port ? | Piste : réseau `10.100.12.x` ; configuration sur le Moodle (wiki salle 315) |
| 3.6 | Temps de cycle de l'automate, et fréquence d'échange raisonnable avec un PC (10, 50 ou 100 Hz ?). | |
| 3.7 | Si l'IHM locale et le PC donnent des ordres contradictoires, lequel a priorité ? Existe-t-il un mode « commande externe » ? | |
| 3.8 | Existe-t-il un automate de rechange, un simulateur (type PLCSIM) ou un banc d'E/S pour développer sans immobiliser la cellule ? | |

## 4. Modes de marche et démarrage

| N° | Question | Réponse |
|---|---|---|
| 4.1 | Séquence complète de mise en service : alimentation, air comprimé, réarmement, choix du mode. | |
| 4.2 | Modes disponibles (manuel, automatique, pas à pas, maintenance) et passage de l'un à l'autre. | |
| 4.3 | État des actionneurs à la mise sous tension : position par défaut des aiguillages, stoppeurs levés ou baissés. | |
| 4.4 | Que fait l'automate s'il perd la liaison avec le PC ? Peut-on ajouter un chien de garde qui met la cellule en état sûr (par exemple tous les stoppeurs levés) ? | |
| 4.5 | Séquence d'arrêt normale et état dans lequel la cellule doit être laissée pour les TP. | |

## 5. Navettes

| N° | Question | Réponse |
|---|---|---|
| 5.1 | Nombre de navettes réellement disponibles sur chaque cellule (le jumeau en prévoit jusqu'à 4 par rail). | |
| 5.2 | Modèle de navette, génération du matériel et mode d'alimentation (rail alimenté, autre). | |
| 5.3 | Dimensions (longueur, largeur, hauteur), masse, charge utile maximale, dimensions et fixation du plateau. Un cube de 5 cm en PLA peut-il être posé sans support ? | |
| 5.4 | Vitesse nominale, réglage possible, accélération et distance d'arrêt (le jumeau suppose 0,2 m/s constants). | |
| 5.5 | Le sens de circulation est-il unique ? Lequel, vu de dessus ? | |
| 5.6 | Les navettes ont-elles un anticollision embarqué ? Quelle distance reste entre deux navettes accumulées à l'arrêt ? | |
| 5.7 | Une navette roule-t-elle dès qu'elle est alimentée, ou sur ordre ? Repart-elle seule quand un stoppeur s'efface ? | |
| 5.8 | Peut-on ajouter ou retirer une navette en fonctionnement ? Selon quelle procédure ? | |
| 5.9 | Comportement des navettes à l'arrêt d'urgence ou à la coupure d'alimentation. | |
| 5.10 | Les navettes sont-elles numérotées ou repérables visuellement (étiquette, couleur) ? | |

## 6. Aiguillages (A1 à A4)

| N° | Question | Réponse |
|---|---|---|
| 6.1 | Les noms A1 à A4 du jumeau correspondent-ils au marquage réel de chaque cellule ? Où se trouve chaque aiguillage sur le plan ? | |
| 6.2 | Type d'aiguillage (plateau rotatif ?) et nombre de positions réellement utilisées. Le modèle 3D en prévoit 3, le jumeau n'en utilise que 2 (extérieur, intérieur). | |
| 6.3 | Actionneur (pneumatique, électrique), signal de commande (une ou deux sorties) et retour de position (capteurs de fin de course ?). | |
| 6.4 | Temps de basculement (le jumeau suppose 0,3 s). | |
| 6.5 | Les aiguillages de convergence (A2, A4) sont-ils motorisés ? Que se passe-t-il si une navette arrive sur un aiguillage mal positionné : blocage, déraillement, interdiction par l'automate ? | |
| 6.6 | L'automate interdit-il de basculer un aiguillage quand une navette est dessus ou proche ? Quel capteur sert à ce verrouillage (le jumeau bloque à moins de 0,35 m) ? | |

## 7. Stoppeurs et postes

| N° | Question | Réponse |
|---|---|---|
| 7.1 | Nombre et position des stoppeurs de chaque cellule. Le jumeau en place 4 par rail, un avant chaque aiguillage ; ceux de A2 et A4 agissent sur les deux branches qui convergent. Est-ce exact ? | |
| 7.2 | Y a-t-il un stoppeur à chaque poste de travail ? Dans le jumeau, l'arrêt aux postes est fait par le logiciel, sans stoppeur. | |
| 7.3 | Pour chaque stoppeur : actionneur, signal de commande, état de repos si l'air ou l'alimentation est coupé, capteur de position, temps de manœuvre (le jumeau suppose 0,1 s). | |
| 7.4 | Comment une navette s'arrête-t-elle contre un stoppeur : moteur coupé, patinage, came ? | |
| 7.5 | Combien de postes par cellule, et où ? Le jumeau en prévoit 4, en deux paires d'indexation sur les branches extérieures. Quel robot dessert quels postes ? | |
| 7.6 | Les postes ont-ils une unité d'indexation (lève-palette, pion de centrage) ? Avec quels actionneurs et capteurs ? Quelle précision de positionnement de la navette au poste ? | |

## 8. Capteurs

| N° | Question | Réponse |
|---|---|---|
| 8.1 | Liste réelle des capteurs et de leurs noms. Le jumeau utilise DZI1 à DZI4 (postes), DA1 à DA4 (entrée d'aiguillage), DA1E à DA4E et DA1I à DA4I (branches), et un capteur 0,10 m avant chaque stoppeur. Que signifient exactement « DZI » et « DA » ? | |
| 8.2 | Technologie de chaque capteur (inductif, optique, magnétique), type de signal (TOR PNP/NPN, IO-Link) et objet détecté (navette, plateau, charge). | |
| 8.3 | Position exacte de chaque capteur (cote sur plan ou mesure sur place). | |
| 8.4 | Existe-t-il des capteurs non modélisés : présence de charge, position des aiguillages et des stoppeurs, pression d'air, état de l'arrêt d'urgence, carter ouvert ? | |
| 8.5 | Temps de réponse ou filtrage dans l'automate (pour reproduire les fronts en simulation). | |

## 9. Sécurité

| N° | Question | Réponse |
|---|---|---|
| 9.1 | Chaîne d'arrêt d'urgence : où sont les boutons, que coupe-t-elle (alimentation des rails, air), comment se réarme-t-elle ? L'automate peut-il lire son état pour le transmettre à ROS ? | |
| 9.2 | Protections en place (carters, barrières) et règles de co-activité avec les robots voisins. Existe-t-il un verrouillage entre les robots et le Montrac ? | |
| 9.3 | Quelles protections l'automate assure-t-il déjà (verrouillages, anticollision) que nos commandes ne doivent jamais contourner ? | |
| 9.4 | Conditions pour un premier essai en lecture seule (aucune commande envoyée), puis pour un premier essai de commande d'un stoppeur ou d'un aiguillage. | |

## 10. Géométrie et implantation

| N° | Question | Réponse |
|---|---|---|
| 10.1 | De quel fichier viennent les points SolidWorks utilisés par le jumeau (276 points, 14 segments) ? Est-il à jour par rapport à l'installation ? | |
| 10.2 | Le commit du 04/09/2026 sur la branche `fix-staubli-placement` déplace les rails droits d'environ 5 cm. Ces valeurs viennent-elles d'une mesure sur place ? Faut-il les reporter sur `INTERNSHIP-ALI-2026` ? | |
| 10.3 | Hauteur du plan de rail et position de chaque cellule par rapport aux bases des robots (pour les repères TF robot–navette). | |
| 10.4 | Convention gauche/droite : en entrant dans la salle, quelle cellule est à gauche et quels robots la bordent ? Les documents divergent : le rapport de stage associe le rail droit au HC10DT et au Stäubli, alors que la configuration du jumeau et le dossier de spécifications associent le HC10DT et le KUKA au rail gauche. | |
| 10.5 | Peut-on photographier et mesurer la cellule (armoire, plaques signalétiques, aiguillages, capteurs) ? | |

## 11. Équivalent réel des tutoriels Montrac

Le client souhaite que chaque vidéo du dossier `tutorial_videos` ait un équivalent sur le matériel réel. Cinq vidéos concernent le Montrac :

| Vidéo | Fonction simulée | Question pour le réel | Réponse |
|---|---|---|---|
| 08 | Envoyer une navette vers un poste | Peut-on amener une navette donnée à un poste donné ? Par quelle séquence de stoppeurs et d'aiguillages ? | |
| 09 | Ajouter, démarrer, arrêter, réinitialiser, retirer une navette | Lesquelles de ces opérations existent en réel ? Lesquelles demandent une intervention manuelle ? | |
| 10 | Basculer les aiguillages en sécurité | Commande et verrouillages réels des aiguillages (section 6). | |
| 11 | Lever et baisser un stoppeur | Commande réelle des stoppeurs (section 7). | |
| 12 | Lire les capteurs d'occupation | Lecture réelle des capteurs (section 8). | |

## 12. Hypothèses du jumeau à confirmer

Valeurs relevées dans `mfja_3rd_floor_gz`, branche `INTERNSHIP-ALI-2026`, commit `2321def`.

| Élément | Valeur dans le jumeau | Valeur réelle ou commentaire |
|---|---|---|
| Navettes par rail | Jusqu'à 4 (L1–L4 à gauche, R1–R4 à droite) | |
| Sens de circulation | Unique : A14 → A1 → A12 → A2 → A23 → A3 → A34 → A4 → A14 | |
| Aiguillages | 4 par rail, 2 positions (E extérieur, I intérieur) | |
| Temps de basculement d'un aiguillage | 0,3 s | |
| Stoppeurs | 4 par rail, un avant chaque aiguillage ; 0 = libre, 1 = bloqué | |
| Temps de manœuvre d'un stoppeur | 0,1 s | |
| Postes | 4 par rail, sur les branches extérieures (A12E, A34E) ; arrêt géré par le logiciel | |
| Capteurs | Binaires, et indiquent quelle navette passe | |
| Vitesse des navettes | 0,2 m/s constants, sans accélération | |
| Distance minimale entre navettes | 0,33 m (centre à centre) | |
| Zone protégée autour d'un aiguillage | 0,35 m | |
| Capacité d'une branche intérieure | 2 navettes | |
| Aiguillage de convergence mal positionné | La navette s'arrête en défaut (état `FALLING`) | |
| Arrêt d'urgence | Arrêt logiciel de toutes les navettes | |
| Charge | Boîte posée sur la navette, sans maintien | |

## 13. À relever sur place

- [ ] Photos de l'armoire, de la plaque signalétique de l'automate et de ses modules
- [ ] Photos et références des navettes, aiguillages, stoppeurs et capteurs (étiquettes)
- [ ] Chronométrer un tour complet de navette et mesurer la longueur du circuit pour en déduire la vitesse
- [ ] Filmer un basculement d'aiguillage et une manœuvre de stoppeur (temps de manœuvre)
- [ ] Mesurer l'espacement de deux navettes accumulées derrière un stoppeur
- [ ] Croquis coté de chaque cellule avec la position des capteurs, des stoppeurs et des postes

## Après l'entretien

- Compléter cette fiche et la versionner.
- Rédiger `docs/montrac/HARDWARE.md` : liaison, messages, sécurité, séquence de démarrage.
- Consigner le choix du protocole de communication dans le journal des décisions.
- Mettre à jour la configuration du jumeau avec les valeurs réelles.
