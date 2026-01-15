# 71-STM32-SC035HGS

Ce projet regroupe l’ensemble de la partie visuelle développée pour l’équipe 71.  
Il contient les codes nécessaires à l’utilisation du module caméra **SC035HGS** avec un microcontrôleur **STM32 (NUCLEO-H755ZI-Q)**.

Le projet inclut également les éléments relatifs à l’adaptateur assurant l’interface entre ces deux composants :
- la documentation associée,
- les schémas du PCB réalisés sous **KiCad**, disponibles dans ce dépôt.

---
## Documentation

Les différentes documentations récupérées auprès de Sinoseen ont été archivées dans le dossier doc/.
Celui-ci comprend la description de produit, la datasheet de la caméra ainsi qu'un code d'initialisation fournit par le constructeur.

## Code d’exemple

Un prototype de code permettant la communication **I²C** avec la caméra et l’**IMU** est fourni dans le dossier `STM/`.

Ce code initialise les différents périphériques utilisés sur le coeur M7 :
- le **DVP** (Digital Video Port),
- **I2C1** pour la configuration de la caméra,
- **3 ports GPIO** avec interruptions externes pour tester les sorties d’horloge de la caméra  
  (redondants avec les ports DVP à des fins de débogage),
- **I2C4** pour la communication avec l’IMU.

Le code fournit :
- des fonctions de lecture et d’écriture des registres de la caméra,
- une définition des différentes adresses de registres connues.

Dans son état actuel, le projet implémente une initialisation de la caméra basée sur les valeurs de registres fournies par **Sinoseen**.

La datasheet de la caméra est disponible dans le dossier `doc/`. Celle-ci étant rédigée en chinois, elle ne documente malheureusement pas l’ensemble des registres disponibles.

Ce projet STM doit être considéré comme une **base de réflexion** et un **prototype**.  
Il est recommandé de ne pas s’appuyer aveuglément sur les choix techniques effectués, certains ayant été faits de manière arbitraire ou exploratoire.

---

## Documentation Adaptateur caméra SC031GS pour STM32

Préambule :

Cet adaptateur a été conçu comme interface entre une caméra type SC031GS et une carte STM32, mais peut être utilisé pour d’autres modèles de caméra (comme la SC035HGS) et éventuellement d’autres microcontrôleurs partageant les mêmes caractéristiques.

Cet adaptateur a pour but de fournir à la caméra les trois types d’alimentations qu’elle nécessite, ainsi que d’adapter les tensions des valeurs logiques qu’elle produit, différentes de celles d’une carte STM32.


Description :

Cet adaptateur assure fonctions principales :

1.  Adapter une tension d’alimentation fournie à 3.3 V en trois tensions distinctes 1.5 V, 1.8 V et 2.8 V.

2.  Adapter le niveau des signaux logiques en provenance de la caméra à 1.8 V en signaux logiques de 3.3 V adaptés à une carte STM32.


Il comporte :

-   1 connecteur 21 pins
-   1 connecteur 18 pins
-   1 LDO 1.5 V : LP2985AIM5-1.5/NOPB
-   1 LDO 1.8 V : LP2985-18DBVRE4
-   1 LDO 2.8 V : LP2985A-28DBVR
-   2 level-shifters bidirectionnels 8 bits : LSF0108PWR
-   4 résistances de 10 kΩ
-   2 résistances de 200 kΩ

Il existe également des emplacements prévus pour :

-   3 condensateurs de 0.01 µF
-   2 condensateurs de 0.1 µF

Ces éléments peuvent être ajoutés pour le filtrage des signaux. 


Détails des entrées/sorties :

Les pins seront numérotés de haut en bas, le haut du circuit étant repéré par l’inscription b<>com IMT Atlantique.

Connexions côté microcontrôleur :

Les connexions au microcontrôleur sont situées sur le connecteur 18 pins à droite du circuit. Tous les signaux logiques de ce côté ont pour tension haute 3.3 V.

| N° pin | Nom       | Fonction |
|--------|-----------|----------|
| 1      | STM_SDA   | Ligne de donnée de contrôle I2C, envoyée entre les deux systèmes |
| 2      | STM_SCL   | Ligne d’horloge du bus I2C imposée par la carte |
| 3      | STM_RESET | Signal logique de réinitialisation de la caméra |
| 4      | STM_VSYNC | Signal logique indiquant le début et la fin d’une trame de données |
| 5      | STM_PWDN  | Signal logique permettant de mettre en veille la caméra |
| 6      | STM_HREF  | Signal logique indiquant les périodes de disponibilité des données |
| 7      | DT_CAM_7  | Bit logique n°7 de donnée vidéo envoyé par la caméra |
| 8      | STM_MCLK  | Signal d’horloge à imposer à la caméra pour son fonctionnement interne |
| 9      | DT_CAM_6  | Bit logique n°6 de donnée vidéo envoyé par la caméra |
| 10     | DT_CAM_5  | Bit logique n°5 de donnée vidéo envoyé par la caméra |
| 11     | STM_PCLK  | Signal d’horloge de cadence de sortie des données vidéo, imposé par la caméra |
| 12     | DT_CAM_4  | Bit logique n°4 de donnée vidéo envoyé par la caméra |
| 13     | DT_CAM_0  | Bit logique n°0 de donnée vidéo envoyé par la caméra |
| 14     | DT_CAM_3  | Bit logique n°3 de donnée vidéo envoyé par la caméra |
| 15     | DT_CAM_1  | Bit logique n°1 de donnée vidéo envoyé par la caméra |
| 16     | DT_CAM_2  | Bit logique n°2 de donnée vidéo envoyé par la caméra |
| 17     | +3V3      | Alimentation 3.3V fournie par la carte |
| 18     | GND1      | La masse |


Connexions côté caméra

Les connexions à la caméra sont situées sur le connecteur 21 pins à gauche du circuit. Tous les signaux logiques de ce côté ont pour tension haute 1.8 V. A noter que la caméra possède en réalité 24 connexions, mais 2 d’entre elles ne sont pas attribués et la connexion permettant le contrôle de la LED stroboscopique n’est ici pas utilisée. Le tableau suivant associe également les numéros de pin aux numéros des entrées/sorties de la caméra, indiqués sur sa documentation.

| N° pin | N° cam | Nom | Fonction |
|--------|--------|-----|-----------|
| 1      | 02     | GND | La masse |
| 2      | 03     | SDA | Ligne de donnée de contrôle I2C, envoyée entre les deux systèmes |
| 3      | 04     | +2V8 | Alimentation 2.8 V |
| 4      | 05     | SCL | Ligne d’horloge du bus I2C imposée par la carte |
| 5      | 06     | RESET | Signal logique de réinitialisation de la caméra |
| 6      | 07     | VSYNC | Signal logique indiquant le début et la fin d’une trame de données |
| 7      | 08     | PWDN | Signal logique permettant de mettre en veille la caméra |
| 8      | 09     | HREF | Signal logique indiquant les périodes de disponibilité des données |
| 9      | 10     | +1V5 | Alimentation 1.5 V |
| 10     | 11     | +1V8 | Alimentation 1.8 V |
| 11     | 12     | Y7 | Bit logique n°7 de donnée vidéo envoyé par la caméra |
| 12     | 13     | MCLK | Signal d’horloge à imposer à la caméra pour son fonctionnement interne |
| 13     | 14     | Y6 | Bit logique n°6 de donnée vidéo envoyé par la caméra |
| 14     | 15     | GND | La masse |
| 15     | 16     | Y5 | Bit logique n°5 de donnée vidéo envoyé par la caméra |
| 16     | 17     | PCLK | Signal d’horloge de cadence de sortie des données vidéo, imposé par la caméra |
| 17     | 18     | Y4 | Bit logique n°4 de donnée vidéo envoyé par la caméra |
| 18     | 19     | Y0 | Bit logique n°0 de donnée vidéo envoyé par la caméra |
| 19     | 20     | Y3 | Bit logique n°3 de donnée vidéo envoyé par la caméra |
| 20     | 21     | Y1 | Bit logique n°1 de donnée vidéo envoyé par la caméra |
| 21     | 22     | Y2 | Bit logique n°2 de donnée vidéo envoyé par la caméra |


Utilisation :

Branchement du microcontrôleur

Côté microcontrôleur, il est impératif d’utiliser des résistances de pull-up sur toutes les entrées et sorties. Deux sont déjà incluses dans le circuit pour les connexions STM_SDA et STM_SCL.
A noter que la carte STM32 dispose d’options lui permettant de configurer elle-même ses connexions en pull-up.

Adaptabilité

La marge d’application de cet adaptateur est fortement limitée si les valeurs logiques et d’alimentation de référence du microcontrôleur utilisé sont différentes, notamment par le fait que les circuits de logique et de puissance soit lié (l’alimentation de 3.3 V fourni par le microcontrôleur étant utilisé comme référence logique mais aussi comme alimentation).
Néanmoins, il reste utilisable pour une tension d’alimentation (et donc logique) allant de 2.2 V à 5.5 V, en croisant les marges des différents composants utilisés.

