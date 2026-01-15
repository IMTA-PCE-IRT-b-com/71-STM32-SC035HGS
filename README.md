# 71-STM32-SC035HGS

Ce projet regroupe l’ensemble de la partie visuelle développée pour l’équipe 71.  
Il contient les codes nécessaires à l’utilisation du module caméra **SC035HGS** avec un microcontrôleur **STM32 (NUCLEO-H755ZI-Q)**.

Le projet inclut également les éléments relatifs à l’adaptateur assurant l’interface entre ces deux composants :
- la documentation associée,
- les schémas du PCB réalisés sous **KiCad**, disponibles dans ce dépôt.

---

## Code d’exemple

Un prototype de code permettant la communication **I²C** avec la caméra et l’**IMU** est fourni dans le dossier `STM/`.

Ce code initialise les différents périphériques utilisés :
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
