# STM32_2026_G13

## Contexte
Ce dépôt correspond à notre bureau d’études sur une plante connectée.

Le but était de réaliser un système avec un STM32 capable de mesurer certaines grandeurs (humidité du sol, luminosité) et d’agir en conséquence, notamment pour automatiser l’arrosage.

---

## Travail réalisé
On a réussi à mettre en place :
- la lecture de l’humidité du sol  
- la lecture de la luminosité  

Ces mesures fonctionnent correctement et sont exploitables dans le cadre du projet.

---

## Problème rencontré
On n’a pas pu aller jusqu’au bout du projet, notamment sur la partie asservissement.

L’idée était de piloter un moteur à l’aide d’un MOSFET commandé par le STM32.

Cependant :
- le MOSFET utilisé ne fonctionnait pas correctement  
- des tests ont été faits  
- on n’a pas pu le remplacer à temps  

Du coup, la partie commande (activation du moteur pour l’arrosage) n’a pas pu être finalisée.

---

## Capteurs utilisés / étudiés
On a travaillé avec :
- un capteur d’humidité du sol  
- un capteur de luminosité  
- un capteur à ultrasons  
- un capteur DHT22 (Temperature and Humidity Sensor Pro V1.3)

---

## Conclusion
Même si le projet n’a pas pu être terminé complètement, on a réussi à mettre en place la partie acquisition de données et à comprendre les contraintes liées au matériel.

---
