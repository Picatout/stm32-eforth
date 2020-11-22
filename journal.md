### 2020-11-21

* Je suis bloqué. La carte **STM32G431-NUCLEO-32** utilise une nouvelle version de **STLINK-V3-E** qui ne fonctionne pas correctement avec les outils **STLINK-TOOLS**. J'ai téléchargé et installé la dernière version d'Attolic trueStudio espérant pouvoir travaillé avec cet IDE mais les MCU **STM32G4xx** ne sont même pas supporté par cette version. Je vais donc changer de carte et cibler la carte **blue-pill** à la place.  

### 2020-11-20

* remplissage de la table des vecteurs d'interruptions
* configuration sysclock  source HSE cristal 24Mhz. sysclock=150 Mhz
* GPIOB:8 configuré pour user LED.


