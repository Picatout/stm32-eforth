### 2020-12-01

* Il y a un problème avec la directive **.align expr1,expr2** de l'assembleur **arm-none-eabi-as**. **expr2** est ignorée de sorte que le remplissage ne se fait pas. Ce problème empèche les mot **FIND** et **SAME?** de fonctionner correctement
car la comparaison se faisait sur les mots de 4 octets avec un remplissage à zéro. J'ai donc du modifier les mots en question pour faire une comparaison octets par octets. Je précise que le problème est le même avec les directives **.balign** et **p2align**. 

### 2020-11-21

* Je suis bloqué. La carte **STM32G431-NUCLEO-32** utilise une nouvelle version de **STLINK-V3-E** qui ne fonctionne pas correctement avec les outils **STLINK-TOOLS**. J'ai téléchargé et installé la dernière version d'Attolic trueStudio espérant pouvoir travaillé avec cet IDE mais les MCU **STM32G4xx** ne sont même pas supporté par cette version. Je vais donc changer de carte et cibler la carte **blue-pill** à la place.  

### 2020-11-20

* remplissage de la table des vecteurs d'interruptions
* configuration sysclock  source HSE cristal 24Mhz. sysclock=150 Mhz
* GPIOB:8 configuré pour user LED.


