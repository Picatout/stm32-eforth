### 2020-12-03

* Mis dans un bloc **.if** les mots **SEE** et **DECOMPILE**, pas très utile.

* Ajout du mot **0=**. 

* Renommer le mot **ERASE_SECTOR** en **ERASE_PAGE**  le stm32f103 n'a pas de secteurs. La mémore flash est organisée 
en page de 1024 octets. On ne peut qu'effacer une page et écrire que des mots de 16 bits un à la fois. 

### 2020-12-02

#### session 2

* J'ai opté pour la solution qui consiste à copier le système Forth en mémoire RAM parce que c'était le plus simple.
  Il reste 11468 octets RAM libres.

* Tradionnellement les nombres hexadécimal étaient indiqués lors de la saisie par un signe **$** au début. Hors dans le fichier original **stm32eForth720.s** Il a été remplacé par le caractère **_**. J'ai remis le **$** traditionnel.

* La base numérique par défaut était **16**  je l'ai mise à **10**,  voir **.equ BASEE, 10**.

#### session 1

* Je ne connais pas le jeux d'instructions **thumb v2**. J'apprends donc en travaillant sur ce projet et ça réserve des surprises.  Ainsi le modèle *subroutine threaded* du **stm32eforth720** utilise l'instruction **BL** pour appeller les sous-routines composants un mot. Hors l'instruction **BL** est **pc relative** et le déplacement est limité à *21 bits signés.*  Au départ j'avais choisi d'exécuter les mots du système eForth à partir de la mémoire flash et de définir les mots utilisateurs dans la mémorie RAM. Le problème est que la distance est trop grandes entre la mémoire RAM qui débute à l'adresse **0x20000000** et la mémoire flash qui débute elle à **0x8000000** ou à **0** pour son alias pour faire des appel *pc relative* entre les 2 zones.  Il  ne me reste que 3 options:

    * revenir au modèle initial, qui consiste à copier le système Forth en RAM et tout exécuter en mémoire RAM. Ce système a été conçu pour une carte **STM32F407VB explorer** qui possède 196Ko de mémoire RAM alors que la blue-pill n'en possède que 20Ko. Le système prendrait donc plus de 50% de la mémore RAM. 

    * Compiler les nouvelles définitions directement dans la mémoire FLASH. Donc les nouvelles définitison s'exécuteraient aussi mémoire flash.  

    * Modifier le modèle d'exécution pour un modèle indirect qui utiliserait l'instruction **BLX** au lieu d **BL**. L'appel indirect par registre a l'avantage de donner un accès à la plage complète d'adressage de 32 bits.

### 2020-12-01

* Il y a un problème avec la directive **.align expr1,expr2** de l'assembleur **arm-none-eabi-as**. **expr2** est ignorée de sorte que le remplissage ne se fait pas. Ce problème empèche les mot **FIND** et **SAME?** de fonctionner correctement
car la comparaison se faisait sur les mots de 4 octets avec un remplissage à zéro. J'ai donc du modifier les mots en question pour faire une comparaison octets par octets. Je précise que le problème est le même avec les directives **.balign** et **p2align**. 

### 2020-11-21

* Je suis bloqué. La carte **STM32G431-NUCLEO-32** utilise une nouvelle version de **STLINK-V3-E** qui ne fonctionne pas correctement avec les outils **STLINK-TOOLS**. J'ai téléchargé et installé la dernière version d'Attolic trueStudio espérant pouvoir travaillé avec cet IDE mais les MCU **STM32G4xx** ne sont même pas supporté par cette version. Je vais donc changer de carte et cibler la carte **blue-pill** à la place.  

### 2020-11-20

* remplissage de la table des vecteurs d'interruptions
* configuration sysclock  source HSE cristal 24Mhz. sysclock=150 Mhz
* GPIOB:8 configuré pour user LED.


