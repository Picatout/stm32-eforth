### 2020-12-07

* L'adaption de stm32eForth selon le modèle original de C.H. Thing étant complété, j'entrepris un autre modèle qui lui sera exécuté à partir de la mémoire FLASH. Le fichier source s'appelle [stm32eforth-fl.s](stm32eforth-fl.s)

<hr>

* Corrigé bogue dans **SAVE_IMG** et **LOAD_IMG**.

* **TURNKEY** &nbsp;( -- ) "mot", Initialise la variable **'BOOT** avec l'adresse d'exécution du *mot* passé en argument, ensuite sauvegarde l'image dans la fente **0**. Le mot **COLD** vérifie s'il y une image dans la fente **0** et si c'est le cas la charge automatiquement et le vecteur **'BOOT** est exécuté. 

* **FORGET** &nbsp;( -- ) "mot", Oublie le *mot* ainsi que tous ceux qui ont été définis après celui-ci. 

### 2020-12-06

* Renommé **USER_IMG** EN **IMAGE0**, Retourne l'adresse de la première image. 

* Corrigé bogue dans **ADR>PG**.

<hr>
* Modifié système pour permettre la sauvegarde de plusieurs images. 
  * **IMG_SIZE** &nbsp;( -- n ), Retourne le nombre de pages FLASH requis pour sauvegarder la totalité de la mémoire RAM disponible. 

  * **IMG?** &nbsp;( n -- ), Modifié, il faut maintenant indiquer le numéro de l'image. 

  * **IMG_ADR** &nbsp;( n -- a ), Retourne l'adresse *a* de l'image à partir de son numéro *n*.

  * **ERASE_IMG** &nbsp;( n -- ), Efface l'image numéro *n*. 

  * **LOAD_IMG** &nbsp;( n -- ), Modifié, maintenant il faut indiquer le numéro de l'image à charger.

  * **SAVE_IMG** &nbsp;( n -- ), Modifié, maintenant il faut indiquer le numéro de l'image destination. 

* Renommé le mot **PAGE** en **ADR>PG** et ajouter le mot inverse **PG>ADR** 

* Lorsqu'une compilation était avortée suite à une erreur les pointeurs **CP** et **LAST** n'était pas réinitialisé.  

* **USER_BEGIN**, **USER_END** et **USER_IMG** ne sont plus des variables système mais des constantes système.

### 2020-12-05

* Mots ajoutés pour la sauvegarde en mémoire flash

  * **SAVE_IMG** &nbsp;( -- ), Sauvegarde les définitiations créées par l'utilisateur en mémoire FLASH.

  * **LOAD_IMG** &nbsp;( -- ), Charge en mémoire RAM l'image sauvegardée par **SAVE_IMG**. 

  * **IMG?** &nbsp;( -- f ), Vérifie s'il y a une image de sauvegardée en mémoire FLASH. 

  * **FLH_WR** &nbsp;( src dest u -- dest+4u ), écriture de *u* mots de 32 bits dans la mémoire flash. *src* est l'adresse source de données, *dest* est l'adresse destination en flash et *u* est le nombre de mots à écrire. Retourne l'adresse suivant le dernier mot écris *dest+4u*. 

  * **I!** &nbsp;( w a -- ), Écriture d'un mot de 32 bits dans la mémoire flash. *w* est le mot à écire. *a* est l'adresse destination. 

  * **PAGE** &nbsp;( n -- a ), Retourne l'adresse absolue en mémoire flash d'un numéro de page. *n' est le numéro de page. 
  *a* est l'adresse résultante entre **0x8000000-0x8001FFFF** 

  * **ERASE_MPG** ( p n -- ) &nbsp;( -- ), Efface *n* page de mémoire flash à partir de la page *p*. 

  * **EPAGE** &nbsp;( a -- ) Efface la page flash contenant l'adresse *a*. 

  * **UNLOCK** &nbsp;( f -- ) Déverrouille la mémoire flash pour l'écriture. *f* est un indicateur booléen. Pour *f==faux*, i.e. **0**, le verroue est appliqué. Pour f==vraie, i.e **~0**, le verroue est enlevée.  

### 2020-12-04

#### Sauvegarde et restauration de l'image utilistateur

* Sauvegarde 
    * Effacer les pages qui seront utiliser pour la sauvegarde.
    * sauvegarder les variables systèmes. 
    * sauvegarder les définitions. 

* restauration 
    * Au démarrage le mot COLD doit vérifier s'il y a une image de sauvegardée. Et si c'est le cas il doit appeller 
      le mot qui va faire la copie en RAM. 
    * copier les variables système en RAM.
    * copier les définitiions en RAM. 
    * démarrer l'application. 

<hr>

* Ajout des variables systèmes:
  * **USER_IMG**, cette variable système contient l'adresse en mémoire flash ou est sauvergardé l'image RAM.
  * **USER_BEGIN**, cette variable contient l'adrese début de mémoire RAM utilistateur.
  * **USER_END**, cette variable contient l'adresse find de la mémoire RAM utilisateur.

* Ajout de la variable système **SEED** et du mot **RANDOM** pour la génération de nombres pseudo aléatoires.

* Débogué  mots **UNLOCK**, **I!** et **ERASE_PAGE**.

### 2020-12-13

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


