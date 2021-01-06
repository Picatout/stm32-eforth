### 2021-01-06  

#### travail sur [stm32eforth.s](stm32eforth.s)

* Ajout des mots:

  * **DEFER**&nbsp;&nbsp;( -- ) name ,  Création d'un nouveau mot *name* dont l'action sera définit ultérieurement. 

  * **DEFER!**&nbsp;&nbsp;( a -- ) name , Détermine l'action d'un mot différé *name*.  *a* est l'adresse d'exécution du code. 

  * **DEFER@**&nbsp;&nbsp;( -- a ) name, Retourne l'adresse d'exécution du mot différé *name*. 

  * **H.**&nbsp;&nbsp;( u --  ) Imprime un entier non signé en hexadécimal indépendamment de la valeur de la variable système **BASE**. 

### 2021-01-04

#### travail sur [stm32eforth.s](stm32eforth.s)

* Création de la macro **_HEADER_FL** pour créer des mots qui sont exécutés en mémoire flash. Ces mots sont appellé par un appel indirect par l'intruction machine **BLX**. 
La macro cré un code qui lui est copié en mémoire RAM. J'appelle ce code un *proxy*. 

* Ajout du mot **FCALL**&nbsp;&nbsp;( a -- ) Ce mot permet de faire un appel hors de porté d'un branchement relatif en mettant sur la pile l'adresse *a* de la routine.

* Modifier les mots dans la section **outils** pour qu'ils s'exécutent en mémoire flash. Ces mots sont construit avec la macros **_HEADER_FL**.  Cette modificaton permet de
d'économiser de la mémoire RAM. 

### 2021-01-02  

#### travail sur [stm32eforth.s](stm32eforth.s)

* Création des mots suivants:

  * **SET-IVEC**&nbsp;&nbsp;( a n -- ) Assigne l'adresse *a* au vecteur d'interruption *n*. Les interruptions du **core** porte des numéros négatif de **-1** pour **systick** à **-14** pour **NMI**.  Les interruptions externe portent des numéros positif dans l'intervalle **{0..59}**.

  * **RST-IVEC**&nbsp;&nbsp;( n -- ) Réénitialise le vecteur d'interruption *n* à sa valeur par défaut qui consiste à réinitialiser le **MCU**. 

  * **I:**&nbsp;&nbsp;(  -- a ) Débute la compilation d'une routine de service d'interruption ces routines n'ont pas de nom dans le dictionnaire leur adresse est retournée sur la pile pour être utilisée par **SET-IVEC**. 

  * **I;**&nbsp;&nbsp;( a -- a ) Termine la compilation d'une routine de service 
  d'interruption *a* est l'adresse de la routine laissé sur la pile par **I:** .

* Modification de la macro _DOLIT pour diminuer l'effort d'écriture.

* Complété la tranformation de l'entête de dictionnaire pour séparer le dictionnaire du code. Nécéssitait un champ supplémentaire de plusieurs modification au niveau du compilateur.

### 2021-01-01  

#### travail sur [stm32eforth.s](stm32eforth.s)

* Création de la macro **_HEADER** pour faciliter l'écriture des entêtes de dictionnaire. l'entête du dictionnaire contient un champ supplémentaire 
soit le *code field address* qui pointe vers le code. 

* Ajout de **NAME>CFA**;&nbsp;( na -- cfa ) Retourne l'adresse du champ *cfa  (code field address)* de l'entête du dictionaire  à partir du champ *na* 


* Modifier le script de linkage [stm32f103c8t6.ld](board/blue-pill/stm32f103c8t6.ld) pour ajouter des sections afin de séparer le code qui reste en FLASH de celui qui est copié en mémoire RAM.

* Modification pour conserver le dictionnaire en FLASH pour économiser la mémoire RAM. 

#### À faire 

* ajout des mots suivants:

    * **I:**&nbsp;&nbsp;( -- a )  Débute la création d'une routine d'interruption *a* est l'adresse de la routine. 

    * **I;**&nbsp;&nbsp;( a -- a ) Termine la compilation d'une routine d'interruption. *a* est l'adresse de la routine qui
    doit-être inscrite dans le vecteur d'interruption. 

    * **SET-IVEC**&nbsp;&nbsp;( a n -- )  Iniitalise le vecteur d'interruption *n' avec l'adresse *a*. 

    * **RST-IVEC**&nbsp;&nbsp;( n -- )  Réinitialise le vecteur d'interruption *n* avec l'adresse du *default_handler*.

    * **FCALL**&nbsp;&nbsp;( a -- )  Appelle la routine en mémoire flash, *a* est l'adresse de la routine. 

### 2020-12-31  

#### travail sur [stm32eforth.s](stm32eforth.s)

* Transfert de la table des vecteurs d'interruptions au début de la RAM (0x20000000) afin que les applications puissent la modifiée.

### 2020-12-30  

#### travail sur [stm32eforth.s](stm32eforth.s)

*  Modification au à l'outil **SEE** pour le rendre plus utile. Grosse amélioration même si le désassemblage est incomplet:
```
SEE TYPE
 20000F10  nest
 20000F14  BL  2000025C  >R
 20000F18   B806F000
 20000F1C  BL  200005F4  COUNT
 20000F20  BL  2000097C  >CHAR
 20000F24  BL  20000158  EMIT
 20000F28  BL  200001A2   {doNext}
 20000F2C   20000F1C
 20000F30  BL  20000284  DROP
 20000F34  unnest
 ok

```

#### À faire

* créer le mot **FCALL** pour réduire l'utilisation de la RAM en gardant les mots 
les moins utilisé dans la mémoire FLASH. 

### 2020-12-29 

#### travail sur [stm32eforth.s](stm32eforth.s)

* Modifié routine **CALLC** en créant **COMPILE_BLW** afin que **DODOES** puisse utilisé ce facteur commun. 

* Débogué  **DOES>** 

* Testé avec [snake.f](snake.f)  **OK**.

### 2020-12-28 

#### travail sur [stm32eforth.s](stm32eforth.s)

* Ajout d'une file pour la réception UART.

* Ajout de **DOES>** .


### 2020-12-27

#### travail sur [stm32eforth-fl.s](stm32eforth0-fl.s)

* modifié mot **RANDOM**, remplacé valeur absolue de *ABS(SEED)* par un *SEED AND 0x7fffffff*. Appliqué fonction **ABS** à l'argument pour s'assurer qu'il est positif. 

* Débogué [snake](snake.f)


### 2020-12-26

#### travail sur [stm32eforth-fl.s](stm32eforth0-fl.s)

* Modifié initialisation, création de la procédure **vm_init**.

### 2020-12-23

* travail sur snake.f   

### 2020-12-23 

#### travail sur [stm32eforth-fl](stm32eforth0-fl.s)


* Débogage code UART bas niveau.

* retravaillé  **default_handler**.

* travail sur jeu **snake.f**

### 2020-12-22

#### travail sur [stm32eforth-fl](stm32eforth0-fl.s)


* Ajouter une file pour la réception UART afin d'éviter la perte de caractères. pendant la réception de fichier avec SendFile.


### 2020-12-20

#### travail sur [stm32eforth-fl](stm32eforth0-fl.s)


* Adapté jeu snake à partir de [snake.fx](https://github.com/Picatout/ForthEx/blob/master/docs/html/snake.fx).<BR/>Pour l'utilisé sur la blue pill il faut le charger avec la commande:
```
SendFile -s/dev/ttyS0 ansi.f snake.f 
```
En supposant que le terminal branché à la blue pill utilise **/dev/ttyS0** 

### 2020-12-18 

* Corrigé bogue dans mot **'**, après étiquette **TICK1** remplacé **_BRAN ABORT** par **_ADR ABORT**.  Corrigé même erreur après étiquette **SNAME1**.   

### 2020-12-17

* Travail sur ansi.f et SendFile.c 

* Découvert bogue dans **'**. 
### 2020-12-14

* Débogué  **DOES>**. 

### 2020-12-13

* travail sur mot **DOES>**. 
<hr>
* Ajout des mots suivants:

    * **DEFER**&nbsp;&nbsp;( "name" -- ), Création d'une définition vide à laquelle sera affectée ultérieurement une fonction. *name* est le nom du nouveau mot. 
    * **DEFER!**&nbsp;&nbsp;( "name1" "name2" -- ), Affecte l'action *name1* au mot différé *name2*.
    * **DEFER@**&nbsp;&nbsp;( "name" -- a), Obtient l'adresse d'exécution du mot qui est affecté au mot différé *name*.
    
    exemple:
    ```
    blue pill stm32eForth-it v1.00
    DEFER TEST ok
    DEFER! HI TEST ok
    TEST
    blue pill stm32eForth-it v1.00
    ok
    DEFER@ TEST >NAME CR COUNT TYPE
    HI ok
    ```

* Modifié  **DEPTH** , rapportait un élément de trop.

### 2020-12-12

Poursuite du travail sur la version [stm32eforth-fl.s](stm32eforth-fl.s)

* Correction de bogue dans **DOVAR** et **DOCON** qui ne se terminait pas correctement. 

### 2020-12-11

* test de vitesse comparée: 
```
: TEST MSEC @ 1000000 FOR 1 31 LSHIFT DROP NEXT MSEC @ SWAP - . ;
```
  * version *subroutine threaded* 1154 millisecondes. 
  * version *indirect threaded* 1264 millisecondes.

Le MCU fonctionnant à la vitesse maximale de 72Mhz.
<hr>
* Modifié et testé le système de sauvegarde des images utilisateurs.

* **IMAGE0** renommé **IMG_ADR** 

* Vocabulaire des sauvegardes.
    * **IMG_ADR**&nbsp;&nbsp;( -- a ), Retourne l'adresse en mémoire FLASH où est sauvegardée l'image. 
    * **IMG_SIGN**&nbsp;&nbsp;( -- a ),Retourne l'adresse de la variable système signature. Celle-ci contient le mot **IMAG**. 
    * **IMG?**&nbsp;&nbsp;( -- f ), Retourne un indicateur booléen indiquant s'il y a une image de sauvegardée. 
    * **SAVE_IMG**&nbsp;&nbsp;( -- ), S'il y a des définitions utilisateurs dans la mémoire RAM sauvegarde cette image dans la mémoire FLASH. 
    Lorsqu'une image est sauvegardée elle est rechargée automatiquement dans la mémoire RAM au démarrage.
    * **LOAD_IMG**&nbsp;&nbsp;( -- ), Charge en mémoire RAM l'image sauvegardée en mémoire FLASH.

    * **TURNKEY**&nbsp;&nbsp;( -- ) *word*, Initialise le vecteur 'BOOT avec l'adresse d'exécution du mot *word* et sauvegarde l'image. Au démarrage ce mot sera exécuté comme application. **TURNKEY** implique qu'il y a des définitions utilisateurs dans la mémoire RAM. 

### 2020-12-10

* Travaillé sur le nouvelle version [stm32eforth-fl.s](stm32eforth-fl.s), le travail est presque complété ne rest qu'à faire des tests supplémentaires. Pour construire et programmer cette version sur la carte il faut faire:
```  
  make build_fl && make flash_fl
```

### 2020-12-08

* Dans la version [stm32eforth-fl.s](stm32eforth-fl.s) le coeur du système Forth demeure en mémoire FLASH et est exécuté à partir de là. Les mots utilisateurs définis avec **':'** demeurent en mémoire RAM et sont exéctés à partir de là. 
 
* Je dois changer le modèle d'exécution pour passer du modèle *subroutine threaded* au modèle *inderect threaded* du à une limitation du jeu d'instruction *thumb* du CPU ARM-V7M.  l'instruction machine **BL target** est un adressage relatif sur 25 bits signés, donc permet des sauts relatifs dans la plage {-16777216 à 16777214}. Ce qui ne permet pas de faire des sauts entre la mémoire RAM et la mémoire FLASH.  Avec le modèle *indirect threaded* les mot définis par l'utilsateur avec **':'**  seront une liste d'adresses absolues. L'interpréteur interne lit cette liste en chargeant chaque adresse dans un registre pour utiliser une instruction **BLX address** qui permet d'atteindre l'entièreté de l'espace d'adressage de 32 bits. L'inconvénient est un ralentissement de l'exécution. 
Dans cette version le registre **R0** est utilisé comme **IP** i.e. *Instruction Pointer* de la machine virtuelle. Le registre **R4** est utilisé comme **WP** i.e. *Working Register*. Le code de la machine virtuelle va ressemblé à ceci.
```
// inteprète interne du code FORTH 
NEXT:
  LDR WP,[IP],#4
  BLX WP 
  B NEXT 
```


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


