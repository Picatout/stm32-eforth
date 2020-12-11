# stm32-eforth

Adaptation de eforth pour sur la carte **blue-pill**. Je vais travaillé à partir du fichier original de C.H. Ting [stm32eForth720](http://forth.org/OffeteStore/2165_stm32eForth720.zip).  Le code source a été écris en assembleur pour les outils de développements µVision 5. Je vais utilisé **gcc-arm-none-eabi** il faudra donc une réécriture du fichier source original. 

## 2020-12-10

Il y a maintenant 2 versions du eForth.  

* **subroutine threaded** Le code système Forth est copié en mémoire RAM et s'exécute à partir de là. Ça consomme 50% de la mémoire RAM. Pour construire et programmer ce système sur la blue pill il faut faire:
```
make build && make flash 
```

* **indirect threaded** Le système Forth demeure et s'exécute en mémoire FLASH. Seul les définitions de l'utilisateur sont en mémoire RAM et s'éxécute à partir de là. l'aventage est le gain en mémoire RAM. l'inconvient est qu'un système **indirect threaded** est plus lent qu'un système **subroutine threaded**. 
Pour construite cette version il faut faire:
```
make build_fl && make flash_fl 
```


## prérequis

Je travaille sur Ubuntu 20.04 LTS les logiciles suivants doivent-être installés. 

    sudo apt install gcc-arm-none-eabi
    sudo apt install stlink-tools
  
Une description détaillée de l'installation des outils de développement est disponible [ici](https://picatout-jd.blogspot.com/2018/08/pilule-bleue-introduction.html). 
```
Found 1 stlink programmers
 serial:     483f6e066772574857351967
 hla-serial: "\x48\x3f\x6e\x06\x67\x72\x57\x48\x57\x35\x19\x67"
 flash:      65536 (pagesize: 1024)
 sram:       20480
 chipid:     0x0410
 descr:      F1xx Medium-density
jacques@hp15:~/github/stm32-eforth$ 
```

## carte blue pill
![carte](board/blue-pill/board-view-2.jpg)

## communication

La comminication avec la carte se fait par le USART1 configuré à 115200 BAUD, pas de parité, 8 bits et 1 stop. Pas de contrôle de flux.

Pour l'adaptation des niveaux de voltage entre le port RS-232 du PC et la carte **blue-pill** j'ai fabriqué le petit adapteur de niveau que voici.

![schématique](docs/rs-232-level-adaptor-schematic.png)

![assemblage](docs/rs-232-level-adapter-assembly.png)

Côté carte **blue pill** les connection sont les suivantes:

 * **V+** broche **3.3v**.
 * **GND** broche **G**.
 * **TX** broche **A9**
 * **RX** broche **A10**

Côté RS-232

* **TX** broche 2 du connecteur DB-9 femelle.
* **RX** broche 3 du connecteur DB-9 femelle.
* **GND** broche 5 du connecteur DB-9 femelle.

![montage](docs/montage.jpg)

## programmation

Tout le code sera réalisé en assembleur. La première étape sera de configurer le MCU pour qu'il utilise le cristal **8 Mhz** et le PLL pour que la fréquence du MCU soit de **72 Mhz**.  Le code d'initialisation matériel sera dans le fichier [startup.s](board/blue-pill/startup.s).

