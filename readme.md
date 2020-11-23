# stm32-eforth

<<<<<<< HEAD
Adaptation de eforth pour sur la carte **blue-pill**. Je vais travaillé à partir du fichier original de C.H. Ting [stm32eForth720](http://forth.org/OffeteStore/2165_stm32eForth720.zip).  Le code source a été écris en assembleur pour les outils de développements µVision 5. Je vais utilisé **gcc-arm-none-eabi** il faudra donc une réécriture du fichier source original. 
=======
Adaptation de eforth pour sur la carte **STM32G431-NUCLEO-32**. Je vais travaillé à partir du fichier original de C.H. Ting [stm32eForth720](http://forth.org/OffeteStore/2165_stm32eForth720.zip).  Le code source a été écris en assembleur pour les outils de développements µVision 5. Je vais utilisé **gcc-arm-none-eabi** il faudra donc une réécriture du fichier source original. Pour le suivit du projet voir le fichier [journal](journal.md).
>>>>>>> 9ae0a76695678f68e3306f112f36541d472a1cf8

## prérequis

Je travaille sur Ubuntu 20.04 LTS les logiciles suivants doivent-être installés. 

    sudo apt install gcc-arm-none-eabi
    sudo apt install stlink-tools
  
 
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

## Modification matérielle
![carte](board/blue-pill/board-view-2.jpg)
  
## Première étape

Tout le code sera réalisé en assembleur. La première étape sera de configurer le MCU pour qu'il utilise le cristal **8 Mhz** et le PLL pour que la fréquence du MCU soit de **72 Mhz**. 

