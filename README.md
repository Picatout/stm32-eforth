# stm32-eforth

Adaptation de eforth pour sur la carte **STM32G431-NUCLEO-32**. Je vais travaillé à partir du fichier original de C.H. Ting [stm32eForth720](http://forth.org/OffeteStore/2165_stm32eForth720.zip).  Le code source a été écris en assembleur pour les outils de développements µVision 5. Je vais utilisé **gcc-arm-none-eabi** il faudra donc une réécriture du fichier source original. 

## prérequis

Je travaille sur Ubuntu 20.04 LTS les logiciles suivants doivent-être installés. 

    sudo apt install gcc-arm-none-eabi
    sudo apt install stlink-tools
  
 la commande **st-info --probe** ne rapporte pas les bonnes information sur la carte. Il faut créé le fichier **/etc/modprobe.cond** qui contient la ligne suivante:
 
    options usb-storage quirks=483:3744:i


Après redémarrage ça fonctionne:

```
jacques@dv6:~$ st-info --probe
Found 1 stlink programmers
 serial:     303034433030333433313337353130333339333833353338
 hla-serial: "\x30\x30\x34\x43\x30\x30\x33\x34\x33\x31\x33\x37\x35\x31\x30\x33\x33\x39\x33\x38\x33\x35\x33\x38"
 flash:      131072 (pagesize: 2048)
 sram:       32768
 chipid:     0x0468
 descr:      G4 Category-2
jacques@dv6:~$ 
```


  
