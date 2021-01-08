# eForth sur blue-pill

* MCU:  stm32f103c8t6

* Configuré pour utilisé le cristal externe avec le PLL pour sysclock=72 Mhz.


![board-view](docs/board-view-2.jpg)

## Pour construire et flasher

Il faut indiquer dans le fichier [Makefile](Makefile) le numéro de série du programmeur STLINK-V2.

Pour ce modèle de carte il y a 2 versions. La version *subroutine threaded*
```
$ make build && make flash

```
Pour la version *indirect threaded*:
```
$ make build_fl && make flash_fl 
```
