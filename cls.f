\ Efface l'écran du terminal et place le curseur en haut à gauche
: ESC 27 EMIT 91 EMIT ;
: .J 74 EMIT ;
: .H 72 EMIT ;
: CLS ESC 50 EMIT .J \ ESC[2J 
    ESC .H \ ESC[H ;
