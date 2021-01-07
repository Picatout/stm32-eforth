\ jeu serpent
\ utilise ansi.f 
\ pour charger faire:  SendFile/SendFile ansi.f snake.f 
\
\ constantes
128 CONSTANT max-len \ longueur maximale du serpent
\ directions deplacement
0 CONSTANT east
1 CONSTANT south
2 CONSTANT west
3 CONSTANT north
78 CONSTANT play-width \ largeur surface jeu
22 CONSTANT play-height \ hauteur surface jeu
2 CONSTANT x-offset \ pour affichage
3 CONSTANT y-offset \ pour affichage
75 CONSTANT speed \ controle vitesse serpent
CHAR S CONSTANT ar_left \ vire a gauche
CHAR D CONSTANT ar_right \ vire a droite
0 CONSTANT false 
-1 CONSTANT true 

\ variables
VARIABLE score \ pointage
VARIABLE head \ direction serpent
VARIABLE snake-len \ longueur serpent
VARIABLE food \ localisation pastille nourriture
VARIABLE tail \ localisation ajout anneau serpent

\ vector permet de creer des variables tableau 1D
: vector CREATE CELLS ALLOT DOES> SWAP CELLS + ;
\ variables tableaux
4 vector c-head \ contient les caracteres de tete serpent
max-len vector snake \ le corps du serpent

\ initialisation c-head
CHAR < east c-head ! \ tete direction est
CHAR W south c-head ! \ tete direction sud
CHAR > west c-head ! \ tete direction ouest
CHAR V north c-head ! \ tete direction nord

\ converti caractère ASCII en majuscule
: UPCHAR ( c -- C )
    32 NOT AND ;

\ fonctions graphiques
\ conversion entier non signe vers couple {x,y}
: ucoord>xy ( u -- x y )
   256 /MOD ;

\ conversion couple {x,y} vers ucoord
: xy>ucoord ( x y -- u )
   127 AND 256 * SWAP 127 AND + ;

\ imprime le caractère  'c' à la position {x,y}
: xy-put ( c x y -- )
   y-offset + SWAP x-offset + SWAP CURPOS EMIT ;

\ imprime un anneau du serpend à la position 'u'
: draw-ring ( u -- )
    true B/W
    [ CHAR O ] LITERAL SWAP ucoord>xy xy-put
    false B/W ;

\ whiteln ( n -- )
\ dessine une ligne en inverse vidéo 
\ n est le numéro de la ligne 
: whiteln 
    1 SWAP CURPOS 
    true B/W 
    play-width  2+ SPACES 
    false B/W ;

\ dessine les bandes de l'arene
: draw-walls ( -- )
    CLS 
    2 whiteln 
    play-height 3 + whiteln
    true B/W 
    play-height 3 + >R 3
    BEGIN DUP R@ < WHILE 
        1 OVER CURPOS SPACE 
        play-width 2+ OVER CURPOS SPACE 
        1+ 
        REPEAT 
    R>
    2DROP  
    false B/W ;

\ dessine la tête
: draw-head ( -- )
    head @ c-head @
    0 snake @ 
    ucoord>xy 
    xy-put ;

\ dessine le serpent
: draw-snake ( -- )
    draw-head 
    snake-len @ >R 
    1 BEGIN 
        DUP R@ < WHILE 
        DUP snake @ draw-ring 
        1+ 
        REPEAT
    R> 
    2DROP
    1 26 CURPOS  
    ;

\ affiche le status
: status ( -- )
   true B/W 
   1 1 CURPOS CLREOL 
   ." SCORE:" score @ .
   16 1 CURPOS 
   ." LENGTH:" snake-len @ . 
   false B/W ;

\ Lors de la creation d'une pastille il faut valider
\ qu'elle ne superpose pas au serpent.
: valid-food? ( u -- f )
    true SWAP snake-len @ >R 0 
    BEGIN
        DUP R@ < WHILE  ( f u c )
        DUP snake @ 2 PICK = IF 
            2DROP false SWAP R@ ( F T L  )  THEN  
        1+
        REPEAT 
    2DROP 
    R>
    DROP ;

\ creation d'une pastille de nourriture
: new-food ( -- )
    0 
    BEGIN 
        DROP play-width  RANDOM   \ x
        play-height RANDOM  \ y
        xy>ucoord DUP valid-food? 
        UNTIL 
    food ! ;

\ verifie si le serpent se mord.
: snake-bite? ( -- f )
    false 0 snake @  snake-len @ >R 1 
    BEGIN
        DUP R@ < WHILE 
        DUP snake @ 2 PICK = IF 
            2DROP true SWAP R@  THEN
        1+
        REPEAT
    DROP 
    R> 2DROP ;


\ retourne un flag pour chaque coordonnee
\ vrai si le long d'un mur.
: borders? ( u1 -- fy fx )
   ucoord>xy DUP 0= SWAP play-height 1- = OR
   SWAP DUP 0= SWAP play-width 1- = OR ;

\ ajuste SCORE
: score+ ( -- )
   1 food @ borders?
   IF SWAP 2* SWAP THEN
   IF 2* THEN
   score +! true food ! ;

\ rallonge le serpent
: snake+ ( -- )
   snake-len DUP >R @ DUP 1+ R> ! tail @  SWAP snake ! ;

\ dessine pastille nourriture
: draw-food  
    [ CHAR X ] LITERAL 
    food @
    ucoord>xy 
    xy-put ;

\ déplace la tête du serpent 
\ u1 coordonnées actuelle de la tête 
\ u2 nouvelle coordonnées 
: move-head ( u1 -- )
    ucoord>xy
    head @ 
    DUP east = IF DROP SWAP 1+ SWAP ELSE  
    DUP south = IF DROP 1+ ELSE  
    DUP west = IF DROP SWAP 1- 255 AND SWAP ELSE  
    DROP 1-  \ north
    THEN THEN THEN   
    xy>ucoord 
    0 snake ! ;

\ deplace le serpent
: move-snake ( -- )
    0 snake @ DUP
    move-head
    draw-head
    DUP draw-ring \ dessine le premier anneau  
\ déplace anneaux du serpent     
    snake-len @  >R 1 
    BEGIN ( u i -- )
        DUP R@ < WHILE 
        DUP snake @ >R DUP >R snake ! 
        R> R> SWAP  ( u i -- )
        1+
        REPEAT
    R> 2DROP     
    BL OVER ucoord>xy xy-put \ efface le dernier anneau 
    tail ! 
    1 snake @ draw-ring
    1 26 CURPOS 
    ;

\ verification collision avec mur
: wall-bang? ( -- f )
   0 snake @ ucoord>xy  play-height 1- SWAP U<
   SWAP  play-width 1- SWAP U< OR ;

\ verification collision
: collision? ( -- f )
   snake-bite? wall-bang? OR ;

 \ initialisation du serpent
: snake-init ( -- )
   east head ! -1 food !
   play-width 2/ play-height 2/ xy>ucoord snake-len @  >R 0
   BEGIN 
        DUP R@ < WHILE 
        2DUP snake ! 
        SWAP 1- SWAP
        1+
        REPEAT
        2DROP 
        R> DROP  ;

\ touche quitte enfoncée?
: quit-game? ( c -- f )
    [ CHAR Q ] LITERAL = 
    IF true 
    ELSE false 
    THEN ;

\ touche flèche gauche|droite?
: turn? ( c -- f )
    DUP ar_left = 
    IF DROP -1 
    ELSE ar_right =
        IF 1
        ELSE 0 
        THEN 
    THEN 
    DUP  
    head @ + 3 AND head ! 
    ; 

\ lecture clavier touche 'q' quitte le jeu.
: user-key? ( -- f )
   ?KEY IF 
        UPCHAR DUP turn? 
            IF DROP false 
            ELSE quit-game?
            THEN     
        ELSE false
        THEN 
        ;  

\ pastille mangee?
: eaten? ( -- f )
   0 snake @ food @ = ;

\ boucle du jeu
: game-loop ( -- )
  BEGIN 
    speed PAUSE 
    status food @ -1 = IF new-food THEN 
    draw-food  
    user-key? 
    ?DUP NOT IF 
             move-snake eaten? 
                IF score+  snake+ false 
                ELSE
                collision? 
                THEN
            THEN   
    UNTIL ;

\ initialisation du jeu
: game-init ( -- )
\   MSEC SEED ! 
   4 snake-len ! 0 score ! 
   snake-init draw-walls ;

\ partie terminee
: game-over? ( -- f )
   500 PAUSE
   ?KEY IF DROP THEN  
   1 25 CURPOS ." game over <q> leave" 
   KEY UPCHAR quit-game? ;

\ lance le jeux.
: snake-run ( -- )
    MSEC SEED ! 
    BEGIN 
        game-init 
        game-loop 
        game-over?  
        UNTIL 
    CLS ;

