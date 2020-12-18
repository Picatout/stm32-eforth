\ mots pour contrôler la position du curseur sur le terminal
\ en utilisant les séquences ANSI 

\ ESC[ ( -- )
\ envoie la séquence d'échappement ANSI 
: ESC[ 27 EMIT 91 EMIT ;

\ PARAM ( u -- )
\ envoie un paramètre numérique 
: PARAM  <# #S #> TYPE ;

\ envoie un 'J' au terminal 
: .J 74 EMIT ;
\ envoie un 'H' au terminal 
: .H 72 EMIT ;

\ CLS ( -- )
\ efface l'écran et positionne le curseur à {0,0}
: CLS ESC[ 2 PARAM .J \ ESC[2J 
    ESC[ .H ; \ ESC[H 

\ envoie un ';' au terminal 
: .; [ CHAR ;; ] LITERAL  EMIT ;

\ CURPOS ( X Y -- )
\ positionne le curseur x colonne, y ligne 
: CURPOS ESC[ PARAM .; PARAM .H ; 

