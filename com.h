#ifndef COMMECA
#define COMMECA

//***communication meca***
#define mecaADDR 2

#define mecaBusy 1
#define mecaReady 2
#define TIROIR_FERMER 1
#define TIROIR_OUVERT 2
#define TIROIR_DEVERSER 3
#define PEIGNE_OUVERT 4
#define PEIGNE_FERMER 5

//les fonctions
// /!\ne pas associer des actions à 0x01 et 0x02 car c'est les même valeurs que mecaReady et mecaBusy et ça peut poser des problèmes un peu compliqué à expliquer
#define actFermerAvant 0x03
#define actOuvrirAvant 0x04
#define actRangerAvant 0x05
#define actFermerArriere 0x06
#define actOuvrirArriere 0x07
#define actRangerArriere 0x08
#define actResetCoucou 0x09
#define actFermerMoitieAvant 0x0A

#define actNoel 0x10
#define actIntimidation 0x20

#endif
