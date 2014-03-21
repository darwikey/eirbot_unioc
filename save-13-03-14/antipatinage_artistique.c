
int8_t patinage;
#define TOLERANCE_DERAPAGE 0.5
#define RAPPORT_STATIC_DERAPAGE 2.1504

void antipatinage(void * p) {
	static uint32_t old_encodeur_gauche = 0;
	static uint32_t old_encodeur_droit = 0;
	static uint32_t old_moteur_gauche = 0;
	static uint32_t old_moteur_droit = 0;

	uint32_t encodeur_gauche = U_ENC1;
	uint32_t encodeur_droit = U_ENC2;
	uint32_t moteur_gauche = U_ENC0;
	uint32_t moteur_droit = U_ENC3;

	float rapport_roues_gauches = 0
	float rapport_roues_droit = 0

	rapport_roues_gauches = (float)(moteur_gauche - old_moteur_gauche) / (float)(encodeur_gauche - old_encodeur_gauche);
	rapport_roues_droites = (float)(moteur_droit - old_moteur_droit) / (float)(encodeur_droit - old-encodeur-droit);

	if ( abs(rapport_roues_gauches - RAPPORT_STATIC_DERAPAGE) > TOLERANCE_DERAPAGE 
		|| abs(rapport_roues_droites - RAPPORT_STATIC_DERAPAGE) > TOLERANCE_DERAPAGE ) {
		patinage = 1;
	}
	else {
		patinage = 0;
	}

	/* Les valeurs new deviennent old pour le prochain appel de la fonction */
	old_encodeur_gauche = encodeur_gauche;
	old_encodeur_droit = encodeur_droit;
	old_moteur_gauche = moteur_gauche;
	old_moteur_droit = moteur_droit;
}


