#include "../gp2.h"

#include <assert.h>
#include <stdio.h>

int main(int argc, char* argv[]) {
	gp2_cfg_t gp2;
	
	gp2_init(&gp2);
	gp2_init_xya(&gp2, 10, 10, 0);

	gp2_add_point(&gp2, 50, 1000);
	printf("dist=%d :: adc=%d\n", gp2.points[0].dist, gp2.points[0].adc);

	// ATTENTION : le cast en (uint16_t) est obligatoire, si non erreur
#define TEST_DIST(volt, dist) \
	printf("[adc:%d] %d == %d (cm)\n", (uint16_t)(volt*VOLT2ADC), gp2_get_dist(&gp2, (uint16_t)(volt*VOLT2ADC)), dist); \
	assert(gp2_get_dist(&gp2, (uint16_t)(volt*VOLT2ADC)) == dist)

	TEST_DIST(1010, 0);
	TEST_DIST(990, 50);

	TEST_DIST(1000, 50);

	gp2_add_point(&gp2, 100, 500);
	printf("dist=%d :: adc=%d\n", gp2.points[1].dist, gp2.points[1].adc);
	printf("dist=%d :: adc=%d\n", gp2.points[0].dist, gp2.points[0].adc);

	TEST_DIST(1010, 0);
	TEST_DIST(990, 50);
	TEST_DIST(700, 50);
	TEST_DIST(510, 50);
	TEST_DIST(500, 100);
	TEST_DIST(490, 100);
	TEST_DIST(0, 100);

	printf("ok !\n");

	return 0;
}
