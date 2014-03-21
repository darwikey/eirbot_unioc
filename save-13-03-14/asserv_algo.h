#ifndef _ASSERV_ALGO_
#define _ASSERV_ALGO_

int8_t is_skating(void);
int8_t is_oscillating(trajectory_manager_t * traj, int16_t);
int8_t error_test_p(trajectory_manager_t * traj);
int16_t pid_p_tester(asserv_manager_t * asserv, trajectory_manager_t * traj, position_manager_t * pos);
double eval_position_error(double x_ordered, double y_ordered, double x_real, double y_real);   // I
double error_test_i(trajectory_manager_t * traj);
double mean_error_tests_i(trajectory_manager_t * traj);
int8_t work_out_i_best_value(asserv_manager_t* asserv, trajectory_manager_t* traj);
void useless_test(asserv_manager_t *	asserv, trajectory_manager_t *	traj, position_manager_t *	pos);


#endif
