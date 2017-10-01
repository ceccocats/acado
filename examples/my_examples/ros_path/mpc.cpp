#include "../poly_path_code/acado_common.h"
#include "../poly_path_code/acado_auxiliary_functions.h"

#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

float last_steer;
float last_throttle;

/* A template for testing of the solver. */
race::drive_param mpc_control(float A, float B, float C, float D, float init_speed, mpc_ref_t *mpc_ref) {

	/* Some temporary variables. */
	int    i, iter;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (i = 0; i < N*NY; i+= NY) {
		acadoVariables.y[i]   = 2.0;
		// cto, pto, steer and acc to zero 
	}

	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;
	acadoVariables.yN[0] = 2.0;

	/* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.0;
	acadoVariables.x0[3] = init_speed; 
	acadoVariables.x0[6] = last_steer;
	acadoVariables.x0[7] = last_throttle;
#endif

	/* Initialize online data */
	for (i = 0; i < NOD * (N+1); i+= NOD) {
		acadoVariables.od[i+0] = A;
		acadoVariables.od[i+1] = B;
		acadoVariables.od[i+2] = C;
		acadoVariables.od[i+3] = D;

	}

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );

	/* The "real-time iterations" loop. */
	for(iter = 0; iter < NUM_STEPS; ++iter)
	{
        /* Perform the feedback step. */
		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		/* Optional: shift the initialization (look at acado_common.h). */
        /* acado_shiftStates(2, 0, 0); */
		/* acado_shiftControls( 0 ); */

		/* Prepare for the next step. */
		acado_preparationStep();
	}
	/* Read the elapsed time. */
	real_t te = acado_toc( &t );


	//acado_printDifferentialVariables();
	//acado_printControlVariables();
	for (i = 0; i < (N+1)*NX; i+=NX) {
		mpc_ref[i%(N+1)].x     = acadoVariables.x[i];
		mpc_ref[i%(N+1)].y     = acadoVariables.x[i+1];
		mpc_ref[i%(N+1)].phi   = acadoVariables.x[i+2];
		mpc_ref[i%(N+1)].speed = acadoVariables.x[i+3];
	}

	last_steer = acadoVariables.u[0];
	last_throttle = acadoVariables.u[1];

	race::drive_param drive_msg;
	drive_msg.angle = (-acadoVariables.u[0]/0.7)*100;
	drive_msg.velocity = acadoVariables.u[1]*100;
    return drive_msg;
}
