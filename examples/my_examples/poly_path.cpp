#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

using namespace std;

int main( )
{
    USING_NAMESPACE_ACADO

    // Variables:
    DifferentialState   x;  // forward direction
    DifferentialState   y;  // distance from lane
    DifferentialState   o;  // car angle 
    DifferentialState   v;  // velocity
    DifferentialState   A;
    DifferentialState   B;
    DifferentialState   C;
    DifferentialState   D;


    Control   phi;  // the steer angle
    Control   a;    // the acc. of the car

    const double  L = 3.0f; //axes distance

    // Model equations:
    DifferentialEquation f; 


    f << dot( x ) == cos(o + phi)*v;
    f << dot( y ) == sin(o + phi)*v;
    f << dot( o ) == (v/L)*tan(phi);
    f << dot( v ) == a;
    f << dot( A ) == (y           - B*x*x - C*x - D) / (x*x*x);
    f << dot( B ) == (y - A*x*x*x         - C*x - D) / (x*x);
    f << dot( C ) == (y - A*x*x*x - B*x*x       - D) / (x);
    f << dot( D ) == (y - A*x*x*x - B*x*x - C*x    );

    // Reference functions and weighting matrices:
    Function h;
    h << o << v;
    h << A << B << C << D;
    h << phi << a;

    DMatrix W(8, 8);
    W.setIdentity();
    
    Function hN;
    hN << o << v;
    hN << A << B << C << D;

    DMatrix WN(6, 6);
    WN.setIdentity();

    //refs ONLY FOR SIMULATION
    DVector r(8) ;
    r.setAll(0.0);
    r(5) = 10;

    DVector rN(6) ;
    rN.setAll(0.0) ;
    r(5) = 10;

    //
    // Optimal Control Problem
    //
    OCP ocp(0.0, 3.0, 10);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h, r);
    ocp.minimizeLSQEndTerm(WN, hN, rN);
//    ocp.minimizeLSQ(W, h);
//    ocp.minimizeLSQEndTerm(WN, hN);

    ocp.subjectTo( -10.0 <= a <= 10.0 );
    ocp.subjectTo( -5 <= v <= 20 );
    ocp.subjectTo( -0.6 <= phi <= 0.6 );

    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f, identity);

	Process process( dynamicSystem,INT_RK45 );

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	RealTimeAlgorithm alg( ocp, 0.025 );
	alg.set(MAX_NUM_ITERATIONS, 5 );
	alg.set(INFEASIBLE_QP_HANDLING, YES);

	StaticReferenceTrajectory zeroReference;
	Controller controller( alg,zeroReference );

    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	SimulationEnvironment sim( 0.0,5.0,process,controller );

	DVector x0(8);
	x0(0) = 0.1;
	x0(1) = 3.0;
	x0(2) = 0.0;
	x0(3) = 10.0;

	if (sim.init(x0 ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	if (sim.run( ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

    // ...AND PLOT THE RESULTS
    // ----------------------------------------------------------
	VariablesGrid sampledProcessOutput;
	sim.getSampledProcessOutput( sampledProcessOutput );

	VariablesGrid feedbackControl;
	sim.getFeedbackControl( feedbackControl );

	GnuplotWindow w0, w1, w2;
    w0.addSubplot( sampledProcessOutput(0), "x");
    w0.addSubplot( sampledProcessOutput(1), "y");
    w0.plot();
	w1.addSubplot( sampledProcessOutput(2), "car angle [rad]");
	w1.addSubplot( sampledProcessOutput(3), "speed [m/s]");
    w1.plot();
	w2.addSubplot( feedbackControl(0),      "steer angle");
    w2.addSubplot( feedbackControl(1),      "throttle");
    w2.plot();

/*
    // Export the code:
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
    mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
    mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
    mpc.set( NUM_INTEGRATOR_STEPS,        30              );

    mpc.set( QP_SOLVER,                   QP_QPOASES      );
    mpc.set( GENERATE_TEST_FILE,          YES             );
    mpc.set( GENERATE_MAKE_FILE,          YES             );
    mpc.set( GENERATE_MATLAB_INTERFACE,   YES             );
    mpc.set( GENERATE_SIMULINK_INTERFACE, YES             );

    if (mpc.exportCode( "poly_path_code" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );
*/
    return EXIT_SUCCESS;
}
