#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

using namespace std;

int main( )
{
    USING_NAMESPACE_ACADO

    // Variables:
    DifferentialState   p    ;  // the trolley position
    DifferentialState   v    ;  // the trolley velocity 
    DifferentialState   phi  ;  // the excitation angle
    DifferentialState   omega;  // the angular velocity
    Control             a    ;  // the acc. of the trolley

    const double     g = 9.81;  // the gravitational constant 
    const double     b = 0.20;  // the friction coefficient

    // Model equations:
    DifferentialEquation f; 

    f << dot( p ) == v;
    f << dot( v ) == a;
    f << dot( phi ) == omega;
    f << dot( omega ) == -g * sin(phi) - a * cos(phi) - b * omega;

    // Reference functions and weighting matrices:
    Function h, hN;
    h << p << v << phi << omega << a;
    hN << p << v << phi << omega;

    DMatrix W(5, 5);
    W.setIdentity();
    DMatrix WN(4, 4);
    WN.setIdentity();
    WN *= 5;

    DVector r(5) ;
    r.setAll(0.0) ;
    r(0) = 5;

    DVector rN(4) ;
    rN.setAll(0.0) ;
    rN(0) = 5;

    //
    // Optimal Control Problem
    //
    OCP ocp(0.0, 3.0, 10);

    ocp.subjectTo( f );

    ocp.minimizeLSQ(W, h, r);
    ocp.minimizeLSQEndTerm(WN, hN, rN);

    ocp.subjectTo( -1.0 <= a <= 1.0 );
    ocp.subjectTo( -0.5 <= v <= 1.5 );


    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f, identity);

	Process process( dynamicSystem,INT_RK45 );

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	RealTimeAlgorithm alg( ocp,0.05 );
	alg.set( MAX_NUM_ITERATIONS, 2 );
	
	StaticReferenceTrajectory zeroReference;
	Controller controller( alg,zeroReference );

    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	SimulationEnvironment sim( 0.0,10.0,process,controller );

	DVector x0(4);
	x0(0) = 2.0;
	x0(1) = -2.0;
	x0(2) = 0.0;
	x0(3) = 1.0;

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

	GnuplotWindow window;
    window.addSubplot( sampledProcessOutput(0), "trolley Position [m]" );
    window.addSubplot( sampledProcessOutput(1), "trolley velocity [m/s]" );
	window.addSubplot( sampledProcessOutput(2), "phi angle [rad]" );
	window.addSubplot( sampledProcessOutput(3), "omega [rad]" );
	window.addSubplot( feedbackControl(0),      "trolley acceleration [m/s2]" );
	window.plot( );




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

    if (mpc.exportCode( "my_mpc_code" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );
*/

    return EXIT_SUCCESS;
}
