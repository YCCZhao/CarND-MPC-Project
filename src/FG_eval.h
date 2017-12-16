#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
	
class FG_eval {
 public:
  FG_eval();

  virtual ~FG_eval();

  // Same as MPC. 
	// Set the timestep length and duration
	size_t N;
	double dt;

	// Same as MPC. 
	// This value assumes the model presented in the classroom is used.
	//
	// It was obtained by measuring the radius formed by running the vehicle in the
	// simulator around in a circle with a constant steering angle and velocity on a
	// flat terrain.
	//
	// Lf was tuned until the the radius formed by the simulating the model
	// presented in the classroom matched the previous radius.
	//
	// This is the length from front to CoG that has a similar radius.
	double Lf;

  // Same as MPC. 	
	// reference speed
	double ref_v;

	// Same as MPC. 
	// The solver takes all the state variables and actuator
	// variables in a singular vector. Thus, we should to establish
	// when one variable starts and another ends to make our lifes easier.
	size_t x_start;
	size_t y_start;
	size_t psi_start;
	size_t v_start;
	size_t cte_start;
	size_t epsi_start;
	size_t delta_start;
	size_t a_start;
	
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs; 
	
	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	
	void operator()(ADvector& fg, const ADvector& vars) {
		
		// TODO: implement MPC
		// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
		// NOTE: You'll probably go back and forth between this function and
		// the Solver function below.
		// The cost is stored is the first element of `fg`.
		// Any additions to the cost should be added to `fg[0]`.
		fg[0] = 0;

		// Reference State Cost
		// Define the cost related the reference state
		for (unsigned int t = 0; t < N; t++) {
			fg[0] += CppAD::pow(vars[cte_start + t], 2);
			fg[0] += CppAD::pow(vars[epsi_start + t], 2)*10;
			fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2)*0.1;
	}

		// Minimize the use of actuators.
		for (unsigned int t = 0; t < N - 1; t++) {
			fg[0] += CppAD::pow(vars[delta_start + t], 2)*5;
			fg[0] += CppAD::pow(vars[a_start + t], 2)*0.1;
			// inspiration from https://github.com/AeroGeekDean/CarND-MPC-Project
			// try adding penalty for speed + steer
			fg[0] += CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2)*10;
		}

		// Minimize the value gap between sequential actuations.
		for (unsigned int t = 0; t < N - 2; t++) {
			fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2)*800;
			fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}

		// Setup Constraints

		// Initial constraints
		//
		// We add 1 to each of the starting indices due to cost being located at
		// index 0 of `fg`.
		// This bumps up the position of all the other values.
		fg[1 + x_start]    = vars[x_start];
		fg[1 + y_start]    = vars[y_start];
		fg[1 + psi_start]  = vars[psi_start];
		fg[1 + v_start]    = vars[v_start];
		fg[1 + cte_start]  = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// The rest of the constraints
		for (unsigned int t = 1; t < N; t++) {
			AD<double> x1    = vars[x_start + t];
			AD<double> y1    = vars[y_start + t];
			AD<double> psi1  = vars[psi_start + t];
			AD<double> v1    = vars[v_start + t];
			AD<double> cte1  = vars[cte_start + t];
			AD<double> epsi1 = vars[epsi_start + t];
		
			AD<double> x0    = vars[x_start + t - 1];
			AD<double> y0    = vars[y_start + t - 1];
			AD<double> psi0  = vars[psi_start + t - 1];
			AD<double> v0    = vars[v_start + t - 1];
			AD<double> cte0  = vars[cte_start + t - 1];
			AD<double> epsi0 = vars[epsi_start + t - 1];
		
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0     = vars[a_start + t - 1];
			
			// use previous actuations (to account for latency)
			// previous actuation is 0.1s ago = 100 millisecond latency
			if (t > 1) {   
				a0 = vars[a_start + t - 2];
				delta0 = vars[delta_start + t - 2];
			}
		
			AD<double> f0      = coeffs[0] + coeffs[1] * x0\
												 + coeffs[2] * CppAD::pow(x0, 2)\
												 + coeffs[3] * CppAD::pow(x0, 3);
			AD<double> psides0 = CppAD::atan(coeffs[1]\
												 + 2 * coeffs[2] * x0\
												 + 3 * coeffs[3] * CppAD::pow(x0, 2));

			// Here's `x` to get you started.
			// The idea here is to constraint this value to be 0.
			//
			// NOTE: The use of `AD<double>` and use of `CppAD`!
			// This is also CppAD can compute derivatives and pass
			// these to the solver.

			// TODO: Setup the rest of the model constraints
			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta0 * dt);
			fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
		}
	}

};
