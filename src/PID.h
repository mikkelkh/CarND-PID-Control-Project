#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

#define FRAMES_PER_TWIDDLE_ITERATION 1500
#define IGNORE_FIRST_STEPS 100

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double total_error;
  double prev_cte;
  double sum_cte;

  bool initialized;
  int iter;

//  struct coefficients {
//	  double Kp;
//	  double Ki;
//	  double Kd;
//  };

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  double GetValue();

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  /* Twiddle */
  bool twiddle_done;
  double best_err;
  int twiddle_i;
  int twiddle_iterations = FRAMES_PER_TWIDDLE_ITERATION;
  size_t twiddle_var_state;
  bool twiddle_substate = false;
  bool twiddle_subsubstate = false;
  std::vector<double> p;
  std::vector<double> dp;
  void twiddle(double tol, uWS::WebSocket<uWS::SERVER> ws);

  // twiddle_iterations 200, Iteration 39, error = 53.9926, best error = 55.1046, p=[0.771561,0.199,0], dp=[0.161051,0.088209,0.059049]
  // twiddle_iterations 500, Iteration 120, error = 4.75494, best error = 4.91374, p=[0.485012,4.25632,0.000852756], dp=[0.10618,0.436978,0.00307566]
  // twiddle_iterations 1500, Iteration 53, error = 88.9434, best error = 96.5254, p=[0.871971,7.38248,0.00466666], dp=[0.0927237,0.69672,0.00219754]
};

#endif /* PID_H */
