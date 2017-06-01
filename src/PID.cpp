#include "PID.h"
#include <numeric>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
	p_error = 0;
	i_error = 0;
	d_error = 0;
	sum_cte = 0;
	total_error = 0;
	iter = 0;

	best_err = std::numeric_limits<double>::max();
	twiddle_i = 0;
	twiddle_done = true; // set to "true" to optimize parameters
	dp.clear();
	dp.push_back(0.10618); // dKp
	dp.push_back(0.436978); // dKd
	dp.push_back(0.00307566); // dKi
	twiddle_var_state = p.size()-1;

	initialized = false;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	p.clear();
	p.push_back(Kp); // Kp
	p.push_back(Kd); // Kd
	p.push_back(Ki); // Ki
}

void PID::UpdateError(double cte) {
	if (!initialized)
	{
		prev_cte = cte;
		initialized = true;
	}

	double diff_cte = cte-prev_cte;
	sum_cte += cte;
	p_error = cte;
	d_error = diff_cte;
	i_error = sum_cte;
	if (iter > IGNORE_FIRST_STEPS)
		total_error += cte*cte;
	prev_cte = cte;
}

double PID::GetValue() {
	double steer = -Kp*p_error-Kd*d_error-Ki*i_error;
	// Ensure values between -1 and 1
	steer = steer<-1?-1:steer;
	steer = steer>1?1:steer;
	return steer;
}

double PID::TotalError() {
	return total_error;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
	std::string reset_msg = "42[\"reset\",{}]";
	ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

void PID::twiddle(double tol, uWS::WebSocket<uWS::SERVER> ws) {
	if ((iter<twiddle_iterations) || (twiddle_done))
		iter++;
	else
	{
		double sum_dp = std::accumulate(dp.begin(), dp.end(), 0.0);

		if (sum_dp < tol)
		{
			std::cout << "Twiddle done!!!" << std::endl;
			twiddle_done = true;
		}
		else
		{
			TotalError();
			if (twiddle_i==0)
				best_err = total_error;

			std::cout << "Iteration " << twiddle_i << ", error = " << total_error << ", best error = " << best_err << ", p=[" << p[0] <<","<<p[1]<<","<<p[2]<<"], dp=["<<dp[0]<<","<<dp[1]<<","<<dp[2]<<"]" << std::endl;

			if (twiddle_substate==false)
			{
				twiddle_var_state = (twiddle_var_state+1) % p.size();
				p[twiddle_var_state] += dp[twiddle_var_state];
				twiddle_substate = true;
			}
			else
			{

				if (twiddle_subsubstate==false)
				{
					if (total_error < best_err)
					{
						best_err = total_error;
						dp[twiddle_var_state] *= 1.1;
						twiddle_substate = false;
					}
					else
					{
						p[twiddle_var_state] -= 2*dp[twiddle_var_state];
						twiddle_subsubstate = true;
					}
				}
				else
				{
					if (total_error < best_err)
					{
						best_err = total_error;
						dp[twiddle_var_state] *= 1.1;
					}
					else
					{
						p[twiddle_var_state] += dp[twiddle_var_state];
						dp[twiddle_var_state] *= 0.9;
					}
					twiddle_subsubstate = false;
					twiddle_substate = false;
				}
			}
		}

		iter = 0;
		total_error = 0;
		sum_cte = 0;
		initialized = false;

		twiddle_i++;

		Kp = p[0];
		Kd = p[1];
		Ki = p[2];

		Restart(ws);
	}
}
