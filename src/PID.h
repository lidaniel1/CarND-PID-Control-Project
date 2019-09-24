#include <vector>
#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);
  
  void Init_err(double best_err);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  void twiddle(double current_err, int idx);
  
  bool tunecheck();
  
  int state;
  double int_cte;  
  double prev_cte;
 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  
  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  std::vector<double> dp;
  double best_err;
  
};

#endif  // PID_H