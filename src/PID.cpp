#include "PID.h"
#include <vector>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_ ) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  int_cte = 0; // integrate of the error
  prev_cte = 0;

  dp.push_back(0.1);
  dp.push_back(0.5);
  dp.push_back(0.0001);

  state = 0;
  best_err = -1;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  double cte_dot;
  // std::cout << " prev_cte, cte "<<prev_cte<<","<<cte<<std::endl;
  cte_dot = cte - prev_cte;
  int_cte += cte;
  p_error = -Kp * cte;
  d_error = -Kd * cte_dot;
  i_error = -Ki * int_cte;
  prev_cte = cte; 
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double all_error;

  all_error = p_error + d_error + i_error;
  return all_error;  // TODO: Add your total error calc here!
}

void PID::Init_err(double curr_err){
  if (best_err <0){
    best_err = curr_err;
  }
}

void PID::twiddle(double current_err, int idx) {
  // idx: 0 (Kp term), 1: (Kd term), 2: (Ki term),
  // state 0: (p+dp), 
  // state:1 compare error, 
  // state:2 (after state1, error goes down -> ready for next loop),
  // state:3 (after state 1, error goes up, p - 2dp, go to state 4 & 5), 
  // state:4 (err goes down after state 3 -> ready for next loop), 
  // state 5 (error goes up after state 3, k+dp, dp*=0.9, -> ready for next loop )
  if (state == 0){
    if (idx==0){
      Kp += dp[idx];
    } else if(idx==1) {
      Kd += dp[idx];
    } else if(idx==2) {
      Ki += dp[idx];
    }    
    state = 1;
  } 
  else if (state == 1){
    if (current_err < best_err){
      best_err = current_err;
      dp[idx]*=1.1;
      state =2 ;
    } else {
      if (idx==0){
        Kp -= 2*dp[idx];
      }  else if (idx==1){
        Kd -= 2*dp[idx];
      } else if (idx ==2){
        Ki -= 2*dp[idx];
      } 
      state = 3;
    }
  }
  else if (state ==3){
    if (current_err<best_err){
      best_err = current_err;
      dp[idx]*=1.1;
      state = 4;
    } 
    else{
      std::cout<<"idx in twiddle at state5:"<<idx<<std::endl;
      if (idx==0){
        Kp += dp[idx];
        dp[idx]*=0.9;   
      } else if (idx ==1){
        Kd += dp[idx];
        dp[idx]*=0.9;
      } else if (idx ==2){
        std::cout<<"ki before:"<<Ki<<std::endl;
        Ki += dp[idx];
        dp[idx]*=0.9;
        std::cout<<"ki after:"<<Ki<<std::endl;
      }  
      state = 5;
    }
  }

  std::cout<<"idx is:"<<idx<<",  state in twiddle:"<<state<<std::endl;
  std::cout<<"best error in twiddle:"<<best_err<<std::endl;
  std::cout<<"Kp is:"<<Kp<<", dp[0]:"<<dp[0]<<std::endl;
  std::cout<<"Kd is:"<<Kd<<", dp[1]:"<<dp[1]<<std::endl;
  std::cout<<"Ki is:"<<Ki<<", dp[2]:"<<dp[2]<<std::endl;
}

bool PID::tunecheck(){
  //if ((dp[0]+dp[1]+dp[2])<0.2){
  if (best_err < 0.001){
    std::cout<<"done with tuning"<<std::endl;
    std::cout<<"Kp is"<<Kp<<std::endl;   
    std::cout<<"Ki is"<<Ki<<std::endl;
    std::cout<<"Kd is"<<Kd<<std::endl;
    return true;
  }
  else{
    return false;
  }
}

