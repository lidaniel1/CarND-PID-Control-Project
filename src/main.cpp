#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;
  PID pid;
  uWS::WebSocket<uWS::SERVER> ws;
  //PID pid_th;
  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(0.1,1.5,0.00105);
  //pid_th.Init(0.01,0.001, 0.0001);
  int cnt;
  cnt = 0;
  
  // set twiddle_en = true to enable twiddle, otherwise, it will continue to drive
  bool twiddle_en = false;
  double err = 0;
  int idx_loop = 0;
  double throttle = 0.3;

  h.onMessage([&pid, &cnt, &twiddle_en, &err, &idx_loop, &throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                                                                    uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    int max_cnt = 500;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;


          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          if (twiddle_en){
            cnt = cnt + 1;
          //std::cout<<"count is "<<cnt<<std::endl;
          }
          // std::cout<<"steering angle,"<<angle<<std::endl;
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          if (steer_value <= - 1){
            steer_value = -1;
          }
          else if (steer_value >= 1){
            steer_value = 1;
          }

          // simple throttle control

          if (cte <=-0.4 or cte >=0.4){
            throttle*=0.75;
          }
          else{
            if (speed < 60) {
              throttle *=1.05;
            } 
          }


          
          if (throttle<0.3){
            throttle =0.3;
          } else if (throttle >0.75) {
            throttle = 0.75;
          }

          //std::cout<<"throttle "<<throttle<<std::endl;
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;

          if (cnt >= max_cnt/2 and cnt<max_cnt){           
            err += cte*cte;
          }
          if (cnt<max_cnt){
            //std::cout<<"cnt is"<<cnt<<" , cte error"<<cte<<std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          }
          else if (cnt == max_cnt){
            err = err * 2/max_cnt; // err divide by number of samples           
            // reset
            std::cout<<"reset"<<std::endl;
            string msg ="42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else
          { 
            pid.Init_err(err);
            std::cout<<"CTE error sum is :"<<err<<std::endl;
            std::cout<<"integral CTE error is :"<<pid.int_cte<<std::endl;
            // std::cout<<"before first twiddle idx loop is"<<idx_loop<<" state is"<<pid.state<<std::endl;
            pid.twiddle(err, idx_loop);            
            // idx_loop: 0 (Kp term), 1: (Kd term), 2: (Ki term),
            // state 0: base line (p+dp), 
            // state:1 compare error, 
            // state:2 (after state1, error goes down -> ready for next gain),
            // state:3 (after state 1, error goes up, p - 2dp, go to state 4 & 5), 
            // state:4 (err goes down after state 3 -> ready for next term), 
            // state 5 (error goes up after state 3, k+dp, dp*=0.9, -> ready for next gain)
            if (pid.state ==2 or pid.state ==4 or pid.state ==5){
              pid.state = 0;
              //std::cout<<"idx loop is"<<idx_loop<<std::endl;
              idx_loop +=1;
              //std::cout<<"idx loop is"<<idx_loop<<std::endl;
              idx_loop %=3;
              // std::cout<<"idx loop is"<<idx_loop<<" , state is"<<pid.state<<std::endl;              
              pid.twiddle(err, idx_loop);
            } 

            // std::cout<<"idx loop is"<<idx_loop<<" state is"<<pid.state<<std::endl;
            if (pid.tunecheck()){
              std::cout<<"done"<<std::endl;
            }
            else{
              throttle = 0.3; //reset throttle
              cnt = 0;
              err = 0;  // reset error
              pid.int_cte = 0; // reset integral error
              pid.prev_cte = 0; //reset previous cte error             
            }
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }

        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}