#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using Eigen::VectorXd;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];   // 6 points in global coordinate
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];                // also in global coordinate
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          const double Lf = 2.67;

          // std::cout << "x coordinate:" << ptsx[0] << ',' << px << std::endl;

          // use px and py as origin(0,0), and psi = 0
          // transform ptsx and ptxy from global coordinate to vehicle coordinate
          VectorXd x(ptsx.size());
          VectorXd y(ptsy.size());

          for (int i = 0; i < ptsx.size(); i++){
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;
            x[i] = shift_x * cos(0-psi) - shift_y * sin(0-psi);
            y[i] = shift_x * sin(0-psi) + shift_y * cos(0-psi);
          }

          // fit a polynomial to the x,y coordinates
          auto coeffs = polyfit(x, y, 3);

          // calculate the cross track error
          double fx = polyeval(coeffs, 0);   // px = 0
          double cte = fx - 0;               // py = 0

          // calculate the orientation error
          // derivative_coeffs = coeffs(1) + 2 * coeffs(2) * px + 3 * coeffs(3) * px * px;
          // since px = 0:
          double derivative_coeffs = coeffs(1);
          double desired_psi = atan(derivative_coeffs);
          double epsi = 0 - desired_psi;   // psi = 0

          // Latency (100ms latency = 0.1 seconds)
          // Instead of using the current state, we use the state that is 100ms in the future
          const double latency = 0.1;
          px = 0 + v * cos(0) * latency;
          py = 0 + v * sin(0) * latency;
          psi = 0 + v / Lf * -delta * latency;      // The sign of delta needs to be inverted because we are getting j[1]["steering_angle"] from the simulator.
                                                    // In the simulator, “left” is negative and “right” is positive, while psi is measured the other
                                                    // way around, i.e. “left” is negative in simulator (but angles are measured counter-clockwise).
          v += a * latency;
          cte += v * sin(epsi) * latency;
          epsi += v / Lf * -delta * latency;

          VectorXd state(6);

          state << px, py, psi, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          double steer_value = vars[0] / deg2rad(25);
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          //   steering value back. Otherwise the values will be in between
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value;   // because steering value from the simulator is in the opposite direction
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Green line
           */
          for(int i = 2; i < vars.size(); i+=2){
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Yellow line
           */
          int num_waypoints = 61;
          for (int i = 4; i < num_waypoints; i+=4){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
