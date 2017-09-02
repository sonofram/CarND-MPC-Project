#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "MPC.h"
#include <math.h>
#include <fstream>
#include <iostream>
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}



int main()
{
  uWS::Hub h;

  MPC mpc;

  //Initial parameters. Setting up reference path.

  //h.onMessage([&pid,&cte_data](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          //std::cout << "telemetry";
          vector<double> ptsx = j[1]["ptsx"];
		  vector<double> ptsy = j[1]["ptsy"];
		  double px = j[1]["x"];
		  double py = j[1]["y"];
		  double psi = j[1]["psi"];
		  double v = j[1]["speed"];
		  double steering_angle = j[1]["steering_angle"];
		  double throttle = j[1]["throttle"];
		  int N = ptsx.size();

		  // Convert to the vehicle coordinate system
		  Eigen::VectorXd ptsx_vc(N);
		  Eigen::VectorXd ptsy_vc(N);
		  for(int i = 0; i < N; i++) {
			ptsx_vc[i] = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
			ptsy_vc[i] = (ptsy[i] - py) * cos(psi) - (ptsx[i] - px) * sin(psi);
		  }

		  //std::cout << ptsx_vc;
		  //std::cout << ptsy_vc;

		  // TODO: fit a polynomial to the above x and y coordinates
		  auto coeffs = polyfit(ptsx_vc, ptsy_vc, 3);

		  // Calculate cte and epsi errors
		  double cte = polyeval(coeffs,0)-0 ;

		  double epsi = psi - atan(coeffs[1]) ;

		  std::cout << "cte: "<< cte << " epsi: " << epsi << std::endl;

		  Eigen::VectorXd state(6);
		  state << 0,0,0, v, cte, epsi;

		  std::vector<double> x_vals = {state[0]};
		  std::vector<double> y_vals = {state[1]};
		  std::vector<double> psi_vals = {state[2]};
		  std::vector<double> v_vals = {state[3]};
		  std::vector<double> cte_vals = {state[4]};
		  std::vector<double> epsi_vals = {state[5]};
		  std::vector<double> delta_vals = {};
		  std::vector<double> a_vals = {};

		  auto vars = mpc.Solve(state, coeffs);
          steering_angle = vars[0];
          throttle = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
         // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steering_angle/(deg2rad(25));
          msgJson["throttle"] = throttle;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (int i = 2; i < vars.size(); i ++) {
                   if (i%2 == 0) {
                     mpc_x_vals.push_back(vars[i]);
                   }
                   else {
                     mpc_y_vals.push_back(vars[i]);
                   }
          }//for loop ending

		 msgJson["mpc_x"] = mpc_x_vals;
		 msgJson["mpc_y"] = mpc_y_vals;

         vector<double> next_x_vals;
         vector<double> next_y_vals;

          for (double i = 0; i < 100; i += 3){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });



  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
  //h.onDisconnection([&h,&cte_data](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
	//std::cout << "Closing file and connection" << std::endl;
	ws.close();
	//if (cte_data.is_open()){
	//	cte_data.close();
	//}

    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
