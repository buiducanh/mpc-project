#include "json.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

const double Lf = 2.67;
// for convenience
using json = nlohmann::json;

// overload << for cout to print vector
template <typename T>
ostream& operator<<(ostream& output, std::vector<T> const& values)
{
    for (auto const& value : values)
    {
        output << value << std::endl;
    }
    return output;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
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
    int order)
{
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

// Translate map coordinates to vehicle coordinates
vector<double> mapc2vehc(double x_map, double y_map, double psi,
    double xv_map, double yv_map)
{
  x_map -= xv_map;
  y_map -= yv_map;
  double x_veh = x_map * cos(psi) + y_map * sin(psi);
  double y_veh = y_map * cos(psi) - x_map * sin(psi);
  return { x_veh, y_veh };
}

// Translate vehicle coordinates to map coordinates
vector<double> vehc2mapc(double x_veh, double y_veh, double psi,
    double xv_map, double yv_map)
{
  double x_map = xv_map + (x_veh * cos(psi) - y_veh * sin(psi));
  double y_map = yv_map + (y_veh * cos(psi) + x_veh * sin(psi));
  return { x_map, y_map };
}

int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          for (int i = 0; i < ptsx.size(); i++) {
            vector<double> veh_c = mapc2vehc(ptsx[i], ptsy[i], psi, px, py);
            ptsx[i] = veh_c[0];
            ptsy[i] = veh_c[1];
          }
          Eigen::Map<Eigen::VectorXd> ptsx_eigen(&ptsx[0], ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_eigen(&ptsy[0], ptsy.size());

          Eigen::VectorXd coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);

          // plot reference trajectory
          int trajectory_len = 50; // meters
          int n_plot_points = 10;
          vector<double> ref_x_vals;
          vector<double> ref_y_vals;
          for (int i = 0; i < n_plot_points; i++) {
            double x_veh = i * (trajectory_len / n_plot_points);
            double y_veh = polyeval(coeffs, x_veh);
            ref_x_vals.push_back(x_veh);
            ref_y_vals.push_back(y_veh);
          }

          Eigen::VectorXd state(6);

          state << 0, 0, 0, v, polyeval(coeffs, 0), -atan(coeffs[1]);

          /*
          cout << endl
               << endl
               << "< ------------------------------------------------------------>" << endl
               << endl
               << endl
               << endl;
          cout << "mpC SOLVING"
               << endl
               << endl;
          cout << "< ------------------------------------------------------------>" << endl
               << endl
               << endl;
               */

          vector<vector<double>> result = mpc.Solve(state, coeffs);
          // convert map coordinates to vehicle coordinates
          double steer_value = result[0][0];
          double throttle_value = result[1][0];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals = result[2];
          vector<double> mpc_y_vals = result[3];

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = ref_x_vals;
          vector<double> next_y_vals = ref_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char* message, size_t length) {
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
