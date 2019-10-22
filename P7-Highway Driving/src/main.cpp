#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

#define MIN_DIST_FOR_COLL 30.0
double ref_vel = 0;
int current_lane = 1;




int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
iss >> d_x;
iss >> d_y;
map_waypoints_x.push_back(x);
map_waypoints_y.push_back(y);
map_waypoints_s.push_back(s);
map_waypoints_dx.push_back(d_x);
map_waypoints_dy.push_back(d_y);
	}




	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
		&map_waypoints_dx, &map_waypoints_dy]
		(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side 
					//   of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					/**
					* TODO: define a path made up of (x,y) points that the car will visit
					*   sequentially every .02 seconds
					*/
					int prev_path_size = previous_path_x.size();
					bool too_close = false;

					bool lane_safe[3] = { true,true,true };


					if (prev_path_size > 0)
					{
						car_s = end_path_s;
					}
					for (int i = 0; i < sensor_fusion.size(); i++)
					{
						//if car in my line
						float d = sensor_fusion[i][6];

						if ((d > 0) && (d < 12))
						{
							for (unsigned char lane_num = 0; lane_num < 3; lane_num++)
							{
								//consider only cars in the same lane
								if (d < (2 + 4 * lane_num + 2) && d >(2 + 4 * lane_num - 2))
								{
									double vx = sensor_fusion[i][3];
									double vy = sensor_fusion[i][4];
									double vehicle_s = sensor_fusion[i][5];
									double v_speed = sqrt((vx*vx) + (vy*vy));

									//new s = previous path length*speed*time elapsed
									vehicle_s += prev_path_size * (0.02)*v_speed;
									
									
									
									
									if (lane_num == current_lane)
									{
										if ( (vehicle_s > car_s) && ((vehicle_s - car_s) < MIN_DIST_FOR_COLL))
										{
											lane_safe[lane_num] = false;
										}
									}
									else
									{
										if(
											(abs(vehicle_s - car_s) < MIN_DIST_FOR_COLL) ||
											(abs(car_s - vehicle_s) < MIN_DIST_FOR_COLL)
										)
										{
											lane_safe[lane_num] = false;
										}
									}
									
								}
							}
						}
					}

					if (lane_safe[current_lane] == false)
					{
						ref_vel -= 0.224;

						switch (current_lane)
						{
						case 0:
							if (lane_safe[1] == true)
								current_lane = 1;
							break;

						case 1:
							if (lane_safe[0] == true)
							{
								current_lane = 0;
							}
							else
							{
								if (lane_safe[2] == true)
									current_lane = 2;
							}
							break;

						case 2:
							if (lane_safe[1] == true)
								current_lane = 1;
							break;

						default:
							break;
						}
					}
					else if (ref_vel < 49.6)
					{
						ref_vel += 0.224;
					}




					vector<double> pts_x;
					vector<double> pts_y;
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);


					//Collect last 2 points from the previous path
					if (prev_path_size < 2)
					{
						//use 2 points that make the path tangent to the car
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);

						pts_x.push_back(prev_car_x);
						pts_x.push_back(car_x);

						pts_y.push_back(prev_car_y);
						pts_y.push_back(car_y);

					}
					else
					{
						ref_x = previous_path_x[prev_path_size - 1];
						ref_y = previous_path_y[prev_path_size - 1];

						double prev_car_x = previous_path_x[prev_path_size - 2];
						double prev_car_y = previous_path_y[prev_path_size - 2];

						//get the angle
						ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);

						pts_x.push_back(prev_car_x);
						pts_x.push_back(ref_x);

						pts_y.push_back(prev_car_y);
						pts_y.push_back(ref_y);

					}

					// Add three 30m spaced points in future
					vector<double> next_wp0 = getXY((car_s + 30), (2 + 4 * current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY((car_s + 60), (2 + 4 * current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY((car_s + 90), (2 + 4 * current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

					pts_x.push_back(next_wp0[0]);
					pts_x.push_back(next_wp1[0]);
					pts_x.push_back(next_wp2[0]);

					pts_y.push_back(next_wp0[1]);
					pts_y.push_back(next_wp1[1]);
					pts_y.push_back(next_wp2[1]);


					for (int i = 0; i < pts_x.size(); ++i)
					{
						//shift car reference angle to 0 degree
						double shift_x = pts_x[i] - ref_x;
						double shift_y = pts_y[i] - ref_y;

						pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
						pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
					}

					tk::spline s;
					s.set_points(pts_x, pts_y);

					//push the previous path points
					for (int i = 0; i < previous_path_x.size(); i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					//let's choose x = 30
					double target_x = 30.0;

					//get the corresponding y point
					double target_y = s(target_x);
					double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

					double x_add_on = 0;


					for (int i = 0; i < 50 - previous_path_x.size(); i++)
					{
						//number of points
						double N = (target_dist / (0.02*ref_vel / 2.24));

						//new x point
						double x_point = x_add_on + (target_x / N);

						//correspoding y point of x point
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						// rotate back to normal
						x_point = ((x_ref * cos(ref_yaw)) - (y_ref * sin(ref_yaw)));
						y_point = ((x_ref * sin(ref_yaw)) + (y_ref * cos(ref_yaw)));

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}  // end "telemetry" if
			}
			else {
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
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}
