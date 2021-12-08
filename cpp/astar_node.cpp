/**
**  Astar ROS
**/
#include <ros/ros.h>
#include <asl_turtlebot/AStar.h>
#include <math.h>       /* round, floor, ceil, trunc */
#include "asl_turtlebot/AStarOutput.h"
#include "asl_turtlebot/StochOccupancyGrid.h"

#include <set>
#include <map>
#include <vector>

std::pair<int,int> snap_to_grid(std::pair<double, double> x, int resolution) {
	std::pair<int, int> newX;
	newX[0] = resolution * round(x[0] / resolution);
	newX[1] = resolution * round(x[1] / resolution);
	return newX;
}

bool isFree(std::pair<int, int> state, double window_size, double resolution, int width, int height, int origin_x, int origin_y) {
	double p_total = 1.0;
	int lower = -1 * (int) round((window_size - 1) / 2);
	int upper = (int) round((window_size - 1) / 2);
	for (int dx = lower; dx < upper + 1; dx++) {
		for (int dy = lower, dy < upper + 1; dy++) {
			std::pair<double, double> old_xy = {state[0] + resolution * dx, state[1] + resolution * dy};
			std::pair<int, int> xy = snap_to_grid(old_xy);
			int grid_x = (int) ((x - origin_x) / resolution);
			int grid_y = (int) ((y - origin_y) / resolution);

			if (xy[0] > 0 && xy[1] > 0 && xy[0] < width && xy[1] < height) {
				p_total *= (1.0 - max(0, (float) probs[grid_y * width + grid_x] / 100))
			}
		}
	}
	return (1 - p_total) < 0.5;

struct AStarAlgorithmParams {
	set<std::pair<int, int>> closed_set;
	set<std::pair<int, int>> open_set;

	map<std::pair<int, int>, int> est_cost_through;
	map<std::pair<int, int>, int> cost_to_arrive;
	map<std::pair<int, int>, std::pair<int, int>> came_from;

	vector<std::pair<int, int>> path;
}

bool is_state_free(x, AStar::Request &input_params) {
	return isFree(x) && x[0] > 0 && x[1] > 0 && x[0] < input_params.width && x[1] < input_params.height;
}

std::vector<std::pair<int, int>> get_neighbors(AStar::Request input, int r, std::pair<int, int> xy) {
	std::vector<std::pair<int, int>> offsets = {{r, 0}, {-r, 0}, {0, r}, {0, -r}, {r,r}, {-r, r}, {r, -r}, {-r, -r}};
	std::vector<std::pair<int, int>> output = {};
	for (auto o : offsets) {
		std::pair<int, int> new_point  = {xy[0] + o[0], xy[1] + o[1]};
		if (is_state_free(new_point, input)) {
			output.push_back(new_point);
		}
	}
	return output;
}

double distance(std::pair<int, int> x1, std::pair<int, int> x2) {
	return (x1[0] - x2[0]) * (x1[0] - x2[0]) + (x1[1] - x2[1]) * (x1[1] - x2[1])
}

std::pair<int, int> find_best_est_cost_through(AStarAlgorithmParams params) {
	double min = 100 * 10000;
	std::pair<int, int> min_x;
	for (auto x : params.open_set) {
		if (params.est_cost_through[x] < min) {
			min_x = x;
			min = params.est_cost_through[x];
		}
	}
	return min_x;
}

void reconstruct_path(AStarAlgorithmParams &params, AStar::Request req) {
	params.path = {{req.x_goal[0], req.x_goal[1]}}; // reset the path
	auto current_point = params.path[0];
	while (current_point != {req.x_init[0], req.x_init[1]}) {
		params.path.push_back(params.came_from[current_point]);
		current_point = params.came_from[current_point];
	}
	std::reverse(params.path.begin(), params.path.end());
}

nav_msgs::AStarOutput create_path(AStarAlgorithmParams &params) {
	int* x_values = new int[params.path.length()];
	int* y_values = new int[params.path.length()];
	for (int i = 0; i < params.path.length; i++) {
		x_values[i] = params.path[i];
		y_values[i] = params.path[i];
	}
	nav_msgs::AStarOutput output;
	output.x_values = x_values;
	output.y_values = y_values;
	return output;
}

bool astar_path_plan(AStar::Request &req, AStar::Response &res) {
	AStarAlgorithmParams algo_params;
	std::pair<int, int> x_init = {req.x_init[0], req.x_init[1]};
	std::pair<int, int> x_goal = {req.x_goal[0], req.x_goal[1]};

	algo_params.open_set.add(x_init);
	algo_params.cost_to_arrive[x_init] = 0;
	self.est_cost_through[x_init] = distance(x_init, x_goal);

	while (algo_params.open_set.size() > 0) {
		std::pair<int, int> x_current;
		int min = 100000;
		for (auto point : algo_params.est_cost_through) {
			if (algo_params.open_set.count(point) != 1) {
				continue;
			}
			if (algo_params.est_cost_through[point] < min) {
				x_current = point;
				min = algo_params.est_cost_through[point];
			}	

		}

		if (x_current == x_goal) {
			reconstruct_path(algo_params, req);
			LOG_INFO("Solution found");
			res.status = true;
			res.output_path = create_path(algo_params);
			return true;
		}

		algo_params.open_set.remove(x_current);
		algo_params.closed_set.add(x_current);

		for (auto neighbor : get_neighbors(x_current)) {
			if (algo_params.closed_set.contains(neighbor)) continue;

			tentative_cost_to_arrive = algo_params.cost_to_arrive[x_current] + distance(x_current, neighbor);
			if (algo_params.open_set.contains(x_neighbor) != 1) {
				algo_params.open_set.add(x_neighbor);
			}
			else if (tentative_cost_to_arrive > algo_params.cost_to_arrive[neighbor]) continue;

			algo_params.came_from[neighbor] = x_current;
			algo_params.cost_to_arrive[neighbor] = tentative_cost_to_arrive;
			algo_params.est_cost_through[neighbor] = tenattive_cost_to_arrive + distance(x_neighbor, x_goal);
		}
	}
	res.status = false;
	return false;
}

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "astar_node");
  ros::NodeHandle nh;

  ROS_INFO("Initializing astar node.");
  ros::ServiceServer service = n.advertiseService("astar_planner", astar_path_plan);
  ROS_INFO("Advertising astar_planner");

  // Don't exit the program.
  ros::spin();
}
