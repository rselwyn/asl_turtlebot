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
	newX.first = resolution * round(x.first / resolution);
	newX.second = resolution * round(x.second / resolution);
	return newX;
}

bool isFree(std::pair<int, int> state, double window_size, double resolution, int width, int height, int origin_x, int origin_y) {
	double p_total = 1.0;
	int lower = -1 * (int) round((window_size - 1) / 2);
	int upper = (int) round((window_size - 1) / 2);
	for (int dx = lower; dx < upper + 1; dx++) {
		for (int dy = lower; dy < upper + 1; dy++) {
			std::pair<double, double> old_xy = {state.first + resolution * dx, state.second + resolution * dy};
			std::pair<int, int> xy = snap_to_grid(old_xy, resolution);
			int grid_x = (int) ((xy.first - origin_x) / resolution);
			int grid_y = (int) ((xy.second - origin_y) / resolution);

			if (xy.first > 0 && xy.second > 0 && xy.first < width && xy.second < height) {
				p_total *= (1.0 - max(0, (float) probs[grid_y * width + grid_x] / 100))
			}
		}
	}
	return (1 - p_total) < 0.5;
}

struct AStarAlgorithmParams {
	std::set<std::pair<int, int>> closed_set;
	std::set<std::pair<int, int>> open_set;

	std::map<std::pair<int, int>, int> est_cost_through;
	std::map<std::pair<int, int>, int> cost_to_arrive;
	std::map<std::pair<int, int>, std::pair<int, int>> came_from;

	std::vector<std::pair<int, int>> path;
};

bool is_state_free(std::pair<int,int> x, asl_turtlebot::AStar::Request &input_params) {
	return isFree(x) && x.first > 0 && x.second > 0 && x.first < input_params.width && x.second < input_params.height;
}

std::vector<std::pair<int, int>> get_neighbors(asl_turtlebot::AStar::Request input, int r, std::pair<int, int> xy) {
	std::vector<std::pair<int, int>> offsets = {{r, 0}, {-r, 0}, {0, r}, {0, -r}, {r,r}, {-r, r}, {r, -r}, {-r, -r}};
	std::vector<std::pair<int, int>> output = {};
	for (auto o : offsets) {
		std::pair<int, int> new_point  = {xy.first + o.first, xy.second + o.second};
		if (is_state_free(new_point, input)) {
			output.push_back(new_point);
		}
	}
	return output;
}

double distance(std::pair<int, int> x1, std::pair<int, int> x2) {
	return (x1.first - x2.first) * (x1.first - x2.first) + (x1.second - x2.second) * (x1.second - x2.second)
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

void reconstruct_path(AStarAlgorithmParams &params, asl_turtlebot::AStar::Request req) {
	params.path = {{req.x_goal.first, req.x_goal.second}}; // reset the path
	auto current_point = params.path.first;
	while (current_point != {req.x_init.first, req.x_init.second}) {
		params.path.push_back(params.came_from[current_point]);
		current_point = params.came_from[current_point];
	}
	std::reverse(params.path.begin(), params.path.end());
}

asl_turtlebot::AStarOutput create_path(AStarAlgorithmParams &params) {
	int* x_values = new int[params.path.size()];
	int* y_values = new int[params.path.size()];
	for (int i = 0; i < params.path.size(); i++) {
		x_values[i] = params.path[i];
		y_values[i] = params.path[i];
	}
	nav_msgs::AStarOutput output;
	output.x_values = x_values;
	output.y_values = y_values;
	return output;
}

bool astar_path_plan(asl_turtlebot::AStar::Request &req, asl_turtlebot::AStar::Response &res) {
	AStarAlgorithmParams algo_params;
	std::pair<int, int> x_init = {req.x_init.first, req.x_init.second};
	std::pair<int, int> x_goal = {req.x_goal.first, req.x_goal.second};

	algo_params.open_set.insert(x_init);
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
			ROS_INFO("Solution found");
			res.status = true;
			res.output_path = create_path(algo_params);
			return true;
		}

		algo_params.open_set.erase(x_current);
		algo_params.closed_set.insert(x_current);

		for (auto neighbor : get_neighbors(x_current)) {
			if (algo_params.closed_set.count(neighbor) == 1) continue;

			double tentative_cost_to_arrive = algo_params.cost_to_arrive[x_current] + distance(x_current, neighbor);
			if (algo_params.open_set.count(neighbor) != 1) {
				algo_params.open_set.insert(neighbor);
			}
			else if (tentative_cost_to_arrive > algo_params.cost_to_arrive[neighbor]) continue;

			algo_params.came_from[neighbor] = x_current;
			algo_params.cost_to_arrive[neighbor] = tentative_cost_to_arrive;
			algo_params.est_cost_through[neighbor] = tenattive_cost_to_arrive + distance(neighbor, x_goal);
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
  ros::ServiceServer service = nh.advertiseService("astar_planner", astar_path_plan);
  ROS_INFO("Advertising astar_planner");

  // Don't exit the program.
  ros::spin();
}
