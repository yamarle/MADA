#ifndef MADA_UTIL
#define MADA_UTIL

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Para enviar goals al agente
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseActionGoal.h>


// ESTO HABRÁ QUE TOCARLO
//#define MIN_DIST 0.8
#define MIN_DIST 2.0
#define MAX_LINEAR_VEL 0.7
//#define MAX_LINEAR_VEL 0.1
#define MAX_ANGULAR_VEL 3.14
#define MIN_PROXIMITY_RANGE 0.5

#define GOAL_DISTANCE 0.35
#define GOAL_ANGLE 0.05
#define ROBOT_SIZE 0.35

using namespace std;

namespace mada_util{

	struct Config{

	    int scen = -1;
	    string scen_type;
	    int seed = 0;
	    int vel_speed = 0;
	    string strat;
	    float occ_thres = 0.0;
	    int forget_dynamic_areas = 0;

	    void show();

		void readConfig(string result_filename);
	};

	void showPoints(vector<geometry_msgs::Point> points);

	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; // Para enviar los objetivos al robot
	bool sendGoal(geometry_msgs::Pose goalPose, string map_topic);

	// Utilidades
	double length(geometry_msgs::Point p);
	double length(geometry_msgs::Twist v);
	double lengthSquared(geometry_msgs::Point p);
	geometry_msgs::Point normalize(geometry_msgs::Point p);
	geometry_msgs::Twist normalize(geometry_msgs::Twist v);
	geometry_msgs::Point leftNormalVector(geometry_msgs::Point p);
	geometry_msgs::Point rightNormalVector(geometry_msgs::Point p);
	double angleTo(geometry_msgs::Point p1, geometry_msgs::Point p2);
	double normalizeAngle(double angle);
	double polarAngle(geometry_msgs::Point p);
	double toRadian(double angle);
	int sign(double value);
	vector<geometry_msgs::Pose> points2Poses(vector<geometry_msgs::Point> points);
	vector<geometry_msgs::Point> poses2Points(vector<geometry_msgs::Pose> poses);

	string robotName(string str);
	void setCostmapsParams(ros::NodeHandle nh, string node_name, string glc_name, string llc_name, string robot_name, float sX, float sY, float wX, float wY);
	void setLocalCostmapsParams(ros::NodeHandle nh, string node_name, string llc_name, string robot_name, float sX, float sY, float wX, float wY);

	// Posiciones en el mapa
	geometry_msgs::Pose getRobotPose();
	pair<double, double> getRobotPosition();
	pair<double, double> getRobotPosition(int robot_no);
	pair<double, double> getRobotMapPosition(int robot_no);
	vector<pair<double, double>> getTeamPositions(int team_size);
	pair<int, int> transformToGridCoordinates(const nav_msgs::OccupancyGrid& map, pair<double, double> robotPos);
	pair<int, int> transformToGridCoordinates(const nav_msgs::OccupancyGrid& map, geometry_msgs::Pose pose);
	geometry_msgs::Point transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, double x, double y);
	geometry_msgs::Point transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, pair<double, double> pos);
	vector<geometry_msgs::Point> transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, vector<geometry_msgs::Pose> poses);

	void toGridCoordinates(double x, double y, const nav_msgs::OccupancyGrid& map, double &gx, double &gy);
	vector<geometry_msgs::Pose> poses2GridCoordinates(vector<geometry_msgs::Pose> poses, const nav_msgs::OccupancyGrid& map);
	vector<geometry_msgs::Point> points2GridCoordinates(vector<geometry_msgs::Point> points, const nav_msgs::OccupancyGrid& map);

	void poses2GridCoordinates(vector<geometry_msgs::Pose> poses, const nav_msgs::OccupancyGrid& map, vector<int> &x, vector<int> &y);
	void points2GridCoordinates(vector<geometry_msgs::Point> poses, const nav_msgs::OccupancyGrid& map, vector<int> &x, vector<int> &y);

	geometry_msgs::Pose transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, pair<int, int> robotGridPos);
	geometry_msgs::Pose transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, int x, int y);
	geometry_msgs::Point tfGridPos2MapPoint(const nav_msgs::OccupancyGrid& map, int x, int y);
	vector<geometry_msgs::Point> tfGridPos2MapPos(const nav_msgs::OccupancyGrid& map, vector<int> x, vector<int> y);
	vector<geometry_msgs::PoseStamped> transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, vector<int> x, vector<int> y);
	vector<geometry_msgs::PoseStamped> poses2PoseStamped(vector<geometry_msgs::Pose> poses);
	vector<geometry_msgs::PoseStamped> poses2PoseStamped(vector<geometry_msgs::Pose> poses, string frame);

	void tfPoseStamped2GridCoordinates(vector<geometry_msgs::PoseStamped> poses, const nav_msgs::OccupancyGrid& map, vector<int> &x, vector<int> &y);

	vector<geometry_msgs::Point> buildGraphPoints(vector<int> x, vector<int> y, vector<vector<bool>> graph, nav_msgs::OccupancyGrid map);
	vector<geometry_msgs::Point> buildTreePoints(vector<int> x, vector<int> y, vector<vector<int>> tree, nav_msgs::OccupancyGrid map);
	vector<geometry_msgs::Point> linkPoints(vector<geometry_msgs::Point> points, vector<vector<bool>> graph);
	vector<geometry_msgs::Point> linkPoints(vector<geometry_msgs::Point> points, vector<vector<int>> tree);
	vector<geometry_msgs::Point> linkPoints(vector<geometry_msgs::Point> points, vector<int> branch);

	geometry_msgs::Point32 rotatePoint(geometry_msgs::Pose pose, double x, double y);

	vector<geometry_msgs::Point> footprint(geometry_msgs::Pose pose, double sx, double sy);
	vector<geometry_msgs::Point> footprintOr(geometry_msgs::Pose pose, double sx, double sy);
	geometry_msgs::PolygonStamped footprintNoOr(geometry_msgs::Pose pose, double sX, double sY, string baseFrame);
	vector<geometry_msgs::Point> footprintNoOr(geometry_msgs::Pose pose, double sX, double sY);
	vector<geometry_msgs::Point> footprintOr(geometry_msgs::Pose pose, double sx, int num_points);
	vector<geometry_msgs::Point> footprintNoOr(geometry_msgs::Pose pose, double sx, int num_points);
	geometry_msgs::PolygonStamped pointsToPolygonStamped(vector<geometry_msgs::Point> points, string frame);
	vector<geometry_msgs::Point> circlePoints(double x, double y, double r, double rs);
	vector<geometry_msgs::Point> circlePointsTr(double radius, int num_points);

	bool checkDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2, float d, float ad);
	pair<float, float> poseDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
	float pointDist(geometry_msgs::Point p1, geometry_msgs::Point p2);
	float orientationDist(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

	geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2);
	geometry_msgs::Pose calculateNextPosition(const geometry_msgs::Pose& currentPose, float velocity, float time_delta);

	// Laser
	void _laserPosCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Point>& laserPos);
	void laserPosCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Point>& laserPos);
	void laserPosesCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Pose>& laserPoses);
	vector<geometry_msgs::Pose> laserPosesTf(vector<geometry_msgs::Pose> laserPoses);
	vector<geometry_msgs::Pose> laserPosesTf(geometry_msgs::Pose robotPose, vector<geometry_msgs::Pose> laserPoses);

	void obstaclesFill(vector<int> &visible_obstacles_x, vector<int> &visible_obstacles_y, vector<geometry_msgs::Point> &visible_obstacles_pos, const nav_msgs::OccupancyGrid& map, vector<vector<float>> static_map, double robotRadius);

	// Mapa
	void _occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, vector<vector<int>>& occGrid);
	void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, nav_msgs::OccupancyGrid& map, vector<vector<int>>& occGrid);
	void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, nav_msgs::OccupancyGrid& map, vector<vector<int>>& occGrid, vector<vector<float>>& grid);
	vector<vector<int>> occupancyGridToVector(const nav_msgs::OccupancyGrid& occupancyGridMsg);
	vector<vector<int>> occupancyGrid2Grid(vector<vector<int>> occupancyGrid);
	void occupancyGrid2Grid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, vector<vector<int>>& occGrid, vector<vector<float>>& grid);
	vector<signed char> grid2occupancyGridData(vector<vector<float>> grid);
	vector<signed char> grid2occupancyGridData(vector<vector<int>> grid);
	vector<signed char> map2occupancyGridData(vector<vector<bool>> map);
	vector<signed char> map2occupancyGridData(vector<vector<int>> map);
	vector<signed char> map2occupancyGridData(vector<vector<float>> map);
	vector<signed char> grad2occupancyGridData(int x, int y, vector<vector<float>> grad);
	vector<signed char> grad2occupancyGridData(vector<vector<float>> grad);

	vector<vector<float>> occupancyGrid2FMatrix(nav_msgs::OccupancyGrid occupancyGridMsg);

	// Planificación de caminos
	vector<vector<float>> navigableGradient(vector<vector<float>> obst_grad, float size);
	vector<vector<float>> navigableGrid(vector<vector<float>> obst_grad, float size);
	nav_msgs::Path transformToMapPath(vector<int> x, vector<int> y, const nav_msgs::OccupancyGrid& mapMsg);
	vector<geometry_msgs::Pose> convertPointsToPoses(vector<geometry_msgs::Point> points);
	geometry_msgs::Pose computePose(geometry_msgs::Point p1, geometry_msgs::Point p2);
	float computePathDistance(const nav_msgs::Path& path);
	float computePathDistance(const nav_msgs::Path& path, int evp);
	float computePathDistance(const vector<geometry_msgs::Pose>& pathPoses);

	vector<geometry_msgs::Pose> path2Poses(nav_msgs::Path path);
	int noMovementPath(nav_msgs::Path path);
	int noMovePath(nav_msgs::Path path);
	pair<float, float> distPose2Path(geometry_msgs::Pose p, nav_msgs::Path path, int evp);

	vector<geometry_msgs::Pose> setFollowerPathPoses(vector<int> x, vector<int> y, vector<vector<float>> navArea, nav_msgs::OccupancyGrid mapMsg, vector<int> vox, vector<int> voy, vector<geometry_msgs::Pose> &path2GoalPoses);
	vector<geometry_msgs::Pose> setFollowerPathPoses_(vector<int> x, vector<int> y, vector<vector<float>> navArea, nav_msgs::OccupancyGrid mapMsg, vector<int> vox, vector<int> voy, vector<geometry_msgs::Pose> &path2GoalPoses);

	void dynObstPos(vector<int> laser_poss_x, vector<int> laser_poss_y, vector<geometry_msgs::Pose> laser_poses, vector<vector<float>> grid, int x, int y, float vrange, vector<int> &obst_poss_x, vector<int> &obst_poss_y, vector<geometry_msgs::Point> &obst_pos);
	void dynObstPos(vector<int> laser_poss_x, vector<int> laser_poss_y, vector<geometry_msgs::Pose> laser_poses, vector<vector<float>> original_grid, vector<int> all_laser_poss_x, vector<int> all_laser_poss_y, vector<vector<float>> &curr_grid, vector<int> &obst_poss_x, vector<int> &obst_poss_y, vector<geometry_msgs::Point> &obst_pos);

	// Navegación
	geometry_msgs::Twist computeVelocities(pair<double, double> start, pair<double, double> goal);
	geometry_msgs::Twist computeVelocities(pair<double, double> start, geometry_msgs::Point goal);
	pair<double, double> selectGoalFromPath(nav_msgs::Path path, pair<double, double> currPos);

	

	void showCmdVel(geometry_msgs::Twist cmd_vel);

	// Las funciones para el cálculo de las velocidades del path follower
	nav_msgs::Path setPlan(geometry_msgs::Pose s, geometry_msgs::Pose g, string topic);
	geometry_msgs::Pose selectDestination(geometry_msgs::Pose pose, vector<geometry_msgs::Pose>& poses, double robotRadius, double &dist, double &dist2goal);
	geometry_msgs::Twist computeVelocities(const geometry_msgs::Pose robotPose, const geometry_msgs::Pose goalPose, double dObstMaxNorm, double maxLinearVel, double maxAngularVel);
	geometry_msgs::Twist computeVelocitiesAcc(const geometry_msgs::Pose robotPose, const geometry_msgs::Pose goalPose, double dObstMaxNorm, double maxLinearVel, double maxAngularVel, double prevLinearVel, double prevAngularVel, double linearAcc, double angularAcc);

	void count(vector<vector<int>> grid);
	

	double distanceToObstacle(double x, double y, double xl, double yl, sensor_msgs::LaserScan scan, double laser_range_threshold);
	double distanceToObstacles(double x, double y, double xl, double yl, sensor_msgs::LaserScan scan, double laser_range_threshold);
	geometry_msgs::Twist dynamicWindowApproach(const geometry_msgs::Pose startPose, const geometry_msgs::Pose goalPose, sensor_msgs::LaserScan scan, double laser_range_threshold, double current_v, double current_w, double min_v, double max_v, double max_w, double max_linear_acc, double max_angular_acc, double dt, double obstacleRadius);
	geometry_msgs::Twist computeVelocitiesDWA(const geometry_msgs::Pose startPose, const geometry_msgs::Pose goalPose, sensor_msgs::LaserScan scan, double laser_range_threshold, double current_v, double current_w, double min_v, double max_v, double max_w, double max_linear_acc, double max_angular_acc, double dt, double obstacleRadius);

	// Representación
	visualization_msgs::MarkerArray setTreeLines(vector<geometry_msgs::Point> points, vector<vector<int>> tree, vector<float> colors, string map_topic, string tree_frame_id, float resolution);
	void clearTreeLines(visualization_msgs::MarkerArray &lines);

};

#endif
