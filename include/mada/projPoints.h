
#include <mada/util/funciones.hpp>
#include <mada/util/generic_fmm.hpp>

#include <mada/mada_util.h>

namespace pointProj{

	Poss<int> points2Poss(vector<geometry_msgs::Point> points);
	Poss<int> points2Poss(vector<geometry_msgs::Point> points, const nav_msgs::OccupancyGrid& map);
	vector<Poss<int>> points2Poss(vector<vector<geometry_msgs::Point>> points, const nav_msgs::OccupancyGrid& map);
	Poss<int> poses2Poss(vector<geometry_msgs::Pose> points);

	vector<geometry_msgs::Point> poss2Points(vector<Poss<int>> poss, const nav_msgs::OccupancyGrid& map);
	vector<geometry_msgs::Point> poss2Points(Poss<int> poss, const nav_msgs::OccupancyGrid& map, float time);
	vector<vector<geometry_msgs::Point>> poss2Points(vector<Poss<int>> poss, const nav_msgs::OccupancyGrid& map, float time);
	vector<vector<geometry_msgs::Point>> appendTrajectories(vector<vector<geometry_msgs::Point>> points, vector<vector<geometry_msgs::Point>> trajectories, float r);
	vector<int> appendTrajectoriesInd(vector<geometry_msgs::Point> centroids, vector<vector<geometry_msgs::Point>> points, vector<vector<geometry_msgs::Point>> &centroids_trajectories, vector<vector<geometry_msgs::Point>> &trajectories, float r);
	void clearTrajectories(vector<vector<geometry_msgs::Point>> &centroids_trajectories, vector<vector<geometry_msgs::Point>> &points_trajectories, vector<geometry_msgs::Point> laser_pos);
	vector<vector<float>> getTimes(vector<vector<geometry_msgs::Point>> trajectories);
	vector<vector<vector<geometry_msgs::Point>>> separateTrajectories(vector<vector<geometry_msgs::Point>> trajectories);
	geometry_msgs::Point centroid(vector<geometry_msgs::Point> points);
	geometry_msgs::Point equidistantPoint(vector<geometry_msgs::Point> points);
	vector<geometry_msgs::Point> clustersCentroids(vector<vector<geometry_msgs::Point>> points);
	float getRadius(geometry_msgs::Point point, vector<geometry_msgs::Point> points);
	vector<float> computeDimensions(vector<geometry_msgs::Point> centroids, vector<vector<geometry_msgs::Point>> points);
	vector<vector<float>> computeDimensions(vector<vector<geometry_msgs::Point>> centroids, vector<vector<vector<geometry_msgs::Point>>> points);
	vector<float> computeLinearVelocitiesT(vector<geometry_msgs::Point> points, float T);
	geometry_msgs::Twist computeCmdVel(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, double time_delta);
	vector<geometry_msgs::Twist> computeVelocitiesT(vector<geometry_msgs::Pose> poses, float T);
	vector<geometry_msgs::Twist> computeVelocitiesDT(vector<geometry_msgs::Pose> poses, float time, float dist);
	vector<geometry_msgs::Twist> estimateVelocities(vector<geometry_msgs::Pose> poses, float time, float dist);
	vector<geometry_msgs::Twist> smoothVelocities(vector<geometry_msgs::Twist> velocities, int ws);
	vector<geometry_msgs::Twist> computeVelocitiesTimes(vector<geometry_msgs::Pose> poses, vector<float> regTimes, float T);
	geometry_msgs::Twist computeMeanVelocity(vector<geometry_msgs::Twist> cmd_vels, int it);
	float computeVelocity(nav_msgs::Path path, vector<float> timeReg, float T);
	float closestDist(float x, float y, vector<geometry_msgs::Point> points);
	float time2ClosestPoint(float x, float y, vector<geometry_msgs::Point> points, float velocity);
	float time2FarthestPoint(float x, float y, vector<geometry_msgs::Point> points, float velocity);
	geometry_msgs::Quaternion calculateOrientation(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2);
	void showPoints(vector<geometry_msgs::Point> points);
	vector<geometry_msgs::Point> poses2Points(vector<geometry_msgs::Pose> poses);
	vector<geometry_msgs::Point> poses2Points(vector<vector<geometry_msgs::Pose>> poses);
	vector<geometry_msgs::Point> points2Plot(vector<geometry_msgs::Point> points);
	vector<geometry_msgs::Point> points2Plot(vector<vector<geometry_msgs::Point>> points);
	geometry_msgs::Pose point2Pose(geometry_msgs::Point point1, geometry_msgs::Point point2);
	vector<geometry_msgs::Pose> points2Poses(vector<geometry_msgs::Point> points);
	vector<geometry_msgs::Pose> points2Poses(vector<geometry_msgs::Point> points, float time_delta);
	vector<geometry_msgs::Pose> points2Poses(vector<geometry_msgs::Point> points, float time_delta, float time);
	vector<geometry_msgs::Pose> trajectories2Poses(vector<vector<geometry_msgs::Point>> trajectories_points);
	geometry_msgs::Pose calculateNextPosition(const geometry_msgs::Pose& currentPose, float velocity, float time_delta);
	geometry_msgs::Pose calculateNextPosition(const geometry_msgs::Pose& currentPose, geometry_msgs::Twist cmdVel, float time_delta);
	geometry_msgs::Pose calculateNextPose(const geometry_msgs::Pose& currentPose, geometry_msgs::Twist cmdVel, float time_delta, float max_dist);
	Poss<int> allLaserPoss(int x, int y, Poss<int> laserPoss);
	vector<vector<geometry_msgs::Point>> distance_clustering(vector<geometry_msgs::Point> points, float r);
	Poss<int> projectedPoss(vector<vector<geometry_msgs::Point>> obst_clusters_points, vector<vector<geometry_msgs::Point>> &obst_centroids_trajectories, vector<vector<geometry_msgs::Point>> &obst_clusters_trajectories, float domin, float obstaclesRegTime, float velT, float velTimeOrig, vector<vector<float>> navArea, vector<geometry_msgs::Point> laser_pos, nav_msgs::OccupancyGrid map_msg, float robotRadius, float robotLinearVel, float &maxObstRadius, float &maxObstVel, vector<geometry_msgs::Point> &res_points, vector<Poss<int>> &projTrajPoss);
	vector<Poss<int>> joinProjWithSet(vector<Poss<int>> proj_poss, vector<Poss<int>> set);

	Poss<int> projection(geometry_msgs::Pose robotPose, vector<geometry_msgs::Point> visible_obstacles_pos, nav_msgs::Path robotPath, vector<vector<geometry_msgs::Point>> &obst_clusters_points, vector<vector<geometry_msgs::Point>> &obst_centroids_trajectories, vector<vector<geometry_msgs::Point>> &obst_clusters_trajectories, vector<float> timeReg, float domin, float obstaclesRegTime, float velT, float velTimeOrig, vector<vector<float>> navArea, vector<geometry_msgs::Point> laser_pos, nav_msgs::OccupancyGrid map_msg, float robotRadius, float robotLinearVel, float &maxObstRadius, float &maxObstVel, vector<geometry_msgs::Point> &all_proj_obst_points, vector<Poss<int>> &projTrajPoss);

	Poss<int> focusProjection(geometry_msgs::Pose robotPose, vector<geometry_msgs::Point> visible_obstacles_pos, vector<vector<float>> map, float robotRadius, nav_msgs::OccupancyGrid mapMsg, vector<vector<geometry_msgs::Point>> &obst_clusters_points, vector<geometry_msgs::Point> &all_proj_obst_points, vector<Poss<int>> &projTrajPoss);

}