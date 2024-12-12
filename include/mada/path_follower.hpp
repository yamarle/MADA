#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>

#include <geometry_msgs/Twist.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <geometry_msgs/PolygonStamped.h>

#include <sensor_msgs/LaserScan.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <mada/mada_util.h>

using namespace std;

class PathFollower{

public:

	// Variables propias del robot
	double maxLinearVel = 0.55;
	double maxAngularVel = 1.0;
	double linearAcc = maxLinearVel/10; // (Valor de la página de ROS: 2.5)
	double angularAcc = maxAngularVel/10; // (Valor de la página de ROS: 3.2)
	double robotRadius = 0.5;

	// Variables que se utilizan para los cálculos
	double dx, dy, targetAngle, angleDiff, cost;
	double bestLinearVel, bestAngularVel,minCost;

	// (v,w) que se calculan para ir al goal
	double linearVel = 0.0;
	double angularVel = 0.0;

	// Para el chequeo de llegada al goal
	double goalDistDiff = 1.0;
	double goalAngleDiff = 1.0;

	//double sleepTime = 1/25; // ESTO ES UNA VARIABLE QUE DEBERÍA CALCULAR ??????
	double sleepTime = 0.1; // Periodo de envio de comandos de velocidad
	
	double goalRefTime; // Periodo con el que se refresca la distancia al goal
	double goalDist = 0.0; // Distancia al goal máxima seleccionada en cada iteración

	ros::Time tini, tend;

	vector<geometry_msgs::Pose> pathPoses; // Poses del camino a seguir
	nav_msgs::Path path;

	ros::NodeHandle n_;
	ros::Subscriber pathSub, poseSub, laserSub;
	ros::Publisher cmdVelPub;
	ros::Publisher totalPathPub; nav_msgs::Path totalPath;
	geometry_msgs::Twist cmd_vel; // Comandos de velocidad que se calculan
	geometry_msgs::Twist _cmd_vel; // Comandos de velocidad anteriores

	string robotPoseTopic, robotCmdVelTopic, laserTopic, footprintTopic, planTopic;
	string baseFrame;
	bool maxVels = false;

	ros::Publisher footprintPub;
	//geometry_msgs::PolygonStamped robotFootprint;
	visualization_msgs::Marker robotFootprint;
	ros::Publisher planPub;
	double sX = 0.0, sY = 0.0;

	vector<float> reg_time; // Instantes en los que se registran las posiciones y las distancias
	vector<float> min_obst_dist; // Variable que almacena la distancia mínima al obstáculo dinámico más cercano
	vector<geometry_msgs::Pose> min_obst_poses; // Pose del camino más cercana al obstáculo dinámico
	vector<float> dist2dest; 
	vector<geometry_msgs::Pose> goalPoses; // almacenar los destinos del robot
	vector<geometry_msgs::Pose> pathDestPoses; // almacenar el final de todos los caminos que recibe el path follower
	vector<geometry_msgs::Pose> robotPoses; // almacenar las poses del robot (para tener la misma cantidad que las poses de los caminos y goals)

	bool poseCh = false;
	bool laserCh = false;
	geometry_msgs::Pose robotPose, goalPose;
	sensor_msgs::LaserScan laserData;
	bool dwaPlanner = false;

	// Funciones
	PathFollower(); // Constructor por defecto
	~PathFollower();
	// args=[pathTopic robotPoseTopic sX sY robotCmdVelTopic laserTopic maxLinearVel maxAngularVel goalDistDiff goalAngleDiff | planner_type angularResolution maxObstacleDistance | footprintTopic planTopic
	PathFollower(string robotFrame, string robotPoseTopic, double sX, double sY, string robotCmdVelTopic, string laserTopic, double maxLinearVel, double maxAngularVel, double linearAcc, double angularAcc, double posDiff, double angleDiff, string planner_type, double angularResolution, double maxObstacleDistance, string footprintTopic, string planTopic); // Constructor para seguimiento simple
	void initializePlotting(string footprintTopic, double sX, double sY, string planTopic);


	// Cálculos
	geometry_msgs::PolygonStamped computeFootprint(geometry_msgs::Pose pose, double sX, double sY);
	nav_msgs::Path setPlan(geometry_msgs::Pose s, geometry_msgs::Pose g);

	geometry_msgs::Pose selectGoal(geometry_msgs::Pose pose, vector<geometry_msgs::Pose>& poses, double maxLenght, double &dist, double &dist2goal);

	void computeVelocities(const geometry_msgs::Pose goalPose, double dObstMaxNorm, double& linearVel, double& angularVel);
	void computeVelocitiesAcc(const geometry_msgs::Pose goalPose, double dObstMaxNorm, double& linearVel, double& angularVel);

	// Seguir camino sin tener en cuenta el entorno
	void initializeVariables(double maxLinearVel, double maxAngularVel, double linearAcc, double angularAcc, double goalDistDiff, double goalAngleDiff);
	void initializeTopics(string robotFrame, string robotPoseTopic, string robotCmdVelTopic);
	void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robotPoseMsg);
	void odomPoseCallback(const nav_msgs::Odometry::ConstPtr& robotPoseMsg);
	PathFollower(string robotFrame, string robotPoseTopic, string robotCmdVelTopic); // Constructor para seguimiento simple
	PathFollower(string robotFrame, string robotPoseTopic, string robotCmdVelTopic, double maxLinearVel, double maxAngularVel, double posDiff, double angleDiff); // Constructor para seguimiento simple
	
	// Las funciones que se usan para seguir el camino
	bool followPath(vector<geometry_msgs::Pose> path); // Sigue el camino
	bool followPathReal(vector<geometry_msgs::Pose> path); // Sigue el camino

};
