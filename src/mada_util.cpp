
#include <mada/mada_util.h>

void mada_util::Config::show(){
    cout<<"CONFIG"<<endl;
    cout<<"Scenario : "<<scen<<endl;
    cout<<"Scenario type : "<<scen_type<<endl;
    cout<<"Seed : "<<seed<<endl;
    cout<<"Obstacles velocity : "<<vel_speed<<endl;
    cout<<"Strategy : "<<strat<<endl;
    cout<<"Occupation threshold : "<<occ_thres<<endl;
    cout<<"Forget areas : "<<forget_dynamic_areas<<endl;
}

void mada_util::Config::readConfig(string result_filename){

    //cout<<"FILENAME : "<<result_filename<<endl;
    int p = -1;
    p = result_filename.find("scen");
    if(p < 0) return;

    //home/yarik/resultados/scen10_typedense_seed20_vel0_stratdynamicProj_occ0.1_olv2.txt
    this->scen = stoi(result_filename.substr(p + 4, result_filename.find("_type")));
    p = result_filename.find("_type") + 5;
    this->scen_type = result_filename.substr(p, result_filename.find("_seed") - p);
    this->seed = stoi(result_filename.substr(result_filename.find("_seed") + 5, result_filename.find("_vel")));
    this->vel_speed = stoi(result_filename.substr(result_filename.find("_vel") + 4, result_filename.find("_strat")));
    p = result_filename.find("_strat") + 6;
    this->strat = result_filename.substr(p, result_filename.find("_occ") - p);
    this->occ_thres = stof(result_filename.substr(result_filename.find("_occ") + 4, result_filename.find("_olv")));
    this->forget_dynamic_areas = stoi(result_filename.substr(result_filename.find("_olv") + 4, result_filename.find(".txt")));

    return;
}

void mada_util::showPoints(vector<geometry_msgs::Point> points)
{
    for(int i=0; i<points.size(); i++){
        cout<<"( "<<points[i].x<<" , "<<points[i].y<<" ) - ";
    }
    cout<<endl;
}

bool mada_util::sendGoal(geometry_msgs::Pose goalPose, string map_topic)
{
    //tell the action client that we want to spin a thread by default
    mada_util::MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(3.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //goal.target_pose.header.frame_id = "base_link"; // Con respecto al robot
    goal.target_pose.header.frame_id = map_topic; // Coordenada global
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose = goalPose;

    ROS_INFO("Sending goal: (%f, %f, %f)", goalPose.position.x, goalPose.position.y, goalPose.orientation.w);
    ac.sendGoal(goal);

    //return ac.waitForResult();
    return true;
} // Envío del objetivo al planificador

double mada_util::length(geometry_msgs::Point p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

double mada_util::length(geometry_msgs::Twist v)
{
    return sqrt(v.linear.x * v.linear.x + v.angular.z * v.angular.z);
}

double mada_util::lengthSquared(geometry_msgs::Point p)
{
    return p.x * p.x + p.y * p.y + p.z * p.z;
}

geometry_msgs::Point mada_util::normalize(geometry_msgs::Point p)
{
    double len = length(p);
    if (len == 0) return p;

    p.x /= len;
    p.y /= len;
    p.z /= len;

    return p;
}

geometry_msgs::Twist mada_util::normalize(geometry_msgs::Twist v)
{
    double len = length(v);
    if (len == 0) return v;

    v.linear.x /= len;
    v.angular.z /= len;

    return v;
}

geometry_msgs::Point mada_util::leftNormalVector(geometry_msgs::Point p)
{
    geometry_msgs::Point res;
    res.x = -p.y;
    res.y = p.x;
    return res;
}

geometry_msgs::Point mada_util::rightNormalVector(geometry_msgs::Point p)
{
    geometry_msgs::Point res;
    res.x = p.y;
    res.y = -p.x;
    return res;
}

double mada_util::angleTo(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return mada_util::polarAngle(p2) - mada_util::polarAngle(p1);
}

double mada_util::normalizeAngle(double angle)
{
    while (angle <= -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;
    return angle;
}

double mada_util::polarAngle(geometry_msgs::Point p)
{
    return normalizeAngle(atan2(p.y, p.x));
}

int mada_util::sign(double value)
{
    if (value == 0)
        return 0;
    else if (value > 0)
        return 1;
    else
        return -1;
}

vector<geometry_msgs::Pose> mada_util::points2Poses(vector<geometry_msgs::Point> points)
{
    vector<geometry_msgs::Pose> res(points.size());
    for(int i=0; i<points.size(); i++)
        res[i].position = points[i];
    return res;
}

vector<geometry_msgs::Point> mada_util::poses2Points(vector<geometry_msgs::Pose> poses)
{
    vector<geometry_msgs::Point> res(poses.size());
    for(int i=0; i<poses.size(); i++)
        res[i] = poses[i].position;
    return res;
}

string mada_util::robotName(string str)
{
    string robot_name;
    int _p = -1;
    _p = str.find("/");
    if(_p >= 0){
        robot_name = str.substr(0, string("robot_").size() + 1) + "/";
    }
    return robot_name;
}

void mada_util::setCostmapsParams(ros::NodeHandle nh, string node_name, string glc_name, string llc_name, string robot_name, float sX, float sY, float wX, float wY)
{
    // Definir parámetros para el costmap global
    nh.setParam(node_name + "/" + glc_name + "/global_frame", "map");
    nh.setParam(node_name + "/" + glc_name + "/robot_base_frame", robot_name + "base_lin");
    nh.setParam(node_name + "/" + glc_name + "/update_frequency", 1.0);
    nh.setParam(node_name + "/" + glc_name + "/publish_frequency", 0.5);
    nh.setParam(node_name + "/" + glc_name + "/transform_tolerance", 0.2);
    nh.setParam(node_name + "/" + glc_name + "/static_map", true);
    nh.setParam(node_name + "/" + glc_name + "/rolling_window", false);

    nh.setParam(node_name + "/" + glc_name + "/footprint", "[[" + to_string(-sX/2) + "," + to_string(-sY/2) + "], [" + to_string(-sX/2) + "," + to_string(sY/2) + "], [" + to_string(sX/2) + "," + to_string(sY/2) + "], [" + to_string(sX/2) + ", 0.0], [" + to_string(sX/2) + ", " + to_string(-sY/2) + "]]");
    //nh.setParam(node_name + "/" + glc_name + "/inflation_radius", inflFact*robotRadius);

    // Definir parámetros para el costmap local
    nh.setParam(node_name + "/" + llc_name + "/global_frame", robot_name + "odom");
    nh.setParam(node_name + "/" + llc_name + "/robot_base_frame", robot_name + "base_link");
    nh.setParam(node_name + "/" + llc_name + "/update_frequency", 5.0);
    nh.setParam(node_name + "/" + llc_name + "/publish_frequency", 2.0);
    nh.setParam(node_name + "/" + llc_name + "/transform_tolerance", 0.2);
    nh.setParam(node_name + "/" + llc_name + "/static_map", false);
    nh.setParam(node_name + "/" + llc_name + "/rolling_window", true);
    nh.setParam(node_name + "/" + llc_name + "/width", wX);
    nh.setParam(node_name + "/" + llc_name + "/height", wY);
    nh.setParam(node_name + "/" + llc_name + "/resolution", 0.05);

    nh.setParam(node_name + "/" + llc_name + "/footprint", "[[" + to_string(-sX/2) + "," + to_string(-sY/2) + "], [" + to_string(-sX/2) + "," + to_string(sY/2) + "], [" + to_string(sX/2) + "," + to_string(sY/2) + "], [" + to_string(sX/2) + ", 0.0], [" + to_string(sX/2) + ", " + to_string(-sY/2) + "]]");
    //nh.setParam(node_name + "/" + llc_name + "/inflation_radius", inflFact*robotRadius);

    // Parámetros del sensor (láser) para el costmap local
    nh.setParam(node_name + "/" + llc_name + "/observation_sources", "base_scan");

    nh.setParam(node_name + "/" + llc_name + "/base_scan/topic", robot_name + "base_scan");
    nh.setParam(node_name + "/" + llc_name + "/base_scan/data_type", "LaserScan");
    nh.setParam(node_name + "/" + llc_name + "/base_scan/expected_update_rate", 0.4);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/observation_persistence", 0.0);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/marking", true);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/clearing", true);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/max_obstacle_height", 0.4);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/min_obstacle_height", 0.08);
}

void mada_util::setLocalCostmapsParams(ros::NodeHandle nh, string node_name, string llc_name, string robot_name, float sX, float sY, float wX, float wY)
{
    // Definir parámetros para el costmap local
    nh.setParam(node_name + "/" + llc_name + "/global_frame", robot_name + "odom");
    nh.setParam(node_name + "/" + llc_name + "/robot_base_frame", robot_name + "base_link");
    nh.setParam(node_name + "/" + llc_name + "/update_frequency", 5.0);
    nh.setParam(node_name + "/" + llc_name + "/publish_frequency", 2.0);
    nh.setParam(node_name + "/" + llc_name + "/transform_tolerance", 0.2);
    nh.setParam(node_name + "/" + llc_name + "/static_map", false);
    nh.setParam(node_name + "/" + llc_name + "/rolling_window", true);
    nh.setParam(node_name + "/" + llc_name + "/width", wX);
    nh.setParam(node_name + "/" + llc_name + "/height", wY);
    nh.setParam(node_name + "/" + llc_name + "/resolution", 0.05);

    nh.setParam(node_name + "/" + llc_name + "/footprint", "[[" + to_string(-sX/2) + "," + to_string(-sY/2) + "], [" + to_string(-sX/2) + "," + to_string(sY/2) + "], [" + to_string(sX/2) + "," + to_string(sY/2) + "], [" + to_string(sX/2) + ", 0.0], [" + to_string(sX/2) + ", " + to_string(-sY/2) + "]]");
    //nh.setParam(node_name + "/" + llc_name + "/inflation_radius", inflFact*robotRadius);

    // Parámetros del sensor (láser) para el costmap local
    nh.setParam(node_name + "/" + llc_name + "/observation_sources", "base_scan");

    nh.setParam(node_name + "/" + llc_name + "/base_scan/topic", robot_name + "base_scan");
    nh.setParam(node_name + "/" + llc_name + "/base_scan/data_type", "LaserScan");
    nh.setParam(node_name + "/" + llc_name + "/base_scan/expected_update_rate", 0.4);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/observation_persistence", 0.0);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/marking", true);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/clearing", true);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/max_obstacle_height", 0.4);
    nh.setParam(node_name + "/" + llc_name + "/base_scan/min_obstacle_height", 0.08);
}

// ******************************************************************************************************************************
//                                                    Posiciones en el mapa
// ******************************************************************************************************************************

geometry_msgs::Pose mada_util::getRobotPose()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::Pose currPose;

    try {
        listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(1.0));
        // (frame a transformar, frame transformada, tiempo an el que se transforma, objeto que almacena la transformada)
        listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);

        currPose.position.x = transform.getOrigin().x(); // <----------- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        currPose.position.y = transform.getOrigin().y(); // <----------- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        currPose.position.z = transform.getOrigin().z();

        currPose.orientation.x = transform.getRotation().x();
        currPose.orientation.y = transform.getRotation().y();
        currPose.orientation.z = transform.getRotation().z();
        currPose.orientation.w = transform.getRotation().w();

        cout<<"POSICIÓN DEL ROBOT: ("<<currPose.position.x<<", "<<currPose.position.y<<")"<<endl;
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }

    return currPose;
}

pair<double, double> mada_util::getRobotPosition()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    pair<double, double> currPosition;

    try {
        listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        // (frame a transformar, frame transformada, tiempo an el que se transforma, objeto que almacena la transformada)
        listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);

        currPosition.first = transform.getOrigin().x();
        currPosition.second = transform.getOrigin().y();
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    return currPosition;
}

pair<double, double> mada_util::getRobotPosition(int robot_no)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    pair<double, double> currPosition;

    try {
    	string robot_str = "/robot_";
    	robot_str += boost::lexical_cast<string>(robot_no);
    	string base_footprint_frame = tf::resolve(robot_str, "base_footprint");

        listener.waitForTransform("/map", base_footprint_frame, ros::Time(0), ros::Duration(10.0));
        // (frame a transformar, frame transformada, tiempo an el que se transforma, objeto que almacena la transformada)
        listener.lookupTransform("/map", base_footprint_frame, ros::Time(0), transform);

        currPosition.first = transform.getOrigin().x();
        currPosition.second = transform.getOrigin().y();
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    return currPosition;
}

pair<double, double> mada_util::getRobotMapPosition(int robot_no) // ????????????????????????????????????????????????????????????????''
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    pair<double, double> currPosition;

    try {
        string robot_str = "/robot_";
        robot_str += boost::lexical_cast<string>(robot_no);
        string base_footprint_frame = tf::resolve(robot_str, "base_footprint");

        listener.waitForTransform(base_footprint_frame, "/map", ros::Time(0), ros::Duration(10.0));
        // (frame a transformar, frame transformada, tiempo an el que se transforma, objeto que almacena la transformada)
        listener.lookupTransform(base_footprint_frame, "/map", ros::Time(0), transform);

        currPosition.first = transform.getOrigin().x();
        currPosition.second = transform.getOrigin().y();
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    return currPosition;
}

pair<int, int> mada_util::transformToGridCoordinates(const nav_msgs::OccupancyGrid& map, pair<double, double> robotPos)
{
  // Calcular la posición del robot en coordenadas del grid
  pair<int, int> res;
  res.second = (robotPos.first - map.info.origin.position.x) / map.info.resolution;
  res.first = (robotPos.second - map.info.origin.position.y) / map.info.resolution;
  return res;
}

pair<int, int> mada_util::transformToGridCoordinates(const nav_msgs::OccupancyGrid& map, geometry_msgs::Pose pose)
{
  // Calcular la posición del robot en coordenadas del grid
  pair<int, int> res;
  res.second = (pose.position.x - map.info.origin.position.x) / map.info.resolution;
  res.first = (pose.position.y - map.info.origin.position.y) / map.info.resolution;

  res.second = (pose.position.x - map.info.origin.position.x) / map.info.resolution;
  res.first = (pose.position.y - map.info.origin.position.y) / map.info.resolution;
  return res;
}

geometry_msgs::Point mada_util::transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, double x, double y)
{
    geometry_msgs::Point res;
    res.x = (x - map.info.origin.position.x) / map.info.resolution;
    res.y = (y - map.info.origin.position.y) / map.info.resolution;
    return res;
}

geometry_msgs::Point mada_util::transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, pair<double, double> pos)
{
    geometry_msgs::Point res;
    res.x = (pos.first - map.info.origin.position.x) / map.info.resolution;
    res.y = (pos.second - map.info.origin.position.y) / map.info.resolution;
    return res;
}


vector<geometry_msgs::Point> mada_util::transform2GridCoordinates(const nav_msgs::OccupancyGrid& map, vector<geometry_msgs::Pose> poses)
{
    vector<geometry_msgs::Point> points(poses.size());
    for(int i=0; i<poses.size(); i++){
        points[i].x = (poses[i].position.x - map.info.origin.position.x) / map.info.resolution;
        points[i].y = (poses[i].position.y - map.info.origin.position.y) / map.info.resolution;
    }
    return points;
}

void mada_util::toGridCoordinates(double x, double y, const nav_msgs::OccupancyGrid& map, double &gx, double &gy)
{
    gx = (x - map.info.origin.position.x) / map.info.resolution;
    gy = (y - map.info.origin.position.y) / map.info.resolution;
}

vector<geometry_msgs::Pose> mada_util::poses2GridCoordinates(vector<geometry_msgs::Pose> poses, const nav_msgs::OccupancyGrid& map)
{
    for(int i=0; i<poses.size(); i++){
        poses[i].position.x = (poses[i].position.x - map.info.origin.position.x) / map.info.resolution;
        poses[i].position.y = (poses[i].position.y - map.info.origin.position.y) / map.info.resolution;
    }
    return poses;
}

vector<geometry_msgs::Point> mada_util::points2GridCoordinates(vector<geometry_msgs::Point> points, const nav_msgs::OccupancyGrid& map)
{
    for(int i=0; i<points.size(); i++){
        points[i].x = (points[i].x - map.info.origin.position.x) / map.info.resolution;
        points[i].y = (points[i].y - map.info.origin.position.y) / map.info.resolution;
    }
    return points;
}

void mada_util::poses2GridCoordinates(vector<geometry_msgs::Pose> poses, const nav_msgs::OccupancyGrid& map, vector<int> &x, vector<int> &y)
{
    x.resize(poses.size()); y.resize(poses.size());
    for(int i=0; i<poses.size(); i++){
        x[i] = (poses[i].position.y - map.info.origin.position.y) / map.info.resolution;
        y[i] = (poses[i].position.x - map.info.origin.position.x) / map.info.resolution;
    }
}

void mada_util::points2GridCoordinates(vector<geometry_msgs::Point> points, const nav_msgs::OccupancyGrid& map, vector<int> &x, vector<int> &y)
{
    x.resize(points.size()); y.resize(points.size());
    for(int i=0; i<points.size(); i++){
        x[i] = (points[i].y - map.info.origin.position.y) / map.info.resolution;
        y[i] = (points[i].x - map.info.origin.position.x) / map.info.resolution;
    }
}

vector<pair<double, double>> mada_util::getTeamPositions(int team_size)
{
	vector<pair<double, double>> currPositions(team_size);
	for (int i = 0; i < team_size; i++) {
		currPositions[i] = mada_util::getRobotPosition(i);
	}
	return currPositions;
}

geometry_msgs::Pose mada_util::transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, pair<int, int> robotGridPos)
{
    // Calcular la posición del robot en el mapa a partir de sus coordenadas del grid
    geometry_msgs::Pose res;
    res.position.x = robotGridPos.first * map.info.resolution + map.info.origin.position.x;
    res.position.y = robotGridPos.second * map.info.resolution + map.info.origin.position.y;
    res.position.z = 0;

    res.orientation.w = 1.0;
    return res;
}

geometry_msgs::Pose mada_util::transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, int x, int y)
{
    // Calcular la posición del robot en el mapa a partir de sus coordenadas del grid
    geometry_msgs::Pose res;
    res.position.x = x * map.info.resolution + map.info.origin.position.x;
    res.position.y = y * map.info.resolution + map.info.origin.position.y;
    res.position.z = 0;

    res.orientation.w = 1.0;

    return res;
}

geometry_msgs::Point mada_util::tfGridPos2MapPoint(const nav_msgs::OccupancyGrid& map, int x, int y)
{
    geometry_msgs::Point res;
    res.x = x * map.info.resolution + map.info.origin.position.x;
    res.y = y * map.info.resolution + map.info.origin.position.y;
    res.z = 0;
    return res;
}

vector<geometry_msgs::Point> mada_util::tfGridPos2MapPos(const nav_msgs::OccupancyGrid& map, vector<int> x, vector<int> y)
{
    vector<geometry_msgs::Point> res(x.size());
    for(int i=0; i<res.size(); i++){
        res[i].x = x[i] * map.info.resolution + map.info.origin.position.x;
        res[i].y = y[i] * map.info.resolution + map.info.origin.position.y;
        res[i].z = 0;
    }
    return res;
}

vector<geometry_msgs::PoseStamped> mada_util::transformFromGridCoordinates(const nav_msgs::OccupancyGrid& map, vector<int> x, vector<int> y)
{
    vector<geometry_msgs::PoseStamped> res(x.size());
    for(int i=0; i<res.size(); i++){
        res[i].pose.position.x = x[i] * map.info.resolution + map.info.origin.position.x;
        res[i].pose.position.y = y[i] * map.info.resolution + map.info.origin.position.y;
        res[i].pose.position.z = 0;
        // Aqui debería calcular la orientación
    }
    return res;
}

vector<geometry_msgs::PoseStamped> mada_util::poses2PoseStamped(vector<geometry_msgs::Pose> poses)
{
    vector<geometry_msgs::PoseStamped> res(poses.size());
    for(int i=0; i<res.size(); i++){
        res[i].pose = poses[i];
    }
    return res;
}

vector<geometry_msgs::PoseStamped> mada_util::poses2PoseStamped(vector<geometry_msgs::Pose> poses, string frame)
{
    vector<geometry_msgs::PoseStamped> res(poses.size());
    for(int i=0; i<res.size(); i++){
        //res[i].header.seq = 0;
        res[i].header.stamp = ros::Time::now();
        res[i].header.frame_id = frame;
        res[i].pose = poses[i];
    }
    return res;
}

void mada_util::tfPoseStamped2GridCoordinates(vector<geometry_msgs::PoseStamped> poses, const nav_msgs::OccupancyGrid& map, vector<int> &x, vector<int> &y)
{
    x.clear(); y.clear();
    for(int i=0; i<poses.size(); i++){
        x.push_back((poses[i].pose.position.x - map.info.origin.position.x)/map.info.resolution);
        y.push_back((poses[i].pose.position.y - map.info.origin.position.y)/map.info.resolution);
    }
}

vector<geometry_msgs::Point> mada_util::buildGraphPoints(vector<int> x, vector<int> y, vector<vector<bool>> graph, nav_msgs::OccupancyGrid map)
{
    vector<geometry_msgs::Point> points;
    for(int i=0; i<graph.size(); i++){
        for(int j=i+1; j<graph[i].size(); j++){
            if(graph[i][j]){
                points.push_back(mada_util::tfGridPos2MapPoint(map, x[i], y[i]));
                points.push_back(mada_util::tfGridPos2MapPoint(map, x[j], y[j]));
            }
        }
    }
    return points;
}

vector<geometry_msgs::Point> mada_util::buildTreePoints(vector<int> x, vector<int> y, vector<vector<int>> tree, nav_msgs::OccupancyGrid map)
{
    vector<geometry_msgs::Point> points;
    for(int i=0; i<tree.size(); i++){
        points.push_back(mada_util::tfGridPos2MapPoint(map, x[tree[i][0]], y[tree[i][0]]));
        points.push_back(mada_util::tfGridPos2MapPoint(map, x[tree[i][1]], y[tree[i][1]]));
    }
    return points;
}

vector<geometry_msgs::Point> mada_util::linkPoints(vector<geometry_msgs::Point> points, vector<vector<bool>> graph)
{
    vector<geometry_msgs::Point> res;
    for(int i=0; i<graph.size(); i++){
        for(int j=i+1; j<graph[i].size(); j++){
            if(graph[i][j]){
                res.push_back(points[i]);
                res.push_back(points[j]);
            }
        }
    }
    return res;
}

vector<geometry_msgs::Point> mada_util::linkPoints(vector<geometry_msgs::Point> points, vector<vector<int>> tree)
{
    vector<geometry_msgs::Point> res;
    for(int i=0; i<tree.size(); i++){
        res.push_back(points[tree[i][0]]);
        res.push_back(points[tree[i][1]]);
    }
    return res;
}

vector<geometry_msgs::Point> mada_util::linkPoints(vector<geometry_msgs::Point> points, vector<int> branch)
{
    vector<geometry_msgs::Point> res;
    if(branch.size())
        for(int i=0; i<branch.size()-1; i++){
            res.push_back(points[branch[i]]);
            res.push_back(points[branch[i+1]]);
        }
    return res;
}

geometry_msgs::Point32 mada_util::rotatePoint(geometry_msgs::Pose pose, double x, double y)
{
    geometry_msgs::Point32 p;
    double th = tf::getYaw(pose.orientation);
    double s = sin(th);
    double c = cos(th);
    p.x = c * (x - p.x) - s * (y - p.y) + x;
    p.y = s * (x - p.x) + c * (y - p.y) + y;
    return p;
}

vector<geometry_msgs::Point> mada_util::footprint(geometry_msgs::Pose pose, double sx, double sy)
{
    vector<geometry_msgs::Point> res;

    // Calcular los vértices del footprint como un rectángulo
    std::vector<geometry_msgs::Point> vertices;
    double half_sX = sx / 2;
    double half_sY = sy / 2;

    // Calcular los vértices del rectángulo no rotado
    tf::Vector3 p1(half_sX, half_sY, 0);
    tf::Vector3 p2(half_sX, -half_sY, 0);
    tf::Vector3 p3(-half_sX, -half_sY, 0);
    tf::Vector3 p4(-half_sX, half_sY, 0);

    // Rotar los vértices según la orientación del robot
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    p1 = tf::quatRotate(q, p1);
    p2 = tf::quatRotate(q, p2);
    p3 = tf::quatRotate(q, p3);
    p4 = tf::quatRotate(q, p4);

    // Trasladar los vértices según la posición del robot
    p1 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p2 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p3 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p4 += tf::Vector3(pose.position.x, pose.position.y, 0);

    // Convertir a geometry_msgs::Point y agregar al vector de puntos del marcador
    geometry_msgs::Point gp1, gp2, gp3, gp4;
    tf::pointTFToMsg(p1, gp1);
    tf::pointTFToMsg(p2, gp2);
    tf::pointTFToMsg(p3, gp3);
    tf::pointTFToMsg(p4, gp4);

    res.push_back(gp1);
    res.push_back(gp2);
    res.push_back(gp2);
    res.push_back(gp3);
    res.push_back(gp3);
    res.push_back(gp4);
    res.push_back(gp4);
    res.push_back(gp1);

    return res;
}

vector<geometry_msgs::Point> mada_util::footprintOr(geometry_msgs::Pose pose, double sx, double sy)
{
    vector<geometry_msgs::Point> res;

    // Calcular los vértices del footprint como un rectángulo
    std::vector<geometry_msgs::Point> vertices;
    double half_sX = sx / 2;
    double half_sY = sy / 2;

    // Calcular los vértices del rectángulo no rotado
    tf::Vector3 p1( half_sX,  half_sY, 0); // Arr, der
    tf::Vector3 p2( half_sX, -half_sY, 0); // Ab,  der
    tf::Vector3 p3(-half_sX, -half_sY, 0); // Ab,  izq
    tf::Vector3 p4(-half_sX,  half_sY, 0); // Arr, izq

    tf::Vector3 p5(half_sX, 0, 0); // Arr, centr

    // Rotar los vértices según la orientación del robot
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    p1 = tf::quatRotate(q, p1);
    p2 = tf::quatRotate(q, p2);
    p3 = tf::quatRotate(q, p3);
    p4 = tf::quatRotate(q, p4);

    p5 = tf::quatRotate(q, p5);

    // Trasladar los vértices según la posición del robot
    p1 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p2 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p3 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p4 += tf::Vector3(pose.position.x, pose.position.y, 0);

    p5 += tf::Vector3(pose.position.x, pose.position.y, 0);

    // Convertir a geometry_msgs::Point y agregar al vector de puntos del marcador
    geometry_msgs::Point gp1, gp2, gp3, gp4, gp5;
    tf::pointTFToMsg(p1, gp1);
    tf::pointTFToMsg(p2, gp2);
    tf::pointTFToMsg(p3, gp3);
    tf::pointTFToMsg(p4, gp4);

    tf::pointTFToMsg(p5, gp5);

    res.push_back(gp1);
    res.push_back(gp2);
    res.push_back(gp2);
    res.push_back(gp3);
    res.push_back(gp3);
    res.push_back(gp4);
    res.push_back(gp4);
    res.push_back(gp1);

    res.push_back(gp5);
    res.push_back(gp3);
    res.push_back(gp3);
    res.push_back(gp4);
    res.push_back(gp4);
    res.push_back(gp5);

    return res;

}

geometry_msgs::PolygonStamped mada_util::footprintNoOr(geometry_msgs::Pose pose, double sX, double sY, string baseFrame)
{
    geometry_msgs::PolygonStamped res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = baseFrame;
    geometry_msgs::Point32 p;
    
    //p.x = -sX/2; p.y = -sY/2; res.polygon.points.push_back(p);
    //p.x = -sX/2; p.y = sY/2; res.polygon.points.push_back(p);
    //p.x = sX/2; p.y = sY/2; res.polygon.points.push_back(p);
    //p.x = sX/2; p.y = -sY/2; res.polygon.points.push_back(p);

    p.x = -sX/2; p.y = -sY/2; res.polygon.points.push_back(p);
    p.x = sX/2; p.y = -sY/2; res.polygon.points.push_back(p);
    p.x = sX/2; p.y = 0; res.polygon.points.push_back(p);
    p.x = sX/2; p.y = sY/2; res.polygon.points.push_back(p);
    p.x = -sX/2; p.y = sY/2; res.polygon.points.push_back(p);
    p.x = sX/2; p.y = 0; res.polygon.points.push_back(p);
    p.x = -sX/2; p.y = -sY/2; res.polygon.points.push_back(p);
    p.x = -sX/2; p.y = sY/2; res.polygon.points.push_back(p);

    return res;
}

vector<geometry_msgs::Point> mada_util::footprintNoOr(geometry_msgs::Pose pose, double sX, double sY)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p;

    
    p.x = sX/2; p.y = -sY/2; res.push_back(p);
    p.x = sX/2; p.y = 0; res.push_back(p);
    p.x = sX/2; p.y = sY/2; res.push_back(p);
    p.x = -sX/2; p.y = sY/2; res.push_back(p);
    p.x = sX/2; p.y = 0; res.push_back(p);
    p.x = -sX/2; p.y = -sY/2; res.push_back(p);
    p.x = -sX/2; p.y = sY/2; res.push_back(p);
    p.x = -sX/2; p.y = -sY/2; res.push_back(p);

    return res;
}

vector<geometry_msgs::Point> mada_util::footprintOr(geometry_msgs::Pose pose, double radius, int num_points)
{
    vector<geometry_msgs::Point> res;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);

    // Calcular los puntos alrededor del círculo
    for (int i = 0; i < num_points; i++){
        double angle = 2 * M_PI * i / num_points;
        tf::Vector3 p(radius * cos(angle), radius * sin(angle), 0);

        // Rotar los puntos según la orientación del robot
        p = tf::quatRotate(q, p);

        // Trasladar los puntos según la posición del robot
        p += tf::Vector3(pose.position.x, pose.position.y, 0);

        geometry_msgs::Point gp;
        tf::pointTFToMsg(p, gp);
        res.push_back(gp);
    }

    // Puntos para el triángulo que indica la orientación
    tf::Vector3 p1(radius, 0, 0);           // Punto en el borde del círculo
    tf::Vector3 p2(0, radius, 0); // Punto intermedio arriba
    tf::Vector3 p3(0, -radius, 0); // Punto intermedio abajo
    //tf::Vector3 p2(radius * cos(2 * M_PI / 3), radius * sin(2 * M_PI / 3), 0); // Punto en el borde del círculo a 120 grados
    //tf::Vector3 p3(radius * cos(4 * M_PI / 3), radius * sin(4 * M_PI / 3), 0); // Punto en el borde del círculo a 240 grados


    // Rotar los puntos del triángulo según la orientación del robot
    p1 = tf::quatRotate(q, p1);
    p2 = tf::quatRotate(q, p2);
    p3 = tf::quatRotate(q, p3);

    // Trasladar los puntos según la posición del robot
    p1 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p2 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p3 += tf::Vector3(pose.position.x, pose.position.y, 0);

    // Convertir a geometry_msgs::Point y agregar al vector de puntos del marcador
    geometry_msgs::Point gp1, gp2, gp3;
    tf::pointTFToMsg(p1, gp1);
    tf::pointTFToMsg(p2, gp2);
    tf::pointTFToMsg(p3, gp3);

    // Agregar los puntos del triángulo al vector de resultados
    res.push_back(gp1);
    res.push_back(gp2);
    res.push_back(gp2);
    res.push_back(gp3);
    res.push_back(gp3);
    res.push_back(gp1);

    return res;
}

vector<geometry_msgs::Point> mada_util::footprintNoOr(geometry_msgs::Pose pose, double radius, int num_points)
{
    vector<geometry_msgs::Point> res;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);

    // Calcular los puntos alrededor del círculo
    for (int i = 0; i < num_points; i++){
        double angle = 2 * M_PI * i / num_points;
        tf::Vector3 p(radius * cos(angle), radius * sin(angle), 0);

        // Rotar los puntos según la orientación del robot
        p = tf::quatRotate(q, p);

        // Trasladar los puntos según la posición del robot
        p += tf::Vector3(pose.position.x, pose.position.y, 0);

        geometry_msgs::Point gp;
        tf::pointTFToMsg(p, gp);
        res.push_back(gp);
    }

    // Puntos para el triángulo que indica la orientación
    tf::Vector3 p1(radius, 0, 0);           // Punto en el borde del círculo
    tf::Vector3 p2(0, radius, 0); // Punto intermedio arriba
    tf::Vector3 p3(0, -radius, 0); // Punto intermedio abajo

    // Rotar los puntos del triángulo según la orientación del robot
    p1 = tf::quatRotate(q, p1);
    p2 = tf::quatRotate(q, p2);
    p3 = tf::quatRotate(q, p3);

    // Trasladar los puntos según la posición del robot
    p1 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p2 += tf::Vector3(pose.position.x, pose.position.y, 0);
    p3 += tf::Vector3(pose.position.x, pose.position.y, 0);

    // Convertir a geometry_msgs::Point y agregar al vector de puntos del marcador
    geometry_msgs::Point gp1, gp2, gp3;
    tf::pointTFToMsg(p1, gp1);
    tf::pointTFToMsg(p2, gp2);
    tf::pointTFToMsg(p3, gp3);

    // Agregar los puntos del triángulo al vector de resultados
    res.push_back(gp1);
    res.push_back(gp2);
    res.push_back(gp2);
    res.push_back(gp3);
    res.push_back(gp3);
    res.push_back(gp1);

    return res;
}

geometry_msgs::PolygonStamped mada_util::pointsToPolygonStamped(vector<geometry_msgs::Point> points, string frame)
{
    geometry_msgs::PolygonStamped res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = frame;
    geometry_msgs::Point32 p;

    for(int i=0; i<points.size(); i++){
        p.x = points[i].x;
        p.y = points[i].y;
        p.z = points[i].z;
        res.polygon.points.push_back(p);
    }

    return res;

}

vector<geometry_msgs::Point> mada_util::circlePoints(double x, double y, double r, double rs)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p1, p2;

    for (double angle = 0; angle < 2 * M_PI; angle += rs)
    {
        p1.x = x + r * cos(angle);
        p1.y = y + r * sin(angle);
        p1.z = 0.0;

        angle += 0.1;
        p2.x = x + r * cos(angle);
        p2.y = y + r * sin(angle);
        p2.z = 0.0;

        res.push_back(p1);
        res.push_back(p2);
    }
    return res;
}

vector<geometry_msgs::Point> mada_util::circlePointsTr(double radius, int num_points)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p1, p2, p3;

    for (int i = 0; i < num_points; i++) {
        double angle = 2 * M_PI * i / num_points;
        
        geometry_msgs::Point p;
        p.x = radius * cos(angle);
        p.y = radius * sin(angle);
        
        res.push_back(p);
    }

    p1.x = radius; p1.y = 0; p1.z = 0;   // Punto en el borde del círculo
    p2.x = 0; p2.y = radius; p2.z = 0;   // Punto intermedio arriba
    p3.x = 0; p3.y = -radius; p3.z = 0;  // Punto intermedio abajo

    // Agregar puntos del triángulo a la lista
    res.push_back(p1);
    res.push_back(p2);
    res.push_back(p2);
    res.push_back(p3);
    res.push_back(p3);
    res.push_back(p1);


    return res;
}

bool mada_util::checkDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2, float d, float ad)
{
    pair<float, float> dist = poseDistance(p1,p2);
    if(dist.first<=d && dist.second<=ad) return true;
    return false;
}

pair<float, float> mada_util::poseDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    pair<float, float> res;
    res.first = sqrt(pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2));
    float o1 = tf::getYaw(p1.orientation);
    float o2 = tf::getYaw(p2.orientation);
    res.second = abs(atan2(sin(o1-o2), cos(o1-o2)));
    return res;
}

float mada_util::pointDist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

float mada_util::orientationDist(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
    float o1 = tf::getYaw(q1);
    float o2 = tf::getYaw(q2);
    return abs(atan2(sin(o1-o2), cos(o1-o2)));
}

geometry_msgs::Quaternion mada_util::calculateOrientation(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2){
    float dx = point2.x - point1.x;
    float dy = point2.y - point1.y;
    float yaw = std::atan2(dy, dx);
    
    // Crear un cuaternión a partir del ángulo de yaw
    geometry_msgs::Quaternion orientation;
    orientation.w = std::cos(yaw / 2.0);
    orientation.z = std::sin(yaw / 2.0);
    
    return orientation;
} // Función para calcular la orientación a partir de dos puntos

geometry_msgs::Pose mada_util::calculateNextPosition(const geometry_msgs::Pose& currentPose, float velocity, float time_delta){
    geometry_msgs::Pose nextPose;
    nextPose.position.x = currentPose.position.x + velocity * time_delta * std::cos(tf::getYaw(currentPose.orientation));
    nextPose.position.y = currentPose.position.y + velocity * time_delta * std::sin(tf::getYaw(currentPose.orientation));
    nextPose.orientation = currentPose.orientation; // Mantener la orientación
    return nextPose;
} // Función para calcular la posición siguiente en movimiento rectilíneo uniforme

// ******************************************************************************************************************************
//                                                         Laser
// ******************************************************************************************************************************


void mada_util::_laserPosCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Point>& laserPos){
    //ROS_INFO("=====================> LaserScan (val,angle)=(%f,%f", msg->range_min,msg->angle_min);
    laserPos.clear();
    // Obtener los parámetros del láser scan
    float angle_min = laser_scan->angle_min;
    float angle_max = laser_scan->angle_max;
    float angle_increment = laser_scan->angle_increment;
    float range_max = laser_scan->range_max;
    float range_min = laser_scan->range_min;

    // Calcular la cantidad de muestras en el láser scan
    int num_samples = (angle_max - angle_min) / angle_increment;

    // Iterar sobre las muestras del láser scan
    for (int i = 0; i < num_samples; ++i){
        // Calcular el ángulo correspondiente a la muestra actual
        float angle = angle_min + i * angle_increment;

        // Obtener la distancia medida por el láser en la muestra actual
        float range = laser_scan->ranges[i];

        // Verificar si la distancia medida está dentro del rango válido
        if (range >= range_min && range <= range_max){
            // Calcular la posición en el grid
            float x = range * cos(angle);
            float y = range * sin(angle);

            // Crear un punto de geometría con la posición en el grid
            geometry_msgs::Point position;
            position.x = x;
            position.y = y;
            position.z = 0.0;  // Supongamos que el grid se encuentra en el plano z=0

            // Agregar la posición al vector de posiciones en el grid
            laserPos.push_back(position);
        }
    }
    //cout<<laserPos.size()<<" posiciones"<<endl;
}

void mada_util::laserPosCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Point>& laserPos){
    //ROS_INFO("=====================> LaserScan (val,angle)=(%f,%f", msg->range_min,msg->angle_min);
    laserPos.clear();

    float angle;

    // Iterar sobre las muestras del láser scan
    for (int i = 0; i < laser_scan->ranges.size(); ++i){
        // Calcular el ángulo correspondiente a la muestra actual
        angle = laser_scan->angle_min + i * laser_scan->angle_increment;

        // Verificar si la distancia medida está dentro del rango válido
        if (laser_scan->ranges[i] >= laser_scan->range_min && laser_scan->ranges[i] <= laser_scan->range_max){
            // Calcular la posición en el grid
            float x = laser_scan->ranges[i] * cos(angle);
            float y = laser_scan->ranges[i] * sin(angle);

            // Crear un punto de geometría con la posición en el grid
            geometry_msgs::Point position;
            position.x = x;
            position.y = y;
            position.z = 0.0;  // Supongamos que el grid se encuentra en el plano z=0

            // Agregar la posición al vector de posiciones en el grid
            laserPos.push_back(position);
        }
    }
    //cout<<laserPos.size()<<" posiciones"<<endl;
}

void mada_util::laserPosesCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, vector<geometry_msgs::Pose>& laserPoses)
{
    laserPoses.clear();

    float angle;
    geometry_msgs::Pose pose;
    // Iterar sobre las muestras del láser scan
    for (int i = 0; i < laser_scan->ranges.size(); ++i){
        // Calcular el ángulo correspondiente a la muestra actual
        angle = laser_scan->angle_min + i * laser_scan->angle_increment;

        // Verificar si la distancia medida está dentro del rango válido
        if (laser_scan->ranges[i] >= laser_scan->range_min && laser_scan->ranges[i] <= laser_scan->range_max){
            // Crear un punto de geometría con la posición en el grid
            pose.position.x = laser_scan->ranges[i] * cos(angle);
            pose.position.y = laser_scan->ranges[i] * sin(angle);
            pose.position.z = 0.0;  // Supongamos que el grid se encuentra en el plano z=0

            // Creo el quaternion
            pose.orientation = tf::createQuaternionMsgFromYaw(angle);

            // Agregar la posición al vector de posiciones en el grid
            laserPoses.push_back(pose);
        }
    }
}

vector<geometry_msgs::Pose> mada_util::laserPosesTf(vector<geometry_msgs::Pose> laserPoses)
{
    tf::TransformListener tfListener;

    geometry_msgs::PoseStamped transformedPose, laserPose;
    vector<geometry_msgs::Pose> transformedPoses(laserPoses.size());
    for(int i=0; i<laserPoses.size(); i++){
        try {
            laserPose.header.frame_id = "/scan";
            laserPose.pose = laserPoses[i];
            // Realiza la transformación a través del marco de referencia global (map)
            tfListener.transformPose("/map", laserPose, transformedPose);

            // La pose transformada está ahora en transformedPose.pose
            // Puedes utilizar transformedPose.pose para representarla en tu grid de ocupación
            transformedPoses[i] = transformedPose.pose;
        } catch (tf::TransformException& ex) {
            ROS_WARN("Error en la transformacion del laser: %s", ex.what());
        }
    }
    return transformedPoses;
}

vector<geometry_msgs::Pose> mada_util::laserPosesTf(geometry_msgs::Pose robotPose, vector<geometry_msgs::Pose> laserPoses){

    // Realiza la transformación para cada pose del láser
    std::vector<geometry_msgs::Pose> transformedPoses;
    for (const auto& laserPose : laserPoses){
        // Aplica la transformación de la pose del láser al marco de referencia del robot
        geometry_msgs::Pose transformedPose;
        transformedPose.position.x = laserPose.position.x + robotPose.position.x;
        transformedPose.position.y = laserPose.position.y + robotPose.position.y;
        transformedPose.position.z = laserPose.position.z + robotPose.position.z;

        tf2::Quaternion q_robot(robotPose.orientation.x, robotPose.orientation.y, robotPose.orientation.z, robotPose.orientation.w);
        tf2::Quaternion q_laser(laserPose.orientation.x, laserPose.orientation.y, laserPose.orientation.z, laserPose.orientation.w);

        tf2::Quaternion q_transformed = q_robot * q_laser;
        q_transformed.normalize();

        transformedPose.orientation.x = q_transformed.x();
        transformedPose.orientation.y = q_transformed.y();
        transformedPose.orientation.z = q_transformed.z();
        transformedPose.orientation.w = q_transformed.w();

        transformedPoses.push_back(transformedPose);
    }
    return transformedPoses;
}


// ******************************************************************************************************************************
//                                                         Mapa
// ******************************************************************************************************************************

void mada_util::_occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, vector<vector<int>>& occGrid)
{
    // Convertir el mensaje OccupancyGrid a un objeto vector<vector<int>>
    occGrid = occupancyGridToVector(*occupancyGridMsg);
}

void mada_util::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, nav_msgs::OccupancyGrid& map, vector<vector<int>>& occGrid)
{
    // Almacenar el mapa
    map = *occupancyGridMsg;
    // Y el grid
    occGrid = occupancyGridToVector(*occupancyGridMsg);
}

vector<vector<int>> mada_util::occupancyGridToVector(const nav_msgs::OccupancyGrid& occupancyGridMsg)
{
    vector<vector<int>> occGrid(occupancyGridMsg.info.height, vector<int>(occupancyGridMsg.info.width, 0));
    // Copiar los datos de ocupación del mapa al vector 2D
    for (size_t y = 0; y < occupancyGridMsg.info.height; ++y){
        for (size_t x = 0; x < occupancyGridMsg.info.width; ++x){
            // Obtener el valor de ocupación de la celda
            occGrid[y][x] = occupancyGridMsg.data[y * occupancyGridMsg.info.width + x];
        }
    }

    return occGrid;
}

vector<vector<int>> mada_util::occupancyGrid2Grid(vector<vector<int>> occupancyGrid)
{
    // Copiar los datos de ocupación del mapa al vector 2D
    for (size_t y = 0; y < occupancyGrid.size(); ++y){
        for (size_t x = 0; x < occupancyGrid[x].size(); ++x){
            if(occupancyGrid[y][x]>0) occupancyGrid[y][x] = 1;
            else occupancyGrid[y][x] = 0;
        }
    }

    return occupancyGrid;
}

void mada_util::gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, nav_msgs::OccupancyGrid& map, vector<vector<int>>& occGrid, vector<vector<float>>& grid)
{
    // Almacenar el mapa
    map = *occupancyGridMsg;
    // Y los grids
    //occGrid = occupancyGridToVector(*occupancyGridMsg);
    //grid = occupancyGrid2Grid(occGrid);

    occupancyGrid2Grid(occupancyGridMsg, occGrid, grid);
}

void mada_util::occupancyGrid2Grid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGridMsg, vector<vector<int>>& occGrid, vector<vector<float>>& grid)
{
    occGrid.resize(occupancyGridMsg->info.height, vector<int>(occupancyGridMsg->info.width, 0));
    grid.resize(occupancyGridMsg->info.height, vector<float>(occupancyGridMsg->info.width, 0));
    // Copiar los datos de ocupación del mapa al vector 2D
    for (size_t y = 0; y < occupancyGridMsg->info.height; ++y){
        for (size_t x = 0; x < occupancyGridMsg->info.width; ++x){
            // Obtener el valor de ocupación de la celda
            occGrid[y][x] = occupancyGridMsg->data[y * occupancyGridMsg->info.width + x];
            if(occGrid[y][x]==0) grid[y][x] = 1;
            else grid[y][x] = 0;
        }
    }
}

vector<signed char> mada_util::grid2occupancyGridData(vector<vector<float>> grid)
{
    vector<signed char> res;
    for(int i=0; i<grid.size(); i++)
        for(int j=0; j<grid[i].size(); j++)
            if(grid[i][j])
                res.push_back(0);
            else if(grid[i][j] == 0)
                res.push_back(100);
    return res;
}

vector<signed char> mada_util::grid2occupancyGridData(vector<vector<int>> grid)
{
    vector<signed char> res;
    for(int i=0; i<grid.size(); i++)
        for(int j=0; j<grid[i].size(); j++)
            if(grid[i][j])
                res.push_back(0);
            else if(grid[i][j] == 0)
                res.push_back(100);
    return res;
}

vector<signed char> mada_util::map2occupancyGridData(vector<vector<bool>> map)
{
    vector<signed char> res;
    for(int i=0; i<map.size(); i++)
        for(int j=0; j<map[i].size(); j++)
            res.push_back(map[i][j]);
    return res;
}

vector<signed char> mada_util::map2occupancyGridData(vector<vector<int>> map)
{
    vector<signed char> res;
    for(int i=0; i<map.size(); i++)
        for(int j=0; j<map[i].size(); j++)
            res.push_back(map[i][j]);
    return res;
}

vector<signed char> mada_util::map2occupancyGridData(vector<vector<float>> map)
{
    vector<signed char> res;
    for(int i=0; i<map.size(); i++)
        for(int j=0; j<map[i].size(); j++)
            res.push_back(map[i][j]);
    return res;
}

vector<signed char> mada_util::grad2occupancyGridData(int x, int y, vector<vector<float>> grad)
{
    vector<signed char> res;
    if(grad.size() == 0) return res;

    // Propagar un frente de onda desde (x, y) por todo el gradiente
    vector<int> qx, qy, qv;
    vector<int> nx, ny;
    vector<float> nv;
    int _x, _y;
    int xm, ym; float vm;
    vector<pair<int, int>> m = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    //vector<pair<int, int>> m = {{0, 1}, {1, 1}, {1, 0}, {-1, 0}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
    qx.push_back(x); qy.push_back(y); qv.push_back(0);
    vector<vector<int>> _grad(grad.size(), vector<int>(grad[0].size(), 0));
    bool f;
    while(qx.size()){
        vm = 10000000;
        for(int i=0; i<m.size(); i++){
            _x = qx[0] + m[i].first;
            _y = qy[0] + m[i].second;
            if(grad[_x][_y] && grad[_x][_y]<10000000){ // Solo donde se ha calculado el gradiente, no es un obstáculo y el frente no ha pasado por aqui
                nx.push_back(_x); ny.push_back(_y); nv.push_back(grad[_x][_y]);
                //qx.push_back(_x); qy.push_back(_y); qv.push_back(qv[0]+1);
                //_grad[_x][_y] = qv[0]+1;
                //grad[_x][_y] = 0;
            }
        }

        while(nx.size()){
            for(int i=0; i<qx.size(); i++){
                if(qv[i] > nv[0]){
                    qx.insert(qx.begin()+i, nx[0]); qy.insert(qy.begin()+i, ny[0]);
                    qv.insert(qv.begin()+i, nv[0]);
                    grad[nx[0]][ny[0]] = 0;
                    _grad[nx[0]][ny[0]] = qv[0]+1;
                    f = true;
                    break;
                }
            }
            nx.erase(nx.begin()); ny.erase(ny.begin()); nv.erase(nv.begin());
        }
        qx.erase(qx.begin()); qy.erase(qy.begin()); qv.erase(qv.begin());
        cout<<"------------------------>"<<qx.size()<<endl;
    }
    
    for(int i=0; i<_grad.size(); i++)
        for(int j=0; j<_grad[i].size(); j++)
            res.push_back(_grad[i][j]);

    return res;
}

vector<signed char> mada_util::grad2occupancyGridData(vector<vector<float>> grad)
{
    vector<signed char> res;
    float max = 0;
    for(int i=0; i<grad.size(); i++)
        for (int j=0; j<grad[i].size(); j++)
            if(grad[i][j]<1000000 && grad[i][j]>max) max=grad[i][j];

    //cout<<"---------------------------->"<<max<<endl;
    float v;
    if(max<10) v = 10;
    else v = 5;
    for(int i=0; i<grad.size(); i++)
        for(int j=0; j<grad[i].size(); j++)
            res.push_back(grad[i][j]*v);

    return res;
}

vector<vector<float>> mada_util::occupancyGrid2FMatrix(nav_msgs::OccupancyGrid occupancyGridMsg)
{
    vector<vector<float>> res(occupancyGridMsg.info.height, vector<float>(occupancyGridMsg.info.width, 0));
    // Copiar los datos de ocupación del mapa al vector 2D
    for (size_t y = 0; y < occupancyGridMsg.info.height; ++y){
        for (size_t x = 0; x < occupancyGridMsg.info.width; ++x){
            res[y][x] = occupancyGridMsg.data[y * occupancyGridMsg.info.width + x];
        }
    }

    return res;
}


// ******************************************************************************************************************************
//                                               Planificación de caminos
// ******************************************************************************************************************************

vector<vector<float>> mada_util::navigableGradient(vector<vector<float>> obst_grad, float size)
{
    // Esto equivaldría a una especie de inflado de los obstáculos
    for(int i=0; i<obst_grad.size(); i++)
        for(int j=0; j<obst_grad[i].size(); j++)
            if(obst_grad[i][j] <= size) obst_grad[i][j] = 0;
    return obst_grad;
} // Dejo los valores del gradiente de los obstáculos → Los caminos se alejarán de los obstáculos

vector<vector<float>> mada_util::navigableGrid(vector<vector<float>> obst_grad, float size)
{
    for(int i=0; i<obst_grad.size(); i++)
        for(int j=0; j<obst_grad[i].size(); j++)
            if(obst_grad[i][j] <= size) obst_grad[i][j] = 0;
            else obst_grad[i][j] = 1;
    return obst_grad;
} // Los valores son [0,1] → Se obtienen los caminos más cortos evitando los obstáculos

nav_msgs::Path mada_util::transformToMapPath(vector<int> x, vector<int> y, const nav_msgs::OccupancyGrid& mapMsg)
{
    // Por si al final uso un navegador propio de ROS
    nav_msgs::Path res;
    geometry_msgs::PoseStamped currPose;
    for(int i=0; i<x.size(); i++){
        currPose.pose.position.x = x[i] * mapMsg.info.resolution + mapMsg.info.origin.position.x;
        currPose.pose.position.y = y[i] * mapMsg.info.resolution + mapMsg.info.origin.position.y;
        res.poses.push_back(currPose);
    }
    return res;
}

vector<geometry_msgs::Pose> mada_util::convertPointsToPoses(vector<geometry_msgs::Point> points)
{
    vector<geometry_msgs::Pose> poses;

    if (points.size() < 2){
        ROS_WARN("Not enough points in the path");
        return poses;
    }
    geometry_msgs::Pose pose;
    for (size_t i = 0; i < points.size() - 1; ++i){

        pose.position = points[i];

        // Calculate orientation (yaw) between current point and next point
        double dx = points[i + 1].x - points[i].x;
        double dy = points[i + 1].y - points[i].y;
        double orientation = atan2(dy, dx);

        // Convert orientation to quaternion representation
        pose.orientation = tf::createQuaternionMsgFromYaw(orientation);

        poses.push_back(pose);
    }

    // El último punto conserva la orientación del penúltimo para no girarte
    geometry_msgs::Pose lastPose;
    lastPose.position = points.back();
    lastPose.orientation  = pose.orientation;
    poses.push_back(lastPose);

    return poses;
}

geometry_msgs::Pose mada_util::computePose(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    geometry_msgs::Pose pose;
    pose.position = p1;
    pose.orientation = tf::createQuaternionMsgFromYaw(atan2(p2.y - p1.y, p2.x - p1.x));
    return pose;
}

float mada_util::computePathDistance(const nav_msgs::Path& path)
{
    float res = 0;

    if(path.poses.size() > 1)
    for(int i=1; i<path.poses.size(); i++)
        res += hypot(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x, path.poses[i].pose.position.y - path.poses[i-1].pose.position.y);

    return res;
}

float mada_util::computePathDistance(const nav_msgs::Path& path, int evp)
{
    float res = 0;

    if(path.poses.size() > 1)
    for(int i=path.poses.size()-1; i>=0 && i>evp; i--)
        res += hypot(path.poses[i+1].pose.position.x - path.poses[i].pose.position.x, path.poses[i+1].pose.position.y - path.poses[i].pose.position.y);

    return res;
}

float mada_util::computePathDistance(const vector<geometry_msgs::Pose>& pathPoses)
{
    float res = 0;

    if(pathPoses.size() > 1)
    for(int i=1; i<pathPoses.size(); i++)
        res += hypot(pathPoses[i].position.x - pathPoses[i-1].position.x, pathPoses[i].position.y - pathPoses[i-1].position.y);

    return res;
}

vector<geometry_msgs::Pose> mada_util::path2Poses(nav_msgs::Path path)
{
    vector<geometry_msgs::Pose> res;
    for(int i=0; i<path.poses.size(); i++)
        res.push_back(path.poses[i].pose);
    return res;   
}

int mada_util::noMovementPath(nav_msgs::Path path)
{
    int res = 0;
    if(path.poses.size() < 2) return res;
    float dist;
    for(int i=path.poses.size()-2; i>=0; i--){
        dist = sqrt(pow(path.poses[i].pose.position.x - path.poses[i+1].pose.position.x, 2) + pow(path.poses[i].pose.position.y - path.poses[i+1].pose.position.y, 2));
        if(dist == 0){
            res++;
        }else{
            break;
        }
    }
    return res;
}

int mada_util::noMovePath(nav_msgs::Path path)
{
    int res = 0;
    if(path.poses.size() < 2) return res;
    pair<float, float> dist;
    for(int i=path.poses.size()-2; i>=0; i--){
        dist = poseDistance(path.poses[i].pose, path.poses[i+1].pose);
        if(dist.first + dist.second == 0){
            res++;
        }else{
            break;
        }
    }
    return res;
}

pair<float, float> mada_util::distPose2Path(geometry_msgs::Pose p, nav_msgs::Path path, int evp)
{
    pair<float, float> res, d;
    res.first = res.second = 0;
    d.first = d.second = 0;
    int n = path.poses.size();
    if(evp >= n) evp = (n-1);
    for(int i = n-1; i>=n-evp; i--){
        d = poseDistance(p, path.poses[i].pose);
        res.first += d.first;
        res.second += d.second;
    }
    return res;
} // Distancia de una pose a las últimas "evp" poses del camino "path"

vector<geometry_msgs::Pose> mada_util::setFollowerPathPoses(vector<int> x, vector<int> y, vector<vector<float>> navArea, nav_msgs::OccupancyGrid mapMsg, vector<int> vox, vector<int> voy, vector<geometry_msgs::Pose> &path2GoalPoses)
{
    vector<geometry_msgs::Pose> path2Follow;
    // Obtener el camino que se le va a enviar al path follower
    // Pasar camino en el grid a puntos del mapa
    path2GoalPoses = mada_util::convertPointsToPoses(mada_util::tfGridPos2MapPos(mapMsg, y, x));

    // Obtener la distancia del camino a los obstáculos más cercanos
    vector<float> pathGradValues(x.size(), 0);
    path2Follow = path2GoalPoses;
    float maxv = -1;
    for(int i=0; i<x.size(); i++){
        pathGradValues[i] = navArea[x[i]][y[i]];
        if(maxv < pathGradValues[i]) maxv = pathGradValues[i];
    }

    // Incluir las distancias de los obstáculos dinámicos en el camino
    float _d;
    for(int i=0; i<x.size(); i++){
        for(int j=0; j<vox.size(); j++){
            _d = hypot(vox[j] - x[i], voy[j] - y[i]);
            if(pathGradValues[i] < _d){
                pathGradValues[i] = _d;
            }
            if(maxv < pathGradValues[i]) maxv = pathGradValues[i];
        }
    }

    // Normalizar los valores de las distancias del camino
    if(maxv)
        for(int i=0; i<x.size(); i++){
            //pathGradValues[i] /= maxv;
            //path2Follow[i].position.z = pathGradValues[i]; // Distancia sobre el grid
            path2Follow[i].position.z = pathGradValues[i] * mapMsg.info.resolution; // Distancia sobre el mapa
        }

    return path2Follow;
}

vector<geometry_msgs::Pose> mada_util::setFollowerPathPoses_(vector<int> x, vector<int> y, vector<vector<float>> navArea, nav_msgs::OccupancyGrid mapMsg, vector<int> vox, vector<int> voy, vector<geometry_msgs::Pose> &path2GoalPoses)
{
    vector<geometry_msgs::Pose> path2Follow;
    // Obtener el camino que se le va a enviar al path follower
    // - (position.x, position.y): posisión
    // - (orientation.z, orientation.w): orientación
    // - position.z: distancia al obstáculo más cercano (estático o dinámico)
    // - orientation.x: distancia al obstáculo estático más cercano
    // - orientation.y: distancia al obstáculo dinámico más cercano

    // Pasar camino en el grid a puntos del mapa
    path2GoalPoses = mada_util::convertPointsToPoses(mada_util::tfGridPos2MapPos(mapMsg, y, x));

    // Obtener la distancia del camino a los obstáculos más cercanos
    vector<float> pathGradValues(x.size(), 0);
    path2Follow = path2GoalPoses;
    float maxv = -1;
    for(int i=0; i<x.size(); i++){
        pathGradValues[i] = navArea[x[i]][y[i]];
        path2Follow[i].orientation.x = (navArea[x[i]][y[i]] * mapMsg.info.resolution);
        if(maxv < pathGradValues[i]) maxv = pathGradValues[i];
    }

    // Incluir las distancias de los obstáculos dinámicos en el camino
    float _d, __d;
    for(int i=0; i<x.size(); i++){
        path2Follow[i].orientation.y = INFINITY;
        for(int j=0; j<vox.size(); j++){
            _d = hypot(vox[j] - x[i], voy[j] - y[i]);
            if(pathGradValues[i] > _d){
                pathGradValues[i] = _d;
            }
            if(maxv < pathGradValues[i]) maxv = pathGradValues[i];

            __d = (_d * mapMsg.info.resolution);
            if(path2Follow[i].orientation.y > __d){
                path2Follow[i].orientation.y = __d;
            }
        }
    }

    // Normalizar los valores de las distancias del camino
    if(maxv)
        for(int i=0; i<x.size(); i++){
            //pathGradValues[i] /= maxv;
            //path2Follow[i].position.z = pathGradValues[i]; // Distancia sobre el grid
            path2Follow[i].position.z = pathGradValues[i] * mapMsg.info.resolution; // Distancia sobre el mapa
        }

    return path2Follow;
}

void mada_util::dynObstPos(vector<int> laser_poss_x, vector<int> laser_poss_y, vector<geometry_msgs::Pose> laser_poses, vector<vector<float>> grid, int x, int y, float vrange, vector<int> &obst_poss_x, vector<int> &obst_poss_y, vector<geometry_msgs::Point> &obst_pos)
{
    obst_poss_x.clear();
    obst_poss_y.clear();
    obst_pos.clear();
    
    float d;
    for(int i=0; i<laser_poss_x.size(); i++){
        if(grid[laser_poss_x[i]][laser_poss_y[i]]){
            d = hypot(x - laser_poss_x[i], y - laser_poss_y[i]);
            if(abs(d-vrange)>3){
                obst_poss_x.push_back(laser_poss_x[i]);
                obst_poss_y.push_back(laser_poss_y[i]);
                obst_pos.push_back(laser_poses[i].position);
            }
        }
    }
}

void mada_util::dynObstPos(vector<int> laser_poss_x, vector<int> laser_poss_y, vector<geometry_msgs::Pose> laser_poses, vector<vector<float>> original_grid, vector<int> all_laser_poss_x, vector<int> all_laser_poss_y, vector<vector<float>> &curr_grid, vector<int> &obst_poss_x, vector<int> &obst_poss_y, vector<geometry_msgs::Point> &obst_pos)
{

    obst_poss_x.clear();
    obst_poss_y.clear();
    obst_pos.clear();

    for(int i=0; i<laser_poss_x.size(); i++){
        if(original_grid[laser_poss_x[i]][laser_poss_y[i]]){
            obst_poss_x.push_back(laser_poss_x[i]);
            obst_poss_y.push_back(laser_poss_y[i]);
            obst_pos.push_back(laser_poses[i].position);
        }
    }

    for(int i=0; i<all_laser_poss_x.size(); i++){
        if(!curr_grid[all_laser_poss_x[i]][all_laser_poss_y[i]]){
            curr_grid[all_laser_poss_x[i]][all_laser_poss_y[i]] = original_grid[all_laser_poss_x[i]][all_laser_poss_y[i]];
        }
    }
}


// ******************************************************************************************************************************
//                                                         Navegación
// ******************************************************************************************************************************

geometry_msgs::Twist mada_util::computeVelocities(pair<double, double> start, pair<double, double> goal)
{
    geometry_msgs::Twist res;

    double dist = sqrt(pow(start.first - goal.first, 2) + pow(start.second - goal.second, 2));

    //if(dist > MIN_DIST){
        res.linear.x = min(0.5 * dist, MAX_LINEAR_VEL);
        res.angular.z = min(4 * atan2(pow(start.second - goal.second, 2), pow(start.first - goal.first, 2)), MAX_ANGULAR_VEL);
    //}

    return res;
}

geometry_msgs::Twist mada_util::computeVelocities(pair<double, double> start, geometry_msgs::Point goal)
{
    geometry_msgs::Twist res;

    double dist = sqrt(pow(start.first - goal.x, 2) + pow(start.second - goal.y, 2));

    //if(dist > MIN_DIST){
        res.linear.x = min(0.5 * dist, MAX_LINEAR_VEL);
        res.angular.z = min(4 * atan2(pow(start.second - goal.y, 2), pow(start.first - goal.x, 2)), MAX_ANGULAR_VEL);
    //}

    return res;
}

void mada_util::showCmdVel(geometry_msgs::Twist cmd_vel)
{
    cout<<"("<<cmd_vel.linear.x<<", "<< cmd_vel.angular.z<<")"<<endl;
}

// Las funciones para el cálculo de las velocidades del path follower

geometry_msgs::Twist mada_util::computeVelocities(const geometry_msgs::Pose robotPose, const geometry_msgs::Pose goalPose, double dObstMaxNorm, double maxLinearVel, double maxAngularVel)
{   
    geometry_msgs::Twist res;

    // Cálculo del ángulo de desviación
    double angleDiff = atan2(goalPose.position.y - robotPose.position.y, goalPose.position.x - robotPose.position.x) - tf::getYaw(robotPose.orientation);

    // Ajuste del ángulo de desviación en el rango [-pi, pi]
    if (angleDiff > M_PI)
        angleDiff -= 2.0 * M_PI;
    else if (angleDiff < -M_PI)
        angleDiff += 2.0 * M_PI;

    // Cálculo de las velocidades lineal y angular (ajustado con las aceleraciones lineal y angular máximas)
    res.angular.z = maxAngularVel * angleDiff;

    if(angleDiff){
        //linearVel = maxLinearVel * goalPose.position.z * abs(maxAngularVel - abs(angularVel));
        if(abs(angleDiff) < M_PI/2)
            res.linear.x = maxLinearVel * dObstMaxNorm * (1-(abs(res.angular.z)/maxAngularVel));
        else
            res.linear.x = 0;
    }else{
        res.linear.x = maxLinearVel; // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }

    return res;
} // no se tiene en cuenta nada

geometry_msgs::Twist mada_util::computeVelocitiesAcc(const geometry_msgs::Pose robotPose, const geometry_msgs::Pose goalPose, double dObstMaxNorm, double maxLinearVel, double maxAngularVel, double prevLinearVel, double prevAngularVel, double linearAcc, double angularAcc)
{

    geometry_msgs::Twist res;

    // Cálculo del ángulo de desviación
    double angleDiff = atan2(goalPose.position.y - robotPose.position.y, goalPose.position.x - robotPose.position.x) - tf::getYaw(robotPose.orientation);

    // Ajuste del ángulo de desviación en el rango [-pi, pi]
    if (angleDiff > M_PI)
        angleDiff -= 2.0 * M_PI;
    else if (angleDiff < -M_PI)
        angleDiff += 2.0 * M_PI;

    // Cálculo de las velocidades lineal y angular (ajustado con las aceleraciones lineal y angular máximas)
    res.angular.z = maxAngularVel * angleDiff;
    if(res.angular.z < 0)  // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        res.angular.z = max(res.angular.z, prevAngularVel - abs(angularAcc));
    else
        res.angular.z = min(res.angular.z, prevAngularVel + abs(angularAcc));

    if(angleDiff){
        //res.linear.x = maxLinearVel * goalPose.position.z * abs(maxAngularVel - abs(res.angular.z));
        if(abs(angleDiff) < M_PI/2)
            res.linear.x = maxLinearVel * dObstMaxNorm * (1-(abs(res.angular.z)/maxAngularVel));
            //res.linear.x = maxLinearVel * dObstMaxNorm * abs(maxAngularVel - abs(res.angular.z));
        else
            res.linear.x = 0;
        //res.linear.x = maxLinearVel * abs(maxAngularVel - abs(angularVel));
        res.linear.x = min(res.linear.x, prevLinearVel + linearAcc); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }else{
        //linearVel = maxLinearVel;
        res.linear.x = min(res.linear.x, prevLinearVel + linearAcc); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }

    return res;
}

nav_msgs::Path mada_util::setPlan(geometry_msgs::Pose s, geometry_msgs::Pose g, string topic)
{
    nav_msgs::Path res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = topic;

    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now(); p.header.frame_id = topic;
    p.pose = s; res.poses.push_back(p);
    g.position.z = 0.0; // ESTO ES PARA CUANDO EL CAMINO NO ES 3D
    p.header.stamp = ros::Time::now(); p.header.frame_id = topic;
    p.pose = g; res.poses.push_back(p);

    return res;
}

geometry_msgs::Pose mada_util::selectDestination(geometry_msgs::Pose pose, vector<geometry_msgs::Pose>& poses, double robotRadius, double &dist, double &dist2goal)
{
    // - pose: posición del robot
    // - poses: poses del camino
    // - maxLenght: distancia que se considera para alcanzar el goal
    // - dist: distancia mínima normalizada del camino al obstáculo más cercano
    // - dist2goal: distancia real desde punto del camino seleccionado al obstáculo

    geometry_msgs::Pose res;

    double d, _d;

    double dmin = INFINITY;

    // Disáncia mínima del camino a los obstáculos (estáticos o dinámicos)
    double dom = INFINITY; 

    dist = 0;

    // Encuentro el punto más cercano del camino al robot
    int ind = -1;
    int ind_min_dist = -1;
    d = INFINITY;
    for(int i=0; i<poses.size(); i++){
        // Distancia del robot al punto del camino
        _d = hypot(pose.position.x - poses[i].position.x, pose.position.y - poses[i].position.y);
        if(_d < d){
            d = _d;
            ind = i;
        }
        //if(dist < poses[i].position.z) dist = poses[i].position.z;
        if(dist < poses[i].orientation.x) dist = poses[i].orientation.x;
        if(poses[i].orientation.y != INFINITY && dist < poses[i].orientation.y) dist = poses[i].orientation.y;

        // Almacenar la distancia mínima del camino al obstáculo dinámico más cercano
        if(dmin > poses[i].orientation.y){
            ind_min_dist = i;
            dmin = poses[i].orientation.y;
        }

    }

    // Aquí tengo las distancias del camino al obstáculo:
    // - estático más cercano
    // - dinámico más cercano

    double dmax = dist;
    if(ind > -1){
        // Elimino todos los puntos hasta el punto más cercano
        //poses.erase(poses.begin() + ind);
        poses.erase(poses.begin(), poses.begin() + ind);
        // en el caso de estar dentro del camino, elimino todos los puntos que están "dentro del robot"
        while(poses.size())
            if(hypot(pose.position.x - poses[0].position.x, pose.position.y - poses[0].position.y) <= robotRadius){
                //if(dist < poses[0].position.z) dist = poses[0].position.z;
                poses.erase(poses.begin());
            }else break;
    }

    // Hasta aquí lo que he hecho es:
    // - eliminar todos los puntos hasta el más cercano al robot
    // - eliminar los puntos que están "dentro del radio" del robot
    // - encontrar la distancia del punto del camino que más lejos se encuentra del obstáculo más cercano

    dist2goal = INFINITY;
    while(poses.size() > 1){
        _d = hypot(pose.position.x - poses[0].position.x, pose.position.y - poses[0].position.y);
        if(dist2goal > poses[0].orientation.x) dist2goal = poses[0].orientation.x; // Almacenar la distancia al obstáculo
        if(dist2goal > poses[0].orientation.y) dist2goal = poses[0].orientation.y; // Almacenar la distancia al obstáculo
        if(_d <= poses[0].orientation.x && (_d <= poses[0].orientation.y - 2*robotRadius)){ // La distancia 
            //if(dist < poses[0].position.z) dist = poses[0].position.z;
            if(dist < poses[0].orientation.x) dist = poses[0].orientation.x;
            if(poses[0].orientation.y != INFINITY && dist < poses[0].orientation.y) dist = poses[0].orientation.y;
            poses.erase(poses.begin());
        }else{
            dist /= dmax;
            return poses[0];
        }
    }

    dist /= dmax;
    return poses[0];
}

// ******************************************************************************************************************************
//                                                         Representación
// ******************************************************************************************************************************

visualization_msgs::MarkerArray mada_util::setTreeLines(vector<geometry_msgs::Point> points, vector<vector<int>> tree, vector<float> colors, string map_topic, string tree_frame_id, float resolution)
{
    visualization_msgs::MarkerArray res;
    res.markers.resize(tree.size());
    for(int i=0; i<tree.size(); i++){
        res.markers[i].id = i;
        res.markers[i].header.seq = i;
        res.markers[i].header.stamp = ros::Time::now();
        res.markers[i].header.frame_id = map_topic;
        res.markers[i].ns = tree_frame_id;
        res.markers[i].action = visualization_msgs::Marker::ADD;
        res.markers[i].pose.orientation.w = 1;
        res.markers[i].type = visualization_msgs::Marker::LINE_LIST;
        res.markers[i].scale.x = res.markers[i].scale.y = resolution;
        res.markers[i].color.b = colors[0];
        res.markers[i].color.r = colors[1];
        res.markers[i].color.g = colors[2];
        res.markers[i].color.a = 1.0;

        res.markers[i].points.push_back(points[tree[i][0]]);
        res.markers[i].points.push_back(points[tree[i][1]]);
    }
    return res;
}

void mada_util::clearTreeLines(visualization_msgs::MarkerArray &lines)
{
    for(int i=0; i<lines.markers.size(); i++){
        //lines.markers[i].points.clear();
        lines.markers[i].action = visualization_msgs::Marker::DELETEALL; 
    }
    lines.markers.clear();
}

// ******************************************************************************************************************************
//                                                         Basura
// ******************************************************************************************************************************

void mada_util::count(vector<vector<int>> grid)
{
    int obst, fs, us;
    obst = fs = us = 0;
    for(int i=0; i<grid.size(); i++)
        for(int j=0; j<grid[i].size(); j++)
            if(grid[i][j]==0){
                obst++;
            }else if(grid[i][j]>0){
                fs++;
            }else if(grid[i][j]<0){
                us++;
            }
    cout<<"Libre: "<<fs<<endl;
    cout<<"Obst : "<<obst<<endl;
    cout<<"Desc : "<<us<<endl;
}

double mada_util::distanceToObstacles(double x, double y, double xl, double yl, sensor_msgs::LaserScan scan, double laser_range_threshold)
{
    double min_dist = 0;
    
    for (int i = 0; i < scan.ranges.size(); i++) {
        if(scan.ranges[i] < laser_range_threshold && scan.ranges[i] > scan.range_min){
            // Calcular el ángulo del rayo láser actual
            double angle = scan.angle_min + i * scan.angle_increment;

            // Convertir la lectura de distancia polar a coordenadas cartesianas
            double obs_x = xl + scan.ranges[i] * cos(angle);
            double obs_y = yl + scan.ranges[i] * sin(angle);

            double dist = hypot(x - obs_x, y - obs_y);
            min_dist += dist;

        }
    }

    return min_dist;
}

geometry_msgs::Twist mada_util::dynamicWindowApproach(const geometry_msgs::Pose startPose, const geometry_msgs::Pose goalPose, sensor_msgs::LaserScan scan, double laser_range_threshold, double current_v, double current_w, double min_v, double max_v, double max_w, double max_linear_acc, double max_angular_acc, double dt, double obstacleRadius)
{
    // Calcular el ángulo theta del robot a partir de la orientación
    double theta = 2.0 * atan2(startPose.orientation.z, startPose.orientation.w); // Simplificación para obtener el ángulo de rotación en 2D

    double goal_theta = 2.0 * atan2(goalPose.orientation.z, goalPose.orientation.w);

    cout<<" limites : ("<<min_v<<" -> "<<max_v<<" ) - ("<<-max_w<<" -> "<<max_w<<")"<<endl;

    // Generar ventana dinámica basada en las restricciones actuales del robot
    double min_w;
    double minv = std::max( min_v, current_v - max_linear_acc * dt);
    double maxv = std::min( max_v, current_v + max_linear_acc * dt);
    double minw = std::max(-max_w, current_w - max_angular_acc * dt);
    double maxw = std::min( max_w, current_w + max_angular_acc * dt);

    //minw = -max_w;
    //maxw = max_w;

    minv = min_v; maxv = max_v;
    minw = -max_w; maxw = max_w;

    geometry_msgs::Twist best_command;
    double best_score = 1e6;

    int n_velocities = 20; // número de muestras de velocidades evaluadas

    float dv = (maxv - minv) / (float)n_velocities;
    float dw = (maxw - minw) / (float)n_velocities;

    double score;

    cout<<" ventana : ("<<minv<<" -> "<<maxv<<" ) - ("<<minw<<" -> "<<maxw<<") con deltas ("<<dv<<", "<<dw<<")"<<endl;
    cout<<endl;

    // Bucle para evaluar diferentes combinaciones de velocidad lineal y angular
    for(double v = minv; v <= maxv; v += dv){
        for(double w = minw; w <= maxw; w += dw){
            // Simular la nueva posición del robot
            double sim_x, sim_y, sim_theta;

            /*
            if (fabs(w) > 1e-3) {  // Si hay velocidad angular significativa
                sim_x = startPose.position.x + (v / w) * (sin(theta + w * dt) - sin(theta));
                sim_y = startPose.position.y - (v / w) * (cos(theta + w * dt) - cos(theta));
            } else {  // Movimiento rectilíneo
                sim_x = startPose.position.x + v * cos(theta) * dt;
                sim_y = startPose.position.y + v * sin(theta) * dt;
            }
            sim_theta = theta + w * dt;
            */

            if (fabs(w) > 1e-3) {  // Si hay velocidad angular significativa
                sim_x = startPose.position.x + (v / w) * (sin(theta + w) - sin(theta));
                sim_y = startPose.position.y - (v / w) * (cos(theta + w) - cos(theta));
            } else {  // Movimiento rectilíneo
                sim_x = startPose.position.x + v * cos(theta);
                sim_y = startPose.position.y + v * sin(theta);
            }
            sim_theta = theta + w;

            // Evaluar la trayectoria
            double heading_score = hypot(goalPose.position.x - sim_x, goalPose.position.y - sim_y); // Proximidad al objetivo
            //double heading_angle = abs(atan2(sin(goal_theta-sim_theta), cos(goal_theta-sim_theta)));

            double distance_to_obstacle = distanceToObstacles(sim_x, sim_y, startPose.position.x, startPose.position.y, scan, laser_range_threshold);
            
            /*
            if(distance_to_obstacle < obstacleRadius) {
                continue; // Saltar trayectorias que colisionen
            }
            */
            
            //double score = heading_score + heading_angle + 1.0 / distance_to_obstacle; // Combina distancia al objetivo y a los obstáculos
            //cout<<"("<<v<<", "<<w<<") lleva a ("<<sim_x<<", "<<sim_y<<", "<<sim_theta<<") / ("<<goalPose.position.x<<", "<<goalPose.position.y<<", "<<goal_theta<<")"<<" con ["<<heading_score<<" + "<<heading_angle<<" + "<<1.0/distance_to_obstacle<<"] = "<<score<<endl;
            
            if(distance_to_obstacle)
                score = heading_score + 1.0 / distance_to_obstacle; // Combina distancia al objetivo y a los obstáculos
            else
                score = heading_score;
            cout<<"("<<v<<", "<<w<<") lleva a ("<<sim_x<<", "<<sim_y<<") / ("<<goalPose.position.x<<", "<<goalPose.position.y<<")"<<" con ["<<heading_score<<" + "<<1.0/distance_to_obstacle<<"] = "<<score<<" ~~~ "<<distance_to_obstacle<<endl;

            // Guardar el comando con mejor puntaje
            if (score < best_score) {
                best_score = score;
                best_command.linear.x = v;
                best_command.angular.z = w;
            }

            
        }
    }

    cout<<endl;
    cout<<"best move : ("<<best_command.linear.x<<", "<<best_command.angular.z<<"): "<<best_score<<endl;

    //cin.get();

    return best_command;
}

double mada_util::distanceToObstacle(double x, double y, double xl, double yl, sensor_msgs::LaserScan scan, double laser_range_threshold)
{
    double min_dist = 1e6;
    
    for (int i = 0; i < scan.ranges.size(); i++) {
        if(scan.ranges[i] < laser_range_threshold && scan.ranges[i] > scan.range_min){
            // Calcular el ángulo del rayo láser actual
            double angle = scan.angle_min + i * scan.angle_increment;

            // Convertir la lectura de distancia polar a coordenadas cartesianas
            double obs_x = xl + scan.ranges[i] * cos(angle);
            double obs_y = yl + scan.ranges[i] * sin(angle);

            double dist = hypot(x - obs_x, y - obs_y);
            if(dist < min_dist){
                min_dist = dist;
            }

        }
    }

    return min_dist;
}

geometry_msgs::Twist mada_util::computeVelocitiesDWA(const geometry_msgs::Pose startPose, const geometry_msgs::Pose goalPose, sensor_msgs::LaserScan scan, double laser_range_threshold, double current_v, double current_w, double min_v, double max_v, double max_w, double max_linear_acc, double max_angular_acc, double dt, double obstacleRadius)
{

    // Calcular el ángulo theta del robot a partir de la orientación
    double theta = 2.0 * atan2(startPose.orientation.z, startPose.orientation.w); // Simplificación para obtener el ángulo de rotación en 2D

    double goal_theta = 2.0 * atan2(goalPose.orientation.z, goalPose.orientation.w);

    cout<<" limites : ("<<min_v<<" -> "<<max_v<<" ) - ("<<-max_w<<" -> "<<max_w<<")"<<endl;

    // Generar ventana dinámica basada en las restricciones actuales del robot
    double min_w;
    double minv = std::max( min_v, current_v - max_linear_acc * dt);
    double maxv = std::min( max_v, current_v + max_linear_acc * dt);
    double minw = std::max(-max_w, current_w - max_angular_acc * dt);
    double maxw = std::min( max_w, current_w + max_angular_acc * dt);

    //minw = -max_w;
    //maxw = max_w;

    minv = min_v; maxv = max_v;
    minw = -max_w; maxw = max_w;

    geometry_msgs::Twist best_command;
    double best_x, best_y, best_theta, best_distance;

    double best_score = 1e6;

    int n_velocities = 50; // número de muestras de velocidades evaluadas

    float dv = (maxv - minv) / (float)n_velocities;
    float dw = (maxw - minw) / (float)n_velocities;

    double heading_score;
    double score;
    double distance_to_obstacle;
    double sim_x, sim_y, sim_theta;

    cout<<" ventana : ("<<minv<<" -> "<<maxv<<" ) - ("<<minw<<" -> "<<maxw<<") con deltas ("<<dv<<", "<<dw<<")"<<endl;
    cout<<endl;

    // Sacar la mejor combinación de velocidades lineal y angular
    for(double v = minv; v <= maxv; v += dv){
        for(double w = minw; w <= maxw; w += dw){
            // Simular la nueva posición del robot
            if (fabs(w) > 1e-3) {  // Si hay velocidad angular significativa
                sim_x = startPose.position.x + (v / w) * (sin(theta + w) - sin(theta));
                sim_y = startPose.position.y - (v / w) * (cos(theta + w) - cos(theta));
            } else {  // Movimiento rectilíneo
                sim_x = startPose.position.x + v * cos(theta);
                sim_y = startPose.position.y + v * sin(theta);
            }
            sim_theta = theta + w;

            // Evaluar la trayectoria
            heading_score = hypot(goalPose.position.x - sim_x, goalPose.position.y - sim_y); // Proximidad al objetivo
            //double heading_angle = abs(atan2(sin(goal_theta-sim_theta), cos(goal_theta-sim_theta)));

            distance_to_obstacle = distanceToObstacle(sim_x, sim_y, startPose.position.x, startPose.position.y, scan, laser_range_threshold);

            // Descartar si laa trayectoría lleva a colisión
            if(distance_to_obstacle <= obstacleRadius) continue;
            
            score = 0.5*heading_score + 1.0 / distance_to_obstacle; // Combina distancia al objetivo y a los obstáculos            

            //cout<<"("<<v<<", "<<w<<") lleva a ("<<sim_x<<", "<<sim_y<<") / ("<<goalPose.position.x<<", "<<goalPose.position.y<<")"<<" con ["<<heading_score<<" + "<<1.0/distance_to_obstacle<<"] = "<<score<<" ~~~ "<<distance_to_obstacle<<endl;
            cout<<"("<<v<<", "<<w<<") lleva a ("<<sim_x<<", "<<sim_y<<", "<<distance_to_obstacle<<") - ("<<goalPose.position.x<<", "<<goalPose.position.y<<")"<<" con ["<<heading_score<<" + "<<1.0/distance_to_obstacle<<"] = "<<score<<endl;

            // Guardar el comando con mejor puntaje
            if(best_score > score){
                best_score = score;
                best_command.linear.x = v;
                best_command.angular.z = w;

                best_x = sim_x; best_y = sim_y; best_theta = sim_theta;
                best_distance = distance_to_obstacle;
            }

        }
    }

    cout<<endl;
    cout<<"best move : ("<<best_command.linear.x<<", "<<best_command.angular.z<<"): "<<best_score<<" a ("<<best_x<<", "<<best_y<<", "<<best_theta<<"): "<<best_distance<<" / "<<obstacleRadius<<endl;

    return best_command;

}


