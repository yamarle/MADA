#include <mada/mada_master.hpp>

vector<bool> _regsWithObst;
float xrobot, yrobot;
float _xrobot = 0, _yrobot = 0;
RobotsListener::Positions _obstaclePositions;
int _Nobst = 0;
vector<geometry_msgs::Pose> _centroidPoses;
Poss<int> centroids;
int _xr, _yr, _xg, _yg;

// Variables para los resultados
int reached_goals = 0;
int aborted_goals = 0;
int collisions_robots = 0;
int collisions_obstacles = 0;
float time_not_moving = 0; float tnm_reg_time = 0;
float time_to_reach_goal = 0; float mt2g_reg_time = 0; float time2goal, ttime2goal = 0;
float time_to_abort_goal = 0;
float mean_path_length = 0;
float mean_dist_to_closest_obst = 0;
float min_dist_to_obst = INFINITY;

//int critical_collisions_robots = 0;
//int critical_collisions_obstacles = 0;

float d2o;
float min_d2o = INFINITY;
float reg_delta = 0.5; // Medio segundo para registrar el
float nm_dist = 0;
int nregs = 0;

bool colliding = false, nm_colliding = false, m_colliding = false;
bool not_moving = false;

// Para almacenar el camino completo recorrido por el robot
nav_msgs::Path totalPath; geometry_msgs::PoseStamped totalPathPose;

string result_filename;

void register_results()
{
    min_d2o = INFINITY;
    for(int i=0; i<_Nobst; i++){
        d2o = hypot(xrobot - _obstaclePositions.poses[i].position.x, yrobot - _obstaclePositions.poses[i].position.y);
        if(min_dist_to_obst > d2o) min_dist_to_obst = d2o; // Distancia mínima al obstáculo más cercano
        if(min_d2o > d2o) min_d2o = d2o;
    }
    if(min_d2o < INFINITY)
        mean_dist_to_closest_obst += min_d2o;
    nregs++; // Para despues sacar la media de la distancia a los obstáculos

    if(hypot(xrobot - _xrobot, yrobot - _yrobot) <= nm_dist){ // El robot está parado
        not_moving = true;
        // registro la posición
        if(tnm_reg_time == 0){ // El robot se estaba moviendo
            tnm_reg_time = ros::Time::now().toSec(); // Registro el tiempo cuando el robot se ha quedado parado
        }
    }else{ // El robot se esta moviendo
        not_moving = false;
        if(tnm_reg_time){ // Ha estado parado
            // Acumulo el tiempo que ha estado parado
            time_not_moving += (ros::Time::now().toSec() - tnm_reg_time);
        }
        tnm_reg_time = 0; // Reinicio el tiempo de registro
    }

    if(colliding){ // El robot está colisionando con un obstáculo
        if(not_moving){ // El robot no se estaba moviendo
            if(nm_colliding == false){
                nm_colliding = true;
                collisions_obstacles++;
            }
        }else{
            if(m_colliding == false && nm_colliding==false){
                m_colliding = true;
                collisions_robots++;
            }
        }
    }else{
        nm_colliding = false;
    }

}

void shutdownCallback(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data) {
        ROS_INFO("CLOSING ALL");

        // Finaliza el nodo
        //ros::shutdown();

        // Cierra todo
        std::system("rosnode kill --all");
        std::system("killall -9 rosmaster");
        std::system("killall -9 roscore");

        std::system("rosclean purge -h");
    }
}


int main(int argc, char** argv) {
    
    std::string node_name = "mada_master_node";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Parñametros de entrada
    string map_topic;

    int Nr = 0;
    string robots_base_name;
    string robot_pose_name;
    float rx, ry;
    
    int No = 0;
    string obstacles_base_name;
    string obstacle_pose_name;
    float ox, oy;

    float collision_distance = 0;
    float inflFact = 0;

    result_filename = "env_man_res.txt";

    ros::NodeHandle nh_priv("~");
    nh_priv.getParam("map_topic", map_topic);
    
    nh_priv.getParam("Nr", Nr);
    nh_priv.getParam("robots_base_name", robots_base_name);
    nh_priv.getParam("robot_pose_name", robot_pose_name);
    nh_priv.getParam("rx", rx);
    nh_priv.getParam("ry", ry);

    nh_priv.getParam("No", No);
    nh_priv.getParam("obstacles_base_name", obstacles_base_name);
    nh_priv.getParam("obstacle_pose_name", obstacle_pose_name);
    nh_priv.getParam("ox", ox);
    nh_priv.getParam("oy", oy);

    nh_priv.getParam("collision_distance", collision_distance);
    nh_priv.getParam("inflFact", inflFact);
    nh_priv.getParam("result_filename", result_filename);

    cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    cout<<"	   MASTER   "<<endl;
    cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    cout<<"map_topic      : "<<map_topic<<endl;
    cout<<"Nrobots        : "<<Nr<<endl;
    cout<<"robots names   : "<<robots_base_name<<endl;
    cout<<"robots poses   : "<<robot_pose_name<<endl;
    cout<<"robots size    : "<<rx<<" x "<<ry<<endl;
    cout<<"Nobstacles     : "<<No<<endl;
    cout<<"obstacles names: "<<obstacles_base_name<<endl;
    cout<<"obstacles poses: "<<obstacle_pose_name<<endl;
    cout<<"obstacles sizes: "<<ox<<" x "<<oy<<endl;
    cout<<"collision dist : "<<collision_distance<<endl;
    cout<<"inflation fact : "<<inflFact<<endl;
    cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;

    float infl_dist = inflFact * collision_distance;

    vector<string> robot_names;
    vector<string> obstacle_names;

    if(Nr && No){
        for(int i=0; i<Nr; i++)
        	robot_names.push_back(robots_base_name + to_string(i));

        if(robots_base_name.compare(obstacles_base_name) == 0){
        	for(int i=Nr; i<Nr+No; i++)
        		obstacle_names.push_back(obstacles_base_name + to_string(i));
        }else{
        	for(int i=0; i<No; i++)
        		obstacle_names.push_back(obstacles_base_name + to_string(i));
        }
    }else{
        robot_names.resize(1);
    }

    MadaMaster::MadaMaster master(map_topic, robot_names, robot_pose_name, rx, ry, obstacle_names, obstacle_pose_name, ox, oy, collision_distance, infl_dist);

    ros::Publisher rr_coll_pub = nh.advertise<std_msgs::Bool>("mada_master/rr_collision", 10);
    ros::Publisher oo_coll_pub = nh.advertise<std_msgs::Bool>("mada_master/oo_collision", 10);
    ros::Publisher ro_coll_pub = nh.advertise<std_msgs::Bool>("mada_master/ro_collision", 10);
    std_msgs::Bool collMsg;

    ros::Subscriber shutdown_sub = nh.subscribe("/shutdown_signal", 10, shutdownCallback);

    // Calcular las particiones
    int xr, yr;
    vector<vector<float>> grid, obstGrad, navGrad, navArea;
    nav_msgs::OccupancyGrid map = master.get_map();
    float inflDist = (inflFact*hypot(rx, ry))/map.info.resolution;
    fmm_segment::Segments fmm_segments;
    RobotsListener::Positions robotPositions = master.getRobotPositions();
    RobotsListener::Positions obstaclePositions = master.getObstaclePositions();
    grid = master.get_grid();
    xr = robotPositions.x[0]; yr = robotPositions.y[0];
    MadaMaster::computeInitialMaps(xr, yr, grid, inflDist, obstGrad, navArea, navGrad, fmm_segments);
    vector<geometry_msgs::Pose> centroidPoses(fmm_segments.centroids.x.size());
    for(int i=0; i<fmm_segments.centroids.x.size(); i++){
        centroidPoses[i].position.x = fmm_segments.centroids.y[i] * map.info.resolution + map.info.origin.position.x;
        centroidPoses[i].position.y = fmm_segments.centroids.x[i] * map.info.resolution + map.info.origin.position.y;
    }

    vector<bool> regsWithObst(fmm_segments.centroids.x.size(), false);

    double robotRadius = hypot(rx, ry);
    int robotRadiusGrid = robotRadius / map.info.resolution;

    nm_dist = map.info.resolution/10;


    _regsWithObst = regsWithObst;
    _centroidPoses = centroidPoses;

    centroids = fmm_segments.centroids;
    _Nobst = No;

    while(ros::ok()){

    	// Actualizar las posiciones de todos los agentes
    	master.updatePositions();

    	// Actualizar los footprints de todos los agentes
    	master.updateFootprints();

    	// Calcular distancias entre todos
    	master.updateRRDistances();
    	master.updateOODistances();
    	master.updateRODistances();

    	// Publicar las colisiones
    	collMsg.data = master.getRRCollision();
    	rr_coll_pub.publish(collMsg);

    	collMsg.data = master.getOOCollision();
    	oo_coll_pub.publish(collMsg);
        colliding = master.getROCollision();
    	collMsg.data = colliding;
    	ro_coll_pub.publish(collMsg);

        // --------------------------------------------------------------------------------------------
        // Registro de los resultados
        // --------------------------------------------------------------------------------------------

        // Posiciones del robot
        robotPositions = master.getRobotPositions();
        xr = robotPositions.x[0]; yr = robotPositions.y[0];
        
        _xr = xr; _yr = yr;
        xrobot = robotPositions.poses[0].position.x; yrobot = robotPositions.poses[0].position.y;

        // Posiciones de los obstáculos
        obstaclePositions = master.getObstaclePositions();
        _obstaclePositions = obstaclePositions;

        // Registrar resultados
        register_results();

        // Almacenar la posición previa del robot (para los resultados)
        _xrobot = xrobot; _yrobot = yrobot;

        totalPathPose.header.stamp = ros::Time::now(); totalPathPose.header.frame_id = map_topic; totalPathPose.pose = robotPositions.poses[0];
        totalPath.poses.push_back(totalPathPose);
        
        // --------------------------------------------------------------------------------------------

    	ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return 0;

}