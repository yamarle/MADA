
// Funciones que usa nuestro método
// Utilidades
#include <mada/mada_util.h>
#include <mada/util/funciones.hpp>
#include <mada/util/fmm_2.hpp>
#include <mada/util/path.hpp>
#include <mada/util/bresenham.hpp>
#include <mada/util/geometry.hpp>
#include <mada/util/generic_fmm.hpp>

#include <mada/projPoints.h>
#include <mada/mada_publisher.hpp>

// Exploración
#include <mada/exploration_functions.hpp>
#include <mada/dynamic_areas.hpp>

// Para sincronizar los mensajes
#include <message_filters/subscriber.h>

// Para las funciones del laser
#include <tf/message_filter.h>
#include <laser_geometry/laser_geometry.h>

#include <mada/mada_master.hpp>

#include <chrono>

// Para la planificación de caminos
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <navfn/navfn_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

// TODAS LAS VARIABLES GLOBALES QUE VA A USAR EL EXPLORADOR

// PÁRAMETROS DE ENTRADA
double sX, sY; // Tamaño del robot
string robot_name;
string map_topic, pose_topic, laser_topic, pathToFollow_topic;
string robot_frame;
double laser_range;

double occup_thres = 0;
int forget_dynamic_areas = 0;

int monitoring_type = 0; // 0: once, 1: max_mission_time

// Posiciones que usa el robot
pair<int, int> gridPosition, goalGridPosition, waypointGrid;
Poss<int> currPoss;
geometry_msgs::PoseWithCovarianceStamped realPose;
geometry_msgs::Pose currPose, goalPose, waypointPose;
geometry_msgs::Point explPoint, goalPoint, wayPoint;

nav_msgs::Odometry robotOdomPoseMsg, _robotOdomPoseMsg;

// Posiciones del laser del explorador
sensor_msgs::LaserScan laserMsg; // El mensaje inicial de ROS
vector<geometry_msgs::Point> laserPos, laserGridPos;
vector<geometry_msgs::Pose> laserPoses, laserGridPoses;
Poss<int> laserPoss;
geometry_msgs::Pose robotLaserPose;
pair<int, int> robotLaserPosition;

bool inCollision = false;

// El grid del mapa (de ocupación → el que uso para planificar mis cosas: FMM, segmentar, path planning ... )
vector<vector<float>> grid;
int gsx, gsy;

nav_msgs::OccupancyGrid mapMsg;
sensor_msgs::PointCloud laserPointCloud;

// Cosas para el planificador/navegador
double maxLinearVel = 0.65;
double maxAngularVel = 1.0;
double robotRadius = 0.5;
double inflFact = 1.0;

int robotRadiusGrid;

double permissive_rotation_velocity = 0.25;

// Variables para los resultados
nav_msgs::Path totalPath; geometry_msgs::PoseStamped totalPathPose;
float missionTime = 0; // Tiempo que se emplea para terminar la misión
float max_mission_time = 600; // Tiempo máximo que se considera para terminar la missión
string result_filename = "resultado.txt";
ros::Time beginTime, endTime;

// (v,w) que se calculan para ir al goal
double linearVel = 0.0;
double angularVel = 0.0;
double linearAcc = maxLinearVel/10; // (Valor de la página de ROS: 2.5)
double angularAcc = maxAngularVel/10; // (Valor de la página de ROS: 3.2)
geometry_msgs::Twist cmd_vel; // Comandos de velocidad que se calculan
geometry_msgs::Twist _cmd_vel; // Comandos de velocidad anteriores
geometry_msgs::Twist cmd_vel_laser; // Comandos de velocidad que se registran cuando se registran las posiciones del láser

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robotPoseMsg)
{
    currPose = robotPoseMsg->pose.pose;
    gridPosition = mada_util::transformToGridCoordinates(mapMsg, currPose);
    currPoss.x[0] = gridPosition.first; currPoss.y[0] = gridPosition.second;

    //totalPathPose.pose = currPose;
    //totalPath.poses.push_back(totalPathPose);

}

void odomPoseCallback(const nav_msgs::Odometry::ConstPtr& robotPoseMsg)
{
    currPose = robotPoseMsg->pose.pose;
    gridPosition = mada_util::transformToGridCoordinates(mapMsg, currPose);
    currPoss.x[0] = gridPosition.first; currPoss.y[0] = gridPosition.second;

    robotOdomPoseMsg = *robotPoseMsg;

    //totalPathPose.pose = currPose;
    //totalPath.poses.push_back(totalPathPose);

}

void colliderCallback(std_msgs::Bool collMsg)
{
    inCollision = collMsg.data;
}

//****************************************************************************************************************

class LaserScanToPointCloud{

public:

    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    string mapTopic;
    LaserScanToPointCloud(ros::NodeHandle n, string mapTopic) : 
        n_(n),
        laser_sub_(n_, laser_topic, 10), // queue size (cantidad de mensajes almacenados)
        laser_notifier_(laser_sub_, listener_, robot_frame, 10) // queue size (cantidad de mensajes almacenados)
    {
        this->mapTopic = mapTopic;
        laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        //scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        sensor_msgs::PointCloud cloud;

        laserMsg = *scan_in; // Almaceno el propio mensaje

        // Almaceno la pose desde la cual se han registrado los laserazos
        robotLaserPose = currPose;
        robotLaserPosition = mada_util::transformToGridCoordinates(mapMsg, currPose);
        // Y los comandos de velocidad aplicados al robot cuando se han registrado los laserazos
        cmd_vel_laser = cmd_vel;

        try{
            projector_.transformLaserScanToPointCloud(mapTopic, *scan_in, cloud, listener_, laserMsg.range_max + 0.00001);
            //projector_.transformLaserScanToPointCloud(mapTopic, laserMsg, cloud, listener_, laserMsg.range_max + 0.00001);
        }catch (tf::TransformException& e){
            cout << e.what();
            return;
        }

        // Do something with cloud.

        laserPoses.clear();
        laserGridPoses.clear();
        laserGridPos.clear();
        laserPoss.clear();

        float angle;
        geometry_msgs::Pose pose;

        for(int i=0; i<cloud.points.size(); i++){
            // Formar las variables refenretes al laser
            // Crear un punto de geometría con la posición en el grid
            pose.position.x = cloud.points[i].x;
            pose.position.y = cloud.points[i].y;
            pose.position.z = cloud.points[i].z;

            // Creo el quaternion
            angle = laserMsg.angle_min + i * laserMsg.angle_increment;
            pose.orientation = tf::createQuaternionMsgFromYaw(angle);

            laserPoses.push_back(pose);

            laserGridPoses.push_back(pose);
            laserGridPos.push_back(mada_util::transform2GridCoordinates(mapMsg, pose.position.x, pose.position.y));
            laserPoss.push(laserGridPos[laserGridPos.size()-1].y, laserGridPos[laserGridPos.size()-1].x);
            

        }

    }
};


// ***************************************************************************************************************************************

void obstaclesFill(Poss<int> &visible_obstacles, vector<geometry_msgs::Point> &visible_obstacles_pos, const nav_msgs::OccupancyGrid& map, vector<vector<float>> static_map, double robotRadius)
{
    // "Agrupar" los obstáculos por cercanía
    vector<vector<geometry_msgs::Point>> obst_clusters_points = pointProj::distance_clustering(visible_obstacles_pos, robotRadius);
    int _x, _y;
    vector<Poss<int>> obst_clusters(obst_clusters_points.size());
    for(int i=0; i<obst_clusters_points.size(); i++){
        for(int j=0; j<obst_clusters_points[i].size(); j++){
            _x = (obst_clusters_points[i][j].y - map.info.origin.position.y)/map.info.resolution;
            _y = (obst_clusters_points[i][j].x - map.info.origin.position.x)/map.info.resolution;
            if(check_pos_in_grid(_x, _y, static_map))
                obst_clusters[i].push(_x, _y);
            //obst_clusters[i] = fix_poss_in_map(obst_clusters[i], static_map);
        }
    }

    // "Unir" los puntos (insertar puntos al medio)
    dynamic_areas::Areas obst_areas = dynamic_areas::obtain_areas(obst_clusters, static_map);
    visible_obstacles.clear();
    visible_obstacles_pos.clear();
    for(int i=0; i<obst_areas.poss.size(); i++){
        visible_obstacles.append(obst_areas.poss[i]);
        for(int j=0; j<obst_areas.poss[i].x.size(); j++){
            visible_obstacles_pos.push_back(mada_util::tfGridPos2MapPoint(map, obst_areas.poss[i].y[j], obst_areas.poss[i].x[j]));
        }
    }
}

// ***************************************************************************************************************************************

int main(int argc, char** argv) {
    
    std::string node_name = "mada_dwa";

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ros::Rate loopRate(50);

    // Leer los parámetros de entrada del planner
    ros::NodeHandle nh_priv("~"); // Nodo para la lectura de los parámetros
    mada_util::getPlannerParams(nh_priv, robot_name, map_topic, pose_topic, robot_frame, laser_topic, pathToFollow_topic, sX, sY, laser_range, maxLinearVel, maxAngularVel, occup_thres, forget_dynamic_areas, max_mission_time, inflFact, monitoring_type);

    cout<<"**************************************************"<<endl;
    cout<<"   MADA PLANNER"<<endl;
    cout<<"robot name:  "<<robot_name<<endl;
    cout<<"map topic:   "<<map_topic<<endl;
    cout<<"pose topic:  "<<pose_topic<<endl;
    cout<<"robot frame: "<<robot_frame<<endl;
    cout<<"laser topic: "<<laser_topic<<endl;
    cout<<"path topic:  "<<pathToFollow_topic<<endl;
    cout<<"tamaño:      "<<sX<<" x "<<sY<<endl;
    cout<<"rango laser: "<<laser_range<<endl;
    cout<<"vel lineal:  "<<maxLinearVel<<endl;
    cout<<"vel angular: "<<maxAngularVel<<endl;
    cout<<"occup thres: "<<occup_thres<<endl;
    cout<<"forget obst: "<<forget_dynamic_areas<<endl;
    cout<<"max time:    "<<max_mission_time<<endl;
    cout<<"infl fact:   "<<inflFact<<endl;
    cout<<"monitor  :   "<<monitoring_type<<endl;
    cout<<"**************************************************"<<endl;

    mada_util::Config conf;
    conf.readConfig(result_filename);

    robotRadius = hypot(sX, sY);

    currPose.position.x = -INF;

    currPoss(1);

    permissive_rotation_velocity = maxAngularVel/4;
    if(permissive_rotation_velocity < 0.25) permissive_rotation_velocity = 0.25;

    // razón del fin de la misión
    // - 0: la misión ha terminado por haber colisionado con un obstáculo
    // - 1: la misión ha terminado explorando todo el escenario
    int mission_end = -1;
    int wait_it = 0;

    // Para la monitorización recurrente
    vector<float> monitored_rec;
    vector<float> distances_rec;
    vector<float> times_rec;

    //nav_msgs::OccupancyGrid map;
    vector<vector<int>> occGrid;
    Poss<int> obst;
    ros::Subscriber occGridSub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, boost::bind(mada_util::gridCallback, _1, boost::ref(mapMsg), boost::ref(occGrid), boost::ref(grid)));

    // Subscriber laser
    LaserScanToPointCloud lstopc(nh, map_topic);
    ros::Subscriber poseSub;
    if(pose_topic.find("amcl")>=0 && pose_topic.find("amcl")<pose_topic.size()-1){
        // Subscriber pose con AMCL y fake localization
        poseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1, &amclPoseCallback);
    }else if((pose_topic.find("odom")>=0 && pose_topic.find("odom")<pose_topic.size()-1) || (pose_topic.find("ground_truth")>=0 && pose_topic.find("ground_truth")<pose_topic.size()-1)){
        // Subscriber pose con odometría
        poseSub = nh.subscribe<nav_msgs::Odometry>(pose_topic, 100, &odomPoseCallback);
    }

    // Suscriptor para chequear las colisiones entre el robot y los obstáculos
    ros::Subscriber collSub = nh.subscribe<std_msgs::Bool>("mada_master/ro_collision", 100, &colliderCallback);

    float waitTime = 1;
    ros::Time mapTB, mapTE;
    mapTB = ros::Time::now();
    while(!grid.size() || currPose.position.x < -10000 || gridPosition.first < -10000){
        // Me quedo aquí hasta recibir el mapa
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        
        mapTE = ros::Time::now();

        cout<<"WAITING MAP AND POSE ... ("<<waitTime - (mapTE.toSec()-mapTB.toSec())<<")"<<endl;

        if(mapTE.toSec() - mapTB.toSec() > waitTime) return 0;
    }

    waitTime = 3;
    robotLaserPosition.first = -100000;
    while(robotLaserPosition.first < -10000){
        ros::spinOnce(); 
        ros::Duration(0.1).sleep();
        
        mapTE = ros::Time::now();

        cout<<"WAITING FOR LASER ... ("<<waitTime - (mapTE.toSec()-mapTB.toSec())<<")"<<endl;

        if(mapTE.toSec() - mapTB.toSec() > waitTime) return 0;
    }
    
    if(laser_range > laserMsg.range_max) laser_range = laserMsg.range_max;

    robotRadiusGrid = robotRadius / mapMsg.info.resolution;

    // ------------------------------------------------------------------

    cout<<endl<<endl;
    cout<<"----------> "<<robot_name<<endl;
    cout<<endl;

    std::string llc_name = "local_costmap";

    // Fijar los parámetros del costmaps

    // Definir parámetros para el costmap local
    mada_util::setLocalCostmapsParams(nh, node_name, llc_name, robot_name, sX, sY, robotRadiusGrid, robotRadiusGrid);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    costmap_2d::Costmap2DROS local_costmap(llc_name, tfBuffer);

    base_local_planner::TrajectoryPlannerROS local_planner("base_local_planner", &tfBuffer, &local_costmap);

    // Variables para el planificador global
    geometry_msgs::PoseStamped currPosPlanner;
    geometry_msgs::PoseStamped goalPosPlanner;
    vector<geometry_msgs::PoseStamped> global_plan;

    // Para mover al robot
    ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>(robot_name + "cmd_vel", 10);
    geometry_msgs::Twist cmd_vel;

    gsx = grid.size(); gsy = grid[0].size();

    Path path2Goal;
    vector<geometry_msgs::Pose> path2GoalPoses, path2Follow;
    
    vector<vector<float>> obstGrad; // El gradiente desde los obstáculos
    vector<vector<float>> navArea; // Grid que se usa para la planificación de los movimientos del agente
    vector<vector<float>> navAreaOrig; // El original del anterior
    vector<vector<float>> grad; // Gradiente para todo
    fmm_segment::Segments fmm_segments;
    vector<Poss<int>> positions; // Posiciones libres y obstáculos
    vector<geometry_msgs::Point> centroidPoints;

    int syso;

    if(grid.size()){
        // ESPERAR A RECIBIR EL MAPA

        // Calculo el gradiente desde la posición del agente para obtener el espacio libre en el que se va a mover
        FMM gr(gridPosition.first, gridPosition.second, grid);
        grad = gr.compute_gradient_();

        // Cargarme las posiciones "no alcanzables" del grid
        positions = fix_free_space(grid, grad);

        // Gradiente de los obstáculos
        FMM ogr(positions[1].x, positions[1].y, grid);
        obstGrad = ogr.compute_gradient_();

        //navArea = mada_util::navigableGradient(obstGrad, (1.1*robotRadius)/mapMsg.info.resolution);
        navArea = mada_util::navigableGradient(obstGrad, (inflFact*robotRadius)/mapMsg.info.resolution);
        // Cargarme las posiciones que están cerca de los obstáculos
        positions = fix_free_space(grid, navArea);

        // Segmentar el entorno
        fmm_segments = fmm_segment::compute_segments(grid, navArea);
        for(int i=0; i<fmm_segments.frontier_poss.size(); i++) fmm_segments.contour_poss[i] = geometry::sort_points(fmm_segments.centroids.x[i], fmm_segments.centroids.y[i], fmm_segments.contour_poss[i]);

        // Almaceno una copia del área navegable para comparar durante la navegación
        navAreaOrig = navArea;

        // Centroides de los segmentos sobre el mapa
        centroidPoints = mada_util::tfGridPos2MapPos(mapMsg, fmm_segments.centroids.y, fmm_segments.centroids.x);

    }

    vector<vector<float>> static_grid = grid;

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //                         VARIABLES PARA PUBLICAR COSAS PARA REPRESENTAR EN RViZ
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    mada_publisher::Publishers publishers;
    publishers.initializePublishers(nh, robot_name, map_topic, mapMsg, pathToFollow_topic, totalPath, fmm_segments, centroidPoints);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //                         VARIABLES PARA PUBLICAR COSAS PARA EL PATH FOLLOWER
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Footprint del robot
    ros::Publisher footprintPub = nh.advertise<geometry_msgs::PolygonStamped>(robot_name  + "robot_ftprint", 10);
    geometry_msgs::PolygonStamped footprint;
    footprint.header.seq = 0; footprint.header.stamp = ros::Time::now(); footprint.header.frame_id = map_topic;

    // "Camino" del robot al punto que genera el path follower
    ros::Publisher pfPathPub = nh.advertise<nav_msgs::Path>(robot_name  + "pathFollowerPlan", 10);
    nav_msgs::Path pfPath;
    pfPath.header.seq = 0; pfPath.header.stamp = ros::Time::now(); pfPath.header.frame_id = map_topic;

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    int ax, ay; // Dimensiones del agente
    Poss<int> agent, agent_; // Posiciones que ocupa el agente sobre el grid
    //ax = ay = ceil(2*0.35/mapMsg.info.resolution);
    ax = ceil(2*sX / mapMsg.info.resolution);
    ay = ceil(2*sY / mapMsg.info.resolution);
    Poss<int> free_poss = positions[0]; // Todas las posiciones a ser alcanzadas

    // Variables que se usan en el método de exploración
    int vrange = (int)(laser_range/mapMsg.info.resolution); // Rango del laser (RECUERDA MIRAR EN STAGE) // ←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←
    vector<vector<int>> static_areas_map = fmm_segments.map;
    vector<vector<float>> static_map = grid;
    vector<Poss<int>> areas_poss = fmm_segments.poss;
    vector<vector<bool>> areas_graph = fmm_segments.graph;
    Poss<int> area_centroids = fmm_segments.centroids;
    int Nstatic_areas = area_centroids.x.size();
    exploration::Areas_exploration expl_obj(areas_poss, static_areas_map, static_map, vrange);
    int Nposs2explore = 0;
    for(int i=0; i<areas_poss.size(); i++)
        Nposs2explore += areas_poss[i].x.size();

    // Inicializar lo que observa el agente
    Poss<int> visible_poss, visible_obstacles;
    vector<geometry_msgs::Point> visible_obstacles_pos; // Esto es simplemente para representación en RViz
    Poss<int> obstacles_list; // Lista completa de obstáculos (los que se ven actualmente + los que se han visto previamente)
    vector<vector<bool>> obstacles_poss_map(gsx, vector<bool>(gsy, false)); // Mapa en el que se almacenan las posiciones de los obstáculos
    
    vector<float> map_values; // variable que almacena los que se está viendo (obstáculo o espacio libre)

    // Las variables de las posiciones de los obstáculos dinámicos
    dynamic_areas::Areas areas;
    vector<vector<bool>> trajectories_map; // Mapa que almacena las trayectorías que recorren los obstáculos dinámicos
    trajectories_map = obstacles_poss_map;
    vector<Poss<int>> clusters; // Grupos de las trayectorías que han recorrido los obstáculos dinámicos
    Poss<int> obstacles_trajectories_points; // Todos los puntos que han recorrido todos los obstáculos dinámicos
    int number_of_visited_points = 0;

    vector<geometry_msgs::Point> obstacles_trajectories_pos;

    vector<Poss<int>> active_areas_contours;

    float infl_dist = inflFact*robotRadiusGrid; // Distancia de inflado

    vector<int> number_of_obstacles;
    int max_obst = 0;
    
    vector<float> maxSegDist = fmm_segment::max_seg_dist(fmm_segments.centroids, fmm_segments.contour_poss);
    vector<vector<float>> distance_between_segments = fmm_segment::distance_between_segments(fmm_segments.centroids, fmm_segments.graph);
    vector<float> segment_obs_points = exploration::number_of_expl_points(maxSegDist, maxSegDist, vrange);
    vector<vector<int>> tree;
    vector<int> branch;
    vector<float> tree_costs;
    vector<bool> dynamism(fmm_segments.centroids.x.size(), false);

    vector<bool> dynamism_in_stareas;

    vector<float> segmentTimeout(maxSegDist.size(), 0);
    for(int i=0; i<segmentTimeout.size(); i++){
        segmentTimeout[i] = (2*maxSegDist[i]*mapMsg.info.resolution)/maxLinearVel;
    }
    vector<float> _segmentTimeout = segmentTimeout;

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    path2Goal.clear(); path2GoalPoses.clear();

    pair<float, float> poseDist;

    ros::Time currTimeBegin, currTimeEnd;

    bool explore = true;
    bool replan = false;
    bool colliding = false;

    // Variables para la determinar si se insertan las áreas dinámicas o no
    float rot = 0;
    vector<vector<int>> visible_obstacles_map(gsx, vector<int>(gsy, 0));
    vector<vector<int>> seen_map(gsx, vector<int>(gsy, 0));
    vector<vector<float>> last_seen_map(gsx, vector<float>(gsy, -1));

    int intrusions = 0;
    int collisions = 0;

    vector<double> vranges = {(laserMsg.range_max/mapMsg.info.resolution), (double)vrange};
    vector<Poss<int>> rayPoss(2);

    std::chrono::time_point<std::chrono::steady_clock> collTimeBegin, collTimeEnd;
    std::chrono::duration<double> coll_elapsed_time;
    double coll_duration = 0;

    collTimeBegin = chrono::steady_clock::time_point::min();

    std::chrono::time_point<std::chrono::steady_clock> start_timer;
    std::chrono::time_point<std::chrono::steady_clock> end_timer;

    start_timer = std::chrono::steady_clock::time_point::min();
    end_timer = std::chrono::steady_clock::time_point::min();

    std::chrono::duration<double> elapsed_time;
    double duration = 0;

    beginTime = ros::Time::now();

    vector<geometry_msgs::Pose> path2Plot;
    vector<geometry_msgs::Point> obstacles2Plot;

    int cont = 0;
    while(ros::ok() && explore && max_mission_time > 0){ // Hasta que explore todo el espacoio libre

        currTimeBegin = ros::Time::now();

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // LO que ve el explorador
        //laserPoss = fix_poss_in_map(laserPoss, grid);
        laserPoss = fix_poss_in_map(laserPoss, gsx, gsy); // eliminar las posiciones que quedan fuera del grid (por si hay un error en la localización)

        // Si estoy "mal localizado", descarto la lectura del láser
        //if(mada_util::noMovementPath(totalPath) < 10)
        /*
        if(cmd_vel.angular.z < -permissive_rotation_velocity || cmd_vel.angular.z > permissive_rotation_velocity){
            laserPoss.clear();
        }
        */

        if(cmd_vel.angular.z && abs(_cmd_vel.angular.z-cmd_vel.angular.z)) laserPoss.clear();
        else{
            path2Plot = path2GoalPoses;
            obstacles2Plot = visible_obstacles_pos;
        }

        //visible_poss = geometry::allRayPoss(gridPosition.first, gridPosition.second, laserPoss, vrange);
        //rayPoss = geometry::allRayPoss_ranges(gridPosition.first, gridPosition.second, laserPoss, vranges);
        //rayPoss = geometry::allRayPoss_ranges(robotLaserPosition.first, robotLaserPosition.second, laserPoss, vranges);
        rayPoss = geometry::allRayPoss_ranges(robotLaserPosition.first, robotLaserPosition.second, laserPoss, vranges, static_map);
        visible_poss = rayPoss[0];

        // Obtener las posiciones que ocupa el agente sobre el grid
        agent = agentPoss(gridPosition.first, gridPosition.second, ax, ay);

        // Añadir las posiciones del agente a las visibles
        visible_poss.append(agent);

        // -----------------------------------------------------------------------------------------------------------

        // Actualizo el "mapa de navegación"
        dynamic_areas::updateNavigableMapInfl(navArea, navAreaOrig, laserPoss, visible_poss, gridPosition.first, gridPosition.second, vrange, infl_dist, static_map);

        // Extraer unicamente los obstáculos dinámicos
        mada_util::dynObstPos(laserPoss.x, laserPoss.y, laserPoses, navAreaOrig, robotLaserPosition.first, robotLaserPosition.second, vrange, visible_obstacles.x, visible_obstacles.y, visible_obstacles_pos);

        // Aquí ya tengo las posiciones de los obstáculos dinámicos, ahora tengo que "juntarlos"
        //obstaclesFill(visible_obstacles, visible_obstacles_pos, mapMsg, static_map, 1.5*robotRadius);
        visible_obstacles = dynamic_areas::inflatePoss(visible_obstacles, static_map, robotRadiusGrid);

        // Actualizar el listado de obstáculos
        //dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, global_map, static_map);
        dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, navArea, navAreaOrig);

        dynamic_areas::obstaclesFixInMap(obstacles_list, navAreaOrig);

        // Actualizar el mapa que almacena las trayectorias recorridas por los obstáculos
        dynamic_areas::update_trajectories_map(visible_obstacles, trajectories_map, obstacles_trajectories_points, number_of_visited_points);

        // Agrupar trayectorias
        clusters = adjacency_clustering::cluster(obstacles_trajectories_points, static_map);
        //clusters = adjacency_clustering::cluster(obstacles_trajectories_points, robotRadiusGrid);
        //clusters = distance_clustering::cluster(obstacles_trajectories_points, 1.5*robotRadiusGrid);

        // -------------------------------------------------------------------
        
        // Obtener las áreas dinámicas
        areas = dynamic_areas::obtain_areas(clusters, static_map);

        // -----------------------------------------------------------------------------------------------------------
        // Actualizar las variables de las áreas dinámicas
        // y
        // Olvidar las áreas dinámicas
        number_of_obstacles = dynamic_areas::forget_areas(forget_dynamic_areas, areas, obstacles_list, visible_obstacles, clusters, trajectories_map, obstacles_trajectories_points, number_of_visited_points);

        // -----------------------------------------------------------------------------------------------------------

        // Insertar la información de las áreas dinámicas en el mapa para planificar/navegar
        navArea = dynamic_areas::navigable_grad_oth_infl(navAreaOrig, free_poss, obstacles_list, areas.poss, areas.contours_edges, static_map, infl_dist, number_of_obstacles, occup_thres, areas.map[gridPosition.first][gridPosition.second]);
        // Añadir tambien los obstáculos ("VISIBLES") EN EL GRID
        dynamic_areas::inflatePoss(visible_obstacles, static_map, infl_dist, navArea);
        for(int i=0; i<visible_obstacles.x.size(); i++){
            if(fmm_segments.map[visible_obstacles.x[i]][visible_obstacles.y[i]]>=0 && !dynamism[fmm_segments.map[visible_obstacles.x[i]][visible_obstacles.y[i]]]) dynamism[fmm_segments.map[visible_obstacles.x[i]][visible_obstacles.y[i]]] = true;
        }
        // -----------------------------------------------------------------------------------------------------------

        // Actualizar las variables de exploración (mapas basicamente)
        //expl_obj.update_all(visible_poss, map_values);
        map_values = get_values_from_map(rayPoss[1], navArea);
        expl_obj.update_all(rayPoss[1], map_values);

        // SELECCIÓN DE LOS NUEVOS OBJETIVOS
        // Actualizar los objetivos
        expl_obj.update_targets();

        // Calcular nuevo objetivo (partición + conjunto de puntos + goal) y el camino hacia él
        expl_obj.targets_and_path(gridPosition.first, gridPosition.second, areas_graph, distance_between_segments, segment_obs_points, fmm_segments.centroids, navArea, maxSegDist, vrange, path2Goal, tree, tree_costs, branch);

        // Obtener las particiones que atraviesa el camino al goal
        vector<int> traversed_segments = obtain_traversed_segments(path2Goal, fmm_segments.map, 0);
        expl_obj.fix_target_and_path(traversed_segments, gridPosition.first, gridPosition.second, navArea, fmm_segments.centroids, maxSegDist, vrange, path2Goal);

        // Comprobar si no es posible obtener camino, por los obstáculos dinámicos
        if(!path2Goal.tam && navArea[gridPosition.first][gridPosition.second]){ // El robot no está en inflado y no es posible obtener camino
            if(check_pos_in_grid(expl_obj.getGoalx(), expl_obj.getGoaly(), gsx, gsy) && fmm_segments.map[expl_obj.getGoalx()][expl_obj.getGoaly()]>=0 && !navArea[expl_obj.getGoalx()][expl_obj.getGoaly()]){
                dynamism_in_stareas.clear(); dynamism_in_stareas.resize(Nstatic_areas); dynamism_in_stareas[fmm_segments.map[expl_obj.getGoalx()][expl_obj.getGoaly()]] = true;
                expl_obj.update_poss_with_traversability(gridPosition.first, gridPosition.second, navArea, dynamism_in_stareas);
            }
        }

        // -----------------------------------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------------------------------

        // Comprobar si el hay un obstáculo en el goal
        /*
        replan = false;
        //if(path2Goal.x.size() && laserPoss.x.size()){ // ---> Se ha encontrado un camino al goal
        if(path2Goal.x.size()){ // ---> Se ha encontrado un camino al goal
            //if(laserPoss.x.size()){
            //agent_ = agentPoss(expl_obj.getGoalx(), expl_obj.getGoaly(), ax, ay); // El volumen del robot en la posisción del goal
            agent_ = agentPoss(path2Goal.x[path2Goal.x.size()-1], path2Goal.y[path2Goal.y.size()-1], ax, ay); // El volumen del robot en la posisción del goal
            vector<float> agent_in_goal_values_d = get_values_from_map(agent_, navArea);
            vector<float> agent_in_goal_values_s = get_values_from_map(agent_, navAreaOrig);
            for(int i=0; i<agent_in_goal_values_d.size(); i++){
                if(agent_in_goal_values_d[i] == 0 && agent_in_goal_values_s[i] != 0){
                    replan = true;
                    break;
                }
            }

            if(replan){ // No es posible ir al goal, se actualiza el árbol, se vuelve a obtener el goal/camino
                dynamism_in_stareas.clear(); dynamism_in_stareas.resize(Nstatic_areas); dynamism_in_stareas[fmm_segments.map[path2Goal.x[path2Goal.x.size()-1]][path2Goal.y[path2Goal.y.size()-1]]] = true;
                expl_obj.update_poss_with_traversability(gridPosition.first, gridPosition.second, navArea, dynamism_in_stareas);
            }

            // -------------------------------------------------------------------------------------------------
            // Obtener el camino que se le va a enviar al path follower
            // Pasar camino en el grid a puntos del mapa
            if(laserPoss.x.size()){
                path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, visible_obstacles.x, visible_obstacles.y, path2GoalPoses);
            }else{
                //path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, obstacles_list.x, obstacles_list.y, path2GoalPoses);
                for(int i=0; i<path2Follow.size(); i++)
                    path2Follow[i].orientation.x = 0.00001;
            }
        }else{
            //path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, visible_obstacles.x, visible_obstacles.y, path2GoalPoses);
            for(int i=0; i<path2Follow.size(); i++)
                path2Follow[i].orientation.x = 0.00001;
        }
        */
        
        replan = false;
        //if(path2Goal.x.size() && laserPoss.x.size()){ // ---> Se ha encontrado un camino al goal
        if(path2Goal.x.size()){ // ---> Se ha encontrado un camino al goal
            if(laserPoss.x.size()){

                replan = dynamic_areas::check_positions(path2Goal.x, path2Goal.y, navAreaOrig, navArea, ax, ay);
                if(replan){ // No es posible ir al goal, se actualiza el árbol, se vuelve a obtener el goal/camino
                    dynamism_in_stareas.clear(); dynamism_in_stareas.resize(Nstatic_areas); dynamism_in_stareas[fmm_segments.map[path2Goal.x[path2Goal.x.size()-1]][path2Goal.y[path2Goal.y.size()-1]]] = true;
                    expl_obj.update_poss_with_traversability(gridPosition.first, gridPosition.second, navArea, dynamism_in_stareas);
                }

                // -------------------------------------------------------------------------------------------------
                // Obtener el camino que se le va a enviar al path follower
                // Pasar camino en el grid a puntos del mapa
                path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, visible_obstacles.x, visible_obstacles.y, path2GoalPoses);
            }else{
                //cout<<"no veo"<<endl;
                mada_util::setFollowerPathPoses(path2Follow);
            }
        }else{
            if(laserPoss.x.size()){
                if(check_pos_in_grid(expl_obj.getGoalx(), expl_obj.getGoaly(), gsx, gsy) && fmm_segments.map[expl_obj.getGoalx()][expl_obj.getGoaly()]>=0 && navArea[expl_obj.getGoalx()][expl_obj.getGoaly()] == 0){ // El goal no está en inflado
                    dynamism_in_stareas.clear(); dynamism_in_stareas.resize(Nstatic_areas); dynamism_in_stareas[fmm_segments.map[expl_obj.getGoalx()][expl_obj.getGoaly()]] = true;
                    expl_obj.update_poss_with_traversability(gridPosition.first, gridPosition.second, navArea, dynamism_in_stareas);
                }
            }
            //path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, visible_obstacles.x, visible_obstacles.y, path2GoalPoses);
            mada_util::setFollowerPathPoses(path2Follow);
        }

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Navegador (DWA)
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // Ejecutar el planificador local (DWA)
        //if(path2Goal.x.size()){
        if(path2Follow.size()){

            // Guardar la velocidad previa
            _cmd_vel = cmd_vel;

            local_costmap.updateMap();

            // Fijar el plan global
            global_plan.clear();
            for(int i=0; i<path2Follow.size(); i++){
                geometry_msgs::PoseStamped _ps;
                _ps.header.stamp = ros::Time::now();
                _ps.header.frame_id = map_topic;
                _ps.pose = path2Follow[i];
                global_plan.push_back(_ps);
            }

            // Calcular plan local
            local_planner.setPlan(global_plan);

            // Calcular las velocidades a aplicar
            local_planner.computeVelocityCommands(cmd_vel);

            poseDist = mada_util::distPose2Path(currPose, totalPath, 50);
            //cout<<" >>>>> DISTANCIAS >>>>> "<<poseDist.first<<" + "<<poseDist.second<<endl;
            if(cmd_vel.linear.x == 0 || (poseDist.first + poseDist.second) == 0){ // NO hay un comando de velocidad viable
            //if(cmd_vel.linear.x == 0 || path2Goal.x.size()){ // NO hay un comando de velocidad viable
                // giro en una de las direcciones para poder encontrar un camino viable que venga del planificador
                if(_cmd_vel.angular.z > 0)
                    cmd_vel.angular.z = maxAngularVel;
                else
                    cmd_vel.angular.z = -maxAngularVel;
            }

            // Publicar velocidades
            //cmdVelPub.publish(cmd_vel);
            //_cmd_vel = cmd_vel;

        }else{
            cout<<"NO ME MUEVO"<<endl;
        }

        // Publico footprint del robot
        //footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::footprintOr(currPose, sX/2, (int)18), map_topic));
        //footprintPub.publish(mada_util::footprintNoOr(currPose, sX/2, (int)18, robot_frame));
        footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::footprintNoOr(currPose, sX, sY), robot_frame));
        //footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::circlePointsTr(robotRadius/2, 18), robot_frame));
        //footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::footprintNoOr(currPose, robotRadius/2, 18), map_topic));

        // Publico "camino"
        waypointPose.orientation.x = waypointPose.orientation.y = 0.0;
        pfPath = mada_util::setPlan(currPose, waypointPose, map_topic);
        pfPathPub.publish(pfPath);
        // Publico velocidades
        cmdVelPub.publish(cmd_vel);
        //_cmd_vel = cmd_vel;

        //cout<<"VELOCIDADES PUBLICADAS "; mada_util::showCmdVel(cmd_vel);

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // Actualizar la variable para continuar monitorizando o no
        explore = expl_obj.remaining_poss2explore();

        // Chequear colisión
        /*
        colliding = false;
        if(mada_util::noMovementPath(totalPath) < 10){
            agent_ = agentPoss(gridPosition.first, gridPosition.second, ax, ay);
            vector<float> agent_in_goal_values_d = get_values_from_map(agent_, navArea);
            vector<float> agent_in_goal_values_s = get_values_from_map(agent_, navAreaOrig);
            for(int i=0; i<agent_in_goal_values_d.size(); i++){
                if(agent_in_goal_values_d[i] == 0 && agent_in_goal_values_s[i] != 0){
                    colliding = true;
                    break;
                }
            }
            if(colliding){
                if(collTimeBegin == chrono::steady_clock::time_point::min()) collTimeBegin = chrono::steady_clock::now();
                collTimeEnd = chrono::steady_clock::now();
                coll_elapsed_time = collTimeEnd - collTimeBegin;
                coll_duration = coll_elapsed_time.count();
                if(coll_duration >= 5) wait_it = 20;
            }else{
                collTimeBegin = chrono::steady_clock::time_point::min();
            }
        }
        */
        if(inCollision){
            cout<<"UUUUUUUUUY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            poseDist = mada_util::distPose2Path(currPose, totalPath, 10);
            //if((poseDist.first + poseDist.second) == 0){
            if((poseDist.first) == 0){
                cout<<"COLLIDING"<<endl;
                wait_it++;
            }
        }

        // Chequear la parada en el goal
        poseDist = mada_util::distPose2Path(currPose, totalPath, 20);
        if(navArea[gridPosition.first][gridPosition.second] && (poseDist.first + poseDist.second) == 0){
            if(start_timer == chrono::steady_clock::time_point::min()) start_timer = chrono::steady_clock::now();
            // Comprobar cuanto tiempo llevo en el punto
            end_timer = chrono::steady_clock::now();
            elapsed_time = end_timer - start_timer;
            duration = elapsed_time.count();
            _segmentTimeout[fmm_segments.map[gridPosition.first][gridPosition.second]] -= duration;
            if(_segmentTimeout[fmm_segments.map[gridPosition.first][gridPosition.second]] <= 0){
                // Quitar lo puntos del área de la exploración
                if(expl_obj.get_target_area() == fmm_segments.map[gridPosition.first][gridPosition.second])
                    expl_obj.erase_area(expl_obj.get_target_area());
            }
        }else{
            // Reiniciar los tiempos
            _segmentTimeout[fmm_segments.map[gridPosition.first][gridPosition.second]] = segmentTimeout[fmm_segments.map[gridPosition.first][gridPosition.second]];
            start_timer = end_timer = std::chrono::steady_clock::time_point::min();
        }

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // PUBLICAR LOS TOPICS DE LAS COSAS QUE QUIERO REPRESENTAR
        if(visible_obstacles.x.size() == 0) visible_obstacles_pos = obstacles2Plot;
        if(path2GoalPoses.size() == 0) path2GoalPoses = path2Plot;
        publishers.publishTopics(expl_obj, areas, obstacles_trajectories_points, visible_obstacles_pos, number_of_obstacles, traversed_segments, tree, centroidPoints, totalPath, path2GoalPoses, map_topic, mapMsg);

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        if(wait_it == 20){
            explore = false;
            intrusions++;
            collisions++;
            if(mission_end < 0) mission_end = 0;
        }

        totalPathPose.pose = currPose;
        totalPath.poses.push_back(totalPathPose);

        cont++;

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        loopRate.sleep();
        
        currTimeEnd = ros::Time::now();

        max_mission_time -= (currTimeEnd.toSec() - currTimeBegin.toSec());

    }

    cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
    cmdVelPub.publish(cmd_vel);

    cout<<"FINISHED"<<endl;

    endTime = ros::Time::now();
    missionTime = endTime.toSec() - beginTime.toSec();

    if(collisions == 0 && mission_end < 0) mission_end = 1;

    int Nreal_poss2explore = expl_obj.Nreal_poss2explore();
    float explored = exploration::explored_area(expl_obj.get_real_seen_map(), navAreaOrig);

    cout<<"Travelled distance: "<<mada_util::computePathDistance(totalPath)<<endl;
    cout<<"Mission time: "<<missionTime<<endl;
    cout<<"Explored: "<<explored<<endl;

    if(mission_end == 0) cout<<"Collision"<<endl;
    else if(mission_end == 1) cout<<"Mission accomplished"<<endl;

    // "Avisar" de que la ejecución se ha terminado
    ros::Publisher shutdown_pub = nh.advertise<std_msgs::Bool>("/shutdown_signal", 10);
    std_msgs::Bool shutdown_msg;
    shutdown_msg.data = true;
    for (int i = 0; i < 5; ++i) { // Envía la señal varias veces para asegurar recepción
        shutdown_pub.publish(shutdown_msg);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    ros::shutdown(); // "Cerrar" este nodo

    return 0;
}
