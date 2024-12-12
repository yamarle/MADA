
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

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Para las funciones del laser
#include <tf/message_filter.h>
#include <laser_geometry/laser_geometry.h>

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

    std::string node_name = "greedy_dwa";
    
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ros::Rate loopRate(500);

    if(argc == 17){
        robot_name = argv[1];
        map_topic = argv[2];
        pose_topic = argv[3];
        robot_frame = argv[4];
        laser_topic = argv[5];
        pathToFollow_topic = argv[6];
        sX = atof(argv[7]);
        sY = atof(argv[8]);
        laser_range = atof(argv[9]);
        maxLinearVel = atof(argv[10]);
        maxAngularVel = atof(argv[11]);
        occup_thres = atof(argv[12]);
        forget_dynamic_areas = atoi(argv[13]);
        max_mission_time = atof(argv[14]);
        inflFact = atof(argv[15]);
        monitoring_type = atoi(argv[16]);
    }else if(argc == 18){
        robot_name = argv[1];
        map_topic = argv[2];
        pose_topic = argv[3];
        robot_frame = argv[4];
        laser_topic = argv[5];
        pathToFollow_topic = argv[6];
        sX = atof(argv[7]);
        sY = atof(argv[8]);
        laser_range = atof(argv[9]);
        maxLinearVel = atof(argv[10]);
        maxAngularVel = atof(argv[11]);
        occup_thres = atof(argv[12]);
        forget_dynamic_areas = atoi(argv[13]);
        max_mission_time = atof(argv[14]);
        inflFact = atof(argv[15]);
        monitoring_type = atoi(argv[16]);
        result_filename = argv[17];
    }

    cout<<argc<<" argumentos"<<endl;
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
    int coll_it = 0;

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
    cout<<"pose recibida"<<endl;

    float waitTime = 1;
    ros::Time mapTB, mapTE;
    mapTB = ros::Time::now();
    while(!grid.size() || currPose.position.x < -10000 || gridPosition.first < -10000){
        // Me quedo aquí hasta recibir el mapa
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        
        mapTE = ros::Time::now();

        cout<<"ESPERANDO MAPA Y LOCALIZARME ... ("<<waitTime - (mapTE.toSec()-mapTB.toSec())<<")"<<endl;

        if(mapTE.toSec() - mapTB.toSec() > waitTime) return 0;
    }

    waitTime = 3;
    robotLaserPosition.first = -100000;
    while(robotLaserPosition.first < -10000){
        ros::spinOnce(); 
        ros::Duration(0.1).sleep();
        
        mapTE = ros::Time::now();

        cout<<"ESPERANDO SINCRONIZAR POSICIÓN LASER ... ("<<waitTime - (mapTE.toSec()-mapTB.toSec())<<")"<<endl;

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

    Path path2Goal, _path2Goal;
    vector<geometry_msgs::Pose> path2GoalPoses, path2Follow, _path2Follow;
    
    vector<vector<float>> obstGrad; // El gradiente desde los obstáculos
    vector<vector<float>> navArea; // Grid que se usa para la planificación de los movimientos del agente
    vector<vector<float>> navAreaOrig; // El original del anterior
    vector<vector<float>> grad; // Gradiente para todo
    fmm_segment::Segments fmm_segments;
    vector<Poss<int>> positions; // Posiciones libres y obstáculos
    vector<geometry_msgs::Point> centroidPoints;

    int syso;

    beginTime = ros::Time::now();

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

    endTime = ros::Time::now();

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

    // Inicializar lo que observa el agente (Sobre el grid)
    Poss<int> visible_poss, visible_obstacles;
    vector<geometry_msgs::Point> visible_obstacles_pos;
    Poss<int> obstacles_list; // Lista completa de obstáculos (los que se ven actualmente + los que se han visto previamente)
    vector<vector<bool>> obstacles_poss_map(gsx, vector<bool>(gsy, false)); // Mapa en el que se almacenan las posiciones de los obstáculos
    
    vector<float> map_values; // variable que almacena los que se está viendo (obstáculo o espacio libre)

    float infl_dist = inflFact*robotRadiusGrid; // Distancia de inflado

    vector<double> vranges = {(laserMsg.range_max/mapMsg.info.resolution), (double)vrange};
    vector<Poss<int>> rayPoss(2);
    rayPoss = geometry::allRayPoss_ranges(robotLaserPosition.first, robotLaserPosition.second, laserPoss, vranges);

    exploration::Map_exploration expl_obj(static_map, static_map, vrange);
    map_values = get_values_from_map(rayPoss[1], navArea); // Qué es lo que se está viendo
    expl_obj.update_maps(gridPosition.first, gridPosition.second, rayPoss[1], map_values); // Actualizar las utilidades en base a lo que se ve


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

    std::chrono::time_point<std::chrono::steady_clock> collTimeBegin, collTimeEnd;
    std::chrono::duration<double> coll_elapsed_time;
    double coll_duration = 0;

    collTimeBegin = chrono::steady_clock::time_point::min();

    beginTime = ros::Time::now();

    int cont = 0;
    while(ros::ok() && explore && max_mission_time > 0){ // Hasta que explore todo el espacoio libre

        currTimeBegin = ros::Time::now();

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // LO que ve el explorador
        //laserPoss = fix_poss_in_map(laserPoss, grid);
        laserPoss = fix_poss_in_map(laserPoss, gsx, gsy); // eliminar las posiciones que quedan fuera del grid (por si hay un error en la localización)

        // Si estoy "mal localizado", descarto la lectura del láser
        //if(mada_util::noMovementPath(totalPath) < 10)
        if(cmd_vel.angular.z < -permissive_rotation_velocity || cmd_vel.angular.z > permissive_rotation_velocity){
            laserPoss.clear();
        }

        //visible_poss = geometry::allRayPoss(gridPosition.first, gridPosition.second, laserPoss, vrange);
        //rayPoss = geometry::allRayPoss_ranges(gridPosition.first, gridPosition.second, laserPoss, vranges);
        rayPoss = geometry::allRayPoss_ranges(robotLaserPosition.first, robotLaserPosition.second, laserPoss, vranges);
        visible_poss = rayPoss[0];

        // Obtener las posiciones que ocupa el agente sobre el grid
        agent = agentPoss(gridPosition.first, gridPosition.second, ax, ay);

        // Añadir las posiciones del agente a las visibles
        visible_poss.append(agent);

        map_values = get_values_from_map(visible_poss, navArea); // Qué es lo que se está viendo

        // -----------------------------------------------------------------------------------------------------------

        // Actualizo el "mapa de navegación"
        dynamic_areas::updateNavigableMapInfl(navArea, navAreaOrig, laserPoss, visible_poss, gridPosition.first, gridPosition.second, vrange, infl_dist, static_map);

        // Extraer unicamente los obstáculos dinámicos
        mada_util::dynObstPos(laserPoss.x, laserPoss.y, laserPoses, navAreaOrig, robotLaserPosition.first, robotLaserPosition.second, vrange, visible_obstacles.x, visible_obstacles.y, visible_obstacles_pos);
                
        // Aquí ya tengo las posiciones de los obstáculos dinámicos, ahora tengo que "juntarlos"
        //obstaclesFill(visible_obstacles, visible_obstacles_pos, mapMsg, static_map, robotRadius);

        dynamic_areas::inflatePoss(visible_obstacles, static_map, infl_dist, navArea);

        // Actualizar las variables de exploración (mapas basicamente)
        //expl_obj.update_all(visible_poss, map_values);
        map_values = get_values_from_map(rayPoss[1], navArea);
        expl_obj.update_maps(gridPosition.first, gridPosition.second, rayPoss[1], map_values);

        // Obtener camino al goal
        path2Goal = expl_obj.path2next_goal(gridPosition.first, gridPosition.second, navArea);

        // -----------------------------------------------------------------------------------------------------------

        // Comprobar si el hay un obstáculo en el goal
        replan = false;
        if(path2Goal.x.size()){
            //path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, visible_obstacles.x, visible_obstacles.y, path2GoalPoses);
            if(laserPoss.x.size()){
                path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, visible_obstacles.x, visible_obstacles.y, path2GoalPoses);
            }else{
                //path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, obstacles_list.x, obstacles_list.y, path2GoalPoses);
                for(int i=0; i<path2Follow.size(); i++)
                    path2Follow[i].orientation.x = 0.00001;
            }
        }else{
            if(navArea[gridPosition.first][gridPosition.second] || laserPoss.x.size() == 0){
                path2Goal.x = {gridPosition.first, gridPosition.first}; path2Goal.y = {gridPosition.second, gridPosition.second};
                path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, visible_obstacles.x, visible_obstacles.y, path2GoalPoses);
            }else{
                for(int i=0; i<path2Follow.size(); i++)
                    path2Follow[i].orientation.x = 0.00001;
            }
        }

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Navegador (DWA)
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // Ejecutar el planificador local (DWA)
        //if(path2Goal.x.size()){
        if(path2Follow.size()){
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
            cmdVelPub.publish(cmd_vel);

            _cmd_vel = cmd_vel;

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

        _cmd_vel = cmd_vel;

        //cout<<"VELOCIDADES PUBLICADAS "; mada_util::showCmdVel(cmd_vel);

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // Actualizar la variable para continuar monitorizando o no
        if(path2Goal.tam == 0){
            //wait_it++;
            poseDist = mada_util::distPose2Path(currPose, totalPath, 10);
            if((poseDist.first + poseDist.first) == 0)
                wait_it++;
            else
                wait_it = 0;
        }else{
            wait_it = 0;
        }

        // Chequear colisión
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
                if(coll_duration >= 5) coll_it = 20;
            }else{
                collTimeBegin = chrono::steady_clock::time_point::min();
            }
        }

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // PUBLICAR LOS TOPICS DE LAS COSAS QUE QUIERO REPRESENTAR
        publishers.publishTopics(expl_obj, visible_obstacles_pos, totalPath, path2GoalPoses, map_topic);

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        if(wait_it == 20){
            explore = false;
            if(mission_end < 0) mission_end = 0;
        }

        if(coll_it == 20){
            explore = false;
            intrusions++;
            collisions++;
            if(mission_end < 0) mission_end = 1;
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

    float explored = exploration::explored_area(expl_obj.get_seen_map(), navAreaOrig);

    cout<<"Travelled distance: "<<mada_util::computePathDistance(totalPath)<<endl;
    cout<<"Mission time: "<<missionTime<<endl;
    cout<<"Explored: "<<explored<<endl;

    if(mission_end == 0) cout<<"Collision"<<endl;
    else if(mission_end == 1) cout<<"Mission accomplished"<<endl;

    //ros::shutdown(); // "Cerrar" este nodo

    // He terminado, "cierro" todos los nodos
    //syso = std::system("rosnode kill --all");
    //syso = std::system("killall -9 rosmaster");
    //syso = std::system("killall -9 roscore");

    //syso = std::system("rosclean purge -h");

    return 0;
}
