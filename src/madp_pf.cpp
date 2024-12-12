
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
float precomputationTime; // Tiempo que se ha requerido para realizar los cálculos offline (gradientes, segmentar entorno, ...)
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

vector<Poss<int>> clusterPoss(Poss<int> poss, vector<vector<int>> segmentsMap, int nsegments, vector<int> &segs)
{
    vector<Poss<int>> res;
    segs.clear();
    bool f;
    for(int i=0; i<poss.x.size(); i++){
        if(segmentsMap[poss.x[i]][poss.y[i]] >= 0 && segmentsMap[poss.x[i]][poss.y[i]] < nsegments-1){
            f = false;
            for(int j=0; j<segs.size(); j++){
                if(segmentsMap[poss.x[i]][poss.y[i]] == segs[j]){
                    res[j].push(poss.x[i], poss.y[i]);
                    f = true;
                    break;
                }
            }
            if(!f){
                res.resize(res.size()+1);
                res[res.size()-1].push(poss.x[i], poss.y[i]);
                segs.push_back(segmentsMap[poss.x[i]][poss.y[i]]);
            }
        }
    }
    return res;   
} // Separar un conjunto de puntos por segmentos que atraviesan

dynamic_areas::Areas segs2areas(vector<int> segs, fmm_segment::Segments segments)
{
    dynamic_areas::Areas res;

    res.N = segs.size();
    res.centroids(res.N); res.real_centroids(res.N);
    res.contours.resize(res.N); res.contours_edges.resize(res.N);
    res.poss.resize(res.N);
    res.map.resize(segments.map.size(), vector<int>(segments.map[0].size(),-1));
    for(int i=0; i<segs.size(); i++){
        res.centroids.x[i] = res.real_centroids.x[i] = segments.centroids.x[segs[i]];
        res.centroids.y[i] = res.real_centroids.y[i] = segments.centroids.y[segs[i]];
        res.contours[i] = segments.contour_poss[segs[i]];
        res.contours_edges[i] = segments.contour_poss[segs[i]];
        res.poss[i] = segments.poss[segs[i]];
        for(int j=0; j<res.poss[i].x.size(); j++){
            res.map[res.poss[i].x[j]][res.poss[i].y[j]] = i;
        }
        for(int j=0; j<res.contours[i].x.size(); j++){
            res.map[res.contours[i].x[j]][res.contours[i].y[j]] = i;
        }
    }

    return res;
} // Transformar los segmentos en el objeto de áreas dinámicas

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer");
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
    cout<<"result file: "<<result_filename<<endl;

    mada_util::Config conf;
    conf.readConfig(result_filename);

    robotRadius = hypot(sX, sY);

    currPose.position.x = -INF;

    currPoss(1);

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
    cout<<"pose recibida"<<endl;

    bool save = true;

    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tfBuffer);
    while(!tfBuffer.canTransform(map_topic, robot_name + "base_laser_link", ros::Time(), ros::Duration(3.0))){
        cout<<"ESPERANDO tf "<<map_topic<<" a "<<robot_name + "base_laser_link"<<endl;
    }

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
    precomputationTime = endTime.toSec() - beginTime.toSec();

    vector<vector<float>> static_grid = grid;

    //save = false;

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

    // Comandos de velocidad para mover al robot
    ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>(pathToFollow_topic, 1);

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

    // Inicializar lo que observa el agente (Sobre el grid)
    Poss<int> visible_poss, visible_obstacles;
    vector<geometry_msgs::Point> visible_obstacles_pos;
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

    vector<int> dyn_segs; // Segmentos en los que se hay obstáculos dinámicos

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

    vector<int> Nposs2explore_tot;
    vector<float> dist2static_obstacles, dist2dynamic_obstacles;
    vector<float> travelled_distance;
    vector<float> computationTime;

    int intrusions = 0;
    int collisions = 0;

    vector<double> vranges = {(laserMsg.range_max/mapMsg.info.resolution), (double)vrange};
    vector<Poss<int>> rayPoss(2);

    std::chrono::time_point<std::chrono::steady_clock> computationTimeBegin, computationTimeEnd;
    std::chrono::duration<double> comp_elapsed_time;
    double comp_duration = 0;

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

    int cont = 0;
    while(ros::ok() && explore && max_mission_time > 0){ // Hasta que explore todo el espacoio libre

        currTimeBegin = ros::Time::now();

        computationTimeBegin = chrono::steady_clock::now();

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

        // -----------------------------------------------------------------------------------------------------------

        // Actualizo el "mapa de navegación"
        dynamic_areas::updateNavigableMapInfl(navArea, navAreaOrig, laserPoss, visible_poss, gridPosition.first, gridPosition.second, vrange, infl_dist, static_map);

        // Extraer unicamente los obstáculos dinámicos
        mada_util::dynObstPos(laserPoss.x, laserPoss.y, laserPoses, navAreaOrig, robotLaserPosition.first, robotLaserPosition.second, vrange, visible_obstacles.x, visible_obstacles.y, visible_obstacles_pos);
                
        // Aquí ya tengo las posiciones de los obstáculos dinámicos, ahora tengo que "juntarlos"
        obstaclesFill(visible_obstacles, visible_obstacles_pos, mapMsg, static_map, robotRadius);

        // Actualizar el listado de obstáculos
        //dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, global_map, static_map);
        dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, navArea, navAreaOrig);

        dynamic_areas::obstaclesFixInMap(obstacles_list, navAreaOrig);

        // Actualizar el mapa que almacena las trayectorias recorridas por los obstáculos
        dynamic_areas::update_trajectories_map(visible_obstacles, trajectories_map, obstacles_trajectories_points, number_of_visited_points);

        // Agrupar trayectorias
        clusters = clusterPoss(obstacles_trajectories_points, fmm_segments.map, fmm_segments.centroids.x.size(), dyn_segs);
        
        // Obtener las áreas dinámicas
        //areas = dynamic_areas::obtain_areas(clusters, static_map);
        areas = segs2areas(dyn_segs, fmm_segments);

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

        // Almacenar el camino antiguo
        //_path2Goal = path2Goal;

        // Calcular nuevo objetivo (partición + conjunto de puntos + goal) y el camino hacia él
        expl_obj.targets_and_path(gridPosition.first, gridPosition.second, areas_graph, distance_between_segments, segment_obs_points, fmm_segments.centroids, navArea, maxSegDist, vrange, path2Goal, tree, tree_costs, branch);

        // Obtener las particiones que atraviesa el camino al goal
        vector<int> traversed_segments = obtain_traversed_segments(path2Goal, fmm_segments.map, 0);
        expl_obj.fix_target_and_path(traversed_segments, gridPosition.first, gridPosition.second, navArea, fmm_segments.centroids, maxSegDist, vrange, path2Goal);

        // Comprobar si no es posible obtener camino, por los obstáculos dinámicos
        if(!path2Goal.tam && navArea[gridPosition.first][gridPosition.second]){ // El robot no está en inflado y no es posible obtener camino
            if(check_pos_in_grid(expl_obj.getGoalx(), expl_obj.getGoaly(), gsx, gsy) && navArea[expl_obj.getGoalx()][expl_obj.getGoaly()]){
                dynamism_in_stareas.clear(); dynamism_in_stareas.resize(Nstatic_areas); dynamism_in_stareas[fmm_segments.map[expl_obj.getGoalx()][expl_obj.getGoaly()]] = true;
                expl_obj.update_poss_with_traversability(gridPosition.first, gridPosition.second, navArea, dynamism_in_stareas);
            }
        }

        // -----------------------------------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------------------------------

        // Comprobar si el hay un obstáculo en el goal
        replan = false;
        //if(path2Goal.x.size() && laserPoss.x.size()){ // ---> Se ha encontrado un camino al goal
        if(path2Goal.x.size()){ // ---> Se ha encontrado un camino al goal
            if(laserPoss.x.size()){
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
                path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, visible_obstacles.x, visible_obstacles.y, path2GoalPoses);
                // Almacenar el camino antiguo
                //_path2Goal = path2Goal;
            }
        }else{
            //path2Follow = mada_util::setFollowerPathPoses_(path2Goal.x, path2Goal.y, navArea, mapMsg, visible_obstacles.x, visible_obstacles.y, path2GoalPoses);
            for(int i=0; i<path2Follow.size(); i++)
                path2Follow[i].orientation.x = 0.00001;
        }

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Path Follower
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        //cout<<"following path ..."<<endl;
        if(path2Follow.size()){
            double d, d2g;
            waypointPose = mada_util::selectDestination(currPose, path2Follow, robotRadius, d, d2g);

            // Calcular (v,w)
            if(path2Follow.size()){
                if(d2g > 2*robotRadius) cmd_vel = mada_util::computeVelocities(currPose, waypointPose, 1, maxLinearVel, maxAngularVel);
                else cmd_vel = mada_util::computeVelocities(currPose, waypointPose, d, maxLinearVel, maxAngularVel);

                //if(d2g > 2*robotRadius) cmd_vel = mada_util::computeVelocitiesAcc(currPose, waypointPose, 1, maxLinearVel, maxAngularVel, _cmd_vel.linear.x, _cmd_vel.angular.z, linearAcc, angularAcc);
                //else cmd_vel = mada_util::computeVelocitiesAcc(currPose, waypointPose, 1, maxLinearVel, maxAngularVel, _cmd_vel.linear.x, _cmd_vel.angular.z, linearAcc, angularAcc);
            }else cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
        }else{
            cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
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

        //mada_util::showCmdVel(cmd_vel);

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // Actualizar la variable para continuar monitorizando o no
        explore = expl_obj.remaining_poss2explore();

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
                if(coll_duration >= 5) wait_it = 20;
            }else{
                collTimeBegin = chrono::steady_clock::time_point::min();
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

        computationTimeEnd = chrono::steady_clock::now();

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // PUBLICAR LOS TOPICS DE LAS COSAS QUE QUIERO REPRESENTAR

        publishers.publishTopics(expl_obj, areas, obstacles_trajectories_points, visible_obstacles_pos, number_of_obstacles, traversed_segments, tree, centroidPoints, totalPath, path2GoalPoses, map_topic, mapMsg);

        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        if(wait_it == 20){
            explore = false;
            intrusions++;
            collisions++;
            if(mission_end < 0) mission_end = 0;
        }

        if(monitoring_type){
            if(explore == false){
                if(save){
                // Almacenar las variables
                    monitored_rec.push_back(exploration::explored_area(expl_obj.get_real_seen_map(), navAreaOrig));
                    distances_rec.push_back(mada_util::computePathDistance(totalPath));
                    times_rec.push_back(max_mission_time);
                }

                // Reiniciar variables: 
                expl_obj.set(areas_poss, static_areas_map, static_map, vrange); // "exporador"
                explore = true; // variable para continuar explorando

                totalPath.poses.clear(); // Camino total recorrido por el robot
                if(!navArea[gridPosition.first][gridPosition.second])
                    navArea = navAreaOrig; // Área navegable
                //obstacles_trajectories_points.clear(); // Posisiones recorridas por los obstáculos
            }
        }

        totalPathPose.pose = currPose;
        totalPath.poses.push_back(totalPathPose);

        cont++;

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        loopRate.sleep();
        
        currTimeEnd = ros::Time::now();

        max_mission_time -= (currTimeEnd.toSec() - currTimeBegin.toSec());

        if(save){
            // Almacenar variables de los resultados
            Nposs2explore_tot.push_back(expl_obj.Nremaining_poss2explore());
            dist2dynamic_obstacles.push_back(distance2closest_point(gridPosition.first, gridPosition.second, visible_obstacles) * mapMsg.info.resolution);
            dist2static_obstacles.push_back(obstGrad[gridPosition.first][gridPosition.second] * mapMsg.info.resolution);
            travelled_distance.push_back(mada_util::computePathDistance(totalPath));
            comp_elapsed_time = computationTimeEnd - computationTimeBegin;
            comp_duration = comp_elapsed_time.count();
            computationTime.push_back(comp_duration);
        }

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

    // Guardar los resultados en fichero
    if(save){

        // Variables totales de la misión
        vector<float> res_vect = {mada_util::computePathDistance(totalPath), precomputationTime, missionTime, explored, (float)intrusions, (float)collisions, (float)mission_end};
        vector<float> size_vect = {(float)sX, (float)sY, (float)robotRadius, (float)ax, (float)ay};
        save_vect(result_filename.c_str(), res_vect);
        save_vect_(result_filename.c_str(), size_vect, "size");
        save_vect_(result_filename.c_str(), Nposs2explore_tot, "poss2explore");
        save_vect_(result_filename.c_str(), travelled_distance, "distance");
        save_vect_(result_filename.c_str(), dist2dynamic_obstacles, "dist2dynObst");
        save_vect_(result_filename.c_str(), dist2static_obstacles, "dist2stObst");
        save_vect_(result_filename.c_str(), computationTime, "computationTime");

        // Variables para la misión de monitorización recurrente (coverage)
        save_vect_(result_filename.c_str(), distances_rec, "distances_rec");
        save_vect_(result_filename.c_str(), times_rec, "times_rec");
        save_vect_(result_filename.c_str(), monitored_rec, "monitored_rec");
    }

    //ros::shutdown(); // "Cerrar" este nodo

    // He terminado, "cierro" todos los nodos
    //syso = std::system("rosnode kill --all");
    //syso = std::system("killall -9 rosmaster");
    //syso = std::system("killall -9 roscore");

    //syso = std::system("rosclean purge -h");

    return 0;
}
