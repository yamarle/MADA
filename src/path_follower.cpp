#include <mada/path_follower.hpp>

PathFollower::PathFollower()
{
	//n_ = nh;
} // Constructor por defecto

PathFollower::~PathFollower(){}

PathFollower::PathFollower(string robotFrame, string robotPoseTopic, double sX, double sY, string robotCmdVelTopic, string laserTopic, double maxLinearVel, double maxAngularVel, double linearAcc, double angularAcc, double posDiff, double angleDiff, string planner_type, double angularResolution, double maxObstacleDistance, string footprintTopic, string planTopic)
{

    goalRefTime = sleepTime;

    initializeTopics(robotFrame, robotPoseTopic, robotCmdVelTopic);
    //robotRadius = std::max(sX,sY);
    robotRadius = sqrt(pow(sX,2)+pow(sY,2));
    // Distancia (máxima) a la que se considera que se va a encontrar el goal
    goalDist = max(maxLinearVel * goalRefTime + robotRadius, 2*robotRadius);
    initializeVariables(maxLinearVel, maxAngularVel, linearAcc, angularAcc, posDiff, angleDiff);
    initializePlotting(footprintTopic, sX, sY, planTopic);

} // Constructor para seguimiento simple

geometry_msgs::PolygonStamped PathFollower::computeFootprint(geometry_msgs::Pose pose, double sX, double sY)
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

nav_msgs::Path PathFollower::setPlan(geometry_msgs::Pose s, geometry_msgs::Pose g)
{
    nav_msgs::Path res;
    res.header.stamp = ros::Time::now();
    res.header.frame_id = "map";

    geometry_msgs::PoseStamped p;
    p.pose = s; res.poses.push_back(p);
    g.position.z = 0.0; // ESTO ES PARA CUANDO EL CAMINO NO ES 3D
    p.pose = g; res.poses.push_back(p);

    return res;
}

void PathFollower::initializePlotting(string footprintTopic, double sX, double sY, string planTopic)
{
    footprintPub = n_.advertise<geometry_msgs::PolygonStamped>(footprintTopic, 10);

    /*
    footprintPub = n_.advertise<visualization_msgs::Marker>(footprintTopic, 10);
    robotFootprint.header.frame_id = footprintTopic;
    robotFootprint.id = 0;
    robotFootprint.ns = "robot_footprint";
    robotFootprint.action = visualization_msgs::Marker::ADD;
    robotFootprint.type = visualization_msgs::Marker::LINE_LIST;
    robotFootprint.lifetime = ros::Duration(1.0); // El marcador durará 1 segundo en RViz
    robotFootprint.scale.x = 0.02;
    */

    this->sX = sX; this->sY = sY;
    planPub = n_.advertise<nav_msgs::Path>(planTopic, 10);
} // Inicializar 

geometry_msgs::Pose PathFollower::selectGoal(geometry_msgs::Pose pose, vector<geometry_msgs::Pose>& poses, double maxLenght, double &dist, double &dist2goal)
{
    // - pose: posición del robot
    // - poses: poses del camino
    // - maxLenght: distancia que se considera para alcanzar el goal
    // - dist: distancia mínima normalizada del camino al obstáculo más cercano
    // - dist2goal: distancia real desde punto del camino seleccionado al obstáculo

    geometry_msgs::Pose res;

    double d, _d;

    // Distancias del camino a los obstáculos
    double ddo = 0; // dinámicos
    double dso = 0; // estáticos

    // Distancias mínimas del camino a los obsáculos
    double ddom = INFINITY; // dinámicos
    double dsom = INFINITY; // estáticos

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
        // Distancias del robot a los obstáculos
        // d_{robot-obst} = sqrt( d_{robot_path}^2 + d_{obst_path}^2 )
        dso = sqrt((_d*_d)+(poses[i].orientation.x*poses[i].orientation.x)); // Estático
        ddo = sqrt((_d*_d)+(poses[i].orientation.y*poses[i].orientation.y)); // Dinámico

        // Almacenar las distancias mínimas
        if(ddom > ddo) ddom = ddo; // dinámico
        if(dsom > dso) dsom = dso; // estático

        if(dom > ddom) dom = ddom; // total
        if(dom > dsom) dom = dsom; // total

        // Almacenar la distancia mínima del camino al obstáculo dinámico más cercano
        if(dmin > poses[i].orientation.y){
            ind_min_dist = i;
            dmin = poses[i].orientation.y;
        }

    }

    // Aquí tengo las distancias del camino al obstáculo:
    // - estático más cercano
    // - dinámico más cercano


    // Almacenar la distancia del camino al obstáculo dinámico más cercano
    //if(dmin < INFINITY) min_obst_dist.push_back(dmin);
    min_obst_dist.push_back(dmin);
    if(ind_min_dist < 0){
        geometry_msgs::Pose p;
        min_obst_poses.push_back(p);
    }else{
        min_obst_poses.push_back(poses[ind_min_dist]);
    }

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
        //if(poses[1].position.z > poses[0].position.z && _d <= maxLenght){ // Si me estoy alejando de los obstáculos
        //if(_d <= poses[0].orientation.x && (_d <= poses[0].orientation.y - robotRadius) && _d <= maxLenght){ // Si me estoy alejando de los obstáculos
        if(_d <= poses[0].orientation.x && (_d <= poses[0].orientation.y - 2*robotRadius)){ // La distancia 
            //if(dist < poses[0].position.z) dist = poses[0].position.z;
            if(dist < poses[0].orientation.x) dist = poses[0].orientation.x;
            if(poses[0].orientation.y != INFINITY && dist < poses[0].orientation.y) dist = poses[0].orientation.y;
            poses.erase(poses.begin());
        }else{
            // Almacenar la distancia del robot al destino (goal sobre el camino)
            dist2dest.push_back(_d);

            dist /= dmax;
            return poses[0];
        }
    }

    dist /= dmax;
    return poses[0];
}

// *********************************************************************************************************************************
// 							FUNCIONES PARA SEGUIR CAMINO SIMPLE (SIN TENER EN CUENTA EL ENTORNO)
// *********************************************************************************************************************************


void PathFollower::initializeVariables(double maxLinearVel, double maxAngularVel, double linearAcc, double angularAcc, double goalDistDiff, double goalAngleDiff)
{
	this->maxLinearVel = maxLinearVel;
    this->maxAngularVel = maxAngularVel;
    this->linearAcc = linearAcc;
	this->angularAcc = angularAcc;
	this->goalDistDiff = goalDistDiff;
	this->goalAngleDiff = goalAngleDiff;

    cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
    _cmd_vel = cmd_vel;
}

void PathFollower::initializeTopics(string robotFrame, string robotPoseTopic, string robotCmdVelTopic)
{
	//poseSub = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robotPoseTopic, 1, &PathFollower::robotPoseCallback, this);
    if(robotPoseTopic.find("amcl")>=0 && robotPoseTopic.find("amcl")<robotPoseTopic.size()-1){
        // Subscriber pose con AMCL
        poseSub = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robotPoseTopic, 1, &PathFollower::amclPoseCallback, this);
    }else if((robotPoseTopic.find("odom")>=0 && robotPoseTopic.find("odom")<robotPoseTopic.size()-1) || (robotPoseTopic.find("ground_truth")>=0 && robotPoseTopic.find("ground_truth")<robotPoseTopic.size()-1)){
        // Subscriber pose con odometría
        poseSub = n_.subscribe<nav_msgs::Odometry>(robotPoseTopic, 1, &PathFollower::odomPoseCallback, this);
    }

	cmdVelPub = n_.advertise<geometry_msgs::Twist>(robotCmdVelTopic, 1);

    totalPathPub = n_.advertise<nav_msgs::Path>("totalPath", 10);

    baseFrame = robotFrame;
}

void PathFollower::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robotPoseMsg)
{
	robotPose = robotPoseMsg->pose.pose;
	poseCh = true;

    geometry_msgs::PoseStamped _p; _p.pose = robotPoseMsg->pose.pose;
    totalPath.poses.push_back(_p);
    totalPath.header.frame_id = "map";
    totalPath.header.stamp = ros::Time::now();
    totalPathPub.publish(totalPath); // Publico
}

void PathFollower::odomPoseCallback(const nav_msgs::Odometry::ConstPtr& robotPoseMsg)
{
    robotPose = robotPoseMsg->pose.pose;
    poseCh = true;

    geometry_msgs::PoseStamped _p; _p.pose = robotPoseMsg->pose.pose;
    totalPath.poses.push_back(_p);
    totalPath.header.frame_id = "map";
    totalPath.header.stamp = ros::Time::now();
    totalPathPub.publish(totalPath); // Publico
}

PathFollower::PathFollower(string robotFrame, string robotPoseTopic, string robotCmdVelTopic) 
{
	//n_ = nh;
	initializeTopics(robotFrame, robotPoseTopic, robotCmdVelTopic);
}// Constructor para seguimiento simple

PathFollower::PathFollower(string robotFrame, string robotPoseTopic, string robotCmdVelTopic, double maxLinearVel, double maxAngularVel, double posDiff, double angleDiff)
{
	//n_ = nh;
	robotPose.position.x = -100000;
	initializeTopics(robotFrame, robotPoseTopic, robotCmdVelTopic);
	initializeVariables(maxLinearVel, maxAngularVel, linearAcc, angularAcc, posDiff, angleDiff);
}// Constructor para seguimiento simple

void PathFollower::computeVelocities(const geometry_msgs::Pose goalPose, double dObstMaxNorm, double& linearVel, double& angularVel)
{
    // Cálculo del ángulo de desviación
    double angleDiff = atan2(goalPose.position.y - robotPose.position.y, goalPose.position.x - robotPose.position.x) - tf::getYaw(robotPose.orientation);

    // Ajuste del ángulo de desviación en el rango [-pi, pi]
    if (angleDiff > M_PI)
        angleDiff -= 2.0 * M_PI;
    else if (angleDiff < -M_PI)
        angleDiff += 2.0 * M_PI;

    // Cálculo de las velocidades lineal y angular (ajustado con las aceleraciones lineal y angular máximas)
    angularVel = maxAngularVel * angleDiff;

    if(angleDiff){
        //linearVel = maxLinearVel * goalPose.position.z * abs(maxAngularVel - abs(angularVel));
        if(abs(angleDiff) < M_PI/2)
            linearVel = maxLinearVel * dObstMaxNorm * (1-(abs(angularVel)/maxAngularVel));
        else
            linearVel = 0;
    }else{
        linearVel = maxLinearVel; // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }   
} // no se tiene en cuenta nada

void PathFollower::computeVelocitiesAcc(const geometry_msgs::Pose goalPose, double dObstMaxNorm, double& linearVel, double& angularVel)
{
    // Cálculo del ángulo de desviación
    double angleDiff = atan2(goalPose.position.y - robotPose.position.y, goalPose.position.x - robotPose.position.x) - tf::getYaw(robotPose.orientation);

    // Ajuste del ángulo de desviación en el rango [-pi, pi]
    if (angleDiff > M_PI)
        angleDiff -= 2.0 * M_PI;
    else if (angleDiff < -M_PI)
        angleDiff += 2.0 * M_PI;

    // Cálculo de las velocidades lineal y angular (ajustado con las aceleraciones lineal y angular máximas)
    angularVel = maxAngularVel * angleDiff;
    if(angularVel < 0)  // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        angularVel = max(angularVel, _cmd_vel.angular.z - abs(angularAcc));
    else
        angularVel = min(angularVel, _cmd_vel.angular.z + abs(angularAcc));

    if(angleDiff){
        //linearVel = maxLinearVel * goalPose.position.z * abs(maxAngularVel - abs(angularVel));
        if(abs(angleDiff) < M_PI/2)
            linearVel = maxLinearVel * dObstMaxNorm * (1-(abs(angularVel)/maxAngularVel));
            //linearVel = maxLinearVel * dObstMaxNorm * abs(maxAngularVel - abs(angularVel));
        else
            linearVel = 0;
        //linearVel = maxLinearVel * abs(maxAngularVel - abs(angularVel));
        linearVel = min(linearVel, _cmd_vel.linear.x + linearAcc); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }else{
        //linearVel = maxLinearVel;
        linearVel = min(maxLinearVel, _cmd_vel.linear.x + linearAcc); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }    

} // Se tiene en cuenta el ángulo al goal y las aceleraciones del robot

// *********************************************************************************************************************************

bool PathFollower::followPath(vector<geometry_msgs::Pose> path)
{

    //ROS_INFO("SIGUIENDO CAMINO (poses) ... ");
	if(path.size() == 0) return 0;
	while(!poseCh){ // Esperar a leer la posición del robot
        ROS_INFO("ESPERANDO POSICION DEL ROBOT ... ");
        ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	
	pathPoses = path;
	
	while(ros::ok()){

        //tini = ros::Time::now();

        // Publicar el footprint del robot
        footprintPub.publish(computeFootprint(robotPose, sX, sY)); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::footprintOr(robotPose, sX, sY), "map")); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::footprintOr(robotPose, sX/2, (int)18), "map")); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        if(pathPoses.size() > 1){

            double d, d2g;

            // Almacenar los tiempos
            reg_time.push_back(ros::Time::now().toSec());
            // Almacenar la pose del robot
            robotPoses.push_back(robotPose);
            // Almacenar el goal del robot (final del camino)
            pathDestPoses.push_back(pathPoses[pathPoses.size()-1]);

            // Obtener el siguiente punto del camino a alcanzar y "alcualizar" el camino
            goalPose = selectGoal(robotPose, pathPoses, goalDist, d, d2g);

            // Almacenar los destinos del robot (goal sobre el camino)
            goalPoses.push_back(goalPose);

    		// Calcular (v,w)
            if(pathPoses.size()){
                if(d2g > 2*robotRadius) computeVelocitiesAcc(goalPose, 1, cmd_vel.linear.x, cmd_vel.angular.z);
                else computeVelocitiesAcc(goalPose, d, cmd_vel.linear.x, cmd_vel.angular.z);
            }else cmd_vel.linear.x = cmd_vel.angular.z = 0.0;

            _cmd_vel = cmd_vel; // Almacenar las velocidades anteriores

            // Publicar la trayectoría que sigue el robot
            goalPose.orientation.x = goalPose.orientation.y = 0.0; // quitar las distancias a los obstáculos (estático y dinámico)
            //planPub.publish(setPlan(robotPose, goalPose));
            planPub.publish(mada_util::setPlan(robotPose, goalPose, "map"));

        }else{
            cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
        }

        // Publicar (v,w)
        cmdVelPub.publish(cmd_vel);

        ros::spinOnce();
        ros::Duration(sleepTime).sleep(); // tiempo en segundos
	}

    //ROS_INFO("publico (%f, %f)",cmd_vel.linear.x, cmd_vel.angular.z);

	return true;
} // Sigue el camino

bool PathFollower::followPathReal(vector<geometry_msgs::Pose> path)
{

    //ROS_INFO("SIGUIENDO CAMINO (poses) ... ");
    if(path.size() == 0) return 0;
    while(!poseCh){ // Esperar a leer la posición del robot
        ROS_INFO("ESPERANDO POSICION DEL ROBOT ... ");
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    
    pathPoses = path;
    
    while(ros::ok()){

        //tini = ros::Time::now();

        // Publicar el footprint del robot
        //footprintPub.publish(computeFootprint(robotPose, sX, sY)); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::footprintOr(robotPose, sX, sY), "map")); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::footprintOr(robotPose, sX/2, (int)5), "map")); // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::circlePointsTr(robotRadius/2, 18), baseFrame));
        //footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::footprintOr(robotPose, sX, sY), "map"));
        footprintPub.publish(mada_util::pointsToPolygonStamped(mada_util::footprintNoOr(robotPose, robotRadius/2, 18), "map"));

        if(pathPoses.size() > 1){

            double d, d2g;

            // Almacenar los tiempos
            reg_time.push_back(ros::Time::now().toSec());
            // Almacenar la pose del robot
            robotPoses.push_back(robotPose);
            // Almacenar el goal del robot (final del camino)
            pathDestPoses.push_back(pathPoses[pathPoses.size()-1]);

            // Obtener el siguiente punto del camino a alcanzar y "alcualizar" el camino
            goalPose = selectGoal(robotPose, pathPoses, goalDist, d, d2g);

            // Almacenar los destinos del robot (goal sobre el camino)
            goalPoses.push_back(goalPose);

            // Calcular (v,w)
            if(pathPoses.size()){
                if(d2g > 2*robotRadius) computeVelocities(goalPose, 1, cmd_vel.linear.x, cmd_vel.angular.z);
                else computeVelocities(goalPose, d, cmd_vel.linear.x, cmd_vel.angular.z);
            }else cmd_vel.linear.x = cmd_vel.angular.z = 0.0;

            _cmd_vel = cmd_vel; // Almacenar las velocidades anteriores

            // Publicar la trayectoría que sigue el robot
            goalPose.orientation.x = goalPose.orientation.y = 0.0; // quitar las distancias a los obstáculos (estático y dinámico)
            planPub.publish(setPlan(robotPose, goalPose));

        }else{
            cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
        }

        // Publicar (v,w)
        if(cmd_vel.linear.x != 0.0 || cmd_vel.angular.z != 0.0) // <----- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            cmdVelPub.publish(cmd_vel);

        //tend = ros::Time::now();

        //cout<<"TIEMPO DE PF ---> "<<tend<<" - "<<tini<<" = "<<tend-tini<<endl;

        ros::spinOnce();
        ros::Duration(sleepTime).sleep(); // tiempo en segundos
    }

    //ROS_INFO("publico (%f, %f)",cmd_vel.linear.x, cmd_vel.angular.z);

    return true;
} // Sigue el camino
