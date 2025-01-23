
// Seguir caminos
#include <mada/path_follower.hpp>

PathFollower *pf;
//vector<geometry_msgs::PoseStamped> path2GoalPoses;
vector<geometry_msgs::Pose> path2GoalPoses;
bool recoveryMode = false;

bool realW = false;

void pathPosesCallback(const nav_msgs::Path pathMsg)
{
    //cout<<"ME HE SUSCRITO"<<endl;
	// transformar a poses
    path2GoalPoses.clear();
    for(int i=0; i<pathMsg.poses.size(); i++)
        path2GoalPoses.push_back(pathMsg.poses[i].pose);

	if(path2GoalPoses.size()){ // Se ha recibido el camino
		//ROS_INFO("CAMINO RECIBIDO");
        if(realW)
            pf->followPathReal(path2GoalPoses); // Seguir el camino  
        else
            pf->followPath(path2GoalPoses); // Seguir el camino
	}else{ // NO se ha recibido el camino
		ROS_INFO("CAMINO VACÍO");
        pf->followPath(path2GoalPoses); // Seguir el camino
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    ros::NodeHandle nh;

    // Obtener las variables con lo parámetros para seguir el camino
    string robotName;
    string pathTopic;
    string robotFrame;
    string poseTopic, cmdVelTopic, laserTopic;
    string footprintTopic, planTopic; // Para representación
    double sX, sY; // Tamaño del robot
    double maxLinearVel, maxAngularVel, robotRadius;
    double linearAcc, angularAcc;
    double goalDistDiff, goalAngleDiff, angularResolution, maxObstacleDistance;
    string planner_type; // {'sim', 'real'}
    
    
    // args=[pathTopic robotPoseTopic sX sY robotCmdVelTopic laserTopic maxLinearVel maxAngularVel goalDistDiff goalAngleDiff |...
    // planner_type angularResolution maxObstacleDistance | footprintTopic planTopic]
    if(argc == 21){
        robotName = argv[1];
        pathTopic = argv[2];
        robotFrame = argv[3];
        poseTopic = argv[4];
        sX = atof(argv[5]); sY = atof(argv[6]);
        cmdVelTopic = argv[7];
        laserTopic = argv[8];
        maxLinearVel = atof(argv[9]);
        maxAngularVel = atof(argv[10]);
        linearAcc = atof(argv[11]);
        angularAcc = atof(argv[12]);
        goalDistDiff = atof(argv[13]);
        goalAngleDiff = atof(argv[14]);
        planner_type = argv[15];
        angularResolution = atof(argv[16]);
        maxObstacleDistance = atof(argv[17]);
        footprintTopic = argv[18];
        planTopic = argv[19];
        recoveryMode = atoi(argv[20]);
    }else{
        cout<<argc<<endl;
        ROS_INFO("PARAMETROS INCORRECTOS");
        return 0;
    }

    cout<<"**********************************************************************"<<endl;
    cout<<"PARÁMETROS DEL NAVEGADOR"<<endl;
    cout<<"camino:   "<<pathTopic<<endl;
    cout<<"robot f:  "<<robotFrame<<endl;
    cout<<"robot p:  "<<poseTopic<<endl;
    cout<<"tamaño:   "<<sX<<" x "<<sY<<endl;
    cout<<"cmd_vel:  "<<cmdVelTopic<<endl;
    cout<<"laser:    "<<laserTopic<<endl;

    cout<<"max vel:  "<<maxLinearVel<<endl;
    cout<<"max ang:  "<<maxAngularVel<<endl;
    cout<<"lin acc:  "<<linearAcc<<endl;
    cout<<"ang acc:  "<<angularAcc<<endl;
    cout<<"dis2goal: "<<goalDistDiff<<endl;
    cout<<"ang2goal: "<<goalAngleDiff<<endl;

    cout<<"planner:  "<<planner_type<<endl;
    cout<<"angRes:   "<<angularResolution<<endl;
    cout<<"dis2obst: "<<maxObstacleDistance<<endl;

    cout<<"ftprint:  "<<footprintTopic<<endl;
    cout<<"planPath: "<<planTopic<<endl;

    cout<<"**********************************************************************"<<endl;

    if(planner_type.compare("sim") == 0)
        realW = false;
    else if(planner_type.compare("real") == 0)
        realW = true;

    // Inicializar PathFollower
    //PathFollower(string robotPoseTopic, double sX, double sY, string robotCmdVelTopic, string laserTopic, double maxLinearVel, double maxAngularVel, double posDiff, double angleDiff, string planner_type, double angularResolution, double maxObstacleDistance, string footprintTopic, string planTopic); // Constructor para seguimiento simple
    pf = new PathFollower(robotFrame, poseTopic, sX, sY, cmdVelTopic, laserTopic, maxLinearVel, maxAngularVel, linearAcc, angularAcc, goalDistDiff, goalAngleDiff, planner_type, angularResolution, maxObstacleDistance, footprintTopic, planTopic);

    // Suscribir al path follower
    ros::Subscriber pfSub = nh.subscribe<nav_msgs::Path>(pathTopic, 1, &pathPosesCallback);
    //pf.followPath(path2GoalPoses);


    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;
    cout<<"**********************************************************************"<<endl;

    ros::spin();

    return 0;

}