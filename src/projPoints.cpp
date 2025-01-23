
#include <mada/projPoints.h>

Poss<int> pointProj::points2Poss(vector<geometry_msgs::Point> points)
{
    Poss<int> res;
    for(int i=0; i<points.size(); i++)
        res.push(points[i].y, points[i].x);
    return res;
}

Poss<int> pointProj::points2Poss(vector<geometry_msgs::Point> points, const nav_msgs::OccupancyGrid& map)
{
    Poss<int> res;
    for(int i=0; i<points.size(); i++){
        res.push((points[i].y - map.info.origin.position.x)/map.info.resolution, (points[i].x - map.info.origin.position.y)/map.info.resolution);
    }
    return res;
}

vector<Poss<int>> pointProj::points2Poss(vector<vector<geometry_msgs::Point>> points, const nav_msgs::OccupancyGrid& map)
{
    vector<Poss<int>>res;
    for(int i=0; i<points.size(); i++){
        res.push_back(points2Poss(points[i], map));
    }
    return res;
}

Poss<int> pointProj::poses2Poss(vector<geometry_msgs::Pose> points)
{
    Poss<int> res;
    for(int i=0; i<points.size(); i++)
        res.push(points[i].position.y, points[i].position.x);
    return res;
}

vector<geometry_msgs::Point> pointProj::poss2Points(vector<Poss<int>> poss, const nav_msgs::OccupancyGrid& map)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p;
    for(int i=0; i<poss.size(); i++){
        for(int j=0; j<poss[i].x.size(); j++){
            p.y = poss[i].x[j] * map.info.resolution + map.info.origin.position.x;
            p.x = poss[i].y[j] * map.info.resolution + map.info.origin.position.y;
            res.push_back(p);
        }
    }
    return res;
}

vector<geometry_msgs::Point> pointProj::poss2Points(Poss<int> poss, const nav_msgs::OccupancyGrid& map, float time)
{
    vector<geometry_msgs::Point> res(poss.x.size());
    geometry_msgs::Point p;
    for(int i=0; i<poss.x.size(); i++){
        p.y = poss.x[i] * map.info.resolution + map.info.origin.position.x;
        p.x = poss.y[i] * map.info.resolution + map.info.origin.position.y;
        p.z = time;
        //res.push_back(p);
        res[i] = p;
    }
    return res;
}

vector<vector<geometry_msgs::Point>> pointProj::poss2Points(vector<Poss<int>> poss, const nav_msgs::OccupancyGrid& map, float time)
{
    vector<vector<geometry_msgs::Point>> res(poss.size());
    geometry_msgs::Point p;
    for(int i=0; i<poss.size(); i++){
        for(int j=0; j<poss[i].x.size(); j++){
            p.y = poss[i].x[j] * map.info.resolution + map.info.origin.position.x;
            p.x = poss[i].y[j] * map.info.resolution + map.info.origin.position.y;
            p.z = time;
            res[i].push_back(p);
        }
    }
    return res;
}

vector<vector<geometry_msgs::Point>> pointProj::appendTrajectories(vector<vector<geometry_msgs::Point>> points, vector<vector<geometry_msgs::Point>> trajectories, float r)
{
    // - points: puntos de los obstáculos ya agrupados
    // - trajectories: las trayectorías registradas

    int cl;
    while(points.size()){
        while(points[0].size()){
            // Encontrar la trayectoría con la cuál hay que agrupar el punto
            cl = -1;
            for(int i=0; i<trajectories.size(); i++){
                for(int j=0; j<trajectories[i].size(); j++){
                    if(hypot(points[0][0].x - trajectories[i][j].x, points[0][0].y - trajectories[i][j].y) <= r){
                        cl = i;
                        break;
                    }
                }
                if(cl >= 0) break;
            }
            if(cl<0){
                trajectories.resize(trajectories.size() + 1);
                trajectories[trajectories.size()-1].push_back(points[0][0]);
            }else{
                trajectories[cl].push_back(points[0][0]);
            }
            points[0].erase(points[0].begin());
        }
        points.erase(points.begin());
    }

    return trajectories;
}

vector<int> pointProj::appendTrajectoriesInd(vector<geometry_msgs::Point> centroids, vector<vector<geometry_msgs::Point>> points, vector<vector<geometry_msgs::Point>> &centroids_trajectories, vector<vector<geometry_msgs::Point>> &trajectories, float r)
{
    // - centroids: centroides de los obstáculos visibles
    // - points: puntos de los obstáculos ya agrupados
    // - centroids_trajectories: trayectorías registradas de los centroides de los obstáculos
    // - trajectories: las trayectorías registradas
    // - r: radio con el que se considera que un punto pertenece a una trayectoría de un obstáuclo determinado

    vector<int> res(centroids.size(), -1);
    int n = 0;

    int cl;
    while(points.size()){
        while(points[0].size()){
            // Encontrar la trayectoría con la cuál hay que agrupar el punto
            cl = -1;
            for(int i=0; i<trajectories.size(); i++){
                for(int j=0; j<trajectories[i].size(); j++){
                    if(hypot(points[0][0].x - trajectories[i][j].x, points[0][0].y - trajectories[i][j].y) <= r){
                        cl = i;
                        break;
                    }
                }
                if(cl >= 0) break;
            }
            if(cl<0){
                trajectories.resize(trajectories.size() + 1); // Trayectoria nuevo
                trajectories[trajectories.size()-1].push_back(points[0][0]); // Inserto punto

                centroids_trajectories.resize(centroids_trajectories.size() + 1); // Centroide de trayectoría nueva
                centroids_trajectories[centroids_trajectories.size()-1].push_back(centroids[0]); // Inserto centroide
            }else{
                trajectories[cl].push_back(points[0][0]); // Inserto punto en la trayectoría
            }
            points[0].erase(points[0].begin());
        }
        points.erase(points.begin());

        if(cl>=0){
            centroids_trajectories[cl].push_back(centroids[0]);
            res[n] = cl;
        }
        centroids.erase(centroids.begin());
        n++;
    }

    return res;
}

void pointProj::clearTrajectories(vector<vector<geometry_msgs::Point>> &centroids_trajectories, vector<vector<geometry_msgs::Point>> &points_trajectories, vector<geometry_msgs::Point> laser_pos)
{
    bool f;
    int i = 0;
    while(i<points_trajectories.size()){
        f = false;
        for(int j=0; j<points_trajectories[i].size(); j++){
            for(int k=0; k<laser_pos.size(); k++){
                if(points_trajectories[i][j].x == laser_pos[k].x && points_trajectories[i][j].y == laser_pos[k].y){
                    f = true;
                    break;
                }
            }
            if(f) break;
        }
        if(!f){
            centroids_trajectories[i].clear();
            points_trajectories[i].clear();
            centroids_trajectories.erase(centroids_trajectories.begin() + i);
            points_trajectories.erase(points_trajectories.begin() + i);
        }else{
            i++;
        }
    }
}

vector<vector<float>> pointProj::getTimes(vector<vector<geometry_msgs::Point>> trajectories)
{
    vector<vector<float>> res(trajectories.size());
    for(int i=0; i<trajectories.size(); i++){
        if(trajectories[i].size() == 0) continue;
        res[i].push_back(trajectories[i][0].z);
        for(int j=1; j<trajectories[i].size(); j++){
            if(trajectories[i][j-1].z != trajectories[i][j].z){
                res[i].push_back(trajectories[i][j].z);
            }
        }
    }
    return res;
}

vector<vector<vector<geometry_msgs::Point>>> pointProj::separateTrajectories(vector<vector<geometry_msgs::Point>> trajectories)
{
    vector<vector<vector<geometry_msgs::Point>>> res(trajectories.size());
    for(int i=0; i<trajectories.size(); i++){
        res[i].resize(res[i].size()+1);
        if(!trajectories[i].size()) continue;
        res[i][res[i].size()-1].push_back(trajectories[i][0]);
        if(trajectories[i].size() < 2) continue;
        for(int j=1; j<trajectories[i].size(); j++){
            if(trajectories[i][j].z != res[i][res[i].size()-1][res[i][res[i].size()-1].size()-1].z){
                res[i].resize(res[i].size()+1);
            }
            res[i][res[i].size()-1].push_back(trajectories[i][j]);
        }
    }
    return res;
}

geometry_msgs::Point pointProj::centroid(vector<geometry_msgs::Point> points)
{
    geometry_msgs::Point res;
    res.x = res.y = 0.0;
    if(!points.size()) return res;
    for(int i=0; i<points.size(); i++){
        res.x += points[i].x; res.y += points[i].y;
    }
    res.x /= points.size(); res.y /= points.size();
    return res;
}

geometry_msgs::Point pointProj::equidistantPoint(vector<geometry_msgs::Point> points)
{
    geometry_msgs::Point res;
    //res.x = res.y = 0.0;
    res = centroid(points);
    if(!points.size()) return res;
    double dist;
    for (int iter = 0; iter < 100; ++iter) { // Número de iteraciones predeterminado
        geometry_msgs::Point gradient;
        gradient.x = gradient.y = 0.0;

        // Calcular el gradiente
        for (int i=0; i<points.size(); i++){
            dist = hypot(res.x - points[i].x, res.y - points[i].y);
            gradient.x += (res.x - points[i].x) / dist;
            gradient.y += (res.y - points[i].y) / dist;
        }

        // Actualizar el punto usando el descenso de gradiente
        res.x -= 0.01 * gradient.x; // Tasa de aprendizaje predeterminada
        res.y -= 0.01 * gradient.y;
    }

    return res;
}

vector<geometry_msgs::Point> pointProj::clustersCentroids(vector<vector<geometry_msgs::Point>> points)
{
    vector<geometry_msgs::Point> res(points.size());

    for(int i=0; i<points.size(); i++){
        res[i] = centroid(points[i]);
    }

    return res;
}

float pointProj::getRadius(geometry_msgs::Point point, vector<geometry_msgs::Point> points)
{
    float res = 0, d;
    for(int i=0; i<points.size(); i++){
        d = hypot(point.x - points[i].x, point.y - points[i].y);
        if(res < d) res = d;
    }
    return res;
}

vector<float> pointProj::computeDimensions(vector<geometry_msgs::Point> centroids, vector<vector<geometry_msgs::Point>> points)
{
    vector<float> res(centroids.size(), 0);
    for(int i=0; i<centroids.size(); i++)
        res[i] = getRadius(centroids[i], points[i]);
    return res;
}

vector<vector<float>> pointProj::computeDimensions(vector<vector<geometry_msgs::Point>> centroids, vector<vector<vector<geometry_msgs::Point>>> points)
{
    vector<vector<float>> res(centroids.size());
    for(int i=0; i<centroids.size(); i++){
        res[i] = computeDimensions(centroids[i], points[i]);
    }
    return res;
}

vector<float> pointProj::computeLinearVelocitiesT(vector<geometry_msgs::Point> points, float T)
{
    // - points: almacena los tiempos en los que se han registrado los puntos en "z"
    vector<float> res;
    if(points.size() == 0) return res;
    geometry_msgs::Point p = points[0];
    float d = 0, t = 0;
    for(int i=1; i<points.size(); i++){
        t = (points[i].z - p.z);
        d = hypot(points[i].x - p.x, points[i].y - p.y);
        if(t >= T){
            p = points[i];
            res.push_back(d/t);
            t = d = 0;
        }
    }
    return res;
} // Calcular velocidades a partir de los puntos "points"

geometry_msgs::Twist pointProj::computeCmdVel(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, double time_delta)
{
    geometry_msgs::Twist res;

    // Calcular la diferencia en posiciones
    double delta_x = pose2.position.x - pose1.position.x;
    double delta_y = pose2.position.y - pose1.position.y;

    // Calcular la diferencia en orientaciones
    tf::Quaternion q1, q2;
    tf::quaternionMsgToTF(pose1.orientation, q1);
    tf::quaternionMsgToTF(pose2.orientation, q2);
    tf::Transform tf_pose1(q1, tf::Vector3(pose1.position.x, pose1.position.y, pose1.position.z));
    tf::Transform tf_pose2(q2, tf::Vector3(pose2.position.x, pose2.position.y, pose2.position.z));
    tf::Transform tf_delta = tf_pose1.inverseTimes(tf_pose2);

    // Extraer la velocidad lineal y angular a partir de la diferencia en posiciones y orientaciones
    res.linear.x = std::sqrt(delta_x * delta_x + delta_y * delta_y) / time_delta;
    res.angular.z = tf::getYaw(tf_delta.getRotation()) / time_delta;

    return res;
}

vector<geometry_msgs::Twist> pointProj::computeVelocitiesT(vector<geometry_msgs::Pose> poses, float T)
{
    vector<geometry_msgs::Twist> res;
    if(poses.size() < 2) return res;
    geometry_msgs::Pose p = poses[0];

    float t;
    for(int i=1; i<poses.size(); i++){
        t = (poses[i].position.z - p.position.z);
        if(t >= T){
            res.push_back(computeCmdVel(poses[i], p, t));
            p = poses[i];
        }
    }
    return res;
}

vector<geometry_msgs::Twist> pointProj::computeVelocitiesDT(vector<geometry_msgs::Pose> poses, float time, float dist)
{
    vector<geometry_msgs::Twist> res;
    if(poses.size() < 2) return res;
    geometry_msgs::Pose p = poses[0];

    float t;
    float d;
    for(int i=1; i<poses.size(); i++){
        t = (poses[i].position.z - p.position.z);
        d = hypot(poses[i].position.x - p.position.x, poses[i].position.y - p.position.y);
        if(t >= time && d >= dist){
            res.push_back(computeCmdVel(poses[i], p, t));
            p = poses[i];
        }
    }
    return res;
}

vector<geometry_msgs::Twist> pointProj::estimateVelocities(vector<geometry_msgs::Pose> poses, float time, float dist)
{
    vector<geometry_msgs::Twist> res;
    if(poses.size() < 2) return res;
    geometry_msgs::Pose p = poses[0];

    float t;
    float d;
    geometry_msgs::Twist v;
    for(int i=1; i<poses.size(); i++){
        t = (poses[i].position.z - p.position.z);
        d = hypot(poses[i].position.x - p.position.x, poses[i].position.y - p.position.y);
        if(t >= time){ // ha pasado el periodo de espera
            v = computeCmdVel(poses[i], p, t);
            if(d >= dist){ // El obstáculo se ha movido lo suficiente
                v.angular.z = 0.0;
                res.push_back(v);
                p = poses[i];
            }else{
                v.linear.x = 0.0;
                res.push_back(v);
            }
        }
    }
    return res;
}

vector<geometry_msgs::Twist> pointProj::smoothVelocities(vector<geometry_msgs::Twist> velocities, int ws)
{
    vector<geometry_msgs::Twist> res;
    int n = velocities.size();

    if (ws <= 0 || ws > n) return res;

    geometry_msgs::Twist v;
    v.linear.x = v.angular.z = 0.0;

    for (int i = 0; i <= n - ws; i++) {
        float suma = 0.0f;
        for (int j = i; j < i + ws; j++) {
            v.linear.x += velocities[j].linear.x;
            v.angular.z += velocities[j].angular.z;
        }
        v.linear.x /= ws;
        v.angular.z /= ws;
        res.push_back(v);
    }

    return res;
}

vector<geometry_msgs::Twist> pointProj::computeVelocitiesTimes(vector<geometry_msgs::Pose> poses, vector<float> regTimes, float T)
{
    vector<geometry_msgs::Twist> res;
    if(poses.size() == 0) return res;
    geometry_msgs::Pose p = poses[0];
    float t, _t;
    _t = regTimes[0];
    for(int i=1; i<poses.size(); i++){
        t = (regTimes[i] - _t);
        if(t >= T){
            res.push_back(computeCmdVel(poses[i], p, t));
            p = poses[i];
            _t = regTimes[i];
        }
    }
    return res;
}

geometry_msgs::Twist pointProj::computeMeanVelocity(vector<geometry_msgs::Twist> cmd_vels, int it)
{
    // - cmd_vels: todas las velocidades registradas
    // - it: cantidad de velocidades registradas que se consideran para la media
    geometry_msgs::Twist res;
    res.linear.x = res.angular.z = 0.0;
    int num = 0;
    for(int i=cmd_vels.size()-1; i>=0 && num<it; i--){
        res.linear.x += cmd_vels[i].linear.x;
        res.angular.z += cmd_vels[i].angular.z;
        num++;
    }
    if(num){
        res.linear.x /= num;
        res.angular.z /= num;
    }
    return res;
}

float pointProj::computeVelocity(nav_msgs::Path path, vector<float> timeReg, float T)
{
    if(path.poses.size() == 0) return 0;
    //float res = 0;
    float t = 0;
    float d = 0;
    for(int i=path.poses.size()-1; i>0; i--){
        if(t > T){
            return (d/t);
        }else{
            t += (timeReg[i] - timeReg[i-1]);
            d += hypot(path.poses[i-1].pose.position.x - path.poses[i].pose.position.x, path.poses[i-1].pose.position.y - path.poses[i].pose.position.y);
        }
    }
    if(t == 0) return 0;
    return (d/t);
}

float pointProj::closestDist(float x, float y, vector<geometry_msgs::Point> points)
{
    float res = INFINITY, d;
    for(int i=0; i<points.size(); i++){
        d = hypot(x - points[i].x, y - points[i].y);
        if(res > d) res = d;
    }
    return res;
}

float pointProj::time2ClosestPoint(float x, float y, vector<geometry_msgs::Point> points, float velocity)
{
    float res = INFINITY, d;
    for(int i=0; i<points.size(); i++){
        d = hypot(x - points[i].x, y - points[i].y);
        if(res > d) res = d;
    }
    //return res;
    return (res/velocity);
} // Tiempo de (x,y) al punto más alejado

float pointProj::time2FarthestPoint(float x, float y, vector<geometry_msgs::Point> points, float velocity)
{
    float res = 0.0, d;
    for(int i=0; i<points.size(); i++){
        d = hypot(x - points[i].x, y - points[i].y);
        if(res < d) res = d;
    }
    //return res;
    return (res/velocity);
} // Tiempo de (x,y) al punto más alejado

geometry_msgs::Quaternion pointProj::calculateOrientation(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2){
    float dx = point2.x - point1.x;
    float dy = point2.y - point1.y;
    float yaw = std::atan2(dy, dx);
    
    // Crear un cuaternión a partir del ángulo de yaw
    geometry_msgs::Quaternion orientation;
    orientation.w = std::cos(yaw / 2.0);
    orientation.z = std::sin(yaw / 2.0);
    
    return orientation;
} // Función para calcular la orientación a partir de dos puntos

void pointProj::showPoints(vector<geometry_msgs::Point> points)
{
    for(int i=0; i<points.size(); i++)
        cout<<points[i].x<<", "<<points[i].y<<", "<<points[i].z<<endl;
}

vector<geometry_msgs::Point> pointProj::poses2Points(vector<geometry_msgs::Pose> poses)
{
    vector<geometry_msgs::Point> res(poses.size());
    for(int i=0; i<poses.size(); i++){
        res[i].x = poses[i].position.x;
        res[i].y = poses[i].position.y;
        res[i].z = 0;
    }
    return res;
}

vector<geometry_msgs::Point> pointProj::poses2Points(vector<vector<geometry_msgs::Pose>> poses)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p;
    for(int i=0; i<poses.size(); i++){
        for(int j=0; j<poses[i].size(); j++){
            p.x = poses[i][j].position.x;
            p.y = poses[i][j].position.y;
            p.z = 0.0;
            res.push_back(p);
        }
    }
    return res;
}

vector<geometry_msgs::Point> pointProj::points2Plot(vector<geometry_msgs::Point> points)
{
    for(int i=0; i<points.size(); i++){
        points[i].z = 0.0;
    }
    return points;
}

vector<geometry_msgs::Point> pointProj::points2Plot(vector<vector<geometry_msgs::Point>> points)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p;
    for(int i=0; i<points.size(); i++){
        for(int j=0; j<points[i].size(); j++){
            p.x = points[i][j].x;
            p.y = points[i][j].y;
            p.z = 0.0;
            res.push_back(p);
        }
    }
    return res;
}

geometry_msgs::Pose pointProj::point2Pose(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    geometry_msgs::Pose pose;
    pose.position = point1;
    pose.orientation = calculateOrientation(point1, point2);
    return pose;
}

vector<geometry_msgs::Pose> pointProj::points2Poses(vector<geometry_msgs::Point> points)
{
    vector<geometry_msgs::Pose> poses(points.size());
    if(points.size() < 2) return poses;

    for(int i=0; i<points.size()-1; i++){
        poses[i].position = points[i];
        poses[i].orientation = calculateOrientation(points[i], points[i+1]);
    }
    // La última pose tiene la orientación de la penúltima
    poses[poses.size()-1].position = points[points.size()-1];
    poses[poses.size()-1].orientation = poses[poses.size()-2].orientation;
    return poses;
}

vector<geometry_msgs::Pose> pointProj::points2Poses(vector<geometry_msgs::Point> points, float time_delta)
{
    vector<geometry_msgs::Pose> poses;
    if(points.size() < 2) return poses;

    geometry_msgs::Pose pose;
    pose.position = points[points.size()-1];
    poses.push_back(pose);
    float t = points[points.size()-1].z;
    for(int i=points.size()-2; i>=0; i--){
        //cout<<t<<" - "<<points[i].z<<" = "<<t - points[i].z<<" > "<<time_delta<<endl;
        if((t - points[i].z) >= time_delta){
            t = points[i].z;
            //poses[poses.size()-1].orientation = calculateOrientation(points[i], poses[poses.size()-1].position);
            poses[0].orientation = calculateOrientation(points[i], poses[0].position);

            pose.position = points[i];
            //poses.push_back(pose);
            poses.insert(poses.begin(), pose);
        }
    }

    //poses[poses.size()-1].orientation = calculateOrientation(points[0], poses[poses.size()-1].position);
    poses[0].orientation = calculateOrientation(points[0], poses[0].position);

    return poses;
}

vector<geometry_msgs::Pose> pointProj::points2Poses(vector<geometry_msgs::Point> points, float time_delta, float time)
{
    vector<geometry_msgs::Pose> poses;
    if(points.size() < 2) return poses;

    geometry_msgs::Pose pose; // Variable para ir guardando
    pose.position = points[points.size()-1];
    poses.push_back(pose);
    float t = points[points.size()-1].z;
    float tt = 0;
    for(int i=points.size()-2; i>=0 && tt<=time; i--){
        //cout<<t<<" - "<<points[i].z<<" = "<<t - points[i].z<<" > "<<time_delta<<" ||| "<<tt<<" <= "<<time<<endl;
        if((t - points[i].z) >= time_delta){
            tt += (t - points[i].z);
            t = points[i].z;
            //poses[poses.size()-1].orientation = calculateOrientation(points[i], poses[poses.size()-1].position);
            poses[0].orientation = calculateOrientation(points[i], poses[0].position);

            pose.position = points[i];
            //poses.push_back(pose);
            poses.insert(poses.begin(), pose);
        }
    }

    //poses[poses.size()-1].orientation = calculateOrientation(points[0], poses[poses.size()-1].position);
    poses[0].orientation = calculateOrientation(points[0], poses[0].position);

    return poses;
}

vector<geometry_msgs::Pose> pointProj::trajectories2Poses(vector<vector<geometry_msgs::Point>> trajectories_points)
{
    vector<geometry_msgs::Pose> poses(trajectories_points.size());
    for(int i=0; i<trajectories_points.size(); i++){
        if(trajectories_points[i].size() > 1){
            poses[i].position = trajectories_points[i][trajectories_points[i].size()-1];
            poses[i].orientation = calculateOrientation(trajectories_points[i][trajectories_points[i].size()-2], trajectories_points[i][trajectories_points[i].size()-1]);
        }
    }
    return poses;
}

geometry_msgs::Pose pointProj::calculateNextPosition(const geometry_msgs::Pose& currentPose, float velocity, float time_delta)
{
    geometry_msgs::Pose nextPose;
    nextPose.position.x = currentPose.position.x + velocity * time_delta * std::cos(tf::getYaw(currentPose.orientation));
    nextPose.position.y = currentPose.position.y + velocity * time_delta * std::sin(tf::getYaw(currentPose.orientation));
    nextPose.orientation = currentPose.orientation; // Mantener la orientación
    return nextPose;
} // Función para calcular la posición siguiente en movimiento rectilíneo uniforme

geometry_msgs::Pose pointProj::calculateNextPosition(const geometry_msgs::Pose& currentPose, geometry_msgs::Twist cmdVel, float time_delta)
{
    geometry_msgs::Pose nextPose;
    nextPose.position.x = currentPose.position.x + cmdVel.linear.x * time_delta * std::cos(tf::getYaw(currentPose.orientation));
    nextPose.position.y = currentPose.position.y + cmdVel.linear.x * time_delta * std::sin(tf::getYaw(currentPose.orientation));
    double theta = tf::getYaw(currentPose.orientation);
    theta += (cmdVel.angular.z * time_delta);
    nextPose.orientation = tf::createQuaternionMsgFromYaw(theta);
    return nextPose;
} // Función para calcular la posición siguiente en movimiento

geometry_msgs::Pose pointProj::calculateNextPose(const geometry_msgs::Pose& currentPose, geometry_msgs::Twist cmdVel, float time_delta, float max_dist)
{
    geometry_msgs::Pose nextPose;
    nextPose.position.x = currentPose.position.x + cmdVel.linear.x * time_delta * std::cos(tf::getYaw(currentPose.orientation));
    nextPose.position.y = currentPose.position.y + cmdVel.linear.x * time_delta * std::sin(tf::getYaw(currentPose.orientation));
    while(hypot(nextPose.position.x - currentPose.position.x, nextPose.position.y - currentPose.position.y) > max_dist && time_delta > 0.01){
        time_delta -= (time_delta/10);
        nextPose.position.x = currentPose.position.x + cmdVel.linear.x * time_delta * std::cos(tf::getYaw(currentPose.orientation));
        nextPose.position.y = currentPose.position.y + cmdVel.linear.x * time_delta * std::sin(tf::getYaw(currentPose.orientation));
    }
    double theta = tf::getYaw(currentPose.orientation);
    theta += (cmdVel.angular.z * time_delta);
    nextPose.orientation = tf::createQuaternionMsgFromYaw(theta);
    return nextPose;
} // Función para calcular la posición siguiente en movimiento, capado por la distancia máxima (radio)

vector<vector<geometry_msgs::Point>> pointProj::distance_clustering(vector<geometry_msgs::Point> points, float r)
{
    vector<vector<geometry_msgs::Point>> res;
    geometry_msgs::Point p;
    vector<geometry_msgs::Point> centr;
    float d;

    while(points.size()){
        int npm = 0, ind;
        for(int i=0; i<points.size(); i++){
            int np = 0;
            for(int j=0; j<points.size(); j++){
                if(hypot(points[i].x - points[j].x, points[i].y - points[j].y) <= r){
                    np++;
                }
            }
            if(np > npm){
                npm = np;
                ind = i;
            }
        }

        if(npm){
            centr.push_back(points[ind]);
            res.resize(res.size()+1);
            int it = 0;
            while(it<points.size()){
                if(hypot(centr[centr.size()-1].x - points[it].x, centr[centr.size()-1].y - points[it].y) <= r){
                    res[res.size()-1].push_back(points[it]);
                    points.erase(points.begin()+it);
                }else{
                    it++;
                }
            }
            centr[centr.size()-1] = centroid(res[res.size()-1]);
            //centr[centr.size()-1] = equidistantPoint(res[res.size()-1]);
            res[res.size()-1].push_back(centr[centr.size()-1]);
        }
    }

    return res;
}

Poss<int> pointProj::projectedPoss(vector<vector<geometry_msgs::Point>> obst_clusters_points, vector<vector<geometry_msgs::Point>> &obst_centroids_trajectories, vector<vector<geometry_msgs::Point>> &obst_clusters_trajectories, float domin, float obstaclesRegTime, float velT, float velTimeOrig, vector<vector<float>> navArea, vector<geometry_msgs::Point> laser_pos, nav_msgs::OccupancyGrid map_msg, float robotRadius, float robotLinearVel, float &maxObstRadius, float &maxObstVel, vector<geometry_msgs::Point> &res_points, vector<Poss<int>> &projTrajPoss)
{
    res_points.clear();
    Poss<int> res;

    // Almacenar los puntos del mapa
    vector<Poss<int>> obst_clusters = points2Poss(obst_clusters_points, map_msg);

    float velTime = velTimeOrig;
    int NObst = obst_clusters_points.size();

    // Almacenar los centroides
    Poss<int> obst_centroids; obst_centroids(NObst);
    vector<geometry_msgs::Point> obst_centroids_points(NObst);
    for(int i=0; i<NObst; i++){
        // Posiciones sobre el grid
        obst_centroids.x[i] = obst_clusters[i].x[obst_clusters[i].x.size()-1];
        obst_centroids.y[i] = obst_clusters[i].y[obst_clusters[i].y.size()-1];
        // Puntos sobre el mapa
        obst_centroids_points[i] = obst_clusters_points[i][obst_clusters_points[i].size()-1];
        // Tiempos cuando se han visto los obstáculos (centroides)
        //obst_centroids_points[obst_centroids_points.size()-1].z = obstaclesRegTime;
        obst_centroids_points[i].z = obstaclesRegTime;
        // Registrar en todos los puntos de los obstáculos
        for(int j=0; j<obst_clusters_points[i].size(); j++)
            obst_clusters_points[i][j].z = obstaclesRegTime;
    }

    // Obtener las dimensiones de los obstáculos
    vector<float> obst_radius = computeDimensions(obst_centroids_points, obst_clusters_points);
    // Quedarme con el radio máximo para fijar las dimensiones de todos los obstáculos
    for(int i=0; i<obst_radius.size(); i++)
        //if(maxObstRadius < obst_radius[i]){
        if(maxObstRadius < obst_radius[i] && obst_radius[i] < 4*robotRadius){
            maxObstRadius = obst_radius[i];
        }

    // Registrar los puntos de las trayectorías (centroides y puntos totales visibles)
    vector<int> obst_traj_ind = appendTrajectoriesInd(obst_centroids_points, obst_clusters_points, obst_centroids_trajectories, obst_clusters_trajectories, robotRadius);

    // Quitar las trayectorías de los obstáculos que ya no son visibles
    clearTrajectories(obst_centroids_trajectories, obst_clusters_trajectories, laser_pos);

    // Cantidad de obstáculos visibles (trayectorías que recorren)
    int NObstTraj = obst_centroids_trajectories.size();

    // Redimensionar las variables de las trayectorías
    vector<vector<geometry_msgs::Pose>> obst_centroids_trajectories_poses(NObstTraj);
    vector<vector<geometry_msgs::Twist>> obst_traj_cmd_vel(NObstTraj);
    vector<geometry_msgs::Twist> obst_mean_cmd_vel(NObstTraj);
    vector<vector<geometry_msgs::Pose>> obst_centroids_poses_proj(NObstTraj);
    vector<float> obst_traj_radius(NObstTraj, 0);

    //float maxObstVel = 0;
    int xaux, yaux;
    geometry_msgs::Pose auxPose;

    //maxObstVel = 0;
    for(int i=0; i<NObstTraj; i++){
        // Pasar a poses los puntos de las trayectorías de los obstáculos
        //obst_centroids_trajectories_poses[i] = points2Poses(obst_centroids_trajectories[i], velT);
        //obst_centroids_trajectories_poses[i] = points2Poses(obst_centroids_trajectories[i], velTime);
        obst_centroids_trajectories_poses[i] = points2Poses(obst_centroids_trajectories[i], velT, velTimeOrig);
        
        // Obtener las velocidades de los obstáculos (en base a las trayectorías de los centroides)
        obst_traj_cmd_vel[i] = estimateVelocities(obst_centroids_trajectories_poses[i], velT, robotRadius/4);
        //obst_traj_cmd_vel[i] = estimateVelocities(obst_centroids_trajectories_poses[i], velT, map_msg.info.resolution);
        obst_traj_cmd_vel[i] = smoothVelocities(obst_traj_cmd_vel[i], velTimeOrig/velT);
        obst_mean_cmd_vel[i] = computeMeanVelocity(obst_traj_cmd_vel[i], velTimeOrig/velT);

        if(obst_mean_cmd_vel[i].linear.x) obst_mean_cmd_vel[i].angular.z = 0.0;
        else if(obst_mean_cmd_vel[i].angular.z) obst_mean_cmd_vel[i].linear.x = 0.0;
    }


    for(int i=0; i<NObstTraj; i++){
        // Almacenar la velocidad del obstáculo más rápido
        if(maxObstVel < obst_mean_cmd_vel[i].linear.x){
            maxObstVel = obst_mean_cmd_vel[i].linear.x;
        }

        if(obst_mean_cmd_vel[i].linear.x) obst_mean_cmd_vel[i].angular.z = 0.0;
        else if(obst_mean_cmd_vel[i].angular.z) obst_mean_cmd_vel[i].linear.x = 0.0;
    }

    // Ajustar el tiempo de proyección de la velocidad y poses del obstáculo
    //velTime = velTimeOrig - domin/maxObstVel;
    velTime = domin/(robotLinearVel+maxObstVel); // Supuesto tiempo en el que colisionaré con el obstáculo más cercano
    if(velTime <= 0 || velTime > velTimeOrig) // Por si he calculado mal
        velTime = velTimeOrig;

    float timeMin = robotLinearVel/(2*robotRadius); // Tiempo mínimo de proyección (el tiempo que tarda el robot en "llegar a la siguiente posición")
    if(velTime < timeMin)
        velTime = timeMin;

    //cout<<"velTime i: "<<velTime<<endl;

    projTrajPoss.clear();
    projTrajPoss.resize(NObstTraj);

    for(int i=0; i<NObstTraj; i++){

        // Supongo que el obstáculo que estoy viendo se va a mover a la velocidad del obstáculo más rápido que he visto
        if(obst_mean_cmd_vel[i].linear.x)
            obst_mean_cmd_vel[i].linear.x = maxObstVel;

        // Proyectar las poses del obstáculo
        if(obst_centroids_trajectories_poses[i].size() == 0) continue; // No hay trayectoría, el obstáculo no se va a mover
        obst_centroids_poses_proj[i].push_back(obst_centroids_trajectories_poses[i][obst_centroids_trajectories_poses[i].size()-1]);
        for(float j=velT; j<=velTime; j+=velT){
            if(obst_mean_cmd_vel[i].linear.x == 0) continue; // NO se ha detectado movimiento, el obstáculo no se va a mover
            // Obtener la siguiente posición en base a la velocidad "observada" del obstáculo
            //auxPose = calculateNextPose(obst_centroids_poses_proj[i][obst_centroids_poses_proj[i].size()-1], obst_mean_cmd_vel[i], velT, obst_traj_radius[i]);
            //auxPose = calculateNextPose(obst_centroids_poses_proj[i][obst_centroids_poses_proj[i].size()-1], obst_mean_cmd_vel[i], velT, maxObstRadius);
            auxPose = calculateNextPose(obst_centroids_poses_proj[i][obst_centroids_poses_proj[i].size()-1], obst_mean_cmd_vel[i], velT, maxObstRadius);
            //cout<<"PUNTO SIGUIENTE : ("<<auxPose.position.x<<", "<<auxPose.position.y<<")"<<endl;
            if(isnan(auxPose.position.x) || isnan(auxPose.position.y)) break;
            // No insertar si la posición proyectada es un obstáculo
            xaux = (auxPose.position.y - map_msg.info.origin.position.x)/map_msg.info.resolution;
            yaux = (auxPose.position.x - map_msg.info.origin.position.y)/map_msg.info.resolution;
            if(navArea[xaux][yaux] == 0) break;
            // Almacenar el punto en la trayectoría proyectado sobre el mapa
            obst_centroids_poses_proj[i].push_back(auxPose);
            // Almacenar el punto proyectado sobre el grid
            res.push(xaux, yaux);
            projTrajPoss[i].push(xaux, yaux); // Almaceno el centroide de la proyección
            // Almacenar el punto proyectado sobre el mapa
            res_points.push_back(auxPose.position);
        }
        
    }

    return res;
} // Proyectar las posiciones de los obstáculos


vector<Poss<int>> pointProj::joinProjWithSet(vector<Poss<int>> proj_poss, vector<Poss<int>> set)
{
    vector<Poss<int>> res = set;
    int ind;
    float d, dmin;
    for(int i=0; i<proj_poss.size(); i++){
        if(proj_poss[i].x.size() == 0) continue;
        dmin = INFINITY;
        ind = -1;
        for(int j=0; j<set.size(); j++){
            for(int k=0; k<set[j].x.size(); k++){
                d = hypot(proj_poss[i].x[0] - set[j].x[k], proj_poss[i].y[0] - set[j].y[k]);
                if(d == 0){
                    dmin = d;
                    ind = j;
                    break;
                }
                if(dmin > d){
                    dmin = d;
                    ind = j;
                }
            }
            if(dmin == 0) break;
        }
        if(ind >= 0){
            res[ind].append(proj_poss[i]);
        }
    }
    return res;
}

Poss<int> pointProj::projection(geometry_msgs::Pose robotPose, vector<geometry_msgs::Point> visible_obstacles_pos, nav_msgs::Path robotPath, vector<vector<geometry_msgs::Point>> &obst_clusters_points, vector<vector<geometry_msgs::Point>> &obst_centroids_trajectories, vector<vector<geometry_msgs::Point>> &obst_clusters_trajectories, vector<float> timeReg, float domin, float obstaclesRegTime, float velT, float velTimeOrig, vector<vector<float>> navArea, vector<geometry_msgs::Point> laser_pos, nav_msgs::OccupancyGrid map_msg, float robotRadius, float robotLinearVel, float &maxObstRadius, float &maxObstVel, vector<geometry_msgs::Point> &all_proj_obst_points, vector<Poss<int>> &projTrajPoss)
{
    Poss<int> all_proj_obst_poss;

    if(visible_obstacles_pos.size() == 0) return all_proj_obst_poss;

    // Distancia al obstáculo dinámico más cercano
    domin = pointProj::closestDist(robotPose.position.x, robotPose.position.y, visible_obstacles_pos);

    // Tiempo estimado de colisión con el obstáculo más cercano
    float velTime = domin/robotLinearVel;

    // Calcular la velocidad que lleva el robot
    float mean_velocity = computeVelocity(robotPath, timeReg, velTime); // Calcular la velocidad media de los últimos "velTime" segundos

    //cout<<"velTime a: "<<velTime<<endl;

    // Con la velocidad que llevo, donde voy a estar en el siguiente timestep
    if(velTime > 4*robotRadius/mean_velocity)
        velTime = 4*robotRadius/mean_velocity;

    //cout<<"velTime d: "<<velTime<<endl;

    // Agrupar los puntos visibles de los obstáculos
    obst_clusters_points = distance_clustering(visible_obstacles_pos, robotRadius);

    if(domin){
        //all_proj_obst_poss.clear();
        all_proj_obst_poss = projectedPoss(obst_clusters_points, obst_centroids_trajectories, obst_clusters_trajectories, domin, obstaclesRegTime, velT, velTime, navArea, laser_pos, map_msg, robotRadius, robotLinearVel, maxObstRadius, maxObstVel, all_proj_obst_points, projTrajPoss);
    }else{
        // Limpiar las variables de las trayectorías
        obst_centroids_trajectories.clear();
        obst_clusters_trajectories.clear();
        projTrajPoss.clear();
        all_proj_obst_points.clear();
    }

    return all_proj_obst_poss;
}

Poss<int> pointProj::focusProjection(geometry_msgs::Pose robotPose, vector<geometry_msgs::Point> visible_obstacles_pos, vector<vector<float>> map, float robotRadius, nav_msgs::OccupancyGrid mapMsg, vector<vector<geometry_msgs::Point>> &obst_clusters_points, vector<geometry_msgs::Point> &all_proj_obst_points, vector<Poss<int>> &projTrajPoss)
{
    Poss<int> all_proj_obst_poss;

    if(visible_obstacles_pos.size() == 0) return all_proj_obst_poss;

    // Agrupar los puntos visibles de los obstáculos
    obst_clusters_points = distance_clustering(visible_obstacles_pos, robotRadius);

    vector<float> orientations(obst_clusters_points.size(), 0);
    geometry_msgs::Point centr, point;
    int x, y;

    all_proj_obst_points.clear();
    projTrajPoss.clear();
    projTrajPoss.resize(obst_clusters_points.size());
    for(int i=0; i<obst_clusters_points.size(); i++){
        // Obtener el centroide de los puntos de los obstáculos
        centr = pointProj::centroid(obst_clusters_points[i]);
        //xc = (centr.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
        //yc = (centr.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;
        // Obtener orientación hacia el robot
        orientations[i] = atan2(robotPose.position.y - centr.y, robotPose.position.x - centr.x);
        // Proyectar el punto
        point.x = centr.x + robotRadius * cos(orientations[i]);
        point.y = centr.y + robotRadius * sin(orientations[i]);

        x = (point.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
        y = (point.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;

        if(check_pos_in_grid(x, y, map) && map[x][y]){
            // Almacenar los puntos
            all_proj_obst_points.push_back(point);
            projTrajPoss[i].push(x, y);
        }
    }

    return all_proj_obst_poss;
}

