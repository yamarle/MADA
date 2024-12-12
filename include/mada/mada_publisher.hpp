#ifndef MADA_PUBLISHER_HPP
#define MADA_PUBLISHER_HPP

#include <mada/util/funciones.hpp>
#include <mada/util/generic_fmm.hpp>
#include <mada/exploration_functions.hpp>
#include <mada/dynamic_areas.hpp>

#include <mada/mada_util.h>

namespace mada_publisher{

vector<geometry_msgs::Point> poss2Points(Poss<int> poss, const nav_msgs::OccupancyGrid& map)
{
    vector<geometry_msgs::Point> res;
    geometry_msgs::Point p;
    
    for(int i=0; i<poss.x.size(); i++){
        p.y = poss.x[i] * map.info.resolution + map.info.origin.position.x;
        p.x = poss.y[i] * map.info.resolution + map.info.origin.position.y;
        res.push_back(p);
    }
    
    return res;
}

vector<geometry_msgs::Point> poss2LineList(vector<Poss<int>> poss, const nav_msgs::OccupancyGrid& mapMsg)
{
    vector<geometry_msgs::Point> res;
    vector<geometry_msgs::Point> _p;
    for(int i=0; i<poss.size(); i++){
        if(poss[i].x.size()){
            _p = poss2Points(poss[i], mapMsg);
            for(int j=0; j<_p.size()-1; j++){
                res.push_back(_p[j]);
                res.push_back(_p[j+1]);
            }
            res.push_back(_p[_p.size()-1]);
            res.push_back(_p[0]);
        }
    }

    return res;
}

void textSelection(vector<int> indSel, Poss<int> poss, vector<float> vals, Poss<int> &p, vector<float> &v)
{
	p.clear(); v.clear();
	for(int i=0; i<indSel.size(); i++){
		p.push(poss.x[indSel[i]], poss.y[indSel[i]]);
		v.push_back(vals[indSel[i]]);
	}
}

void updateTextMarkers(visualization_msgs::MarkerArray& marker_array, vector<int> xpos, vector<int> ypos, vector<float> valores, nav_msgs::OccupancyGrid map, double altura_texto, vector<double> color_texto)
{
    marker_array.markers.clear(); // Limpiar el MarkerArray para actualizar los marcadores

    geometry_msgs::Point p;
    string s;
    double nfrac, frac;
    for (size_t i = 0; i < xpos.size(); ++i) {
        visualization_msgs::Marker text_marker;

        text_marker.header.frame_id = map.header.frame_id;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "dynamic_text_namespace";
        text_marker.id = i;  // ID único para cada texto
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;

        // Establecer la posición del marcador
        p.y = xpos[i] * map.info.resolution + map.info.origin.position.x;
        p.x = ypos[i] * map.info.resolution + map.info.origin.position.y;
        text_marker.pose.position = p;
        text_marker.pose.orientation.w = 1.0;  // Sin rotación

        // Configuración de escala y color del texto
        text_marker.scale.z = altura_texto;  // Altura del texto
        text_marker.color.r = color_texto[0];
        text_marker.color.g = color_texto[1];
        text_marker.color.b = color_texto[2];
        text_marker.color.a = 1.0;

        // Valores para mostrar
        frac = std::modf(valores[i], &nfrac);
        if(frac == 0){
            s = to_string(nfrac);
            s = s.substr(0,s.find('.')); // Quitar la parte posterior al punto (los decimales que son 0's)
        }else{
            //s = to_string(valores[i]).substr(0,4);
            s = to_string(valores[i]);
            s = s.substr(0,s.find('.')+3); // Quedarme con dos decimales
        }
        
        text_marker.text = s;

        // Añadir el marcador al MarkerArray
        marker_array.markers.push_back(text_marker);
    }
}

class Publishers{

public:

	Publishers(){}

	ros::NodeHandle nh;

	ros::Publisher pathPub, totalPathPub, explMapPub, gradientPub, segmMapPub, segFrontPub, targetPosPub, dynObstPub;
	nav_msgs::Path rosPath;
	nav_msgs::OccupancyGrid explorationMap;
	nav_msgs::OccupancyGrid gradient;
	nav_msgs::OccupancyGrid segmMap;
	visualization_msgs::Marker segFrontPoints, segGraphPoints, segTreePoints, segBranchPoints;
	visualization_msgs::Marker targetPoints;
	visualization_msgs::Marker dynObstMarker, dynAreasMarker, dynTrajMarker; // Obstáculo visible, Área dinámica construida, Trayectoría recorrida por el obstáculo

	void initializePublishers(ros::NodeHandle n, string robot_name, string map_topic, const nav_msgs::OccupancyGrid& mapMsg, string pathToFollow_topic, nav_msgs::Path totalPath, fmm_segment::Segments fmm_segments, vector<geometry_msgs::Point> centroidPoints)
	{

		nh = n;

	    // Para publicar el camino planificado con FMM
	    pathPub = nh.advertise<nav_msgs::Path>(robot_name + "pathToGoal", 10);
	    rosPath.header.seq = 0; rosPath.header.stamp = ros::Time::now(); rosPath.header.frame_id = map_topic;

	    // Para publicar el camino total recorrido por el robot
	    totalPathPub = nh.advertise<nav_msgs::Path>(robot_name + "totalPath", 10);
	    totalPath.header.stamp = ros::Time::now(); totalPath.header.frame_id = map_topic;

	    // Para publicar el mapa de exploración
	    explMapPub = nh.advertise<nav_msgs::OccupancyGrid>(robot_name + "explorationMap", 10);
	    explorationMap.header.seq = 0; explorationMap.header.stamp = ros::Time::now(); explorationMap.header.frame_id = map_topic;
	    explorationMap.info = mapMsg.info; // Debería ser lo mismo
	    //explorationMap.data = mada_util::grid2occupancyGridData(explored_map);

	    // Para publicar el gradiente
	    gradientPub = nh.advertise<nav_msgs::OccupancyGrid>(robot_name + "gradient", 10);
	    gradient.header.seq = 0; gradient.header.stamp = ros::Time::now(); gradient.header.frame_id = map_topic;
	    gradient.info = mapMsg.info; // Debería ser lo mismo

	    // Para publicar el mapa de los segmentos
	    segmMapPub = nh.advertise<nav_msgs::OccupancyGrid>(robot_name + "segmentsMap", 10);
	    segmMap.header.seq = 0; segmMap.header.stamp = ros::Time::now(); segmMap.header.frame_id = map_topic;
	    //segmMap.info = mapMsg.info; segmMap.data = mada_util::map2occupancyGridData(fmm_segments.map);

	    // Para publicar las fronteras de los segmentos
	    segFrontPub = nh.advertise<visualization_msgs::Marker>(robot_name + "segFront", 10);
	    vector<bool> gr;
	    initializeSegments(segFrontPoints, segGraphPoints, segTreePoints, segBranchPoints, fmm_segments.contour_poss, fmm_segments.graph, centroidPoints, map_topic, mapMsg);

	    // Para publicar los objetivos del agente
	    targetPosPub = nh.advertise<visualization_msgs::Marker>(robot_name + "targetPos", 10);
	    initializeTargetPoints(targetPoints, map_topic, mapMsg);

	    // Para publicar los obstáculos dinámicos visibles
	    dynObstPub = nh.advertise<visualization_msgs::Marker>(robot_name + "visibleDynObst", 10);
	    initializeDynAreas(dynObstMarker, dynAreasMarker, dynTrajMarker, map_topic, mapMsg);
	}

	void initializeDynAreas(visualization_msgs::Marker &dynObstMarker, visualization_msgs::Marker &dynAreasMarker, visualization_msgs::Marker &dynTrajMarker, string map_topic, const nav_msgs::OccupancyGrid& mapMsg)
	{
		dynObstMarker.id = 0;
	    dynAreasMarker.id = 1;
	    dynTrajMarker.id = 2;
	    dynObstMarker.header.seq = dynAreasMarker.header.seq = dynTrajMarker.header.seq = 0;
	    dynObstMarker.header.stamp = dynAreasMarker.header.stamp = dynTrajMarker.header.stamp = ros::Time::now();
	    dynObstMarker.header.frame_id = dynAreasMarker.header.frame_id = dynTrajMarker.header.frame_id = map_topic;
	    dynObstMarker.ns = "visibleDynObst";
	    dynAreasMarker.ns = "dynamicAreas";
	    dynTrajMarker.ns = "obstacleTraj";
	    dynObstMarker.action = dynAreasMarker.action = dynTrajMarker.action = visualization_msgs::Marker::ADD;
	    dynObstMarker.pose.orientation.w= dynAreasMarker.pose.orientation.w = dynTrajMarker.pose.orientation.w = 1;
	    dynObstMarker.type = visualization_msgs::Marker::POINTS;
	    dynAreasMarker.type = visualization_msgs::Marker::LINE_LIST;
	    dynTrajMarker.type = visualization_msgs::Marker::POINTS;
	    dynObstMarker.scale.x = dynObstMarker.scale.y = mapMsg.info.resolution;
	    dynObstMarker.color.r = dynObstMarker.color.b = dynObstMarker.color.g = 0.01; dynObstMarker.color.a = 1.0;

	    dynAreasMarker.scale.x = dynAreasMarker.scale.y = mapMsg.info.resolution/1.5;
	    dynAreasMarker.color.r = 0.7; dynAreasMarker.color.a = 1.0;

	    dynTrajMarker.scale.x = dynTrajMarker.scale.y = mapMsg.info.resolution/2;
	    dynTrajMarker.color.r = 0.7; dynTrajMarker.color.a = 1.0;
	}

	void initializeTargetPoints(visualization_msgs::Marker &targetPoints, string map_topic, const nav_msgs::OccupancyGrid& mapMsg)
	{
		targetPoints.id = 0;
	    targetPoints.header.seq = 0;
	    targetPoints.header.stamp = ros::Time::now();
	    targetPoints.header.frame_id = map_topic;
	    targetPoints.ns = "targetPoints";
	    targetPoints.action = visualization_msgs::Marker::ADD;
	    targetPoints.pose.orientation.w = 1;
	    targetPoints.type = visualization_msgs::Marker::POINTS;
	    targetPoints.scale.x = targetPoints.scale.y = mapMsg.info.resolution;
	    targetPoints.color.r = 1.0; targetPoints.color.a = 1.0;
	}

	void initializeSegments(visualization_msgs::Marker &segFrontPoints, visualization_msgs::Marker &segGraphPoints, visualization_msgs::Marker &segTreePoints, visualization_msgs::Marker &segBranchPoints, vector<Poss<int>> contour_poss, vector<vector<bool>> graph, vector<geometry_msgs::Point> centroidPoints, string map_topic, const nav_msgs::OccupancyGrid& mapMsg)
	{
		segFrontPoints.id = 0;
	    segGraphPoints.id = 1;
	    segTreePoints.id = 2;
	    segBranchPoints.id = 3;
	    segFrontPoints.header.seq = segGraphPoints.header.seq = segTreePoints.header.seq = segBranchPoints.header.seq = 0;
	    segFrontPoints.header.stamp = segGraphPoints.header.stamp = segTreePoints.header.stamp = segBranchPoints.header.stamp = ros::Time::now();
	    segFrontPoints.header.frame_id = segGraphPoints.header.frame_id = segTreePoints.header.frame_id = segBranchPoints.header.frame_id = map_topic;
	    segFrontPoints.ns = segGraphPoints.ns = segTreePoints.ns = segBranchPoints.ns = "segPoints";
	    segFrontPoints.action = segGraphPoints.action = segTreePoints.action = segBranchPoints.action = visualization_msgs::Marker::ADD;
	    segFrontPoints.pose.orientation.w = segGraphPoints.pose.orientation.w = segTreePoints.pose.orientation.w = segBranchPoints.pose.orientation.w = 1;
	    //segFrontPoints.type = visualization_msgs::Marker::POINTS;
	    segFrontPoints.type = visualization_msgs::Marker::LINE_LIST;
	    segGraphPoints.type = visualization_msgs::Marker::LINE_LIST;
	    segTreePoints.type = visualization_msgs::Marker::LINE_LIST;
	    segBranchPoints.type = visualization_msgs::Marker::LINE_LIST;
	    segFrontPoints.scale.x = segFrontPoints.scale.y = segGraphPoints.scale.x = segGraphPoints.scale.y = mapMsg.info.resolution/2;
	    segTreePoints.scale.x = segTreePoints.scale.y = mapMsg.info.resolution;
	    segBranchPoints.scale.x = segBranchPoints.scale.y = mapMsg.info.resolution;
	    segFrontPoints.color.b = 1.0; segFrontPoints.color.a = 1.0;
	    segGraphPoints.color.r = 1.0; segGraphPoints.color.a = 0.6;
	    segTreePoints.color.b = 1.0; segTreePoints.color.a = 1.0;
	    vector<geometry_msgs::Point> _p;
	    for(int i=0; i<contour_poss.size(); i++){
	        _p = mada_util::tfGridPos2MapPos(mapMsg, contour_poss[i].y, contour_poss[i].x);
	        for(int j=1; j<_p.size(); j++){
	        	segFrontPoints.points.push_back(_p[j-1]);
	        	segFrontPoints.points.push_back(_p[j]);
	        }
        	segFrontPoints.points.push_back(_p[_p.size()-1]);
        	segFrontPoints.points.push_back(_p[0]);


	        //_p = mada_util::tfGridPos2MapPos(mapMsg, contour_poss[i].y, contour_poss[i].x);
	        //segFrontPoints.points.insert(segFrontPoints.points.end(), _p.begin(), _p.end());
	    }

	    segFrontPoints.color.b = segFrontPoints.color.r = segFrontPoints.color.g = 0.6;
	    //segGraphPoints.points = mada_util::buildGraphPoints(fmm_segments.centroids.y, fmm_segments.centroids.x, fmm_segments.graph, mapMsg);
	    segGraphPoints.points = mada_util::linkPoints(centroidPoints, graph);
	}

	void publishTopics(exploration::Map_exploration expl_obj, vector<geometry_msgs::Point> visible_obstacles_pos, nav_msgs::Path totalPath, vector<geometry_msgs::Pose> path2GoalPoses, string map_topic)
	{
		// Publicar el mapa de exploración
        explorationMap.data = mada_util::map2occupancyGridData(expl_obj.get_seen_map());
        explMapPub.publish(explorationMap); // Publico

        // Publicar el gradiente
        gradient.data = mada_util::grad2occupancyGridData(expl_obj.get_gradient());
        gradientPub.publish(gradient); // Publico

        // Publicar los obstáculos dinámicos vistos por el agente
        dynObstMarker.points = visible_obstacles_pos;
        dynObstPub.publish(dynObstMarker);

        // Publicar las posiciones objetivo del robot
        geometry_msgs::Point target;
        target.x = path2GoalPoses[path2GoalPoses.size()-1].position.x;
        target.y = path2GoalPoses[path2GoalPoses.size()-1].position.y;
        targetPoints.points.clear();
        targetPoints.points.push_back(target);
        targetPosPub.publish(targetPoints);

        // Publicar los segmentos
        segmMapPub.publish(segmMap); // Publico

        // Publicar el camino que sigue el robot
        //rosPath.poses = mada_util::poses2PoseStamped(path2GoalPoses);
        rosPath.poses = mada_util::poses2PoseStamped(path2GoalPoses, map_topic);
        rosPath.header.frame_id = map_topic;
        rosPath.header.stamp = ros::Time::now();
        pathPub.publish(rosPath); // Publico

        totalPath.header.stamp = ros::Time::now(); totalPath.header.frame_id = map_topic;
        totalPathPub.publish(totalPath); // Publico
        
	}

	void publishTopics(exploration::Areas_exploration expl_obj, dynamic_areas::Areas areas, Poss<int> obstacles_trajectories_points, vector<geometry_msgs::Point> visible_obstacles_pos, vector<int> number_of_obstacles, vector<int> traversed_segments, vector<vector<int>> tree, vector<geometry_msgs::Point> centroidPoints, nav_msgs::Path totalPath, vector<geometry_msgs::Pose> path2GoalPoses, string map_topic, const nav_msgs::OccupancyGrid& mapMsg)
	{
		// Publicar el mapa de exploración
        explorationMap.data = mada_util::map2occupancyGridData(expl_obj.get_real_seen_map());
        explMapPub.publish(explorationMap); // Publico

        // Publicar el gradiente
        gradient.data = mada_util::grad2occupancyGridData(expl_obj.get_gradient());
        gradientPub.publish(gradient); // Publico

        // Publicar los obstáculos dinámicos vistos por el agente
        dynObstMarker.points = visible_obstacles_pos;

        for(int i=0; i<dynObstMarker.points.size(); i++){
			dynObstMarker.points[i].z = 0.01;
        }

        dynObstPub.publish(dynObstMarker);
        dynTrajMarker.points = poss2Points(obstacles_trajectories_points, mapMsg);
        dynObstPub.publish(dynTrajMarker);

        vector<Poss<int>> active_areas_contours;
        for(int i=0; i<areas.N; i++)
            if(number_of_obstacles[i]) active_areas_contours.push_back(areas.contours[i]);
        dynAreasMarker.points = poss2LineList(active_areas_contours, mapMsg);

        for(int i=0; i<dynAreasMarker.points.size(); i++){
			dynAreasMarker.points[i].z = 0.015;
        }

        dynObstPub.publish(dynAreasMarker);

        // Publicar los puntos de los segmentos
        //segFrontPoints.color.b = 1.0; segFrontPoints.color.a = 0.25;
        segFrontPub.publish(segFrontPoints);
        segGraphPoints.color.r = 1.0; segGraphPoints.color.a = 0.10;
        segFrontPub.publish(segGraphPoints);

        // Publicar los centroides de los segmentos con el grafo del entorno
        segTreePoints.points = mada_util::linkPoints(centroidPoints, tree); // Representar árbol
        segTreePoints.color.b = 1.0; segTreePoints.color.a = 0.25;
        segFrontPub.publish(segTreePoints);

        //segBranchPoints.points = mada_util::linkPoints(centroidPoints, branch); // Representar árbol
        if(traversed_segments.size() && traversed_segments[0]<0) traversed_segments.erase(traversed_segments.begin());
        segBranchPoints.points = mada_util::linkPoints(centroidPoints, traversed_segments); // Representar árbol
        segBranchPoints.color.b = 1.0; segBranchPoints.color.a = 1;
        segFrontPub.publish(segBranchPoints);

        // Publicar las posiciones objetivo del agente
        if(expl_obj.get_target_area() >= 0){
            Poss<int> _target = expl_obj.get_poss2explore()[expl_obj.get_target_area()];
            targetPoints.points = mada_util::tfGridPos2MapPos(mapMsg, _target.y, _target.x);
            //targetPoints.color.r = 1.0; targetPoints.color.a = 0.25;
            targetPosPub.publish(targetPoints);
        }

        // Publicar los segmentos
        segmMapPub.publish(segmMap); // Publico

        // Publicar el camino que sigue el robot
        //rosPath.poses = mada_util::poses2PoseStamped(path2GoalPoses);
        rosPath.poses = mada_util::poses2PoseStamped(path2GoalPoses, map_topic);

        for(int i=0; i<rosPath.poses.size(); i++){
			rosPath.poses[i].pose.position.z = 0.02;
        }

        rosPath.header.frame_id = map_topic;
        rosPath.header.stamp = ros::Time::now();
        pathPub.publish(rosPath); // Publico

        totalPath.header.stamp = ros::Time::now(); totalPath.header.frame_id = map_topic;
        totalPathPub.publish(totalPath); // Publico
	}

};

}

#endif