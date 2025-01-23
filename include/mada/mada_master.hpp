#ifndef MADA_MASTER_HPP
#define MADA_MASTER_HPP

#include <mada/mada_robots_listener.hpp>
#include <std_msgs/Bool.h>

namespace MadaMaster{

	void computeInitialMaps(const int& x, const int& y, vector<vector<float>>& grid, const float& infl_dist, vector<vector<float>>& obstGrad, vector<vector<float>>& navArea, vector<vector<float>>& navGrad, fmm_segment::Segments& fmm_segments)
	{
		vector<Poss<int>> positions;
		if(grid.size()){
	        // ESPERAR A RECIBIR EL MAPA

	        // Calculo el gradiente desde la posición del agente para obtener el espacio libre en el que se va a mover
	        FMM gr(x, y, grid);
	        vector<vector<float>> grad = gr.compute_gradient_();

	        // Cargarme las posiciones "no alcanzables" del grid
	        positions = fix_free_space(grid, grad);

	        // Gradiente de los obstáculos
	        FMM ogr(positions[1].x, positions[1].y, grid);
	        obstGrad = ogr.compute_gradient_();

	        //navArea = mada_util::navigableGradient(obstGrad, (1.1*robotRadius)/mapMsg.info.resolution);
	        navArea = mada_util::navigableGradient(obstGrad, infl_dist);
	        // Cargarme las posiciones que están cerca de los obstáculos
	        positions = fix_free_space_(grid, navArea);

	        // Gradiente desde el robot
	        FMM cgr(x, y, navArea); // Voy a alejar al agente de los obstáculos
	        navGrad = cgr.compute_gradient_();

	        // Segmentar el entorno
	        fmm_segments = fmm_segment::compute_segments(grid, navArea);
	        for(int i=0; i<fmm_segments.frontier_poss.size(); i++) fmm_segments.contour_poss[i] = geometry::sort_points(fmm_segments.centroids.x[i], fmm_segments.centroids.y[i], fmm_segments.contour_poss[i]);
	    }
	}

	std::vector<geometry_msgs::Point> generateFootprint(const geometry_msgs::Pose& pose, const float& size_x, const float& size_y)
	{
	    std::vector<geometry_msgs::Point> footprint;

	    // Obtener la posición (centro) y la orientación (yaw) del robot
	    float cx = pose.position.x;
	    float cy = pose.position.y;
	    float yaw = tf::getYaw(pose.orientation);  // Convertir la orientación a yaw (ángulo sobre el eje z)

	    // Tamaños del robot (medidas del rectángulo)
	    float half_x = size_x / 2.0;
	    float half_y = size_y / 2.0;

	    // Coordenadas locales de los vértices del rectángulo (sin rotar ni trasladar)
	    std::vector<std::pair<float, float>> local_vertices = { { half_x,  half_y}, {-half_x,  half_y}, {-half_x, -half_y}, { half_x, -half_y} };

	    // Rotar y trasladar los vértices al sistema global
	    for (const auto& vertex : local_vertices) {
	        // Rotación de los puntos (punto * matriz de rotación 2D)
	        float x_rot = vertex.first * cos(yaw) - vertex.second * sin(yaw);
	        float y_rot = vertex.first * sin(yaw) + vertex.second * cos(yaw);

	        // Trasladar al sistema global (sumando la posición del robot)
	        geometry_msgs::Point global_point;
	        global_point.x = cx + x_rot;
	        global_point.y = cy + y_rot;
	        global_point.z = 0.0;  // Considerando el robot en 2D

	        footprint.push_back(global_point);
	    }

	    // Cerrar el rectángulo
	    footprint.push_back(footprint[0]);

	    return footprint;
	}

	template <typename T>
	T clamp(const T& value, const T& min, const T& max)
	{
	    return (value < min) ? min : (value > max) ? max : value;
	}

	// Producto escalar entre dos puntos
	float dotProduct(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
	{
	    return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	// Norma cuadrada de un punto (vector)
	float squaredNorm(const geometry_msgs::Point& a)
	{
	    return a.x * a.x + a.y * a.y + a.z * a.z;
	}
	
	float distanceBetweenSegments(const geometry_msgs::Point& P1, const geometry_msgs::Point& P2, const geometry_msgs::Point& Q1, const geometry_msgs::Point& Q2)
	{
		// Función para calcular la distancia más corta entre dos segmentos de recta

	    // Vectores directores de los segmentos
	    geometry_msgs::Point d1, d2, r;
	    d1.x = P2.x - P1.x;
	    d1.y = P2.y - P1.y;
	    d1.z = P2.z - P1.z;

	    d2.x = Q2.x - Q1.x;
	    d2.y = Q2.y - Q1.y;
	    d2.z = Q2.z - Q1.z;

	    r.x = P1.x - Q1.x;
	    r.y = P1.y - Q1.y;
	    r.z = P1.z - Q1.z;

	    geometry_msgs::Point _p;

	    float a = squaredNorm(d1); // ||d1||^2
	    float e = squaredNorm(d2); // ||d2||^2
	    float f = dotProduct(d2, r); // d2 . r

	    // Si ambos segmentos son puntos (longitud cero)
	    if (a <= 1e-6 && e <= 1e-6) {
	        return sqrt(squaredNorm(r));
	    }

	    // Si el primer segmento es un punto
	    if (a <= 1e-6) {
	        float t = clamp(dotProduct(d2, r) / e, 0.0f, 1.0f);
	        geometry_msgs::Point closestPoint;
	        closestPoint.x = Q1.x + t * d2.x;
	        closestPoint.y = Q1.y + t * d2.y;
	        closestPoint.z = Q1.z + t * d2.z;
	        _p.x = P1.x - closestPoint.x;
	        _p.y = P1.y - closestPoint.y;
	        _p.z = P1.z - closestPoint.z;
	        return sqrt(squaredNorm(_p));
	    }

	    // Si el segundo segmento es un punto
	    if (e <= 1e-6) {
	        float t = clamp(-dotProduct(d1, r) / a, 0.0f, 1.0f);
	        geometry_msgs::Point closestPoint, _p;
	        closestPoint.x = P1.x + t * d1.x;
	        closestPoint.y = P1.y + t * d1.y;
	        closestPoint.z = P1.z + t * d1.z;
	        _p.x = Q1.x - closestPoint.x;
	        _p.y = Q1.y - closestPoint.y;
	        _p.z = Q1.z - closestPoint.z;
	        return sqrt(squaredNorm(_p));
	    }

	    // Ambos segmentos tienen longitud no cero
	    float b = dotProduct(d1, d2); // d1 . d2
	    float c = dotProduct(d1, r);  // d1 . r

	    float denominator = a * e - b * b; // Determinante

	    float s, t;

	    // Si los segmentos no son paralelos
	    if (denominator != 0.0f) {
	        s = clamp((b * f - c * e) / denominator, 0.0f, 1.0f);
	    } else {
	        s = 0.0f; // Arbitrario, segmentos paralelos
	    }

	    t = clamp((b * s + f) / e, 0.0f, 1.0f);

	    // Recalcular s para estar dentro de los límites del segmento
	    s = clamp((b * t - c) / a, 0.0f, 1.0f);

	    // Puntos más cercanos en cada segmento
	    geometry_msgs::Point closestPointOnP, closestPointOnQ;
	    closestPointOnP.x = P1.x + s * d1.x;
	    closestPointOnP.y = P1.y + s * d1.y;
	    closestPointOnP.z = P1.z + s * d1.z;

	    closestPointOnQ.x = Q1.x + t * d2.x;
	    closestPointOnQ.y = Q1.y + t * d2.y;
	    closestPointOnQ.z = Q1.z + t * d2.z;

	    // Distancia entre los puntos más cercanos
	    _p.x = closestPointOnP.x - closestPointOnQ.x;
	    _p.y = closestPointOnP.y - closestPointOnQ.y;
	    _p.z = closestPointOnP.z - closestPointOnQ.z;
	    return sqrt(squaredNorm(_p));
	}


	float distanceBetweenRobots(const vector<geometry_msgs::Point>& robot1, const vector<geometry_msgs::Point>& robot2)
	{
		float res = INFINITY, d;
		for(int i=0; i<robot1.size()-1; i++){
			for(int j=0; j<robot2.size()-1; j++){
				d = distanceBetweenSegments(robot1[i], robot1[i+1], robot2[j], robot2[j+1]);
				if(res > d) res = d;
			}
		}
		return res;
	}

	class MadaMaster
    {
	private:
		ros::NodeHandle nh_;

        ros::Subscriber map_sub; // Suscriptor del mapa
        string map_topic; // Topic del mapa

        vector<string> robot_names; // Lista de nombres de robots
        vector<geometry_msgs::Pose> robot_poses; // Lista de poses de robots
        RobotsListener::Positions robotPositions;
        RobotsListener::RobotsListener *robotsListener;
        float rX, rY;

        vector<string> obstacle_names; // Lista de nombres de obstáculos
        vector<geometry_msgs::Pose> obstacle_poses; // Lista de poses de obstáculos
        RobotsListener::Positions obstaclePositions;
        RobotsListener::RobotsListener *obstaclesListener;
        float oX, oY;

        float collision_distance = 0.1; // Distancia para considerar colisión
        float infl_dist = 0; // Distancia de inflado para los robots

        vector<vector<float>> rr_distances;
        vector<vector<float>> oo_distances;
        vector<vector<float>> ro_distances;

        bool rr_coll = false;
        bool oo_coll = false;
        bool ro_coll = false;

        int Nr = 0;
        int No = 0;

        vector<nav_msgs::Path> robotPaths;
        vector<nav_msgs::Path> obstaclePaths;

        vector<vector<geometry_msgs::Point>> robots;
        vector<vector<geometry_msgs::Point>> obstacles;

        nav_msgs::OccupancyGrid map;
        vector<vector<float>> grid;

        float val; // Variable auxiliar para mis cálculos

	public:
		// Constructores
		MadaMaster(string map_topic, const vector<string>& robot_names, string robot_pose_name, float rX, float rY, const vector<string>& obstacle_names, string obstacle_pose_name, float oX, float oY)
		{
			set_basics(map_topic, robot_names, robot_pose_name, rX, rY, obstacle_names, obstacle_pose_name, oX, oY);
		}

		MadaMaster(string map_topic, const vector<string>& robot_names, string robot_pose_name, float rX, float rY, const vector<string>& obstacle_names, string obstacle_pose_name, float oX, float oY, float collision_distance, float infl_dist)
		{
			set_basics(map_topic, robot_names, robot_pose_name, rX, rY, obstacle_names, obstacle_pose_name, oX, oY);
			set_distances(collision_distance, infl_dist);
		}

		// Fijado de variables
		void set_basics(string map_topic, const vector<string>& robot_names, string robot_pose_name, float rX, float rY, const vector<string>& obstacle_names, string obstacle_pose_name, float oX, float oY)
		{
			this->map_topic = map_topic;
			this->robot_names = robot_names;
			this->obstacle_names = obstacle_names;

			// Suscribirse a las cosas
			// Mapa
			map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, &MadaMaster::mapCallback, this);
			// Poses de robots
			robotsListener = new RobotsListener::RobotsListener(robot_names, robot_pose_name);
			// Poses de obstáculos
			obstaclesListener = new RobotsListener::RobotsListener(obstacle_names, obstacle_pose_name);

			// Inicializar variables
			Nr = robot_names.size(); // Nº de robots
			No = obstacle_names.size(); // Nº de obstáculos

			this->rX = rX; this->rY = rY;
			this->oX = oX; this->oY = oY;

			rr_distances.resize(Nr, vector<float>(Nr, INFINITY)); // Distancias robot - robot
			oo_distances.resize(No, vector<float>(No, INFINITY)); // Distancias obstáculo - obstáculo
			ro_distances.resize(Nr, vector<float>(No, INFINITY)); // Distancias robot - obstáculo

			robotPaths.resize(Nr);
			obstaclePaths.resize(No);

			// Inicializar las posiciones de los robots y obstáculos
			initializeAll();

			// Fijar las posiciones
			updatePositions();

			robots.resize(Nr);
			obstacles.resize(No);
		}

		void set_distances(float collision_distance, float infl_dist)
		{
			this->collision_distance = collision_distance;
			this->infl_dist = infl_dist;
		}

		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg)
		{
			map = *mapMsg;
			grid.resize(mapMsg->info.height, vector<float>(mapMsg->info.width, 0));
		    for (size_t y = 0; y < mapMsg->info.height; ++y){
		        for (size_t x = 0; x < mapMsg->info.width; ++x){
		            // Obtener el valor de ocupación de la celda
		            val = mapMsg->data[y * mapMsg->info.width + x];
		            if(val==0) grid[y][x] = 1;
		            else grid[y][x] = 0;
		        }
		    }
		}

		void initializeAll()
		{
			// Inicializar el mapa y las poses de todos los agentes (robots y obstáculos)
			while (ros::ok() && grid.size() == 0 && !robotsListener->areAllPosesInitialized() && !obstaclesListener->areAllPosesInitialized()) {
		        cout<<"Esperando las poses de los obstáculos..."<<endl;
		        ros::spinOnce(); // Procesar callbacks
		        ros::Duration(0.1).sleep();
		    }
		}

		void updatePositions()
		{
			robotPositions = robotsListener->getPositions(map);
			obstaclePositions = obstaclesListener->getPositions(map);
		}

		void updateFootprints()
		{
			for(int i=0; i<Nr; i++){
				robots[i] = generateFootprint(robotPositions.poses[i], rX, rY);
			}
			for(int i=0; i<No; i++){
				obstacles[i] = generateFootprint(obstaclePositions.poses[i], oX, oY);
			}
		}

		void updateRRDistances()
		{
			rr_coll = false;
			for(int i=0; i<Nr; i++){
				for(int j=i+1; j<Nr; j++){
					rr_distances[i][j] = distanceBetweenRobots(robots[i], robots[j]);
					if(rr_distances[i][j] <= collision_distance) rr_coll = true;
				}
			}
		}

		void updateOODistances()
		{
			oo_coll = false;
			for(int i=0; i<No; i++){
				for(int j=0; j<No; j++){
					if(i!=j){
						oo_distances[i][j] = distanceBetweenRobots(obstacles[i], obstacles[j]);
						if(oo_distances[i][j] <= collision_distance) oo_coll = true;
					}
				}
			}
		}

		void updateRODistances()
		{
			ro_coll = false;
			for(int i=0; i<Nr; i++){
				for(int j=0; j<No; j++){
					ro_distances[i][j] = distanceBetweenRobots(robots[i], obstacles[j]);
					if(ro_distances[i][j] <= collision_distance) ro_coll = true;
				}
			}
		}

		bool collision()
		{
			return rr_coll * oo_coll * ro_coll;
		}

		bool getRRCollision()
		{
			return rr_coll;
		}

		bool getOOCollision()
		{
			return oo_coll;
		}

		bool getROCollision()
		{
			return ro_coll;
		}

		RobotsListener::Positions getRobotPositions()
		{
			return robotPositions;
		}

		RobotsListener::Positions getObstaclePositions()
		{
			return obstaclePositions;
		}

		vector<vector<float>> get_grid()
		{
			return grid;
		}

		nav_msgs::OccupancyGrid get_map()
		{
			return map;
		}

    };

}
#endif