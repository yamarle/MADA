// SFMM: Social Fast Marching Method
// Paper: "Fast Marching Solution for the Social Path Planning Problem",
// Authors: Javier V. Gómez, Nikolaos Mavridis and Santiago Garrido

#ifndef SOCIAL_FMM_HPP
#define SOCIAL_FMM_HPP

#include <mada/mada_util.h>
#include <mada/projPoints.h>

#include <mada/util/funciones.hpp>
#include <mada/util/generic_fmm.hpp>
#include <mada/util/fmm_2.hpp>
#include <mada/util/path.hpp>
#include <mada/util/cluster.hpp>

#include <mada/dynamic_areas.hpp>


namespace social_FMM{

	float intimate_distance = 0.45;
	float personal_distance = 1.2;

	// Cálculo de las variables SIN orientación
	float computePhi(int x1, int y1, int x2, int y2, float sigmax, float sigmay)
	{
		return exp(-0.5*(pow(x1-x2,2)/pow(sigmax,2) + pow(y1-y2,2)/pow(sigmay,2)) );
	}

	// Cálculo de las variables CON orientación
	float computeRho(float sigmax, float sigmay, float theta)
	{
		return (( (sigmay*sigmay - sigmax*sigmax) * sin(theta) * cos(theta) ) / (sigmax*sigmay));
	}

	bool estaDelante(double x_p, double y_p, double x_q, double y_q, double theta)
	{
		// Vector entre la posición (x_p, y_p) y el punto (x_q, y_q)
		double dx = x_q - x_p;
		double dy = y_q - y_p;

		// Vector de orientación (coseno y seno del ángulo theta)
		double dir_x = cos(theta);
		double dir_y = sin(theta);

		// Producto escalar entre el vector de dirección y el vector hacia el punto
		double producto_escalar = dx * dir_x + dy * dir_y;

		// Si el producto escalar es positivo, el punto está delante
		return producto_escalar > 0;
	}

	double calcularDesalineacion(double x_p, double y_p, double x_q, double y_q, double theta) {
	    // Vector entre la posición (x_p, y_p) y el punto (x_q, y_q)
	    double dx = x_q - x_p;
	    double dy = y_q - y_p;
	    
	    // Vector de orientación (coseno y seno del ángulo theta)
	    double dir_x = cos(theta);
	    double dir_y = sin(theta);
	    
	    // Longitud del vector hacia el punto
	    double distancia = sqrt(dx * dx + dy * dy);
	    
	    // Evitar división por cero en caso de que el punto esté exactamente en la misma posición
	    if (distancia == 0) {
	        return 0.0; // Perfectamente alineado si estamos en el mismo punto
	    }
	    
	    // Producto escalar entre el vector de orientación y el vector hacia el punto
	    double producto_escalar = dx * dir_x + dy * dir_y;
	    
	    // Si el producto escalar es negativo, el punto está detrás
	    if (producto_escalar < 0) {
	        return 0.0; // Desalineación es 0 si el punto está detrás
	    }
	    
	    // Cálculo del coseno del ángulo entre los vectores
	    double cos_alpha = producto_escalar / distancia;
	    
	    // Desalineación es 1 - |cos(alpha)|
	    return 1.0 - fabs(cos_alpha);
	}

	float computePhi(int x1, int y1, int x2, int y2, float sigmax, float sigmay, float theta)
	{
		float rho = computeRho(sigmax, sigmay, theta);
		float rho_ = 1-pow(rho, 2);

		float val = 1/(2 * M_PI * sigmax * sigmay * sqrt(rho_));

		float eval = 1 / (2*rho_);

		float dx = x1 - x2;
		float dy = y1 - y2;

		float xcontr = pow(dx,2)/pow(sigmax,2);
		float ycontr = pow(dy,2)/pow(sigmay,2);

		float corrcontr = (2*rho*dx*dy)/(sigmax*sigmay);

		float phi = val * exp(-eval * (xcontr - corrcontr + ycontr));

		return phi;
	}

	Poss<int> getPoss(Poss<int> obstPoss, vector<vector<float>> static_map, float infl_range)
	{
		Poss<int> res;

		FMM gr_do(obstPoss.x, obstPoss.y, static_map);
		gr_do.expand_dist(infl_range, res);

		return res;
	} // Quedarme con los puntos que correponden al "inflado" de los obstáculos

	void only_line(int x, int y, float dist, vector<int> &xi, vector<int> &yi, vector<float> &vi)
	{
		Poss<int> res = bresenham::points(x, y, x + dist, y);
		xi = res.x;
		yi = res.y;
		vi.resize(res.x.size(), 0);
	}

	void social_infl(int x, int y, vector<vector<float>> static_map, float frontal_range, float back_range, vector<int> &xi, vector<int> &yi, vector<float> &vi)
	{
		vector<vector<float>> gauss_map = static_map;

		// Propagar unicamente las posiciones del inflado de la orientación del "humano"
		Poss<int> inflPoss;
		FMM gr_do(x, y, static_map);
		gr_do.expand_dist(frontal_range, inflPoss);
		//gr_do.expand_dist(800, inflPoss);

		Poss<int> all_possd;
		Poss<int> all_possa;
		vector<float> vald, vala;
		float maxd, maxa;
		maxd = maxa = 0;
		float val;

		for(int i=0; i<inflPoss.x.size(); i++){
			if(estaDelante(x, y, inflPoss.x[i], inflPoss.y[i], 0)){
				val = computePhi(x, y, inflPoss.x[i], inflPoss.y[i], frontal_range, back_range, 0);

				all_possd.push(inflPoss.x[i], inflPoss.y[i]);
				vald.push_back(val);
				if(maxd < val) maxd = val;

			}else{
				val = computePhi(x, y, inflPoss.x[i], inflPoss.y[i], back_range, back_range);

				all_possa.push(inflPoss.x[i], inflPoss.y[i]);
				vala.push_back(val);
				if(maxa < val) maxa = val;

			}
		}

		// Normalizar 
		for(int i=0; i<vald.size(); i++){
			vald[i] /= maxd;

			xi.push_back(all_possd.x[i] - x);
			yi.push_back(all_possd.y[i] - y);
			//vi.push_back(vald[i]);
			vi.push_back(1 - vald[i]);
		}

		for(int i=0; i<vala.size(); i++){
			vala[i] /= maxa;

			xi.push_back(all_possa.x[i] - x);
			yi.push_back(all_possa.y[i] - y);
			//vi.push_back(vala[i]);
			vi.push_back(1 - vala[i]);
		}

	}

	void tfPoint(double x_p, double y_p, double x_q, double y_q, double theta, double &x_tf, double &y_tf)
	{
	    // Trasladar el punto Q con respecto al origen P
	    double x_trasladado = x_q + x_p;
	    double y_trasladado = y_q + y_p;

	    // Aplicar la rotación
	    x_tf = x_trasladado * cos(theta) - y_trasladado * sin(theta);
	    y_tf = x_trasladado * sin(theta) + y_trasladado * cos(theta);

	}

	Poss<int> tfPoints_(Poss<int> poss, int x, int y, double theta)
	{
		int x_trasladado, y_trasladado, x_rotado, y_rotado;
		for(int i=0; i<poss.x.size(); i++){
			x_trasladado = poss.x[i] - x;
		    y_trasladado = poss.y[i] - y;

		    // Aplicar la rotación
		    x_rotado = x_trasladado * cos(theta) - y_trasladado * sin(theta);
		    y_rotado = x_trasladado * sin(theta) + y_trasladado * cos(theta);

		    poss.x[i] = x_rotado + x;
		    poss.y[i] = y_rotado + y;

		}
		return poss;
	}

	Poss<int> tfPoints(Poss<int> poss, int x, int y, double theta)
	{
		int x_trasladado, y_trasladado, x_rotado, y_rotado;
		for(int i=0; i<poss.x.size(); i++){
		    // Aplicar la rotación
		    x_rotado = poss.x[i] * cos(theta) - poss.y[i] * sin(theta);
		    y_rotado = poss.x[i] * sin(theta) + poss.y[i] * cos(theta);

		    poss.x[i] = x_rotado + x;
		    poss.y[i] = y_rotado + y;

		}
		return poss;
	}

	void generateSocialModel(float sigmax, float sigmay, vector<int> &xsm, vector<int> &ysm, vector<float> &vsm)
	{
		vector<vector<float>> map(10*sigmax, vector<float>(10*sigmay, 1));
		social_infl(5*sigmax, 5*sigmay, map, sigmax, sigmay, xsm, ysm, vsm);
		//social_infl(5*sigmax, 5*sigmay, map, sigmay, sigmax, xsm, ysm, vsm);
	}

	void insertSocialModel(pair<int, int> gridPosition, vector<vector<float>> &navArea, vector<vector<geometry_msgs::Point>> obst_clusters_points, vector<Poss<int>> proj_obst_poss, Poss<int> gaussian_poss, vector<float> gaussian_infl, nav_msgs::OccupancyGrid mapMsg, int robotRadiusGrid,float static_obst_max)
	{
		if(proj_obst_poss.size() != obst_clusters_points.size()) return;
		vector<float> orientations(proj_obst_poss.size(), 0); // Orientación de los obstáculos con respecto al grid
        Poss<int> obstPoss; obstPoss(proj_obst_poss.size());

        Poss<int> obst_infl_poss; // Poses del inflado de cada obstáculo dinámico

        //cout<<proj_obst_poss.size()<<"/"<<obst_clusters_points.size()<<" obstáuclos"<<endl;
        for(int i=0; i<proj_obst_poss.size(); i++){
            geometry_msgs::Point centr = pointProj::centroid(obst_clusters_points[i]);
            //obstPoss.push((centr.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution, (centr.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution);
            obstPoss.x[i] = (centr.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
            obstPoss.y[i] = (centr.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;
            if(proj_obst_poss[i].x.size() <= 1){
                //cout<<"puntos NO proyectados"<<endl;
                //obstPoss.push(proj_obst_poss[i].x[0], proj_obst_poss[i].y[0]);
                //orientations[i] = atan2(gridPosition.first - obstPoss.x[i], gridPosition.second - obstPoss.y[i]);
                orientations[i] = atan2(gridPosition.second - obstPoss.y[i], gridPosition.first - obstPoss.x[i]);

                proj_obst_poss[i].push(obstPoss.x[i] + 2*robotRadiusGrid*cos(orientations[i]), obstPoss.y[i] + 2*robotRadiusGrid*sin(orientations[i]));

            }else if(proj_obst_poss[i].x.size() > 1){
                //cout<<"puntos proyectados"<<endl;
                //obstPoss.x[i] = proj_obst_poss[i].x[0]; obstPoss.y[i] = proj_obst_poss[i].y[0];
                //orientations[i] = atan2(proj_obst_poss[i].x[proj_obst_poss[i].x.size()-1] - proj_obst_poss[i].x[0], proj_obst_poss[i].y[proj_obst_poss[i].y.size()-1] - proj_obst_poss[i].y[0]);
                orientations[i] = atan2(proj_obst_poss[i].y[proj_obst_poss[i].y.size()-1] - proj_obst_poss[i].y[0], proj_obst_poss[i].x[proj_obst_poss[i].x.size()-1] - proj_obst_poss[i].x[0]);
            }
        }
        //cout<<"inserto las posiciones"<<endl;
        //obstacles_infl_map = social_FMM::computeObstInflMap(obstPoss, orientations, static_map, personal_distance, intimate_distance);
        vector<vector<float>> obstacles_infl_map(navArea.size(), vector<float>(navArea[0].size(), 0));
        for(int i=0; i<obstPoss.x.size(); i++){
        	//cout<<"obst "<<i<<" ("<<obstPoss.x[i]<<", "<<obstPoss.y[i]<<") - "<<orientations[i]<<endl;
            //obst_infl_poss = social_FMM::tfPoints(gaussian_poss, obstPoss.x[i], obstPoss.y[i], geometry::ang2rad(orientations[i]));
            obst_infl_poss = social_FMM::tfPoints(gaussian_poss, obstPoss.x[i], obstPoss.y[i], orientations[i]);
            for(int j=0; j<obst_infl_poss.x.size(); j++){
                if(check_pos_in_grid(obst_infl_poss.x[j], obst_infl_poss.y[j], navArea) && navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]]){
                    obstacles_infl_map[obst_infl_poss.x[j]][obst_infl_poss.y[j]] = static_obst_max * gaussian_infl[j];
                    if(navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]] > obstacles_infl_map[obst_infl_poss.x[j]][obst_infl_poss.y[j]]){
                        navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]] = obstacles_infl_map[obst_infl_poss.x[j]][obst_infl_poss.y[j]];
                    }
                }
            }
        }
	}

	void pushSocialModel(pair<int, int> gridPosition, vector<vector<float>> &navArea, vector<vector<geometry_msgs::Point>> obst_clusters_points, vector<Poss<int>> proj_obst_poss, Poss<int> gaussian_poss, vector<float> gaussian_infl, nav_msgs::OccupancyGrid mapMsg, int robotRadiusGrid,float static_obst_max)
	{
		if(proj_obst_poss.size() != obst_clusters_points.size()) return;
		vector<float> orientations(proj_obst_poss.size(), 0); // Orientación de los obstáculos con respecto al grid
        Poss<int> obstPoss; obstPoss(proj_obst_poss.size());

        Poss<int> obst_infl_poss; // Poses del inflado de cada obstáculo dinámico

        //cout<<proj_obst_poss.size()<<"/"<<obst_clusters_points.size()<<" obstáuclos"<<endl;
        for(int i=0; i<proj_obst_poss.size(); i++){
            geometry_msgs::Point centr = pointProj::centroid(obst_clusters_points[i]);
            //obstPoss.push((centr.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution, (centr.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution);
            obstPoss.x[i] = (centr.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
            obstPoss.y[i] = (centr.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;
            if(proj_obst_poss[i].x.size() <= 1){
                //cout<<"puntos NO proyectados"<<endl;
                //obstPoss.push(proj_obst_poss[i].x[0], proj_obst_poss[i].y[0]);
                //orientations[i] = atan2(gridPosition.first - obstPoss.x[i], gridPosition.second - obstPoss.y[i]);
                orientations[i] = atan2(gridPosition.second - obstPoss.y[i], gridPosition.first - obstPoss.x[i]);

                proj_obst_poss[i].push(obstPoss.x[i] + 2*robotRadiusGrid*cos(orientations[i]), obstPoss.y[i] + 2*robotRadiusGrid*sin(orientations[i]));

            }else if(proj_obst_poss[i].x.size() > 1){
                //cout<<"puntos proyectados"<<endl;
                //obstPoss.x[i] = proj_obst_poss[i].x[0]; obstPoss.y[i] = proj_obst_poss[i].y[0];
                //orientations[i] = atan2(proj_obst_poss[i].x[proj_obst_poss[i].x.size()-1] - proj_obst_poss[i].x[0], proj_obst_poss[i].y[proj_obst_poss[i].y.size()-1] - proj_obst_poss[i].y[0]);
                orientations[i] = atan2(proj_obst_poss[i].y[proj_obst_poss[i].y.size()-1] - proj_obst_poss[i].y[0], proj_obst_poss[i].x[proj_obst_poss[i].x.size()-1] - proj_obst_poss[i].x[0]);
            }
        }
        //cout<<"inserto las posiciones"<<endl;
        //obstacles_infl_map = social_FMM::computeObstInflMap(obstPoss, orientations, static_map, personal_distance, intimate_distance);
        vector<vector<float>> obstacles_infl_map(navArea.size(), vector<float>(navArea[0].size(), 0));
        for(int i=0; i<obstPoss.x.size(); i++){
        	//cout<<"obst "<<i<<" ("<<obstPoss.x[i]<<", "<<obstPoss.y[i]<<") - "<<orientations[i]<<endl;
            //obst_infl_poss = social_FMM::tfPoints(gaussian_poss, obstPoss.x[i], obstPoss.y[i], geometry::ang2rad(orientations[i]));
            obst_infl_poss = social_FMM::tfPoints(gaussian_poss, obstPoss.x[i], obstPoss.y[i], orientations[i]);
            for(int j=0; j<obst_infl_poss.x.size(); j++){
                if(check_pos_in_grid(obst_infl_poss.x[j], obst_infl_poss.y[j], navArea)){
                    navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]] = static_obst_max * gaussian_infl[j];
                }
            }
        }
	}

	void focusSocialModel(pair<int, int> gridPosition, vector<vector<float>> &navArea, vector<vector<geometry_msgs::Point>> obst_clusters_points, vector<Poss<int>> proj_obst_poss, Poss<int> gaussian_poss, vector<float> gaussian_infl, nav_msgs::OccupancyGrid mapMsg, int robotRadiusGrid, float static_obst_max)
	{
		//if(proj_obst_poss.size() != obst_clusters_points.size()) return;
		vector<float> orientations(obst_clusters_points.size(), 0); // Orientación de los obstáculos con respecto al grid
        Poss<int> obstPoss; obstPoss(obst_clusters_points.size());

        Poss<int> obst_infl_poss; // Poses del inflado de cada obstáculo dinámico

        for(int i=0; i<obst_clusters_points.size(); i++){
        	geometry_msgs::Point centr = pointProj::centroid(obst_clusters_points[i]);
            //obstPoss.push((centr.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution, (centr.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution);
            obstPoss.x[i] = (centr.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
            obstPoss.y[i] = (centr.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;
            orientations[i] = atan2(gridPosition.second - obstPoss.y[i], gridPosition.first - obstPoss.x[i]);    
        }

        //cout<<"inserto las posiciones"<<endl;
        //obstacles_infl_map = social_FMM::computeObstInflMap(obstPoss, orientations, static_map, personal_distance, intimate_distance);
        vector<vector<float>> obstacles_infl_map(navArea.size(), vector<float>(navArea[0].size(), 0));
        for(int i=0; i<obstPoss.x.size(); i++){
        	//cout<<"obst "<<i<<" ("<<obstPoss.x[i]<<", "<<obstPoss.y[i]<<") - "<<orientations[i]<<endl;
            //obst_infl_poss = social_FMM::tfPoints(gaussian_poss, obstPoss.x[i], obstPoss.y[i], geometry::ang2rad(orientations[i]));
            obst_infl_poss = social_FMM::tfPoints(gaussian_poss, obstPoss.x[i], obstPoss.y[i], orientations[i]);
            for(int j=0; j<obst_infl_poss.x.size(); j++){
                if(check_pos_in_grid(obst_infl_poss.x[j], obst_infl_poss.y[j], navArea) && navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]]){
                    navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]] = static_obst_max * gaussian_infl[j];
                }
            }
        }
	} // Esta es la función que enfoca la gaussiana a partir de conjuntos de puntos (pertenecientes a cada obstáculo)

	void focusSocialModel(pair<int, int> gridPosition, vector<vector<float>> &navArea, vector<int> obst_poss_x, vector<int> obst_poss_y, vector<float> obst_or, Poss<int> gaussian_poss, vector<float> gaussian_infl, int robotRadiusGrid,float static_obst_max)
	{
		vector<vector<float>> obstacles_infl_map(navArea.size(), vector<float>(navArea[0].size(), 0));
		Poss<int> obst_infl_poss; // Poses del inflado de cada obstáculo dinámico
        for(int i=0; i<obst_poss_x.size(); i++){
            obst_infl_poss = social_FMM::tfPoints(gaussian_poss, obst_poss_x[i], obst_poss_y[i], obst_or[i]);
            for(int j=0; j<obst_infl_poss.x.size(); j++){
                if(check_pos_in_grid(obst_infl_poss.x[j], obst_infl_poss.y[j], navArea) && navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]]){
                    navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]] = static_obst_max * gaussian_infl[j];
                }
            }
        }
	}

	float findValue(Poss<int> poss, vector<vector<float>> map)
	{
		for(int i=0; i<poss.x.size(); i++){
			if(map[poss.x[i]][poss.y[i]]){
				return map[poss.x[i]][poss.y[i]];
			}
		}
		return 0;
	}

	void insertSocialModel(pair<int, int> gridPosition, vector<vector<float>> &navArea, vector<vector<int>> dynAreasMap, vector<Poss<int>> dynAreasPoss, vector<vector<geometry_msgs::Point>> obst_clusters_points, vector<Poss<int>> proj_obst_poss, Poss<int> gaussian_poss, vector<float> gaussian_infl, nav_msgs::OccupancyGrid mapMsg, int robotRadiusGrid, float static_obst_max)
	{
		if(proj_obst_poss.size() != obst_clusters_points.size()) return;
		vector<float> orientations(proj_obst_poss.size(), 0); // Orientación de los obstáculos con respecto al grid
        Poss<int> obstPoss; obstPoss(proj_obst_poss.size());

        Poss<int> obst_infl_poss; // Poses del inflado de cada obstáculo dinámico
        float da_value; // Valor del área dinámica

        //cout<<proj_obst_poss.size()<<"/"<<obst_clusters_points.size()<<" obstáuclos"<<endl;
        for(int i=0; i<proj_obst_poss.size(); i++){
            geometry_msgs::Point centr = pointProj::centroid(obst_clusters_points[i]);
            //obstPoss.push((centr.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution, (centr.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution);
            obstPoss.x[i] = (centr.y - mapMsg.info.origin.position.y) / mapMsg.info.resolution;
            obstPoss.y[i] = (centr.x - mapMsg.info.origin.position.x) / mapMsg.info.resolution;

            if(check_pos_in_grid(obstPoss.x[i], obstPoss.y[i], navArea) && dynAreasMap[obstPoss.x[i]][obstPoss.y[i]]>=0){
            	//if(navArea[obstPoss.x[i]][obstPoss.y[i]] == 0) 
            	//da_value = navArea[obstPoss.x[i]][obstPoss.y[i]];
            	da_value = findValue(dynAreasPoss[dynAreasMap[obstPoss.x[i]][obstPoss.y[i]]], navArea);
            }else{
            	da_value = static_obst_max;
            }
            //cout<<"valor área "<<i<<" : "<<da_value<<endl;

            if(proj_obst_poss[i].x.size() <= 1){
                //cout<<"puntos NO proyectados"<<endl;
                //obstPoss.push(proj_obst_poss[i].x[0], proj_obst_poss[i].y[0]);
                //orientations[i] = atan2(gridPosition.first - obstPoss.x[i], gridPosition.second - obstPoss.y[i]);
                orientations[i] = atan2(gridPosition.second - obstPoss.y[i], gridPosition.first - obstPoss.x[i]);

                proj_obst_poss[i].push(obstPoss.x[i] + 2*robotRadiusGrid*cos(orientations[i]), obstPoss.y[i] + 2*robotRadiusGrid*sin(orientations[i]));

            }else if(proj_obst_poss[i].x.size() > 1){
                //cout<<"puntos proyectados"<<endl;
                //obstPoss.x[i] = proj_obst_poss[i].x[0]; obstPoss.y[i] = proj_obst_poss[i].y[0];
                //orientations[i] = atan2(proj_obst_poss[i].x[proj_obst_poss[i].x.size()-1] - proj_obst_poss[i].x[0], proj_obst_poss[i].y[proj_obst_poss[i].y.size()-1] - proj_obst_poss[i].y[0]);
                orientations[i] = atan2(proj_obst_poss[i].y[proj_obst_poss[i].y.size()-1] - proj_obst_poss[i].y[0], proj_obst_poss[i].x[proj_obst_poss[i].x.size()-1] - proj_obst_poss[i].x[0]);
            }
        }
        //cout<<"inserto las posiciones"<<endl;
        //obstacles_infl_map = social_FMM::computeObstInflMap(obstPoss, orientations, static_map, personal_distance, intimate_distance);
        vector<vector<float>> obstacles_infl_map(navArea.size(), vector<float>(navArea[0].size(), 0));
        for(int i=0; i<obstPoss.x.size(); i++){
        	//cout<<"obst "<<i<<" ("<<obstPoss.x[i]<<", "<<obstPoss.y[i]<<") - "<<orientations[i]<<endl;
            //obst_infl_poss = social_FMM::tfPoints(gaussian_poss, obstPoss.x[i], obstPoss.y[i], geometry::ang2rad(orientations[i]));
            obst_infl_poss = social_FMM::tfPoints(gaussian_poss, obstPoss.x[i], obstPoss.y[i], orientations[i]);
            for(int j=0; j<obst_infl_poss.x.size(); j++){
                if(check_pos_in_grid(obst_infl_poss.x[j], obst_infl_poss.y[j], navArea) && navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]]){
                    obstacles_infl_map[obst_infl_poss.x[j]][obst_infl_poss.y[j]] = da_value * gaussian_infl[j];
                    if(navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]] > obstacles_infl_map[obst_infl_poss.x[j]][obst_infl_poss.y[j]]){
                        navArea[obst_infl_poss.x[j]][obst_infl_poss.y[j]] = obstacles_infl_map[obst_infl_poss.x[j]][obst_infl_poss.y[j]];
                    }
                }
            }
        }
	}

};

#endif