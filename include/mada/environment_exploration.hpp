
#ifndef ENVIRONMENT_EXPLORATION_HPP
#define ENVIRONMENT_EXPLORATION_HPP

#include "utilities/dynamic.hpp"
#include "dynamic_areas.hpp"
#include "exploration_functions.hpp"
#include "navigation_functions.hpp"

#include "utilities/generic_fmm.hpp"
#include "utilities/graph_functions.hpp"

#include "utilities/visibility_functions.hpp"

namespace environment_exploration{

	vector<int> collision_data(string filename, Poss<int> agents_poss, Poss<int> obst_poss, int it)
	{
		vector<int> res;
		for(int i=0; i<agents_poss.x.size(); i++){
			for(int j=0; j<obst_poss.x.size(); j++){
				if(agents_poss.x[i] == obst_poss.x[j] && agents_poss.y[i] == obst_poss.y[j]){
					res.push_back(agents_poss.x[i]); res.push_back(agents_poss.y[i]);
					break; // El agente no puede colicionar en 2 posiciones al mismo tiempo
				}
			}
		}
		if(res.size()){
			res.insert(res.begin(), it); // Insertar el instante
			// Guardar el vector
			save_vect_(filename.c_str(), res); // Insertar en el mismo fichero
		}
		return res;
	} // Formar y guardar el vector con las colisiones: posición e instante

	vector<int> iteration_data(Poss<int> agents_poss, Poss<int> goals_poss, bool to_goal)
	{
	    vector<int> it_data;
	    it_data.push_back(to_goal);
	    for(int i=0; i<agents_poss.x.size(); i++) it_data.push_back(agents_poss.x[i]);
	    for(int i=0; i<agents_poss.y.size(); i++) it_data.push_back(agents_poss.y[i]);
	    for(int i=0; i<goals_poss.x.size(); i++) it_data.push_back(goals_poss.x[i]);
	    for(int i=0; i<goals_poss.x.size(); i++) it_data.push_back(goals_poss.y[i]);
	    return it_data;
	} // Formar el vector para cada iteración con: las posiciones de los agetnes, sus objetivos y el destino que llevan

	vector<float> exploration_results(vector<vector<int>> temp_map, vector<vector<float>> static_map)
	{
		vector<float> res(2,0); // Media, varianza
		int n = 0;
		for(int i=0; i<temp_map.size(); i++)
			for(int j=0; j<temp_map[0].size(); j++)
				if(static_map[i][j] > 0){
					res[0]+=temp_map[i][j];
					n++;
				}
		res[0] /= n;

		for(int i=0; i<temp_map.size(); i++)
			for(int j=0; j<temp_map[0].size(); j++)
				if(static_map[i][j] > 0){
					res[1] += pow(temp_map[i][j] - res[0],2);
				}
		res[1] /= n;
		return res;
	} // Resultados referentes a la exploración

	struct Result{
		int mission_time = 0;
		int collisions = 0;
		int non_explored_points;
		int invasions = 0;
		int intrusions = 0;
		float explored_mean = 0;
		float explored_var = 0;
	}; // resultados globales de la misión de exploración

	Result simple_exploration(Poss<int> agents_poss, vector<vector<float>> static_map, Poss<int> free_poss, Poss<int> obstacles, vector<vector<float>> navigable_areas_map, int mission_time, int vrange, int direction_time, int direction_seed, string results_folder, bool collisions)
	{
		Result res;

		Poss<int> goals_poss; goals_poss(1);

		goals_poss = agents_poss;

		res.non_explored_points = free_poss.x.size();
		vector<vector<float>> navigation_grid;

		// Variables de los obstáculos
		int no = obstacles.x.size();
		Poss<int> other_agents;
		vector<int> obstacles_direction(no, 0);
    	vector<int> direction_update(no, 0);
    	vector<vector<float>> navigable_areas_map_ini = navigable_areas_map;
    	Poss<int> obstacles_list; // Lista completa de obstáculos (los que se ven actualmente + los que se han visto previamente)
    	vector<vector<bool>> obstacles_poss_map(static_map.size(), vector<bool>(static_map[0].size(),false)); // Mapa
    	vector<vector<int>> obst_temp_map(static_map.size(), vector<int>(static_map[0].size(), 0));
	 	vector<vector<int>> _obst_temp_map = obst_temp_map;
	 	vector<vector<int>> temp_map = obst_temp_map;

	 	// Mapa "REAL" de lo que se ha visto
	 	vector<vector<int>> real_seen_map(static_map.size(), vector<int>(static_map[0].size(), -1));

	 	vector<int> number_of_obstacles;

		// Inicializar el grid "real"
		vector<vector<float>> world_grid;
		world_grid = static_map;
		for(int i=0; i<obstacles.x.size(); i++) world_grid[obstacles.x[i]][obstacles.y[i]] = 0;

		// Inicializar lo que observa el agente
		Poss<int> visible_poss, visible_obstacles, local_map_poss;
		vector<vector<float>> global_map, local_map;
		global_map = static_map;
		navigation::update_agent_information_no_infl(agents_poss.x[0], agents_poss.y[0], world_grid, static_map, global_map, local_map, local_map_poss, visible_poss, visible_obstacles, vrange);

	    // Las variables de las posiciones de los obstáculos dinámicos
	    vector<vector<bool>> trajectories_map, real_trajectories_map;
	    trajectories_map = real_trajectories_map = obstacles_poss_map;
	    vector<Poss<int>> clusters, real_clusters;
	    Poss<int> obstacles_trajectories_points, real_obstacles_trajectories_points;
	    int number_of_visited_points = 0, real_number_of_visited_points = 0;
	    
	    // Inicializar las variables de los obstáculos que observa el agente
	    dynamic_areas::update_trajectories_map(visible_obstacles, trajectories_map, obstacles_trajectories_points, number_of_visited_points); // Obstáculos que ve el agente

	    // Inicializar las variables de los obstáculos "reales"
	    dynamic_areas::update_trajectories_map(obstacles, real_trajectories_map, real_obstacles_trajectories_points, real_number_of_visited_points); // Todos los obstáculos

	    // Variables de exploración
		exploration::Learn expl_obj(static_map, static_map, vrange);
		vector<float> map_values; // variable que almacena los que se está viendo (obstáculo o espacio libre)
		map_values = get_values_from_map(visible_poss, world_grid); // Qué es lo que se está viendo
	    expl_obj.update_maps(agents_poss.x[0], agents_poss.y[0], visible_poss, map_values); // Actualizar las utilidades en base a lo que se ve
	    expl_obj.initialize_estimated_map();

    	Path goal_path;

    	// Guardar variables
    	vector<int> it_data;
    	vector<int> datos(2); datos[0] = no; datos[1] = 0;
    	bool to_goal = true;
    	if(results_folder.size()){
		    // Percibido
		    obstacles_trajectories_points.save((results_folder+"obstacles_traj_"+to_string(0)+".txt").c_str()); // Trayectorías de los obstáculos
		    // Real
		    obstacles.save((results_folder+"obstacles_"+to_string(0)+".txt").c_str()); // Obstáculos
		    real_obstacles_trajectories_points.save((results_folder+"real_obstacles_traj_"+to_string(0)+".txt").c_str()); // Trayectorías de los obstáculos

    		// Para guardar la información de los goals a cada iteración
        	//save_vect((results_folder+"datos.txt").c_str(), datos);
		    it_data = iteration_data(agents_poss, goals_poss, to_goal);
		    save_vect((results_folder+"it_data.txt").c_str(), it_data);
		}

	    // ITERAR
		for(int it=1; it<mission_time; it++){
			//cout<<"it: "<<it<<endl;
			// --------------------------------------------------------------------------------------------------------------------------------
	        // MOVIMIENTO DE LOS OBSTÁCULOS DINÁMICOS
	        //dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, other_agents, direction_time, direction_seed);
	        if(collisions)
	        	dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, other_agents, direction_time);
	        else
	        	dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, agents_poss, direction_time);
	        world_grid = static_map;
	        for(int i=0; i<obstacles.x.size(); i++) world_grid[obstacles.x[i]][obstacles.y[i]] = 0;
	        // --------------------------------------------------------------------------------------------------------------------------------

        	// --------------------------------------------------------------------------------------------------------------------------------
        	// ACTUALIZACIÓN DE LAS VARIABLES DEL MOVIMIENTO DE "TODOS" LOS OBSTÁCULOS DINÁMICOS
		    dynamic_areas::update_trajectories_map(obstacles, real_trajectories_map, real_obstacles_trajectories_points, real_number_of_visited_points);
		    // --------------------------------------------------------------------------------------------------------------------------------

		    // --------------------------------------------------------------------------------------------------------------------------------
		    // ACTUALIZACIÓN DE LAS VARIABLES DEL MOVIMIENTO DE LOS OBSTÁCULOS DINÁMICOS "VISIBLES"
	        // Actualizar de lo que está viendo el agente
	        navigation::update_agent_information_no_infl(agents_poss.x[0], agents_poss.y[0], world_grid, static_map, global_map, local_map, local_map_poss, visible_poss, visible_obstacles, vrange);
	       	
	        dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, global_map, static_map);
	        for(int i=0; i<visible_obstacles.x.size(); i++) obst_temp_map[visible_obstacles.x[i]][visible_obstacles.y[i]]++;
        	dynamic_areas::update_trajectories_map(obstacles_list, trajectories_map, obstacles_trajectories_points, number_of_visited_points);
	        // --------------------------------------------------------------------------------------------------------------------------------

	        // --------------------------------------------------------------------------------------------------------------------------------
	        // MOVER AL AGENTE
        	// ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        	// PONER COSAS DE INFORMACIÓN DE CADA UNA DE LAS ÁREAS DINÁMICAS
        	// ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        	map_values = get_values_from_map(visible_poss, world_grid);
        	// Actualizar las variables de exploración (mapas basicamente)
        	expl_obj.update_maps(agents_poss.x[0], agents_poss.y[0], visible_poss, map_values);
        	// Obtener camino al goal
        	navigation_grid = static_map;
        	for(int i=0; i<visible_obstacles.x.size(); i++) navigation_grid[visible_obstacles.x[i]][visible_obstacles.y[i]] = 0;
        	goal_path = expl_obj.path2next_goal(agents_poss.x[0], agents_poss.y[0], navigation_grid);
        	vector<vector<float>> goal_grad = expl_obj.gradient2next_goal(agents_poss.x[0], agents_poss.y[0], navigation_grid);

        	// Mover el agente
        	if(goal_path.tam){
		        goals_poss.x[0] = goal_path.x[goal_path.tam-1]; goals_poss.y[0] = goal_path.y[goal_path.tam-1];
		        navigation::move_agent_along_path(agents_poss.x[0], agents_poss.y[0], goal_path, navigation_grid);
		    }else{
		    	//cout<<"No existe camino"<<endl;
		    	if(!res.non_explored_points){
			    	res.mission_time = it;
			        mission_time = 0;
			    }
		    }
	        // --------------------------------------------------------------------------------------------------------------------------------

        	// Actualizar la cantidad de puntos que quedan por explorar
	        for(int i=0; i<visible_poss.x.size(); i++){
	            if(static_map[visible_poss.x[i]][visible_poss.y[i]]){
	                if(temp_map[visible_poss.x[i]][visible_poss.y[i]] == 0) res.non_explored_points--;
	                temp_map[visible_poss.x[i]][visible_poss.y[i]]++;
	            }
	            real_seen_map[visible_poss.x[i]][visible_poss.y[i]] = map_values[i];
	        }

            // Contabilizar las colisiones
		    for(int i=0; i<no; i++){
		        if(agents_poss.x[0] == obstacles.x[i] && agents_poss.y[0] == obstacles.y[i]) res.collisions++;
		        if(sqrt(pow(agents_poss.x[0] - obstacles.x[i],2) + pow(agents_poss.y[0] - obstacles.y[i],2)) <= sqrt(2)) res.invasions++;
		    }
		    // Contabilizar las invasiones
		    if(navigable_areas_map_ini[agents_poss.x[0]][agents_poss.y[0]]) res.intrusions++;

            // Actualizar el tiempo de la misión
            res.mission_time = it;

			// Guardar
			if(results_folder.size()){
		        real_obstacles_trajectories_points.save((results_folder+"real_obstacles_traj_"+to_string(it)+".txt").c_str()); // Trayectorías de los obstáculos
		        obstacles.save((results_folder+"obstacles_"+to_string(it)+".txt").c_str()); // Obstáculos
		        obstacles_trajectories_points.save((results_folder+"obstacles_traj_"+to_string(it)+".txt").c_str()); // Trayectorías de los obstáculos
		        //save_matr((results_folder+"temporal_map_"+to_string(it)+".txt").c_str(), expl_obj.get_seen_map());
		        save_matr((results_folder+"temporal_map_"+to_string(it)+".txt").c_str(), real_seen_map);
		        save_matr((results_folder+"navigation_map_"+to_string(it)+".txt").c_str(), navigation_grid);
		        //save_matr((results_folder+"navigation_map_"+to_string(it)+".txt").c_str(), goal_grad);
		        goal_path.save((results_folder+"goal_path_"+to_string(it)+".txt").c_str());
		        datos[0] = no; datos[1] = it;
		        save_vect((results_folder+"datos.txt").c_str(), datos);

		        it_data = iteration_data(agents_poss, goals_poss, to_goal);
		        save_vect_((results_folder+"it_data.txt").c_str(), it_data); // Insertar en el mismo fichero

		        collision_data((results_folder+"colisiones.txt"), agents_poss, obstacles, it);
		    }

            if(res.non_explored_points == 0) break;

		}

		if(results_folder.size()){
			save_matr((results_folder+"temp_map.txt").c_str(), temp_map);
		}

		vector<float> expl_res = exploration_results(temp_map, static_map);
		res.explored_mean = expl_res[0];
		res.explored_var = expl_res[1];

		return res;
	}

	Result simple_exploration(Poss<int> agents_poss, vector<vector<float>> static_map, Poss<int> free_poss, fmm_segment::Segments fmm_segments, Poss<int> obstacles, vector<vector<float>> navigable_areas_map, int mission_time, int vrange, int direction_time, int direction_seed, string results_folder, bool collisions)
	{
		Result res;

		Poss<int> goals_poss; goals_poss(1);

		// Variables de los segmentos
		vector<vector<int>> static_areas_map = fmm_segments.map;
		vector<Poss<int>> areas_poss = fmm_segments.poss;
		vector<vector<bool>> areas_graph = fmm_segments.graph;
		Poss<int> area_centroids = fmm_segments.centroids;

		goals_poss = agents_poss;

		res.non_explored_points = free_poss.x.size();
		vector<vector<float>> navigation_grid;

		// Variables de los obstáculos
		int no = obstacles.x.size();
		Poss<int> other_agents;
		vector<int> obstacles_direction(no, 0);
    	vector<int> direction_update(no, 0);
    	vector<vector<float>> navigable_areas_map_ini = navigable_areas_map;
    	Poss<int> obstacles_list; // Lista completa de obstáculos (los que se ven actualmente + los que se han visto previamente)
    	vector<vector<bool>> obstacles_poss_map(static_map.size(), vector<bool>(static_map[0].size(),false)); // Mapa
    	vector<vector<int>> obst_temp_map(static_map.size(), vector<int>(static_map[0].size(), 0));
	 	vector<vector<int>> _obst_temp_map = obst_temp_map;
	 	vector<vector<int>> temp_map = obst_temp_map;

	 	// Mapa "REAL" de lo que se ha visto
	 	vector<vector<int>> real_seen_map(static_map.size(), vector<int>(static_map[0].size(), -1));

	 	vector<int> number_of_obstacles;

		// Inicializar el grid "real"
		vector<vector<float>> world_grid;
		world_grid = static_map;
		for(int i=0; i<obstacles.x.size(); i++) world_grid[obstacles.x[i]][obstacles.y[i]] = 0;

		// Inicializar lo que observa el agente
		Poss<int> visible_poss, visible_obstacles, local_map_poss;
		vector<vector<float>> global_map, local_map;
		global_map = static_map;
		navigation::update_agent_information_no_infl(agents_poss.x[0], agents_poss.y[0], world_grid, static_map, global_map, local_map, local_map_poss, visible_poss, visible_obstacles, vrange);

	    // Las variables de las posiciones de los obstáculo dinámicos
	    vector<vector<bool>> trajectories_map, real_trajectories_map;
	    trajectories_map = real_trajectories_map = obstacles_poss_map;
	    vector<Poss<int>> clusters, real_clusters;
	    Poss<int> obstacles_trajectories_points, real_obstacles_trajectories_points;
	    int number_of_visited_points = 0, real_number_of_visited_points = 0;
	    
	    // Inicializar las variables de los obstáculos que observa el agente
	    dynamic_areas::update_trajectories_map(visible_obstacles, trajectories_map, obstacles_trajectories_points, number_of_visited_points); // Obstáculos que ve el agente

	    // Inicializar las variables de los obstáculos "reales"
	    dynamic_areas::update_trajectories_map(obstacles, real_trajectories_map, real_obstacles_trajectories_points, real_number_of_visited_points); // Todos los obstáculos

	    // Variables de exploración
		//exploration::Learn expl_obj(static_map, static_map, vrange);
		exploration::Areas_exploration expl_obj(areas_poss, static_areas_map, static_map, vrange);
		vector<float> map_values; // variable que almacena los que se está viendo (obstáculo o espacio libre)
		map_values = get_values_from_map(visible_poss, world_grid); // Qué es lo que se está viendo

    	Path goal_path;

    	vector<vector<int>> tree;
    	vector<bool> dynamism(fmm_segments.centroids.x.size(),false);
    	// Obtener la distancia máxima de cada segmento
    	vector<float> max_seg_dist(fmm_segments.centroids.x.size(),0);
    	float cd;
    	for(int i=0; i<fmm_segments.centroids.x.size(); i++){
	        for(int j=0; j<fmm_segments.contour_poss[i].x.size(); j++){
	            cd = geometry::distance(fmm_segments.centroids.x[i], fmm_segments.centroids.y[i], fmm_segments.contour_poss[i].x[j], fmm_segments.contour_poss[i].y[j]);
	            if(max_seg_dist[i]<cd){
	                max_seg_dist[i] = cd;
	            }
	        }
	    }

    	// Guardar variables
    	vector<int> it_data;
    	vector<int> datos(2); datos[0] = no; datos[1] = 0;
    	bool to_goal = true;
    	if(results_folder.size()){
		    // Percibido
		    obstacles_trajectories_points.save((results_folder+"obstacles_traj_"+to_string(0)+".txt").c_str()); // Trayectorías de los obstáculos
		    // Real
		    obstacles.save((results_folder+"obstacles_"+to_string(0)+".txt").c_str()); // Obstáculos
		    real_obstacles_trajectories_points.save((results_folder+"real_obstacles_traj_"+to_string(0)+".txt").c_str()); // Trayectorías de los obstáculos

    		// Para guardar la información de los goals a cada iteración
        	//save_vect((results_folder+"datos.txt").c_str(), datos);
		    it_data = iteration_data(agents_poss, goals_poss, to_goal);
		    save_vect((results_folder+"it_data.txt").c_str(), it_data);
		}

	    // ITERAR
		for(int it=1; it<mission_time; it++){

			//cout<<"it: "<<it<<endl;
			//cout<<"it: "<<it<<": "<<expl_obj.get_target_area()<<" - "<<expl_obj.get_target_set()<<"("<<expl_obj.get_target_poss().size()<<")"<<": "<<expl_obj.get_target_set_poss().x.size()<<endl;
			//if(expl_obj.get_target_poss().size() > 1) cin.get();

			// --------------------------------------------------------------------------------------------------------------------------------
	        // MOVIMIENTO DE LOS OBSTÁCULOS DINÁMICOS
	        //dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, other_agents, direction_time, direction_seed);
	        if(collisions)
	        	dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, other_agents, direction_time);
	        else
	        	dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, agents_poss, direction_time);
	        world_grid = static_map;
	        for(int i=0; i<obstacles.x.size(); i++) world_grid[obstacles.x[i]][obstacles.y[i]] = 0;
	        // --------------------------------------------------------------------------------------------------------------------------------

        	// --------------------------------------------------------------------------------------------------------------------------------
        	// ACTUALIZACIÓN DE LAS VARIABLES DEL MOVIMIENTO DE "TODOS" LOS OBSTÁCULOS DINÁMICOS
		    dynamic_areas::update_trajectories_map(obstacles, real_trajectories_map, real_obstacles_trajectories_points, real_number_of_visited_points);
		    // --------------------------------------------------------------------------------------------------------------------------------

		    // --------------------------------------------------------------------------------------------------------------------------------
		    // ACTUALIZACIÓN DE LAS VARIABLES DEL MOVIMIENTO DE LOS OBSTÁCULOS DINÁMICOS "VISIBLES"
	        // Actualizar de lo que está viendo el agente
	        navigation::update_agent_information_no_infl(agents_poss.x[0], agents_poss.y[0], world_grid, static_map, global_map, local_map, local_map_poss, visible_poss, visible_obstacles, vrange);
	       	
	        dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, global_map, static_map);
	        for(int i=0; i<visible_obstacles.x.size(); i++) obst_temp_map[visible_obstacles.x[i]][visible_obstacles.y[i]]++;
        	dynamic_areas::update_trajectories_map(obstacles_list, trajectories_map, obstacles_trajectories_points, number_of_visited_points);
	        // --------------------------------------------------------------------------------------------------------------------------------

	        // --------------------------------------------------------------------------------------------------------------------------------

	        // MOVER AL AGENTE
        	// ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        	// PONER COSAS DE INFORMACIÓN DE CADA UNA DE LAS ÁREAS DINÁMICAS
        	// ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        	map_values = get_values_from_map(visible_poss, world_grid);
        	// Actualizar las variables de exploración (mapas basicamente)
        	expl_obj.update_all(visible_poss, map_values);
        	// Obtener camino al goal
        	navigation_grid = static_map;
        	for(int i=0; i<visible_obstacles.x.size(); i++) navigation_grid[visible_obstacles.x[i]][visible_obstacles.y[i]] = 0;

			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        	// SELECCIÓN DE LOS NUEVOS OBJETIVOS
    		// Reiniciar los objetivos
    		expl_obj.update_targets();
	    	// Todo en una misma función
    		//expl_obj.targets_and_path(agents_poss.x[0], agents_poss.y[0], areas_graph, fmm_segments.centroids, navigation_grid, dynamism, max_seg_dist, vrange, goal_path, tree);
    		expl_obj.targets_and_path(agents_poss.x[0], agents_poss.y[0], areas_graph, fmm_segments.centroids, navigation_grid, max_seg_dist, vrange, goal_path, tree);
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        	// Mover el agente
        	if(goal_path.tam){
		        goals_poss.x[0] = goal_path.x[goal_path.tam-1]; goals_poss.y[0] = goal_path.y[goal_path.tam-1];
		        navigation::move_agent_along_path(agents_poss.x[0], agents_poss.y[0], goal_path, navigation_grid);
		    }else{
		    	//cout<<"No existe camino"<<endl;
		    	if(!res.non_explored_points){
			    	res.mission_time = it;
			        mission_time = 0;
			    }
		    }
	        // --------------------------------------------------------------------------------------------------------------------------------

        	// Actualizar la cantidad de puntos que quedan por explorar
	        for(int i=0; i<visible_poss.x.size(); i++){
	            if(static_map[visible_poss.x[i]][visible_poss.y[i]]){
	                if(temp_map[visible_poss.x[i]][visible_poss.y[i]] == 0) res.non_explored_points--;
	                temp_map[visible_poss.x[i]][visible_poss.y[i]]++;
	            }
	            real_seen_map[visible_poss.x[i]][visible_poss.y[i]] = map_values[i];
	        }

            // Contabilizar las colisiones
		    for(int i=0; i<no; i++){
		        if(agents_poss.x[0] == obstacles.x[i] && agents_poss.y[0] == obstacles.y[i]) res.collisions++;
		        if(sqrt(pow(agents_poss.x[0] - obstacles.x[i],2) + pow(agents_poss.y[0] - obstacles.y[i],2)) <= sqrt(2)) res.invasions++;
		    }
		    // Contabilizar las invasiones
		    if(navigable_areas_map_ini[agents_poss.x[0]][agents_poss.y[0]]) res.intrusions++;

            // Actualizar el tiempo de la misión
            res.mission_time = it;

			// Guardar
			if(results_folder.size()){
				if(obstacles.x.size()){
			        real_obstacles_trajectories_points.save((results_folder+"real_obstacles_traj_"+to_string(it)+".txt").c_str()); // Trayectorías de los obstáculos
			        obstacles.save((results_folder+"obstacles_"+to_string(it)+".txt").c_str()); // Obstáculos
			        obstacles_trajectories_points.save((results_folder+"obstacles_traj_"+to_string(it)+".txt").c_str()); // Trayectorías de los obstáculos
			    }
		        //save_matr((results_folder+"temporal_map_"+to_string(it)+".txt").c_str(), expl_obj.get_seen_map());
		        save_matr((results_folder+"temporal_map_"+to_string(it)+".txt").c_str(), real_seen_map);
		        save_matr((results_folder+"navigation_map_"+to_string(it)+".txt").c_str(), navigation_grid);
		        //save_matr((results_folder+"navigation_map_"+to_string(it)+".txt").c_str(), goal_grad);
		        goal_path.save((results_folder+"goal_path_"+to_string(it)+".txt").c_str());
		        datos[0] = no; datos[1] = it;
		        save_vect((results_folder+"datos.txt").c_str(), datos);

		        it_data = iteration_data(agents_poss, goals_poss, to_goal);
		        save_vect_((results_folder+"it_data.txt").c_str(), it_data); // Insertar en el mismo fichero

		        collision_data((results_folder+"colisiones.txt"), agents_poss, obstacles, it);
		    }

            if(res.non_explored_points == 0) break;

		}

		if(results_folder.size()){
			save_matr((results_folder+"temp_map.txt").c_str(), temp_map);
		}

		vector<float> expl_res = exploration_results(temp_map, static_map);
		res.explored_mean = expl_res[0];
		res.explored_var = expl_res[1];

		return res;
	}

	// ***************************************************************************************************************************************************
	// ***************************************************************************************************************************************************
	// ***************************************************************************************************************************************************

	Result exploration_with_static_areas(Poss<int> agents_poss, vector<vector<float>> static_map, Poss<int> free_poss, fmm_segment::Segments fmm_segments, Poss<int> obstacles, vector<vector<float>> navigable_areas_map, int mission_time, int vrange, int direction_time, int direction_seed, string results_folder, float occup_thres, bool collisions)
	{
		Result res;

		Poss<int> goals_poss; goals_poss(1);
		goals_poss = agents_poss;

		// Variables de los segmentos
		vector<vector<int>> static_areas_map = fmm_segments.map;
		vector<Poss<int>> areas_poss = fmm_segments.poss;
		vector<vector<bool>> areas_graph = fmm_segments.graph;
		Poss<int> area_centroids = fmm_segments.centroids;

		int Nareas = areas_poss.size();

		res.non_explored_points = free_poss.x.size();
		vector<vector<float>> navigation_grid;

		// Variables de los obstáculos
		int no = obstacles.x.size();
		Poss<int> other_agents;
		vector<int> obstacles_direction(no, 0);
    	vector<int> direction_update(no, 0);
    	vector<vector<float>> navigable_areas_map_ini = navigable_areas_map;
    	Poss<int> obstacles_list; // Lista completa de obstáculos (los que se ven actualmente + los que se han visto previamente)
    	vector<vector<bool>> obstacles_poss_map(static_map.size(), vector<bool>(static_map[0].size(),false)); // Mapa
    	vector<vector<int>> obst_temp_map(static_map.size(), vector<int>(static_map[0].size(), 0));
	 	vector<vector<int>> _obst_temp_map = obst_temp_map;
	 	vector<vector<int>> temp_map = obst_temp_map;

	 	// Mapa "REAL" de lo que se ha visto
	 	vector<vector<int>> real_seen_map(static_map.size(), vector<int>(static_map[0].size(), -1));

	 	vector<int> number_of_obstacles, number_of_obstacles_real;
	 	int max_obst = 0;

		// Inicializar el grid "real"
		vector<vector<float>> world_grid;
		world_grid = static_map;
		for(int i=0; i<obstacles.x.size(); i++) world_grid[obstacles.x[i]][obstacles.y[i]] = 0;

		// Inicializar lo que observa el agente
		Poss<int> visible_poss, visible_obstacles, local_map_poss;
		vector<vector<float>> global_map, local_map;
		global_map = static_map;
		navigation::update_agent_information_no_infl(agents_poss.x[0], agents_poss.y[0], world_grid, static_map, global_map, local_map, local_map_poss, visible_poss, visible_obstacles, vrange);

	    // Las variables para ello
	    vector<vector<bool>> trajectories_map, real_trajectories_map;
	    trajectories_map = real_trajectories_map = obstacles_poss_map;
	    vector<Poss<int>> clusters, real_clusters;
	    Poss<int> obstacles_trajectories_points, real_obstacles_trajectories_points;
	    int number_of_visited_points = 0, real_number_of_visited_points = 0;
	    
	    // Inicializar las variables de los obstáculos que observa el agente
	    dynamic_areas::update_trajectories_map(visible_obstacles, trajectories_map, obstacles_trajectories_points, number_of_visited_points); // Obstáculos que ve el agente

	    // Inicializar las variables de los obstáculos "reales"
	    dynamic_areas::update_trajectories_map(obstacles, real_trajectories_map, real_obstacles_trajectories_points, real_number_of_visited_points); // Todos los obstáculos

	    // Variables de exploración
		//exploration::Learn expl_obj(static_map, static_map, vrange);
		exploration::Areas_exploration expl_obj(areas_poss, static_areas_map, static_map, vrange);
		vector<float> map_values; // variable que almacena los que se está viendo (obstáculo o espacio libre)
		map_values = get_values_from_map(visible_poss, world_grid); // Qué es lo que se está viendo

    	Path goal_path;
    	vector<vector<int>> tree;
    	vector<bool> dynamism(fmm_segments.centroids.x.size(),false);
    	// Obtener la distancia máxima de cada segmento
    	vector<float> max_seg_dist(fmm_segments.centroids.x.size(),0);
    	float cd;
    	for(int i=0; i<fmm_segments.centroids.x.size(); i++){
	        for(int j=0; j<fmm_segments.contour_poss[i].x.size(); j++){
	            cd = geometry::distance(fmm_segments.centroids.x[i], fmm_segments.centroids.y[i], fmm_segments.contour_poss[i].x[j], fmm_segments.contour_poss[i].y[j]);
	            if(max_seg_dist[i]<cd){
	                max_seg_dist[i] = cd;
	            }
	        }
	    }

    	// Guardar variables
    	vector<int> it_data;
    	vector<int> datos(2); datos[0] = no; datos[1] = 0;
    	bool to_goal = true;
    	if(results_folder.size()){
		    // Percibido
		    obstacles_trajectories_points.save((results_folder+"obstacles_traj_"+to_string(0)+".txt").c_str()); // Trayectorías de los obstáculos
		    //for(int i=0; i<areas.N; i++) areas.contours[i].save((results_folder+"it_"+to_string(0)+"_area_"+to_string(i)+".txt").c_str());
		    // Real
		    obstacles.save((results_folder+"obstacles_"+to_string(0)+".txt").c_str()); // Obstáculos
		    real_obstacles_trajectories_points.save((results_folder+"real_obstacles_traj_"+to_string(0)+".txt").c_str()); // Trayectorías de los obstáculos
		    //for(int i=0; i<real_areas.N; i++) real_areas.contours[i].save((results_folder+"it_"+to_string(0)+"_real_area_"+to_string(i)+".txt").c_str());

    		// Para guardar la información de los goals a cada iteración
        	//save_vect((results_folder+"datos.txt").c_str(), datos);
		    it_data = iteration_data(agents_poss, goals_poss, to_goal);
		    save_vect((results_folder+"it_data.txt").c_str(), it_data);
		}

	    // ITERAR
		for(int it=1; it<mission_time; it++){
			//cout<<"it: "<<it<<endl;
			// --------------------------------------------------------------------------------------------------------------------------------
	        // MOVIMIENTO DE LOS OBSTÁCULOS DINÁMICOS
	        //dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, other_agents, direction_time, direction_seed);
	        if(collisions)
	        	dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, other_agents, direction_time);
	        else
	        	dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, agents_poss, direction_time);
	        world_grid = static_map;
	        for(int i=0; i<obstacles.x.size(); i++) world_grid[obstacles.x[i]][obstacles.y[i]] = 0;
	        // --------------------------------------------------------------------------------------------------------------------------------

        	// --------------------------------------------------------------------------------------------------------------------------------
        	// ACTUALIZACIÓN DE LAS VARIABLES DEL MOVIMIENTO DE "TODOS" LOS OBSTÁCULOS DINÁMICOS
		    dynamic_areas::update_trajectories_map(obstacles, real_trajectories_map, real_obstacles_trajectories_points, real_number_of_visited_points);
		    // --------------------------------------------------------------------------------------------------------------------------------

		    // --------------------------------------------------------------------------------------------------------------------------------
		    // ACTUALIZACIÓN DE LAS VARIABLES DEL MOVIMIENTO DE LOS OBSTÁCULOS DINÁMICOS "VISIBLES"
	        // Actualizar de lo que está viendo el agente
	        navigation::update_agent_information_no_infl(agents_poss.x[0], agents_poss.y[0], world_grid, static_map, global_map, local_map, local_map_poss, visible_poss, visible_obstacles, vrange);
	       	
	        dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, global_map, static_map);
	        for(int i=0; i<visible_obstacles.x.size(); i++) obst_temp_map[visible_obstacles.x[i]][visible_obstacles.y[i]]++;
        	dynamic_areas::update_trajectories_map(obstacles_list, trajectories_map, obstacles_trajectories_points, number_of_visited_points);
	        // --------------------------------------------------------------------------------------------------------------------------------

	        // --------------------------------------------------------------------------------------------------------------------------------
	        // ACTUALIZAR LAS VARIABLES DE LAS ÁREAS DINÁMICAS
	        number_of_obstacles = graph_navigation::number_of_obstacles_in_segments_zs(obstacles_list, static_areas_map, Nareas);
	        number_of_obstacles_real = number_of_obstacles;
	        max_obst = 0;
	        for(int i=0; i<Nareas; i++){
	        	if(max_obst < number_of_obstacles[i]) max_obst = number_of_obstacles[i];
	        	number_of_obstacles[i] = number_of_obstacles[i] ? number_of_obstacles[i] : 1; // Para que las zonas que no se ven tengan menos prioridad que espacio vacío
	        }

	        // --------------------------------------------------------------------------------------------------------------------------------

	        // --------------------------------------------------------------------------------------------------------------------------------
	        // MOVER AL AGENTE
	        map_values = get_values_from_map(visible_poss, world_grid);
        	// Actualizar las variables de exploración (mapas basicamente)
        	expl_obj.update_all(visible_poss, map_values);
	        // Obtener el mapa para la "navegación"
        	//navigation_grid = dynamic_areas::navigable_grid(static_map, free_poss, obstacles_list, areas_poss, number_of_obstacles);
        	navigation_grid = dynamic_areas::navigable_grid_occup_thres(static_map, free_poss, obstacles_list, areas_poss, number_of_obstacles_real, occup_thres, static_areas_map[agents_poss.x[0]][agents_poss.y[0]]);
        	// Añadir tambien los obstáculos ("VISIBLES") DENTRO DEL GRID
	        for(int i=0; i<visible_obstacles.x.size(); i++){
	        	navigation_grid[visible_obstacles.x[i]][visible_obstacles.y[i]] = 0;
	        	if(!dynamism[fmm_segments.map[visible_obstacles.x[i]][visible_obstacles.y[i]]]) dynamism[fmm_segments.map[visible_obstacles.x[i]][visible_obstacles.y[i]]] = true;
	        }

        	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        	// SELECCIÓN DE LOS NUEVOS OBJETIVOS
    		// Reiniciar los objetivos
    		expl_obj.update_targets();
	    	// Todo en una misma función
    		//expl_obj.targets_and_path(agents_poss.x[0], agents_poss.y[0], areas_graph, fmm_segments.centroids, navigation_grid, dynamism, max_seg_dist, vrange, goal_path, tree);
    		expl_obj.targets_and_path(agents_poss.x[0], agents_poss.y[0], areas_graph, fmm_segments.centroids, navigation_grid, max_seg_dist, vrange, goal_path, tree);

    		if(!goal_path.tam){
    			vector<bool> dynamism_in_stareas; // Dinamismo en las áreas estáticas (zonas en las que está dividido el entorno)
    			dynamism_in_stareas = dynamic_areas::in_area(obstacles_list, static_areas_map, Nareas);
    			expl_obj.update_poss_with_traversability(agents_poss.x[0], agents_poss.y[0], navigation_grid, dynamism_in_stareas);
    			expl_obj.targets_and_path(agents_poss.x[0], agents_poss.y[0], areas_graph, fmm_segments.centroids, navigation_grid, max_seg_dist, vrange, goal_path, tree);
    		}

			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        	// Mover el agente
        	if(goal_path.tam){
		        goals_poss.x[0] = goal_path.x[goal_path.tam-1]; goals_poss.y[0] = goal_path.y[goal_path.tam-1];
		        navigation::move_agent_along_path(agents_poss.x[0], agents_poss.y[0], goal_path, navigation_grid);
		    }else{
		    	//cout<<"No existe camino"<<endl;
		    	//if(!res.non_explored_points){
			    	//res.mission_time = it;
			        //mission_time = 0;
			    //}

			    res.mission_time = it;
		        mission_time = 0;
		    }
	        // --------------------------------------------------------------------------------------------------------------------------------

        	// Actualizar la cantidad de puntos que quedan por explorar
	        for(int i=0; i<visible_poss.x.size(); i++){
	            if(static_map[visible_poss.x[i]][visible_poss.y[i]]){
	                if(temp_map[visible_poss.x[i]][visible_poss.y[i]] == 0) res.non_explored_points--;
	                temp_map[visible_poss.x[i]][visible_poss.y[i]]++;
	            }
	            real_seen_map[visible_poss.x[i]][visible_poss.y[i]] = map_values[i];
	        }

            // Contabilizar las colisiones
		    for(int i=0; i<no; i++){
		        if(agents_poss.x[0] == obstacles.x[i] && agents_poss.y[0] == obstacles.y[i]) res.collisions++;
		        if(sqrt(pow(agents_poss.x[0] - obstacles.x[i],2) + pow(agents_poss.y[0] - obstacles.y[i],2)) <= sqrt(2)) res.invasions++;
		    }
		    // Contabilizar las invasiones
		    if(navigable_areas_map_ini[agents_poss.x[0]][agents_poss.y[0]]) res.intrusions++;

            // Actualizar el tiempo de la misión
            res.mission_time = it;

			// Guardar
			if(results_folder.size()){
		        real_obstacles_trajectories_points.save((results_folder+"real_obstacles_traj_"+to_string(it)+".txt").c_str()); // Trayectorías de los obstáculos
		        //for(int i=0; i<real_areas.N; i++) real_areas.contours[i].save((results_folder+"it_"+to_string(it)+"_real_area_"+to_string(i)+".txt").c_str());
		        obstacles.save((results_folder+"obstacles_"+to_string(it)+".txt").c_str()); // Obstáculos
		        obstacles_trajectories_points.save((results_folder+"obstacles_traj_"+to_string(it)+".txt").c_str()); // Trayectorías de los obstáculos
		        //save_matr((results_folder+"temporal_map_"+to_string(it)+".txt").c_str(), expl_obj.get_seen_map());
		        save_matr((results_folder+"temporal_map_"+to_string(it)+".txt").c_str(), real_seen_map);
		        save_matr((results_folder+"navigation_map_"+to_string(it)+".txt").c_str(), navigation_grid);
		        //save_matr((results_folder+"navigation_map_"+to_string(it)+".txt").c_str(), goal_grad);
		        goal_path.save((results_folder+"goal_path_"+to_string(it)+".txt").c_str());
		        //for(int i=0; i<areas.N; i++)
		        //    areas.contours[i].save((results_folder+"it_"+to_string(it)+"_contour_area_"+to_string(i)+".txt").c_str());
		        datos[0] = no; datos[1] = it;
		        save_vect((results_folder+"datos.txt").c_str(), datos);

		        it_data = iteration_data(agents_poss, goals_poss, to_goal);
		        save_vect_((results_folder+"it_data.txt").c_str(), it_data); // Insertar en el mismo fichero

		        collision_data((results_folder+"colisiones.txt"), agents_poss, obstacles, it);
		    }

            if(res.non_explored_points == 0) break;

		}

		if(results_folder.size()){
			save_matr((results_folder+"temp_map.txt").c_str(), temp_map);
		}

		vector<float> expl_res = exploration_results(temp_map, static_map);
		res.explored_mean = expl_res[0];
		res.explored_var = expl_res[1];

		return res;
	}

	// ***************************************************************************************************************************************************
	// ***************************************************************************************************************************************************
	// ***************************************************************************************************************************************************

	bool compare_poss(Poss<int> p1, Poss<int> p2, vector<vector<float>> simple_grid)
	{
		if(p1.x.size() != p2.x.size()) return false;
		for(int i=0; i<p1.x.size(); i++) simple_grid[p1.x[i]][p1.y[i]] = -2;
		for(int i=0; i<p2.x.size(); i++)
			if(simple_grid[p2.x[i]][p2.y[i]] != -2) return false;
		return true;
	}

	Poss<int> compare_poss(Poss<int> p1, Poss<int> p2)
	{
		Poss<int> res;
		vector<bool> b1(p1.x.size(), false), b2(p2.x.size(), false);
		for(int i=0; i<p1.x.size(); i++)
			for(int j=0; j<p2.x.size(); j++)
				if(p1.x[i] == p2.x[j] && p1.y[i] == p2.y[j]){
					b1[i] = true; b2[i] = true;
				}
		for(int i=0; i<p1.x.size(); i++)
			if(!b1[i]) res.push(p1.x[i], p1.y[i]);
		for(int i=0; i<p2.x.size(); i++)
			if(!b2[i]) res.push(p2.x[i], p2.y[i]);
		return res;
	}

	Result exploration_with_dynamic_areas(Poss<int> agents_poss, vector<vector<float>> static_map, Poss<int> free_poss, fmm_segment::Segments fmm_segments, Poss<int> obstacles, vector<vector<float>> navigable_areas_map, int mission_time, int vrange, int direction_time, int direction_seed, string results_folder, float occup_thres, bool collisions)
	{
		Result res;

		Poss<int> goals_poss; goals_poss(1);
		goals_poss = agents_poss;

		// Variables de los segmentos
		vector<vector<int>> static_areas_map = fmm_segments.map;
		vector<Poss<int>> areas_poss = fmm_segments.poss;
		vector<vector<bool>> areas_graph = fmm_segments.graph;
		Poss<int> area_centroids = fmm_segments.centroids;
		int Nstatic_areas = area_centroids.x.size();

		res.non_explored_points = free_poss.x.size();
		vector<vector<float>> navigation_grid;

		// Variables de los obstáculos
		int no = obstacles.x.size();
		Poss<int> other_agents;
		vector<int> obstacles_direction(no, 0);
    	vector<int> direction_update(no, 0);
    	vector<vector<float>> navigable_areas_map_ini = navigable_areas_map;
    	Poss<int> obstacles_list; // Lista completa de obstáculos (los que se ven actualmente + los que se han visto previamente)
    	vector<vector<bool>> obstacles_poss_map(static_map.size(), vector<bool>(static_map[0].size(),false)); // Mapa
    	vector<vector<int>> obst_temp_map(static_map.size(), vector<int>(static_map[0].size(), 0));
	 	vector<vector<int>> _obst_temp_map = obst_temp_map;
	 	vector<vector<int>> temp_map = obst_temp_map;

	 	// Mapa "REAL" de lo que se ha visto
	 	vector<vector<int>> real_seen_map(static_map.size(), vector<int>(static_map[0].size(), -1));

	 	vector<int> number_of_obstacles, number_of_obstacles_real;
	 	int max_obst = 0;

		// Inicializar el grid "real"
		vector<vector<float>> world_grid;
		world_grid = static_map;
		for(int i=0; i<obstacles.x.size(); i++) world_grid[obstacles.x[i]][obstacles.y[i]] = 0;

		// Inicializar lo que observa el agente
		Poss<int> visible_poss, visible_obstacles, local_map_poss;
		vector<vector<float>> global_map, local_map;
		global_map = static_map;
		navigation::update_agent_information_no_infl(agents_poss.x[0], agents_poss.y[0], world_grid, static_map, global_map, local_map, local_map_poss, visible_poss, visible_obstacles, vrange);
		
		//Poss<int> visible_poss2, visible_obstacles2;
		//visible_poss2 = visibility::obtain_visible_poss(agents_poss.x[0], agents_poss.y[0], world_grid, vrange);
		//visible_obstacles2 = navigation::obtain_new_obstacles(visible_poss, world_grid, static_map);

		// Áreas que obtiene el agente
	    dynamic_areas::Areas areas, real_areas;
	    dynamic_areas::Areas_information areas_information, real_areas_information;
	    // Las variables para ello
	    vector<vector<bool>> trajectories_map, real_trajectories_map;
	    trajectories_map = real_trajectories_map = obstacles_poss_map;
	    vector<Poss<int>> clusters, real_clusters;
	    Poss<int> obstacles_trajectories_points, real_obstacles_trajectories_points;
	    int number_of_visited_points = 0, real_number_of_visited_points = 0;
	    
	    // Inicializar las variables de los obstáculos que observa el agente
	    dynamic_areas::update_trajectories_map(visible_obstacles, trajectories_map, obstacles_trajectories_points, number_of_visited_points); // Obstáculos que ve el agente
	    clusters = adjacency_clustering::cluster(visible_obstacles, static_map);
	    areas = dynamic_areas::obtain_areas(clusters, static_map);

	    // Inicializar las variables de los obstáculos "reales"
	    dynamic_areas::update_trajectories_map(obstacles, real_trajectories_map, real_obstacles_trajectories_points, real_number_of_visited_points); // Todos los obstáculos
	    real_clusters = adjacency_clustering::cluster(obstacles, static_map);
	    real_areas = dynamic_areas::obtain_areas(real_clusters, static_map);

	    // Variables de exploración
		//exploration::Learn expl_obj(static_map, static_map, vrange);
		exploration::Areas_exploration expl_obj(areas_poss, static_areas_map, static_map, vrange);
		vector<float> map_values; // variable que almacena los que se está viendo (obstáculo o espacio libre)
		map_values = get_values_from_map(visible_poss, world_grid); // Qué es lo que se está viendo

    	Path goal_path;

    	vector<Poss<int>> areas_poss_infl;
    	vector<vector<int>> areas_map_infl;

    	vector<vector<int>> tree;
    	vector<bool> dynamism(fmm_segments.centroids.x.size(),false);
    	// Obtener la distancia máxima de cada segmento
    	vector<float> max_seg_dist(fmm_segments.centroids.x.size(),0);
    	float cd;
    	for(int i=0; i<fmm_segments.centroids.x.size(); i++){
	        for(int j=0; j<fmm_segments.contour_poss[i].x.size(); j++){
	            cd = geometry::distance(fmm_segments.centroids.x[i], fmm_segments.centroids.y[i], fmm_segments.contour_poss[i].x[j], fmm_segments.contour_poss[i].y[j]);
	            if(max_seg_dist[i]<cd){
	                max_seg_dist[i] = cd;
	            }
	        }
	    }

    	// Guardar variables
    	vector<int> it_data;
    	vector<int> datos(2); datos[0] = no; datos[1] = 0;
    	bool to_goal = true;
    	if(results_folder.size()){
		    // Percibido
		    obstacles_trajectories_points.save((results_folder+"obstacles_traj_"+to_string(0)+".txt").c_str()); // Trayectorías de los obstáculos
		    for(int i=0; i<areas.N; i++) areas.contours[i].save((results_folder+"it_"+to_string(0)+"_area_"+to_string(i)+".txt").c_str());
		    // Real
		    obstacles.save((results_folder+"obstacles_"+to_string(0)+".txt").c_str()); // Obstáculos
		    real_obstacles_trajectories_points.save((results_folder+"real_obstacles_traj_"+to_string(0)+".txt").c_str()); // Trayectorías de los obstáculos
		    for(int i=0; i<real_areas.N; i++) real_areas.contours[i].save((results_folder+"it_"+to_string(0)+"_real_area_"+to_string(i)+".txt").c_str());

    		// Para guardar la información de los goals a cada iteración
        	//save_vect((results_folder+"datos.txt").c_str(), datos);
		    it_data = iteration_data(agents_poss, goals_poss, to_goal);
		    save_vect((results_folder+"it_data.txt").c_str(), it_data);
		}

	    // ITERAR
		for(int it=1; it<mission_time; it++){
			//cout<<"it: "<<it<<endl;
			// --------------------------------------------------------------------------------------------------------------------------------
	        // MOVIMIENTO DE LOS OBSTÁCULOS DINÁMICOS
	        //dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, other_agents, direction_time, direction_seed);
	        if(collisions)
	        	dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, other_agents, direction_time);
	        else
	        	dynamic_fmm::random_movement(obstacles, navigable_areas_map, obstacles_direction, direction_update, agents_poss, direction_time);
	        world_grid = static_map;
	        for(int i=0; i<obstacles.x.size(); i++) world_grid[obstacles.x[i]][obstacles.y[i]] = 0;
	        // --------------------------------------------------------------------------------------------------------------------------------

        	// --------------------------------------------------------------------------------------------------------------------------------
        	// ACTUALIZACIÓN DE LAS VARIABLES DEL MOVIMIENTO DE "TODOS" LOS OBSTÁCULOS DINÁMICOS
		    dynamic_areas::update_trajectories_map(obstacles, real_trajectories_map, real_obstacles_trajectories_points, real_number_of_visited_points);
		    // Clusterizar trayectorias
		    real_clusters = adjacency_clustering::cluster(real_obstacles_trajectories_points, static_map);
		    // Obtener áreas
		    real_areas = dynamic_areas::obtain_areas(real_clusters, static_map);
		    // --------------------------------------------------------------------------------------------------------------------------------

		    // --------------------------------------------------------------------------------------------------------------------------------
		    // ACTUALIZACIÓN DE LAS VARIABLES DEL MOVIMIENTO DE LOS OBSTÁCULOS DINÁMICOS "VISIBLES"
	        // Actualizar de lo que está viendo el agente
	        navigation::update_agent_information_no_infl(agents_poss.x[0], agents_poss.y[0], world_grid, static_map, global_map, local_map, local_map_poss, visible_poss, visible_obstacles, vrange);

	        dynamic_areas::update_obstacles_list(obstacles_list, obstacles_poss_map, visible_obstacles, visible_poss, global_map, static_map);
	        for(int i=0; i<visible_obstacles.x.size(); i++) obst_temp_map[visible_obstacles.x[i]][visible_obstacles.y[i]]++;
        	dynamic_areas::update_trajectories_map(obstacles_list, trajectories_map, obstacles_trajectories_points, number_of_visited_points);
	        // Agrupar trayectorias
        	clusters = adjacency_clustering::cluster(obstacles_trajectories_points, static_map);
        	// Obtener las áreas
	        areas = dynamic_areas::obtain_areas(clusters, static_map);
	        // --------------------------------------------------------------------------------------------------------------------------------

	        // --------------------------------------------------------------------------------------------------------------------------------
	        // ACTUALIZAR LAS VARIABLES DE LAS ÁREAS DINÁMICAS
	        number_of_obstacles = graph_navigation::number_of_obstacles_in_segments_zs(obstacles_list, areas.map, areas.N);
	        number_of_obstacles_real = number_of_obstacles;
	        max_obst = 0;
	        for(int i=0; i<areas.N; i++){
	        	if(max_obst < number_of_obstacles[i]) max_obst = number_of_obstacles[i];
	        	number_of_obstacles[i] = number_of_obstacles[i] ? number_of_obstacles[i] : 1; // Para que las zonas que no se ven tengan menos prioridad que espacio vacío
	        }
	        // --------------------------------------------------------------------------------------------------------------------------------

	        // --------------------------------------------------------------------------------------------------------------------------------
	        // MOVER AL AGENTE
	        map_values = get_values_from_map(visible_poss, world_grid);
        	// Actualizar las variables de exploración (mapas basicamente)
        	expl_obj.update_all(visible_poss, map_values);

        	// Inflar las áreas dinámicas
        	//areas_map_infl = areas.map;
        	//areas_poss_infl = areas.poss;
        	//dynamic_areas::inflate_areas(areas_poss_infl, areas.contours_edges, areas_map_infl);
	        // Obtener el mapa para la "navegación"
        	//navigation_grid = dynamic_areas::navigable_grid(static_map, free_poss, obstacles_list, areas.poss, number_of_obstacles);
        	navigation_grid = dynamic_areas::navigable_grid_occup_thres(static_map, free_poss, obstacles_list, areas.poss, number_of_obstacles, occup_thres, areas.map[agents_poss.x[0]][agents_poss.y[0]]);
        	//navigation_grid = dynamic_areas::navigable_grid_infl(static_map, free_poss, obstacles_list, areas_poss, number_of_obstacles);
        	// Añadir tambien los obstáculos ("VISIBLES") DENTRO DEL GRID
	        for(int i=0; i<visible_obstacles.x.size(); i++){
	        	navigation_grid[visible_obstacles.x[i]][visible_obstacles.y[i]] = 0;
	        	if(!dynamism[fmm_segments.map[visible_obstacles.x[i]][visible_obstacles.y[i]]]) dynamism[fmm_segments.map[visible_obstacles.x[i]][visible_obstacles.y[i]]] = true;
	        }

        	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        	// SELECCIÓN DE LOS NUEVOS OBJETIVOS
    		// Reiniciar los objetivos
    		expl_obj.update_targets();
	    	// Todo en una misma función
    		//expl_obj.targets_and_path(agents_poss.x[0], agents_poss.y[0], areas_graph, fmm_segments.centroids, navigation_grid, dynamism, max_seg_dist, vrange, goal_path, tree);
    		expl_obj.targets_and_path(agents_poss.x[0], agents_poss.y[0], areas_graph, fmm_segments.centroids, navigation_grid, max_seg_dist, vrange, goal_path, tree);

    		if(!goal_path.tam){
    			vector<bool> dynamism_in_stareas; // Dinamismo en áreas dinámicas
    			dynamism_in_stareas = dynamic_areas::in_area(areas.poss, static_areas_map, Nstatic_areas);
    			expl_obj.update_poss_with_traversability(agents_poss.x[0], agents_poss.y[0], navigation_grid, dynamism_in_stareas);
    			expl_obj.targets_and_path(agents_poss.x[0], agents_poss.y[0], areas_graph, fmm_segments.centroids, navigation_grid, max_seg_dist, vrange, goal_path, tree);
    		}
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        	// Mover el agente
        	if(goal_path.tam){
		        goals_poss.x[0] = goal_path.x[goal_path.tam-1]; goals_poss.y[0] = goal_path.y[goal_path.tam-1];
		        navigation::move_agent_along_path(agents_poss.x[0], agents_poss.y[0], goal_path, navigation_grid);
		    }else{
		    	//cout<<"No existe camino"<<endl;
		    	//if(!res.non_explored_points){
			    //	res.mission_time = it;
			      //  mission_time = 0;
			    //}

			    res.mission_time = it;
		        mission_time = 0;
		    }
	        // --------------------------------------------------------------------------------------------------------------------------------

        	// Actualizar la cantidad de puntos que quedan por explorar
	        for(int i=0; i<visible_poss.x.size(); i++){
	            if(static_map[visible_poss.x[i]][visible_poss.y[i]]){
	                if(temp_map[visible_poss.x[i]][visible_poss.y[i]] == 0) res.non_explored_points--;
	                temp_map[visible_poss.x[i]][visible_poss.y[i]]++;
	            }
	            real_seen_map[visible_poss.x[i]][visible_poss.y[i]] = map_values[i];
	        }

            // Contabilizar las colisiones
		    for(int i=0; i<no; i++){
		        if(agents_poss.x[0] == obstacles.x[i] && agents_poss.y[0] == obstacles.y[i]) res.collisions++;
		        if(sqrt(pow(agents_poss.x[0] - obstacles.x[i],2) + pow(agents_poss.y[0] - obstacles.y[i],2)) <= sqrt(2)) res.invasions++;
		    }
		    // Contabilizar las invasiones
		    if(navigable_areas_map_ini[agents_poss.x[0]][agents_poss.y[0]]) res.intrusions++;

            // Actualizar el tiempo de la misión
            res.mission_time = it;

			// Guardar
			if(results_folder.size()){
		        real_obstacles_trajectories_points.save((results_folder+"real_obstacles_traj_"+to_string(it)+".txt").c_str()); // Trayectorías de los obstáculos
		        for(int i=0; i<real_areas.N; i++) real_areas.contours[i].save((results_folder+"it_"+to_string(it)+"_real_area_"+to_string(i)+".txt").c_str());
		        obstacles.save((results_folder+"obstacles_"+to_string(it)+".txt").c_str()); // Obstáculos
		        obstacles_trajectories_points.save((results_folder+"obstacles_traj_"+to_string(it)+".txt").c_str()); // Trayectorías de los obstáculos
			    
		        //save_matr((results_folder+"temporal_map_"+to_string(it)+".txt").c_str(), expl_obj.get_seen_map());
		        save_matr((results_folder+"temporal_map_"+to_string(it)+".txt").c_str(), real_seen_map);
		        save_matr((results_folder+"navigation_map_"+to_string(it)+".txt").c_str(), navigation_grid);
		        goal_path.save((results_folder+"goal_path_"+to_string(it)+".txt").c_str());
		        for(int i=0; i<areas.N; i++)
		            areas.contours[i].save((results_folder+"it_"+to_string(it)+"_contour_area_"+to_string(i)+".txt").c_str());
		        
		        // Formar las aristas
		        if(tree.size()){
		        	vector<Poss<int>> tree_poss = tree_functions::form_tree_pos(fmm_segments.centroids, tree);

		        	vector<int> tree_vector(4,0);
		        	for(int i=0; i<tree_poss.size(); i++){
		        		// Vector de las posiciones
		        		tree_vector[0] = tree_poss[i].x[0]; tree_vector[1] = tree_poss[i].y[0];
		        		tree_vector[2] = tree_poss[i].x[1]; tree_vector[3] = tree_poss[i].y[1];
		        		// Guardar
		        		save_vect_((results_folder+"tree_poss_"+to_string(it)+".txt").c_str(), tree_vector); // Insertar en el mismo fichero
		        	}

		        	tree_functions::save(tree_poss, (results_folder+"tree_poss_"+to_string(it)+".txt"));
		        	
		        }

		        datos[0] = no; datos[1] = it;
		        save_vect((results_folder+"datos.txt").c_str(), datos);

		        it_data = iteration_data(agents_poss, goals_poss, to_goal);
		        save_vect_((results_folder+"it_data.txt").c_str(), it_data); // Insertar en el mismo fichero

		        collision_data((results_folder+"colisiones.txt"), agents_poss, obstacles, it);

		        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		        // SOLO PARA LAS IMAGENES
		        vector<vector<float>> im_gradient = compute_gradient(agents_poss.x[0], agents_poss.y[0], navigation_grid);
		        save_matr((results_folder+"gradient_"+to_string(it)+".txt").c_str(), im_gradient);

		        Poss<int> expl_poss2save = join_poss(expl_obj.get_target_poss());
		        expl_poss2save.save((results_folder+"explor_poss_"+to_string(it)+".txt").c_str());
		        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		    }

            if(res.non_explored_points == 0) break;
            
            //cout<<"dale"<<endl; cin.get();

		}

		if(results_folder.size()){
			save_matr((results_folder+"temp_map.txt").c_str(), temp_map);
		}

		vector<float> expl_res = exploration_results(temp_map, static_map);
		res.explored_mean = expl_res[0];
		res.explored_var = expl_res[1];

		return res;
	}

	void show_results(Result resultados)
	{
		cout<<"it: "<<resultados.mission_time<<endl;
	    if(resultados.non_explored_points) cout<<resultados.non_explored_points<<" puntos por explorar <----------------------------"<<endl;
	    else cout<<resultados.non_explored_points<<" puntos por explorar"<<endl;
	    cout<<resultados.collisions<<" colisiones"<<endl;
	    cout<<resultados.invasions<<" invasiones"<<endl;
	    cout<<resultados.intrusions<<" intrusiones"<<endl;
	    cout<<"exploracion -> mean: "<<resultados.explored_mean<<" | var: "<<resultados.explored_var<<endl;
	}

}


#endif