// Funciones para las áreas dinámicas

#ifndef DYNAMIC_AREAS_HPP
#define DYNAMIC_AREAS_HPP

#include <mada/util/fmm_2.hpp>
#include <mada/util/cluster.hpp>

namespace dynamic_areas{

	vector<bool> in_area(Poss<int> positions, vector<vector<int>> areas_map, int nareas)
	{
		vector<bool> res(nareas, false);
		for(int i=0; i<positions.x.size(); i++){
			if(areas_map[positions.x[i]][positions.y[i]] >= 0)
			if(!res[areas_map[positions.x[i]][positions.y[i]]]){
				res[areas_map[positions.x[i]][positions.y[i]]] = true;
				nareas--;
			}
			if(!nareas) break;
		}
		return res;
	}

	vector<bool> in_area(vector<Poss<int>> positions, vector<vector<int>> areas_map, int nareas)
	{
		vector<bool> res(nareas, false);
		for(int i=0; i<positions.size(); i++){
			for(int j=0; j<positions[i].x.size(); j++){
				if(areas_map[positions[i].x[j]][positions[i].y[j]] >= 0)
				if(!res[areas_map[positions[i].x[j]][positions[i].y[j]]]){
					res[areas_map[positions[i].x[j]][positions[i].y[j]]] = true;
					nareas--;
				}
				if(!nareas) break;
			}
			if(!nareas) break;
		}
		return res;
	}

	vector<float> areasMeanDist(vector<Poss<int>> areas_poss, vector<vector<int>> areas_map, vector<vector<float>> distGrad)
	{
	    vector<float> res(areas_poss.size(), 0);
	    for(int i=0; i<areas_poss.size(); i++){
	        for(int j=0; j<areas_poss[i].x.size(); j++){
	            if(areas_map[areas_poss[i].x[j]][areas_poss[i].y[j]] >= 0){
	                res[i] += distGrad[areas_poss[i].x[j]][areas_poss[i].y[j]];
	            }
	        }
	        res[i] /= areas_poss[i].x.size();
	    }
	    return res;
	}

	vector<float> areasMeanDist(Poss<int> poss, int Nareas, vector<vector<int>> areas_map, vector<vector<float>> distGrad)
	{
	    vector<float> res(Nareas, 0);
	    vector<int> N(Nareas, 0);
	    for(int i=0; i<poss.x.size(); i++){
	    	if(areas_map[poss.x[i]][poss.y[i]] >= 0){
	    		res[areas_map[poss.x[i]][poss.y[i]]] += distGrad[poss.x[i]][poss.y[i]];
	    		N[areas_map[poss.x[i]][poss.y[i]]]++;
	    	}
	    }
	    for(int i=0; i<Nareas; i++)
	    	if(N[i]) res[i]/=N[i];
	    return res;
	}

	vector<float> areasMeanDist(Poss<int> poss, vector<Poss<int>> areas_poss, vector<vector<int>> areas_map, vector<vector<float>> distGrad)
	{
	    vector<float> res(areas_poss.size(), 0);
	    for(int i=0; i<poss.x.size(); i++){
	    	if(areas_map[poss.x[i]][poss.y[i]] >= 0){
	    		res[areas_map[poss.x[i]][poss.y[i]]] += distGrad[poss.x[i]][poss.y[i]];
	    	}
	    }
	    for(int i=0; i<areas_poss.size(); i++)
	    	res[i]/=areas_poss[i].x.size();
	    return res;
	}

	vector<float> areasMaxDist(Poss<int> poss, vector<Poss<int>> areas_poss, vector<vector<int>> areas_map, vector<vector<float>> distGrad)
	{
	    vector<float> res(areas_poss.size(), 0);
	    for(int i=0; i<poss.x.size(); i++){
	    	if(areas_map[poss.x[i]][poss.y[i]] >= 0 && res[areas_map[poss.x[i]][poss.y[i]]] < distGrad[poss.x[i]][poss.y[i]]){
	    		res[areas_map[poss.x[i]][poss.y[i]]] = distGrad[poss.x[i]][poss.y[i]];
	    	}
	    }
	    return res;
	}

	Path pat2closest_free_area(int x, int y, vector<vector<float>> navigable_grid, vector<Poss<int>> areas_poss, vector<bool> occupied)
	{
		Path res;
		Poss<int> poss;
		for(int i=0; i<occupied.size(); i++)
			if(!occupied[i]) poss.append(areas_poss[i]);
		Poss<int> destination;
		FMM gr(x, y, navigable_grid);
		vector<vector<float>> grad = gr.pos_coverage(1, poss, destination);
		if(destination.x.size()){
			res.gradient_descent_(grad, destination.x[0], destination.x[0]);
		}
		return res;
	}

	void obstaclesFixInMap(Poss<int> &pos, vector<vector<float>> map)
	{
		int i = 0;
		while(i<pos.x.size()){
			if(!check_pos_in_grid(pos.x[i], pos.y[i], map.size(), map[0].size()) || map[pos.x[i]][pos.y[i]] == 0){
				pos.erase(i);
			}else{
				i++;
			}
		}
	}

	void update_trajectories_map(Poss<int> pos, vector<vector<bool>> &map, Poss<int> &traj_points)
	{
	    for(int i=0; i<pos.x.size(); i++)
	        if(map[pos.x[i]][pos.y[i]] == false){
	            traj_points.x.push_back(pos.x[i]);
	            traj_points.y.push_back(pos.y[i]);
	            map[pos.x[i]][pos.y[i]] = true;
	        }
	}

	void update_trajectories_map(Poss<int> pos, vector<vector<bool>> &map, Poss<int> &traj_points, int &nvisited_points)
	{
	    for(int i=0; i<pos.x.size(); i++)
	        if(map[pos.x[i]][pos.y[i]] == false){
	            traj_points.x.push_back(pos.x[i]);
	            traj_points.y.push_back(pos.y[i]);
	            map[pos.x[i]][pos.y[i]] = true;
	            nvisited_points++;
	        }
	}

	void update_trajectories_map(Poss<int> pos, float time, vector<vector<float>> &seen_pos_map, Poss<int> &seen_points, vector<float> &seen_times)
	{
	    for(int i=0; i<pos.x.size(); i++){
	        if(seen_pos_map[pos.x[i]][pos.y[i]] < 0){ // El punto no se había visto previamente, insertar
	            seen_points.x.push_back(pos.x[i]);
	            seen_points.y.push_back(pos.y[i]);
	            seen_times.push_back(time);
	        }else{ // El punto ya se havisto antes
	        	for(int j=0; j<seen_points.x.size(); j++){ // Encontrar en la lista de puntos vistos
	        		if(seen_points.x[j] == pos.x[i] && seen_points.y[j] == pos.y[i]){
	        			seen_times[j] = time; // Actualizar el instante de observación del punto
	        		}
	        	}
	        }
	        seen_pos_map[pos.x[i]][pos.y[i]] = time;
	    }
	}

	void update_trajectories_map(Poss<int> pos, vector<vector<bool>> &map, Poss<int> &traj_points, vector<vector<int>> areas_map, vector<Poss<int>> &traj_points_pa)
	{
	    for(int i=0; i<pos.x.size(); i++)
	        if(map[pos.x[i]][pos.y[i]] == false && areas_map[pos.x[i]][pos.y[i]] >= 0){
	            traj_points.x.push_back(pos.x[i]);
	            traj_points.y.push_back(pos.y[i]);
	            map[pos.x[i]][pos.y[i]] = true;

	            traj_points_pa[areas_map[pos.x[i]][pos.y[i]]].x.push_back(pos.x[i]);
	            traj_points_pa[areas_map[pos.x[i]][pos.y[i]]].y.push_back(pos.y[i]);
	        }
	}

	void update_obstacles_list(Poss<int> &obstacles_list, vector<vector<bool>> &obstacles_poss_map, Poss<int> visible_obstacles, Poss<int> visible_poss, vector<vector<float>> global_map, vector<vector<float>> static_map)
	{
	    // - obstacles_list: lista de las posiciones de todos los obstáculos
	    // - su posición sobre el mapa
	    // - visible_obstacles: los obstáculos que ve el agente actualmente
	    // - global_map: mapa global (con los obstáculos visibles ya incluidos)
	    // - static_map: mapa con los obstáculos estáticos

	    for(int i=0; i<visible_poss.x.size(); i++){
	        if(static_map[visible_poss.x[i]][visible_poss.y[i]] && global_map[visible_poss.x[i]][visible_poss.y[i]] && obstacles_poss_map[visible_poss.x[i]][visible_poss.y[i]]){ // Era un obstáculo y ahora no lo es
	            // Quitar de la lista
	            obstacles_poss_map[visible_poss.x[i]][visible_poss.y[i]] = false;
	            obstacles_list.erase(obstacles_list.find(visible_poss.x[i], visible_poss.y[i]));
	        }else if(static_map[visible_poss.x[i]][visible_poss.y[i]] && !global_map[visible_poss.x[i]][visible_poss.y[i]] && obstacles_poss_map[visible_poss.x[i]][visible_poss.y[i]] == false){ // NO era un obstáculo y ahora lo es
	            // Insertar en la lista
	            obstacles_poss_map[visible_poss.x[i]][visible_poss.y[i]] = true;
	            obstacles_list.push(visible_poss.x[i], visible_poss.y[i]);
	            if(!static_map[visible_poss.x[i]][visible_poss.y[i]]){
	            	cout<<"PUES SI LO AÑADO AQUÍ como punto visible"<<endl;
	            }
	        }
	    }

	    for(int i=0; i<visible_obstacles.x.size(); i++){
	        if(static_map[visible_poss.x[i]][visible_poss.y[i]] && !obstacles_poss_map[visible_obstacles.x[i]][visible_obstacles.y[i]]){
	            obstacles_poss_map[visible_obstacles.x[i]][visible_obstacles.y[i]] = true;
	            obstacles_list.push(visible_obstacles.x[i], visible_obstacles.y[i]);
	            if(!static_map[visible_poss.x[i]][visible_poss.y[i]]){
	            	cout<<"PUES SI LO AÑADO AQUÍ como obstáculo"<<endl;
	            }
	        }
	    }
	} // Actualizar la lista de obstáculos en el entorno (los que se ven actualmente y los que se han visto antes)

	void update_obstacles_list_wm(Poss<int> &obstacles_list, vector<vector<bool>> &obstacles_poss_map, vector<vector<float>> &movement, Poss<int> visible_obstacles, Poss<int> visible_poss, vector<vector<float>> global_map, vector<vector<float>> static_map)
	{
	    // - obstacles_list: lista de las posiciones de todos los obstáculos
	    // - su posición sobre el mapa
	    // - movement: vector con la orientación del agente y el tiempo que lleva con esa orientación
	    // - visible_obstacles: los obstáculos que ve el agente actualmente
	    // - global_map: mapa global (con los obstáculos visibles ya incluidos)
	    // - static_map: mapa con los obstáculos estáticos

	    for(int i=0; i<visible_poss.x.size(); i++){
	        if(static_map[visible_poss.x[i]][visible_poss.y[i]] && global_map[visible_poss.x[i]][visible_poss.y[i]] && obstacles_poss_map[visible_poss.x[i]][visible_poss.y[i]]){ // Era un obstáculo y ahora no lo es
	            // Quitar de la lista
	            obstacles_poss_map[visible_poss.x[i]][visible_poss.y[i]] = false;
	            obstacles_list.erase(obstacles_list.find(visible_poss.x[i], visible_poss.y[i]));
	        }else if(static_map[visible_poss.x[i]][visible_poss.y[i]] && !global_map[visible_poss.x[i]][visible_poss.y[i]] && obstacles_poss_map[visible_poss.x[i]][visible_poss.y[i]] == false){ // NO era un obstáculo y ahora lo es
	            // Insertar en la lista
	            obstacles_poss_map[visible_poss.x[i]][visible_poss.y[i]] = true;
	            obstacles_list.push(visible_poss.x[i], visible_poss.y[i]);
	        }
	    }

	    for(int i=0; i<visible_obstacles.x.size(); i++){
	        if(static_map[visible_poss.x[i]][visible_poss.y[i]] && !obstacles_poss_map[visible_obstacles.x[i]][visible_obstacles.y[i]]){
	            obstacles_poss_map[visible_obstacles.x[i]][visible_obstacles.y[i]] = true;
	            obstacles_list.push(visible_obstacles.x[i], visible_obstacles.y[i]);
	        }
	    }
	} // Actualizar la lista de obstáculos en el entorno (los que se ven actualmente y los que se han visto antes) y "su movimiento"

	vector<vector<float>> poss_gradient(Poss<int> poss, float distance, vector<vector<float>> grid, Poss<int> free_poss, Poss<int> obst_poss)
	{
	    vector<vector<float>>  grad;

	    FMM gr(poss.x, poss.y, grid);
	    grad = gr.compute_gradient_();
	    if(distance)
	    for(int i=0; i<free_poss.x.size(); i++)
	        grad[free_poss.x[i]][free_poss.y[i]] = grad[free_poss.x[i]][free_poss.y[i]] > distance ? distance : grad[free_poss.x[i]][free_poss.y[i]];

	    // Para que esos puntos sean navegables
	    for(int i=0; i<poss.x.size(); i++) grad[poss.x[i]][poss.y[i]] = 0.0001;

	    // Para los obstáculos estáticos
	    for(int i=0; i<obst_poss.x.size(); i++) grad[obst_poss.x[i]][obst_poss.y[i]] = 0;

	    return grad;
	} // Gradiente para evitar un conjunto de puntos

	vector<vector<float>> countour_gradient_vertices(vector<Poss<int>> contours, float distance, vector<vector<float>> grid, Poss<int> free_poss, Poss<int> obst_poss)
	{
	    vector<vector<float>>  grad;

	    Poss<int> pos;
	    for(int i=0; i<contours.size()-1; i++)
	        pos.append(contours[i]);

	    FMM gr(pos.x, pos.y, grid);

	    grad = gr.compute_gradient_();
	    if(distance)
	    for(int i=0; i<free_poss.x.size(); i++)
	        grad[free_poss.x[i]][free_poss.y[i]] = grad[free_poss.x[i]][free_poss.y[i]] > distance ? distance : grad[free_poss.x[i]][free_poss.y[i]];

	    // Para que esos puntos sean navegables
	    for(int i=0; i<pos.x.size(); i++) grad[pos.x[i]][pos.y[i]] = 0.0001;

	    // Para los obstáculos estáticos
	    for(int i=0; i<obst_poss.x.size(); i++) grad[obst_poss.x[i]][obst_poss.y[i]] = 0;

	    return grad;
	} // Gradiente para evitar zonas (contornos), únicamente utilizando los vértices de los contornos (esquinas de los polígonos)

	vector<vector<float>> countour_gradient_edges(vector<Poss<int>> contours, float distance, vector<vector<float>> grid, Poss<int> free_poss, Poss<int> obst_poss)
	{
	    vector<vector<float>> grad;

	    Poss<int> pos;
	    for(int i=0; i<contours.size(); i++){
	        for(int j=0; j<contours[i].x.size()-1; j++){
	            pos.append(bresenham::points(contours[i].x[j], contours[i].y[j], contours[i].x[j+1], contours[i].y[j+1]));
	        }
	        pos.append(bresenham::points(contours[i].x[contours[i].x.size()-1], contours[i].y[contours[i].y.size()-1], contours[i].x[0], contours[i].y[0]));
	    }

	    FMM gr(pos.x, pos.y, grid);

	    grad = gr.compute_gradient_();
	    if(distance)
	    for(int i=0; i<free_poss.x.size(); i++)
	        grad[free_poss.x[i]][free_poss.y[i]] = grad[free_poss.x[i]][free_poss.y[i]] > distance ? distance : grad[free_poss.x[i]][free_poss.y[i]];

	    // Para que esos puntos sean navegables
	    for(int i=0; i<pos.x.size(); i++) grad[pos.x[i]][pos.y[i]] = 0.0001;

	    // Para los obstáculos estáticos
	    for(int i=0; i<obst_poss.x.size(); i++) grad[obst_poss.x[i]][obst_poss.y[i]] = 0;

	    return grad;
	} // Gradiente para evitar zonas (contornos), únicamente utilizando las posiciones de las aristas de los contornos (esquinas de los polígonos)

	/*
	void assign_value(Poss<int> poss, float val, vector<vector<float>> &grid)
	{
	    for(int i=0; i<poss.x.size(); i++)
	        if(grid[poss.x[i]][poss.y[i]]) grid[poss.x[i]][poss.y[i]] = val;
	} // Asignar un valor a las posiciones de un grid
	*/

	vector<vector<float>> navigable_grid(vector<vector<float>> static_grid, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<int> number_of_obstacles)
	{
	    vector<vector<float>> res = static_grid;
	    int val = 0;
	    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);

	    assign_value(free_poss, val + 5, res); // Las posiciones libres tienen prioridad

	    for(int i=0; i<number_of_obstacles.size(); i++)
	        assign_value(areas_poss[i], val - number_of_obstacles[i] + 1, res);

	    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

	    return res;
	} // Construcción del grid de navegación, asignando valores a las distintas regiones

	vector<vector<float>> navigable_grid_occup_thres(vector<vector<float>> static_grid, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<int> number_of_obstacles, float occupancy_threshold, int my_segment)
	{
	    vector<vector<float>> res = static_grid;
	    int val = 0, ap;
	    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);

	    assign_value(free_poss, val + 5, res); // Las posiciones libres tienen prioridad

	    for(int i=0; i<number_of_obstacles.size(); i++){
	    	ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
	    	//cout<<(float)number_of_obstacles[i]/ap<<" <= "<<occupancy_threshold<<endl;
	    	if((float)number_of_obstacles[i]/ap <= occupancy_threshold){ // área navegable
	        	assign_value(areas_poss[i], val - number_of_obstacles[i] + 1, res);
	        }else{ // área no navegable
	        	if(i != my_segment)
	        		assign_value(areas_poss[i], 0, res);
	        	else
	        		assign_value(areas_poss[i], val - number_of_obstacles[i] + 1, res);
	        }
	    }

	    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

	    return res;
	} // Construcción del grid de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado

	vector<vector<float>> navigable_grid_occup_thres(vector<vector<float>> static_grid, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<int> number_of_obstacles, float occupancy_threshold, vector<bool> agents_segments)
	{
	    vector<vector<float>> res = static_grid;
	    int val = 0, ap;
	    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);

	    assign_value(free_poss, val + 5, res); // Las posiciones libres tienen prioridad

	    for(int i=0; i<number_of_obstacles.size(); i++){
	    	ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
	    	//cout<<(float)number_of_obstacles[i]/ap<<" <= "<<occupancy_threshold<<endl;
	    	if((float)number_of_obstacles[i]/ap <= occupancy_threshold){ // área navegable
	        	assign_value(areas_poss[i], val - number_of_obstacles[i] + 1, res);
	        }else{ // área no navegable
	        	if(!agents_segments[i])
	        		assign_value(areas_poss[i], 0, res);
	        	else
	        		assign_value(areas_poss[i], val - number_of_obstacles[i] + 1, res);
	        }
	    }

	    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

	    return res;
	} // Construcción del grid de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado


	vector<vector<float>> navigable_grid_infl(vector<vector<float>> static_grid, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<int> number_of_obstacles)
	{

		// Para que las zonas que no se ven tengan menos prioridad que espacio vacío
		for(int i=0; i<number_of_obstacles.size(); i++)
			number_of_obstacles[i] = number_of_obstacles[i] ? number_of_obstacles[i] : 1;

	    vector<vector<float>> res = static_grid;
	    int val = 0;
	    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);

	    assign_value(free_poss, val + 5, res); // Las posiciones libres tienen prioridad

	    for(int i=0; i<number_of_obstacles.size(); i++)
	        assign_value(areas_poss[i], val - number_of_obstacles[i] + 1, res);

	    // Inflado
	    // Con un valor muy pequeño -> puede pasar, pero trata de evitar las posiciones cercanas a los obstáculos
	    for(int i=0; i<obst_poss.x.size(); i++)
	    	inflate_pos(obst_poss.x[i], obst_poss.y[i], res, 1, 0.001);

	    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

	    return res;
	} // Construcción del grid de navegación, asignando valores a las distintas regiones (incluye inflado de las regiones para no acercarse a ellas)

	void close_obstacles(Poss<int> obst, vector<vector<float>> &grid)
	{
	    // Comprobar la cantidad de vecinos que tiene cada obstáculo
	    for(int i=0; i<obst.x.size(); i++){
	        if(!grid[obst.x[i]+1][obst.y[i]+1]) grid[obst.x[i]][obst.y[i]+1] = grid[obst.x[i]+1][obst.y[i]] = 0;
	        if(!grid[obst.x[i]+1][obst.y[i]-1]) grid[obst.x[i]+1][obst.y[i]] = grid[obst.x[i]][obst.y[i]-1] = 0;
	        if(!grid[obst.x[i]-1][obst.y[i]-1]) grid[obst.x[i]][obst.y[i]-1] = grid[obst.x[i]-1][obst.y[i]-1] = 0;
	        if(!grid[obst.x[i]-1][obst.y[i]+1]) grid[obst.x[i]-1][obst.y[i]-1] = grid[obst.x[i]-1][obst.y[i]] = 0;
	    }
	} // Añadir obstáculos extra para encerrar por completo los contornos

	Poss<int> propagate_label(int x, int y, int label, vector<vector<float>> grid, vector<vector<int>> &label_map)
	{
	    Poss<int> res;
	    if(!grid[x][y]) return res;
	    Poss<int> q;
	    q.x.push_back(x); q.y.push_back(y);
	    vector<vector<float>> aux_grid = grid;
	    aux_grid[x][y] = 0;
	    int x_, y_;
	    while(q.x.size()){
	        label_map[q.x[0]][q.y[0]] = label;
	        res.x.push_back(q.x[0]); res.y.push_back(q.y[0]);
	        for(int i=0; i<geometry::nx4.size(); i++){
	            x_ = q.x[0] + geometry::nx4[i]; y_ = q.y[0] + geometry::ny4[i];
	            if(aux_grid[x_][y_] && grid[x_][y_]){ // NO se ha pasado por aquí y no es un obstáculo
	                q.x.push_back(x_); q.y.push_back(y_);
	                aux_grid[x_][y_] = 0;
	            }
	        }
	        q.erase(0);
	    }
	    return res;
	}

	int select_area(vector<Poss<int>> areas, vector<vector<int>> temp_map, int thres)
	{
	    for(int i=0; i<areas.size(); i++){
	        for(int j=0; j<areas[i].x.size(); j++){
	            if(temp_map[areas[i].x[j]][areas[i].y[j]] < thres){ // Área no observada durante el tiempo requerido
	                return i;
	            }
	        }
	    }
	    return -1;
	} // Seleccionar el área a explorar

	int select_area(vector<Poss<int>> areas, vector<vector<int>> temp_map, int thres, int x, int y)
	{
	    float d = INF, bd = INF;
	    int res = -1;
	    for(int i=0; i<areas.size(); i++){
	        for(int j=0; j<areas[i].x.size(); j++){
	            if(temp_map[areas[i].x[j]][areas[i].y[j]] < thres){ // Área no observada durante el tiempo requerido
	                d = sqrt(pow(x - areas[i].x[j],2)+pow(y - areas[i].y[j],2));
	                if(bd > d){
	                    bd = d;
	                    res = i;
	                }
	            }
	        }
	    }
	    return res;
	} // Seleccionar el área a explorar en base a la distancia Euclidea desde la posicón del agente

	vector<Poss<int>> select_areas(vector<Poss<int>> areas, vector<vector<int>> temp_map, int thres)
	{
	    vector<Poss<int>> res;
	    for(int i=0; i<areas.size(); i++){
	        for(int j=0; j<areas[i].x.size(); j++){
	            if(temp_map[areas[i].x[j]][areas[i].y[j]] < thres){ // Área no observada durante el tiempo requerido
	                res.push_back(areas[i]); // Seguir inspeccionando el área i
	                break;
	            }
	        }
	    }
	    return res;
	} // Seleccionar el conjunto de áreas a explorar

	void select_goal(int &x, int &y, Poss<int> all_poss, vector<vector<float>> grid, vector<vector<float>> grad, vector<vector<int>> temp_vis_map)
	{
	    float min_dist = INF;
	    int min_obs = RAND_MAX;
	    for(int i=0; i<all_poss.x.size(); i++){
	        if(grid[all_poss.x[i]][all_poss.y[i]]){
	            if(min_obs > temp_vis_map[all_poss.x[i]][all_poss.y[i]]){
	                x = all_poss.x[i]; y = all_poss.y[i];
	                min_obs = temp_vis_map[all_poss.x[i]][all_poss.y[i]];
	                min_dist = grad[all_poss.x[i]][all_poss.y[i]];
	            }else if(min_obs == temp_vis_map[all_poss.x[i]][all_poss.y[i]]){
	                if(min_dist > grad[all_poss.x[i]][all_poss.y[i]]){
	                    x = all_poss.x[i]; y = all_poss.y[i];
	                    //min_obs = temp_vis_map[all_poss.x[i]][all_poss.y[i]];
	                    min_dist = grad[all_poss.x[i]][all_poss.y[i]];
	                }
	            }
	        }
	    }
	} // Selección del punto menos observado de la lista de posiciones

	void select_goal(int &x, int &y, Poss<int> all_poss, vector<vector<float>> grid, vector<vector<float>> grad, vector<vector<int>> temp_vis_map, int thres)
	{
	    float min_dist = INF;
	    for(int i=0; i<all_poss.x.size(); i++){
	        if(grid[all_poss.x[i]][all_poss.y[i]]){
	            if(temp_vis_map[all_poss.x[i]][all_poss.y[i]] < thres){
	                if(min_dist > grad[all_poss.x[i]][all_poss.y[i]]){
	                    x = all_poss.x[i]; y = all_poss.y[i];
	                    min_dist = grad[all_poss.x[i]][all_poss.y[i]];
	                }
	            }
	        }
	    }
	} // Selección de un objetivo en base a la distancia y el tiempo de observación de las posiciones

	Poss<int> informative_goal(int x, int y, vector<vector<float>> &grad, vector<Poss<int>> areas_poss, vector<vector<float>> navigation_grid, vector<vector<float>> static_grid, vector<vector<int>> temp_vis_map, float vrange)
	{
	    Poss<int> res; res(1);

	    Poss<int> all_poss;
	    for(int i=0; i<areas_poss.size(); i++) all_poss.append(areas_poss[i]);

	    if(all_poss.x.size()){ // Seleccionar goal
	        // Seleccionar la posición más cercana del área menos vista
	        FMM gr(x, y, navigation_grid);
	        grad = gr.compute_gradient_();
	    }else{ // Todavía no se ha observado nada dinámico
	        // Seleccionar la posición menos vista más cercana
	        FMM gr(x, y, static_grid);
	        grad = gr.expand_dist(INF, all_poss);
	    }

	    select_goal(res.x[0], res.y[0], all_poss, navigation_grid, grad, temp_vis_map);

	    return res;
	} // Selección del goal más cercano y menos "visto" a las áreas con dinamismo

	Poss<int> informative_goal(int x, int y, vector<vector<float>> &grad, Poss<int> area_poss, vector<vector<float>> navigation_grid, vector<vector<float>> static_grid, vector<vector<int>> temp_vis_map, float vrange)
	{
	    vector<Poss<int>> areas_poss = {area_poss};
	    return informative_goal(x, y, grad, areas_poss, navigation_grid, static_grid, temp_vis_map, vrange);
	} // Lo mismo que antes pero para las posiciones de un único área

	Poss<int> informative_goal(int x, int y, vector<vector<float>> &grad, vector<Poss<int>> areas_poss, Poss<int> free_poss, vector<vector<float>> navigation_grid, vector<vector<float>> static_grid, vector<vector<int>> temp_vis_map, float vrange, int temp_thres)
	{
	    Poss<int> res; res(1);

	    Poss<int> all_poss;
	    for(int i=0; i<areas_poss.size(); i++) all_poss.append(areas_poss[i]);

	    if(all_poss.x.size()){ // Seleccionar goal
	        // Seleccionar la posición más cercana del área menos vista
	        FMM gr(x, y, navigation_grid);
	        grad = gr.compute_gradient_();

	        select_goal(res.x[0], res.y[0], all_poss, navigation_grid, grad, temp_vis_map, temp_thres);
	        if(res.x[0]==0 && res.y[0]==0){ // No se ha encontrado un punto que que esté por debajo del tiempo de observación
	            // Seleccionar el siguiente "de exploración"
	            select_goal(res.x[0], res.y[0], free_poss, navigation_grid, grad, temp_vis_map);
	        }
	    }else{ // Todavía no se ha observado nada dinámico
	        // Seleccionar la posición menos vista más cercana
	        //FMM gr(x, y, static_grid);
	        FMM gr(x, y, navigation_grid);
	        //grad = gr.expand_dist(INF, all_poss);
	        grad = gr.expand_dist(INF, free_poss);

	        select_goal(res.x[0], res.y[0], free_poss, navigation_grid, grad, temp_vis_map);
	    }

	    return res;
	} // Selección del goal más cercano y menos "visto" a las áreas con dinamismo

	vector<int> extract_movement(vector<Poss<int>> areas_poss, vector<vector<int>> obst_poss_map, vector<vector<int>> _obst_poss_map)
	{
	    vector<int> res(2,0);
	    // - 0: Cantidad de movimiento
	    // - 1: Cantidad de movimiento nuevo
	    for(int i=0; i<areas_poss.size(); i++){
	        for(int j=0; j<areas_poss[i].x.size(); j++){
	            if(obst_poss_map[areas_poss[i].x[j]][areas_poss[i].y[j]] > _obst_poss_map[areas_poss[i].x[j]][areas_poss[i].y[j]]){
	                res[0]++;
	                if(_obst_poss_map[areas_poss[i].x[j]][areas_poss[i].y[j]] == 0) res[1]++;
	                //cout<<"("<<areas_poss[i].x[j]<<", "<<areas_poss[i].y[j]<<"): "<<_obst_poss_map[areas_poss[i].x[j]][areas_poss[i].y[j]]<<"/"<<obst_poss_map[areas_poss[i].x[j]][areas_poss[i].y[j]]<<endl;
	            }
	        }
	    }
	    return res;
	} // Cantidad de movimiento que se observa

	vector<int> extract_movement(Poss<int> obst_poss, vector<vector<int>> obst_poss_map, vector<vector<int>> _obst_poss_map)
	{
	    vector<int> res(2,0);
	    // - 0: Cantidad de movimiento
	    // - 1: Cantidad de movimiento nuevo
	    for(int j=0; j<obst_poss.x.size(); j++){
	        if(obst_poss_map[obst_poss.x[j]][obst_poss.y[j]] > _obst_poss_map[obst_poss.x[j]][obst_poss.y[j]]){
	            res[0]++;
	            if(_obst_poss_map[obst_poss.x[j]][obst_poss.y[j]] == 0) res[1]++;
	            //cout<<"("<<obst_poss.x[j]<<", "<<obst_poss.y[j]<<"): "<<_obst_poss_map[obst_poss.x[j]][obst_poss.y[j]]<<"/"<<obst_poss_map[obst_poss.x[j]][obst_poss.y[j]]<<endl;
	        }
	    }
	    return res;
	} // Cantidad de movimiento que se observa

	vector<vector<bool>> obtain_adjacency(vector<Poss<int>> poss, vector<vector<int>> poss_map)
	{
	    vector<vector<bool>> res(poss.size(),vector<bool>(poss.size(),false));
	    int x, y;
	    for(int i=0; i<poss.size(); i++){
	        for(int j=0; j<poss[i].x.size(); j++){
	            for(int k=0; k<geometry::nx8.size(); k++){
	                x = poss[i].x[j] + geometry::nx8[k]; y = poss[i].y[j] + geometry::ny8[k];
	                if(x>=0 && x<poss_map.size() && y>=0 && y<poss_map[0].size()) // Asegurar que no me salga del grid
	                if(poss_map[x][y] >=0 && poss_map[poss[i].x[j]][poss[i].y[j]] != poss_map[x][y]){
	                    res[poss_map[poss[i].x[j]][poss[i].y[j]]][poss_map[x][y]] = true;
	                    res[poss_map[x][y]][poss_map[poss[i].x[j]][poss[i].y[j]]] = true;
	                    //cout<<"("<<poss[i].x[j]<<", "<<poss[i].y[j]<<") y ("<<x<<", "<<y<<"): "<<poss_map[poss[i].x[j]][poss[i].y[j]]<<" - "<<poss_map[x][y]<<endl;
	                    //cin.get();
	                }
	            }
	        }
	    }
	    return res;
	}

	bool check_observation(Poss<int> poss, vector<vector<int>> observed_time_map, int time)
	{
	    for(int i=0; i<poss.x.size(); i++)
	        if(observed_time_map[poss.x[i]][poss.y[i]] < time) return false;
	    return true;
	}

	vector<bool> areAreasChanging(vector<Poss<int>> areas_poss, vector<int> areas)
	{
	    // - areas_poss: posiciones actuales de las áreas dinámicas
	    // - areas: 
	    int n = areas_poss.size();
	    vector<bool> res(n, false);
	    for(int i=0; i<n; i++){
	        if(i < areas.size())
	        if(areas_poss[i].x.size() != areas[i]){
	            res[i] = true;
	        }
	    }
	    return res;
	}

	vector<vector<float>> get_distances(vector<Poss<int>> areas_poss, vector<vector<float>> dist_grad)
	{
	    vector<vector<float>> res(areas_poss.size());
	    for(int i=0; i<areas_poss.size(); i++){
	        res[i].resize(areas_poss[i].x.size());
	        for(int j=0; j<areas_poss[i].x.size(); j++){
	            res[i][j] = dist_grad[areas_poss[i].x[j]][areas_poss[i].y[j]];
	        }
	    }
	    return res;   
	}

	template <typename T>
	vector<float> distance_in_area(T xc, T yc, Poss<T> area_poss)
	{
	    vector<float> res(area_poss.x.size(),0);
	    for(int i=0; i<area_poss.x.size(); i++) res[i] = sqrt(pow(xc - area_poss.x[i],2)+pow(yc - area_poss.y[i],2));
	    return res;   
	}
	    
	template <typename T>
	int number_of_max(vector<T> val)
	{
	    int res = 0;
	    int ind = -1;
	    T max = -(T)RAND_MAX;
	    for(int i=0; i<val.size(); i++)
	        if(max < val[i]){
	            max = val[i];
	            ind = i;
	        }
	    if(ind>=0){
	        for(int i=0; i<val.size(); i++)
	            if(val[i] == max) res++;
	    }
	    return res;
	}

	void inflate_areas(vector<Poss<int>> &areas_poss, vector<Poss<int>> areas_contours_poss, vector<vector<int>> &areas_map)
	{
		int x, y;
		for(int i=0; i<areas_contours_poss.size(); i++){
			for(int j=0; j<areas_contours_poss[i].x.size(); j++){
				for(int k=1; k<navigation::move_x.size(); k++){
					x = areas_contours_poss[i].x[j] + navigation::move_x[k];
					y = areas_contours_poss[i].y[j] + navigation::move_y[k];
					if(areas_map[x][y] < 0){ // No pertenecía al área
						areas_map[x][y] = areas_map[areas_contours_poss[i].x[j]][areas_contours_poss[i].y[j]];
						areas_poss[i].x.push_back(x); areas_poss[i].y.push_back(y);
					}
				}
			}
		}
	}

	struct Areas{
		int N = 0; // Cantidad de áreas
		Poss<int> centroids; // Centroides de las áreas para etiquetar
		Poss<int> real_centroids; // Centroides geométricos de las áreas
		vector<Poss<int>> contours; // Contornos de las áreas (VÉRTICES: solo puntos)
		vector<Poss<int>> contours_edges; // Contornos de las áreas (ARISTAS: los puntos de la línea que forma la arista)
		vector<Poss<int>> poss; // Todas las posiciones de las áreas
		vector<vector<int>> map; // Mapa de las áreas

		void erase_area(int ind){
			if(ind < 0 || ind > N) return;
			for(int i=0; i<poss[ind].x.size(); i++)
				map[poss[ind].x[i]][poss[ind].y[i]] = 0;
			centroids.erase(ind);
			real_centroids.erase(ind);
			contours.erase(contours.begin() + ind);
			contours_edges.erase(contours_edges.begin() + ind);
			poss.erase(poss.begin() + ind);
			N--;
		}

	}; // Variables de las áreas navegables

	Areas obtain_areas(vector<Poss<int>> clusters, vector<vector<float>> static_map)
	{
		Areas res;
		res.N = clusters.size();
		res.centroids(res.N); res.real_centroids(res.N);
		res.contours.resize(res.N); res.contours_edges.resize(res.N);
		res.poss.resize(res.N);
		res.map.resize(static_map.size(), vector<int>(static_map[0].size(),-1));

		Poss<int> centroid;
		vector<vector<float>> contour_grid = static_map;
		for(int i=0; i<res.N; i++){
			if(!clusters[i].x.size()) continue;
            res.contours[i] = geometry::convex_hull(clusters[i]);

            res.contours_edges[i] = geometry::polygon_edges(res.contours[i]);
            //dynamic_areas::assign_value(res.contours_edges[i], 0, contour_grid);
            assign_value(res.contours_edges[i], 0, contour_grid);

            // Centroides para despues etiquetar
            centroid = res.contours[i].centroid();
            centroid.x[0] = round(centroid.x[0]); centroid.y[0] = round(centroid.y[0]);
            res.real_centroids.x[i] = round(centroid.x[0]); res.real_centroids.y[i] = round(centroid.y[0]); // Almacenar el centroide original
            if(geometry::isInsidePolygon(centroid.x[0], centroid.y[0], res.contours[i])){ // Centroide dentro del contorno
                res.centroids.x[i] = centroid.x[0]; res.centroids.y[i] = centroid.y[0];
            }else{ // Centroide queda fuera del contorno
                res.centroids.x[i] = 0; res.centroids.y[i] = 0;
            }

            // Etiquetar los puntos que quedan dentro del contorno
            res.poss[i] = dynamic_areas::propagate_label(res.centroids.x[i], res.centroids.y[i], i, contour_grid, res.map);
            //all_areas_poss[i].append(contours_edges[i]); // Incluir los contornos
            for(int j=0; j<res.contours_edges[i].x.size(); j++)
                if(static_map[res.contours_edges[i].x[j]][res.contours_edges[i].y[j]]) res.poss[i].push(res.contours_edges[i].x[j], res.contours_edges[i].y[j]);
            
            //areas_points[i] = res.areas_poss[i].x.size(); // Cantidad de puntos dentro de cada área dinámica (área real)

            // Etiquetar las posiciones del contorno
            for(int j=0; j<res.contours_edges[i].x.size(); j++){
                res.map[res.contours_edges[i].x[j]][res.contours_edges[i].y[j]] = i;
            }
        }

        return res;

	}

	struct Areas_information{
	    // Información que necesito para el tiempo de observación
	    // Obstáculos que se ven dentro de cada área
	    vector<Poss<int>> obst_poss; // Sus posiciones
	    vector<int> nobst; // Cantidad de agentes
	    // Distancias propias de las áreas
	    vector<vector<float>> distances_area_poss2obst; // Distancias de los puntos a los obstáculo estáticos
	    // [4] 0: mínima centroide - contorno, 1: media centroide - contorno, 2: máxima centroide - contorno, (Distancia Euclídea)
	    //     3: centroide - obstáculos estáticos , 4: distancia media del área a los obstáculos, 5: distancia máxima del área a los obstáculos (Gradiente)
	    vector<vector<float>> distances_of_areas;
	    // Distancias de los obstáculo dentro de sus respectivas áreas
	    vector<vector<float>> distances_of_obstacles; // [2] 0: obstáculos - centroide, 1: obstáculos - obstáculos estáticos
	};
	
	Areas_information obtain_areas_information(vector<Poss<int>> all_areas_poss, vector<vector<int>> areas_map, Poss<int> areas_centroids, vector<Poss<int>> areas_contours, Poss<int> obstacles_poss, vector<vector<float>> obst_grad)
	{
	    Areas_information res;
	    int n = all_areas_poss.size();
	    if(n == 0) return res;
	    // Distancias propias de cada área
	    float d, maxd, mind;
	    res.distances_area_poss2obst.resize(n);
	    res.distances_of_areas.resize(6, vector<float>(n, -1));
	    for(int i=0; i<n; i++){
	    	res.distances_of_areas[3][i] = obst_grad[areas_centroids.x[i]][areas_centroids.y[i]];
	        res.distances_area_poss2obst[i].resize(all_areas_poss[i].x.size(),-1);
	        for(int j=0; j<res.distances_area_poss2obst[i].size(); j++){
	            res.distances_area_poss2obst[i][j] = obst_grad[all_areas_poss[i].x[j]][all_areas_poss[i].y[j]];
	            res.distances_of_areas[4][i] += res.distances_area_poss2obst[i][j];
	            if(res.distances_of_areas[5][i] < res.distances_area_poss2obst[i][j]) res.distances_of_areas[5][i] = res.distances_area_poss2obst[i][j];
	        }
	        if(all_areas_poss[i].x.size()) res.distances_of_areas[4][i] /= res.distances_area_poss2obst[i].size();
	        maxd = -1; mind = INF;
	        for(int j=0; j<areas_contours[i].x.size(); j++){
	        	d = sqrt(pow(areas_centroids.x[i]-areas_contours[i].x[j],2)+pow(areas_centroids.y[i]-areas_contours[i].y[j],2));
	        	res.distances_of_areas[1][i] += d;
	        	if(maxd < d) maxd = d;
	        	if(mind > d) mind = d;
	        }
	        if(areas_contours[i].x.size()) res.distances_of_areas[1][i] /= areas_contours[i].x.size();
	        res.distances_of_areas[0][i] = mind;
	        res.distances_of_areas[2][i] = maxd;
	    }
	    // Separar los obstáculos dinámicos que hay en cada área y obtener las distancias de los obstáculos dentro de sus respectivas áreas
	    res.obst_poss.resize(n); res.nobst.resize(n,0);
	    res.distances_of_obstacles.resize(2, vector<float>(obstacles_poss.x.size(), -1));
	    int iarea;
	    for(int i=0; i<obstacles_poss.x.size(); i++){
	        iarea = areas_map[obstacles_poss.x[i]][obstacles_poss.y[i]];
	        if(iarea >= 0){
	            res.obst_poss[iarea].push(obstacles_poss.x[i], obstacles_poss.y[i]);
	            res.nobst[iarea]++;
	            res.distances_of_obstacles[0][i] = sqrt(pow(obstacles_poss.x[i]-areas_centroids.x[iarea],2) + pow(obstacles_poss.y[i]-areas_centroids.y[iarea],2));
	        }
	        res.distances_of_obstacles[1][i] = obst_grad[obstacles_poss.x[i]][obstacles_poss.y[i]];
	    }
	    return res;
	}

	void show_areas_information(Areas_information areas_information)
	{
		cout<<"----------------------------------------------------------------"<<endl;
        cout<<"INFO areas"<<endl;
        cout<<"----------------------------------------------------------------"<<endl;
        if(areas_information.distances_of_areas.size()){
            sh_vect_h(areas_information.nobst,"nobstaculos");
            cout<<endl<<"Distancias del área"<<endl;
            sh_vect_h(areas_information.distances_of_areas[0]," min(d_ca)");
            sh_vect_h(areas_information.distances_of_areas[1],"mean(d_ca)");
            sh_vect_h(areas_information.distances_of_areas[2]," max(d_ca)");
            sh_vect_h(areas_information.distances_of_areas[3],"     Do(c)");
            sh_vect_h(areas_information.distances_of_areas[4]," mean(D_o)");
            sh_vect_h(areas_information.distances_of_areas[5],"  max(D_o)");
            cout<<endl<<"Distancias de los obstáculos"<<endl;
            sh_vect_h(areas_information.distances_of_obstacles[0],"d_co");
            sh_vect_h(areas_information.distances_of_obstacles[1]," D_o");
        }
        cout<<"----------------------------------------------------------------"<<endl;
	}

	struct Obstacles{
		Poss<int> poss; // Posiciones donde se encuentran los obstáculos
		vector<vector<bool>> map; // Mapa con las posiciones de los obstáculos
		vector<vector<bool>> trajectories_map;  // Mapa con los recorridos de los obstáculos
		int map_visited_points = 0; // Cantidad de puntos del mapa que han sido visitados por los obstáculos
		Poss<int> trajectories_poss; // Las posiciones que han recorrido los obstáculos
		//vector<vector<int>> temp_map, _temp_map;
	};

	struct DynAreas{
		Areas areas;
		vector<Poss<int>> clusters;
		vector<int> movement;
		Areas_information info;
	};

	void update_obstacles_variables(Obstacles &obst_obj, vector<vector<float>> static_map, vector<vector<float>> global_map, Poss<int> visible_poss, Poss<int> visible_obstacles)
	{
		// Actualizar la lista de obstáculos, el mapa y las trayectorías de los obstáculos
		update_trajectories_map(obst_obj.poss, obst_obj.trajectories_map, obst_obj.trajectories_poss, obst_obj.map_visited_points);
		// Actualizar la lista de los obstáculos
		update_obstacles_list(obst_obj.poss, obst_obj.map, visible_obstacles, visible_poss, global_map, static_map);
	} // Actualización de las variables de los obstáculos

	DynAreas compute_dynamic_areas(Obstacles &obst_obj, vector<vector<float>> static_map, vector<vector<float>> global_map, Poss<int> visible_poss, Poss<int> visible_obstacles)
	{
		DynAreas res;

		// Actualización de las variables de los obstáculos
		update_obstacles_variables(obst_obj, static_map, global_map, visible_poss, visible_obstacles);

		res.clusters = adjacency_clustering::cluster(obst_obj.trajectories_poss, static_map);
		res.areas = obtain_areas(res.clusters, static_map);

		return res;
	}

	void updateNavigableMap(vector<vector<float>> &navigable_map, vector<vector<float>> original_map, Poss<int> laser_poss, Poss<int> all_laser_poss, int ax, int ay)
	{
	    // Todos los puntos del laser (bresenham)
	    bool b;
	    for(int i=0; i<all_laser_poss.x.size(); i++){
	        b = true;
	        for(int j=0; j<laser_poss.x.size(); j++){
	            if(all_laser_poss.x[i] == laser_poss.x[j] && all_laser_poss.y[i] == laser_poss.y[j]){
	                b = false;
	            }
	        }
	        if(b){
	            navigable_map[all_laser_poss.x[i]][all_laser_poss.y[i]] = original_map[all_laser_poss.x[i]][all_laser_poss.y[i]];
	        }
	    }

	    // Los puntos visibles con el laser son obstáculos
	    for(int i=0; i<laser_poss.x.size(); i++){
	        //if(navigable_map[laser_poss.x[i]][laser_poss.y[i]]){
	            navigable_map[laser_poss.x[i]][laser_poss.y[i]] = 0;
	            // Incluyo las posiciones del agente (volumen)
	            Poss<int> agent_vol = agentPoss(laser_poss.x[i], laser_poss.y[i], ax, ay);
	            agent_vol = fix_poss_in_map(agent_vol, original_map.size(), original_map[0].size());
	            for(int j=0; j<agent_vol.x.size(); j++){
	                navigable_map[agent_vol.x[j]][agent_vol.y[j]] = 0;
	            }
	        //}
	    }

	}

	void _updateNavigableMap(vector<vector<float>> &navigable_map, vector<vector<float>> original_map, Poss<int> laser_poss, Poss<int> all_laser_poss, int ax, int ay, int x, int y, float vrange)
	{
	    // Todas las posiciones que se están viendo
	    for(int i=0; i<all_laser_poss.x.size(); i++){
	        navigable_map[all_laser_poss.x[i]][all_laser_poss.y[i]] = original_map[all_laser_poss.x[i]][all_laser_poss.y[i]];
	    }

	    // Las posisciones hasta las que llega el laser
	    float d;
	    for(int i=0; i<laser_poss.x.size(); i++){
	        if(original_map[laser_poss.x[i]][laser_poss.y[i]]){ // Estoy viendo un obstáculo que no está en el mapa estático
	            d = hypot(x - laser_poss.x[i], y - laser_poss.y[i]);
	            //if(d < vrange){ // No es el rango máximo del laser
	            if(abs(d-vrange)>3){ // ">3" : es para asegurar que quede fuera (Por si hay algún error de píxel)           
	                //if(navigable_map[laser_poss.x[i]][laser_poss.y[i]]){ // Tampoco en el mapa dinámico
	                navigable_map[laser_poss.x[i]][laser_poss.y[i]] = 0;
	                // Incluyo las posiciones del agente (volumen)
	                Poss<int> agent_vol = agentPoss(laser_poss.x[i], laser_poss.y[i], ax, ay);
	                agent_vol = fix_poss_in_map(agent_vol, original_map.size(), original_map[0].size());
	                for(int j=0; j<agent_vol.x.size(); j++){
	                    navigable_map[agent_vol.x[j]][agent_vol.y[j]] = 0;
	                }
	            }
	        }
	    }
	}

	Poss<int> inflatePoss(Poss<int> poss, vector<vector<float>> velocity_map, float dist)
	{
	    // Inflar los obstáculos con FMM e insertar en el "mapa de navegación"
	    FMM gr_do(poss.x, poss.y, velocity_map);
	    Poss<int> exp_poss; // Posiciones que se van a "expandir"
	    gr_do.expand_dist(dist, exp_poss);
	    return exp_poss;
	}

	void inflatePoss(Poss<int> poss, vector<vector<float>> velocity_map, float dist, vector<vector<float>> &infl_map)
	{
	    // Inflar los obstáculos con FMM e insertar en el "mapa de navegación"
	    FMM gr_do(poss.x, poss.y, velocity_map);
	    Poss<int> exp_poss; // Posiciones que se van a "expandir"
	    vector<vector<float>> grad_do = gr_do.expand_dist(dist, exp_poss);
	    assign_value(exp_poss, 0, infl_map); // Fijar las posiciones "expandidas" como obstáculos
	}

	void inflatePoss(Poss<int> poss, vector<vector<float>> velocity_map, float dist, vector<vector<float>> &infl_map, Poss<int> &exp_poss)
	{
	    // Inflar los obstáculos con FMM e insertar en el "mapa de navegación"
	    FMM gr_do(poss.x, poss.y, velocity_map);
	    exp_poss.clear();
	    vector<vector<float>> grad_do = gr_do.expand_dist(dist, exp_poss);
	    assign_value(exp_poss, 0, infl_map); // Fijar las posiciones "expandidas" como obstáculos
	}

	void inflatePoss(Poss<int> poss, vector<vector<float>> velocity_map, float dist, float value, vector<vector<float>> &infl_map)
	{
	    // Inflar los obstáculos con FMM e insertar en el "mapa de navegación"
	    FMM gr_do(poss.x, poss.y, velocity_map);
	    Poss<int> exp_poss; // Posiciones que se van a "expandir"
	    vector<vector<float>> grad_do = gr_do.expand_dist(dist, exp_poss);
	    assign_value(exp_poss, value, infl_map); // Fijar las posiciones "expandidas" como obstáculos
	}

	void updateNavigableMapInfl(vector<vector<float>> &navigable_map, vector<vector<float>> original_map, Poss<int> laser_poss, Poss<int> all_laser_poss, int x, int y, float vrange, float infl_dist, vector<vector<float>> infl_velocity_map)
	{
	    // Todas las posiciones que se están viendo
	    for(int i=0; i<all_laser_poss.x.size(); i++){
	        navigable_map[all_laser_poss.x[i]][all_laser_poss.y[i]] = original_map[all_laser_poss.x[i]][all_laser_poss.y[i]];
	    }

	    // Las posisciones hasta las que llega el laser
	    float d;
	    Poss<int> infl_poss;
	    for(int i=0; i<laser_poss.x.size(); i++){
	        if(original_map[laser_poss.x[i]][laser_poss.y[i]]){ // Estoy viendo un obstáculo que no está en el mapa estático
	            d = hypot(x - laser_poss.x[i], y - laser_poss.y[i]);
	            //if(d < vrange){ // No es el rango máximo del laser
	            if(abs(d-vrange)>3){ // ">3" : es para asegurar que quede fuera (Por si hay algún error de píxel)           
	                //if(navigable_map[laser_poss.x[i]][laser_poss.y[i]]){ // Tampoco en el mapa dinámico
	            	infl_poss.push(laser_poss.x[i], laser_poss.y[i]);
	            }
	        }
	    }
	    inflatePoss(infl_poss, infl_velocity_map, infl_dist, navigable_map); // <<<--------------------*******************
	}

	void clearMap(vector<vector<float>> &navigable_map, const vector<vector<float>>& original_map, const Poss<int>& laser_poss)
	{
		// Todas las posiciones que se están viendo
	    for(int i=0; i<laser_poss.x.size(); i++){
	        navigable_map[laser_poss.x[i]][laser_poss.y[i]] = original_map[laser_poss.x[i]][laser_poss.y[i]];
	    }
	}

	void eraseTrajectoriesPoints(const Poss<int>& pos, vector<vector<bool>> &map, Poss<int> &traj_points)
	{
	    for(int i=0; i<pos.x.size(); i++)
	        if(map[pos.x[i]][pos.y[i]] == true){
	            traj_points.erase_point(pos.x[i], pos.y[i]); // Eliminar punto
	            map[pos.x[i]][pos.y[i]] = false; // Marcar en el mapa
	        }
	}

	void eraseTrajectoriesPoints(const Poss<int>& pos, vector<vector<bool>> &map, Poss<int> &traj_points, int &nvisited_points)
	{
	    for(int i=0; i<pos.x.size(); i++)
	        if(map[pos.x[i]][pos.y[i]] == true){
	            traj_points.erase_point(pos.x[i], pos.y[i]); // Eliminar punto
	            map[pos.x[i]][pos.y[i]] = false; // Marcar en el mapa
	            nvisited_points--;
	        }
	}

	void eraseTrajectoriesPoints(const Poss<int>& pos, vector<vector<bool>> &trajectories_map, Poss<int> &traj_points, const vector<vector<int>>& areas_map, vector<Poss<int>> &traj_points_pa)
	{
	    for(int i=0; i<pos.x.size(); i++)
	        if(trajectories_map[pos.x[i]][pos.y[i]] == true){
	            traj_points.erase_point(pos.x[i], pos.y[i]); // Eliminar punto
	            trajectories_map[pos.x[i]][pos.y[i]] = false; // Marcar en el mapa
	            if(areas_map[pos.x[i]][pos.y[i]] >= 0){
	            	traj_points_pa[areas_map[pos.x[i]][pos.y[i]]].erase_point(pos.x[i], pos.y[i]); // Eliminar punto
	            }
	        }
	}

	void eraseTrajectoriesPoints(const vector<Poss<int>>& pos, vector<vector<bool>> &trajectories_map, Poss<int> &traj_points, vector<Poss<int>> &traj_points_pa)
	{
	    for(int i=0; i<pos.size(); i++){
	    	if(pos[i].x.size() == 0){
	    		if(traj_points_pa[i].x.size()){
	    			dynamic_areas::eraseTrajectoriesPoints(traj_points_pa[i], trajectories_map, traj_points);
	    			traj_points_pa[i].clear();
	    		}
	    	}
	    }
	}

	vector<Poss<int>> possPerArea(const Poss<int>& poss, const vector<vector<int>>& areas_map, int Nareas)
	{
		vector<Poss<int>> possPerArea(Nareas);
		for(int i=0; i<poss.x.size(); i++){
			if(areas_map[poss.x[i]][poss.y[i]] >= 0){
				possPerArea[areas_map[poss.x[i]][poss.y[i]]].push(poss.x[i], poss.y[i]);
			}
		}
		return possPerArea;
	}

	vector<int> obstaclesPerArea(const Poss<int>& obstacles, const vector<vector<int>>& areas_map, int nareas)
	{
	    vector<int> res(nareas,0);
	    if(!obstacles.x.size()) return res;
	    for(int i=0; i<obstacles.x.size(); i++){
	        if(areas_map[obstacles.x[i]][obstacles.y[i]] >= 0) res[areas_map[obstacles.x[i]][obstacles.y[i]]]++;
	    }
	    return res;
	} // Cantidad de obstáculos por áreas

	vector<int> obstaclesPerAreaNz(const Poss<int>& obstacles, const vector<vector<int>>& areas_map, int nareas)
	{
	    vector<int> res(nareas,0);
	    if(!obstacles.x.size()) return res;
	    for(int i=0; i<obstacles.x.size(); i++){
	        if(areas_map[obstacles.x[i]][obstacles.y[i]] >= 0) res[areas_map[obstacles.x[i]][obstacles.y[i]]]++;
	    }
	    // Asegurar que las áreas tengan al menos un obstáculo para evitar las áreas
	    for(int i=0; i<res.size(); i++)
	    	if(res[i] == 0) res[i] = 1;
	    return res;
	} // Cantidad de obstáculos por áreas (se considera que todas las áreas tienen al menos un obstáculo)

	vector<int> forget_areas(int forget_dynamic_areas, Areas areas, Poss<int> obstacles_list, Poss<int> visible_obstacles, vector<Poss<int>> clusters, vector<vector<bool>> &trajectories_map, Poss<int> &obstacles_trajectories_points, int &number_of_visited_points)
	{
		vector<int> number_of_obstacles;
		if(forget_dynamic_areas == 0){ // NO olvidar nada (CONSERVADOR)
            number_of_obstacles = dynamic_areas::obstaclesPerAreaNz(obstacles_list, areas.map, areas.N); // Conservar al menos 1 obstáculo (virtual) por área
        }else if(forget_dynamic_areas == 1){ // Olvidar las áreas, conservar los puntos de trayectorías (OLVIDÓN)
            number_of_obstacles = dynamic_areas::obstaclesPerArea(obstacles_list, areas.map, areas.N); // No conservar obstáculos en áreas si no los hay
        }else if (forget_dynamic_areas == 2){ // Olvidar las áreas, olvidar los puntos de trayectorias (PASOTA TOTAL)
            number_of_obstacles = dynamic_areas::obstaclesPerArea(obstacles_list, areas.map, areas.N); // No conservar obstáculos en áreas si no los hay
            // Eliminar los puntos
            for(int i=0; i<number_of_obstacles.size(); i++){
                if(number_of_obstacles[i] == 0){
                    dynamic_areas::eraseTrajectoriesPoints(clusters[i], trajectories_map, obstacles_trajectories_points, number_of_visited_points);
                }
            }
        }else if(forget_dynamic_areas == 3){ // No considerar las áreas de los obstáculos visibles ahora mismo
            number_of_obstacles = dynamic_areas::obstaclesPerArea(obstacles_list, areas.map, areas.N);
            for(int i=0; i<visible_obstacles.x.size(); i++){
                if(areas.map[visible_obstacles.x[i]][visible_obstacles.y[i]] >= 0){
                    number_of_obstacles[areas.map[visible_obstacles.x[i]][visible_obstacles.y[i]]] = 0;
                }
            }
        }
        return number_of_obstacles;
	}

	vector<int> obstacles_sort(vector<int> no)
	{
	    int n = no.size();
	    int max, ind;
	    vector<int> res(n, 0);
	    for(int i=0; i<n; i++){
	        max = 0;
	        for(int j=0; j<n; j++){
	            if(max < no[j]){
	                max = no[j];
	                ind = j;
	            }
	        }
	        if(max){
	            no[ind] = 0;
	            res[ind] = i + 1;
	        }
	    }
	    return res;
	}

	vector<vector<float>> navigable_norm_grad_occup_thres(vector<vector<float>> norm_grad, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<int> number_of_obstacles, float occupancy_threshold, int my_segment)
	{
	    // norm_grad: gradiente de obstáculos normalizado [0- 1]
	    vector<vector<float>> res = norm_grad;
	    int val = 0, ap;
	    //number_of_obstacles = norm_v(number_of_obstacles);
	    number_of_obstacles = obstacles_sort(number_of_obstacles);
	    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);
	    else return res;

	    //increase_value(free_poss, val + 1, res); // Las posiciones libres tienen prioridad
	    increase_value(free_poss, 1, res); // Las posiciones libres tienen prioridad

	    for(int i=0; i<number_of_obstacles.size(); i++){
	        ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
	        if((float)number_of_obstacles[i]/ap <= occupancy_threshold){ // área navegable
	            increase_value(areas_poss[i], 0, res);
	        }else{ // área no navegable
	            if(i != my_segment){
	                assign_value(areas_poss[i], 0, res);
	            }else{
	                increase_value(areas_poss[i], 0, res);
	            }
	        }
	    }

	    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

	    return res;
	} // Construcción del gradiente de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado

	vector<vector<float>> navigable_grad_occup_thres(vector<vector<float>> grad, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<int> number_of_obstacles, float occupancy_threshold, int my_segment)
	{
	    // norm_grad: gradiente de obstáculos normalizado [0- 1]
	    vector<vector<float>> res = grad;
	    int val = 0, ap;
	    number_of_obstacles = obstacles_sort(number_of_obstacles);
	    if(number_of_obstacles.size()) val = max_v(number_of_obstacles);
	    else return res;

	    vector<float> no(number_of_obstacles.size(), 0);
	    val = max_v(number_of_obstacles);
	    for(int i=0; i<no.size(); i++) no[i] = (float)number_of_obstacles[i]/val;

	    val = 2;

	    // Las posiciones libres tienen prioridad
	    increase_value(free_poss, val + 1, res); // Subir el gradiente por encima de los obstáculos

	    for(int i=0; i<number_of_obstacles.size(); i++){
	        ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
	        if((float)number_of_obstacles[i]/ap <= occupancy_threshold){ // área navegable
	            assign_value(areas_poss[i], val - no[i], res);
	        }else{ // área no navegable
	            if(i != my_segment){
	                assign_value(areas_poss[i], 0, res);
	            }else{
	                assign_value(areas_poss[i], val - no[i], res);
	            }
	        }
	    }

	    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

	    return res;
	} // Construcción del gradiente de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado

	vector<vector<float>> navigable_grad_occup_thres_infl(vector<vector<float>> grad, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<Poss<int>> areas_contours, int rx, int ry, vector<int> number_of_obstacles, float occupancy_threshold, int my_segment)
	{
	    // grad: gradiente de obstáculos
	    // free_poss: posiciones libres
	    // obst_poss: posiciones de los obstáculos estáticos
	    // areas_poss: posiciones de las áreas dinámicas
	    // areas_contours: posiciones unicamente de los contornos de las áreas (para los inflados)
	    // rx, ry: tamaño del robot
	    // number_of_obstacles: numero de posiciones que ocupan los obstáculos dentro de cada área dinámica
	    // occupancy_threshold: umbral de ocupación por debajo del cual un área se considera inatravesable
	    // my_segment: segmento en el que se encuentra el robot (para no quedar atascado en una partición no transitable)
	    vector<vector<float>> res = grad;
	    int val = 0, ap;

	    vector<int> number_of_steps = obstacles_sort(number_of_obstacles); // Esto de aquí ya no es la cantidad de obstáculos directa sino el orden de la cantidad de obstáculos

	    if(number_of_steps.size()) val = max_v(number_of_steps);
	    else return res;

	    vector<float> no(number_of_steps.size(), 0);

	    val = max_v(number_of_obstacles);
	    for(int i=0; i<no.size(); i++) no[i] = (float)number_of_obstacles[i]/val;

	    val = 2; // Esto es porque los valores de no estan normalizados

	    // Las posiciones libres tienen prioridad
	    increase_value(free_poss, val + 1, res); // Subir el gradiente por encima de los obstáculos

	    Poss<int> vol;
	    for(int i=0; i<number_of_steps.size(); i++){
	        ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
	        if((float)number_of_steps[i]/ap <= occupancy_threshold){ // área navegable
	            assign_value(areas_poss[i], val - no[i], res);
	            // Esto es para que el inflado del área tenga el valor del área dinámica
	            for(int j=0; j<areas_contours[i].x.size(); j++){
	                vol = agentPoss(areas_contours[i].x[j], areas_contours[i].y[j], rx, ry);
	                assign_value(vol, val - no[i], res);
	            }
	        }else{ // área no navegable
	            if(i != my_segment){
	                assign_value(areas_poss[i], 0, res);
	            }else{
	                assign_value(areas_poss[i], val - no[i], res);
	                // Esto es para que el inflado del área tenga el valor del área dinámica
	                for(int j=0; j<areas_contours[i].x.size(); j++){
	                    vol = agentPoss(areas_contours[i].x[j], areas_contours[i].y[j], rx, ry);
	                    assign_value(vol, val - no[i], res);
	                }
	            }
	        }
	    }

	    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

	    return res;
	} // Construcción del gradiente de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado

	vector<vector<float>> navigable_grad_oth_infl(vector<vector<float>> grad, Poss<int> free_poss, Poss<int> obst_poss, vector<Poss<int>> areas_poss, vector<Poss<int>> areas_contours, vector<vector<float>> infl_velocity_map, int radius, vector<int> number_of_obstacles, float occupancy_threshold, int my_segment)
	{
	    // grad: gradiente de obstáculos
	    // free_poss: posiciones libres
	    // obst_poss: posiciones de los obstáculos dinámicos
	    // areas_poss: posiciones de las áreas dinámicas
	    // areas_contours: posiciones unicamente de los contornos de las áreas (para los inflados)
	    // rx, ry: tamaño del robot
	    // number_of_obstacles: numero de posiciones que ocupan los obstáculos dentro de cada área dinámica
	    // occupancy_threshold: umbral de ocupación por debajo del cual un área se considera inatravesable
	    // my_segment: segmento en el que se encuentra el robot (para no quedar atascado en una partición no transitable)
	    vector<vector<float>> res = grad;
	    int val = 0, ap;

	    vector<int> number_of_steps = obstacles_sort(number_of_obstacles); // Esto de aquí ya no es la cantidad de obstáculos directa sino el orden de la cantidad de obstáculos

	    if(number_of_steps.size()) val = max_v(number_of_steps);
	    else return res;

	    vector<float> no(number_of_steps.size(), 0);

	    val = max_v(number_of_obstacles);
	    for(int i=0; i<no.size(); i++) no[i] = (float)number_of_obstacles[i]/val;

	    val = 2; // Esto es porque los valores de no estan normalizados

	    // Las posiciones libres tienen prioridad
	    increase_value(free_poss, val + 1, res); // Subir el gradiente por encima de los obstáculos

	    Poss<int> infl_poss;

	    Poss<int> vol;
	    for(int i=0; i<number_of_steps.size(); i++){
	    	if(number_of_obstacles[i] == 0) continue; // Es un área dinámica SIN obstáculos ---> área navegable como en espacio libre
	        ap = areas_poss[i].x.size() ? areas_poss[i].x.size() : 1;
	        if((float)number_of_steps[i]/ap <= occupancy_threshold){ // área navegable
	            assign_value(areas_poss[i], val - no[i], res);
	            // Esto es para que el inflado del área tenga el valor del área dinámica
	            inflatePoss(areas_contours[i], infl_velocity_map, radius, val - no[i], res);
	        }else{ // área no navegable
	            if(i != my_segment){
	                assign_value(areas_poss[i], 0, res);
	            }else{
	                assign_value(areas_poss[i], val - no[i], res);
	                // Esto es para que el inflado del área tenga el valor del área dinámica
	                inflatePoss(areas_contours[i], infl_velocity_map, radius, val - no[i], res);
	            }
	        }
	    }

	    assign_value(obst_poss, 0, res); // Para evitar los obstáculos

	    return res;
	} // Construcción del gradiente de navegación, asignando valores a las distintas regiones e incluyendo descartar algunas regiones en el caso de que haya un nivel de ocupación determinado


}

#endif