#ifndef EXPLORATION_FUNCTIONS_HPP
#define EXPLORATION_FUNCTIONS_HPP

#include <mada/util/funciones.hpp>
#include <mada/util/fmm_2.hpp>
#include <mada/util/generic_fmm.hpp>
#include <mada/util/graph_functions.hpp>
#include <mada/util/geometry.hpp>
#include <mada/util/cluster.hpp>
#include <mada/navigation_functions.hpp>

namespace exploration{

	void initialize_variables(Poss<int> &non_explored_points, vector<vector<bool>> &non_explored_map, vector<vector<float>> static_map)
	{
		non_explored_map.resize(static_map.size(), vector<bool>(static_map[0].size(), false));
		for(int i=0; i<non_explored_map.size(); i++)
			for(int j=0; j<non_explored_map[0].size(); j++)
				if(static_map[i][j]){
					non_explored_map[i][j] = true;
					non_explored_points.push(i,j);
				}
	} // Basicamente extraer las posiciones libres

	vector<vector<bool>> initialize_map(Poss<int> free_poss, int sx, int sy)
	{
		vector<vector<bool>> res(sx, vector<bool>(sy, false));
		for(int i=0; i<free_poss.x.size(); i++) res[free_poss.x[i]][free_poss.y[i]] = true;
		return res;
	}

	Poss<int> non_explored_points(vector<vector<bool>> non_explored_map)
	{
		Poss<int> res;
		for(int i=0; i<non_explored_map.size(); i++)
			for(int j=0; j<non_explored_map[0].size(); j++)
				if(non_explored_map[i][j] == true) res.push(i,j);
		return res;
	}

	void remove_explored_points(vector<vector<bool>> &non_explored_map, Poss<int> visible_poss)
	{
	    for(int i=0; i<visible_poss.x.size(); i++)
	        non_explored_map[visible_poss.x[i]][visible_poss.y[i]] = true;
	}

	void remove_explored_points(Poss<int> &non_explored_points, vector<vector<bool>> &non_explored_map, Poss<int> visible_poss)
	{
	    while(visible_poss.x.size()){
	        if(non_explored_map[visible_poss.x[0]][visible_poss.y[0]] == false){
	            non_explored_map[visible_poss.x[0]][visible_poss.y[0]] = true;
	            for(int i=0; i<non_explored_points.x.size(); i++){
	                if(visible_poss.x[0] == non_explored_points.x[i] && visible_poss.y[0] == non_explored_points.y[i]){
	                    non_explored_points.erase(i);
	                    break;
	                }
	            }
	        }
	        visible_poss.erase(0);
	    }
	}

	void merge_non_explored_maps(vector<vector<vector<bool>>> &maps)
	{
		for(int i=0; i<maps[0].size(); i++)
			for(int j=0; j<maps[0][0].size(); j++)
				for(int k=0; k<maps.size(); k++){
					if(maps[k][i][j]){
						for(int k_=0;  k_<maps.size(); k_++) maps[k_][i][j] = true;
						break;
					}
				}
	} // Unificar los mapas de los puntos no explorados

	void merge_non_explored_maps(vector<vector<vector<bool>>> &maps, vector<int> groups)
	{
		for(int i=0; i<maps[0].size(); i++)
			for(int j=0; j<maps[0][0].size(); j++)
				for(int k=0; k<maps.size(); k++){
					if(maps[k][i][j]){
						for(int k_=0;  k_<maps.size(); k_++)
							if(groups[k] == groups[k_]) maps[k_][i][j] = true;
					}
				}
	} // Unificar los mapas de los puntos no explorados unicamente entre los mismos grupos

	float explored_area(vector<vector<float>> seen_map, vector<vector<float>> map)
	{
		float res = 0;
		int Ntot = 0;
		int Nseen = 0;
		if(seen_map.size() != map.size()) return -1;
		for(int i=0; i<seen_map.size(); i++)
			for(int j=0; j<seen_map[i].size(); j++)
				if(map[i][j] < INF && map[i][j]){
					Ntot++;
					if(seen_map[i][j] > 0){
						Nseen++;
					}
				}
		return (float)Nseen/(float)Ntot;
	}

	vector<float> number_of_expl_points(vector<float> min_dist, vector<float> max_dist, float vr)
	{
		// - min_dist: distancia del centroide al punto más cercano del área
		// - max_dist: distancia del centroide al punto más lejano del área
		// - vr: rango de visión
	    vector<float> res(min_dist.size(), 0);
	    for(int i=0; i<res.size(); i++){
	        //res[i] = ceil(min_dist[i]/vr) * ceil(max_dist[i]/vr);
	        res[i] = ceil(min_dist[i]/(2*vr)) * ceil(max_dist[i]/(2*vr));
	    }
	    return res;
	}


	class Map_exploration{
		
		private:

			bool known_map;
			int sx, sy;
			vector<vector<float>> static_map;
			vector<vector<float>> env_grad;
			vector<vector<float>> grad;
			float vrange;
			vector<vector<float>> estimated_utility_map;
			vector<vector<float>> utility_map;

			vector<vector<float>> seen_map;

			int non_explored_points = 0;
		
		public:

			Map_exploration(vector<vector<float>> grid, vector<vector<float>> grad, float vrange)
			{
				static_map = grid;
				env_grad = grad;
				this->vrange = vrange;
				sx = grid.size(); sy = grid[0].size();

				known_map = true;

				seen_map.resize(sx, vector<float>(sy, -1));

				utility_map.resize(sx, vector<float>(sy,-1));
				estimated_utility_map.resize(sx, vector<float>(sy,-1));
				for(int i=0; i<sx; i++)
					for(int j=0; j<sy; j++)
						if(!static_map[i][j]){
							utility_map[i][j] = 0;
							estimated_utility_map[i][j] = 0;
							seen_map[i][j] = 0;
						}else non_explored_points++;
			} // Inicializar las variables CON mapa del entorno conocido

			void set(vector<vector<float>> grid, vector<vector<float>> grad, float vrange)
			{
				static_map = grid;
				env_grad = grad;
				this->vrange = vrange;
				sx = grid.size(); sy = grid[0].size();

				known_map = true;

				seen_map.clear();
				utility_map.clear();
				estimated_utility_map.clear();

				seen_map.resize(sx, vector<float>(sy, -1));

				utility_map.resize(sx, vector<float>(sy,-1));
				estimated_utility_map.resize(sx, vector<float>(sy,-1));
				for(int i=0; i<sx; i++)
					for(int j=0; j<sy; j++)
						if(!static_map[i][j]){
							utility_map[i][j] = 0;
							estimated_utility_map[i][j] = 0;
							seen_map[i][j] = 0;
						}else non_explored_points++;
			} // Fijar las variables CON mapa del entorno conocido

			Map_exploration(vector<vector<float>> grid, float vrange)
			{
				Map_exploration(grid, grid, vrange);
			}

			Map_exploration(int sx, int sy, float vrange)
			{
				this->sx = sx;
				this->sy = sy;
				this->vrange = vrange;

				known_map = false;

				static_map.resize(sx, vector<float>(sy, 1));
				env_grad.resize(sx, vector<float>(sy,-1));

				utility_map.resize(sx, vector<float>(sy,-1));
				estimated_utility_map.resize(sx, vector<float>(sy,-1));

				seen_map.resize(sx, vector<float>(sy, -1));

			} // Inicializar las variables SIN mapa del entorno conocido

			Map_exploration(float vrange)
			{
				Map_exploration(0, 0, vrange);
			}

			void resize_maps(bool ndim, int new_size)
			{
				if(!ndim){
					sx = new_size;
					static_map.resize(new_size, vector<float>(sy, 1));
					env_grad.resize(new_size, vector<float>(sy,-1));
					utility_map.resize(new_size, vector<float>(sy,-1));
					estimated_utility_map.resize(new_size, vector<float>(sy,-1));
					seen_map.resize(new_size, vector<float>(sy, -1));
				}else{
					sy = new_size;
					for(int i=0; i<sx; i++){
						static_map[i].resize(new_size, 1);
						env_grad[i].resize(new_size, -1);
						utility_map[i].resize(new_size, -1);
						estimated_utility_map[i].resize(new_size, -1);
						seen_map[i].resize(new_size, -1);
					}
				}
			}

			void initialize_estimated_map()
			{
				if(known_map){ // Mapa conocido
					// Obtener las utilidades, basadaas en el rango de visión, de cada celda del grid
					for(int i=0; i<sx; i++)
						for(int j=0; j<sy; j++)
							if(static_map[i][j])
								estimated_utility_map[i][j] = geometry::circle_poss_on_grid(i, j, vrange, static_map).x.size();
				}
			}

			void update_maps(int x, int y, Poss<int> visible_poss, vector<float> poss_values)
			{
				if(!known_map){ // Si el mapa no era conocido, actualizar
					int max;
					max = max_v(visible_poss.x);
					if(max >= sx) this->resize_maps(0, max+1);
					max = max_v(visible_poss.y);
					if(max >= sy) this->resize_maps(1, max+1);
					for(int i=0; i<visible_poss.x.size(); i++){
						if(seen_map[visible_poss.x[i]][visible_poss.y[i]] < 0) non_explored_points--;
						static_map[visible_poss.x[i]][visible_poss.y[i]] = poss_values[i];
					}
				}

				for(int i=0; i<visible_poss.x.size(); i++){
					if(seen_map[visible_poss.x[i]][visible_poss.y[i]] < 0) non_explored_points--;
					seen_map[visible_poss.x[i]][visible_poss.y[i]] = poss_values[i];
				}

				utility_map[x][y] = visible_poss.x.size();

				// Actualizar el valor de las utilidades estimadas
				estimated_utility_map[x][y] = estimated_utility_map[x][y] < utility_map[x][y] ? utility_map[x][y] : estimated_utility_map[x][y];
			}

			Path path2next_goal(int xstart, int ystart, vector<vector<float>> navigation_grid)
			{
				Path res;

				Poss<int> covered;

				if(known_map){
					FMM gr(xstart, ystart, navigation_grid);
					grad = gr.expand2map_value(1, -1, seen_map, covered); // Exploración → ir en direción al punto "no visto" más cercano
				}else{
					FMM gr(xstart, ystart, static_map);
					grad = gr.expand2map_value(1, -1, seen_map, covered); // Exploración → ir en direción al punto "no visto" más cercano
				}

				if(covered.x.size()){
					res.gradient_descent_(grad, covered.x[0], covered.y[0]); // Camino de (xstart, ystart) -> (xgoal, ygoal) "(covered.x[0], covered.y[0])"
				}else{
					float min_dist = INF;
					covered(1);
					for(int i=0; i<seen_map.size(); i++){
						for(int j=0; j<seen_map[0].size(); j++){
							if(seen_map[i][j] == -1 && min_dist > grad[i][j]){
								covered.x[0] = i; covered.y[0] = j;
								min_dist = grad[i][j];
							}
						}
					}
					if(min_dist < INF)
						res.gradient_descent_(grad, covered.x[0], covered.y[0]); // Camino de (xstart, ystart) -> (xgoal, ygoal) "(covered.x[0], covered.y[0])"
				}

				return res;
			}

			vector<vector<float>> get_utilities_map()
			{
				return utility_map;
			}

			vector<vector<float>> get_seen_map()
			{
				return seen_map;
			}

			vector<vector<float>> get_estimated_utilities_map()
			{
				return estimated_utility_map;
			}

			vector<vector<float>> get_static_map()
			{
				return static_map;
			}

			int Nremaining_poss2explore()
			{
				return non_explored_points;
			}

			vector<vector<float>> get_gradient()
			{
				return grad;
			}

	};

	class Areas_exploration{
		private:
			int sx, sy;
			vector<vector<float>> seen_map;
			vector<vector<float>> real_seen_map;
			vector<vector<float>> static_map;
			float vrange;
			vector<Poss<int>> poss2explore, poss2explore_ini;
			vector<Poss<int>> target_poss;
			vector<vector<int>> areas_map, areas_map_ini;
			vector<vector<float>> grad;
			int target_area = -1;
			int target_set = -1;

			bool next_goal = false;
			int xg, yg; // Posición el goal

			// Variables asociadas al árbol/grafo
			vector<vector<bool>> areas_graph;
			vector<vector<int>> all_branches;
			vector<float> branches_costs;
			int selected_branch = 0;
			vector<float> areas_tree_costs;
			
		public:

			Areas_exploration(){}

			Areas_exploration(vector<Poss<int>> areas_poss, vector<vector<int>> areas_map, vector<vector<float>> static_map, float vrange)
			{
				this->sx = static_map.size(); this->sy = static_map[0].size();
				this->static_map = static_map;
				this->areas_map = areas_map;
				this->areas_map_ini = areas_map;
				this->vrange = vrange;
				poss2explore = areas_poss;
				poss2explore_ini = areas_poss;
				next_goal = false;
				seen_map.resize(sx, vector<float>(sy, -1));
				for(int i=0; i<sx; i++)
					for(int j=0; j<sy; j++)
						if(static_map[i][j] == 0) seen_map[i][j] = 0;
				real_seen_map = seen_map;
			}

			void set(vector<Poss<int>> areas_poss, vector<vector<int>> areas_map, vector<vector<float>> static_map, float vrange)
			{
				this->sx = static_map.size(); this->sy = static_map[0].size();
				this->static_map = static_map;
				this->areas_map = areas_map;
				this->areas_map_ini = areas_map;
				this->vrange = vrange;
				poss2explore = areas_poss;
				poss2explore_ini = areas_poss;
				next_goal = false;
				seen_map.clear();
				seen_map.resize(sx, vector<float>(sy, -1));
				for(int i=0; i<sx; i++)
					for(int j=0; j<sy; j++)
						if(static_map[i][j] == 0) seen_map[i][j] = 0;
				real_seen_map = seen_map;
			}

			void select_closest_area(int x, int y, vector<vector<float>> navigation_grid)
			{
				// Calcular gradiente hasta el primer punto "no visto"
				Poss<int> point;
				FMM gr(x, y, navigation_grid);
				grad = gr.expand2map_value(1, -1, seen_map, point);
				if(point.x.size()){ // Se ha alcanzado el punto no visto
					target_area = areas_map[point.x[0]][point.y[0]];
					// Selección del conjunto de los puntos que se van a explorar
					//target_poss = adjacency_clustering::cluster(poss2explore[target_area], static_map);
				}
			} // Seleccionar el área que se va a visitar

			void select_closest_area_(int x, int y, vector<vector<float>> navigation_grid)
			{
				Poss<int> point, poss = join_poss(poss2explore);
				FMM gr(x, y, navigation_grid);
				grad = gr.pos_coverage(1, poss, point);
				if(point.x.size()){
					target_area = areas_map[point.x[0]][point.y[0]];
				}
			}

			void select_next_area(int x, int y, vector<vector<bool>> areas_graph)
			{
				int current_area;

				if(areas_map[x][y] >= 0) current_area = areas_map[x][y];
				else current_area = areas_map_ini[x][y];

				if(poss2explore[current_area].x.size() == 0){
					int min = RAND_MAX;
					for(int i=0; i<poss2explore.size(); i++){
						//if(areas_graph[i][current_area]) cout<<current_area<<" -> "<<i<<endl;
						// NO es el mismo segmento, es vecino, que todavía contiene puntos por explorar, tiene menos puntos que los demás
						if(i != current_area && areas_graph[i][current_area] && poss2explore[i].x.size() && min > poss2explore[i].x.size()){
							min = poss2explore[i].x.size();
							target_area = i;
						}
					}
				}else{
					target_area = current_area;
				}
			}

			void select_next_area(int x, int y, vector<vector<bool>> areas_graph, vector<vector<float>> navigation_grid)
			{
				select_next_area(x, y, areas_graph); // Seleccionar área adyacente que tiene menos puntos (de área más pequeña)
				// Si no hay vecinos seleccionar área más cercana
				if(target_area < 0) select_closest_area(x, y, navigation_grid);
			}

			vector<vector<vector<int>>> compute_tree(int x, int y, vector<vector<bool>> areas_graph)
			{
				vector<vector<float>> distances_between_areas(areas_graph.size(), vector<float>(areas_graph.size(), 0));
				for(int i=0; i<areas_graph.size(); i++)
					for(int j=i+1; j<areas_graph.size(); j++)
						if(areas_graph[i][j]) distances_between_areas[i][j] = distances_between_areas[j][i] = 1;
				
				return tree_functions::generate_tree_from_graph_dw_cm(areas_map_ini[x][y], areas_graph, distances_between_areas);
			} // Obtener el árbol (enlaces/aristas organidas por profundidades)

			int descend_tree_fd(vector<vector<vector<int>>> tree_bd, vector<float> costs)
			{
				int res = -1;
				int min = RAND_MAX;
				for(int i=0; i<tree_bd.size(); i++){ // Cada profundidad
					for(int j=0; j<tree_bd[i].size(); j++){ // Cada arista
						if(poss2explore[tree_bd[i][j][1]].x.size() && min > costs[tree_bd[i][j][1]]){
							min = costs[tree_bd[i][j][1]];
							res = tree_bd[i][j][1];
						}
					}
					if(res >= 0) return res;
				}
				return res;
			} // Descender el árbol desde la raíz hasta la primera profundidad donde queden puntos por explorar

			int descend_tree(vector<vector<vector<int>>> tree_bd, vector<float> costs)
			{
				int res = -1;
				int min = RAND_MAX;
				for(int i=0; i<tree_bd.size(); i++){ // Cada profundidad
					for(int j=0; j<tree_bd[i].size(); j++){ // Cada arista
						if(poss2explore[tree_bd[i][j][1]].x.size() && min > costs[tree_bd[i][j][1]]){
							min = costs[tree_bd[i][j][1]];
							res = tree_bd[i][j][1];
						}
					}
					//if(res >= 0) return res;
				}
				return res;
			} // Descender el árbol desde la raíz hasta la primera profundidad donde queden puntos por explorar de todo el árbol

			int select_first_depth_target(vector<vector<vector<int>>> tree_bd, vector<float> costs)
			{
				int res = -1;
				int min = RAND_MAX;
				if(tree_bd.size())
				for(int i=0; i<tree_bd[0].size(); i++)
					if(poss2explore[tree_bd[0][i][1]].x.size() && min > costs[tree_bd[0][i][1]]){
						min = costs[tree_bd[0][i][1]];
						res = tree_bd[0][i][1];
					}
				return res;
			}

			// *****************************************************************************************************
			// *****************************************************************************************************
			// *****************************************************************************************************

			template <typename T>
			vector<int> select_first_vertices(int root, vector<vector<bool>> tree_matrix, vector<T> costs)
			{
				vector<int> res;

				if(costs[root]){
					res.push_back(root);
					return res;
				}

				queue<int> q;
				q.push(root);

				while(!q.empty()){
					//cout<<"cola: "<<q.size()<<" - "<<q.front()<<"/"<<npoints.size()<<"/"<<tree_matrix.size()<<endl;
					for(int i=0; i<costs.size(); i++){
						if(q.front()!=i && tree_matrix[q.front()][i]){
							if(costs[i]){
								if(!costs[q.front()]) res.push_back(i);
							}else{
								q.push(i);
								tree_matrix[i][q.front()] = tree_matrix[q.front()][i] = false;
							}
						}
					}
					q.pop();
				}
				
				return res;
			} // Seleccionar los primeros vértices en los queden puntos por explorar

			int lowest_cost_selection(vector<vector<int>> tree, vector<int> npoints, vector<float> branch_costs)
			{
				int res = -1;
				vector<int> candidates = select_first_vertices(tree[0][0], tree_functions::ind2matr(tree, npoints.size()), npoints);
				float min = INF;
				//sh_vect_h(candidates,"candidatos");
				for(int i=0; i<candidates.size(); i++)
					if(min > branch_costs[candidates[i]]){
						res = candidates[i];
						min = branch_costs[candidates[i]];
					}
				//cout<<"seleccionado "<<res<<" con "<<npoints[res]<<" puntos y con coste "<<min<<endl;
				return res;
			}
 
			// *****************************************************************************************************
			// *****************************************************************************************************
			// *****************************************************************************************************

			vector<float> obtain_tree_costs(vector<vector<bool>> areas_graph, vector<int> areas_depths, vector<vector<float>> navigation_grid)
			{
				int n = areas_graph.size();
				vector<float> res(n,0);
				vector<int> npoints(n, 0); int max_np = 0;
				vector<float> navigation_var(n, 0); float max_nav = 0;
				for(int i=0; i<n; i++){
					npoints[i] = poss2explore[i].x.size()*areas_depths[i];
					if(max_np < npoints[i]) max_np = npoints[i];
					navigation_var[i] = navigation_grid[poss2explore_ini[i].x[0]][poss2explore_ini[i].y[0]];
					if(max_nav < navigation_var[i]) max_nav = navigation_var[i];
				}
				
				if(max_nav > 1){
					for(int i=0; i<n; i++){
						npoints[i] /= max_np;
						navigation_var[i] = 1 - navigation_var[i]/max_nav;
						res[i] = navigation_var[i]*(float)npoints[i];
					}
				}else{
					for(int i=0; i<n; i++){
						res[i] = (float)npoints[i];
					}
				}
				return res;
			}

			vector<float> obtain_tree_costs(vector<vector<bool>> areas_graph, vector<vector<float>> navigation_grid)
			{
				int n = areas_graph.size();
				vector<float> res(n,0);
				vector<int> npoints(n, 0); int max_np = 0;
				vector<float> navigation_var(n, 0); float max_nav = 0;
				for(int i=0; i<n; i++){
					npoints[i] = poss2explore[i].x.size();
					if(max_np < npoints[i]) max_np = npoints[i];
					if(navigation_grid[poss2explore_ini[i].x[0]][poss2explore_ini[i].y[0]]){
						navigation_var[i] = navigation_grid[poss2explore_ini[i].x[0]][poss2explore_ini[i].y[0]];
						if(max_nav < navigation_var[i]) max_nav = navigation_var[i];
					}
				}
				if(max_nav > 1 && max_np){
					for(int i=0; i<n; i++){
						npoints[i] /= max_np;
						navigation_var[i] = 1 - navigation_var[i]/max_nav;
						res[i] = navigation_var[i] + (float)npoints[i];
					}
				}else{
					for(int i=0; i<n; i++){
						res[i] = (float)npoints[i];
					}
				}
				return res;
			}

			vector<float> obtain_tree_costs(vector<int> tree_depths)
			{
				int n = poss2explore.size();
				vector<float> npoints(n, 0);
				for(int i=0; i<n; i++)
					npoints[i] = poss2explore[i].x.size() * tree_depths[i];

				return npoints;
			}

			vector<float> obtain_tree_costs(vector<Poss<int>> positions)
			{
				vector<float> npoints(positions.size(), 0);
				for(int i=0; i<positions.size(); i++)
					npoints[i] = positions[i].x.size();
				return npoints;
			}

			vector<float> points_to_see()
			{
				int n = poss2explore.size();
				vector<float> npoints(n, 0);
				for(int i=0; i<n; i++)
					npoints[i] = poss2explore[i].x.size();

				return npoints;
			}

			vector<float> obtain_area_costs(vector<float> segment_obs_points)
			{
				vector<float> res(poss2explore.size(), 0);
				for(int i=0; i<poss2explore.size(); i++){
					if(!poss2explore[i].x.size()){ // Se ha visto todo el área
						res[i] = 0; // Coste cero
					}else{ // Quedan puntos por ver
						if(segment_obs_points[i] == 1){ // Se ve todo el segmento desde el centroide
							res[i] = 1;
						}else if(segment_obs_points[i] > 1){
							res[i] = vrange * (ceil(((float)poss2explore[i].x.size()/(float)poss2explore_ini[i].x.size())*segment_obs_points[i])-1);
						}
					}
					//res[i] = ceil(segment_obs_points[i] * (float)poss2explore[i].x.size()/(float)poss2explore_ini[i].x.size());
				}
				return res;
			} // Costes de trabajo de los segmentos basados en el área que queda por "explorar"

			vector<float> obtain_tree_costs(vector<vector<int>> tree, vector<vector<float>> costs_between_segments)
			{
				int n = poss2explore.size();
				vector<float> res(n, 0);
				vector<float> dist(n, 0);
				float md = 0, mp = 0;
				for(int i=0; i<tree.size(); i++){
					dist[tree[i][1]] = costs_between_segments[tree[i][0]][tree[i][1]];
					res[tree[i][1]] = poss2explore[tree[i][1]].x.size();
					if(md < dist[tree[i][1]]) md = dist[tree[i][1]];
					if(mp < res[tree[i][1]]) mp = res[tree[i][1]];
				}
				for(int i=0; i<n; i++){
					res[i] = 0.5*res[i]/mp + 0.5*dist[i]/md;
				}
				return res;
			}

			void select_closest_set(int x, int y, vector<vector<float>> navigation_grid)
			{
				target_set = -1; // ???????????
				if(target_area < 0) return;

				// Selección del conjunto de los puntos que se van a explorar
				target_poss = adjacency_clustering::cluster(poss2explore[target_area], static_map);

				int min = RAND_MAX;
				for(int i=0; i<target_poss.size(); i++){
					if(target_poss[i].x.size() && min > target_poss[i].x.size()){
						min = target_poss[i].x.size();
						target_set = i;
					}
				}
			} // Seleccionar el conjunto de puntos (del área/segmento) que se va a visitar

			void select_closest_point()
			{
				target_poss.clear();
				target_poss.push_back(poss2explore[target_area]);
				target_set = 0;
			} // Para poder seleccionar el punto más cercano, no visto, del área objetivo


			Poss<int> select_closest_point(int x, int y, vector<vector<float>> navigation_grid){
				// Calcular gradiente hasta el primer punto "no visto"
				Poss<int> point;
				FMM gr(x, y, navigation_grid);
				grad = gr.expand2map_value(1, -1, seen_map, point);
				return point;
			}

			Path path2closest_point(int x, int y, vector<vector<float>> navigation_grid)
			{
				Path res;
				//cout<<"uniendo ..."<<endl;
				Poss<int> point;
				//Poss<int> poss = join_poss(poss2explore);
				//cout<<"unidos"<<endl;
				FMM gr(x, y, navigation_grid);
				//grad = gr.pos_coverage(1, poss, point);
				grad = gr.expand2map_value(1, -1, seen_map, point);
				//cout<<"calculado "<<point.x.size()<<endl;
				if(point.x.size()){
					//cout<<"VALOR DEL PUNTO: "<<navigation_grid[point.x[0]][point.y[0]]<<endl;
					res.gradient_descent_(grad, point.x[0], point.y[0]);
					/*
					for(int i=0; i<res.tam; i++)
						cout<<seen_map[res.x[i]][res.y[i]]<<", ";
					cout<<endl;
					*/
				}
				return res;
			}

			Path path2next_goal(int x, int y, vector<vector<float>> navigation_grid)
			{
				Path res;

				if(target_set < 0) return res;

				Poss<int> point;
				FMM gr(x, y, navigation_grid);
				grad = gr.pos_coverage(1, target_poss[target_set], point);
				if(point.x.size()){
					res.gradient_descent_(grad, point.x[0], point.y[0]);
				}

				return res;
			}

			void compute_tree_variables(int x, int y, vector<vector<bool>> areas_graph, vector<vector<float>> costs_between_segments, vector<vector<int>> &tree, vector<float> &branch_costs)
			{
				tree.clear();
				// CONSTRUCCIÓN DEL ÁRBOL DE LOS SEGMENTOS
				vector<int> npoints(poss2explore.size(), 0);
				for(int i=0; i<poss2explore.size(); i++) npoints[i] = poss2explore[i].x.size();
				vector<vector<vector<int>>> tree_bd = tree_functions::generate_tree_from_graph_uw_cv(areas_map_ini[x][y], areas_graph, npoints);
				//vector<vector<vector<int>>> tree_bd = tree_functions::generate_tree_from_graph_dw(areas_map_ini[x][y], areas_graph);

				vector<int> depths(areas_graph.size(), -1);
				for(int i=0; i<tree_bd.size(); i++)
					for(int j=0; j<tree_bd[i].size(); j++){
						tree.push_back(tree_bd[i][j]);
						depths[tree_bd[i][j][0]] = i;
						depths[tree_bd[i][j][1]] = i+1;
					}
				// OBTENCIÓN DE LOS COSTES DE LAS RAMAS DEL ÁRBOL
				//vector<float> branch_costs = points_to_see(); // nº puntos
				//branch_costs = obtain_tree_costs(tree, costs_between_segments); // nº puntos + distancias
				branch_costs = points_to_see(); // nº puntos

				// Acumular los costes de las ramas
				branch_costs = tree_functions::accumulate_tree_variable(tree_functions::all_paths_ascend(tree), branch_costs);

				// SELECCIÓN DEL ÁREA OBJETIVO
				target_area = select_first_depth_target(tree_bd, branch_costs);
				//target_area = descend_tree_fd(tree_bd, branch_costs);
			}

			int select_target_area(vector<float> npoints, vector<float> branch_costs, vector<vector<int>> all_paths)
			{
				int res = -1;
				int ind = -1;
				int isel; // Vértice seleccionado
				int cont = 1;
				float cost;
				vector<bool> todo(all_paths.size(), true);
				while(res < 0){
					// Encontrar el nodo de menor coste
					cost = INFINITY;
					isel = -1;
					for(int i=0; i<all_paths.size(); i++){
						if(todo[i]){
							ind = all_paths[i].size() - cont;
							if(ind>0){
								if(res < 0){ // Todavía no ha habido ningún segmento por ver
									if(npoints[all_paths[i][ind]]){
										res = all_paths[i][ind];
									}else{
										if(cost > branch_costs[all_paths[i][ind]]){
											cost = branch_costs[all_paths[i][ind]];
											isel = all_paths[i][ind];
											// hay que seleccionar las ramas que tienen este vértice
										}
									}
								}else{ // Ya se ha encontrado un segmento que hay que ver
									if(npoints[all_paths[i][ind]] && cost > branch_costs[all_paths[i][ind]]){
										cost = branch_costs[all_paths[i][ind]];
										res = all_paths[i][ind];
									}
								}
							}
						}
					}

					if(res >= 0){
						return res;
					}else{
						// seleccionar las ramas que tienen este vértice
						bool f = false;
						for(int i=0; i<all_paths.size(); i++){
							for(int j=0; j<all_paths.size(); j++){
								if(all_paths[i][j] == isel){
									f = true;
									break;
								}
							}
							if(!f) todo[i] = false;
						}
					}

					cont++;
				}

				return res;
			}

			vector<float> accumulate_path_costs(vector<vector<int>> all_paths, vector<float> vertex_costs, vector<vector<float>> edge_costs)
			{
				vector<float> res(all_paths.size(), 0);
				for(int i=0; i<all_paths.size(); i++){
					for(int j=0; j<all_paths[i].size()-1; j++){
						res[i] += (vertex_costs[all_paths[i][j]] + edge_costs[all_paths[i][j]][all_paths[i][j+1]]);
					}
				}
				return res;
			}

			vector<float> _accumulate_vertex_costs(vector<vector<int>> all_paths, vector<float> vertex_costs)
			{
				vector<float> res(vertex_costs.size(), 0);
				for(int i=0; i<all_paths.size(); i++){
					for(int j=0; j<all_paths[i].size(); j++){
						res[all_paths[i][j]] += vertex_costs[all_paths[i][j]];
					}
				}
				return res;
			}

			vector<float> accumulate_vertex_costs(vector<vector<int>> all_paths, vector<float> vertex_costs)
			{
				vector<float> res(all_paths.size(), 0);
				for(int i=0; i<all_paths.size(); i++){
					for(int j=0; j<all_paths[i].size(); j++){
						res[i] += vertex_costs[all_paths[i][j]];
					}
				}
				return res;
			}

			vector<float> accumulate_edge_costs(vector<vector<int>> all_paths, vector<vector<float>> edge_costs)
			{
				vector<float> res(all_paths.size(), 0);
				for(int i=0; i<all_paths.size(); i++){
					for(int j=0; j<all_paths[i].size()-1; j++){
						res[i] += edge_costs[all_paths[i][j]][all_paths[i][j+1]];
					}
				}
				return res;
			}


			vector<float> accumulate_tree_costs(vector<vector<int>> all_paths, vector<float> vertex_costs, vector<vector<float>> edge_costs)
			{
				vector<float> res(vertex_costs.size(), 0);

				for(int i=0; i<all_paths.size(); i++){
					for(int j=all_paths[i].size()-1; j>0; j--){
						res[all_paths[i][j]] += (vertex_costs[all_paths[i][j]] + edge_costs[all_paths[i][j]][all_paths[i][j-1]]);
					}
				}

				return res;
			}

			int select_branch(vector<float> branch_costs)
			{
				int res = -1;
				float cost = INFINITY;
				for(int i=0; i<branch_costs.size(); i++){
					if(branch_costs[i] && cost > branch_costs[i]){
						cost = branch_costs[i];
						res = i;
					}
				}
				return res;
			}

			int select_branch(vector<float> vertex_costs, vector<float> edge_costs)
			{
				int res = -1;
				float cost = INFINITY;
				float min_cost = INFINITY;
				for(int i=0; i<vertex_costs.size(); i++){
					if(vertex_costs[i]){ // Rama que todavía no se ha finalizado (no se han visto todas las particiones)
						cost = vertex_costs[i] + edge_costs[i];
						if(min_cost > cost){
							min_cost = cost;
							res = i;
						}
					}
				}
				return res;
			}

			int select_branch(vector<float> vertex_costs, vector<float> edge_costs, vector<float> &costs)
			{
				int res = -1;
				float cost = INFINITY;
				float min_cost = INFINITY;
				for(int i=0; i<vertex_costs.size(); i++){
					if(vertex_costs[i]){ // Rama que todavía no se ha finalizado (no se han visto todas las particiones)
						cost = vertex_costs[i] + edge_costs[i];
						costs[i] = vertex_costs[i];
						if(min_cost > cost){
							min_cost = cost;
							res = i;
						}
					}
				}
				return res;
			}

			int select_maxu_branch(vector<float> vertex_costs, vector<float> edge_costs, vector<float> &costs)
			{
				int res = -1;
				float cost = -INFINITY;
				float max_cost = -INFINITY;
				for(int i=0; i<vertex_costs.size(); i++){
					if(vertex_costs[i]){ // Rama que todavía no se ha finalizado (no se han visto todas las particiones)
						cost = vertex_costs[i] + edge_costs[i];
						costs[i] = vertex_costs[i];
						if(max_cost < cost){
							max_cost = cost;
							res = i;
						}
					}
				}
				return res;
			}

			int get_target_from_path(vector<int> path, vector<float> nv_points)
			{
				for(int i=path.size()-1; i>=0; i--)
					if(nv_points[path[i]])
						return path[i];
				return -1;
			}

			int get_target_from_paths(vector<float> branch_costs, vector<vector<int>> paths, vector<float> nv_points)
			{
				int res = -1;
				// Encontrar el camino menos costoso
				float cost = INFINITY;
				int ind = -1;
				for(int i=0; i<branch_costs.size(); i++){
					if(cost > branch_costs[i]){
						cost = branch_costs[i];
						ind = i;
					}
				}

				// Seleccionar el primer área por ver del camino
				if(ind >= 0){
					for(int i=paths[ind].size()-1; i>=0; i--)
						if(nv_points[paths[ind][i]])
							return paths[ind][i];
				}

				return res;
			}

			vector<float> compute_areas_tree_costs(vector<vector<bool>> areas_graph, vector<vector<int>> branches, vector<float> branches_costs)
			{
				vector<float> res(areas_graph.size(), 0);

				for(int i=0; i<branches.size(); i++){
					for(int j=0; j<branches[i].size(); j++){
						res[branches[i][j]] = branches_costs[i];
					}
				}

				return res;
			}

			void compute_tree(int x, int y, vector<vector<bool>> areas_graph, vector<vector<float>> costs_between_segments, vector<float> segment_obs_points, vector<vector<int>> &tree, vector<float> &branch_costs, vector<int> &branch)
			{
				tree.clear();
				branch.clear();
				
				vector<float> vertex_costs = obtain_area_costs(segment_obs_points); // Coste de trabajo dentro del área
				tree = tree_functions::build_tree(areas_map_ini[x][y], areas_graph, costs_between_segments, vertex_costs);
				
				// Obtener todos los caminos desde los vértices más profundos ("hojas") hasta la raíz (posición del robot)
				vector<vector<int>> all_paths = tree_functions::all_paths_ascend(tree);
				//vector<vector<int>> all_paths = tree_functions::all_paths_descend(tree);
				//cout<<"caminos calculados"<<endl;
				// Acumular los costes totales de los caminos (llegar + trabajar)
				//branch_costs = accumulate_path_costs(all_paths, vertex_costs, costs_between_segments); // <---
				//vector<float> vc = accumulate_vertex_costs(all_paths, vertex_costs);
				vector<float> vc = accumulate_vertex_costs(all_paths, _accumulate_vertex_costs(all_paths, vertex_costs));
				vector<float> ec = accumulate_edge_costs(all_paths, costs_between_segments);

				// El objetivo es el primer área con puntos por ver, descendiendo el árbol
				// seleccionando en cada profundidad la rama de menor coste

				// Obtener la cantidad de puntos que quedan por explorar en cada área
				//vector<float> nv_points = points_to_see();
				//sh_vect_h(nv_points, "resto");
				//target_area = get_target_from_paths(branch_costs, all_paths, nv_points);
				vector<float> total_costs(all_paths.size(), 0); // Costes totales de las ramas
				//int ind = select_branch(branch_costs);
				//int ind = select_branch(vc, ec);
				int ind = select_branch(vc, ec, total_costs);

				// Almacenar las variables del grafo/árbol <--- ESTO ES UNA CHAPUZA, QUITARLO DE AQUÍ
				this->areas_graph = areas_graph;
				this->all_branches = all_paths;
				this->branches_costs = total_costs;
				this->selected_branch = ind;
				this->areas_tree_costs = compute_areas_tree_costs(areas_graph, all_branches, branches_costs);

				//cout<<"RAMA "<<ind<<endl;
				if(ind >= 0){
					branch = all_paths[ind];
					target_area = get_target_from_path(branch, points_to_see());
					branch = reverse(branch);
					/*
					// Almacenar la rama como árb
					branches(reverse(all_paths[ind]));
					// Almacenar el coste de la rama como vector
					float bc = branch_costs[ind];
					branch_costs.clear(); branch_costs.push_back(bc);
					*/
				}else{
					target_area = -1;
				}	
			}

			void area_from_path(Path path)
			{
				if(path.tam){
					xg = path.x[path.tam-1]; yg = path.y[path.tam-1];
					//target_area = areas_map[xg][yg];
					target_area = areas_map_ini[xg][yg];
					next_goal = true;
				}
			} // Actualizar las variables de objetivo a partir del caminos

			int action_selection(int x, int y, bool select_centroid, int xc, int yc)
			{
				// - 0: no hacer nada, seguir camino
				// - 1: selecctionar nuevo área
				// - 2: selecctionar nuevo objetivo dentro del área
				// - 3: no hacer nada, seguir camino (NO tiene sentido pero paso de quitar esto, no vaya a ser que la lie)

				//if(!next_goal) return 1; // No había goal

				if(!poss2explore[target_area].x.size()){ // Se ha terminado con el área objetivo, seleccionar área
					return 1;
					//if(!centroid_selection) return 1; // Seleccionar conjunto más cercano
					//else return 2; // Seleccionar centroide
				}

				if(!select_centroid){ // No iba al centroide
					//cout<<"NO iba al centroide"<<endl;
					if(areas_map[xg][yg]<0) return 2; // Se ha visto el objetivo, seleccionar nuevo objetivo
				}else{ // Iba al centroide
					//cout<<"iba al centroide"<<endl;
					if(x == xc && y == yc){ // Y he llegado
						//cout<<"y he llegado"<<endl;
						if(poss2explore[target_area].x.size()){
							//cout<<"me quedan posiciones por OBSERVAR"<<endl;
							return 3;
						}
					}
				}

				return 0; // Seguir el camino
			}

			void recompute_target(int x, int y, vector<vector<bool>> areas_graph, vector<vector<float>> costs_between_segments, vector<float> segment_obs_points, vector<vector<int>> &tree, vector<float> &branch_costs, vector<int> &branch)
			{
				if(areas_map_ini[x][y]>=0 && !poss2explore[areas_map_ini[x][y]].x.size()){ // Se ha terminado con el área en el que se encuentra el agente
					// Se obtiene el árbol y el área objetivo
					compute_tree(x, y, areas_graph, costs_between_segments, segment_obs_points, tree, branch_costs, branch);
				}else{ // No se ha terminado con el área en el que está el agente
					// Permanecer en el mismo área
					target_area = areas_map_ini[x][y];
				}
			}

			void compute_target(int x, int y, vector<vector<bool>> areas_graph, vector<vector<float>> costs_between_segments, vector<float> segment_obs_points, Poss<int> centroids, vector<float> max_seg_dist, float vrange, vector<vector<int>> &tree, vector<float> &branch_costs, vector<int> &branch)
			{
				if(areas_map_ini[x][y]>=0 && !poss2explore[areas_map_ini[x][y]].x.size()){ // Se ha terminado con el área en el que se encuentra el agente
					// Se obtiene el árbol y el área objetivo
					compute_tree(x, y, areas_graph, costs_between_segments, segment_obs_points, tree, branch_costs, branch);
					if(max_seg_dist[target_area] <= vrange){
						//cout<<"Fijo goal en el centroide"<<endl;
						xg = centroids.x[target_area]; yg = centroids.y[target_area];
					}
				}else{ // No se ha terminado con el área en el que está el agente
					// Permanecer en el mismo área
					target_area = areas_map_ini[x][y];
				}
			}

			void compute_target(int x, int y, vector<vector<bool>> areas_graph, vector<vector<float>> costs_between_segments, vector<float> segment_obs_points, vector<vector<int>> &tree, vector<float> &branch_costs, vector<int> &branch)
			{
				if(target_area < 0)
					recompute_target(x, y, areas_graph, costs_between_segments, segment_obs_points, tree, branch_costs, branch);
			}

			void targets_and_path(int x, int y, vector<vector<bool>> areas_graph, vector<vector<float>> costs_between_segments, vector<float> segment_obs_points, Poss<int> centroids, vector<vector<float>> navigation_grid, vector<float> max_seg_dist, float vrange, Path &path, vector<vector<int>> &tree, vector<float> &branch_costs, vector<int> &branch)
			{

				// Selección de la acción
				int action;
				if(target_area >= 0){ // Probablemente primera iteración, se selecciona la siguiente acción
					//cout<<"selecciono acción"<<endl;
					action = action_selection(x, y, vrange >= max_seg_dist[target_area], centroids.x[target_area], centroids.y[target_area]);
				}else{
					action = 1;
				}

				//if(!action) return; // Seguir el camino, aquí no se hace nada
				if(!action){
					//cout<<"SIGO CON LA MISMA ACCIÓN, SIGO EL CAMINO al goal ("<<x<<", "<<y<<") → ("<<xg<<", "<<yg<<")"<<endl;
					// Recalcular camino (por si hay dinamismo)
					//path = navigation::path2goal(x, y, navigation_grid, xg, yg);
					path = navigation::path2goal(x, y, navigation_grid, xg, yg, grad);
					//if(target_area>=0) cout<<"Centroide ("<<centroids.x[target_area]<<", "<<centroids.y[target_area]<<")"<<endl;
					// Terminar, aquí no se hace nada más (el goal es el correcto)
					return;
				}

				//if(areas_map_ini[x][y] < 0) return;

				path.clear();

				//cout<<"ACTION: "<<action<<endl;

				if(action == 1){ // No quedan puntos por ver, buscar un área nueva

					//cout<<"área "<<areas_map_ini[x][y]<<endl;
					/*
					if(areas_map_ini[x][y]>=0){
						cout<<poss2explore[areas_map_ini[x][y]].x.size()<<" puntos por ver"<<endl;
					}
					*/

					// ESTA CONDICIÓN ESTÁ BIEN?
					if(areas_map_ini[x][y]>=0 && !poss2explore[areas_map_ini[x][y]].x.size()){ // Se ha terminado con el área en el que se encuentra el agente
						// Se obtiene el árbol y el área objetivo
						//cout<<"obtener nuevo área"<<endl;
						//cout<<"("<<x<<", "<<y<<"): "<<areas_map_ini[x][y]<<endl;
						//compute_tree_variables(x, y, areas_graph, costs_between_segments, tree, branch_costs);
						compute_tree(x, y, areas_graph, costs_between_segments, segment_obs_points, tree, branch_costs, branch);
						//cout<<"nuevo área: "<<target_area<<endl;
					}else{ // No se ha terminado con el área en el que está el agente
						// Permanecer en el mismo área
						//cout<<"mismo área"<<endl;
						target_area = areas_map_ini[x][y];
					}

					// ÁREA SELECCIONADA, ENCONTRAR OBJETIVO

					if(target_area < 0){ // No se ha encontrado un área objetivo
						//cout<<"seleccionando objetivo más cercano -> camino más corto"<<endl;
						/*
						cout<<"!!!!!!!!!!!!!!!!"<<endl;
						cout<<"!!!!!!!!!!!!!!!!"<<endl;
						cout<<"!!!!!!!!!!!!!!!!"<<endl;
						cout<<"CAMINO MÁS CORTO"<<endl;
						cout<<"!!!!!!!!!!!!!!!!"<<endl;
						cout<<"!!!!!!!!!!!!!!!!"<<endl;
						cout<<"!!!!!!!!!!!!!!!!"<<endl;
						*/
						path = path2closest_point(x, y, navigation_grid);
						//path = path2next_goal_(x, y, navigation_grid);
						//cout<<"seleccionado"<<endl;
						area_from_path(path);
						//cout<<"área obtenida "<<target_area<<endl;
						//return;
						if(max_seg_dist[target_area] <= vrange){
							//cout<<"Fijo goal en el centroide"<<endl;
							xg = centroids.x[target_area]; yg = centroids.y[target_area];
							//path = navigation::path2goal(x, y, navigation_grid, xg, yg);
							path = navigation::path2goal(x, y, navigation_grid, xg, yg, grad);
							area_from_path(path);
							//cout<<"hecho"<<endl;
						}
					}else{ // Se ha encontrado área objetivo
						// Seleccionar el conjunto más cercano
						//cout<<"Área "<<target_area<<" seleccionada"<<endl;
						if(max_seg_dist[target_area] > vrange){
							//cout<<"Al conjunto más cercano"<<endl;
							select_closest_set(x, y, navigation_grid);
							path = path2next_goal(x, y, navigation_grid);
							area_from_path(path);
						}else{
							//cout<<"Al centroide de "<<target_area<<endl;
							//cout<<"Al centroide"<<endl;
							xg = centroids.x[target_area]; yg = centroids.y[target_area];
							//path = navigation::path2goal(x, y, navigation_grid, xg, yg);
							path = navigation::path2goal(x, y, navigation_grid, xg, yg, grad);
							area_from_path(path);
						}
					}
				}else if(action == 2){ // Permanecer en el mismo área
					//cout<<"Permanecer en el mismo área"<<endl;
					target_area = areas_map_ini[xg][yg];
					if(max_seg_dist[target_area] > vrange){
						//cout<<"Al conjunto de menos puntos"<<endl;
						select_closest_set(x, y, navigation_grid);
						path = path2next_goal(x, y, navigation_grid);
						area_from_path(path);
					}else{
						//cout<<"Al centroide de "<<target_area<<endl;
						//cout<<"Al centroide"<<endl;
						xg = centroids.x[target_area]; yg = centroids.y[target_area];
						//path = navigation::path2goal(x, y, navigation_grid, xg, yg);
						path = navigation::path2goal(x, y, navigation_grid, xg, yg, grad);
						area_from_path(path);
					}
				}else{
					//cout<<"Al conjunto de menos puntos"<<endl;
					target_area = areas_map_ini[xg][yg];
					select_closest_set(x, y, navigation_grid);
					path = path2next_goal(x, y, navigation_grid);
					area_from_path(path);
				}
				//cout<<action<<" "<<"("<<xg<<", "<<yg<<")"<<endl;

				if(xg==0 && yg==0) path.clear();

			}

			bool fix_target(vector<int> traversed_segments)
			{
				for(int i=0; i<traversed_segments.size(); i++){
					if(traversed_segments[i] >= 0 && poss2explore[traversed_segments[i]].x.size() > 0){
						target_area = traversed_segments[i];
						return true;
					}
				}
				return false;
			}

			void fix_target_and_path(vector<int> traversed_segments, int x, int y, vector<vector<float>> navigation_grid, Poss<int> centroids, vector<float> max_seg_dist, float vrange, Path &path)
			{
				for(int i=0; i<traversed_segments.size(); i++){
					if(traversed_segments[i] >= 0 && poss2explore[traversed_segments[i]].x.size() > 0){
						target_area = traversed_segments[i];
						if(max_seg_dist[target_area] > vrange){
							//cout<<"AL CONJUNTO MÄS CERCANO DE "<<target_area<<endl;
							select_closest_set(x, y, navigation_grid);
							path = path2next_goal(x, y, navigation_grid);
							area_from_path(path);
						}else{
							//cout<<"AL CENTROIDE DE "<<target_area<<endl;
							//cout<<"Al centroide"<<endl;
							xg = centroids.x[target_area]; yg = centroids.y[target_area];
							//path = navigation::path2goal(x, y, navigation_grid, xg, yg);
							path = navigation::path2goal(x, y, navigation_grid, xg, yg, grad);
							area_from_path(path);
						}
						//path = navigation::path2goal(x, y, navigation_grid, xg, yg, grad);
						return;
					}
				}
			}

			void erase_from_target_poss(int x, int y)
			{
				for(int j=0; j<target_poss.size(); j++){
					if(target_poss[j].erase_point(x,y)){
						break; // NO puede estar en 2 conjuntos a la vez
					}
				}
			}

			void update_all(int x, int y, float value)
			{
				if(seen_map[x][y] < 0){ // Posición que no se había visto
					// Eliminar de la lista de puntos por "ver"
					poss2explore[areas_map[x][y]].erase_point(x, y);
					if(areas_map[x][y] == target_area){ // Posición del área que se está explorando
						erase_from_target_poss(x, y);
					}
					// Fijar valores para no volver a seleccionar
					areas_map[x][y] = -1;
					//seen_map[visible_poss.x[i]][visible_poss.y[i]] = poss_values[i];
					// El goal se ha visto
					if(xg == x && yg == y){
						if(next_goal) next_goal = false;
					}
				}
				seen_map[x][y] = value;
				real_seen_map[x][y] = value;
			} // Actualización de las variables

			void update_all(Poss<int> visible_poss, vector<float> poss_values)
			{
				for(int i=0; i<visible_poss.x.size(); i++){
					if(seen_map[visible_poss.x[i]][visible_poss.y[i]] < 0){ // Posición que no se había visto
						// Eliminar de la lista de puntos por "ver"
						poss2explore[areas_map[visible_poss.x[i]][visible_poss.y[i]]].erase_point(visible_poss.x[i], visible_poss.y[i]);
						if(areas_map[visible_poss.x[i]][visible_poss.y[i]] == target_area){ // Posición del área que se está explorando
							erase_from_target_poss(visible_poss.x[i], visible_poss.y[i]);
						}
						// Fijar valores para no volver a seleccionar
						areas_map[visible_poss.x[i]][visible_poss.y[i]] = -1;
						//seen_map[visible_poss.x[i]][visible_poss.y[i]] = poss_values[i];
						// El goal se ha visto
						if(xg == visible_poss.x[i] && yg == visible_poss.y[i]){
							if(next_goal) next_goal = false;
						}
					}
					seen_map[visible_poss.x[i]][visible_poss.y[i]] = poss_values[i];
					real_seen_map[visible_poss.x[i]][visible_poss.y[i]] = poss_values[i];

				}
			} // Actualización de las variables

			void update_maps(int x, int y, float value)
			{
				// Fijar valores para no volver a seleccionar
				areas_map[x][y] = -1;
				seen_map[x][y] = value;
				real_seen_map[x][y] = value;
			} // Actualización de los mapas

			void update_maps(Poss<int> visible_poss, vector<float> poss_values)
			{
				for(int i=0; i<visible_poss.x.size(); i++){
					// Fijar valores para no volver a seleccionar
					areas_map[visible_poss.x[i]][visible_poss.y[i]] = -1;
					seen_map[visible_poss.x[i]][visible_poss.y[i]] = poss_values[i];
					real_seen_map[visible_poss.x[i]][visible_poss.y[i]] = poss_values[i];
				}
			} // Actualización de los mapas

			void update_targets()
			{
				if(target_set >= 0)
					if(target_poss[target_set].x.size() == 0){
						target_set = -1;
						//next_goal = false;
					}
				if(target_area >= 0)
					if(poss2explore[target_area].x.size() == 0){
						target_area = -1;
						//next_goal = false;
					}
			} // Reiniciar los objetivos para posteriormente poder

			void remove_point(int x, int y)
			{
				if(seen_map[x][y] < 0){ // Posición no vista todavía
					// Eliminar de la lista de puntos por "ver"
					poss2explore[areas_map[x][y]].erase_point(x,y);
					if(areas_map[x][y] == target_area){ // Posición del área que se está explorando
						for(int j=0; j<target_poss.size(); j++){
							if(target_poss[j].erase_point(x,y)){
								break;
							}
						}
					}
				}
				// Fijar valores para no volver a seleccionar
				areas_map[x][y] = -1;
				seen_map[x][y] = static_map[x][y]; // Voy a considerar que tiene el valor del mapa estático
			}

			void erase_area(int area_ind)
			{
				if(poss2explore[area_ind].x.size()){ // NO está vacía
					for(int i=0; i<poss2explore[area_ind].x.size(); i++){
						seen_map[poss2explore[area_ind].x[i]][poss2explore[area_ind].y[i]] = static_map[poss2explore[area_ind].x[i]][poss2explore[area_ind].y[i]];
						areas_map[poss2explore[area_ind].x[i]][poss2explore[area_ind].y[i]] = -1;
					}
					poss2explore[area_ind].clear();
				}
			} // Eliminar todos los puntos de un área (y sus respectivas variables)

			int update_poss_with_traversability(int x, int y, vector<vector<float>> navigable_grid, vector<bool> dynamic)
			{
				// Obtener el gradiente de todos los puntos alcanzables
				int discarded_points = 0;
				FMM gr(x, y, navigable_grid);
				vector<vector<float>> achievable_area = gr.compute_gradient_();
				// Comprobar si los segmentos son alcanzables
				for(int i=0; i<poss2explore.size(); i++){
					//cout<<"a: "<<poss2explore[i].x.size()<<" - "<<i<<endl;
					if(!dynamic[i] && poss2explore[i].x.size()){ // Área no explorada con dinamismo
					//if(poss2explore[i].x.size()){ // Área no explorada
						if(achievable_area[poss2explore[i].x[0]][poss2explore[i].y[0]] < INF){ // alcanzable
							// No hacer nada
							continue; // Pasar al siguiente segmento
						}else{ // NO alcanzable
							erase_area(i); // Eliminar el área
							continue; // Pasar al siguiente segmento
						}
					}
					//cout<<"áreas eliminadas"<<endl;
					// Eliminar los puntos que no sean alcanzables
					int it = 0;
					while(it < poss2explore[i].x.size()){
						//if(achievable_area[poss2explore[i].x[it]][poss2explore[i].y[it]] < INF){ // es alcanzable
						if(!navigable_grid[poss2explore[i].x[it]][poss2explore[i].y[it]] && achievable_area[poss2explore[i].x[it]][poss2explore[i].y[it]] < INF){ // es alcanzable
							it++;
						}else{ // No es alcanzable
							// Eliminar
							discarded_points++;
							poss2explore[i].erase(it); // Del conjunto de puntos por segmento
							if(areas_map[poss2explore[i].x[it]][poss2explore[i].y[it]] == target_area){
								erase_from_target_poss(poss2explore[i].x[it], poss2explore[i].y[it]); // Del conjunto de los puntos objetivo
							}
							// No volver a seleccionar
							areas_map[poss2explore[i].x[it]][poss2explore[i].y[it]] = -1;
							seen_map[poss2explore[i].x[it]][poss2explore[i].y[it]] = static_map[poss2explore[i].x[it]][poss2explore[i].y[it]];
						}
					}
					//if(areas_map_ini[x][y]<0 || (areas_map_ini[x][y]>=0 && dynamic[areas_map_ini[x][y]])){
					if(areas_map_ini[x][y]>=0 && dynamic[areas_map_ini[x][y]]){
						target_area = -1; // Para NO volver a seleccionar
						//path = dynamic::pat2closest_free_area(x, y, navigable_grid, poss2explore, dynamic);
					}
				}
				return discarded_points;
			}

			//int update_poss_with_traversability(int x, int y, vector<vector<float>> navigable_grid, vector<bool> dynamic, vector<Poss<int>> &discarded_poss)
			int update_poss_with_traversability(int x, int y, vector<vector<float>> navigable_grid, vector<bool> dynamic, vector<bool> &discarded_areas)
			{
				// Obtener el gradiente de todos los puntos alcanzables
				int discarded_points = 0;
				FMM gr(x, y, navigable_grid);
				vector<vector<float>> achievable_area = gr.compute_gradient_();
				// Comprobar si los segmentos son alcanzables
				for(int i=0; i<poss2explore.size(); i++){
					//cout<<"a: "<<poss2explore[i].x.size()<<" - "<<i<<endl;
					if(!dynamic[i] && poss2explore[i].x.size()){ // Área no explorada con dinamismo
					//if(poss2explore[i].x.size()){ // Área no explorada
						if(achievable_area[poss2explore[i].x[0]][poss2explore[i].y[0]] < INF){ // alcanzable
							// No hacer nada
							continue; // Pasar al siguiente segmento
						}else{ // NO alcanzable
							//discarded_poss.push_back(poss2explore[i]);
							discarded_areas[i] = true;
							erase_area(i); // Eliminar el área
							continue; // Pasar al siguiente segmento
						}
					}
					//cout<<"áreas eliminadas"<<endl;
					// Eliminar los puntos que no sean alcanzables
					int it = 0;
					while(it < poss2explore[i].x.size()){
						//if(achievable_area[poss2explore[i].x[it]][poss2explore[i].y[it]] < INF){ // es alcanzable
						if(!navigable_grid[poss2explore[i].x[it]][poss2explore[i].y[it]] && achievable_area[poss2explore[i].x[it]][poss2explore[i].y[it]] < INF){ // es alcanzable
							it++;
						}else{ // No es alcanzable
							// Eliminar
							discarded_points++;
							poss2explore[i].erase(it); // Del conjunto de puntos por segmento
							if(areas_map[poss2explore[i].x[it]][poss2explore[i].y[it]] == target_area){
								erase_from_target_poss(poss2explore[i].x[it], poss2explore[i].y[it]); // Del conjunto de los puntos objetivo
							}
							// No volver a seleccionar
							areas_map[poss2explore[i].x[it]][poss2explore[i].y[it]] = -1;
							seen_map[poss2explore[i].x[it]][poss2explore[i].y[it]] = static_map[poss2explore[i].x[it]][poss2explore[i].y[it]];
						}
					}
					//if(areas_map_ini[x][y]<0 || (areas_map_ini[x][y]>=0 && dynamic[areas_map_ini[x][y]])){
					if(areas_map_ini[x][y]>=0 && dynamic[areas_map_ini[x][y]]){
						target_area = -1; // Para NO volver a seleccionar
						//path = dynamic::pat2closest_free_area(x, y, navigable_grid, poss2explore, dynamic);
					}
				}
				return discarded_points;
			}

			int get_target_area()
			{
				return target_area;
			}

			int get_target_set()
			{
				return target_set;
			}

			bool exists_goal()
			{
				return next_goal;
			}

			Pos<int> get_goal()
			{
				Pos<int> res;
				res.x = xg; res.y = yg;
				return res;
			}

			void get_goal(int &xg, int &yg)
			{
				xg = this->xg; yg = this->yg;
			}

			int getGoalx(){ return xg; }
			int getGoaly(){ return yg; }

			vector<Poss<int>> get_target_poss()
			{
				return target_poss;
			}

			vector<Poss<int>> get_poss2explore()
			{
				return poss2explore;
			}

			Poss<int> get_all_poss2explore()
			{
				return join_poss(poss2explore);
			}

			bool remaining_poss2explore()
			{
				for(int i=0; i<poss2explore.size(); i++)
					if(poss2explore[i].x.size()) return true;
				return false;
			}

			int Nremaining_poss2explore()
			{
				int res = 0;
				for(int i=0; i<poss2explore.size(); i++)
					res += poss2explore[i].x.size();
				return res;
			}

			vector<int> get_Nposs2explore()
			{
				vector<int> res(poss2explore.size(), 0);
				for(int i=0; i<poss2explore.size(); i++)
					res[i] = poss2explore[i].x.size();
				return res;
			}

			Poss<int> get_target_set_poss()
			{
				Poss<int> res;
				if(target_set >= 0) res = target_poss[target_set];
				return res;
			}

			vector<vector<float>> get_seen_map()
			{
				return seen_map;
			}

			vector<vector<int>> get_areas_map()
			{
				return areas_map;
			}

			vector<vector<float>> get_real_seen_map()
			{
				return real_seen_map;
			}

			int Nreal_poss2explore()
			{
				int res = 0;
				for(int i=0; i<real_seen_map.size(); i++)
					for(int j=0; j<real_seen_map[i].size(); j++)
						if(real_seen_map[i][j] > 0) res++;
				return res;
			}

			vector<vector<float>> get_gradient()
			{
				return grad;
			}

			vector<vector<bool>> get_graph()
			{
				return areas_graph;
			}

			vector<vector<int>> get_branches()
			{
				return all_branches;
			}

			vector<float> get_branches_costs()
			{
				return branches_costs;
			}

			int get_branch_index()
			{
				return selected_branch;
			}

			vector<float> get_areas_tree_costs()
			{
				return areas_tree_costs;
			}

	};

}

#endif
