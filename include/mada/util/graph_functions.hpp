// TODAS LAS SOPLAPOLLECES PARA REPRESENTAR EL GRAFO

#ifndef GRAPH_FUNCTIONS_HPP
#define GRAPH_FUNCTIONS_HPP

#include "funciones.hpp"
#include "path.hpp"

namespace graph_functions{

	// ---------------------------------------------------------------------------------
	// ---------------------------------------------------------------------------------
	// ---------------------------------------------------------------------------------
	//  							FUNCIONES DE GRAFOS
	// ---------------------------------------------------------------------------------
	// ---------------------------------------------------------------------------------
	// ---------------------------------------------------------------------------------

	vector<vector<float>> tf2graphCosts(vector<vector<bool>> graph, vector<float> vertex_costs)
	{
		vector<vector<float>> res(vertex_costs.size(), vector<float>(vertex_costs.size(), 0));
		for(int i=0; i<vertex_costs.size(); i++){
			for(int j=i+1; j<vertex_costs.size(); j++){
				if(graph[i][j]){
					if(vertex_costs[j])
						res[i][j] = res[j][i] = vertex_costs[j];
					else
						res[i][j] = res[j][i] = 0.0000001;
					//cout<<"NODOs "<<i<<" , "<<j<<" → "<<res[i][j]<<endl;
				}
			}
		}
		return res;
	}

	vector<int> connectivity(vector<vector<bool>> graph)
	{
		vector<int> res(graph.size(), 0);
		for(int i=0; i<graph.size(); i++)
			for(int j=0; j<graph.size(); j++)
				if(graph[i][j]) res[i]++;
		return res;
	} // Cantidad de aristas que tiene cada vértice del grafo

	vector<int> connectivity_se(vector<vector<bool>> graph)
	{
		vector<int> res(graph.size(), 0);
		for(int i=0; i<graph.size(); i++)
			for(int j=0; j<graph.size(); j++)
				if(i!=j && graph[i][j]) res[i]++;
		return res;
	} // Cantidad de aristas que tiene cada vértice del grafo (excluyendo a si mismo)

	vector<int> neighbors(vector<vector<bool>> graph, int v)
	{
		vector<int> res;
		for(int i=0; i<graph.size(); i++)
			if(graph[v][i]) res.push_back(i);
		return res;
	}

	template <typename T>
	vector<Poss<int>> form_graph_pos_matr(Poss<int> pos, vector<vector<T>> graph)
	{
	    vector<Poss<int>> res;

	    for(int i=0; i<pos.x.size(); i++){
	        for(int j=i+1; j<pos.x.size(); j++){
	            if(graph[i][j]){
	                res.resize(res.size()+1);
	                res[res.size()-1].push(pos.x[i],pos.y[i]);
	                res[res.size()-1].push(pos.x[j],pos.y[j]);
	            }
	        }
	    }

	    return res;   
	} // Forma el vector con las posiciones del grafo (Para despues representar)

	vector<Poss<int>> form_graph_pos_ind(Poss<int> pos, vector<vector<int>> graph_ind)
	{
	    vector<Poss<int>> res;

	    for(int i=0; i<graph_ind.size(); i++){
            res.resize(res.size()+1);
            res[res.size()-1].push(pos.x[graph_ind[i][0]],pos.y[graph_ind[i][0]]);
            res[res.size()-1].push(pos.x[graph_ind[i][1]],pos.y[graph_ind[i][1]]);
	    }

	    return res;   
	} // Formar el vector con las posiciones del grafo (Para despues representar)

	template <typename T>
	void save(vector<Poss<T>> poss, string filename)
	{
		vector<T> values(4,0);
    	for(int i=0; i<poss.size(); i++){
    		// Vector de las posiciones
    		values[0] = poss[i].x[0]; values[1] = poss[i].y[0];
    		values[2] = poss[i].x[1]; values[3] = poss[i].y[1];
    		// Guardar (en el mismo fichero)
    		save_vect_(filename.c_str(), values);
    	}
	}

	vector<vector<Path>> form_paths(Poss<int> centr, vector<vector<bool>> adj, vector<vector<float>> grid)
	{
		vector<vector<Path>> res(centr.x.size(),vector<Path>(centr.x.size()));
		vector<vector<float>> grad;
		for(int i=0; i<centr.x.size(); i++){
			for(int j=i+1; j<centr.x.size(); j++){
				if(i!=j && adj[i][j]){
					FMM gr(centr.x[i],centr.y[i],grid);
					//grad = gr.compute_gradient_();
					grad = gr.expand2goal(centr.x[j],centr.y[j]);
					res[i][j].gradient_descent_(grad,centr.x[j],centr.y[j]);
					res[j][i] = res[i][j];
				}
			}
		} 

		return res;
	} // Caminos (CONSIDERANDO LOS OBSTÁCULOS) entre los segmentos adyacentes

	template <typename T>
	T obtain_cost(vector<int> path, vector<vector<T>> costs)
	{
		T res = 0;
		for(int i=0; i<path.size()-1; i++) res += costs[path[i]][path[i+1]];
		return res;
	}

	template <typename T>
	void normalize_costs(vector<T> vertex_costs, vector<vector<T>> edge_costs, vector<T> &vc, vector<vector<T>> &ec)
	{
		T max = -INFINITY;
		for(int i=0; i<vertex_costs.size(); i++)
			if(max < vertex_costs[i]) max = vertex_costs[i];

		for(int i=0; i<edge_costs.size(); i++)
			for(int j=0; j<edge_costs[i].size(); j++)
				if(max < edge_costs[i][j]) max = edge_costs[i][j];

		vc = vertex_costs;
		for(int i=0; i<vc.size(); i++) vc[i] /= max;

		ec = edge_costs;
		for(int i=0; i<ec.size(); i++)
			for(int j=0; j<ec[i].size(); j++)
				ec[i][j] /= max;
	}

	template <typename T>
	vector<int> Dijkstra(vector<vector<T>> costs, int s)
	{
		queue<int> q; q.push(s); // Lista de los vértices ya hechos
		queue<int> qo; // Lista de vértices para abrir

		int n = costs.size();
		vector<T> tdist(n, -1); // distancia total desde el origen hasta el nodo
		tdist[q.front()] = 0;

		int imin; T min, val;
		vector<int> res(n,-2); res[s] = -1;

		while(!q.empty()){
			// Obtener la lista de vecinos para abrir
			for(int i=0; i<n; i++){
				if(costs[q.front()][i] && tdist[i]<0){ // Vecinos "no abiertos" de q(1)
					qo.push(i); // Vecinos para abrir
				}
			}
			// Conectar los vértices de la lista
			while(!qo.empty()){
				min = (T)RAND_MAX; imin = n;
				for(int i=0; i<n; i++){
					if(costs[i][qo.front()] && tdist[i]>=0){ // Existe arista y es un vértice ya fijado
						val = costs[i][qo.front()]+tdist[i]; // Coste total de alcanzar el vértice
						if(min > val){
							min = val;
							imin = i;
						}
					}
				}
				if(imin < n){
					tdist[qo.front()] = min;
					res[qo.front()] = imin;
				}
				q.push(qo.front()); // Insertar como los vértices para 
				qo.pop();
			}
			q.pop();
		}
		return res;
	}

	template <typename T>
	vector<int> Dijkstra_(vector<vector<T>> costs, int s)
	{
		vector<int> q; q.push_back(s); // Lista de los vértices ya hechos
		vector<int> qo; // Lista de vértices para abrir

		int n = costs.size();
		vector<T> tdist(n, -1); // distancia total desde el origen hasta el nodo
		vector<T> distances; vector<int> order;
		tdist[q[0]] = 0;

		int imin; T min, val;
		vector<int> res(n,-2); res[s] = -1;

		while(q.size()){
			// Obtener la lista de vecinos para abrir
			for(int i=0; i<n; i++){
				if(costs[q[0]][i] && tdist[i]<0){ // Vecinos "no abiertos" de q(1)
					qo.push_back(i); // Vecinos para abrir
					distances.push_back(costs[q[0]][i] + tdist[q[0]]);
				}
			}
			// Ordenar los puntos en orden ascendente
			if(distances.size()){
				distances = sort_vect(distances, 1, order);
				qo = sort_vect(qo, order);
			}
			// Conectar los vértices de la lista
			while(qo.size()){
				min = (T)RAND_MAX; imin = n;
				for(int i=0; i<n; i++){
					if(costs[i][qo[0]] && tdist[i]>=0){ // Existe arista y es un vértice ya fijado
						val = costs[i][qo[0]]+tdist[i]; // Coste total de alcanzar el vértice
						if(min > val){
							min = val;
							imin = i;
						}
					}
				}
				if(imin < n){
					tdist[qo[0]] = min;
					res[qo[0]] = imin;
				}
				q.push_back(qo[0]); // Insertar como los vértices para 
				qo.erase(qo.begin());
				distances.erase(distances.begin());
			}
			q.erase(q.begin());
		}
		return res;
	}

	vector<int> extract_path(int s, int g, vector<int> dtree)
	{
		vector<int> res;
		if(dtree[s]>-1) return res; // El punto de partida debe ser el origen/raíz del árbol
		int n = dtree.size();
		while(s!=g && n){ // Para asegurar que no se quede en un bucle infinito en caso de no existir un camino
			res.push_back(g);
			g = dtree[g];
			n--;
		}
		res.push_back(g);
		return res;
	} // Extraer camino, de "s" a "g", del grafo construido con Dijkstra

	vector<vector<int>> path2graph(vector<int> path)
	{
		vector<vector<int>> res;
		for(int i=0; i<path.size()-1; i++)
			res.push_back({path[i], path[i+1]});
		return res;
	} // Pasar el vector de camino a grafo (PARA REPRESENTAR)

	void findAllPaths(const vector<vector<bool>>& graph, vector<int>& path, vector<vector<int>>& allPaths, int current, int destination, vector<bool>& visited)
	{
	    // Obtiene todos los caminos posibles entre dos nodos en el grafo
	    // Complejidad: [p : número de caminos posibles entre los nodos inicial y final; L : longitud promedio de un camino;]
	    // Métrica :  Complejidad en el peor caso    |  Caso Promedio (grafos no densos)
	    // Tiempo  :      O(p⋅L), donde p≤2^V        |       O(E⋅L)
	    // Espacio :      O(V+p⋅L)                   |       O(V+E)

	    // Agregar el nodo actual al camino y marcarlo como visitado
	    path.push_back(current);
	    visited[current] = true;

	    // Caso base: Si llegamos al nodo destino, guardamos el camino actual
	    if (current == destination) {
	        allPaths.push_back(path);
	    } else {
	        // Explorar vecinos del nodo actual
	        for (int neighbor = 0; neighbor < graph.size(); ++neighbor) {
	            // Si hay conexión (graph[current][neighbor] != 0) y no ha sido visitado
	            if (graph[current][neighbor] && !visited[neighbor]) {
	                findAllPaths(graph, path, allPaths, neighbor, destination, visited);
	            }
	        }
	    }

	    // Backtracking: Eliminar el nodo actual del camino y desmarcarlo como visitado
	    path.pop_back();
	    visited[current] = false;
	}

	vector<vector<int>> findAllPaths(const vector<vector<bool>>& graph, int current, int destination)
	{
		vector<bool> visited(graph.size(), false);
        vector<vector<int>> all_paths;
        vector<int> _path;
        findAllPaths(graph, _path, all_paths, current, destination, visited);
        return all_paths;
	}

	vector<int> dijkstra(const vector<vector<float>>& graph, int start, int end, const set<int>& blocked) {
	    int V = graph.size();
	    vector<float> dist(V, numeric_limits<float>::infinity()); // Coste mínimo a cada nodo
	    vector<int> prev(V, -1);                                  // Predecesor para reconstruir el camino

	    // Cola de prioridad (min-heap) usando pair<float, int>
	    priority_queue<pair<float, int>, vector<pair<float, int>>, greater<pair<float, int>>> pq;

	    dist[start] = 0;
	    pq.emplace(0, start);

	    while (!pq.empty()) {
	        // Extraer el nodo con el coste mínimo
	        float current_cost = pq.top().first;
	        int current = pq.top().second;
	        pq.pop();

	        // Si ya estamos en el nodo destino, terminamos
	        if (current == end) break;

	        // Si el nodo actual está bloqueado, lo ignoramos
	        if (blocked.find(current) != blocked.end()) continue;

	        // Relajar las aristas adyacentes
	        for (int neighbor = 0; neighbor < V; ++neighbor) {
	            if (graph[current][neighbor] != 0 && blocked.find(neighbor) == blocked.end()) {
	                float new_cost = current_cost + graph[current][neighbor];
	                if (new_cost < dist[neighbor]) {
	                    dist[neighbor] = new_cost;
	                    prev[neighbor] = current;
	                    pq.emplace(new_cost, neighbor);
	                }
	            }
	        }
	    }

	    // Reconstruir el camino desde el nodo destino al origen
	    vector<int> path;
	    for (int at = end; at != -1; at = prev[at]) {
	        path.push_back(at);
	    }

	    // Si no hay camino válido o el camino no conecta correctamente, devolvemos vacío
	    if (path.empty() || path.back() != start) {
	        return {}; // No se puede reconstruir el camino correctamente
	    }

	    reverse(path.begin(), path.end());
	    return path;
	}


	// Función principal para encontrar el camino de menor coste respetando bloqueos
	vector<int> findPathWithBlocking(const vector<vector<float>>& graph, int start, int end, vector<float>& costs)
	{
	    set<int> blocked_nodes; // Conjunto de nodos bloqueados
	    vector<int> path;

	    while (true) {
	        // Usar Dijkstra para encontrar el camino de menor coste
	        path = dijkstra(graph, start, end, blocked_nodes);

	        // Si no hay camino, devolvemos un camino vacío
	        if (path.empty()) {
	            return {};
	        }

	        // Verificar si el camino pasa por algún nodo bloqueado
	        bool valid_path = true;
	        for (int node : path) {
	            if (costs[node] != 0) {
	                blocked_nodes.insert(node); // Bloqueamos el nodo
	                valid_path = false;
	                break;
	            }
	        }

	        // Si el camino es válido (no pasa por nodos bloqueados), devolvemos el camino
	        if (valid_path) {
	            return path;
	        }
	    }
	}

	vector<int> findPathWithBlockingDev(const vector<vector<float>>& graph, int start, int end, vector<float>& costs, float deviation_ratio)
	{
	    set<int> blocked_nodes; // Conjunto de nodos bloqueados
	    vector<int> last_valid_path; // Último camino válido almacenado
	    vector<int> original_path; // Camino original
	    float original_cost = numeric_limits<float>::infinity(); // Coste del camino original

	    while (true) {
	        // Usar Dijkstra para encontrar el camino de menor coste
	        vector<int> path = dijkstra(graph, start, end, blocked_nodes);

	        //sh_vect_h(path, ("camino "+to_string(cont)).c_str());

	        // Si no hay camino, devolvemos el último camino válido o un camino vacío
	        if (path.empty()) {
	            return last_valid_path;
	        }

	        // Calcular el coste del camino actual
	        float current_cost = 0;
	        for (size_t i = 0; i < path.size() - 1; ++i) {
	            current_cost += graph[path[i]][path[i + 1]];
	        }

	        // Almacenar el coste original si es la primera iteración
	        if (original_cost == numeric_limits<float>::infinity()) {
	            original_cost = current_cost;
	            original_path = path;
	        }

	        // Verificar si el camino pasa por algún nodo bloqueado
	        bool valid_path = true;
	        for (int node : path) {
	            if (costs[node] != 0) {
	                // Bloqueamos el nodo y marcamos que el camino no es válido
	                blocked_nodes.insert(node);
	                valid_path = false;
	                break;
	            }
	        }

	        // Si el camino es válido, lo almacenamos y lo devolvemos
	        if (valid_path) {
	            last_valid_path = path;
	            return path;
	        }

	        // Si el camino no es válido, verificar la desviación
	        if (current_cost > original_cost * (1 + deviation_ratio)) {
	            // Si la desviación excede el ratio permitido, devolvemos el último camino válido
	            //return last_valid_path;
	            return original_path;
	        }

	        // Actualizar el último camino válido si es aceptable
	        last_valid_path = path;
	    }
	}

};

namespace tree_functions{

	// ---------------------------------------------------------------------------------
	// ---------------------------------------------------------------------------------
	// ---------------------------------------------------------------------------------
	//  							FUNCIONES DE ÁRBOLES
	// ---------------------------------------------------------------------------------
	// ---------------------------------------------------------------------------------
	// ---------------------------------------------------------------------------------

	vector<vector<int>> vect2branches(vector<int> v)
	{
		vector<vector<int>> res;
		for(int i=0; i<v.size()-1; i++){
			res.push_back({v[i], v[i+1]});
		}
		return res;
	}

	vector<vector<bool>> ind2matr(vector<vector<int>> tree, int size)
	{
		vector<vector<bool>> res(size, vector<bool>(size,false));
		for(int i=0; i<tree.size(); i++)
			res[tree[i][0]][tree[i][1]] = true;
		return res;
	}

	vector<vector<int>> depths_vect2matr(vector<int> depths)
	{
		vector<vector<int>> res;
		for(int i=0; i<depths.size(); i++){
			if(res.size() < depths[i]+1){
				res.resize(depths[i]+1);
			}
			res[depths[i]].push_back(i);
		}
		return res;
	}

	vector<int> depths_matr2vect(vector<vector<int>> depths)
	{
		vector<int> res;
		for(int i=0; i<depths.size(); i++){
			for(int j=0; j<depths[i].size(); j++){
				if(res.size() < depths[i][j]+1) res.resize(depths[i][j]+1);
				res[depths[i][j]] = i;
			}
		}
		return res;
	}

	vector<int> prune_branch(vector<int> branch, int v)
	{
		while(branch.size()){
			if(v != branch[0]){
				branch.erase(branch.begin());
			}else{
				return branch;
			}
		}
		return branch;
	}

	vector<int> prune_branch(vector<int> branch, int s, int e)
	{
		while(branch.size()){
			if(s != branch[0]){
				branch.erase(branch.begin());
			}else{
				break;
			}
		}

		while(branch.size()){
			if(e != branch[branch.size()-1]){
				branch.erase(branch.end()-1);
			}else{
				break;
			}
		}

		return branch;
	}

	template<typename T>
	vector<vector<int>> sort_by_depths(vector<vector<T>> graph, int root)
	{
	    vector<vector<int>> tree;
	    tree.push_back(vector<int>(1,root));
	    int tree_depth = 0;
	    
	    int n=graph.size();
	    int nd = n-1;

	    vector<bool> to_con(n,true);
	    to_con[root] = false;

	    while(nd){
	        if(nd){
	            tree.push_back(vector<int>(0));
	        }
	        for(int i=0; i<tree[tree_depth].size(); i++){
	            for(int j=0; j<n; j++){
	                if(to_con[j] && graph[tree[tree_depth][i]][j]){
	                    nd--;
	                    to_con[j] = false;
	                    tree[tree_depth+1].push_back(j);
	                }
	            }
	        }
	        if(!tree[tree_depth+1].size()) break;
	        // Incrementar la profundidad del árbol
	        tree_depth++;
	    }

	    //if(!tree[tree_depth].size()){
    	if(!tree[tree.size()-1].size()) tree.erase(tree.end());

	    return tree;
	}

	template<typename T>
	vector<vector<int>> sort_by_depths(int root, vector<vector<T>> graph)
	{
	    int ind;
	    vector<int> _parents, parents, childs; // 2 niveles (profundidades) del árbol
	    parents.push_back(root);
	    int n = graph.size();
	    vector<vector<int>> depths;
	    depths.push_back(parents);
	    vector<bool> todo(n, true);
	    todo[root] = false;
	    while(parents.size()){
	    	_parents = parents;
	    	// Encontrar todos los descendientes de todos los vértices de este nivel
	    	while(parents.size()){
		    	for(int i=0; i<n; i++){
		    		if(todo[i] && i!=parents[0] && graph[parents[0]][i]){
		    			childs.push_back(i);
		    			graph[parents[0]][i] = graph[i][parents[0]] = false;
		    			todo[i] = false;
		    		}
		    	}
		    	parents.erase(parents.begin());
		    }
		    depths.push_back(childs);
		    parents = childs; // Reiniciar los padres
    		childs.clear(); // Reiniciar los hijos
	    }

	    return depths;
	}

	void find_in_tree(int vertex, vector<vector<vector<int>>> tree_bd, int &depth, int &ind)
	{
		for(int i=0; i<tree_bd.size(); i++){
			for(int j=0; j<tree_bd[i].size(); j++){
				if(tree_bd[i][j][1] == vertex){
					ind = j;
					depth = i;
					return;
				}
			}
		}
	} // Encontrar el índice y profundidad del vértice en el árbol (cuando es una "hoja")

	vector<int> ascend_tree(int s, vector<vector<vector<int>>> tree_bd)
	{
		vector<int> res;
		// Encontrar el vértice en el árbol
		int d, ind;
		// Ascender hasta la raíz formado el camino
		while(s!=tree_bd[0][0][0]){
			res.push_back(s);
			find_in_tree(s, tree_bd, d, ind);
			s = tree_bd[d][ind][0];
		}
		res.push_back(s);
		return res;
	}

	vector<vector<int>> generate_tree_links(vector<vector<int>> tree, vector<vector<float>> distances)
	{
	    vector<vector<int>> res;

	    float dist;
	    int ind;
	    for(int i=0; i<tree.size()-1; i++){
	        for(int j=0; j<tree[i+1].size(); j++){
	            dist = INF;
	            for(int k=0; k<tree[i].size(); k++){
	                if(dist > distances[tree[i+1][j]][tree[i][k]]){
	                    dist = distances[tree[i+1][j]][tree[i][k]];
	                    ind = k;
	                }
	            }
	            if(dist < INF){
	                res.resize(res.size()+1);
	                res[res.size()-1].push_back(tree[i][ind]);
	                res[res.size()-1].push_back(tree[i+1][j]);
	            }
	        }
	    }

	    return res;
	} // Devuelve los pares de enlaces entre los segmentos

	vector<vector<int>> generate_tree_from_graph(int root, vector<vector<bool>> graph, vector<vector<float>> distances)
	{
	    vector<vector<int>> res;

	    float dist; int ind;
	    vector<int> _parents, parents, childs; // 2 niveles (profundidades) del árbol
	    parents.push_back(root);
	    int n = graph.size();
	    while(parents.size()){
	    	_parents = parents;
	    	while(parents.size()){
		    	for(int i=0; i<n; i++){
		    		if(i!=parents[0] && graph[parents[0]][i]){
		    			childs.push_back(i);
		    			graph[parents[0]][i] = graph[i][parents[0]] = false;
		    		}
		    	}
		    	parents.erase(parents.begin());
		    }
	    	// Se han encontrado todos los enlaces entre dos profundidades
	    	// Ahora encontrar la mejor conexión entre ellos
		    for(int i=0; i<childs.size(); i++){ // Para cada hijo
		    	// Encontrar el padre que está más cerca
		    	dist = INF; ind = -1;
		    	res.push_back(vector<int>(1,childs[i]));
		    	for(int j=0; j<_parents.size(); j++){
		    		if(distances[_parents[j]][childs[i]] && dist > distances[_parents[j]][childs[i]]){
		    			//dist = distances[childs[i]][_parents[j]];
		    			dist = distances[_parents[j]][childs[i]];
		    			ind = _parents[j];
		    		}
		    	}
		    	if(ind >= 0){
		    		res[res.size()-1].insert(res[res.size()-1].begin(),ind);
		    	}
		    }
    		parents = childs; // Reiniciar los padres
    		childs.clear(); // Reiniciar los hijos
	    }
	    return res;
	} // Devuelve los pares de enlaces entre los segmentos

	vector<vector<vector<int>>> generate_tree_from_graph_bd(int root, vector<vector<bool>> graph, vector<vector<float>> distances)
	{
	    vector<vector<vector<int>>> res;

	    float dist; int ind;
	    vector<int> _parents, parents, childs; // 2 niveles (profundidades) del árbol
	    parents.push_back(root);
	    int n = graph.size();
	    while(parents.size()){
	    	res.resize(res.size()+1);
	    	_parents = parents;
	    	while(parents.size()){
		    	for(int i=0; i<n; i++){
		    		if(i!=parents[0] && graph[parents[0]][i]){
		    			childs.push_back(i);
		    			graph[parents[0]][i] = graph[i][parents[0]] = false;
		    		}
		    	}
		    	parents.erase(parents.begin());
		    }
	    	// Se han encontrado todos los enlaces entre dos profundidades
	    	// Ahora encontrar la mejor conexión entre ellos
		    for(int i=0; i<childs.size(); i++){ // Para cada hijo
		    	// Encontrar el padre que está más cerca
		    	dist = INF; ind = -1;
		    	res[res.size()-1].push_back(vector<int>(1,childs[i]));
		    	for(int j=0; j<_parents.size(); j++){
		    		if(distances[_parents[j]][childs[i]] && dist > distances[_parents[j]][childs[i]]){
		    			//dist = distances[childs[i]][_parents[j]];
		    			dist = distances[_parents[j]][childs[i]];
		    			ind = _parents[j];
		    		}
		    	}
		    	if(ind >= 0){
		    		res[res.size()-1][res[res.size()-1].size()-1].insert(res[res.size()-1][res[res.size()-1].size()-1].begin(),ind);
		    	}
		    }
    		parents = childs; // Reiniciar los padres
    		childs.clear(); // Reiniciar los hijos
	    }
	    return res;
	} // Devuelve los pares de enlaces entre los segmentos (Se conectan los vértices de la misma profundidad si tienen la misma distancia)

	vector<vector<vector<int>>> generate_tree_from_graph_dw(int root, vector<vector<bool>> graph)
	{
	    vector<vector<vector<int>>> res;

	    vector<vector<bool>> graph_ = graph;

	    float dist; int ind;
	    vector<int> _parents, parents, childs; // 2 niveles (profundidades) del árbol
	    parents.push_back(root);
	    int n = graph.size();
	    vector<bool> todo(n, true);
	    todo[root] = false;
	    while(parents.size()){
	    	res.resize(res.size()+1);
	    	_parents = parents;
	    	// Encontrar todos los descendientes de todos los vértices de este nivel
	    	while(parents.size()){
		    	for(int i=0; i<n; i++){
		    		if(todo[i] && i!=parents[0] && graph[parents[0]][i]){
		    			childs.push_back(i);
		    			graph[parents[0]][i] = graph[i][parents[0]] = false;
		    			todo[i] = false;
		    		}
		    	}
		    	parents.erase(parents.begin());
		    }
	    	// Se han encontrado todos los enlaces entre dos profundidades
	    	// Ahora encontrar la mejor conexión entre ellos
		    for(int i=0; i<childs.size(); i++){ // Para cada hijo
		    	// Encontrar el padre que está más cerca
		    	ind = -1;
		    	res[res.size()-1].push_back(vector<int>(1,childs[i]));
		    	for(int j=0; j<_parents.size(); j++){
	    			if(graph_[_parents[j]][childs[i]]){
		    			ind = _parents[j];
		    			graph_[_parents[j]][childs[i]] = graph_[childs[i]][_parents[j]] = 0;
		    		}
		    	}
		    	if(ind >= 0){
		    		res[res.size()-1][res[res.size()-1].size()-1].insert(res[res.size()-1][res[res.size()-1].size()-1].begin(),ind);
		    	}
		    }
    		parents = childs; // Reiniciar los padres
    		childs.clear(); // Reiniciar los hijos
	    }
	    return res;
	} // Construir árbol desde la raíz - Devuelve los pares de enlaces entre los segmentos

	template<typename T>
	vector<vector<vector<int>>> generate_tree_from_graph_dw_cm(int root, vector<vector<bool>> graph, vector<vector<T>> costs)
	{
	    vector<vector<vector<int>>> res;

	    T cost;
	    int ind;
	    vector<int> _parents, parents, childs; // 2 niveles (profundidades) del árbol
	    parents.push_back(root);
	    int n = graph.size();
	    vector<bool> todo(n, true);
	    todo[root] = false;
	    while(parents.size()){
	    	res.resize(res.size()+1);
	    	_parents = parents;
	    	// Encontrar todos los descendientes de todos los vértices de este nivel
	    	while(parents.size()){
		    	for(int i=0; i<n; i++){
		    		if(todo[i] && i!=parents[0] && graph[parents[0]][i]){
		    			childs.push_back(i);
		    			graph[parents[0]][i] = graph[i][parents[0]] = false;
		    			todo[i] = false;
		    		}
		    	}
		    	parents.erase(parents.begin());
		    }
	    	// Se han encontrado todos los enlaces entre dos profundidades
	    	// Ahora encontrar la mejor conexión entre ellos
		    for(int i=0; i<childs.size(); i++){ // Para cada hijo
		    	// Encontrar el padre que está más cerca
		    	cost = INF; ind = -1;
		    	res[res.size()-1].push_back(vector<int>(1,childs[i]));
		    	for(int j=0; j<_parents.size(); j++){
		    		if(costs[_parents[j]][childs[i]] && cost > costs[_parents[j]][childs[i]]){
		    			cost = costs[_parents[j]][childs[i]];
		    			ind = _parents[j];
		    			costs[_parents[j]][childs[i]] = costs[childs[i]][_parents[j]] = 0;
		    		}
		    	}
		    	if(ind >= 0){
		    		res[res.size()-1][res[res.size()-1].size()-1].insert(res[res.size()-1][res[res.size()-1].size()-1].begin(),ind);
		    	}
		    }
    		parents = childs; // Reiniciar los padres
    		childs.clear(); // Reiniciar los hijos
	    }
	    return res;
	} // Construir árbol desde la raíz - Devuelve los pares de enlaces entre los segmentos (NO se conectan los vértices de la misma profundidad)

	vector<vector<vector<int>>> generate_tree_from_graph_uw(int root, vector<vector<bool>> graph)
	{
	    vector<vector<vector<int>>> res;

	    vector<vector<bool>> graph_ = graph;

	    int ind;
	    vector<int> _parents, parents, childs; // 2 niveles (profundidades) del árbol
	    parents.push_back(root);
	    int n = graph.size();
	    vector<vector<int>> depths;
	    depths.push_back(parents);
	    vector<bool> todo(n, true);
	    todo[root] = false;
	    while(parents.size()){
	    	_parents = parents;
	    	// Encontrar todos los descendientes de todos los vértices de este nivel
	    	while(parents.size()){
		    	for(int i=0; i<n; i++){
		    		if(todo[i] && i!=parents[0] && graph[parents[0]][i]){
		    			childs.push_back(i);
		    			graph[parents[0]][i] = graph[i][parents[0]] = false;
		    			todo[i] = false;
		    		}
		    	}
		    	parents.erase(parents.begin());
		    }
		    depths.push_back(childs);
		    parents = childs; // Reiniciar los padres
    		childs.clear(); // Reiniciar los hijos
	    }

	    // Se han encontrado todos los enlaces entre dos profundidades
    	// Ahora encontrar la mejor conexión entre ellos
    	vector<vector<int>> empty_matr;
	    for(int i=depths.size()-2; i; i--){
	    	res.insert(res.begin(), empty_matr); // Insertar una profundidad por encima
	    	for(int j=0; j<depths[i].size(); j++){ // Descendiente ("hijos")
	    		ind = -1;
	    		for(int k=0; k<depths[i-1].size(); k++){ // Padre
	    			if(graph_[depths[i][j]][depths[i-1][k]]){
	    				ind = depths[i-1][k];
	    				break; // No seguir buscando, todos los "padres" tienen el mismo coste
	    			}
	    		}
	    		if(ind >= 0){
	    			childs.push_back(ind); childs.push_back(depths[i][j]);
	    			res[0].push_back(childs);
	    			childs.clear();
	    		}
	    	}
	    }

	    return res;
	} // Construir árbol hasta la raíz - Devuelve los pares de enlaces entre los segmentos

	template<typename T>
	vector<vector<vector<int>>> link_vertices_by_depth(vector<vector<int>> depths, vector<T> costs, vector<vector<bool>> graph)
	{
		vector<vector<vector<int>>> res;

		T cost;
	    int ind;
	    vector<int> childs;
	    for(int i=depths.size()-2; i; i--){
	    	res.insert(res.begin(), vector<vector<int>>()); // Insertar una profundidad por encima
	    	for(int j=0; j<depths[i].size(); j++){ // Descendiente ("hijos")
	    		ind = -1; cost = (T)INF;
	    		for(int k=0; k<depths[i-1].size(); k++){ // Padre
	    			if(graph[depths[i][j]][depths[i-1][k]] && cost > costs[k]){
	    				ind = depths[i-1][k];
	    				cost = costs[k];
	    				//break; // No seguir buscando, todos los "padres" tienen el mismo coste
	    			}
	    		}
	    		if(ind >= 0){
	    			childs.push_back(ind); childs.push_back(depths[i][j]);
	    			res[0].push_back(childs);
	    			childs.clear();
	    		}
	    	}
	    }

		return res;
	}

	template<typename T>
	vector<vector<vector<int>>> generate_tree_from_graph_uw_cv(int root, vector<vector<bool>> graph, vector<T> costs)
	{
	    vector<vector<vector<int>>> res;

	    // Tiene que ser esta función para poder representar las últimas ramas
	    vector<vector<int>> depths = sort_by_depths(root, graph);

	    // Se han encontrado todos los enlaces entre dos profundidades
    	// Ahora encontrar la mejor conexión entre ellos
    	vector<vector<int>> empty_matr;
    	T cost;
	    int ind;
	    vector<int> childs;
	    for(int i=depths.size()-2; i; i--){
	    	res.insert(res.begin(), empty_matr); // Insertar una profundidad por encima
	    	for(int j=0; j<depths[i].size(); j++){ // Descendiente ("hijos")
	    		ind = -1; cost = (T)INF;
	    		for(int k=0; k<depths[i-1].size(); k++){ // Padre
	    			if(graph[depths[i][j]][depths[i-1][k]] && cost > costs[k]){
	    				ind = depths[i-1][k];
	    				cost = costs[k];
	    				//break; // No seguir buscando, todos los "padres" tienen el mismo coste
	    			}
	    		}
	    		if(ind >= 0){
	    			childs.push_back(ind); childs.push_back(depths[i][j]);
	    			res[0].push_back(childs);
	    			childs.clear();
	    		}
	    	}
	    }

	    return res;
	} // Construir árbol hasta la raíz enlazando en base los costes - Devuelve los pares de enlaces entre los segmentos

	vector<vector<vector<int>>> generate_links_by_depths(vector<vector<int>> tree_depths, vector<vector<int>> tree_links)
	{
		vector<vector<vector<int>>> res(tree_depths.size());
		for(int i=0; i<tree_depths.size(); i++){
			for(int j=0; j<tree_depths[i].size(); j++){
				for(int k=0; k<tree_links.size(); k++){
					if(tree_depths[i][j] == tree_links[k][0]) res[i].push_back(tree_links[k]);
				}
			}
		}
		return res;	
	} // Clasificar los enlaces del árbol por profundidades

	vector<vector<int>> depth_tree2tree(vector<vector<vector<int>>> tree_bd)
	{
		vector<vector<int>> tree;
		for(int i=0; i<tree_bd.size(); i++){
			for(int j=0; j<tree_bd[i].size(); j++){
				tree.push_back(tree_bd[i][j]);
			}
		}
		return tree;
	}

	vector<vector<int>> build_tree(int root, vector<vector<bool>> graph, vector<vector<float>> edge_costs, vector<float> vertex_costs)
	{
		vector<vector<int>> tree;
		if(root < 0) return tree;

		int n = graph.size();
		// "Frente de onda"
		vector<int> wv;
		vector<float> wvc;
		vector<bool> todo(n, true);
		// Inicializar el frente
		wv.push_back(root);
		wvc.push_back(vertex_costs[root]);
		todo[root] = false;

		queue<int> nv; // Vecinos

		float cost, min_cost;
		int ind;
		bool found;

		while(wv.size()){
			// Encontrar vecinos del primer elemento del "frente"
			for(int i=0; i<n; i++){
				if(todo[i] && graph[wv[0]][i] && wv[0]!=i){
					nv.push(i);
				}
			}

			// Enlazar a los vecinos encontrados
			while(nv.size()){
				// Encontrar el elmento del "frente" al que conectar
				min_cost = INFINITY; ind = -1;
				for(int i=0; i<wv.size(); i++){
					if(graph[nv.front()][wv[i]]){
						// coste = coste del elemento del frente + coste de arista + coste vértice
						cost = wvc[i] + edge_costs[wv[i]][nv.front()] + vertex_costs[nv.front()];
						if(min_cost > cost){
							min_cost = cost;
							ind = i;
						}
					}
				}

				if(ind >= 0){
					// Encontrar la posisión dentro del "frente" donde insertar el vecino
					found = false;
					for(int i=0; i<wv.size(); i++){
						if(wvc[i] > min_cost){
							wv.insert(wv.begin()+i, nv.front());
							wvc.insert(wvc.begin()+i, min_cost);
							found = true;
							break;
						}
					}
					if(!found){
						wv.push_back(nv.front());
						wvc.push_back(min_cost);
					}

					// Insertar en el árbol
					tree.push_back({wv[ind], nv.front()});
				}

				// Eliminar al vecino encontrado
				todo[nv.front()] = false;
				nv.pop();
			}
			// Eliminar el elemento del frente
			wv.erase(wv.begin());
			wvc.erase(wvc.begin());
		}
		return tree;
	} // Construir árbol hasta la raíz enlazando en base los costes de los vértices y aristas - Devuelve los pares de enlaces entre los segmentos

	vector<vector<int>> build_tree(int root, vector<vector<bool>> graph, vector<vector<float>> edge_costs, vector<float> vertex_costs, vector<float> &accum_costs)
	{
		vector<vector<int>> tree;
		if(root < 0) return tree;

		int n = graph.size();
		// "Frente de onda"
		vector<int> wv;
		vector<float> wvc;
		vector<bool> todo(n, true);
		// Inicializar el frente
		wv.push_back(root);
		wvc.push_back(vertex_costs[root]);
		todo[root] = false;

		queue<int> nv; // Vecinos

		float cost, min_cost;
		int ind;
		bool found;

		accum_costs[root] = 0;

		while(wv.size()){
			// Encontrar vecinos del primer elemento del "frente"
			for(int i=0; i<n; i++){
				if(todo[i] && graph[wv[0]][i] && wv[0]!=i){
					nv.push(i);
				}
			}

			// Enlazar a los vecinos encontrados
			while(nv.size()){
				// Encontrar el elmento del "frente" al que conectar
				min_cost = INFINITY; ind = -1;
				for(int i=0; i<wv.size(); i++){
					if(graph[nv.front()][wv[i]]){
						// coste = coste del elemento del frente + coste de arista + coste vértice
						cost = wvc[i] + edge_costs[wv[i]][nv.front()] + vertex_costs[nv.front()];
						if(min_cost > cost){
							min_cost = cost;
							ind = i;
						}
					}
				}

				if(ind >= 0){
					// Encontrar la posisión dentro del "frente" donde insertar el vecino
					found = false;
					for(int i=0; i<wv.size(); i++){
						if(wvc[i] > min_cost){
							wv.insert(wv.begin()+i, nv.front());
							wvc.insert(wvc.begin()+i, min_cost);
							found = true;
							break;
						}
					}
					if(!found){
						wv.push_back(nv.front());
						wvc.push_back(min_cost);
					}

					// Insertar en el árbol
					tree.push_back({wv[ind], nv.front()});
					// Almacenar los costes
					accum_costs[nv.front()] = min_cost;
				}

				// Eliminar al vecino encontrado
				todo[nv.front()] = false;
				nv.pop();
			}
			// Eliminar el elemento del frente
			wv.erase(wv.begin());
			wvc.erase(wvc.begin());
		}
		return tree;
	} // Lo mismo que la función anterior, pero al mismo tiempo acumula los costes desde la raíz

	vector<vector<int>> build_trees(vector<int> roots, vector<vector<bool>> graph, vector<vector<float>> edge_costs, vector<float> vertex_costs)
	{
		vector<vector<int>> trees;
		if(roots.size() == 0) return trees;

		int n = graph.size();
		// "Frente de onda"
		vector<int> wv;
		vector<float> wvc;
		vector<bool> todo(n, true);
		// Inicializar el frente
		for(int i=0; i<roots.size(); i++){
			if(roots[i]>=0){
				wv.push_back(roots[i]);
				wvc.push_back(vertex_costs[roots[i]]);
				todo[roots[i]] = false;
			}
		}

		queue<int> nv; // Vecinos

		float cost, min_cost;
		int ind;
		bool found;

		while(wv.size()){
			// Encontrar vecinos del primer elemento del "frente"
			for(int i=0; i<n; i++){
				if(todo[i] && graph[wv[0]][i] && wv[0]!=i){
					nv.push(i);
				}
			}

			// Enlazar a los vecinos encontrados
			while(nv.size()){
				// Encontrar el elmento del "frente" al que conectar
				min_cost = INFINITY; ind = -1;
				for(int i=0; i<wv.size(); i++){
					if(graph[nv.front()][wv[i]]){
						// coste = coste del elemento del frente + coste de arista + coste vértice
						cost = wvc[i] + edge_costs[wv[i]][nv.front()] + vertex_costs[nv.front()];
						if(min_cost > cost){
							min_cost = cost;
							ind = i;
						}
					}
				}

				if(ind >= 0){
					// Encontrar la posisión dentro del "frente" donde insertar el vecino
					found = false;
					for(int i=0; i<wv.size(); i++){
						if(wvc[i] > min_cost){
							wv.insert(wv.begin()+i, nv.front());
							wvc.insert(wvc.begin()+i, min_cost);
							found = true;
							break;
						}
					}
					if(!found){
						wv.push_back(nv.front());
						wvc.push_back(min_cost);
					}

					// Insertar en el árbol
					trees.push_back({wv[ind], nv.front()});
				}

				// Eliminar al vecino encontrado
				todo[nv.front()] = false;
				nv.pop();
			}
			// Eliminar el elemento del frente
			wv.erase(wv.begin());
			wvc.erase(wvc.begin());
		}
		return trees;
	} // Construir árbol hasta la raíz enlazando en base los costes de los vértices y aristas - Devuelve los pares de enlaces entre los segmentos

	vector<vector<bool>> generate_tree_matrix(vector<vector<int>> tree_links, int n)
	{
		vector<vector<bool>> res(n,vector<bool>(n,false));
		for(int i=0; i<tree_links.size(); i++)
			res[tree_links[i][0]][tree_links[i][1]] = true;
		return res;
	} // Pasar el árbol a forma de matriz

	vector<int> compute_vertex_depth(vector<vector<int>> tree_depths, int n)
	{
		vector<int> res(n,-1); // -1 si no tienen conexión con el árbol
		int nd = tree_depths.size()-1;
		for(int i=0; i<tree_depths.size(); i++) // profundidad
			for(int j=0; j<tree_depths[i].size(); j++){ // vértices de la profundidad
				if(res[tree_depths[i][j]] < 0) res[tree_depths[i][j]] = 0; // Inicializar el valor del vértice
				res[tree_depths[i][j]] = nd - i;
			}
		return res;
	} // Generar vector con las profundidades de los vértice en el grafo

	vector<int> connectivity(vector<vector<int>> tree_links, int n)
	{
		vector<int> res(n,-1); // -1 si no tienen conexión con el árbol
		for(int i=0; i<tree_links.size(); i++){
			if(res[tree_links[i][0]] < 0) res[tree_links[i][0]] = 0; // Inicializar el valor del vértice
			if(res[tree_links[i][1]] < 0) res[tree_links[i][1]] = 0; // Inicializar el valor del vértice
			res[tree_links[i][0]]++;
		}
		return res;
	} // Conectividad (nº de enlaces) de cada vértice del árbol

	vector<int> cumulative_connectivity(vector<vector<vector<int>>> tree_links, int n)
	{
		vector<int> res(n,-1); // -1 si no tienen conexión con el árbol
		//if(tree_links.size()>2)
		for(int i=tree_links.size()-1; i>=0; i--){ // Profundidades
			for(int j=0; j<tree_links[i].size(); j++){ // Enlaces
				//cout<<i<<" - "<<j<<endl;
				//cout<<tree_links[i][j].size()<<" vértices"<<endl;
				//cout<<tree_links[i][j][0]<<endl;
				if(res[tree_links[i][j][0]] < 0) res[tree_links[i][j][0]] = 0; // Inicializar el valor del vértice
				if(res[tree_links[i][j][1]] < 0) res[tree_links[i][j][1]] = 0; // Inicializar el valor del vértice
				res[tree_links[i][j][0]] += (1 + res[tree_links[i][j][1]]); // El enlace + los enlaces de ese vértice
			}
		}
		return res;
	} // Cantidad de conexiones que "penden" de cada vértice

	vector<int> leafs(vector<vector<int>> tree)
	{
	    vector<int> res;
	    bool desc;
	    for(int i=0; i<tree.size(); i++){
	        desc = false;
	        for(int j=0; j<tree.size(); j++){
	            if(tree[i][1] == tree[j][0]){ // Tiene un descendiente
	                desc = true;
	            }
	        }
	        if(!desc) res.push_back(tree[i][1]);
	    }
	    return res;   
	} // Devuelve los nodos que no tienen descendientes

	vector<vector<int>> all_paths(vector<vector<int>> tree, vector<int> leafs)
	{
	    vector<vector<int>> res(leafs.size());
	    int it, j;
	    for(int i=0; i<leafs.size(); i++){
	        it = leafs[i];
	        res[i].push_back(it);
	        //for(int j=0; j<tree.size(); j++){
	        j = tree.size()-1;
	        while(it != tree[0][0]){
	            if(tree[j][1] == it){
	                it = tree[j][0];
	                res[i].push_back(it);
	            }
	            j--;
	        }
	    }
	    return res;
	} // Todos los caminos desde los nodos "leafs" hasta la raíz del árbol

	vector<vector<int>> all_paths_(vector<vector<int>> tree, vector<int> leafs)
	{
	    vector<vector<int>> res(leafs.size());
	    int it, j;
	    for(int i=0; i<leafs.size(); i++){
	        it = leafs[i];
	        res[i].push_back(it);
	        //for(int j=0; j<tree.size(); j++){
	        j = tree.size()-1;
	        while(it != tree[0][0]){
	            if(tree[j][1] == it){
	                it = tree[j][0];
	                res[i].insert(res[i].begin(), it);
	            }
	            j--;
	        }
	    }
	    return res;
	} // Todos los caminos desde la raíz del árbol hasta todos los nodos "leafs"

	vector<vector<int>> all_paths_ascend(vector<vector<int>> tree)
	{
		return all_paths(tree, leafs(tree));
	} // Todos los caminos desde las "hojas" hasta la raíz

	vector<vector<int>> all_paths_descend(vector<vector<int>> tree)
	{
		return all_paths_(tree, leafs(tree));
	} // Todos los caminos desde la raíz hasta las "hojas"

	template <typename T>
	vector<T> accumulate_tree_variable(vector<vector<int>> all_paths, vector<T> variable)
	{
	    vector<T> res(variable.size(),0);
	    // Acumular los valores sin replicar los valores propios de los nodos de las ramas que ya han sido acumulados
	    vector<bool> acum(variable.size(), true);
	    T val;
	    for(int i=0; i<all_paths.size(); i++){
	        val = variable[all_paths[i][0]];
	        res[all_paths[i][0]] = val;
	        acum[all_paths[i][0]] = false;
	        for(int j=1; j<all_paths[i].size(); j++){
	            if(acum[all_paths[i][j]]){ // Todavía no se ha acumulado nada en este nodo
	                val += variable[all_paths[i][j]];
	                acum[all_paths[i][j]] = false;
	            }
	            res[all_paths[i][j]] += val;
	        }
	    }
	    return res;
	} // Cantidad de objetivos que hay en cada vértice del árbol acumulados hasta la raíz

	template <typename T>
	vector<T> accumulate_tree_costs_(vector<vector<int>> all_paths, vector<T> vcosts, vector<vector<T>> acosts)
	{
		// - all_paths: todos los caminos que llevan hasta la raíz
		// - vcosts: costes de los vértices (coste de "trabajo")
		// - acosts: costes de las aristas (coste de "viaje")
	    vector<T> res(vcosts.size(),0);
	    // Acumular los valores sin replicar los valores propios de los nodos de las ramas que ya han sido acumulados
	    vector<bool> vacum(vcosts.size(), true);
	    vector<vector<bool>> aacum(acosts.size(), vector<bool>(acosts[0].size(), true));
	    T vval, aval;
	    for(int i=0; i<all_paths.size(); i++){
	        vval = vcosts[all_paths[i][0]];
	        aval = 0;
	        res[all_paths[i][0]] = vval;
	        vacum[all_paths[i][0]] = false;
	        for(int j=1; j<all_paths[i].size(); j++){
	            if(vacum[all_paths[i][j]]){ // Todavía no se ha acumulado nada en este nodo
	                vval += vcosts[all_paths[i][j]];
	                vacum[all_paths[i][j]] = false;
	            }
	            if(aacum[all_paths[i][j]][all_paths[i][j-1]]){ // Todavía no se ha acumulado nada en este nodo
	                aval += acosts[all_paths[i][j]][all_paths[i][j-1]];
	                aacum[all_paths[i][j]][all_paths[i][j-1]] = false;
	            }
	            res[all_paths[i][j]] += (vval + aval);
	        }
	    }
	    return res;
	} // Cantidad de objetivos que hay en cada vértice y arista del árbol acumulados hasta la raíz

	template <typename T>
	vector<T> accumulate_tree_costs(vector<vector<int>> all_paths, vector<T> vcosts, vector<vector<T>> acosts)
	{
		// - all_paths: todos los caminos que llevan hasta la raíz
		// - vcosts: costes de los vértices (coste de "trabajo")
		// - acosts: costes de las aristas (coste de "viaje")
	    vector<T> res(vcosts.size(),0);
	    //T vval, aval;
	    for(int i=0; i<all_paths.size(); i++){
	        res[all_paths[i][0]] = vcosts[all_paths[i][0]];
	        vcosts[all_paths[i][0]] = 0;
	        for(int j=1; j<all_paths[i].size(); j++){
	        	// Acumulo los costes del vértice y de la arista
	        	res[all_paths[i][j]] += (vcosts[all_paths[i][j]] + acosts[all_paths[i][j]][all_paths[i][j-1]]);
	        	// Anulo los costes tanto de los vértices como de las aristas para no volver a acumular sus valores
                vcosts[all_paths[i][j]] = 0;
                acosts[all_paths[i][j]][all_paths[i][j-1]] = acosts[all_paths[i][j-1]][all_paths[i][j]] = 0;
	        }
	    }
	    return res;
	} // Cantidad de objetivos que hay en cada vértice y arista del árbol acumulados hasta la raíz

	template <typename T>
	vector<T> accumulate_tree_variable_from_graph(vector<vector<int>> all_paths, vector<vector<T>> variable)
	{
	    vector<T> res(variable.size(),0);
	    // Acumular los valores sin replicar los valores propios de los nodos de las ramas que ya han sido acumulados
	    vector<bool> acum(variable.size(), true);
	    T val;
	    for(int i=0; i<all_paths.size(); i++){
	        val = variable[all_paths[i][0]][all_paths[i][1]] + variable[all_paths[i][1]][all_paths[i][0]];
	        res[all_paths[i][0]] = val;
	        acum[all_paths[i][0]] = false;
	        for(int j=1; j<all_paths[i].size(); j++){
	            if(acum[all_paths[i][j]]){ // Todavía no se ha acumulado nada en este nodo
	                val += (variable[all_paths[i][j]][all_paths[i][j-1]] + variable[all_paths[i][j-1]][all_paths[i][j]]);
	                acum[all_paths[i][j]] = false;
	            }
	            res[all_paths[i][j]] += val;
	        }
	    }
	    return res;
	} // Cantidad de objetivos que hay en cada vértice del árbol acumulados hasta la raíz

	vector<Poss<int>> form_tree_pos(Poss<int> pos, vector<vector<int>> tree_links)
	{
	    vector<Poss<int>> res(tree_links.size());

	    for(int i=0; i<tree_links.size(); i++){
	        res[i](2);
	        res[i].x[0] = pos.x[tree_links[i][0]]; res[i].y[0] = pos.y[tree_links[i][0]];
	        res[i].x[1] = pos.x[tree_links[i][1]]; res[i].y[1] = pos.y[tree_links[i][1]];
	    }

	    return res;   
	} // Forma el vector con las posiciones del árbol (Para despues representar)

	vector<Poss<int>> form_tree_pos(Poss<int> centroids, vector<Poss<int>> adjacent_points, vector<vector<int>> tree_links)
	{
	    vector<Poss<int>> res(tree_links.size());

	    for(int i=0; i<tree_links.size(); i++){
	        res[i](4);
	        res[i].x[0] = centroids.x[tree_links[i][0]]; res[i].y[0] = centroids.y[tree_links[i][0]]; // Centroide agente 1

	        res[i].x[1] = adjacent_points[tree_links[i][0]].x[tree_links[i][1]]; res[i].y[1] = adjacent_points[tree_links[i][0]].y[tree_links[i][1]]; // Adyacente agente 1
	        res[i].x[2] = adjacent_points[tree_links[i][1]].x[tree_links[i][0]]; res[i].y[2] = adjacent_points[tree_links[i][1]].y[tree_links[i][0]]; // Adyacente agente 2

	        res[i].x[3] = centroids.x[tree_links[i][1]]; res[i].y[3] = centroids.y[tree_links[i][1]]; // Centride agente 2
	    }

	    return res;   
	} // Forma el vector con las posiciones del árbol usando puntos de las fronteras (Para despues representar)

	vector<Path> form_tree_paths(Poss<int> centroids, vector<Poss<int>> adjacent_points, vector<vector<int>> tree_links, vector<vector<float>> grad)
	{
	    vector<Path> res(tree_links.size());
	    Path camino;
	    for(int i=0; i<tree_links.size(); i++){
	        res[i].gradient_descent_(grad, adjacent_points[tree_links[i][0]].x[tree_links[i][1]],adjacent_points[tree_links[i][0]].y[tree_links[i][1]]);
	        camino.gradient_descent_(grad, adjacent_points[tree_links[i][1]].x[tree_links[i][0]],adjacent_points[tree_links[i][1]].y[tree_links[i][0]]);
	        res[i].append_pos(camino);
	        camino.clear();
	    }

	    return res;   
	} // Forma el vector con las posiciones del árbol usando puntos de las fronteras (Para despues representar)

	template <typename T>
	void save(vector<Poss<T>> poss, string filename)
	{
		vector<T> values(4,0);
    	for(int i=0; i<poss.size(); i++){
    		// Vector de las posiciones
    		values[0] = poss[i].x[0]; values[1] = poss[i].y[0];
    		values[2] = poss[i].x[1]; values[3] = poss[i].y[1];
    		// Guardar (en el mismo fichero)
    		save_vect_(filename.c_str(), values); 
    	}
	}

};

#endif