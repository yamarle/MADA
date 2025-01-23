
#include <mada/util/nearest_neighbor.hpp>

vector<int> NN::allocate(vector<vector<float>> costs)
{
	// Asignación de las tareas
	int na=costs.size();
	int ng=costs[0].size();

	vector<int> alloc(na,-1);
	vector<bool> to_alloc(ng,true);
	int ind1, ind2;
	float min;
	for(int c=0; c<na; c++){
		// Encontrar el mínimo de la matriz
		min = myINF;
		for(int i=0; i<na; i++){
			if(alloc[i]<0)
			for(int j=0; j<ng; j++){
				if(to_alloc[j] && costs[i][j]<min){
					min = costs[i][j];
					ind1 = i; ind2 = j;
				}
			}
		}
		// Asignar
		alloc[ind1]=ind2;
		to_alloc[ind2]=false;
	}

	return alloc;
}

vector<int> NN::simple(Poss<int> agents, Poss<int> goals, vector<vector<float>> grid){
	int na, ng;
	na = agents.x.size();
	ng = goals.x.size();
	vector<vector<float>> costs(na,vector<float>(ng,myINF));

	// Obtener la martiz de costes
	vector<vector<float>> grad;
	for(int i=0; i<na; i++){
		FMM gr(agents.x[i],agents.y[i],grid);
		grad = gr.compute_gradient_();
		for(int j=0; j<ng; j++)
			if(i!=j)
				costs[i][j] = grad[goals.x[j]][goals.y[j]];
	}

	// Asignación de las tareas
	vector<int> alloc(na,-1);
	vector<bool> to_alloc(ng,true);
	int ind1, ind2;
	float min;
	for(int c=0; c<na; c++){
		// Encontrar el mínimo de la matriz
		min = myINF;
		for(int i=0; i<na; i++){
			if(alloc[i]<0)
			for(int j=0; j<ng; j++){
				if(to_alloc[j] && costs[i][j]<min){
					min = costs[i][j];
					ind1 = i; ind2 = j;
				}
			}
		}
		// Asignar
		alloc[ind1]=ind2;
		to_alloc[ind2]=false;
	}

	return alloc;
} // Asignación al vecino más cercano (los gradientes se calculan dentro)

vector<int> NN::simple(vector<vector<vector<float>>> grad, Poss<int> goals){
	int na, ng;
	na = grad.size();
	ng = goals.x.size();
	vector<vector<float>> costs(na,vector<float>(ng,myINF));

	// Obtener la martiz de costes
	for(int i=0; i<na; i++){
		for(int j=0; j<ng; j++)
			if(i!=j)
				costs[i][j] = grad[i][goals.x[j]][goals.y[j]];
	}

	// Asignación de las tareas
	vector<int> alloc(na,-1);
	vector<bool> to_alloc(ng,true);
	int ind1, ind2;
	float min;
	for(int c=0; c<na; c++){
		// Encontrar el mínimo de la matriz
		min = myINF;
		for(int i=0; i<na; i++){
			if(alloc[i]<0)
			for(int j=0; j<ng; j++){
				if(to_alloc[j] && costs[i][j]<min){
					min = costs[i][j];
					ind1 = i; ind2 = j;
				}
			}
		}
		// Asignar
		alloc[ind1]=ind2;
		to_alloc[ind2]=false;
	}

	return alloc;
} // Asignación al vecino más cercano (los gradientes se pasan por variable)

vector<int> NN::simple(Poss<int> agents, Poss<int> goals, vector<vector<float>> grid, vector<Path> &paths){
	int na, ng;
	na = agents.x.size();
	ng = goals.x.size();
	vector<vector<float>> costs(na,vector<float>(ng,myINF));

	// Obtener la martiz de costes
	vector<vector<float>> grad;
	for(int i=0; i<na; i++){
		FMM gr(agents.x[i],agents.y[i], grid);
		grad = gr.compute_gradient_();
		for(int j=0; j<ng; j++)
			if(i!=j)
				costs[i][j] = grad[goals.x[j]][goals.y[j]];
	}

	// Asignación de las tareas
	vector<int> alloc(na,-1);
	vector<bool> to_alloc(ng,true);
	int ind1, ind2;
	float min;
	for(int c=0; c<na; c++){
		// Encontrar el mínimo de la matriz
		min = myINF;
		for(int i=0; i<na; i++){
			if(alloc[i]<0)
			for(int j=0; j<ng; j++){
				if(to_alloc[j] && costs[i][j]<min){
					min = costs[i][j];
					ind1 = i; ind2 = j;
				}
			}
		}
		// Asignar
		alloc[ind1]=ind2;
		to_alloc[ind2]=false;
	}

	return alloc;
} // Asignación al vecino más cercano, se calculan los caminos (los gradientes se calculan dentro)

vector<int> NN::simple(vector<vector<vector<float>>> grad, Poss<int> goals, vector<Path> &paths){
	int na, ng;
	na = grad.size();
	ng = goals.x.size();
	vector<vector<float>> costs(na,vector<float>(ng,myINF));
	vector<vector<Path>> _paths(na,vector<Path>(ng));

	// Obtener la martiz de costes
	for(int i=0; i<na; i++){
		for(int j=0; j<ng; j++)
			if(i!=j){
				costs[i][j] = grad[i][goals.x[j]][goals.y[j]];
				_paths[i][j].gradient_descent_(grad[i],goals.x[j],goals.y[j]);
			}
	}

	// Asignación de las tareas
	vector<int> alloc(na,-1);
	vector<bool> to_alloc(ng,true);
	int ind1, ind2;
	float min;
	for(int c=0; c<na; c++){
		// Encontrar el mínimo de la matriz
		min = myINF;
		for(int i=0; i<na; i++){
			if(alloc[i]<0)
			for(int j=0; j<ng; j++){
				if(to_alloc[j] && costs[i][j]<min){
					min = costs[i][j];
					ind1 = i; ind2 = j;
				}
			}
		}
		// Asignar
		alloc[ind1]=ind2;
		to_alloc[ind2]=false;
	}

	if(paths.size()!=na) paths.resize(na);
	for(int i=0; i<na; i++){
		if(alloc[i]>=0)
			paths[i] = _paths[i][alloc[i]];
	}

	return alloc;
} // Asignación al vecino más cercano, se calculan los caminos (los gradientes se pasan por variable)

// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
// 												NN recursivo
// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
vector<vector<float>> NN::form_cost_matrix(vector<vector<float>> costs, vector<int> agents, vector<bool> active, vector<int> &ref)
{
	// Recibe:
	// 	- costs: es la matriz de costes completa
	// 	- agents: vector con los índices donde se encuentran los agentes
	// 	- active: vector con los que siguen pendientes (los agentes vienen anulados)
	// Devuelve:
	// 	- new_costs: matriz tamaño nagentes x ngoals_activos
	int na = agents.size(); // Número de agentes
	int n = active.size(); // Número de agentes + puntos
	vector<vector<float>> res(na);
	ref.clear();
	for(int i=0; i<n; i++)
		if(active[i]) ref.push_back(i);
	for(int i=0; i<na; i++){
		//res[i] = costs[i]; // Los costes tal cual
		// Anular los que no están activos (ya hechos)
		for(int j=0; j<n; j++){
			//if(!active[j]) res[i][j] = INF;
			if(active[j]){
				res[i].push_back(costs[agents[i]][j]);
			}
		}
	}
	return res;
} // Devuelve la matriz de costes preparada para el húngaro recursivo

vector<vector<int>> NN::recursive(vector<vector<float>> costs, int n)
{
	// La matriz de costes ya lleva los costes desde las posiciones
	int nalloc = costs.size(); // Número de asignaciones que hay que realizar
	int _n = nalloc;
	// Vectores de asignación
	vector<vector<int>> res(n);
	// Inicializar con los valores iniciales (los primeros valores de cada vector de asignación son los n agentes)
	vector<bool> active(_n,true);
	vector<int> curr_agents(n);
	for(int i=0; i<n; i++){
		res[i].push_back(i);
	}
	vector<int> alloc, alloc_; // Vector auxiliar para la asignación (valor: trabajador, índice: trabajo)

	vector<vector<float>> _costs;
	vector<int> ref(n);
	nalloc-=n; // No hay que asignar los agentes
	while(nalloc){ // Hasta no haber recubierto todos
		//cout<<"Quedan "<<nalloc<<" por asignar"<<endl;
		for(int i=0; i<n; i++){
			active[res[i][res[i].size()-1]]=false;
			curr_agents[i]=res[i][res[i].size()-1];
		}
		//sh_vect_h(active,"active");
		//sh_vect_h(curr_agents,"curr_agents");
		_costs = form_cost_matrix(costs,curr_agents,active,ref);
		//sh_matr_al(_costs,"new_costs");
		//sh_vect_h(ref,"ref");
		// Asignar con el NN
		alloc = allocate(_costs);
		alloc_ = alloc;
		for(int i=0; i<alloc.size(); i++){
			alloc_[i]=ref[alloc[i]];
		}
		//sh_vect_h(alloc,"alloc_fake");
		//sh_vect_h(alloc_,"alloc_real");
		for(int i=0; i<alloc.size(); i++){
			if(alloc[i]>=0){
				//res[alloc[i]].push_back(i); // ESTO NO SÉ SI ES ASÍ
				res[i].push_back(ref[alloc[i]]);
				nalloc--;
			}
		}
	}
	
	return res;	
} // Realiza una asignación utilizando el Húngaro de manera iterativa

