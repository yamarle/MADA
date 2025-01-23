
#include <mada/util/hungarian.hpp>

vector<int> Hungarian::simple(Poss<int> agents, Poss<int> goals, vector<vector<float>> grid){
	int na, ng;
	na = agents.x.size();
	ng = goals.x.size();
	vector<vector<float>> costs(na,vector<float>(ng,myINF));
	vector<int> alloc(na);

	// Obtener la martiz de costes
	vector<vector<float>> grad;
	for(int i=0; i<na; i++){
		FMM gr(agents.x[i],agents.y[i],grid);
		grad = gr.compute_gradient_();
		for(int j=0; j<ng; j++)
			if(i!=j)
				costs[i][j] = grad[goals.x[j]][goals.y[j]];
	}

	hungarian hung;
	hung.solve(costs,alloc);

	return alloc;
} // Húngaro normal (se calculan dentro los gradientes de los agentes)

vector<int> Hungarian::simple(vector<vector<vector<float>>> grad, Poss<int> goals){
	int na, ng;
	na = grad.size();
	ng = goals.x.size();
	vector<vector<float>> costs(na,vector<float>(ng,myINF));
	vector<int> alloc(na);

	// Obtener la martiz de costes
	for(int i=0; i<na; i++){
		for(int j=0; j<ng; j++)
			if(i!=j)
				costs[i][j] = grad[i][goals.x[j]][goals.y[j]];
	}

	hungarian hung;
	hung.solve(costs,alloc);

	return alloc;
} // Húngaro normal (se pasan los gradientes de los agentes)

vector<int> Hungarian::simple_cumulative(vector<vector<vector<float>>> grad, Poss<int> goals, vector<float> ecosts){
	int na, ng;
	na = grad.size();
	ng = goals.x.size();
	vector<vector<float>> costs(na,vector<float>(ng,myINF));
	vector<int> alloc(na);

	// Obtener la martiz de costes
	for(int i=0; i<na; i++){
		for(int j=0; j<ng; j++)
			if(i!=j)
				costs[i][j] = grad[i][goals.x[j]][goals.y[j]]+ecosts[j];
	}

	hungarian hung;
	hung.solve(costs,alloc);

	return alloc;
} // Húngaro normal (se pasan los gradientes de los agentes)

vector<int> Hungarian::simple(Poss<int> agents, Poss<int> goals, vector<vector<float>> grid, vector<Path> &paths){
	int na, ng;
	na = agents.x.size();
	ng = goals.x.size();
	vector<vector<float>> costs(na,vector<float>(ng,myINF));
	vector<vector<Path>> _paths(na,vector<Path>(ng));
	vector<int> alloc(na);

	// Obtener la martiz de costes
	vector<vector<float>> grad;
	for(int i=0; i<na; i++){
		FMM gr(agents.x[i],agents.y[i],grid);
		grad = gr.compute_gradient_();
		for(int j=0; j<ng; j++)
			if(i!=j){
				costs[i][j] = grad[goals.x[j]][goals.y[j]];
				_paths[i][j].gradient_descent_(grad,goals.x[j],goals.y[j]);
			}
	}

	hungarian hung;
	hung.solve(costs,alloc);

	if(paths.size()!=na) paths.resize(na);
	for(int i=0; i<na; i++){
		if(alloc[i]>=0)
			paths[i] = _paths[i][alloc[i]];
	}

	return alloc;
}  // Húngaro normal,se calcualan directamente los caminos (se calculan dentro los gradientes de los agentes)

vector<int> Hungarian::simple(vector<vector<vector<float>>> grad, Poss<int> goals, vector<Path> &paths){
	int na, ng;
	na = grad.size();
	ng = goals.x.size();
	vector<vector<float>> costs(na,vector<float>(ng,myINF));
	vector<vector<Path>> _paths(na,vector<Path>(ng));
	vector<int> alloc(na);

	// Obtener la martiz de costes
	for(int i=0; i<na; i++){
		for(int j=0; j<ng; j++)
			if(i!=j){
				costs[i][j] = grad[i][goals.x[j]][goals.y[j]];
				_paths[i][j].gradient_descent_(grad[i],goals.x[j],goals.y[j]);
			}
	}

	hungarian hung;
	hung.solve(costs,alloc);

	if(paths.size()!=na) paths.resize(na);
	for(int i=0; i<na; i++){
		if(alloc[i]>=0)
			paths[i] = _paths[i][alloc[i]];
	}

	return alloc;
}  // Húngaro normal,se calcualan directamente los caminos (se pasan los gradientes de los agentes)

vector<int> Hungarian::priority(Poss<int> agents, Poss<int> goals, vector<bool> priority, vector<vector<float>> grid, vector<Path> &paths){
	// Los goals que recibe son todas las posiciones que alcanza la cadena

	int na, ng, np; // nª agentes, goals totales, goals prioritarios (relays)
	na = agents.x.size();
	ng = goals.x.size();

	np=0;
	for(int i=0; i<ng; i++)
		if(priority[i]) np++;

	vector<int> alloc(na,-1);

	// Obtener cómo se visitan los objetivos
	if(na>np){ // Hay sufucientes agentes como para formar la cadena y al menos uno para visitar goals
		vector<vector<float>> costs(na,vector<float>(ng,myINF));
		vector<vector<Path>> _paths(na,vector<Path>(ng));
		// Los costes de los prioritarios deben ser mucho menores que los de los que no lo son
		int x_,y_;
		float min = myINF, maxp = 0; // coste mínimo, coste máximo de los prioritarios
		vector<vector<float>> grad;
		for(int i=0; i<na; i++){
			FMM gr(agents.x[i],agents.y[i],grid);
			grad = gr.compute_gradient_();
			for(int j=0; j<ng; j++){
				costs[i][j] = grad[goals.x[j]][goals.y[j]];
				_paths[i][j].gradient_descent_(grad,goals.x[j],goals.y[j]);
				if(min>costs[i][j]){
					x_ = i; y_ = j;
					min = costs[i][j];
				}
				if(priority[j]){
					if(maxp<costs[i][j]) maxp = costs[i][j];
				}
			}
		}

		// Normalizar los costes de los prioritarios, así siempre van a ser elegidos antes que los demás
		for(int i=0; i<na; i++){
			for(int j=0; j<ng; j++){
				if(priority[j] && min){
					costs[i][j]*=(min/maxp);
				}
			}
		}

		// Aplicar Húngaro
		hungarian hung;
		hung.solve(costs,alloc);

		if(paths.size()!=na) paths.resize(na);
		for(int i=0; i<na; i++){
			if(alloc[i]>=0)
				paths[i] = _paths[i][alloc[i]];
		}
		
	}
	return alloc;
} // Húngaro con prioridad para algunos goals (se calculan dentro los gradientes de los agentes)

vector<int> Hungarian::priority(vector<vector<vector<float>>> grad, Poss<int> goals, vector<bool> priority, vector<Path> &paths){
	// Los goals que recibe son todas las posiciones que alcanza la cadena

	int na, ng, np; // nª agentes, goals totales, goals prioritarios (relays)
	na = grad.size();
	ng = goals.x.size();

	np=0;
	for(int i=0; i<ng; i++)
		if(priority[i]) np++;

	vector<int> alloc(na,-1);

	// Obtener cómo se visitan los objetivos
	if(na>np){ // Hay sufucientes agentes como para formar la cadena y al menos uno para visitar goals
		vector<vector<float>> costs(na,vector<float>(ng,myINF));
		vector<vector<Path>> _paths(na,vector<Path>(ng));
		// Los costes de los prioritarios deben ser mucho menores que los de los que no lo son
		int x_,y_;
		float min = myINF, maxp = 0; // coste mínimo, coste máximo de los prioritarios
		for(int i=0; i<na; i++){
			for(int j=0; j<ng; j++){
				costs[i][j] = grad[i][goals.x[j]][goals.y[j]];
				_paths[i][j].gradient_descent_(grad[i],goals.x[j],goals.y[j]);
				if(min>costs[i][j]){
					x_ = i; y_ = j;
					min = costs[i][j];
				}
				if(priority[j]){
					if(maxp<costs[i][j]) maxp = costs[i][j];
				}
			}
		}

		// Normalizar los costes de los prioritarios, así siempre van a ser elegidos antes que los demás
		for(int i=0; i<na; i++){
			for(int j=0; j<ng; j++){
				if(priority[j] && min){
					costs[i][j]*=(min/maxp);
				}
			}
		}

		// Aplicar Húngaro
		hungarian hung;
		hung.solve(costs,alloc);

		if(paths.size()!=na) paths.resize(na);
		for(int i=0; i<na; i++){
			if(alloc[i]>=0)
				paths[i] = _paths[i][alloc[i]];
		}
		
	}
	return alloc;
} // Húngaro con prioridad para algunos goals (se pasan los gradientes de los agentes)

vector<int> Hungarian::priority(vector<vector<vector<float>>> grad, Poss<int> goals, vector<bool> priority){
	// Los goals que recibe son todas las posiciones que alcanza la cadena

	int na, ng, np; // nª agentes, goals totales, goals prioritarios (relays)
	na = grad.size();
	ng = goals.x.size();

	np=0;
	for(int i=0; i<ng; i++)
		if(priority[i]) np++;

	vector<int> alloc(na,-1);

	// Obtener cómo se visitan los objetivos
	if(na>np){ // Hay sufucientes agentes como para formar la cadena y al menos uno para visitar goals
		vector<vector<float>> costs(na,vector<float>(ng,myINF));
		vector<vector<Path>> _paths(na,vector<Path>(ng));
		// Los costes de los prioritarios deben ser mucho menores que los de los que no lo son
		int x_,y_;
		float min = myINF, maxp = 0; // coste mínimo, coste máximo de los prioritarios
		for(int i=0; i<na; i++){
			for(int j=0; j<ng; j++){
				costs[i][j] = grad[i][goals.x[j]][goals.y[j]];
				_paths[i][j].gradient_descent_(grad[i],goals.x[j],goals.y[j]);
				if(min>costs[i][j]){
					x_ = i; y_ = j;
					min = costs[i][j];
				}
				if(priority[j]){
					if(maxp<costs[i][j]) maxp = costs[i][j];
				}
			}
		}

		// Normalizar los costes de los prioritarios, así siempre van a ser elegidos antes que los demás
		for(int i=0; i<na; i++){
			for(int j=0; j<ng; j++){
				if(priority[j] && min){
					costs[i][j]*=(min/maxp);
				}
			}
		}

		// Aplicar Húngaro
		hungarian hung;
		hung.solve(costs,alloc);
		
	}
	return alloc;
} // Húngaro con prioridad para algunos goals (se pasan los gradientes de los agentes)

// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
// 												Húngaro recursivo
// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
vector<vector<float>> Hungarian::form_cost_matrix(vector<vector<float>> costs, vector<int> agents, vector<bool> active, vector<int> &ref)
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

vector<vector<float>> Hungarian::form_cost_matrix(vector<vector<float>> costs, vector<float> cum_costs, vector<int> agents, vector<bool> active, vector<int> &ref)
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
				res[i].push_back(costs[agents[i]][j]+cum_costs[agents[i]]);
			}
		}
	}
	return res;
} // Devuelve la matriz de costes preparada para el húngaro recursivo

vector<vector<int>> Hungarian::recursive(vector<vector<float>> costs, int n)
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
	hungarian hung;
	vector<vector<float>> _costs;
	vector<int> ref(n);
	nalloc-=n; // No hay que asignar los agentes
	while(nalloc){ // Hasta no haber recubierto todos
		for(int i=0; i<n; i++){
			active[res[i][res[i].size()-1]]=false;
			curr_agents[i]=res[i][res[i].size()-1];
		}
		_costs = form_cost_matrix(costs,curr_agents,active,ref);
		// Asignar con el Húngaro
		hung.solve(_costs,alloc);
		alloc_ = alloc;
		for(int i=0; i<alloc.size(); i++){
			alloc_[i]=ref[alloc[i]];
		}
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

vector<vector<int>> Hungarian::recursive_cum(vector<vector<float>> costs, int n)
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
	hungarian hung;
	vector<vector<float>> _costs;
	vector<int> ref(n);
	vector<float> cum_costs(n,0);
	nalloc-=n; // No hay que asignar los agentes
	while(nalloc){ // Hasta no haber recubierto todos
		for(int i=0; i<n; i++){
			active[res[i][res[i].size()-1]]=false;
			curr_agents[i]=res[i][res[i].size()-1];
		}
		//_costs = form_cost_matrix(costs,curr_agents,active,ref);
		_costs = form_cost_matrix(costs,cum_costs,curr_agents,active,ref);
		// Asignar con el Húngaro
		hung.solve(_costs,alloc);
		alloc_ = alloc;
		for(int i=0; i<alloc.size(); i++){
			alloc_[i]=ref[alloc[i]];
		}

		/*
		//cout<<endl<<costs.size()<<" x "<<costs[0].size()<<endl<<endl;
		sh_vect_h(cum_costs,"Costes acumulados");
		cout<<"Tamaño _costs: "<<_costs.size()<<" x "<<_costs[0].size()<<endl;
		cout<<"Tamaño  costs: "<<costs.size()<<" x "<<costs[0].size()<<endl;
		sh_vect_h(alloc,"alloc ");
		sh_vect_h(alloc_,"alloc_");
		sh_vect_h(ref,"Ref");
		sh_vect_h(curr_agents,"AGENTES");
		*/

		//cout<<"Le meto: "<<endl;
		for(int i=0; i<alloc.size(); i++){
			if(alloc[i]>=0){
				//res[alloc[i]].push_back(i); // ESTO NO SÉ SI ES ASÍ
				res[i].push_back(ref[alloc[i]]);
				cum_costs[i]+=costs[curr_agents[i]][ref[alloc[i]]]; // <----------------------------------
				//cout<<ref[i]<<", "<<alloc[i]<<", "<<ref[alloc[i]]<<" -> "<<costs[curr_agents[i]][ref[alloc[i]]]<<endl;
				nalloc--;
			}
		}
		//cin.get();
	}

	return res;	
} // Realiza una asignación utilizando el Húngaro de manera iterativa (costes cumulativos de los agentes)

vector<vector<int>> Hungarian::swap_2opt(vector<vector<int>> initial_alloc, vector<vector<float>> costs, int n)
{
	vector<vector<int>> alloc = initial_alloc;
	vector<int> _alloc;
	float cumc, currc, bestc;
	int ind;
	vector<int> excess(costs.size(),0);

	int gn = costs[0].size();
	for(int g=1; g<costs[0].size(); g++){
		for(int i=0; i<n; i++){
			if(initial_alloc[i][g]){
				ind=-1;
				bestc = costs[initial_alloc[i][g-1]][initial_alloc[i][g]];
				// repasar los costes de los demás agentes
				for(int j=0; j<n; j++){
					if(i!=j && initial_alloc[j][g]){
						// Comprobar si mejora el coste al intercambiar a otro agente
						cumc = costs[initial_alloc[j][g-1]][initial_alloc[j][g]] + costs[initial_alloc[j][g]][initial_alloc[i][g]];
						if(cumc<bestc){
							// Guardar como mejor candidato
							ind = j;
							bestc = cumc;
						}
					}
				}
				if(ind>=0){
					// Asignarle el objetivo del agente i al agente ind
					alloc[ind].insert(alloc[ind].begin()+excess[ind]+g,alloc[i][g]);
					excess[ind]++;
					// Quitar el objetivo de la lista del agente i
					alloc[i].erase(alloc[i].begin()+g);
					// Igualar la cantidad de objetivos en la matriz de asignaciones
					for(int j=0; j<n; j++){
						if(j!=ind)
						alloc[j].push_back(0);
					}
				}
			}
		}
	}

	return alloc;

} // Comprobar si existe un movimiento mejor, y cambiar si es así
