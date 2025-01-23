#ifndef ITERATIVE_ALLOCATION_HPP
#define ITERATIVE_ALLOCATION_HPP

#include "hungarian.hpp"
#include "nearest_neighbor.hpp"

struct alloc_variables{
	vector<vector<float>> costs;
	vector<vector<int>> alloc;

	vector<vector<Path>> paths; // Todos los caminos por agente y entre goals y goal
	vector<Path> ag_paths; // Caminos de los agentes
	vector<Poss<int>> agent_goals; // La asignación de los goals a cada agente
};

vector<vector<vector<float>>> compute_all_gradients(Poss<int> pos, vector<vector<float>> grid)
	{
		// pos: la primera posición es la de origen
		vector<vector<vector<float>>> res(pos.x.size());
		for(int i=0; i<pos.x.size(); i++){
			FMM gr(pos.x[i],pos.y[i],grid);
			res[i] = gr.compute_gradient_();
		}
		return res;
	}

alloc_variables iterative_allocation(Poss<int> pos, vector<vector<vector<float>>> grads, int nagents, uint alg)
{
	alloc_variables res;

	// Formar la matriz de costes
	int npos_tot = pos.x.size();
	res.costs.resize(npos_tot, vector<float>(npos_tot,Hungarian::myINF));
	for(int i=0; i<npos_tot; i++){
		for(int j=i+1; j<npos_tot; j++){
			res.costs[i][j] = grads[i][pos.x[j]][pos.y[j]];
			res.costs[j][i] = res.costs[i][j]; // Hay un error de centímetros, por eso replico
		}
	}

	// Asignación
	if(alg==0) // Nearest Neighbor normal
		res.alloc = NN::recursive(res.costs, nagents);
	else if(alg==1) // Húngaro normal
		res.alloc = Hungarian::recursive(res.costs, nagents);
	else if(alg==2) // Húngaro cumulativo
		res.alloc = Hungarian::recursive_cum(res.costs, nagents);

	res.agent_goals.resize(nagents);
	for(int i=0; i<res.alloc.size(); i++){
		for(int j=1; j<res.alloc[i].size(); j++){
			res.agent_goals[i].x.push_back(pos.x[res.alloc[i][j]]);
			res.agent_goals[i].y.push_back(pos.y[res.alloc[i][j]]);
		}
	}

	// Esto es para completar la matriz
	// únicamente para mostrar por pantalla
	int max_all = 0;
	for(int i=0; i<res.alloc.size(); i++)
		if(max_all<res.alloc[i].size()) max_all=res.alloc[i].size();
	for(int i=0; i<res.alloc.size(); i++)
		while(max_all>res.alloc[i].size()) res.alloc[i].push_back(0);

	// Formar los caminos
	res.paths.resize(nagents);
	for(int i=0; i<res.alloc.size(); i++){
		res.paths[i].resize(res.alloc[i].size());
		//for(int j=1; j<res.alloc[i].size(); j++){
		for(int j=1; j<res.alloc[i].size(); j++){
			if(res.alloc[i][j]){
				res.paths[i][j].gradient_descent_(grads[res.alloc[i][j-1]], pos.x[res.alloc[i][j]], pos.y[res.alloc[i][j]]);
			}
		}
	}

	res.ag_paths.resize(nagents);
	for(int i=0; i<nagents; i++){
		for(int j=0; j<res.paths[i].size(); j++){
			if(res.paths[i][j].tam){
				//res.ag_paths[i].push_front_(res.paths[i][j]);
				res.ag_paths[i].append_(res.paths[i][j]);
			}
		}
	}

	return res;
} // Asignación interativa solo considerando las distancias

alloc_variables iterative_allocation(Poss<int> pos, vector<vector<vector<float>>> grads, int nagents, float speed, float work_time, uint alg)
{
	alloc_variables res;

	// Formar la matriz de costes
	int npos_tot = pos.x.size();
	res.costs.resize(npos_tot, vector<float>(npos_tot,Hungarian::myINF));
	for(int i=0; i<npos_tot; i++){
		for(int j=i+1; j<npos_tot; j++){
			res.costs[i][j] = grads[i][pos.x[j]][pos.y[j]]/speed;
			res.costs[j][i] = res.costs[i][j]; // Hay un error de centímetros, por eso replico
		}
	}

	// Asignación
	if(alg==0) // Nearest Neighbor normal
		res.alloc = NN::recursive(res.costs, nagents);
	else if(alg==1) // Húngaro normal
		res.alloc = Hungarian::recursive(res.costs, nagents);
	else if(alg==2) // Húngaro cumulativo
		res.alloc = Hungarian::recursive_cum(res.costs, nagents);

	res.agent_goals.resize(nagents);
	for(int i=0; i<res.alloc.size(); i++){
		for(int j=1; j<res.alloc[i].size(); j++){
			res.agent_goals[i].x.push_back(pos.x[res.alloc[i][j]]);
			res.agent_goals[i].y.push_back(pos.y[res.alloc[i][j]]);
		}
	}

	// Esto es para completar la matriz
	// únicamente para mostrar por pantalla
	int max_all = 0;
	for(int i=0; i<res.alloc.size(); i++)
		if(max_all<res.alloc[i].size()) max_all=res.alloc[i].size();
	for(int i=0; i<res.alloc.size(); i++)
		while(max_all>res.alloc[i].size()) res.alloc[i].push_back(0);

	// Formar los caminos
	res.paths.resize(nagents);
	for(int i=0; i<res.alloc.size(); i++){
		res.paths[i].resize(res.alloc[i].size());
		//for(int j=1; j<res.alloc[i].size(); j++){
		for(int j=1; j<res.alloc[i].size(); j++){
			if(res.alloc[i][j]){
				res.paths[i][j].gradient_descent_(grads[res.alloc[i][j-1]], pos.x[res.alloc[i][j]], pos.y[res.alloc[i][j]]);
				res.paths[i][j].compute_time(speed);
				res.paths[i][j].insert_wait(work_time);
			}
		}
	}

	res.ag_paths.resize(nagents);
	for(int i=0; i<nagents; i++){
		for(int j=0; j<res.paths[i].size(); j++){
			if(res.paths[i][j].tam){
				if(!res.ag_paths[i].tam){
					res.ag_paths[i].initialize(res.paths[i][j].x[0],res.paths[i][j].y[0]);
				}
				//res.ag_paths[i].push_front_(res.paths[i][j]);
				res.ag_paths[i].append_(res.paths[i][j]);
			}
		}
	}

	return res;
} // Asiganción iterativa considerando los tiempos de viaje y tiempo de trabajo igual para todos los goals

alloc_variables iterative_allocation(Poss<int> pos, vector<vector<vector<float>>> grads, int nagents, float speed, vector<float> work_times, uint alg)
{
	alloc_variables res;

	// Formar la matriz de costes
	int npos_tot = pos.x.size();
	res.costs.resize(npos_tot, vector<float>(npos_tot,Hungarian::myINF));
	for(int i=0; i<npos_tot; i++){
		for(int j=i+1; j<npos_tot; j++){
			res.costs[i][j] = work_times[j]+grads[i][pos.x[j]][pos.y[j]]/speed;
			res.costs[j][i] = res.costs[i][j]; // Hay un error de centímetros, por eso replico
		}
	}

	// Asignación
	if(alg==0) // Nearest Neighbor normal
		res.alloc = NN::recursive(res.costs, nagents);
	else if(alg==1) // Húngaro normal
		res.alloc = Hungarian::recursive(res.costs, nagents);
	else if(alg==2) // Húngaro cumulativo
		res.alloc = Hungarian::recursive_cum(res.costs, nagents);

	res.agent_goals.resize(nagents);
	for(int i=0; i<res.alloc.size(); i++){
		for(int j=1; j<res.alloc[i].size(); j++){
			res.agent_goals[i].x.push_back(pos.x[res.alloc[i][j]]);
			res.agent_goals[i].y.push_back(pos.y[res.alloc[i][j]]);
		}
	}

	// Esto es para completar la matriz
	// únicamente para mostrar por pantalla
	int max_all = 0;
	for(int i=0; i<res.alloc.size(); i++)
		if(max_all<res.alloc[i].size()) max_all=res.alloc[i].size();
	for(int i=0; i<res.alloc.size(); i++)
		while(max_all>res.alloc[i].size()) res.alloc[i].push_back(0);

	// Formar los caminos
	res.paths.resize(nagents);
	for(int i=0; i<res.alloc.size(); i++){
		res.paths[i].resize(res.alloc[i].size());
		//for(int j=1; j<res.alloc[i].size(); j++){
		for(int j=1; j<res.alloc[i].size(); j++){
			if(res.alloc[i][j]){
				res.paths[i][j].gradient_descent_(grads[res.alloc[i][j-1]], pos.x[res.alloc[i][j]], pos.y[res.alloc[i][j]]);
				res.paths[i][j].compute_time(speed);
				res.paths[i][j].insert_wait(work_times[res.alloc[i][j]]);
			}
		}
	}

	res.ag_paths.resize(nagents);
	for(int i=0; i<nagents; i++){
		for(int j=0; j<res.paths[i].size(); j++){
			if(res.paths[i][j].tam){
				//res.ag_paths[i].push_front_(res.paths[i][j]);
				if(!res.ag_paths[i].tam){
					res.ag_paths[i].initialize(res.paths[i][j].x[0],res.paths[i][j].y[0]);
				}
				res.ag_paths[i].append_(res.paths[i][j]);
			}
		}
	}

	return res;
} // Asiganción iterativa considerando los tiempos de viaje y tiempos de trabajo distintos para los goals

void save_alloc_vars(alloc_variables alloc, string alg_name, float red_fact, string folder)
{
	// Caminos
	vector<int> paths_numb;
	paths_numb.push_back(alloc.paths.size());
	paths_numb.push_back(alloc.paths[0].size());
	save_vect((folder+"path_numb_"+alg_name+".txt").c_str(),paths_numb);

	for(int i=0; i<alloc.paths.size(); i++){
		for(int j=1; j<alloc.paths[i].size(); j++){
			for(int k=0; k<alloc.paths[i][j].tam; k++){
				alloc.paths[i][j].x[k]/=red_fact;
				alloc.paths[i][j].y[k]/=red_fact;
			}
			// Guardar
			alloc.paths[i][j].save((folder+alg_name+"_alloc_paths_"+to_string(i)+"_"+to_string(j)+".txt").c_str());
		}
	}

	for(int i=0; i<alloc.ag_paths.size(); i++){
		for(int k=0; k<alloc.ag_paths[i].tam; k++){
			alloc.ag_paths[i].x[k]/=red_fact;
			alloc.ag_paths[i].y[k]/=red_fact;
		}
		// Guardar
		alloc.ag_paths[i].save((folder+alg_name+"_alloc_ag_paths_"+to_string(i)+".txt").c_str());
	}

	// Goals por agente
	for(int i=0; i<alloc.agent_goals.size(); i++){
		for(int j=1; j<alloc.agent_goals[i].x.size(); j++){
			alloc.agent_goals[i].x[j]/=red_fact;
			alloc.agent_goals[i].y[j]/=red_fact;
		}
		alloc.agent_goals[i].save((folder+alg_name+"_alloc_agent_goals"+to_string(i)+".txt").c_str());
	}
}

#endif