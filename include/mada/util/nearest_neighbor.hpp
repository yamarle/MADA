#ifndef NEAREST_NEIGHBOR_HPP
#define NEAREST_NEIGHBOR_HPP

#include "funciones.hpp"
#include "hungarian.hpp"
#include "fmm_2.hpp"
#include "path.hpp"

namespace NN{

	const float myINF = (float)RAND_MAX;

	vector<int> allocate(vector<vector<float>> costs); // Asignar a partir de la matriz de costes
	vector<int> simple(Poss<int> agents, Poss<int> goals, vector<vector<float>> grid); // Asignación al vecino más cercano (los gradientes se calculan dentro)
	vector<int> simple(vector<vector<vector<float>>> grad, Poss<int> goals); // Asignación al vecino más cercano (los gradientes se pasan por variable)
	vector<int> simple(Poss<int> agents, Poss<int> goals, vector<vector<float>> grid, vector<Path> &paths); // Asignación al vecino más cercano, se calculan los caminos (los gradientes se calculan dentro)
	vector<int> simple(vector<vector<vector<float>>> grad, Poss<int> goals, vector<Path> &paths); // Asignación al vecino más cercano, se calculan los caminos (los gradientes se pasan por variable)

	// ---------------------------------------------------------------------------------------------------------
	// 												NN recursivo
	// ---------------------------------------------------------------------------------------------------------
	vector<vector<float>> form_cost_matrix(vector<vector<float>> costs, vector<int> agents, vector<bool> active, vector<int> &ref); // Devuelve la matriz de costes preparada para el húngaro recursivo
	vector<vector<int>> recursive(vector<vector<float>> costs, int n); // Realiza una asignación utilizando el NN de manera iterativa

}

#endif