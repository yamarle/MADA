#ifndef HUNGARIAN_ALL_HPP
#define HUNGARIAN_ALL_HPP

#include "funciones.hpp"
#include "basic_hungarian.hpp"
#include "fmm_2.hpp"
#include "path.hpp"

namespace Hungarian{

	const float myINF = (float)RAND_MAX;

	vector<int> simple(Poss<int> agents, Poss<int> goals, vector<vector<float>> grid); // Húngaro normal (se calculan dentro los gradientes de los agentes)
	vector<int> simple(vector<vector<vector<float>>> grad, Poss<int> goals); // Húngaro normal (se pasan los gradientes de los agentes)
	vector<int> simple_cumulative(vector<vector<vector<float>>> grad, Poss<int> goals, vector<float> ecosts); // Húngaro normal con costes acumulados de los agentes
	vector<int> simple(Poss<int> agents, Poss<int> goals, vector<vector<float>> grid, vector<Path> &paths); // Húngaro normal,se calcualan directamente los caminos (se calculan dentro los gradientes de los agentes)
	vector<int> simple(vector<vector<vector<float>>> grad, Poss<int> goals, vector<Path> &paths); // Húngaro normal,se calcualan directamente los caminos (se pasan los gradientes de los agentes)
	vector<int> priority(Poss<int> agents, Poss<int> goals, vector<bool> priority, vector<vector<float>> grid, vector<Path> &paths); // Húngaro con prioridad para algunos goals (se calculan dentro los gradientes de los agentes)
	vector<int> priority(vector<vector<vector<float>>> grad, Poss<int> goals, vector<bool> priority); // Húngaro con prioridad para algunos goals (se pasan los gradientes de los agentes)
	vector<int> priority(vector<vector<vector<float>>> grad, Poss<int> goals, vector<bool> priority, vector<Path> &paths); // Húngaro con prioridad para algunos goals (se pasan los gradientes de los agentes)

	// ---------------------------------------------------------------------------------------------------------
	// 												Húngaro recursivo
	// ---------------------------------------------------------------------------------------------------------
	vector<vector<float>> form_cost_matrix(vector<vector<float>> costs, vector<int> agents, vector<bool> active, vector<int> &ref); // Devuelve la matriz de costes preparada para el húngaro recursivo
	vector<vector<float>> form_cost_matrix(vector<vector<float>> costs, vector<float> cum_costs, vector<int> agents, vector<bool> active, vector<int> &ref); // Devuelve la matriz de costes preparada para el húngaro recursivo
	vector<vector<int>> recursive(vector<vector<float>> costs, int n); // Realiza una asignación utilizando el Húngaro de manera iterativa
	vector<vector<int>> recursive_cum(vector<vector<float>> costs, int n); // Realiza una asignación utilizando el Húngaro de manera iterativa (costes cumulativos de los agentes)

	vector<vector<int>> swap_2opt(vector<vector<int>> initial_alloc, vector<vector<float>> costs, int n); // Comprobar si existe un movimiento mejor, y cambiar si es así

};

#endif