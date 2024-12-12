
// FUNCIONES PARA MOVIMIENTO ALEATORIO UTILIZANDO FMM

#ifndef DYNAMIC_HPP
#define DYNAMIC_HPP

#include <random>

#include "fmm_2.hpp"
#include "path.hpp"
#include "geometry.hpp"

namespace dynamic_fmm{

	// Movimientos
	static vector<int> move_x={0, 0, 1, 1, 1, 0,-1,-1,-1};
    static vector<int> move_y={0, 1, 1, 0,-1,-1,-1, 0, 1};

    // Funciones para el movimiento
	void random_movement(Poss<int> &pos, vector<vector<float>> &grid); // Generación de movimiento aleatorio
	void random_movement(Poss<int> &pos, vector<vector<float>> &grid, Poss<int> others); // Generación de movimiento aleatorio, hay otros agentes que no se conotrolan
	// Generación de movimiento aleatorio para conjuntos de robots
	void swarm_random_movement(Poss<int> sources, vector<Poss<int>> &robots, vector<vector<float>> &grid, Poss<int> others); // Grupos separados en variables distintas
	void swarm_random_movement(Poss<int> sources, Poss<int> &robots, vector<vector<float>> &grid, Poss<int> others); // Grupos juntos en una única variable

	// Generación de un movimiento aleatorio más duradero (A partir de unos gradientes)
	void generate_grads2move(Poss<int> &pos, vector<bool> &generate, vector<vector<float>> &grid, vector<vector<vector<float>>> &grads, vector<vector<float>> ref_grad, float deviation); // generación del gradiente
	void random_movement(Poss<int> &pos, vector<vector<vector<float>>> grads, vector<vector<float>> &grid, vector<bool> &generate, Poss<int> others);

	// Generación de un movimiento aleatorio más duradero (En una dirección determinada)
	void random_movement(Poss<int> &pos, vector<vector<float>> &grid, vector<int> &direction, vector<int> &time, Poss<int> others, int time_limit);
	void random_movement(Poss<int> &pos, vector<vector<float>> &grid, vector<int> &direction, vector<int> &time, Poss<int> others, int time_limit, int seed);

	void include_obstacles(vector<vector<float>> &grid, Poss<int> robots, int xrobot, int yrobot, float vrange, int srange); // Incluir los obstáculos con rango de visión
	void move(int &xrobot, int &yrobot, int xgoal, int ygoal, vector<vector<float>> grid); // Mover el robot

	void iterate(int xrobot, int yrobot, int xgoal, int ygoal, Poss<int> robots, vector<vector<float>> grid, float vrange, int srange, string folder);

	struct navigable_area{
		int shape; // Forma de las áreas
		vector<Poss<float>> contour_poss;
		vector<Poss<int>> poss;
		vector<vector<float>> grid;
	};
	
	navigable_area generate_navigable_area(vector<vector<float>> grid, int nspaces, vector<int> shape);
	navigable_area generate_navigable_area(vector<vector<float>> grid, int nspaces, vector<int> shape, int points_in_area, int max_iter);

};

#endif