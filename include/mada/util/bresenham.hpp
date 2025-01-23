#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include "funciones.hpp"

namespace bresenham{

	int number_of_walls(vector<vector<float> > grid, int x1, int y1, int x2, int y2); // Número de muros atravesados
	int ray_of_walls(vector<vector<float> > grid, int x1, int y1, int x2, int y2, vector<int> &x, vector<int> &y, vector<int> &n); // Número de muros atravesados (Posiciones de las líneas lanzadas y cantidad de muros atravesados)
	bool exists_wall(vector<vector<float> > grid, int x1, int y1, int x2, int y2, int &x, int &y); // Chequear si se atraviesa un muro (devuleve la posición del muro)
	bool exists_wall(vector<vector<float> > grid, int x1, int y1, int x2, int y2); // Chequear si se atraviesa un muro

	Poss<int> points(int x1, int y1, int x2, int y2); // Todas las posiciones
	Poss<int> points_wall_check(int x1, int y1, int x2, int y2, vector<vector<float> > grid); // Todas las posiciones (hasta chocar con un obstáculo)

	Poss<int> all_rays_poss(int x, int y, Poss<int> poss);
	Poss<int> all_rays_poss(int x, int y, Poss<int> poss, vector<vector<int>> &seen_map);

}

#endif