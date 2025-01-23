#ifndef GRAD_SEGM_DEF_HPP
#define GRAD_SEGM_DEF_HPP

#include "funciones.hpp"

// Clase que para unas posiciones estáticas parte el entorno

class grad_segm{
private:
	int sx, sy;
	// Hay que utilizar los vecinos de la diagonal
	Poss<int> centr;
	vector<int> nx={0, 1, 1, 1, 0,-1,-1,-1};
	vector<int> ny={1, 1, 0,-1,-1,-1, 0, 1};
	int nseg;
	vector<vector<float>> grad;
	vector<vector<int>> segments;
	vector<int> qx,qy;
	vector<float> qd;
	int x,y;
	int x_,y_;
	int xmin,ymin;
	float min;
	int where;
	vector<int> areas;
	Poss<int> far_pos;
	vector<float> max;
	int seg_ind;
	Poss<int> contour;
public:
	grad_segm(Poss<int> p, vector<vector<float>> grad);
	void computation();
	void propagate();
	void to_queue();
	void find_min();

	vector<vector<int>> get_segs();
	vector<int> get_areas();
	Poss<int> get_far_pos();
	vector<float> get_far_val();
	Poss<int> get_contour();
};

#endif
