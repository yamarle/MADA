#ifndef EQ_SEGM_HPP
#define EQ_SEGM_HPP

#include "fmm_2.hpp"
//#include "FMMsegm.hpp"

class eq_segm{
private:
	int sx, sy;
	// Puede que me sirvan para la adyacencia
	vector<int> nx={0, 1, 1, 1, 0,-1,-1,-1};
	vector<int> ny={1, 1, 0,-1,-1,-1, 0, 1};
	int nseg, nseg_;
	vector<vector<float>> grid, aux_grid;
	vector<vector<float>> grad, ref_grad;
	vector<vector<int>> segments;
	vector<int> areas, perimeters;
	vector<int> adjacency;
	Poss<int> pos;
	int x,y,x_,y_;
	float min, max, value;
	int aux;
	int minv, ind;

	vector<int> qx, qy; // los puntos del segmento nuevo

	int tot_area, opt_seg_area;
	int free_space;

	vector<vector<int>> adj_matr;
	void adjacency_();

public:
	eq_segm(int x, int y, vector<vector<float>> grid, int nseg);
	void find_nearest();
	void find_farthest();
	void classify();
	void classify_(int s1, int s2, int new_area_seg);
	void finish();
	void separate(int s1, int s2);
	void computation();
	void computation(string folder);

	vector<vector<int>> get_segs();
	vector<int> get_areas();
	vector<vector<float>> get_grad();
	Poss<int> get_pos();
};

#endif