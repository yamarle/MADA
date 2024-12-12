#ifndef IT_GRAD_SEGM_HPP
#define IT_GRAD_SEGM_HPP

#include "grad_segm_def.hpp"
#include "fmm_2.hpp"
#include "path.hpp"

// Clase que parte el entorno de manera iterativa, moviendo los centroides

class it_grad_segm{
private:
	int max_it, curr_it;
	int nseg;
	vector<vector<float>> grid, grad;
	vector<vector<int>> segments;
	Poss<int> pos, far;
	vector<int> areas;
	vector<float> radius;

	vector<vector<int>> xr_buff, yr_buff;
	int buff_ind;

	Path aux_path;

public:
	it_grad_segm(vector<vector<float>> grid, int nseg, int it);
	it_grad_segm(vector<vector<float>> grid, int nseg, int it, vector<vector<float>> obst_grad);
	void centroid_initialization(vector<vector<float>> obst_grad);
	void iterate();
	void iterate(string folder);

	void move();

	bool check_buffer();

	vector<vector<int>> get_segs();
	vector<vector<float>> get_grad();
	vector<int> get_areas();
	Poss<int> get_pos();
	Poss<int> get_far();
	vector<float> get_radius();
};

#endif