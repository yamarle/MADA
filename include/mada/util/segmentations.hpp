#ifndef SEGMENTATIONS_HPP
#define SEGMENTATIONS_HPP

#include "eq_segm.hpp"
#include "it_grad_segm.hpp"
#include <ctime>

// Se obtienen todas las segmentaciones

namespace segmentation{
	// LLamada a los 3 algoritmos de segmentación que tengo
	// 1: eq_segm s1(x, y, grid, nseg);
	// 2: it_grad_segm s2(grid, nseg, iter, obst_grad);
	// 3: it_grad_segm s3(obst_grad, nseg, iter, obst_grad);
	struct segment{
		vector<vector<int>> segments;
		vector<int> areas;
		Poss<int> centroids;
		float comp_time;
	};

	segment BAP(int x, int y, vector<vector<float>> grid, int nseg)
	{
		// Propagación equitativa
		segment res;
		clock_t t=clock();
		eq_segm s(x, y, grid, nseg);
		s.computation();
		t=clock()-t;
		res.comp_time=(float)t/CLOCKS_PER_SEC;
		res.segments=s.get_segs();
		res.areas=s.get_areas();
		res.centroids=s.get_pos();
		return res;
	}

	segment PAP(vector<vector<float>> grid, int nseg, int iter, vector<vector<float>> obst_grad)
	{
		// Propagación iterativa
		segment res;
		clock_t t=clock();
		it_grad_segm s(grid, nseg, iter, obst_grad);
		s.iterate();
		t=clock()-t;
		res.comp_time=(float)t/CLOCKS_PER_SEC;
		res.segments=s.get_segs();
		res.areas=s.get_areas();
		res.centroids=s.get_pos();
		return res;
	}

	segment RAP(int nseg, int iter, vector<vector<float>> obst_grad)
	{
		// Propagación iterativa sobre gradiente de obstáculos
		segment res;
		clock_t t=clock();
		it_grad_segm s(obst_grad, nseg, iter, obst_grad);
		s.iterate();
		t=clock()-t;
		res.comp_time=(float)t/CLOCKS_PER_SEC;
		res.segments=s.get_segs();
		res.areas=s.get_areas();
		res.centroids=s.get_pos();
		return res;
	}

	segment segmentation(int x, int y, vector<vector<float>> grid, int nseg, int iter, vector<vector<float>> obst_grad, int seg_type)
	{
		segment res;

		if(seg_type == 0) res = BAP(x, y, grid, nseg);
		else if(seg_type == 1) res = PAP(grid, nseg, iter, obst_grad);
		else if(seg_type == 2) res = RAP(nseg, iter, obst_grad);

		return res;
	}

	vector<segment> all_segmentations(int x, int y, vector<vector<float>> grid, int nseg, int iter, vector<vector<float>> obst_grad)
	{
		vector<segment> res(3);
		res[0]=BAP(x, y, grid, nseg);
		res[1]=PAP(grid, nseg, iter, obst_grad);
		res[2]=RAP(nseg, iter, obst_grad);
		return res;
	}

	void save(vector<segment> res, string folder)
	{
		save_matr((folder+"segments1.txt").c_str(),res[0].segments);
		save_matr((folder+"segments2.txt").c_str(),res[1].segments);
		save_matr((folder+"segments3.txt").c_str(),res[2].segments);
		res[0].centroids.save((folder+"centroids1.txt").c_str());
		res[1].centroids.save((folder+"centroids2.txt").c_str());
		res[2].centroids.save((folder+"centroids3.txt").c_str());
		vector<float> times={res[0].comp_time, res[1].comp_time, res[2].comp_time};
		save_vect((folder+"comp_times.txt").c_str(),times);
		save_vect((folder+"areas1.txt").c_str(),res[0].areas);
		save_vect((folder+"areas2.txt").c_str(),res[1].areas);
		save_vect((folder+"areas3.txt").c_str(),res[2].areas);
	}

	void evaluate_areas(vector<segment> res){
		int nseg=res[1].areas.size();
		int opt_area = mean_v(res[0].areas);
		vector<int> mean_d(nseg,0), max_d(nseg,0);
		int v=0;
		cout<<"Área óptima: "<<opt_area<<endl;
		for(int i=0; i<res.size(); i++){
			for(int j=0; j<nseg; j++){
				v=abs(res[i].areas[j]-opt_area);
				mean_d[i]+=v;
				if(v>max_d[i]) max_d[i]=v;
			}
			mean_d[i]/=nseg;
			cout<<"Desviación de "<<i+1<<" media: "<<mean_d[i]<<", max: "<<max_d[i]<<endl;
			//cout<<"Desviación máxima de "<<i+1<<": "<<max_d[i]<<endl;
		}
	}

	vector<vector<int>> evaluate_goals(vector<segment> res, Poss<int> goals){
		int nseg = res[1].areas.size();
		vector<vector<int>> nps(3,vector<int>(nseg,0));
		for(int i=0; i<goals.x.size(); i++){
			for(int j=0; j<3; j++){
				nps[j][res[j].segments[goals.x[i]][goals.y[i]]-1]++;
			}
		}
		return nps;
	}

};

#endif