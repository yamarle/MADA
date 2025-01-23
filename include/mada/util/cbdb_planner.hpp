// CBDB: Crowd-based Dynamic Blockages
// Paper: "Mapping Crowd-based Dynamic Blockages for Navigation in Indoor Environments",
// Authors: Alleff Deus, Guilherme Daudt, Renan Maffei, Mariana Kolberg

#ifndef CBDB_PLANNNER_HPP
#define CBDB_PLANNNER_HPP

#include <mada/mada_util.h>

#include <mada/util/funciones.hpp>
#include <mada/util/generic_fmm.hpp>
#include <mada/util/fmm_2.hpp>
#include <mada/util/path.hpp>
#include <mada/util/cluster.hpp>

namespace CBDB{

	vector<int> numberOfObstaclesPerRegion(Poss<int> obst_poss, float r, vector<vector<int>> region_map, int Nregions)
	{
		vector<int> res(Nregions, 0);
		// Clasificar los puntos por regiones
		vector<Poss<int>> obst_per_reg(Nregions);
		for(int i=0; i<obst_poss.x.size(); i++){
			if(region_map[obst_poss.x[i]][obst_poss.y[i]] >= 0){ // NO estoy en un obstáculo o inflado
				obst_per_reg[region_map[obst_poss.x[i]][obst_poss.y[i]]].push(obst_poss.x[i], obst_poss.y[i]);
			}
		}

		// Obtener cantidad de obstáculos por región
		for(int i=0; i<Nregions; i++){
			if(obst_per_reg[i].x.size()){
				res[i] = adjacency_clustering::cluster(obst_per_reg[i], r).size();
			}
		}

		return res;
	}

	void getRegionDimensions(Poss<int> poss, float &sx, float &sy)
	{
		if(poss.x.size() == 0) return;
		Poss<int> bb = geometry::bounding_box(poss);
		if(bb.x.size() == 4){
			sx = abs(bb.x[0] - bb.x[1]);
			sy = abs(bb.y[0] - bb.y[3]);
		}
	} // Calcular las dimensiones de una región en base a su bounding box

	vector<vector<float>> computePartitionDimensions(vector<Poss<int>> partition_poss)
	{
		vector<vector<float>> res(partition_poss.size(), vector<float>(2, 0));
		for(int i=0; i<partition_poss.size(); i++){
			getRegionDimensions(partition_poss[i], res[i][0], res[i][1]);
		}
		return res;
	} // Calcular las dimensiones de todas las particiones

	vector<float> computeRegionWeights(vector<int> nobst, vector<vector<float>> dimensions, float alpha)
	{
		vector<float> res(nobst.size(), 0);
		for(int i=0; i<nobst.size(); i++){
			res[i] = nobst[i] / (alpha * dimensions[i][0] * dimensions[i][1]);
		}
		return res;
	}

	void computePartitionDimensions(vector<Poss<int>> partition_poss, vector<float> &sx, vector<float> &sy)
	{
		if(partition_poss.size() != sx.size()){
			sx.resize(partition_poss.size()); sy.resize(partition_poss.size());
		}
		for(int i=0; i<partition_poss.size(); i++){
			getRegionDimensions(partition_poss[i], sx[i], sy[i]);
		}
	} // Calcular las dimensiones de todas las particiones

	vector<float> computeRegionWeights(vector<int> nobst, vector<float> sx, vector<float> sy, float alpha)
	{
		vector<float> res(nobst.size(), 0);
		for(int i=0; i<nobst.size(); i++){
			res[i] = nobst[i] / (alpha * sx[i] * sy[i]);
		}
		return res;
	} // ***** eq.(1) *****

	Path compute_path(int xs, int ys, int xg, int yg, vector<vector<float>> grid, vector<vector<int>> segs_map, vector<Poss<int>> segs_poss, vector<float> region_weights, vector<float> w_r, float resolution, vector<Path> &evaluated_paths, vector<float> &costs, vector<bool> &blk_regions)
	{
		Path res;
		Path path, path_best;
		float cost, cost_blk, cost_best;
		cost_best = INFINITY;
		float w_blk;

		while(res.tam == 0){ // Hasta que no se encuentre un camino que vaya por zonas libres de obstáculos
			// Calculo el camino original
			path = navigation::path2goal(xs, ys, grid, xg, yg);

			evaluated_paths.push_back(path);

			if(path.tam == 0){ // No se ha encotrado camino
				costs.push_back(INFINITY);
				return path_best; // devolver el mejor camino que hay
			}

			// Obtener el coste original (distancia acumulada del camino)
			cost = 0;
			for(int i=1; i<path.tam; i++){
				cost += hypot(path.x[i-1] - path.x[i], path.y[i-1] - path.y[i]);
				cost *= resolution;
			}

			// Almacenar las blocking_regions por las que pasa el camino
			vector<int> R_blk; // Las blocking_regions por las cuales pasa el camino
			for(int i=0; i<path.tam; i++){
				//if(segs_map[path.x[i]][path.y[i]] >= 0 && region_weights[segs_map[path.x[i]][path.y[i]]] > 0){ // El camino pasa por región con obstáculos
				// El punto del camino pasa por regiones fuera de obstáculos, no es de la región de inicio (donde está el robot), ni de la región del fin (donde está el goal)
				if(segs_map[path.x[i]][path.y[i]] >= 0 && segs_map[path.x[i]][path.y[i]] != segs_map[xs][ys] && segs_map[path.x[i]][path.y[i]] != segs_map[xg][yg])
				if(region_weights[segs_map[path.x[i]][path.y[i]]] > 0){ // El camino pasa por región con obstáculos
					if(!R_blk.size()){
						R_blk.push_back(segs_map[path.x[i]][path.y[i]]);
					}else{
						if(R_blk[R_blk.size()-1] != segs_map[path.x[i]][path.y[i]]){
							R_blk.push_back(segs_map[path.x[i]][path.y[i]]);
						}
					}
				}
			}

			// Calcular el coste del camino con bloqueo
			if(R_blk.size()){ // El camino atraviesa regiones con obstáculos
				// Actualizar el coste del camino
				cost_blk = cost;
				for(int i=0; i<R_blk.size(); i++){
					w_blk = w_r[R_blk[i]] / (1 - region_weights[R_blk[i]]); // ***** eq.(2) *****
					cost_blk += (w_blk - w_r[R_blk[i]]); // ***** eq.(3) *****

					blk_regions[R_blk[i]] = true;
				}

				costs.push_back(cost_blk);

				// ***** eq.(4) *****
				if(cost_best > cost_blk){ // El camino nuevo es mejor
					// Quedarme con el camino, PERO CONTINÚA LA BÚSQUEDA DE UN CAMINO SIN BLOQUEO
					cost_best = cost_blk;
					path_best = path;
				}

				// "meto un obstáculo virtual" en las blocking_regions
				for(int i=0; i<R_blk.size(); i++){
					for(int j=0; j<segs_poss[R_blk[i]].x.size(); j++){
						grid[segs_poss[R_blk[i]].x[j]][segs_poss[R_blk[i]].y[j]] = 0;
					}
				}

			}else{ // El camino original no pasa por ninguna región con obstáculos -> TERMINA LA BÚSQUEDA
				// ***** eq.(4) *****
				if(cost_best < cost){ // El camino con bloqueo es mejor
					res = path_best; // Me quedo con él
					costs.push_back(cost_best);
				}else{ // El camino nuevo es mejor
					res = path; // ME quedo con el camino libre
					costs.push_back(cost);
				}
			}

			//cin.get();

		}

		return res;
	}

	void show_evaluation(vector<Path> paths, vector<float> costs)
	{
		cout<<paths.size()<<" caminos evaluados:"<<endl;
		for(int i=0; i<paths.size(); i++)
			cout<<i<<" : "<<paths[i].tam<<" - "<<costs[i]<<endl; 
	}

};

#endif