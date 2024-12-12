#ifndef ENVIRONMENT_GRAPH_HPP
#define ENVIRONMENT_GRAPH_HPP

//#include "segmentations.hpp"
#include "segmentation_functions.hpp"
#include "bresenham.hpp"

namespace env_graph{

	struct Graph
	{
		int size = 0;
		Poss<int> pos; // Posiciones de los vértices
		vector<float> val; // Valores de distancias a los obstáculos en los vértices
		vector<vector<bool>> edges; // Aristas

		vector<Path> paths; // Caminos reales entre cada par de nodos adyacentes
		vector<vector<Path>> sort_paths; // Los mismos caminos adyacentes, pero organizados como la matriz

		vector<vector<int>> segments; // Segmentos con los que se ha obtenido el grafo
		vector<vector<float>> obst_grad; // Gradiente con el que se han realizado los cálculos
		vector<int> areas; // Áreas de los segmentos

		vector<Poss<int>> lines; // Puntos (por pares) de los vértices, solo para representar

		vector<vector<bool>> ref_edges; // Aristas refinadas
		vector<Poss<int>> ref_lines; // Puntos (por pares) de los vértices, solo para representar
	};

	vector<vector<Path>> sort_paths(vector<vector<bool>> grafo, vector<Path> paths)
	{
		vector<vector<Path>> res(grafo.size(),vector<Path>(grafo.size()));
		int cont = 0;
		for(int i=0; i<grafo.size(); i++)
			for(int j=i+1; j<grafo.size(); j++)
				if(grafo[i][j]){
					res[i][j] = paths[cont];
					res[j][i] = paths[cont];
					cont++;
				}
		return res;
	}

	Graph compute(Poss<int> obst, vector<vector<float>> grid, float dthres){

		Graph res;	

		int sx = grid.size(), sy = grid[0].size();

		// 1. Obtener el gradiente desde los obstáculos
	    FMM ogr(obst.x,obst.y,grid);
	    vector<vector<float>> ograd = ogr.compute_gradient_();
	    res.obst_grad = ograd;

	    // 2. Sacar los máximos del gradiente
	    float max;
	    int x_, y_;
	    while(1){
	        max = 0;
	        // Encontrar el máximo
	        for(int i=0; i<sx; i++)
	            for(int j=0; j<sy; j++)
	                if(max<ograd[i][j]){
	                    max = ograd[i][j];
	                    x_ = i; y_ = j; 
	                }

	        if(max<=dthres) break; // No queda más espacio libre

	        // Anular las posiciones dentro del radio de max
	        for(int i=x_-max-1; i<x_+max+1; i++)
	            for(int j=y_-max-1; j<y_+max+1; j++)
	                if(i>=0 && i<sx && j>=0 && j<sy){
	                    ograd[i][j] = 0;
	                }

	        // Almacenar
	        res.pos.x.push_back(x_); res.pos.y.push_back(y_);
	        res.val.push_back(max);
	        res.size++;

	    }

	    // 3. Sacar el grafo de las posiciones
	    // Uso la segmentación con gradientes (sin iterar)
	    //FMM sgr(res.pos.x,res.pos.y,grid);
	    FMM sgr(res.pos.x,res.pos.y,res.obst_grad);
	    vector<vector<float>> sgrad = sgr.compute_gradient_();
	    grad_segm gs(res.pos,sgrad);
	    gs.computation();

	    // Variables necesarias
	    res.segments=gs.get_segs(); // Los segmentos
	    res.areas=gs.get_areas();
	    // El grafo
	    res.edges = compute_adjacency(res.segments, res.size);
	    res.lines = adjacent_lines(res.pos, res.edges, grid);

	    return res;
	}

	vector<vector<bool>> refine(Poss<int> pos, vector<vector<bool>> edges, vector<vector<float>> grid, vector<vector<float>> obst_grad)
	{
		vector<vector<bool>> res = edges;
		Poss<int> line_points; // Donde almacenar todos los putos de bresenham
		vector<float> values; // Distancias a los obstáculos
		bool m; // pendiente
		float min, max;
		float val;
		for(int i=0; i<pos.x.size(); i++){
			for(int j=i+1; j<pos.x.size(); j++){
				if(res[i][j]){ // Hay conexión
					// Comprobar 
					line_points = bresenham::points(pos.x[i], pos.y[i], pos.x[j], pos.y[j]);
					// Sacar los valores del valle
					min = INF; max = 0;
					for(int k=0; k<line_points.x.size()-1; k++){
						if(min>obst_grad[line_points.x[k]][line_points.y[k]]) min = obst_grad[line_points.x[k]][line_points.y[k]];
						if(max<obst_grad[line_points.x[k]][line_points.y[k]]) max = obst_grad[line_points.x[k]][line_points.y[k]];
						//values.push_back(obst_grad[line_points.x[k]][line_points.y[k]]);
						if(!min){ // UN MURO, NO HACE FALTA SEGUIR
							res[i][j] = false;
							res[j][i] = false;
							break;
						}
					}

					/*
					// Con diferencia de valores
					if(obst_grad[pos.x[i]][pos.y[i]] != obst_grad[pos.x[j]][pos.y[j]] && (abs(obst_grad[pos.x[i]][pos.y[i]] - min)>1 && abs(obst_grad[pos.x[j]][pos.y[j]] - min)>1)){
						res[i][j] = false;
						res[j][i] = false;
					}
					*/
					
					// Una interpolación muy xusca
					//if(abs(obst_grad[pos.x[i]][pos.y[i]] - obst_grad[pos.x[j]][pos.y[j]])>1){
						val = obst_grad[line_points.x[line_points.x.size()/2]][line_points.y[line_points.x.size()/2]];
						if(obst_grad[pos.x[i]][pos.y[i]] < obst_grad[pos.x[j]][pos.y[j]]){
							if(obst_grad[pos.x[i]][pos.y[i]]>val ||  val>obst_grad[pos.x[j]][pos.y[j]]){
								res[i][j] = false;
								res[j][i] = false;
							}
						}else{
							if(obst_grad[pos.x[i]][pos.y[i]]<val ||  val<obst_grad[pos.x[j]][pos.y[j]]){
								res[i][j] = false;
								res[j][i] = false;
							}
						}
					//}
					
					/*
					// Interpolación más realista
					// CREO QUE NO ES UTILIZABLE, SALVO EN CASO DE USAR UN "CAMINO VORONOI"
					// y = y0 + (x-x0)*(y1-y0)/(x1-x0)
					// d = OD(0) + (x - 0)*(OD(1))-OD(0))/(d12-0)
					val = sqrt(pow(pos.x[i]-pos.x[j],2)+pow(pos.y[i]-pos.y[j],2));
					val = round(obst_grad[pos.x[i]][pos.y[i]] + (val/2) * (obst_grad[pos.x[j]][pos.y[j]]-obst_grad[pos.x[i]][pos.y[i]])/val);
					if(val != obst_grad[line_points.x[line_points.x.size()/2]][line_points.y[line_points.x.size()/2]]){
						res[i][j] = false;
						res[j][i] = false;
					}
					*/

					/*
					// Con diferencia de valores, pero mirando el porcentaje de diferencia
					if(obst_grad[pos.x[i]][pos.y[i]] != obst_grad[pos.x[j]][pos.y[j]]){
						if(abs(obst_grad[pos.x[i]][pos.y[i]] - min)>1 && abs(obst_grad[pos.x[j]][pos.y[j]] - min)>1){
							res[i][j] = false;
							res[j][i] = false;
						}else{
							if(obst_grad[pos.x[i]][pos.y[i]]>obst_grad[pos.x[j]][pos.y[j]]){
								if(obst_grad[pos.x[j]][pos.y[j]]/obst_grad[pos.x[i]][pos.y[i]]>.1){
									res[i][j] = false;
									res[j][i] = false;		
								}
							}else{
								if(obst_grad[pos.x[i]][pos.y[i]]/obst_grad[pos.x[j]][pos.y[j]]>.1){
									res[i][j] = false;
									res[j][i] = false;		
								}
							}
						}
					}
					*/

					//cin.get();
				}
			}
		}
		return res;
	} // Quitar aristas del grafo

	vector<vector<bool>> refine_paths(Poss<int> pos, vector<vector<bool>> edges, vector<vector<Path>> paths, vector<vector<float>> obst_grad)
	{
		vector<vector<bool>> res = edges;

		float min, max;
		int ind;

		float diff, max_diff;

		int n = pos.x.size();
		vector<bool> done(n,false);

		bool separados = false;

		for(int i=0; i<n; i++){
			// Encontrar el máximo
			// Empiezo por ese, descartando conexiones con pasillos, uniendo residuos de espacios rectangulares...
			max = 0;
			for(int j=0; j<n; j++){
				if(!done[j] && max<obst_grad[pos.x[j]][pos.y[j]]){
					max = obst_grad[pos.x[j]][pos.y[j]];
					ind = j;
				}
			}

			// Conexiones del nodo
			if(max){
				done[ind] = true;
				for(int j=0; j<n; j++){
					if(edges[ind][j]){ // Hay una conexión
						min = INF;
						separados = false;
						max_diff = 0;
						cout<<"D: ";
						for(int k=0; k<paths[ind][j].tam; k++){
							cout<<obst_grad[paths[ind][j].x[k]][paths[ind][j].y[k]]<<" ";
							if(min > obst_grad[paths[ind][j].x[k]][paths[ind][j].y[k]]){
								min = obst_grad[paths[ind][j].x[k]][paths[ind][j].y[k]];
							}else{
								//separados = true;
								diff = abs(obst_grad[paths[ind][j].x[k]][paths[ind][j].y[k]] - min);
								if(max_diff < diff) max_diff = diff;
							}
						}
						cout<<endl;

						cout<<max_diff<<" > "<<paths[ind][j].d<<endl;
						if(max_diff)
							if(max_diff > paths[ind][j].d) separados = true;
						else separados = true;
						
						if(separados){
							cout<<"Areas distintas"<<endl;
							res[ind][j] = 0;
							res[j][ind] = 0;
						}else{
							cout<<"Mismo espacio"<<endl;
							
							//cin.get();
						}
						
					}
				}
			}

		}
		return res;
	} // Devuelve la matriz separada por los distintos segmentos (la existencia de aristas representa el mismo espacio)

	vector<vector<bool>> refine_connectivity(Poss<int> pos, vector<vector<bool>> edges, vector<vector<Path>> paths, vector<vector<float>> obst_grad)
	{
		int n = pos.x.size();
		vector<vector<bool>> res(n, vector<bool>(n,false));

		float max;
		int ind;

		for(int i=0; i<n; i++){
			// Encontrar el máximo
			max = 0;
			for(int j=0; j<n; j++){
				if(max<obst_grad[pos.x[j]][pos.y[j]]){
					max = obst_grad[pos.x[j]][pos.y[j]];
					ind = j;
				}
			}

			// Mirar con quién está conectado el nodo
			if(max){
				for(int j=0; j<n; j++){
					if(edges[ind][j]){
						cout<<"D: ";
						for(int k=0; k<paths[ind][j].tam; k++){
							cout<<obst_grad[paths[ind][j].x[k]][paths[ind][j].y[k]]/max<<" ";
						}
						cout<<endl;
						cin.get();
					}
				}
			}

		}

		return res;
	}

	Graph compute_wr(Poss<int> obst, vector<vector<float>> grid, float dthres)
	{
		Graph res;	

		int sx = grid.size(), sy = grid[0].size();

		// 1. Obtener el gradiente desde los obstáculos
	    FMM ogr(obst.x,obst.y,grid);
	    vector<vector<float>> ograd = ogr.compute_gradient_();
		res.obst_grad = ograd;

	    // 2. Sacar los máximos del gradiente
	    float max;
	    int x_, y_;
	    while(1){
	        max = 0;
	        // Encontrar el máximo
	        for(int i=0; i<sx; i++)
	            for(int j=0; j<sy; j++)
	                if(max<ograd[i][j]){
	                    max = ograd[i][j];
	                    x_ = i; y_ = j; 
	                }

	        if(max<=dthres) break; // No queda más espacio libre

	        // Anular las posiciones dentro del radio de max
	        for(int i=x_-max-1; i<x_+max+1; i++)
	            for(int j=y_-max-1; j<y_+max+1; j++)
	                if(i>=0 && i<sx && j>=0 && j<sy){
	                    ograd[i][j] = 0;
	                }

	        // Almacenar
	        res.pos.x.push_back(x_); res.pos.y.push_back(y_);
	        res.val.push_back(max);
	        res.size++;
	    }

	    // 3. Sacar el grafo de las posiciones
	    // Uso la segmentación con gradientes (sin iterar)
	    //FMM sgr(res.pos.x,res.pos.y,grid);
	    FMM sgr(res.pos.x,res.pos.y,res.obst_grad);
	    vector<vector<float>> sgrad = sgr.compute_gradient_();
	    grad_segm gs(res.pos,sgrad);
	    gs.computation();

	    // Variables necesarias
	    res.segments=gs.get_segs(); // Los segmentos
	    res.areas=gs.get_areas();
	    // El grafo
	    res.edges = compute_adjacency(res.segments, res.size);
	    res.lines = adjacent_lines(res.pos, res.edges, grid);

	    res.paths = adjacent_paths(res.pos, res.edges, res.obst_grad);

	    // REFINAR EL GRAFO
	    res.ref_edges = refine(res.pos, res.edges, grid, res.obst_grad);
	    res.ref_lines = adjacent_lines(res.pos, res.ref_edges, grid);

	    return res;
	}

	void save(Graph graph, string results_folder)
	{
		graph.pos.save((results_folder+"graph_pos.txt").c_str());
	    save_vect((results_folder+"graph_val.txt").c_str(),graph.val);
	    // Segmentos
	    save_matr((results_folder+"graph_segments.txt").c_str(),graph.segments);
	    // Segmentos
	    save_matr((results_folder+"graph_obst.txt").c_str(),graph.obst_grad);
	    // Grafo
	    save_poss(graph.lines,1,(results_folder+"graph_edges_"));
	    // Caminos reales del grafo
	    save_paths(graph.paths,1,(results_folder+"graph_paths_"));

	    // Varibales del grafo refinado
	    save_poss(graph.ref_lines,1,(results_folder+"graph_ref_edges_"));
	}

};

#endif