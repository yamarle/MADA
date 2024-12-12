#ifndef SEGMENTATION_FUNCTIONS_HPP
#define SEGMENTATION_FUNCTIONS_HPP

#include "segmentations.hpp"
#include "allocations.hpp"
#include "useless.hpp"
#include <queue>
#include <limits>

// Éstas 2 funciones son exactamente lo mismo que la estimación de los recorridos 
// dentro de los segmentos
vector<vector<float>> delimit_segment(vector<vector<int>> segments, int seg, Poss<int> &obst)
{
	vector<vector<float>> res(segments.size(),vector<float>(segments[0].size(),0));
	for(int i=0; i<segments.size(); i++){
		for(int j=0; j<segments[0].size(); j++){
			if(segments[i][j]==seg){
				res[i][j]=1;
			}else{
				obst.x.push_back(i); obst.y.push_back(j);
			}
		}
	}
	return res;
} // Anular las posiciones fuera del segmento y almacenar las posiciones de los obstáculos (para inicialización de los centoides)

vector<vector<float>> delimit_grid(vector<vector<int>> segm, int seg, vector<vector<float>> grid)
{
	for(int i=0; i<segm.size(); i++){
		for(int j=0; j<segm[0].size(); j++){
			if(segm[i][j]!=seg){
				grid[i][j] = 0;
			}
		}
	}
	return grid;
}

Poss<int> compute_centroids(vector<vector<int>> segments, int nseg, vector<vector<float>> grid, Poss<int> &far_pos)
{
	Poss<int> centr, obst;
	vector<vector<float>> grad, grid_;
	for(int i=0; i<nseg; i++){
		grid_=delimit_segment(segments,i+1,obst);
		FMM gr(obst.x,obst.y,grid_);
		grad=gr.compute_gradient_();
		it_grad_segm s(grid_,1,100,grad);
		s.iterate();
		centr.append(s.get_pos());
		far_pos.append(s.get_far());
		obst.clear();
	}
	return centr;
} // Calcular los centroides de los segmentos utilizando PAP

Poss<int> compute_centroids(vector<vector<int>> segments, int nseg, vector<vector<float>> grid, Poss<int> &far_pos, vector<float> &segm_radius)
{
	Poss<int> centr, obst;
	vector<vector<float>> grad, grid_;
	vector<float> rad;
	for(int i=0; i<nseg; i++){
		grid_=delimit_segment(segments,i+1,obst);
		FMM gr(obst.x,obst.y,grid_);
		grad=gr.compute_gradient_();
		it_grad_segm s(grid_,1,100,grad);
		s.iterate();
		centr.append(s.get_pos());
		far_pos.append(s.get_far());
		rad = s.get_radius();
		segm_radius.push_back(rad[0]); // Al iterar por segmentos, devuelve un único valor
		obst.clear();
	}
	return centr;
} // Calcular los centroides de los segmentos utilizando PAP (Devuelve el "radio" de cada segmento)

Poss<int> random_pos_in_segs(vector<vector<int>> segments, int nseg)
{
	int sx=segments.size(), sy=segments[0].size();
	Poss<int> res; res(nseg);
	for(int i=0; i<nseg; i++){
		res.x[i] = (int(rand())/INF)*sx-1;
		res.y[i] = (int(rand())/INF)*sy-1;
		while(segments[res.x[i]][res.y[i]]!=i+1){
			res.x[i] = (int(rand())/INF)*sx-1;
			res.y[i] = (int(rand())/INF)*sy-1;
		}
	}
	return res;
}

Poss<int> random_pos_in_segs(vector<vector<int>> segments, int nseg, Poss<int> occupied)
{
	int sx=segments.size(), sy=segments[0].size();
	Poss<int> res; res(nseg);
	for(int i=0; i<occupied.x.size(); i++){
		segments[occupied.x[i]][occupied.y[i]]=0;
	}
	for(int i=0; i<nseg; i++){
		res.x[i] = (int(rand())/INF)*sx-1;
		res.y[i] = (int(rand())/INF)*sy-1;
		while(segments[res.x[i]][res.y[i]]!=i+1){
			res.x[i] = (int(rand())/INF)*sx-1;
			res.y[i] = (int(rand())/INF)*sy-1;
		}
	}
	return res;
}

vector<int> goals_segment_membership(vector<vector<int>> segments, Poss<int> goals)
{
	vector<int> res(goals.x.size());
	for(int i=0; i<goals.x.size(); i++){
		res[i] = segments[goals.x[i]][goals.y[i]];
	}
	return res;
} // A qué segmentos pertenecen los goals

Poss<int> points_in_seg(vector<vector<int>> segments, Poss<int> points, int n)
{
	Poss<int> res;
	for(int i=0; i<points.x.size(); i++){
		if(segments[points.x[i]][points.y[i]]==n){
			res.x.push_back(points.x[i]);
			res.y.push_back(points.y[i]);
		}
	}
	return res;
}

Poss<int> far_in_segs(vector<vector<int>> segments, vector<vector<float>> ref_grad, int n)
{
	Poss<int> res; res(n);
	vector<float> max(n,0);

	for(int i=0; i<segments.size(); i++){
		for(int j=0; j<segments[0].size(); j++){
			if(segments[i][j] && max[segments[i][j]-1]<ref_grad[i][j]) {
				res.x[segments[i][j]-1]=i;
				res.y[segments[i][j]-1]=j;
				max[segments[i][j]-1]=ref_grad[i][j];
			}
		}
	}

	return res;
} // Obtener las posiciones de valor mayor del gradiente ref_grad (más alejados)

Poss<int> far_in_segs(vector<vector<int>> segments, vector<vector<float>> ref_grad, int n, vector<float> &max)
{
	Poss<int> res; res(n);

	if(max.size()){
		max.clear();
	}
	max.resize(n,0);

	for(int i=0; i<segments.size(); i++){
		for(int j=0; j<segments[0].size(); j++){
			if(segments[i][j] && max[segments[i][j]-1]<ref_grad[i][j]) {
				res.x[segments[i][j]-1]=i;
				res.y[segments[i][j]-1]=j;
				max[segments[i][j]-1]=ref_grad[i][j];
			}
		}
	}

	return res;
} // Obtener las posiciones de valor mayor del gradiente ref_grad (más alejados) (saca las distancias)

Poss<int> contour_points(vector<vector<int>> segments)
{
	Poss<int> res;

	vector<int> nx={0, 1, 1, 1, 0,-1,-1,-1};
	vector<int> ny={1, 1, 0,-1,-1,-1, 0, 1};
	int x, y;

	int sx = segments.size(), sy = segments[0].size();
	vector<vector<bool>> td(sx,vector<bool>(sy,true));
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			for(int k=0; k<nx.size(); k++){
				x = i+nx[k]; y = j+ny[k];
				if(x>=0 && x<sx && y>=0 && y<sy){
					// No son un obstáculo y no pertenecen al mismo segmento
					if(segments[i][j] && segments[x][y] && segments[i][j]!=segments[x][y]){
						if(td[i][j]){
							res.x.push_back(i); res.y.push_back(j);
						}
						if(td[x][y]){
							res.x.push_back(x); res.y.push_back(y);
						}
						td[i][j]=false; td[x][y]=false;
					}
				}
			}
		}
	}
	return res;
}

vector<vector<int>> contour_points_(vector<vector<int>> segments)
{
	vector<vector<int>> res(4);

	vector<int> nx={0, 1, 1, 1, 0,-1,-1,-1};
	vector<int> ny={1, 1, 0,-1,-1,-1, 0, 1};
	int x, y;

	int sx = segments.size(), sy = segments[0].size();
	vector<vector<bool>> td(sx,vector<bool>(sy,true));
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			for(int k=0; k<nx.size(); k++){
				x = i+nx[k]; y = j+ny[k];
				if(x>=0 && x<sx && y>=0 && y<sy){
					// No son un obstáculo y no pertenecen al mismo segmento
					if(segments[i][j] && segments[x][y] && segments[i][j]!=segments[x][y]){
						if(td[i][j]){
							res[0].push_back(i); res[1].push_back(j);
							res[2].push_back(segments[i][j]); res[3].push_back(segments[x][y]);
						}
						if(td[x][y]){
							res[0].push_back(x); res[1].push_back(y);
							res[2].push_back(segments[i][j]); res[3].push_back(segments[x][y]);
						}
						td[i][j]=false; td[x][y]=false;
					}
				}
			}
		}
	}
	return res;
}

vector<vector<float>> slow_down_gradient(vector<vector<float>> grid, Poss<int> pos)
{
	vector<vector<float>> grad;

	// Obtener el gradiente de las posiciones
	FMM gr(pos.x,pos.y, grid);
	grad = gr.compute_gradient_();
	
	// Ahora en "pos" hay un obstáculo
	// Encontrar el mínimo no nulo
	float min = numeric_limits<float>::max();
	//float min = (float)RAND_MAX;
	for(int i=0; i<grid.size(); i++){
		for(int j=0; j<grid[0].size(); j++){
			// Las posiciones que no son un obstáculo
			if(grid[i][j] && grad[i][j]>0 && min>grad[i][j]){
				min = grad[i][j];
			}
		}
	}

	for(int i=0; i<grid.size(); i++){
		for(int j=0; j<grid[0].size(); j++){
			// Las posiciones que no son un obstáculo y pertenecen al contorno
			if(grid[i][j] && !grad[i][j]){
				grad[i][j] = min;
			}else if(!grid[i][j]){
				grad[i][j] = 0;
			}
		}
	}
	
	/*
	// Original
	// No me gusta como funciona, no corta del todo bien
	// Es mejorbuscar el mínimo del gradiente y fijar las transisiones a ese valor
	for(int i=0; i<grid.size(); i++){
		for(int j=0; j<grid[0].size(); j++){
			if(grid[i][j] && !grad[i][j]){ // Las posiciones que no son un obstáculo
				grad[i][j] = 1;
			}else if(!grid[i][j]){
				grad[i][j] = 0;
			}
		}
	}
	*/

	return grad;
} // Gradiente que reducirá la velocidad de propagación del frente de onda en posiciones "pos"

vector<vector<int>> disable_segments(vector<vector<int>> segm, vector<int> s)
{
	for(int i=0; i<segm.size(); i++){
		for(int j=0; j<segm[0].size(); j++){
			for(int k=0; k<s.size(); k++){
				if(segm[i][j]==s[k]) segm[i][j]=0;
			}
		}
	}
	return segm;
} // Anular los segmentos "s", fijar como obstáculos (valor 0 en el caso de segmentos)

vector<vector<float>> disable_grid(vector<vector<int>> segm, vector<vector<float>> grid)
{
	for(int i=0; i<segm.size(); i++){
		for(int j=0; j<segm[0].size(); j++){
			if(!segm[i][j]){
				grid[i][j] = 0;
			}
		}
	}
	return grid;
}

vector<vector<float>> disable_grid(vector<vector<int>> segm, int seg, vector<vector<float>> grid)
{
	for(int i=0; i<segm.size(); i++){
		for(int j=0; j<segm[0].size(); j++){
			if(segm[i][j]==seg){
				grid[i][j] = 0;
			}
		}
	}
	return grid;
}

Poss<int> disable_centroids(Poss<int> centr, vector<int> s)
{
	Poss<int> res;
	vector<bool> tdisc = ind2bool(centr.x.size(), s);
	for(int i=0; i<centr.x.size(); i++){

	}
	return res;
}

vector<vector<bool>> compute_adjacency(vector<vector<int>> segments, int n)
{
	vector<vector<bool>> res(n, vector<bool>(n,false));
	vector<int> nx={0, 1, 1, 1, 0,-1,-1,-1};
	vector<int> ny={1, 1, 0,-1,-1,-1, 0, 1};
	int x, y;
	int sx=segments.size(), sy=segments[0].size();
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			for(int k=0; k<8; k++){
				x=i+nx[k]; y=j+ny[k];
				if(x>=0 && x<sx && y>=0 && y<sy){
					if(segments[i][j] && segments[x][y] && segments[i][j]!=segments[x][y]){
						res[segments[i][j]-1][segments[x][y]-1] = true;
					}
				}
			}
		}
	}
	return res;
} // Grafo de adyacencia de los segmentos (representado con una matriz de booleanos)

vector<Path> adjacent_paths(Poss<int> centr, vector<vector<bool>> adj, vector<vector<float>> grid)
{
	vector<Path> res;
	vector<vector<float>> grad;
	Path aux;
	for(int i=0; i<centr.x.size(); i++){
		for(int j=i+1; j<centr.x.size(); j++){
			if(i!=j && adj[i][j]){
				FMM gr(centr.x[i],centr.y[i],grid);
				//grad = gr.compute_gradient_();
				grad = gr.expand2goal(centr.x[j],centr.y[j]);
				aux.gradient_descent_(grad,centr.x[j],centr.y[j]);
				res.push_back(aux);
				aux.clear();
				//res[j]=res[i];
			}
		}
	}

	return res;
} // Caminos (CONSIDERANDO LOS OBSTÁCULOS) entre los segmentos adyacentes

vector<vector<Path>> adjacent_paths_(Poss<int> centr, vector<vector<bool>> adj, vector<vector<float>> grid)
{
	vector<vector<Path>> res(centr.x.size(),vector<Path>(centr.x.size()));
	vector<vector<float>> grad;
	for(int i=0; i<centr.x.size(); i++){
		for(int j=i+1; j<centr.x.size(); j++){
			if(i!=j && adj[i][j]){
				FMM gr(centr.x[i],centr.y[i],grid);
				//grad = gr.compute_gradient_();
				grad = gr.expand2goal(centr.x[j],centr.y[j]);
				res[i][j].gradient_descent_(grad,centr.x[j],centr.y[j]);
				res[j][i] = res[i][j];
			}
		}
	}

	return res;
} // Caminos (CONSIDERANDO LOS OBSTÁCULOS) entre los segmentos adyacentes

vector<Poss<int>> adjacent_lines(Poss<int> centr, vector<vector<bool>> adj, vector<vector<float>> grid)
{
	vector<Poss<int>> res;
	vector<vector<float>> grad;
	Poss<int> aux1, aux2;
	aux1(1); aux2(1);
	for(int i=0; i<centr.x.size(); i++){
		for(int j=i+1; j<centr.x.size(); j++){
			if(i!=j && adj[i][j]){
				aux1.x[0] = centr.x[i]; aux1.y[0] = centr.y[i];
				aux2.x[0] = centr.x[j]; aux2.y[0] = centr.y[j];
				aux1.append(aux2);
				res.push_back(aux1);
				aux1.clear(); aux2.clear();
				aux1(1); aux2(1);
			}
		}
	}

	return res;
} // Caminos (EN LÍNEA RECTA) entre los segmentos adyacentes

vector<Poss<int>> segments_points(vector<vector<int>> segments, int segs)
{
	// YA QUE ESTOY, METO LOS OBSTÁCULOS
	vector<Poss<int>> res(segs);
	for(int i=0; i<segments.size(); i++){
		for(int j=0; j<segments[0].size(); j++){
			res[segments[i][j]].x.push_back(i);
			res[segments[i][j]].y.push_back(j);
		}
	}
	return res;
}

// -----------------------------------------------------------------------
// DOS FUNCIONES QUE NO VALEN PARA NADA POR AHORA
int find_root(vector<vector<float>> base_grad, Poss<int> centr)
{
	int ind=0;
	float min = (float)RAND_MAX;
	for(int i=0; i<centr.x.size(); i++){
		if(min>base_grad[centr.x[i]][centr.y[i]]){
			min=base_grad[centr.x[i]][centr.y[i]];
			ind = i;
		}
	}
	return ind;
} // ESTO ES UNA TONTERÍA (LA BASE ESTÁ DENTRO DE UN SEGMENTO)

int segment_of_root(vector<vector<int>> segm, int xbase, int ybase)
{
	return segm[xbase][ybase]-1;
} // ESTO SE PUEDE HACER FUERA
// -----------------------------------------------------------------------

vector<int> segment_degree(vector<vector<bool>> adj, int root)
{
	int n = adj.size();
	vector<int> res(n,0);

	vector<bool> td(n,true);
	td[root] = false;
	queue<int> q; q.push(root);
	int degree = 1;
	while(!q.empty()){
		for(int i=0; i<n; i++){
			if(td[i] && adj[q.front()][i]){
				q.push(i);
				res[i]=res[q.front()]+1;
				td[i]=false;
			}
		}
		q.pop();
	}

	return res;
}


vector<int> select_by_depth(vector<vector<int>> segs, Poss<int> centroids, vector<int> depths, int seld)
{
	vector<int> res;
	for(int i=0; i<depths.size(); i++){
		if(depths[i]==seld){
			res.push_back(segs[centroids.x[i]][centroids.y[i]]);
		}
	}
	return res;
} // Seleción de los índices de los segmentos por profundidad

vector<int> select_by_depths(vector<vector<int>> segs, Poss<int> centroids, vector<int> depths, vector<int> seld)
{
	vector<int> res;
	for(int i=0; i<depths.size(); i++){
		for(int j=0; j<seld.size(); j++){
			if(depths[i]==seld[j]){
				res.push_back(segs[centroids.x[i]][centroids.y[i]]);
			}
		}
	}
	return res;
} // Seleción de los índices de los segmentos por profundidades

vector<Poss<int>> obstacles_per_segs_zs(Poss<int> obstacles, vector<vector<int>> segments, int nsegs)
{
    vector<Poss<int>> res;
    if(!obstacles.x.size()) return res;
    res.resize(nsegs);
    for(int i=0; i<obstacles.x.size(); i++){
        res[segments[obstacles.x[i]][obstacles.y[i]]].x.push_back(obstacles.x[i]);
        res[segments[obstacles.x[i]][obstacles.y[i]]].y.push_back(obstacles.y[i]);
    }
    return res;
} // Separar los obstáculos por segmentos (los segmentos empiezan por "0")

vector<int> number_of_obstacles_in_segments_zs(Poss<int> obstacles, vector<vector<int>> segments, int nsegs)
{
    vector<int> res(nsegs,0);
    if(!obstacles.x.size()) return res;
    for(int i=0; i<obstacles.x.size(); i++){
        if(segments[obstacles.x[i]][obstacles.y[i]]>=0) res[segments[obstacles.x[i]][obstacles.y[i]]]++;
    }
    return res;
} // Cantidad de obstáculos por segmentos (los segmentos empiezan por "0")

vector<Poss<int>> obstacles_per_segs(Poss<int> obstacles, vector<vector<int>> segments, int nsegs)
{
    vector<Poss<int>> res;
    if(!obstacles.x.size()) return res;
    res.resize(nsegs);
    for(int i=0; i<obstacles.x.size(); i++){
        if(segments[obstacles.x[i]][obstacles.y[i]]>0){
            res[segments[obstacles.x[i]][obstacles.y[i]]-1].x.push_back(obstacles.x[i]);
            res[segments[obstacles.x[i]][obstacles.y[i]]-1].y.push_back(obstacles.y[i]);
        }
    }
    return res;
}

vector<int> number_of_obstacles_in_segments(Poss<int> obstacles, vector<vector<int>> segments, int nsegs)
{
    vector<int> res(nsegs,0);
    if(!obstacles.x.size()) return res;
    for(int i=0; i<obstacles.x.size(); i++){
        if(segments[obstacles.x[i]][obstacles.y[i]]>0) res[segments[obstacles.x[i]][obstacles.y[i]]-1]++;
    }
    return res;
}
    
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
// ESTIMACIONES DE LOS TRABAJOS DENTRO DE LOS SEGMENTOS
vector<int> estimated_goals_ud(vector<int> areas, int tot_area, int ngoals)
{	
	int n = areas.size();
	vector<int> res(n,0);
	vector<float> prop(n,0); // Proporción respecto del area total
	int nd = 0;
	for(int i=0; i<n; i++){
		prop[i] = (float)areas[i]/(float)tot_area;
		res[i] = round((float)ngoals*prop[i]);
		if(!res[i]) res[i]=1;
		nd+=res[i];
	}

	// POR AHORA MEJOR DEJARLO ASÍ (con round)
	// Creo que de algún modo representa los goals que pueden a llegar a caer
	// en un segmento a largo plazo

	/*
	sh_vect_h(res,"goals"); sh_vect_h(prop,"areas");
	cout<<"Goals "<<nd<<"/"<<ngoals<<endl;
	*/

	return res;
} // Estimación de la cantidad de goals que habrá en cada segmento (distribución uniforme)

vector<float> travel_time_estimation(vector<vector<int>> segments, Poss<int> centr, vector<int> ngoals, vector<vector<float>> grid, float speed, int iter)
{
	int nseg = centr.x.size();
	vector<float> res(nseg,0);

	vector<vector<float>> grid_, obst_grad;
	Poss<int> obst;
	segmentation::segment seg_vars;
	Poss<int> pos;

	vector<int> tour;
	Path camino;

	vector<vector<vector<float>>> grads;

	Poss<int> pos_ag; // Para guardar

	for(int i=0; i<nseg; i++){
		if(ngoals[i]<=1){
			res[i]=0;
			continue;
		}
		// POSICIONAR LOS GOALS VIRTUALES PARA LA ESTIMACIÓN
		// Grid del segmento
		grid_ = delimit_segment(segments, i+1, obst);
		// Calcular el gradiente de los obstáculos
		FMM gr(obst.x,obst.y,grid_);
		obst_grad = gr.compute_gradient_();
		// Segmentar con PAP
		seg_vars = segmentation::PAP(grid_, ngoals[i], iter, obst_grad);

		// ESTIMAR EL RECORRIDO
		// Camino desde el centroide del segmento del trabajador por los goals virtuales
		pos.x.push_back(centr.x[i]); pos.y.push_back(centr.y[i]);
		pos.append(seg_vars.centroids);
		grads = allocation::compute_all_gradients(pos, grid_);

		if(pos.x.size()<13){
			tour = allocation::brute_force(allocation::form_cost_matrix(pos, grads, speed));
		}else{
			tour = allocation::nn_2o(allocation::form_cost_matrix(pos, grads, speed));
		}

		camino = allocation::compute_path(pos, tour, grads);
		camino.compute_time(speed);		

		pos_ag.append(seg_vars.centroids);

		// guardar el tiempo
		res[i] = camino.t;

		// Limpiar la variable de los obstáculos
		obst.clear();
		pos.clear();
	}

	//pos_ag.save(name);

	return res;
} // Estimar el tiempo que va a estar viajando cada agente en su segmento, situando ngoals con PAP

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
// Variables de la segmentación
struct segmentation_variables{
	int n; // Número de segmentos
	vector<vector<int>> segments; // Segmentos sobre el grid
	Poss<int> centroids; // Centroides de los segmentos
    vector<int> areas; // Areas de los segmentos
    vector<vector<bool>> graph; // Grafo de los segmentos
    vector<int> depth;  // Profundidad de los segmentos desde la base
    Poss<int> far; // Los más alejados de los segmentos
    vector<float> radius;  // Las mayores distancias dentro de los segmentos desde sus respectivos centroides
    vector<vector<float>> grid; // Gradiente que lleva la información de los contornos de las particiones de los trabajadores
    Poss<int> contour; // Puntos delimitadores de los segmentos
    vector<vector<float>> contour_gradient; // Gradiente de los contornos de los segmentos

    vector<Path> graph_paths; // Caminos reales
    vector<Poss<int>> graph_lines;
};

segmentation_variables compute_segmentation_variables(vector<vector<int>> segments, int nseg, vector<int> areas, vector<vector<float>> grid, Poss<int> base)
{
	segmentation_variables res;

	res.segments = segments;
	res.n = nseg;
	res.areas = areas;
	res.centroids = compute_centroids(segments, nseg, grid, res.far, res.radius);
	res.graph = compute_adjacency(segments, nseg);
	res.depth = segment_degree(res.graph, segments[base.x[0]][base.y[0]]-1);
	res.contour = contour_points(segments);
	res.contour_gradient = slow_down_gradient(grid, res.contour);

	res.graph_paths = adjacent_paths(res.centroids, res.graph, grid);
	res.graph_lines = adjacent_lines(res.centroids, res.graph, grid);

	return res;	
} // Calcular las variables de los segmentos

void save_segmentation_variables(segmentation_variables var, string results_folder, int red_fact)
{
	// Segmentos
	save_matr((results_folder+"segments.txt").c_str(),var.segments);
	// Centroides
	save_poss(var.centroids, red_fact, (results_folder+"segment_centroids.txt"));
	// Grafo
	save_matr((results_folder+"segment_graph.txt").c_str(),var.graph);
	// Caminos entre segmentos adyacentes
	save_paths(var.graph_paths, red_fact, (results_folder+"segment_real_paths_"));
	// Lineas entre los segmentos adyacentes
	save_poss(var.graph_lines, red_fact, (results_folder+"segment_direct_paths_"));
	// Puntos de los contornos
	save_poss(var.contour, red_fact, (results_folder+"segment_contour.txt"));
	// Gradiente del contorno
	save_matr((results_folder+"segment_contour_gradient.txt").c_str(), var.contour_gradient);
} // Guardar las variables más importantes

void save_segmentation_variables(segmentation_variables var, string results_folder, int seg_index, int red_fact)
{
	// Segmentos
	save_matr((results_folder+"segments_"+to_string(seg_index)+".txt").c_str(),var.segments);
	// Centroides
	save_poss(var.centroids, red_fact, (results_folder+"segment_"+to_string(seg_index)+"_centroids.txt"));
	// Grafo
	save_matr((results_folder+"segment_"+to_string(seg_index)+"_graph.txt").c_str(),var.graph);
	// Caminos entre segmentos adyacentes
	save_paths(var.graph_paths, red_fact, (results_folder+"segment_"+to_string(seg_index)+"_real_paths_"));
	// Lineas entre los segmentos adyacentes
	save_poss(var.graph_lines, red_fact, (results_folder+"segment_"+to_string(seg_index)+"_direct_paths_"));
	// Puntos de los contornos
	save_poss(var.contour, red_fact, (results_folder+"segment_"+to_string(seg_index)+"_contour.txt"));
	// Gradiente del contorno
	save_matr((results_folder+"segment_"+to_string(seg_index)+"_contour_gradient.txt").c_str(), var.contour_gradient);
} // Guardar las variables más importantes

#endif