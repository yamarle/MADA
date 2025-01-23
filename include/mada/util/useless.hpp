#ifndef USELESS_HPP
#define USELESS_HPP

#include "fmm_2.hpp"
#include "path.hpp"

// Para hacer transformaciones del grid (para FMM)
std::vector<std::vector<int> > to_int(std::vector<std::vector<float> > matrix);
std::vector<std::vector<float> > to_float(std::vector<std::vector<int> > matrix);
std::vector<std::vector<float> > con_grid(std::vector<std::vector<int>> matrix, std::vector<std::vector<float> > grid);
Poss<int> to_poss(std::vector<std::vector<float>> map_pos, float xo, float yo, float res);
std::vector<std::vector<float>> to_map_positions(vector<int> x, vector<int> y, float xo, float yo, float res);
vector<vector<vector<float>>> transform_pos_matrix(vector<Poss<int>> positions, float xo, float yo, float res);

vector<float> paths_times(vector<Path> paths);

bool check_position(int x, int y, vector<vector<float>> reference_gradient);

void inflate_goals(vector<vector<float>> &grid, int x, int y, int nc);
void check_goals_access(Poss<int> &goals_grid, vector<vector<float>> &goals_map, vector<vector<float>> workspace);

vector<float> generate_point(vector<vector<float>> grid); // Generar un punto aleatorio en espacio libre
vector<float> generate_point(vector<vector<float>> grid, vector<float> limits); // Generar un punto aleatorio en espacio libre delimitado
vector<vector<float>> generate_points(vector<vector<float>> grid, int n);
vector<vector<float>> generate_points(vector<vector<float>> grid, int n, vector<float> limits);

vector<vector<float>> free_space_points(vector<vector<float>> grid);
vector<vector<float>> compute_gradient(int x, int y, vector<vector<float>> grid); // calcular gradiente
vector<vector<float>> compute_gradient(vector<int> x, vector<int> y, vector<vector<float>> grid); // calcular gradiente
vector<vector<float>> obst_grad(vector<vector<float>> grid); // gradiente desde los obstáculos

vector<vector<vector<float>>> compute_gradients(Poss<int> start, vector<vector<float>> grid); // Calcular gradientes
vector<vector<vector<float>>> compute_gradients(Poss<int> start, Poss<int> goals, vector<vector<float>> grid); // Calcular gradientes hasta los goals

Poss<int> obst_selection(vector<vector<float>> grid);
Poss<int> unexpl_selection(vector<vector<float>> grid);

vector<vector<float>> uniform_random_matrix(int x, int y, float lb, float ub);

Path compute_tour_path(Poss<int> pos, vector<int> tour, vector<vector<vector<float>>> grads);

vector<vector<float>> form_cost_matrix(vector<vector<vector<float>>> grads, Poss<int> pos); // Formar la matriz de costes para los algoritmos de asignación (TONTERIA)

Poss<int> remove_poss(Poss<int> pos, vector<bool> to_remove);
Poss<int> remove_poss_from_index(Poss<int> pos, vector<int> index);

vector<vector<float>> distance_matrix(Poss<int> pos); // Obtener la matriz de distancias

float gradient_maximum_no_inf(vector<vector<float>> grad);
float matrix_maximum_no_inf(vector<vector<float>> matr);
float matrix_maximum_no_inf(vector<vector<float>> matr, int &x, int &y);
int matrix_maximum_no_inf(vector<vector<int>> matr);
int matrix_maximum_no_inf(vector<vector<int>> matr, int &x, int &y);

template <typename T>
vector<vector<T>> anular_grid(vector<vector<T>> grid, vector<Poss<int>> pos, vector<int> i2r)
{
	for(int i=0; i<i2r.size(); i++)
		for(int j=0; j<pos[i].x.size(); j++)
			grid[pos[i].x[j]][pos[i].y[j]] = 0;
	return grid;
}

template <typename T>
int number_of_free_cells(vector<vector<T>> grid)
{
	int res = 0;
	for(int i=0; i<grid.size(); i++){
		for(int j=0; j<grid[i].size(); j++){
			if(grid[i][j]) res++;
		}
	}
	return res;
}

template <typename T>
vector<T> remove_from_index_v(vector<T> vect, vector<int> ind)
{
	vector<T> res;
	vector<bool> tr = ind2bool(vect.size(),ind);
	for(int i=0; i<vect.size(); i++){
		if(!tr[i])
			res.push_back(vect[i]);
	}
	return res;
}

template <typename T>
vector<T> remove_from_index_v(vector<T> vect, vector<bool> ind)
{
	vector<T> res;
	for(int i=0; i<vect.size(); i++){
		if(!ind[i])
			res.push_back(vect[i]);
	}
	return res;
}


template <typename T>
vector<vector<T>> remove_from_index_m(vector<vector<T>> matr, vector<bool> ind)
{
	vector<vector<T>> res;
	for(int i=0; i<ind.size(); i++){
		if(!ind[i]) res.push_back(matr[i]);
	}
	int i=0;
	while(i<res[0].size()-1){
		if(ind[i]){
			for(int j=0; j<res.size(); j++)
				res[j].erase(res[j].begin()+i);
		}else{
			i++;
		}
	}
	return res;
}

template <typename T>
vector<vector<T>> remove_from_index_m(vector<vector<T>> matr, vector<int> ind)
{
	vector<T> res;
	vector<bool> tr = ind2bool(matr.size(),ind);
	for(int i=0; i<tr.size(); i++){
		if(!tr[i]) res.push_back(matr[i]);
	}
	int i=0;
	while(i<res[0].size()-1){
		if(tr[i]){
			for(int j=0; j<res.size(); j++)
				res[j].erase(res[j].begin()+i);
		}else{
			i++;
		}
	}
	return res;
}

vector<int> obtain_traversed_segments(Path path, vector<vector<int>> segments, int s);
float traversed_segments_cost(vector<int> segs, vector<float> costs);
float compute_path_cost(Path path, vector<vector<int>> segMap, int s, vector<float> segCosts, vector<vector<float>> navMap);

vector<vector<int>> select_goals_in_segment(vector<vector<int>> segments, int seg, Poss<int> goals, vector<int> goals_index);

Path paths_to_path(vector<Path> paths); // Concatenar los caminos en uno

int number_of_reached_goals(vector<int> tour);

vector<int> remaining_goals(vector<int> tour, vector<int> goals);

Poss<int> remaining_goals(vector<int> tour, Poss<int> goals);

Poss<int> find_closest(int x, int y, Poss<int> pos, vector<vector<float>> grid);

template<typename T>
void remove_points(T x, T y, vector<T> &xv, vector<T> &yv, float dist)
{
	// Encuentro el punto del camino más cercano a (x, y)
	int ind = 0;
	float d, dmin = INFINITY;
	for(int i=0; i<xv.size(); i++){
		d = hypot(x - xv[i], y - yv[i]);
		if(dmin >= d){
			dmin = d;
			ind = i;
		}
	}
	// Elimino todos los puntos del camino hasta ese punto
	xv.erase(xv.begin(), xv.begin() + ind); yv.erase(yv.begin(), yv.begin() + ind);
	// Elimino todos los puntos
	while(xv.size())
        if(hypot(x - xv[0], y - yv[0]) <= dist){
            xv.erase(xv.begin()); yv.erase(yv.begin());
        }else break;
}

// --------------------------------------------------------------------------------------------
// FUNCIONES DE GRAFOS
// --------------------------------------------------------------------------------------------
template<typename T>
vector<int> node_connectivity(vector<vector<T>> graph)
{
	vector<int> res(graph.size());
	for(int i=0; i<graph.size(); i++){
		for(int j=0; j<graph.size(); j++){
			if(graph[i][j]) res[i]++;
		}
	}
	return res;
}

template<typename T>
vector<int> graph_contour(vector<vector<T>> graph)
{
	vector<int> res;

	return res;
}

vector<int> find_graph_centroid(vector<vector<bool>> adj); // Calcular el/los centroid/es de un grafo

template<typename T>
vector<vector<T>> delete_edges(vector<vector<T>> graph, vector<int> vertex)
{
	// vertex: son los índices de los vertices que se han eliminado
	for(int vc=0; vc<vertex.size(); vc++){
		for(int i=0; i<graph.size(); i++){
			graph[i][vertex[vc]]=false;
			graph[vertex[vc]][i]=false;
		}
	}
	return graph;
} // Eliminar las aristas(conexiones) de un grafo (para grafos simétricos) en base a unos vertices que he quitado

template<typename T>
vector<vector<T>> delete_vertices(vector<vector<T>> graph, vector<int> vertex)
{
	// vertex: son los índices de los vertices que se han eliminado
	int n = graph.size(), nr=0;
	vector<bool> td(n,false);
	for(int i=0; i<vertex.size(); i++) td[vertex[i]]=true;
	vector<vector<T>> res;

	for(int i=0; i<n; i++){
		if(td[i]){
			nr++;
			res.resize(nr);
		}
	}

	return res;
} // Eliminar los vértices de un grafo

// --------------------------------------------------------------------------------------------

// ======================================================================
// ======================================================================
// ======================================================================
// Tonterias para guardar
// ======================================================================
// ======================================================================
// ======================================================================

void save_path(Path path, int red_fact, string name);
void save_paths(vector<Path> paths, int red_fact, string name);
void save_poss(Poss<int> pos, int red_fact, string name);
void save_poss(vector<Poss<int>> pos, int red_fact, string name);
void save_poss(vector<vector<Poss<int>>> pos, int red_fact, string name);

#endif
	