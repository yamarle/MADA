
#include <mada/util/useless.hpp>

// Para hacer transformaciones del grid (para FMM)
std::vector<std::vector<int> > to_int(std::vector<std::vector<float> > matrix)
{
    int x=matrix.size(),y=matrix[0].size();
    std::vector<std::vector<int>> res(x,std::vector<int>(y,0));
    for(int i=0; i<x; i++)
        for(int j=0; j<y; j++)
            res[i][j]=(int)matrix[i][j];
    return res;
}

std::vector<std::vector<float> > to_float(std::vector<std::vector<int> > matrix)
{
    int x=matrix.size(),y=matrix[0].size();
    std::vector<std::vector<float>> res(x,std::vector<float>(y,0));
    for(int i=0; i<x; i++)
        for(int j=0; j<y; j++)
            res[i][j]=(float)matrix[i][j];
    return res;
}

std::vector<std::vector<float> > con_grid(std::vector<std::vector<int>> matrix, std::vector<std::vector<float> > grid)
{
    int x=matrix.size(),y=matrix[0].size();
    std::vector<std::vector<float>> res(x,std::vector<float>(y,0));
    for(int i=0; i<x; i++)
        for(int j=0; j<y; j++)
            if(grid[i][j]) res[i][j]=(float)matrix[i][j]+1;
    // +1: para que todo el mapa sea navegable, si no las zonas sin señal se interpretan como obstáculos
    return res;
}

Poss<int> to_poss(std::vector<std::vector<float>> map_pos, float xo, float yo, float res)
{
    Poss<int> pos;
    pos(map_pos[0].size());
    for(int i=0; i<map_pos[0].size(); i++){
        pos.x[i] = (map_pos[0][i]-xo)/res;
        pos.y[i] = (map_pos[1][i]-yo)/res;
    }
    return pos;
}

std::vector<std::vector<float>> to_map_positions(vector<int> x, vector<int> y, float xo, float yo, float res)
{
    std::vector<std::vector<float>> pos(2,std::vector<float>(x.size(),0));
    for(int i=0; i<x.size(); i++){
        pos[0][i] = x[i]*res+xo;
        pos[1][i] = y[i]*res+yo;
    }
    return pos;
}

vector<vector<vector<float>>> transform_pos_matrix(vector<Poss<int>> positions, float xo, float yo, float res)
{
    int n = positions.size();
    //vector<vector<vector<float>>> pos(n,vector<vector<float>>(2));
    vector<vector<vector<float>>> pos(n);

    for(int i=0; i<n; i++){
        pos[i] = to_map_positions(positions[i].x, positions[i].y, xo, yo, res);
    }

    return pos;
}

vector<float> paths_times(vector<Path> paths)
{
    vector<float> res(paths.size(),0);
    for(int i=0; i<paths.size(); i++)
        res[i] = paths[i].t;
    return res;
}

bool check_position(int x, int y, vector<vector<float>> reference_gradient)
{
    if(x<0 || x>=reference_gradient.size() || y<0 || y>=reference_gradient[0].size()) return false;
    if(reference_gradient[x][y] == INF) return false;
    return true;
}

void inflate_goals(vector<vector<float>> &grid, int x, int y, int nc)
{
    int sx = grid.size(), sy = grid[0].size();
    for(int i=x-nc; i<x+nc; i++){
        for(int j=y-nc; j<y+nc; j++){
            if(i>=0 && i<sx && j>=0 && j<sy){
                grid[i][j]=0;
            }
        }
    }
}

void check_goals_access(Poss<int> &goals_grid, vector<vector<float>> &goals_map, vector<vector<float>> workspace)
{
    int i=0;
    while(i<goals_grid.x.size()){
        if(workspace[goals_grid.x[i]][goals_grid.y[i]]<INF){
            i++;
        }else{
            goals_grid.x.erase(goals_grid.x.begin()+i);
            goals_grid.y.erase(goals_grid.y.begin()+i);
            goals_map[0].erase(goals_map[0].begin()+i);
            goals_map[1].erase(goals_map[1].begin()+i);
        }
    }
}

vector<float> generate_point(vector<vector<float>> grid)
{
    vector<float> res(2);
    res[0] = static_cast<float>(rand())/static_cast<float>(RAND_MAX/grid.size());
    res[1] = static_cast<float>(rand())/static_cast<float>(RAND_MAX/grid[0].size());
    while(!grid[floor(res[0])][floor(res[1])]){
        res[0] = static_cast<float>(rand())/static_cast<float>(RAND_MAX/grid.size());
        res[1] = static_cast<float>(rand())/static_cast<float>(RAND_MAX/grid[0].size());
    }
    return res;
} // Generar un punto aleatorio en espacio libre

vector<float> generate_point(vector<vector<float>> grid, vector<float> limits)
{
    vector<float> res(2);
    res[0] = limits[0] + static_cast<float>(rand())/static_cast<float>(RAND_MAX/(limits[1]-limits[0]));
    res[1] = limits[2] + static_cast<float>(rand())/static_cast<float>(RAND_MAX/(limits[3]-limits[2]));
    while(!grid[floor(res[0])][floor(res[1])]){
        res[0] = limits[0] + static_cast<float>(rand())/static_cast<float>(RAND_MAX/(limits[1]-limits[0]));
        res[1] = limits[2] + static_cast<float>(rand())/static_cast<float>(RAND_MAX/(limits[3]-limits[2]));
    }
    return res;
} // Generar un punto aleatorio en espacio libre delimitado

vector<vector<float>> generate_points(vector<vector<float>> grid, int n)
{
    vector<vector<float>> res(n);
    for(int i=0; i<n; i++){
        res[i] = generate_point(grid);
        grid[res[i][0]][res[i][1]] = 0;
    }

    return res;
} // Generar puntos aleatorios en espacio libre

vector<vector<float>> generate_points(vector<vector<float>> grid, int n, vector<float> limits)
{
    vector<vector<float>> res(n);
    for(int i=0; i<n; i++){
        res[i] = generate_point(grid, limits);
        grid[res[i][0]][res[i][1]] = 0;
    }

    return res;
} // Generar puntos aleatorios en espacio libre delimitado

vector<vector<float>> free_space_points(vector<vector<float>> grid)
{
    vector<vector<float>> res;
    for(int i=0; i<grid.size(); i++)
        for(int j=0; j<grid[0].size(); j++)
            if(grid[i][j]){
                res.push_back(vector<float>(2));
                res[res.size()-1][0] = i;
                res[res.size()-1][1] = j;
            }
    return res;
}

vector<vector<float>> compute_gradient(int x, int y, vector<vector<float>> grid)
{
    FMM gr(x, y, grid);
    return gr.compute_gradient_();
} // gradiente desde los obstáculos

vector<vector<float>> compute_gradient(vector<int> x, vector<int> y, vector<vector<float>> grid)
{
    FMM gr(x, y, grid);
    return gr.compute_gradient_();
} // gradiente desde los obstáculos

vector<vector<float>> obst_grad(vector<vector<float>> grid)
{
    Poss<int> obst_pos = pos_obs_(grid);
    FMM gr(obst_pos.x,obst_pos.y, grid);
    return gr.compute_gradient_();
} // gradiente desde los obstáculos

vector<vector<vector<float>>> compute_gradients(Poss<int> start, vector<vector<float>> grid)
{
    vector<vector<vector<float>>> res(start.x.size());
    for(int i=0; i<start.x.size(); i++){
        FMM gr(start.x[i], start.y[i], grid);
        res[i] = gr.compute_gradient_();
    }
    return res;
}

vector<vector<vector<float>>> compute_gradients(Poss<int> start, Poss<int> goals, vector<vector<float>> grid)
{
    vector<vector<vector<float>>> res(start.x.size());
    for(int i=0; i<start.x.size(); i++){
        FMM gr(start.x[i], start.y[i], grid);
        res[i] = gr.pos_coverage(goals);
    }
    return res;
}

Poss<int> obst_selection(vector<vector<float>> grid)
{
    // Seleccionar los puntos mejor
    Poss<int> obst_pos;

    int sx = grid.size(), sy = grid[0].size();
    vector<int> nx={0,1,1, 1, 0,-1,-1,-1};
    vector<int> ny={1,1,0,-1,-1,-1, 0, 1};

    vector<vector<bool>> done(sx,vector<bool>(sy,false));
    bool v;
    int x_,y_;
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            v = false;
            for(int k=0; k<8; k++){
                x_ = i+nx[k]; y_ = j+ny[k];
                if(x_<sx && x_>=0 && y_>=0 && y_<sy){
                    if(!grid[i][j] && grid[x_][y_]){
                        v = true;
                        break;
                    }
                }
            }
            if(v){
                obst_pos.x.push_back(i);
                obst_pos.y.push_back(j);
            }
        }
    }
    return obst_pos;
}

Poss<int> unexpl_selection(vector<vector<float>> grid)
{
    Poss<int> unexpl_pos;

    int sx = grid.size(), sy = grid[0].size();
    vector<int> nx={0,1,1, 1, 0,-1,-1,-1};
    vector<int> ny={1,1,0,-1,-1,-1, 0, 1};

    vector<vector<bool>> done(sx,vector<bool>(sy,false));
    bool v;
    int x_,y_;
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            v = false;
            for(int k=0; k<8; k++){
                x_ = i+nx[k]; y_ = j+ny[k];
                if(x_<sx && x_>=0 && y_>=0 && y_<sy){
                    if(grid[i][j]<0 && !grid[x_][y_]){
                        v = true;
                        break;
                    }
                }
            }
            if(v){
                unexpl_pos.x.push_back(i);
                unexpl_pos.y.push_back(j);
            }
        }
    }
    return unexpl_pos;
}

vector<vector<float>> uniform_random_matrix(int x, int y, float lb, float ub)
{
    vector<vector<float>> res(x, vector<float>(y,0));
    for(int i=0; i<x; i++)
        for(int j=0; j<y; j++)
            if(i!=j)
                res[i][j]=((float)rand()/RAND_MAX)*(ub-lb)+lb;
    return res;
}


Path compute_tour_path(Poss<int> pos, vector<int> tour, vector<vector<vector<float>>> grads)
{
    Path res, aux;

    int n = pos.x.size();

    for(int i=0; i<n-1; i++){
        aux.gradient_descent_(grads[tour[i]],pos.x[tour[i+1]],pos.y[tour[i+1]]);
        res.append_(aux);
        aux.clear();
    }

    return res;
}

vector<vector<float>> form_cost_matrix(vector<vector<vector<float>>> grads, Poss<int> pos)
{
    int n = pos.x.size();
    vector<vector<float>> res(n,vector<float>(n,0));

    for(int i=0; i<n; i++){
        for(int j=0; j<n; j++){
            res[i][j] = grads[i][pos.x[j]][pos.y[j]];
        }
    }

    return res;
} // Formar la matriz de costes para los algoritmos de asignación (TONTERIA)

Poss<int> remove_poss(Poss<int> pos, vector<bool> to_remove)
{
    // VAYA TELA
    Poss<int> res;
    for(int i=0; i<pos.x.size(); i++){
        if(!to_remove[i]){
            res.x.push_back(pos.x[i]);
            res.y.push_back(pos.y[i]);
        }
    }
    return res;
}

Poss<int> remove_poss_from_index(Poss<int> pos, vector<int> index)
{
    // VAYA TELA
    vector<bool> tr(pos.x.size(),false);
    for(int i=0; i<index.size(); i++) tr[index[i]]=true;
    Poss<int> res;
    for(int i=0; i<pos.x.size(); i++){
        if(!tr[i]){
            res.x.push_back(pos.x[i]);
            res.y.push_back(pos.y[i]);
        }
    }
    return res;
}

vector<vector<float>> distance_matrix(Poss<int> pos)
{
    vector<vector<float>> res(pos.x.size(), vector<float>(pos.x.size(),0));
    for(int i=0; i<pos.x.size(); i++)
        for(int j=0; j<pos.x.size(); j++)
            res[i][j] = sqrt(pow(pos.x[i]-pos.x[j],2)+pow(pos.y[i]-pos.y[j],2));
    return res;
}

float gradient_maximum_no_inf(vector<vector<float>> grad)
{
    float res = -(float)RAND_MAX;
    for(int i=0; i<grad.size(); i++)
        for(int j=0; j<grad[0].size(); j++)
            if(grad[i][j] < (float)RAND_MAX && res < grad[i][j]){
                res = grad[i][j];
            }
    return res;
}

float matrix_maximum_no_inf(vector<vector<float>> matr)
{
    float res = numeric_limits<float>::min();
    for(int i=0; i<matr.size(); i++)
        for(int j=0; j<matr[0].size(); j++)
            if(matr[i][j] < numeric_limits<float>::max() && res < matr[i][j]){
                res = matr[i][j];
            }
    return res;
}

float matrix_maximum_no_inf(vector<vector<float>> matr, int &x, int &y)
{
    float res = numeric_limits<float>::min();
    for(int i=0; i<matr.size(); i++)
        for(int j=0; j<matr[0].size(); j++)
            if(matr[i][j] < numeric_limits<float>::max() && res < matr[i][j]){
                res = matr[i][j];
                x = i; y = j;
            }
    return res;
}

int matrix_maximum_no_inf(vector<vector<int>> matr)
{
    int res = numeric_limits<int>::min();
    for(int i=0; i<matr.size(); i++)
        for(int j=0; j<matr[0].size(); j++)
            if(matr[i][j] < numeric_limits<int>::max() && res < matr[i][j]){
                res = matr[i][j];
            }
    return res;
}

int matrix_maximum_no_inf(vector<vector<int>> matr, int &x, int &y)
{
    int res = numeric_limits<int>::min();
    for(int i=0; i<matr.size(); i++)
        for(int j=0; j<matr[0].size(); j++)
            if(matr[i][j] < numeric_limits<int>::max() && res < matr[i][j]){
                res = matr[i][j];
                x = i; y = j;
            }
    return res;
}

vector<int> obtain_traversed_segments(Path path, vector<vector<int>> segments, int s)
{
    vector<int> res;
    if(!path.tam) return res;
    res.push_back(segments[path.x[0]][path.y[0]]-s);
    for(int i=1; i<path.x.size(); i++){
        if(segments[path.x[i]][path.y[i]]-s != res[res.size()-1])
            res.push_back(segments[path.x[i]][path.y[i]]-s);
    }
    return res;
} // Obtiene los segmentos que atraviesa un camino "path"  (los segmentos empiezan por 1)

float traversed_segments_cost(vector<int> segs, vector<float> costs)
{
    float res = 0;
    for(int i=0; i<segs.size(); i++){
        res += costs[segs[i]];
    }
    return res;
}

float compute_path_cost(Path path, vector<vector<int>> segMap, int s, vector<float> segCosts, vector<vector<float>> navMap)
{
    float res = 0;
    if(!path.tam) return res;
    int seg = segMap[path.x[0]][path.y[0]]-s;
    for(int i=1; i<path.x.size(); i++){
        if(navMap[path.x[i]][path.y[i]] ==  0) return 0;
        if(segMap[path.x[i]][path.y[i]]-s != seg){
            seg = segMap[path.x[i]][path.y[i]]-s;
            res += segCosts[seg];
        }
    }
    return res;   
}

vector<vector<int>> select_goals_in_segment(vector<vector<int>> segments, int seg, Poss<int> goals, vector<int> goals_index)
{
    vector<vector<int>> res(3);
    for(int i=0; i<goals.x.size(); i++){
        if(segments[goals.x[i]][goals.y[i]]==seg){
            res[0].push_back(goals.x[i]);
            res[1].push_back(goals.y[i]);
            res[2].push_back(goals_index[i]);
        }
    }
    return res;
}

Path paths_to_path(vector<Path> paths)
{
    Path res;

    res = paths[0];
    for(int i=1; i<paths.size(); i++){
        res.append(paths[i]);
    }

    return res;
}

int number_of_reached_goals(vector<int> tour)
{
    int res = 0;
    for(int i=1; i<tour.size(); i++){
        if(tour[i]) res++;
    }
    return res;
}

vector<int> remaining_goals(vector<int> tour, vector<int> goals)
{
    vector<int> res;

    vector<bool> done(goals.size(),false);
    for(int i=1; i<tour.size(); i++){
        if(tour[i]) done[tour[i]] = true;
    }

    for(int i=0; i<done.size(); i++){
        if(!done[i]) res.push_back(goals[i]);
    }

    return res;
}

Poss<int> remaining_goals(vector<int> tour, Poss<int> goals)
{
    Poss<int> res;

    vector<bool> done(goals.x.size(),false);
    for(int i=1; i<tour.size(); i++){
        if(tour[i]) done[tour[i]] = true;
    }

    for(int i=0; i<done.size(); i++){
        if(!done[i]){
            res.x.push_back(goals.x[i]);
            res.y.push_back(goals.y[i]);
        }
    }

    return res;
}

Poss<int> find_closest(int x, int y, Poss<int> pos, vector<vector<float>> grid)
{
    Poss<int> res; res(1);

    FMM gr(x, y, grid);
    vector<vector<float>> grad = gr.compute_gradient_();

    float min = numeric_limits<float>::max();
    for(int i=0; i<pos.x.size(); i++){
        if(min>grad[pos.x[i]][pos.y[i]]){
            res.x[0]= pos.x[i]; res.y[0]= pos.y[i];
            min = grad[pos.x[i]][pos.y[i]];
        }
    }


    return res;
}



// --------------------------------------------------------------------------------------------
// FUNCIONES DE GRAFOS
// --------------------------------------------------------------------------------------------

vector<int> find_graph_centroid(vector<vector<bool>> adj)
{
    vector<int> res;
    // Sacar la conectividad
    // Creo que los que tengan menor conectividad son los de los extremos
    vector<int> conectividad = node_connectivity(adj);
    int n=adj.size();
    queue<int> q;
    for(int i=0; i<n; i++){

    }

    return res;
} // Calcular el/los centroid/es de un grafo

// --------------------------------------------------------------------------------------------

// ======================================================================
// ======================================================================
// ======================================================================
// Tonterias para guardar
// ======================================================================
// ======================================================================
// ======================================================================

void save_path(Path path, int red_fact, string name)
{
    for(int j=0; j<path.tam; j++){
        path.x[j]/=red_fact;
        path.y[j]/=red_fact;
    }
    path.save(name.c_str());
}
    
void save_paths(vector<Path> paths, int red_fact, string name)
{
    for(int i=0; i<paths.size(); i++){
        for(int j=0; j<paths[i].tam; j++){
            paths[i].x[j]/=red_fact;
            paths[i].y[j]/=red_fact;
        }
        paths[i].save((name+to_string(i)+".txt").c_str());
    }
}

void save_poss(Poss<int> pos, int red_fact, string name)
{
    for(int j=0; j<pos.x.size(); j++){
        pos.x[j]/=red_fact;
        pos.x[j]/=red_fact;
    }
    pos.save(name.c_str());
}

void save_poss(vector<Poss<int>> pos, int red_fact, string name)
{
    for(int i=0; i<pos.size(); i++){
        for(int j=0; j<pos[i].x.size(); j++){
            pos[i].x[j]/=red_fact;
            pos[i].x[j]/=red_fact;
        }
        pos[i].save((name+to_string(i)+".txt").c_str());
    }
    
}

void save_poss(vector<vector<Poss<int>>> pos, int red_fact, string name)
{
    for(int i=0; i<pos.size(); i++){
        for(int j=0; j<pos[i].size(); j++){
            for(int k=0; k<pos[i][j].x.size(); k++){
                pos[i][j].x[k]/=red_fact;
                pos[i][j].x[k]/=red_fact;
            }
            pos[i][j].save((name+to_string(i)+"_"+to_string(j)+".txt").c_str());
        }
    }
    
}
    