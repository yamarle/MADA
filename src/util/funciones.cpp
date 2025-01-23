#include <mada/util/funciones.hpp>

Poss<int> agentPoss(int x, int y, int sx, int sy)
{
    Poss<int> res;
    for(int i=x-sx/2; i<=x+sx/2; i++)
        for(int j=y-sy/2; j<=y+sy/2; j++)
            res.push(i,j);
    return res; 
}

float distance2closest_point(int x, int y, Poss<int> points)
{
    float res = INFINITY, d;
    for(int i=0; i<points.x.size(); i++){
        d = hypot(x - points.x[i], y - points.y[i]);
        if(res > d) res = d;
    }
    return res;
}

Poss<int> random_positions(vector<vector<float> > grid, int n){
	Poss<int> res; res(n);
	int sx = grid.size(), sy = grid[0].size();
	for(int i=0; i<n; i++){
		res.x[i] = (int(rand())/INF)*sx-1;
		res.y[i] = (int(rand())/INF)*sy-1;
		while(grid[res.x[i]][res.y[i]]==0){
			res.x[i] = (int(rand())/INF)*sx-1;
			res.y[i] = (int(rand())/INF)*sy-1;
		}
		grid[res.x[i]][res.y[i]]=0;
	}
	return res;
}

Poss<int> bounded_random_positions(vector<vector<float> > grid, int n, int xlb, int xub, int ylb, int yub){
	Poss<int> res; res(n);
	int sx = grid.size(), sy = grid[0].size();
	for(int i=0; i<n; i++){
		res.x[i] = (int(rand())/INF)*(xub-xlb)+xlb;
		res.y[i] = (int(rand())/INF)*(yub-ylb)+ylb;
		while(grid[res.x[i]][res.y[i]]==0){
			res.x[i] = (int(rand())/INF)*(xub-xlb)+xlb;
			res.y[i] = (int(rand())/INF)*(yub-ylb)+ylb;
		}
		grid[res.x[i]][res.y[i]]=0;
	}
	return res;
}

void inflate_pos(int x, int y, vector<vector<float>> &map, int srange)
{
	int sx = map.size(), sy = map[0].size();

	int xl, xu, yl, yu;
	xl = (x-srange) > 0 ? (x-srange) : 0;
	xu = (x+srange) < sx-1 ? (x+srange) : sx-1;
	yl = (y-srange) > 0 ? (y-srange) : 0;
	yu = (y+srange) < sy-1 ? (y+srange) : sy-1;

	for(int j=xl; j<=xu; j++){
        for(int k=yl; k<=yu; k++){
            map[j][k] = 0;
        }
    }

}

void inflate_pos(int x, int y, vector<vector<float>> &map, int srange, float val)
{
	int sx = map.size(), sy = map[0].size();

	int xl, xu, yl, yu;
	xl = (x-srange) > 0 ? (x-srange) : 0;
	xu = (x+srange) < sx-1 ? (x+srange) : sx-1;
	yl = (y-srange) > 0 ? (y-srange) : 0;
	yu = (y+srange) < sy-1 ? (y+srange) : sy-1;

	for(int j=xl; j<=xu; j++){
        for(int k=yl; k<=yu; k++){
            map[j][k] = val;
        }
    }

}

void inflate_pos_iv(int x, int y, vector<vector<float>> &map, int srange, float val)
{
	int sx = map.size(), sy = map[0].size();

	int xl, xu, yl, yu;
	xl = (x-srange) > 0 ? (x-srange) : 0;
	xu = (x+srange) < sx-1 ? (x+srange) : sx-1;
	yl = (y-srange) > 0 ? (y-srange) : 0;
	yu = (y+srange) < sy-1 ? (y+srange) : sy-1;

	for(int j=xl; j<=xu; j++){
        for(int k=yl; k<=yu; k++){
        	if(map[j][k]) map[j][k] = val; // únicamente en las posiciones libres
        }
    }

}

Poss<int> inflated_poss(Poss<int> pos, int srange, int sx, int sy)
{
	Poss<int> res;
	int xl, xu, yl, yu;
	for(int i=0; i<pos.x.size(); i++){
		xl = (pos.x[i]-srange) > 0 ? (pos.x[i]-srange) : 0;
		xu = (pos.x[i]+srange) < sx-1 ? (pos.x[i]+srange) : sx-1;
		yl = (pos.y[i]-srange) > 0 ? (pos.y[i]-srange) : 0;
		yu = (pos.y[i]+srange) < sy-1 ? (pos.y[i]+srange) : sy-1;

		for(int j=xl; j<=xu; j++){
	        for(int k=yl; k<=yu; k++){
	            res.x.push_back(j);
	            res.y.push_back(k);
	        }
	    }
	}
	return res;
}


Poss<int> inflated_poss(Poss<int> pos, vector<int> srange, int sx, int sy)
{
	Poss<int> res;
	int xl, xu, yl, yu;
	for(int i=0; i<pos.x.size(); i++){
		xl = (pos.x[i]-srange[i]) > 0 ? (pos.x[i]-srange[i]) : 0;
		xu = (pos.x[i]+srange[i]) < sx-1 ? (pos.x[i]+srange[i]) : sx-1;
		yl = (pos.y[i]-srange[i]) > 0 ? (pos.y[i]-srange[i]) : 0;
		yu = (pos.y[i]+srange[i]) < sy-1 ? (pos.y[i]+srange[i]) : sy-1;

		for(int j=xl; j<=xu; j++){
	        for(int k=yl; k<=yu; k++){
	            res.x.push_back(j);
	            res.y.push_back(k);
	        }
	    }
	}
	return res;
}

Poss<int> generate_poss_with_srange(vector<vector<float> > grid, int n, int srange){
	Poss<int> res;
	int sx = grid.size(), sy = grid[0].size();
	int x, y;
	for(int i=0; i<n; i++){
		x=(grid.size()-2)*(float)rand()/RAND_MAX+1;
		y=(grid[0].size()-2)*(float)rand()/RAND_MAX+1;
		while(!grid[x][y]){
			x=(grid.size()-2)*(float)rand()/RAND_MAX+1;
			y=(grid[0].size()-2)*(float)rand()/RAND_MAX+1;
		}
		inflate_pos(x, y, grid, srange);
		res.x.push_back(x); res.y.push_back(y);
	}
	return res;
}

Poss<int> generate_poss_with_srange(vector<vector<float> > grid, vector<int> srange){
	Poss<int> res;
	int sx = grid.size(), sy = grid[0].size();
	int x, y;
	for(int i=0; i<srange.size(); i++){
		x=(grid.size()-2)*(float)rand()/RAND_MAX+1;
		y=(grid[0].size()-2)*(float)rand()/RAND_MAX+1;
		while(!grid[x][y]){
			x=(grid.size()-2)*(float)rand()/RAND_MAX+1;
			y=(grid[0].size()-2)*(float)rand()/RAND_MAX+1;
		}
		inflate_pos(x, y, grid, srange[i]);
		res.x.push_back(x); res.y.push_back(y);
	}
	return res;
}

Poss<int> generate_bounded_poss_with_srange(vector<vector<float> > grid, int n, int srange, int xlb, int xub, int ylb, int yub){
	Poss<int> res;
	int sx = grid.size(), sy = grid[0].size();
	int x, y;
	// Asegurar que no cae fuera de los límited del mapa
	xlb = (xlb-srange) > 0 ? (xlb-srange) : 0;
	xub = (xub+srange) < sx-1 ? (xub+srange) : sx-1;
	ylb = (ylb-srange) > 0 ? (ylb-srange) : 0;
	yub = (yub+srange) < sy-1 ? (yub+srange) : sy-1;
	for(int i=0; i<n; i++){
		x = (int(rand())/INF)*(xub-xlb)+xlb;
		y = (int(rand())/INF)*(yub-ylb)+ylb;
		while(!grid[x][y]){
			x = (int(rand())/INF)*(xub-xlb)+xlb;
			y = (int(rand())/INF)*(yub-ylb)+ylb;
		}
		inflate_pos(x, y, grid, srange);
		res.x.push_back(x); res.y.push_back(y);
	}
	return res;
}


Poss<int> circle_poss(int x, int y, int srange, int sx, int sy)
{   
    // Reducir los puntos que se van a evaluar
    int xl, xu, yl, yu;
    xl = (x-srange) > 0 ? (x-srange) : 0;
    xu = (x+srange) < sx-1 ? (x+srange) : sx-1;
    yl = (y-srange) > 0 ? (y-srange) : 0;
    yu = (y+srange) < sy-1 ? (y+srange) : sy-1;
    Poss<int> res;
    for(int i=xl; i<=xu; i++){
        for(int j=yl; j<=yu; j++){
            if(sqrt(pow(x-i,2)+pow(y-j,2))<=srange){ // Dentro del rango
                res.x.push_back(i);
                res.y.push_back(j);
            }
        }
    }
    return res;
} // Todas las posiciones del círculo sobre el grid

Poss<float> circle_poss(float x, float y, float radius, float dt)
{
	Poss<float> res;
	for(float ang=0; ang<=2*M_PI; ang+=dt){
		res.x.push_back((float)x + radius * cos(ang));
		res.y.push_back((float)y + radius * sin(ang));
	}
	return res;
} // Solo el contorno

Poss<int> compute_centroid(Poss<int> pos)
{
	Poss<int> res; res(1);
	for(int i=0; i<pos.x.size(); i++){
		res.x[0] += pos.x[i]; res.y[0] += pos.y[i];
	}
	res.x[0]/=pos.x.size(); res.y[0]/=pos.y.size();
	return res;
}

Poss<int> compute_gradient_centroids(vector<vector<float>> obst_grad, int cell_size)
{
    Poss<int> res;

    int x,y, sx, sy;
    sx = obst_grad.size(); sx = obst_grad.size();
    float val;

    while(true){
        val = 0;
        for(int i=0; i<obst_grad.size(); i++)
            for(int j=0; j<obst_grad[0].size(); j++)
                if(obst_grad[i][j]>cell_size && val<obst_grad[i][j]){
                    x = i; y = j;
                    val = obst_grad[i][j];
                }
        if(val){
            res.x.push_back(x); res.y.push_back(y);
            for(int i=x-val; i<=x+val; i++){
                for(int j=y-val; j<=y+val; j++){
                    obst_grad[i][j] = 0;
                }
            }
        }else{
            break;
        }
    }

    return res;   
}

vector<vector<float> > random_obs(int x, int y, float dens){
	// crear grid
	vector<vector<float> > grid;
	grid.resize(x);
	int i;
	for(i=0; i<x; i++) grid[i].resize(y);
	// número de obstáculos a crear
	//int nobs=(int) (float) x* (float) y*dens;
	int nobs=x*y*dens; // lo hace por defecto
	int x_,y_;
	while(nobs>0){
		x_=x*(float)rand()/RAND_MAX;
		y_=y*(float)rand()/RAND_MAX;
		if (!grid[x_][y_]){
			grid[x_][y_]=1;
			nobs--;
		}
	}
	return grid;
} // generar un grid aleatorio con cierta densidad de obstáculos

Poss<int> pos_obs(vector<vector<float> > grid){
	Poss<int> res;
	for(int i=0; i<grid.size(); i++)
		for(int j=0; j<grid[i].size(); j++)
			if(!grid[i][j]){
				res.x.push_back(i);
				res.y.push_back(j);
			}
	return res;
} // posiciones donde hay obstáculos

Poss<int> pos_obs_wl(vector<vector<float> > grid){
	Poss<int> res;
	for(int i=1;i<grid.size()-1;i++)
		for(int j=1; j<grid[i].size()-1; j++)
			if(!grid[i][j]){
				res.x.push_back(i);
				res.y.push_back(j);
			}
	return res;
} // posiciones donde hay obstáculos

void invertir_obs(vector<vector<float> > &grid){
	for(int i=0;i<grid.size();i++){
		for(int j=0;j<grid[1].size();j++){
			grid[i][j]=grid[i][j]>0?0:1;
			//grid[i][j]=!grid[i][j];
		}
	}
}

void gen_pos(vector<vector<float> > grid, int& x, int& y){
	x=(grid.size()-2)*(float)rand()/RAND_MAX+1;
	y=(grid[0].size()-2)*(float)rand()/RAND_MAX+1;
	int cont = 0;
	while(!grid[x][y]){
		x=(grid.size()-2)*(float)rand()/RAND_MAX+1;
		y=(grid[0].size()-2)*(float)rand()/RAND_MAX+1;
		if(!grid[x][y]) cont++;
		else cont = 0;
		if(cont > grid.size()*grid.size()) break;
	}
} // generar una posición aleatoria, fuera de los obstáculos

bool repe(vector<Pos<int> > pos){
	int s=pos.size();
	for (int i=0; i<s; i++)
		for(int j=0; j<s; j++)
			if(i!=j && pos[i].x==pos[j].x && pos[i].x==pos[j].x) return true;
	return false;
} // Si se repite unza posición

vector<Pos<int> > gen_pos(vector<vector<float> > grid, int n){
	vector<Pos<int> > res(n);
	for(int i=0; i<n; i++){
		gen_pos(grid,res[i].x,res[i].y);
		grid[res[i].x][res[i].y]=0; // HAS CAMBIADO ESTO (CREO QUE OBSTÁCULO ES "0")
	}
	return res;
} // generar posiciones aleatorias, fuera de los obstáculos

Poss<int> gen_pos2(vector<vector<float> > grid, int n){
	Poss<int> res;
	res.x.resize(n);
	res.y.resize(n);
	for(int i=0; i<n; i++){
		gen_pos(grid,res.x[i],res.y[i]);
		grid[res.x[i]][res.y[i]]=0; // HAS CAMBIADO ESTO (CREO QUE OBSTÁCULO ES "0")
	}
	return res;
} // generar posiciones aleatorias, fuera de los obstáculos

bool check_dist(int x, int y, vector<int> xv, vector<int> yv, float d){
	for (int i=0; i<xv.size(); i++){
		if(sqrt(pow(x-xv[i],2)+pow(y-yv[i],2))<d) return true;
	}
	return false;
}

Poss<int> gen_pos_dist(vector<vector<float> > grid, int n, float dist){
	Poss<int> res;
	int x,y;
	float d;
	for(int i=0; i<n; i++){
		gen_pos(grid,x,y);
		while (check_dist(x,y,res.x,res.y,dist)) gen_pos(grid,x,y);
		grid[x][y]=1;
		res.x.push_back(x); res.y.push_back(y);
	}
	return res;
} // generar posiciones aleatorias, fuera de los obstáculos

vector<Poss<int>> grid_positions(vector<vector<float> > grid){
	vector<Poss<int>> res(2); // (0: libre, 1: obstáculos)
	for(int i=0; i<grid.size(); i++){
		for(int j=0; j<grid[0].size(); j++){
			if(grid[i][j]){
				res[0].push(i,j); // Libre
			}else{
				res[1].push(i,j); // Obstáculo
			}
		}
	}
	return res;
} // Posiciones libres y con obstáculos

vector<Poss<int>> grid_positions(vector<vector<float> > grid, vector<vector<float> > reference_grad){
	vector<Poss<int>> res(2); // (0: libre, 1: obstáculos)
	for(int i=0; i<grid.size(); i++){
		for(int j=0; j<grid[0].size(); j++){
			if(grid[i][j]){
				if(reference_grad[i][j] < INF)
					res[0].push(i,j); // Libre
			}else{
				res[1].push(i,j); // Obstáculo
			}
		}
	}
	return res;
} // Posiciones libres y con obstáculos

vector<Poss<int>> fix_free_space(vector<vector<float> > &grid, vector<vector<float> > reference_grad){
	vector<Poss<int>> res(2); // (0: libre, 1: obstáculos)
	for(int i=0; i<grid.size(); i++){
		for(int j=0; j<grid[0].size(); j++){
			if(grid[i][j]){
				if(reference_grad[i][j] < INF)
					res[0].push(i,j); // Libre
				else{
					grid[i][j] = 0;
					res[1].push(i,j); // Obstáculo
				}
			}else{
				res[1].push(i,j); // Obstáculo
			}
		}
	}
	return res;
} // Posiciones libres y con obstáculos

vector<Poss<int>> fix_free_space_(vector<vector<float> > &grid, vector<vector<float> > reference_grad){
	vector<Poss<int>> res(2); // (0: libre, 1: obstáculos)
	for(int i=0; i<grid.size(); i++){
		for(int j=0; j<grid[0].size(); j++){
			if(grid[i][j]){
				if(reference_grad[i][j] < INF && reference_grad[i][j] != 0)
					res[0].push(i,j); // Libre
				else{
					grid[i][j] = 0;
					res[1].push(i,j); // Obstáculo
				}
			}else{
				res[1].push(i,j); // Obstáculo
			}
		}
	}
	return res;
} // Posiciones libres y con obstáculos

//========================================================================================================
//========================================================================================================
//========================================================================================================

Poss<int> load_poss(const char* filename){
	Poss<int> res;
	string line, v;
	ifstream file(filename); // abrir para leer
	int value;
	if(file.is_open()){
		getline(file,line); // la primera lines lleva poss=[
		getline(file,line);
		for(int i=3; i<line.size(); i++){
			if(line[i]==' '){
				value=stoi(v);
				v.clear();
				res.x.push_back(value);
			}
			v+=line[i];
		}
		v.clear(); line.clear();
		getline(file,line);
		for(int i=3; i<line.size(); i++){
			if(line[i]==' '){
				value=stoi(v);
				v.clear();
				res.y.push_back(value);
			}
			v+=line[i];
		}
	}
	return res;
}

vector<vector<float> > load_grid(const char* filename){
	vector<vector<float> > grid;
	string line; // línea y número actual
	float num; // numero actual
	int ic,fc; // contadores de las líneas
	int sx=0,sy=0; // tamaño del grid
	ifstream file(filename); // abrir para leer
	if(file.is_open()){
		getline(file,line);getline(file,line); // las dos primeras línes a la basura (pos si lees un escenario como para FMM)
		// grid
		int i;
		if(getline(file,line)){ // esta linea se descarta
			while(getline(file,line) && line[0]!=']'){
				if(!sy){
					// La primera línea se utiliza para obtener el tamaño del grid en x y redimensionar la matriz del grid
					for(int i=0;i<line.size();i++){
						if(line[i]==' ') sx++;
					}
					grid.resize(sx);
				}
				i=0;
				ic=fc=0;
				while (fc<line.size()){
					if(line[fc]!=' ') fc++;
					else{
						grid[i].push_back(stod(line.substr(ic,fc)));
						i++; // donde guardo
						ic=fc;
						fc++;
					}
				}
				sy++;
			} // recorrer todo el grid
		}
		file.close();
	}else cout<<"No se puede abrir el fichero"<<endl;
	return grid;
} // cargar ps, pd y mapa

vector<vector<float> > load_matr(const char* filename){
	vector<vector<float> > grid;
	string line; // línea y número actual
	float num; // numero actual
	int ic,fc; // contadores de las líneas
	int sx=0,sy=0; // tamaño del grid
	ifstream file(filename); // abrir para leer
	if(file.is_open()){
		// grid
		int i;
		if(getline(file,line)){ // esta linea se descarta
			while(getline(file,line) && line[0]!=']'){
				if(!sy){
					// La primera línea se utiliza para obtener el tamaño del grid en x y redimensionar la matriz del grid
					for(int i=0; i<line.size(); i++){
						if(line[i]==' ') sx++;
					}
					grid.resize(sx);
				}
				i=0;
				ic=fc=0;
				while (fc<line.size()){
					if(line[fc]!=' ') fc++;
					else{
						grid[i].push_back(stod(line.substr(ic,fc)));
						i++; // donde guardo
						ic=fc;
						fc++;
					}
				}
				sy++;
			} // recorrer todo el grid
		}
		file.close();
	}else cout<<"No se puede abrir el fichero"<<endl;
	return grid;
} // cargar ps, pd y mapa

vector<vector<float> > red_res(vector<vector<float> > grid, int res){    
    if(res%2){
        return grid;
    }
    int sx=grid.size(), sy=grid[0].size();
    int sx_=sx/res, sy_=sy/res; // tamaño del nuevo grid
    vector<vector<float> > grid_(sx_,vector<float>(sy_,1));
    int npp=pow(res+1,2);
    int nobst;
    for(int i=0; i<sx_; i++){
        for(int j=0; j<sy_; j++){
        	nobst=0;
            // Comprobar si en el superpíxel del grid original hay obstáculos
            for(int k=(i*res+1)-res/2; k<(i*res+1)+res/2; k++){
            	for(int l=(j*res+1)-res/2; l<(j*res+1)+res/2; l++){
            		if(k>=0 && k<sx && l>=0 && l<sy){
            			if(!grid[k][l]) nobst++;
            		}
            	}
            }
            //cout<<nobst<<endl;
            //if(nobst>=npp/2) grid_[i][j]=0;
            if(nobst) grid_[i][j]=0;
        }
    }
    return grid_;
} // Reducir la resolución del grid, le paso la resolución que quiero

vector<vector<int> > red_res(vector<vector<int> > grid, int res){    
    if(res%2){
        return grid;
    }
    int sx=grid.size(), sy=grid[0].size();
    int sx_=sx/res, sy_=sy/res; // tamaño del nuevo grid
    vector<vector<int> > grid_(sx_,vector<int>(sy_,1));
    int npp=pow(res+1,2);
    int nobst;
    for(int i=0; i<sx_; i++){
        for(int j=0; j<sy_; j++){
        	nobst=0;
            // Comprobar si en el superpíxel del grid original hay obstáculos
            for(int k=(i*res+1)-res/2; k<(i*res+1)+res/2; k++){
            	for(int l=(j*res+1)-res/2; l<(j*res+1)+res/2; l++){
            		if(k>=0 && k<sx && l>=0 && l<sy){
            			if(!grid[k][l]) nobst++;
            		}
            	}
            }
            //cout<<nobst<<endl;
            //if(nobst>=npp/2) grid_[i][j]=0;
            if(nobst) grid_[i][j]=0;
        }
    }
    return grid_;
} // Reducir la resolución del grid, le paso la resolución que quiero


vector<vector<float> > red_res_(vector<vector<float> > grid, int &res){
    int sx=grid.size(), sy=grid[0].size();
    int sx_=sx, sy_=sy;
    res=0; // Reiniciar, por si acaso
    while(sx_>250 || sy_>250){
    	res+=2;
    	sx_=sx/res; sy_=sy/res;
    }
    //cout<<"Tamaño original: "<<sx<<"x"<<sy<<endl;
    //cout<<"Tamaño nuevo: "<<sx_<<"x"<<sy_<<endl;
    vector<vector<float> > grid_(sx_,vector<float>(sy_,1));
    if(!res){
    	//cout<<"La resolución no se reajusta"<<endl;
    	res=1;
    	return grid;
    }
    int npp=pow(res+1,2);
    int nobst;
    for(int i=0; i<sx_; i++){
        for(int j=0; j<sy_; j++){
        	nobst=0;
            // Comprobar si en el superpíxel del grid original hay obstáculos
            for(int k=(i*res+1)-res/2; k<(i*res+1)+res/2; k++){
            	for(int l=(j*res+1)-res/2; l<(j*res+1)+res/2; l++){
            		if(k>=0 && k<sx && l>=0 && l<sy){
            			if(!grid[k][l]) nobst++;
            		}
            	}
            }
            if(nobst>=npp/2) grid_[i][j]=0;
        }
    }
    return grid_;
} // Reducir la resolución del grid, se encuentra la resolución 

vector<vector<float> > red_res__(vector<vector<float> > grid, int &res){
    int sx=grid.size(), sy=grid[0].size();
    if(sx<=250 && sy<=250){
    	res=1;
    	return grid;
    }
    int sx_=sx, sy_=sy;
    res=0; // Reiniciar, por si acaso
    while(sx_>250 || sy_>250){
    	res+=2;
    	sx_=sx/res; sy_=sy/res;
    }
    vector<vector<float> > grid_(sx_,vector<float>(sy_,1)); // 1 es espacio libre
    int nobst;
    for(int i=0; i<sx_; i++){
        for(int j=0; j<sy_; j++){
        	nobst=0;
            // Comprobar si en el superpíxel del grid original hay obstáculos
            for(int k=(i*res+1)-res/2; k<(i*res+1)+res/2; k++){
            	for(int l=(j*res+1)-res/2; l<(j*res+1)+res/2; l++){
            		if(k>=0 && k<sx && l>=0 && l<sy){
            			if(!grid[k][l]) nobst++;
            		}
            	}
            }
            // Si existe algun obstáculo dentro del píxel, es un obstáculo
            if(nobst) grid_[i][j]=0;
        }
    }
    return grid_;
} // Lo mismo que la anterior, pero con que haya un obstáculo dentro del píxel, todo será obstáculo

vector<vector<float>> reverse_gradient(vector<vector<float>> grad)
{
    float max = 0;
    int sx = grad.size(), sy = grad[0].size();
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grad[i][j]<INF && max < grad[i][j]){
                max = grad[i][j];
            }
        }
    }

    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grad[i][j]<INF){
                grad[i][j] = (max - grad[i][j]);
            }
        }
    }
    return grad;   
}

Poss<int> generate_poss_with_sranges(vector<vector<float> > grid, vector<int> sranges){
    Poss<int> res;
    Poss<int> aux, infl;
    aux(1);
    int sx = grid.size(), sy = grid[0].size();
    int x, y;
    bool search = true;
    for(int i=0; i<sranges.size(); i++){        
        while(search){
            // Generar posicón aleatoria
            x=(sx-2)*(float)rand()/RAND_MAX+1;
            y=(sy-2)*(float)rand()/RAND_MAX+1;
            // Inflar
            aux.x[0] = x; aux.y[0] = y;
            infl = inflated_poss(aux, vector<int>(1,sranges[i]), sx, sy);
            // Comprobar que no invade ningún obstáculo
            search = false;
            for(int j=0; j<infl.x.size(); j++){
                if(!grid[infl.x[j]][infl.y[j]]){
                    search = true;
                    break;
                }
            }
        }
        search = true;
        // Fijar como un  obstáculo
        for(int j=0; j<infl.x.size(); j++)
            grid[infl.x[j]][infl.y[j]] = 0;
        // Insertar el punto nuevo
        res.x.push_back(x); res.y.push_back(y);
    }
    return res;
} // Generar unas posiciones para que no "invadan" ningún obstáculo

vector<Poss<int>> generate_infl_poss_with_sranges(vector<vector<float> > grid, vector<int> sranges){
    vector<Poss<int>> res;
    Poss<int> aux, infl;
    aux(1);
    int sx = grid.size(), sy = grid[0].size();
    int x, y;
    bool search = true;
    for(int i=0; i<sranges.size(); i++){        
        while(search){
            // Generar posicón aleatoria
            x=(sx-2)*(float)rand()/RAND_MAX+1;
            y=(sy-2)*(float)rand()/RAND_MAX+1;
            // Inflar
            aux.x[0] = x; aux.y[0] = y;
            infl = inflated_poss(aux, vector<int>(1,sranges[i]), sx, sy);
            // Comprobar que no invade ningún obstáculo
            search = false;
            for(int j=0; j<infl.x.size(); j++){
                if(!grid[infl.x[j]][infl.y[j]]){
                    search = true;
                    break;
                }
            }
        }
        search = true;
        // Fijar como un  obstáculo
        for(int j=0; j<infl.x.size(); j++)
            grid[infl.x[j]][infl.y[j]] = 0;
        // Insertar el punto nuevo
        res.push_back(infl);
    }
    return res;
} // Generar unas posiciones para que no "invadan" ningún obstáculo (Devuelve las posiciones infladas)

void generate_infl_poss_with_sranges(vector<vector<float> > grid, vector<int> sranges, Poss<int> &pos, vector<Poss<int>> &infl)
{
    // Redimensionar
    pos(sranges.size());
    infl.resize(sranges.size());

    Poss<int> aux;
    aux(1);

    int sx = grid.size(), sy = grid[0].size();
    int x, y;
    bool search = true;
    for(int i=0; i<sranges.size(); i++){        
        while(search){
            // Generar posicón aleatoria
            x=(sx-2)*(float)rand()/RAND_MAX+1;
            y=(sy-2)*(float)rand()/RAND_MAX+1;
            // Inflar
            aux.x[0] = x; aux.y[0] = y;
            infl[i] = inflated_poss(aux, vector<int>(1,sranges[i]), sx, sy);
            // Comprobar que no invade ningún obstáculo
            search = false;
            for(int j=0; j<infl[i].x.size(); j++){
                if(!grid[infl[i].x[j]][infl[i].y[j]]){
                    search = true;
                    break;
                }
            }
        }
        search = true;
        // Fijar como un  obstáculo
        for(int j=0; j<infl[i].x.size(); j++)
            grid[infl[i].x[j]][infl[i].y[j]] = 0;
        pos.x[i] = x; pos.y[i] = y;
    }
    
} // Generar unas posiciones para que no "invadan" ningún obstáculo (Devuelve las dos: las posiciones y los inflados)

vector<vector<float>> reverse_gradient(vector<vector<float>> grad, vector<vector<float>> grid)
{
	float max = 0, min = INF;
    int sx = grad.size(), sy = grad[0].size();
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grid[i][j]){
            	if(max < grad[i][j])
                	max = grad[i][j];
                if(min > grad[i][j])
                	min = grad[i][j];
            }
        }
    }

    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grid[i][j]){
            	if(grad[i][j]<INF)
                	grad[i][j] = (max - grad[i][j]);
                else
                	grad[i][j] = (max - min);
            }
        }
    }
    return grad; 
}

vector<vector<float>> reverse_and_normalize_gradient(vector<vector<float>> grad)
{
    float max = 0;
    int sx = grad.size(), sy = grad[0].size();
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grad[i][j]<INF && max < grad[i][j]){
                max = grad[i][j];
            }
        }
    }

    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grad[i][j]<INF){
                grad[i][j] = (max - grad[i][j])/max;
            }
        }
    }
    return grad;   
}

vector<vector<float>> reverse_and_normalize_gradient(vector<vector<float>> grad, vector<vector<float>> grid)
{
	float max = 0, min = INF;
    int sx = grad.size(), sy = grad[0].size();
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grid[i][j]){
            	if(max < grad[i][j])
                	max = grad[i][j];
                if(min > grad[i][j])
                	min = grad[i][j];
            }
        }
    }

    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grid[i][j]){
            	if(grad[i][j]<INF)
                	grad[i][j] = (max - grad[i][j])/max;
                else
                	grad[i][j] = (max - min)/max;
            }
        }
    }
    return grad; 
}

vector<vector<float>> reverse_and_normalize_gradient(vector<vector<float>> grad, float val)
{
    float max = 0;
    int sx = grad.size(), sy = grad[0].size();
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grad[i][j]<INF && max < grad[i][j]){
                max = grad[i][j];
            }
        }
    }

    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grad[i][j]<INF){
                grad[i][j] = val*(max - grad[i][j])/max;
            }
        }
    }
    return grad;   
}

vector<vector<float>> reverse_and_normalize_gradient(vector<vector<float>> grad, vector<vector<float>> grid, float val)
{
	float max = 0, min = INF;
    int sx = grad.size(), sy = grad[0].size();
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grid[i][j]){
            	if(max < grad[i][j])
                	max = grad[i][j];
                if(min > grad[i][j])
                	min = grad[i][j];
            }
        }
    }

    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            if(grid[i][j]){
            	if(grad[i][j]<INF)
                	grad[i][j] = val*(max - grad[i][j])/max;
                else
                	grad[i][j] = val*(max - min)/max;
            }
        }
    }
    return grad; 
}

vector<vector<float>> mapa_obstaculos_inflados(vector<vector<float>> static_grid, Poss<int> obst, int irange)
{
    int sx = static_grid.size(), sy = static_grid[0].size();
    int xl, xu, yl, yu;
    for(int i=0; i<obst.x.size(); i++){
        xl = (obst.x[i]-irange) > 0 ? (obst.x[i]-irange) : 0;
        xu = (obst.x[i]+irange) < sx-1 ? (obst.x[i]+irange) : sx-1;
        yl = (obst.y[i]-irange) > 0 ? (obst.y[i]-irange) : 0;
        yu = (obst.y[i]+irange) < sy-1 ? (obst.y[i]+irange) : sy-1;
        for(int j = xl; j <= xu; j++){
            for(int k = yl; k <= yu; k++){
                static_grid[j][k] = 0;
            }
        }
    }
    return static_grid;
}

vector<bool> ind2bool(int s, vector<int> ind)
{
	vector<bool> res(s,false);
	for(int i=0; i<ind.size(); i++)
		if(ind[i]>=0 && ind[i]<s)
			res[ind[i]] = true;
	return res;
}

vector<int> bool2ind(vector<bool> ind)
{
	vector<int> res;
	for(int i=0; i<ind.size(); i++)
		if(ind[i]) res.push_back(i);
	return res;
}

vector<bool> invert_vector(vector<bool> v)
{
	for(int i=0; i<v.size(); i++)
		v[i] = !v[i];
	return v;
}

void accumulate_occupancy(vector<vector<float>> &occupancy_grid, vector<vector<float>> grid)
{
	if(occupancy_grid.size() != grid.size()) return;
	if(occupancy_grid[0].size() != grid[0].size()) return;
	for(int i=0; i<occupancy_grid.size(); i++)
		for(int j=0; j<occupancy_grid[0].size(); j++)
			if(!grid[i][j])
				occupancy_grid[i][j]++;
}

vector<vector<float>> accumulate_occupancy_(vector<vector<float>> occupancy_grid, vector<vector<float>> grid)
{
	if(occupancy_grid.size() != grid.size()) return occupancy_grid;
	if(occupancy_grid[0].size() != grid[0].size()) return occupancy_grid;
	for(int i=0; i<occupancy_grid.size(); i++)
		for(int j=0; j<occupancy_grid[0].size(); j++)
			if(!grid[i][j])
				occupancy_grid[i][j]++;
	return occupancy_grid;
}

void anular_radio_f(vector<vector<float>> &grid, int x, int y, float r)
{
	// Establecer los limites para no recorrer todo el grid
    int xl, xu, yl, yu;
    xl = (x-r) > 0 ? (x-r) : 0;
    xu = (x+r) < grid.size()-1 ? (x+r) : grid.size()-1;
    yl = (y-r) > 0 ? (y-r) : 0;
    yu = (y+r) < grid[0].size()-1 ? (y+r) : grid[0].size()-1;

    for(int i=xl; i<=xu; i++){
        for(int j=yl; j<=yu; j++){
            if(sqrt(pow(x-i,2)+pow(y-j,2))<=r)
                grid[i][j]=0;
        }
    }
}

void anular_radio_i(vector<vector<int>> &grid, int x, int y, float r)
{
	// Establecer los limites para no recorrer todo el grid
    int xl, xu, yl, yu;
    xl = (x-r) > 0 ? (x-r) : 0;
    xu = (x+r) < grid.size()-1 ? (x+r) : grid.size()-1;
    yl = (y-r) > 0 ? (y-r) : 0;
    yu = (y+r) < grid[0].size()-1 ? (y+r) : grid[0].size()-1;

    for(int i=xl; i<=xu; i++){
        for(int j=yl; j<=yu; j++){
            if(sqrt(pow(x-i,2)+pow(y-j,2))<=r)
                grid[i][j]=0;
        }
    }
}

void anular_rango_f(vector<vector<float>> &grid, int x, int y, float r)
{
	// Establecer los limites para no recorrer todo el grid
    int xl, xu, yl, yu;
    xl = (x-r) > 0 ? (x-r) : 0;
    xu = (x+r) < grid.size()-1 ? (x+r) : grid.size()-1;
    yl = (y-r) > 0 ? (y-r) : 0;
    yu = (y+r) < grid[0].size()-1 ? (y+r) : grid[0].size()-1;

    for(int i=xl; i<=xu; i++){
        for(int j=yl; j<=yu; j++){
            grid[i][j]=0;
        }
    }
}

void anular_rango_i(vector<vector<int>> &grid, int x, int y, float r)
{
	// Establecer los limites para no recorrer todo el grid
    int xl, xu, yl, yu;
    xl = (x-r) > 0 ? (x-r) : 0;
    xu = (x+r) < grid.size()-1 ? (x+r) : grid.size()-1;
    yl = (y-r) > 0 ? (y-r) : 0;
    yu = (y+r) < grid[0].size()-1 ? (y+r) : grid[0].size()-1;

    for(int i=xl; i<=xu; i++){
        for(int j=yl; j<=yu; j++){
            grid[i][j]=0;
        }
    }
}

void set_value_precisionf(float v, int p)
{
	v = roundf(v*(10^p))/(10^p);
}

void set_value_precisiond(double v, int p)
{
	v= round(v*(10^p))/(10^p);
}

void set_vector_precisionf(vector<float> &v, int p)
{
	for(int i=0; i<v.size(); i++)
		v[i] = roundf(v[i]*(10^p))/(10^p);
}

void set_vector_precisiond(vector<double> &v, int p)
{
	for(int i=0; i<v.size(); i++)
		v[i] = round(v[i]*(10^p))/(10^p);
}

void set_matrix_precisionf(vector<vector<float>> &m, int p)
{
	for(int i=0; i<m.size(); i++) set_vector_precisionf(m[i], p);
}

void set_matrix_precisiond(vector<vector<double>> &m, int p)
{
	for(int i=0; i<m.size(); i++) set_vector_precisiond(m[i], p);
}

float angle(float x1, float y1, float x2, float y2, float x3, float y3)
{
	float p12 = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
	float p13 = sqrt(pow(x1-x3,2)+pow(y1-y3,2));
	float p23 = sqrt(pow(x2-x3,2)+pow(y2-y3,2));
	return acos((pow(p12,2)+pow(p13,2)-pow(p23,2))/(2*p12*p13));	
}

void assign_value(Poss<int> poss, float val, vector<vector<float>> &grid)
{
    for(int i=0; i<poss.x.size(); i++)
        if(grid[poss.x[i]][poss.y[i]]) grid[poss.x[i]][poss.y[i]] = val;
} // Asignar un valor a las posiciones de un grid

void assign_values(Poss<int> poss, vector<vector<float>> val, vector<vector<float>> &grid)
{
    for(int i=0; i<poss.x.size(); i++)
        if(grid[poss.x[i]][poss.y[i]]) grid[poss.x[i]][poss.y[i]] = val[poss.x[i]][poss.y[i]];
} // Asignar un valor a las posiciones de un grid

void increase_value(Poss<int> poss, float val, vector<vector<float>> &grid)
{
    for(int i=0; i<poss.x.size(); i++)
        if(grid[poss.x[i]][poss.y[i]]) grid[poss.x[i]][poss.y[i]] += val;
} // Asignar un valor a las posiciones de un grid

void increase_values(Poss<int> poss, vector<vector<float>> val, vector<vector<float>> &grid)
{
    for(int i=0; i<poss.x.size(); i++)
        if(grid[poss.x[i]][poss.y[i]]) grid[poss.x[i]][poss.y[i]] += val[poss.x[i]][poss.y[i]];
} // Asignar un valor a las posiciones de un grid

bool eventTimeout(float &fixTime, float &time, float &_time, float currentTime)
{
    if(fixTime < 0){
        fixTime = currentTime;
        return false;
    }
    time -= (currentTime - fixTime);
    fixTime = currentTime;
    if(time <= 0){
        time = _time;
        fixTime = -1;
        return true;
    }
    return false;
}

bool check_pos_in_grid(int x, int y, int sx, int sy){
	if(x>=0 && x<sx && y>=0 && y<sy) return true;
	else return false;
}

// Guardado de las cosas
vector<int> iteration_data(Pos<int> agent, Pos<int> goal)
{
    vector<int> it_data;
    it_data.push_back(1);
    it_data.push_back(agent.x); it_data.push_back(agent.y);
    it_data.push_back(goal.x); it_data.push_back(goal.y);
    return it_data;
} // Formar el vector para cada iteración con: las posiciones de los agetnes, sus objetivos y el destino que llevan

vector<int> iteration_data(Poss<int> agents_poss, Poss<int> goals_poss)
{
    vector<int> it_data;
    it_data.push_back(agents_poss.x.size());
    for(int i=0; i<agents_poss.x.size(); i++) it_data.push_back(agents_poss.x[i]);
    for(int i=0; i<agents_poss.y.size(); i++) it_data.push_back(agents_poss.y[i]);
    for(int i=0; i<goals_poss.x.size(); i++) it_data.push_back(goals_poss.x[i]);
    for(int i=0; i<goals_poss.x.size(); i++) it_data.push_back(goals_poss.y[i]);
    return it_data;
} // Formar el vector para cada iteración con: las posiciones de los agetnes, sus objetivos y el destino que llevan


vector<vector<float>> matrInt2Float(vector<vector<int>> matr)
{
	vector<vector<float>> res(matr.size(), vector<float>(matr[0].size()));

	for(int i=0; i<res.size(); i++)
		for(int j=0; j<res[i].size(); j++)
			res[i][j] = (float)matr[i][j];

	return res;
}

vector<vector<float>> matrBool2Float(vector<vector<bool>> matr)
{
	vector<vector<float>> res(matr.size(), vector<float>(matr[0].size()));

	for(int i=0; i<res.size(); i++)
		for(int j=0; j<res[i].size(); j++)
			res[i][j] = (float)matr[i][j];

	return res;
}


