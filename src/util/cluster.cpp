#include <mada/util/cluster.hpp>

// ***********************************************************************************************************
// 												MEAN-SHIFT
// ***********************************************************************************************************

vector<vector<float>> mean_shift::simple_movement(vector<vector<float>> points, vector<vector<float>> grid, float radio)
{
	vector<vector<float>> centroids;
    int ncentroids = 0;

    int sx = grid.size(), sy = grid[0].size();

    // Posicionar los centroides
    float xc=0, yc=0, cont=0;
    vector<float> aux(2);
    while(xc<sx){
        while(yc<sy){
            if(grid[xc][yc]){ // Espacio libre
                aux[0] = xc; aux[1] = yc;
                centroids.push_back(aux);
                ncentroids++;
            }
            yc+=radio;
        }
        xc+=radio;
        yc = 0;
    }

    // Mover los centroides hacia la media de los puntos que caen en el radio (con o sin ponderación)
    bool it = true;
    float xm, ym;
    int nodes = points.size();
    while(it){
        it = false;
        for(int i=0; i<ncentroids; i++){
            cont = 0;
            for(int j=0; j<nodes; j++){
                if(sqrt(pow(centroids[i][0]-points[j][0],2)+pow(centroids[i][1]-points[j][1],2))<=radio){
                    xm+=points[j][0];
                    ym+=points[j][1];
                    cont++;
                }
            }
            if(cont){
                xm/=(float)cont;
                ym/=(float)cont;
                if(centroids[i][0]!=xm && centroids[i][1]!=ym && grid[xm][ym]){
                    centroids[i][0]=xm;
                    centroids[i][1]=ym;
                    it = true;
                }
            }
            xm = 0; ym = 0;
        }
    }

	return centroids;
}


vector<vector<vector<float>>> mean_shift::simple_movement_it(vector<vector<float>> points, vector<vector<float>> grid, float radio)
{
	vector<vector<vector<float>>> centroids(1);
    int ncentroids = 0;

    int sx = grid.size(), sy = grid[0].size();

    // Posicionar los centroides
    float xc=0, yc=0, cont=0;
    vector<float> aux(2);
    while(xc<sx){
        while(yc<sy){
            if(grid[xc][yc]){ // Espacio libre
                aux[0] = xc; aux[1] = yc;
                centroids[0].push_back(aux);
                ncentroids++;
            }
            yc+=radio;
        }
        xc+=radio;
        yc = 0;
    }

    // Mover los centroides hacia la media de los puntos que caen en el radio (con o sin ponderación)
    bool it = true;
    float xm, ym;
    vector<vector<float>> centroids_ = centroids[0];
    int nodes = points.size();
    while(it){
        it = false;
        for(int i=0; i<ncentroids; i++){
            cont = 0;
            for(int j=0; j<nodes; j++){
                if(sqrt(pow(centroids_[i][0]-points[j][0],2)+pow(centroids_[i][1]-points[j][1],2))<=radio){
                    xm+=points[j][0];
                    ym+=points[j][1];
                    cont++;
                }
            }
            if(cont){
                xm/=(float)cont;
                ym/=(float)cont;
                if(centroids_[i][0]!=xm && centroids_[i][1]!=ym && grid[xm][ym]){
                    centroids_[i][0]=xm;
                    centroids_[i][1]=ym;
                    it = true;
                }
            }
            xm = 0; ym = 0;
        }
        centroids.push_back(centroids_);
    }

	return centroids;
}

vector<vector<float>> mean_shift::weighted_movement(vector<vector<float>> points, vector<vector<float>> grid, vector<vector<float>> grad, float radio)
{
	vector<vector<float>> centroids, centroids_;
    int ncentroids = 0;

    int sx = grid.size(), sy = grid[0].size();

    // Posicionar los centroides
    float xc=0, yc=0, cont=0;
    vector<float> aux(2);
    while(xc<sx){
        while(yc<sy){
            if(grid[xc][yc]){ // Espacio libre
                aux[0] = xc; aux[1] = yc;
                centroids.push_back(aux);
                ncentroids++;
            }
            yc+=radio;
        }
        xc+=radio;
        yc = 0;
    }

    centroids_ = centroids;

    // Mover los centroides hacia la media de los puntos que caen en el radio (con o sin ponderación)
    bool it = true;
    float xm, ym;
    vector<float> xm_, ym_;
    vector<float> dist;
    float max, w;
    
    int nodes = points.size();
    int iter = 0;
    while(it){
        it = false;
        for(int i=0; i<ncentroids; i++){
            max = 0;
            for(int j=0; j<nodes; j++){
                if(sqrt(pow(centroids[i][0]-points[j][0],2)+pow(centroids[i][1]-points[j][1],2))<=radio){
                    xm_.push_back(points[j][0]);
                    ym_.push_back(points[j][1]);
                    dist.push_back(grad[points[j][0]][points[j][1]]);
                    if(max<grad[points[j][0]][points[j][1]]){
                    	max=grad[points[j][0]][points[j][1]];
                    }
                }
            }

            if(xm_.size()){
            	// Normalizar las distancias
            	for(int j=0; j<dist.size(); j++){
            		dist[j]/=max; // Normalizar el máximo
            		xm+=(xm_[j]*dist[j]);
                	ym+=(ym_[j]*dist[j]);
                	w+=dist[j];
            	}
            	//xm/=xm_.size();
            	//ym/=ym_.size();
            	xm/=w;
            	ym/=w;

               	if(iter==1){
	                if(centroids_[i][0]!=xm && centroids_[i][1]!=ym && grid[xm][ym]){
	                    centroids[i][0]=xm;
	                    centroids[i][1]=ym;
	                    it = true;
	                }
	            }else{
	            	if(centroids[i][0]!=xm && centroids_[i][1]!=ym && grid[xm][ym] && centroids_[i][0]!=xm && centroids_[i][1]!=ym){
	                    centroids[i][0]=xm;
	                    centroids[i][1]=ym;
	                    it = true;
	                }
	            }
                xm=0; ym=0, w=0;
                xm_.clear(); ym_.clear(); dist.clear();
            }
            
        }
        centroids_=centroids;
    }

	return centroids;
}

vector<vector<vector<float>>> mean_shift::weighted_movement_it(vector<vector<float>> points, vector<vector<float>> grid, vector<vector<float>> grad, float radio)
{
	vector<vector<vector<float>>> centroids(1);
    int ncentroids = 0;

    int sx = grid.size(), sy = grid[0].size();

    // Posicionar los centroides
    float xc=0, yc=0, cont=0;
    vector<float> aux(2);
    while(xc<sx){
        while(yc<sy){
            if(grid[xc][yc]){ // Espacio libre
                aux[0] = xc; aux[1] = yc;
                centroids[0].push_back(aux);
                ncentroids++;
            }
            yc+=radio;
        }
        xc+=radio;
        yc = 0;
    }

    // Mover los centroides hacia la media de los puntos que caen en el radio (con o sin ponderación)
    bool it = true;
    float xm, ym, w;
    vector<float> xm_, ym_;
    vector<float> dist;
    float max;
    vector<vector<float>> centroids_ = centroids[0];
    int nodes = points.size();
    while(it){
        it = false;
        for(int i=0; i<ncentroids; i++){
            max = 0;
            for(int j=0; j<nodes; j++){
                if(sqrt(pow(centroids_[i][0]-points[j][0],2)+pow(centroids_[i][1]-points[j][1],2))<=radio){
                    xm_.push_back(points[j][0]);
                    ym_.push_back(points[j][1]);
                    dist.push_back(grad[points[j][0]][points[j][1]]);
                    if(max<grad[points[j][0]][points[j][1]]){
                    	max=grad[points[j][0]][points[j][1]];
                    }
                }
            }

            if(xm_.size()){
            	// Normalizar las distancias
            	for(int j=0; j<dist.size(); j++){
            		dist[j]/=max;
            		xm+=(xm_[j]*dist[j]);
                	ym+=(ym_[j]*dist[j]);
                	w+=dist[j];
            	}
            	//xm/=xm_.size();
            	//ym/=ym_.size();
            	xm/=w;
            	ym/=w;

               	if(centroids.size()==1){
	                if(centroids_[i][0]!=xm && centroids_[i][1]!=ym && grid[xm][ym]){
	                    centroids_[i][0]=xm;
	                    centroids_[i][1]=ym;
	                    it = true;
	                }
	            }else{
	            	if(centroids_[i][0]!=xm && centroids_[i][1]!=ym && grid[xm][ym] && centroids[centroids.size()-2][i][0]!=xm && centroids[centroids.size()-2][i][1]!=ym){
	                    centroids_[i][0]=xm;
	                    centroids_[i][1]=ym;
	                    it = true;
	                }
	            }
                xm=0; ym=0; w=0;
                xm_.clear(); ym_.clear(); dist.clear();
            }
            
        }
        centroids.push_back(centroids_);
    }

	return centroids;
}

// ***********************************************************************************************************
// 												DIST-SHIFT
// ***********************************************************************************************************

vector<vector<float>> dist_shift::movement(vector<vector<float>> grid, vector<vector<float>> dist, float radio)
{
    vector<vector<float>> res;

    return res;
}

vector<vector<vector<float>>> dist_shift::movement_it(vector<vector<float>> grid, vector<vector<float>> dist_grad, float radio)
{
	vector<vector<vector<float>>> centroids(1);
    int ncentroids = 0;

	// Sacar todos los puntos del grid
	vector<vector<float>> points = free_space_points(grid);
	int np = points.size();
	//vector<float> dist(np);
	//for(int i=0; i<np; i++)
	//	dist[i] = dist_grad[points[i][0]][points[i][1]];

	// Posicionar los centroides sobre el grid
    float xc=0, yc=0, cont=0;
    int sx = grid.size(), sy =grid[0].size();
    vector<float> aux(2);
    while(xc<sx){
        while(yc<sy){
            if(grid[xc][yc]){ // Espacio libre
                aux[0] = xc; aux[1] = yc;
                centroids[0].push_back(aux);
                ncentroids++;
            }
            yc+=radio;
        }
        xc+=radio;
        yc = 0;
    }

    vector<vector<float>> centroids_ = centroids[0];

	bool it = true;
	float max, max_;
	float xm, ym;

	while(it){
		it = false;
		for(int i=0; i<ncentroids; i++){
			// Encontrar el "mejor" vecino
			max = dist_grad[centroids_[i][0]][centroids_[i][1]];
			max_ = max;
			for(int j=0; j<np; j++){
				if(sqrt(pow(centroids_[i][0]-points[j][0],2)+pow(centroids_[i][1]-points[j][1],2))<=radio){
					if(max<dist_grad[points[j][0]][points[j][1]]){
						max = dist_grad[points[j][0]][points[j][1]];
						xm = points[j][0];
						ym = points[j][1];
					}
				}
			}

			// Desplazar el centroide
			if(max != max_){
				centroids_[i][0] = xm;
				centroids_[i][1] = ym;
				it = true;
			}

		}
		centroids.push_back(centroids_);
	}

	return centroids;

}
