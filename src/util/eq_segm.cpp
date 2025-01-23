#include <mada/util/eq_segm.hpp>

eq_segm::eq_segm(int x, int y, vector<vector<float>> grid, int nseg){
	this->nseg=nseg;
	this->grid=grid;
	aux_grid=grid;
	pos(nseg);
	pos.x[0]=x; pos.y[0]=y;
	sx=grid.size(); sy=grid[0].size();
	segments.resize(sx,vector<int>(sy,0));
	areas.resize(nseg); perimeters.resize(nseg);
	adjacency.resize(nseg);
	tot_area=0;
	FMM gr(x,y,grid);
	ref_grad=gr.compute_gradient_();
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			if(ref_grad[i][j]<INF) tot_area++;
		}
	}
	opt_seg_area=tot_area/nseg;
	free_space=tot_area;
	adj_matr.resize(nseg,vector<int>(nseg,0));
}

void eq_segm::classify(){
	value=INF;
	// Cuántos he expandido?
	aux=0;
	qx.clear(); qy.clear();
	// Reiniciar la adyacencia
	for(int i=0; i<nseg; i++) adjacency[i]=0;
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			if(aux_grid[i][j] && grad[i][j]<INF){
				aux++;
				qx.push_back(i); qy.push_back(j);
				for(int k=0; k<nx.size(); k++){
					x_=i+nx[k]; y_=j+ny[k];
					if(x_>=0 && x_<sx && y_>=0 && y_<sy){
						if(segments[x_][y_]) adjacency[segments[x_][y_]-1]++;
					}
				}
			}
		}
	}
	//cout<<"He propagado "<<aux<<" - "<<opt_seg_area<<endl;
	//sh_vect_h(areas,"Areas");
	//sh_vect_h(adjacency,"Adyacencia");
	if(aux<opt_seg_area/2 || nseg_==nseg-1){ // segmento suficientemente pequeño
		// Hay que pegar a otro segmento
		minv=RAND_MAX; ind=-1;
		for(int i=0; i<nseg; i++){
			if(areas[i] && areas[i]<minv && adjacency[i]){
				ind=i;
				minv=areas[i];
			}
		}
		for(int i=0; i<qx.size(); i++){
			segments[qx[i]][qy[i]]=ind+1;
			free_space--;
			aux_grid[qx[i]][qy[i]]=0;
		}
		areas[ind]+=aux;
		//cout<<"Junto al segmento "<<ind<<endl;
	}else{
		// Segmento nuevo
		for(int i=0; i<qx.size(); i++){
			segments[qx[i]][qy[i]]=nseg_+1;
			free_space--;
			aux_grid[qx[i]][qy[i]]=0;
		}
		areas[nseg_]=aux;
		pos.x[nseg_]=x; pos.y[nseg_]=y;
		nseg_++;
		//cout<<"Segmento nuevo"<<endl;
	}
}

void eq_segm::find_nearest(){
	// Esto podría hacerse mediante un frente hasta donde haya llegado el gradiente iterativamente
	min=INF;
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			if(!segments[i][j] && ref_grad[i][j]<INF && min>ref_grad[i][j]){
				min=ref_grad[i][j];
				x=i; y=j;
			}
		}
	}
}

void eq_segm::find_farthest(){
	// Esto podría hacerse mediante un frente hasta donde haya llegado el gradiente iterativamente
	max=0;
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			if(!segments[i][j] && ref_grad[i][j]<INF && max<ref_grad[i][j]){
				max=ref_grad[i][j];
				x=i; y=j;
			}
		}
	}
}

void eq_segm::computation(){
	nseg_=0;
	while(free_space){
		// Encontrar el cercano no clasificado
		find_nearest();
		// Expandir "opt_seg_area" nodos
		FMM gr(x,y,aux_grid);
		grad=gr.expand_nodes(opt_seg_area);
		// Clasificar el nuevo segmento
		classify();
	}
	finish();
}

void eq_segm::computation(string folder){
	nseg_=0;
	//save_matr((folder+"ref_grad.txt").c_str(),ref_grad);
	int cont=0;
	while(free_space){
		// Encontrar el cercano no clasificado
		find_nearest();
		// Expandir "opt_seg_area" nodos
		FMM gr(x,y,aux_grid);
		grad=gr.expand_nodes(opt_seg_area);
		// Clasificar el nuevo segmento
		classify();
		//cout<<free_space<<endl;
		save_matr((folder+"grad"+to_string(cont)+".txt").c_str(),grad);
		save_matr((folder+"segs"+to_string(cont)+".txt").c_str(),segments);
		//cin.get();
		cont++;
	}
	finish();
}

void eq_segm::adjacency_(){
	vector<int> nx_={0, 1, 0,-1};
	vector<int> ny_={1, 0,-1, 0};
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			if(grid[i][j]){
				for(int k=0; k<nx_.size(); k++){
					x_=i+nx_[k]; y_=j+ny_[k];
					if(x_>=0 && x_<sx && y_>=0 && y_<sy){
						if(segments[x_][y_] && segments[i][j]!=segments[x_][y_]){
							//cout<<adj_matr.size()<<", "<<adj_matr[0].size()<<endl;
							adj_matr[segments[i][j]-1][segments[x_][y_]-1]++;
							adj_matr[segments[x_][y_]-1][segments[i][j]-1]++;
						}
					}
				}
			}
		}
	}
}

void eq_segm::classify_(int s1, int s2, int new_area_seg){
	find_nearest();
	FMM gr_n(x,y,aux_grid);
	grad=gr_n.expand_nodes(new_area_seg);

	// Cuántos he expandido?
	// Este bucle se debería hacer mirando los vecinos (conozco el punto de partida)
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			if(aux_grid[i][j] && grad[i][j]<INF){
				segments[i][j]=s2+1;
				areas[s2]++;
				aux_grid[i][j]=0;
				free_space--;
			}
		}
	}

	find_farthest();
	FMM gr_f(x,y,aux_grid);
	grad=gr_f.expand_nodes(new_area_seg);

	// Cuántos he expandido?
	// Este bucle se debería hacer mirando los vecinos (conozco el punto de partida)
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			if(aux_grid[i][j] && grad[i][j]<INF){
				segments[i][j]=s1+1;
				areas[s1]++;
				aux_grid[i][j]=0;
				free_space--;
			}
		}
	}

}

void eq_segm::separate(int s1, int s2){
	// s1 es el segmento a pegar a otro
	// s2 es el segmento más grande
	// Nuevo espacio libre
	free_space=areas[s1]+areas[s2];
	// Quitar la etiqueta a los segmentos
	//min=INF;
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			if(segments[i][j]==s1+1 || segments[i][j]==s2+1){
				segments[i][j]=0;
				aux_grid[i][j]=1;
				//free_space++;
				if(ref_grad[i][j]<min){
					//min=ref_grad[i][j];
					x=i; y=j;
				}
			}
		}
	}
	opt_seg_area=(areas[s1]+areas[s2])/2;
	areas[s1]=0; areas[s2]=0;
	// Ahora propagar desde el cercano y desde el alejado
	classify_(s1,s2,opt_seg_area);
	while(free_space){
		find_nearest();
		FMM gr(x,y,aux_grid);
		grad=gr.expand_nodes(opt_seg_area);
		classify();
		//cin.get();
	}
}

void eq_segm::finish(){
	adjacency_(); // Calcular la matriz de adyacencia
	for(int i=0; i<nseg; i++){
		if(areas[i] && areas[i]<opt_seg_area/2){
			// Encontrar el segmento adyacente de mayor área
			minv=0; ind=-1;
			for(int j=0; j<nseg; j++){
				if(minv<areas[j] && adj_matr[i][j]){
					minv=areas[j];
					ind=j;
				}
			}
			// Ahora hay que partir los dos segmentos en 2
			//cout<<"Juntar a un segmento"<<endl;
			separate(i,ind);
		}else if(!areas[i]){
			minv=0;
			for(int j=0; j<nseg; j++){
				if(areas[j] && minv<areas[j]){
					minv=areas[j];
					ind=j;
				}
			}
			//cout<<"Separar un segmento"<<endl;
			separate(i,ind);
		}
	}
}

vector<vector<int>> eq_segm::get_segs(){
	return segments;
}

vector<int> eq_segm::get_areas(){
	return areas;
}

vector<vector<float>> eq_segm::get_grad(){
	return ref_grad;
}

Poss<int> eq_segm::get_pos(){
	return pos;
}