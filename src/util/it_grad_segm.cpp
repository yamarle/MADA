#include <mada/util/it_grad_segm.hpp>

it_grad_segm::it_grad_segm(vector<vector<float>> grid, int nseg, int it){
	this->grid=grid;
	this->nseg=nseg;
	max_it=it;
	// Generar posiciones aleatorias
	pos=random_positions(grid, nseg);
	areas.resize(nseg);
	xr_buff.resize(2,vector<int>(nseg,0));
	yr_buff.resize(2,vector<int>(nseg,0));
	xr_buff[0]=pos.x; yr_buff[0]=pos.y;
}

it_grad_segm::it_grad_segm(vector<vector<float>> grid, int nseg, int it, vector<vector<float>> obst_grad){
	this->grid=grid;
	this->nseg=nseg;
	max_it=it;
	// Generar posiciones aleatorias
	areas.resize(nseg);
	xr_buff.resize(2,vector<int>(nseg,0));
	yr_buff.resize(2,vector<int>(nseg,0));
	pos(nseg);
	centroid_initialization(obst_grad);
	xr_buff[0]=pos.x; yr_buff[0]=pos.y;
}

void it_grad_segm::centroid_initialization(vector<vector<float>> obst_grad){
	int ps, iter;
	float max;
	int x_,y_;
	vector<vector<float>> copy=obst_grad;
	for(iter=0; iter<nseg; iter++){
		max=0;
		for(int i=0; i<obst_grad.size(); i++){
			for(int j=0; j<obst_grad[0].size(); j++){
				if(obst_grad[i][j]<INF && obst_grad[i][j]>max){
					max=obst_grad[i][j];
					x_=i; y_=j;
				}
			}
		}
		if(max){
			// Se ha encontrado una posición no anulada
			pos.x[iter]=x_; pos.y[iter]=y_;
			ps=(int)max;
			ps--;
			for(int i=x_-ps; i<x_+ps; i++){
				for(int j=y_-ps; j<y_+ps; j++){
					if(i>=0 && i<obst_grad.size() && j>=0 && j<obst_grad[0].size()){
						obst_grad[i][j]=INF;
					}
				}
			}
			copy[x_][y_]=INF; // anular, por si hay que generar aleatoriamente
		}else{
			// Todas las posiciones están anuladas, encuentro los centroides que quedan de manera aleatoria
			break;
		}
	}
	if(iter<nseg-1){
		// No hay sufuciente espacio para posicionar todos los centroides
		for(int i=iter; i<nseg; i++){
			pos.x[i]=(int(rand())/INF)*copy.size()-1;
			pos.y[i]=(int(rand())/INF)*copy[0].size()-1;
			while(obst_grad[pos.x[i]][pos.y[i]]==INF){
				pos.x[i] = (int(rand())/INF)*copy.size()-1;
				pos.y[i] = (int(rand())/INF)*copy[0].size()-1;
			}
			obst_grad[pos.x[i]][pos.y[i]]=INF;
		}
	}
}

bool it_grad_segm::check_buffer(){
	int cont=0;

	if(curr_it%2) buff_ind = 0;
	else buff_ind = 1;

	for(int i=0; i<nseg; i++){
		if(pos.x[i]==xr_buff[buff_ind][i] && pos.y[i]==yr_buff[buff_ind][i]){
			cont++;
		}else{
			xr_buff[buff_ind][i]=pos.x[i];
			yr_buff[buff_ind][i]=pos.y[i];
		}
	}
	if(cont==nseg) return true;
	else return false;
} // Comprobar si lòs centroides están repitiendo las posiciones

void it_grad_segm::move(){
	for(int i=0; i<nseg; i++){
		aux_path.gradient_descent_(grad,far.x[i],far.y[i]);
		if(aux_path.tam>1){
			pos.x[i]=aux_path.x[1]; pos.y[i]=aux_path.y[1];
		}
		aux_path.clear();
	}
}

void it_grad_segm::iterate(){
	bool stop=false;
	curr_it=0;
	FMM *gr;
	grad_segm *gs;
	while(curr_it<max_it && !stop){
		gr=new FMM(pos.x,pos.y,grid);
		grad=gr->compute_gradient_();
		delete(gr);
		gs=new grad_segm(pos, grad);
		gs->computation();
		far=gs->get_far_pos();
		radius=gs->get_far_val();
		segments=gs->get_segs();
		areas=gs->get_areas();
		delete(gs);
		move();
		stop=check_buffer();
		curr_it++;
	}
}

void it_grad_segm::iterate(string folder){
	bool stop=false;
	curr_it=0;
	FMM *gr;
	grad_segm *gs;
	while(curr_it<max_it && !stop){
		gr=new FMM(pos.x,pos.y,grid);
		grad=gr->compute_gradient_();
		delete(gr);
		gs=new grad_segm(pos, grad);
		gs->computation();
		far=gs->get_far_pos();
		radius=gs->get_far_val();
		segments=gs->get_segs();
		areas=gs->get_areas();
		delete(gs);
		move();
		stop=check_buffer();
		pos.save((folder+"it_grad_centr"+to_string(curr_it)+".txt").c_str());
		far.save((folder+"it_grad_far"+to_string(curr_it)+".txt").c_str());
		save_matr((folder+"it_grad_segs"+to_string(curr_it)+".txt").c_str(), segments);
		save_matr((folder+"it_grad_grad"+to_string(curr_it)+".txt").c_str(), grad);
		curr_it++;
	}
}

vector<vector<int>> it_grad_segm::get_segs(){
	return segments;
}

vector<vector<float>> it_grad_segm::get_grad(){
	return grad;
}

vector<int> it_grad_segm::get_areas(){
	return areas;
}

Poss<int> it_grad_segm::get_pos(){
	return pos;
}

Poss<int> it_grad_segm::get_far(){
	return far;
}

vector<float> it_grad_segm::get_radius(){
	return radius;
}