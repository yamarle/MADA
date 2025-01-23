#include <mada/util/grad_segm_def.hpp>

grad_segm::grad_segm(Poss<int> p, vector<vector<float>> grad)
{
	centr=p;
	sx=grad.size(); sy=grad[0].size();
	segments.resize(sx, vector<int>(sy,0));
	this->grad=grad;
	nseg=p.x.size();
	for(int i=0; i<nseg; i++){
		segments[p.x[i]][p.y[i]]=i+1;
	}
	far_pos(nseg);
	areas.resize(nseg,1);
	max.resize(nseg,0);
	seg_ind=0;
}

void grad_segm::computation(){
	for(int i=0; i<nseg; i++){
		seg_ind=i+1;
		qx.push_back(centr.x[i]); qy.push_back(centr.y[i]);
		qd.push_back(0);
		while(qx.size()){
			propagate();
		}
	}
}

void grad_segm::find_min()
{
	min=INF;
	for(int i=0; i<nx.size(); i++){
		x_=x+nx[i]; y_=y+ny[i];
		if(x_>=0 && x_<sx && y_>=0 && y_<sy){
			if(min>grad[x_][y_]){
				xmin=x_; ymin=y_;
				min=grad[x_][y_];
			}
		}
	}
}

void grad_segm::to_queue()
{
	where=-1;
	for(int i=0; i<qx.size(); i++){
		if(qd[i]<=min){
			where=i;
		}
	}
	// Insertar
	if(where>=0){
		qx.insert(qx.begin()+where+1,x);
		qy.insert(qy.begin()+where+1,y);
		qd.insert(qd.begin()+where+1,min);
	}else{
		qx.insert(qx.begin(),x);
		qy.insert(qy.begin(),y);
		qd.insert(qd.begin(),min);
	}
}

void grad_segm::propagate()
{
	for(int i=0; i<nx.size(); i++){
		x=qx[0]+nx[i]; y=qy[0]+ny[i];
		if(x>=0 && x<sx && y>=0 && y<sy){
			if(!segments[x][y] && grad[x][y]<INF){
				find_min();
				if(min<INF && segments[xmin][ymin]==seg_ind){
					// Etiquetar
					segments[x][y]=seg_ind;
					// Insertar
					//qx.push_back(x); qy.push_back(y); qd.push_back(min);
					to_queue();
					// Acumular área
					areas[seg_ind-1]++;
					// Almacenar el máximo
					if(max[seg_ind-1]<grad[x][y]){
						far_pos.x[seg_ind-1]=x;
						far_pos.y[seg_ind-1]=y;
						max[seg_ind-1]=grad[x][y];
					}
				}
			}
		}
	}
	qx.erase(qx.begin()); qy.erase(qy.begin()); qd.erase(qd.begin());
}

vector<vector<int>> grad_segm::get_segs()
{
	return segments;
}

vector<int> grad_segm::get_areas()
{
	return areas;
}

Poss<int> grad_segm::get_far_pos()
{
	return far_pos;
}

vector<float> grad_segm::get_far_val()
{
	return max;
}

Poss<int> grad_segm::get_contour()
{
	return contour;
}