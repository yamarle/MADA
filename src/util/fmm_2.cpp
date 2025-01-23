#include <mada/util/fmm_2.hpp>

int FMM::presence(int x,int y){
	for(int i=0;i<ws;i++){
		if(x==xw[i] && y==yw[i]){
			return i;
		}
	}
	return -1;
}

FMM::FMM(int x, int y, vector<vector<float> > grid){
	dx={0,1,0,-1};dy={1,0,-1,0};
	tam_p = 1;
	// tamaño
	sx=grid.size();
	sy=grid[0].size();
	this->grid=grid;
	//invertir_obs(this->grid); // porque le he pasado la matriz donde los 1's representan los obstáculos
	// Inicializar las variables para el FMM
	//xs=x;ys=y;
	//initialize();
	// redimensionar la matrices del gradiente
	rhs.resize(sx,vector<float>(sy,INF_VAL));
	//for(int i=0;i<sx;++i) rhs[i].resize(sy);
	v.resize(sx,vector<float>(sy,INF_VAL));
	//for(int i=0;i<sx;++i) v[i].resize(sy);
	// menos el goal
	rhs[x][y]=0;
	// inicialización del frente
	//~=============================================== 
	// reservar espacio de antemano para el frente
	xw.reserve(sx*2+sy*2);
	yw.reserve(sx*2+sy*2);
	cw.reserve(sx*2+sy*2);
	// el caso extremo
	//xw.reserve(sx*sy);
	//yw.reserve(sx*sy);
	//cw.reserve(sx*sy);
	//~===============================================
	xw.push_back(x);
	yw.push_back(y);
	cw.push_back(0);
	ws=1; // tamaño del frente de onda
	expanded_nodes = ws;
} // constructor

FMM::FMM(vector<int> x, vector<int> y, vector<vector<float> > grid){
	dx={0,1,0,-1};dy={1,0,-1,0};
	tam_p = 1;
	// tamaño
	sx=grid.size();
	sy=grid[0].size();
	this->grid=grid;
	// Hasta aquí es lo mismo que lo anterior
	//initialize_sev(x,y);
	rhs.resize(sx,vector<float>(sy,INF_VAL));
	//for(i=0;i<sx;++i) rhs[i].resize(sy);
	v.resize(sx,vector<float>(sy,INF_VAL));
	// menos el goal
	for(int i=0; i<x.size(); i++) rhs[x[i]][y[i]]=0;
	// inicialización del frente
	//~===============================================
	// reservar espacio de antemano para el frente
	xw.reserve(sx*2+sy*2);
	yw.reserve(sx*2+sy*2);
	cw.reserve(sx*2+sy*2);
	// el caso extremo
	//xw.reserve(sx*sy);
	//yw.reserve(sx*sy);
	//cw.reserve(sx*sy);
	//~===============================================
	for(int i=0; i<x.size(); i++){
		xw.push_back(x[i]);
		yw.push_back(y[i]);
		cw.push_back(0);
	}

	ws=x.size(); // tamaño del frente de onda
	expanded_nodes = ws;
} // constructor

FMM::~FMM(){
	//cout<<"Se destruye el gradiente"<<endl;
} // destructor

void FMM::compute_gradient(){
	while (ws>0){
		propagate();
	} // la condición de parada del gradiente
} // calcular el gradiente sobre el escenario entero

vector<vector<float> > FMM::compute_gradient_(){
	while (ws>0){
		propagate();
	} // la condición de parada del gradiente
	return rhs;
} 

vector<vector<float> > FMM::expand_nodes(int n){
	while (expanded_nodes<n && ws>0){
		propagate();
	} // la condición de parada del gradiente
	return rhs;
} // hasta haber expandido una cantidad de nodos determinada

vector<vector<float> > FMM::expand2goal(int x, int y){
	while (rhs[x][y]==INF_VAL && ws>0){
		propagate();
	} // la condición de parada del gradiente
	return rhs;
} // hasta alcanzar el goal

vector<vector<float> > FMM::expand_dist(float dist){
	while(ws>0 && cw[0]<=dist){
		propagate();
	}
	return rhs;
} // hasta haber alcanzado una cierta distancia

vector<vector<float> > FMM::expand_dist(float dist, Poss<int> &pos){
	while (ws>0 && cw[0]<=dist){
		pos.x.push_back(xw[ws-1]); pos.y.push_back(yw[ws-1]);
		propagate();
	}
	return rhs;
} // hasta haber alcanzado una cierta distancia

vector<vector<float> > FMM::pos_coverage(Poss<int> pos){
	vector<int> te;
	while (ws>0 && pos.x.size()){
		propagate();
		for(int i=0; i<pos.x.size(); i++){
			if(rhs[pos.x[i]][pos.y[i]]<INF_VAL){
				te.push_back(i);
			}
		}
		pos = pos.erase(te);
		te.clear(); // Limpiar el vector
	}
	return rhs;
} // hasta haber cubierto las posiciones "pos"

vector<vector<float> > FMM::pos_coverage(int n, Poss<int> pos){
	vector<int> te;
	while (ws>0 && n){
		propagate();
		for(int i=0; i<pos.x.size(); i++){
			if(rhs[pos.x[i]][pos.y[i]]<INF_VAL){
				te.push_back(i);
				n--;
				if(!n) return rhs;
			}
		}
		pos = pos.erase(te);
		te.clear(); // Limpiar el vector
	}
	return rhs;
} // hasta haber cubierto "n" posiciones de "pos"

vector<vector<float> > FMM::pos_coverage(int n, Poss<int> pos, Poss<int> &covered){
	vector<int> te;
	while (ws>0 && n){
		propagate();
		for(int i=0; i<pos.x.size(); i++){
			if(rhs[pos.x[i]][pos.y[i]]<INF_VAL){
				te.push_back(i);
				n--;
				covered.x.push_back(pos.x[i]); covered.y.push_back(pos.y[i]);
				if(!n) return rhs;
			}
		}
		pos = pos.erase(te);
		te.clear(); // Limpiar el vector
	}
	return rhs;
} // hasta haber cubierto "n" posiciones de "pos"

vector<vector<float> > FMM::pos_coverage(vector<int> n, vector<Poss<int>> pos, vector<Poss<int>> &covered){
	vector<int> te;
	int ntot = 0;
	for(int i=0; i<n.size(); i++) ntot += n[i];
	while (ws>0 && ntot){
		propagate();
		for(int c = 0; c<pos.size(); c++){
			if(n[c]){
				for(int i=0; i<pos[c].x.size(); i++){
					if(rhs[pos[c].x[i]][pos[c].y[i]] < INF_VAL){
						te.push_back(i);
						n[c]--;
						ntot--;
						covered[c].x.push_back(pos[c].x[i]); covered[c].y.push_back(pos[c].y[i]);
						if(!ntot) return rhs;
					}
				}
				pos[c] = pos[c].erase(te);
				te.clear(); // Limpiar el vector
			}
		}
	}
	return rhs;
} // hasta haber cubierto "n" posiciones de "pos"

vector<vector<float> > FMM::expand2map_value(int n, int value, vector<vector<int>> map){
	while (ws>0 && n){
		//if(map[xw[0]][yw[0]] == value) n--;
		for(int i=0; i<xw.size(); i++){
			if(map[xw[i]][yw[i]] == value){
				n--;
				if(!n) return rhs;
			}
		}
		propagate();
	}
	return rhs;
} // Hasta cubrir "n" de "map" que contienen el valor "value"

vector<vector<float> > FMM::expand2map_value(int n, float value, vector<vector<float>> map){
	while (ws>0 && n){
		//if(map[xw[0]][yw[0]] == value) n--;
		for(int i=0; i<xw.size(); i++){
			if(map[xw[i]][yw[i]] == value){
				n--;
				if(!n) return rhs;
			}
		}
		propagate();
	}
	return rhs;
} // Hasta cubrir "n" de "map" que contienen el valor "value"

vector<vector<float> > FMM::expand2map_value(int n, int value, vector<vector<int>> map, Poss<int> &covered){
	while (ws>0 && n){
		/*
		if(map[xw[0]][yw[0]] == value){
			covered.push(xw[0], yw[0]);
			n--;
		}
		*/
		for(int i=0; i<xw.size(); i++){
			if(map[xw[i]][yw[i]] == value){
				n--;
				covered.push(xw[i], yw[i]);
				if(!n) return rhs;
			}
		}
		propagate();
	}
	return rhs;
} // Hasta cubrir "n" de "map" que contienen el valor "value"

vector<vector<float> > FMM::expand2map_value(int n, float value, vector<vector<float>> map, Poss<int> &covered){
	while (ws>0 && n){
		/*
		if(map[xw[0]][yw[0]] == value){
			covered.push(xw[0], yw[0]);
			n--;
		}
		*/
		for(int i=0; i<xw.size(); i++){
			if(map[xw[i]][yw[i]] == value){
				n--;
				covered.push(xw[i], yw[i]);
				if(!n) return rhs;
			}
		}
		propagate();
	}
	return rhs;
} // Hasta cubrir "n" de "map" que contienen el valor "value"

void FMM::save_grad(string folder){
	int cont=0;
	while (ws>0){
		propagate();
		save_matr((folder+"grad"+to_string(cont)+".txt").c_str(),rhs);
		cont++;
	} // la condición de parada del gradiente
} // calcular el gradiente sobre el escenario entero

void FMM::propagate(){
	// se selecciona el último elemento del frente y se elimina de él
	ax=xw[ws-1]; xw.pop_back();
	ay=yw[ws-1]; yw.pop_back();
	ac=cw[ws-1]; cw.pop_back();
	ws--;
	if(v[ax][ay]>rhs[ax][ay]){ // seleccionar los vecinos todavía no calculados
		v[ax][ay]=rhs[ax][ay];
		for(int i=0;i<4;i++){ // para los vecinos
			nx=ax+dx[i];
			ny=ay+dy[i];
			if(nx>=0 && nx<sx && ny>=0 && ny<sy){ // si no me he salido del grid
				//~ update_vertex();
				//cout<<"("<<nx<<", "<<ny<<"): "<<grid[nx][ny]<<endl;
				//~ if (!grid[nx][ny]) update_vertex(); // actualizar si no estoy en un obstáculo
				if (grid[nx][ny] && rhs[nx][ny] && v[nx][ny]==INF_VAL){
					update_vertex(); // actualizar si no estoy en un obstáculo, en un goal y no estoy recalculando
					requeue(); // reordenar el frente con los costes de menor a mayor
				}
				//~ if (rhs[nx][ny] && v[nx][ny]==INF_VAL) update_vertex(); // actualizar si no estoy en un obstáculo, en un goal y no estoy recalculando
			}
		}
	}
} // propagar el frente

void FMM::update_vertex(){
	if(rhs[nx][ny]==INF_VAL) expanded_nodes++; // implica que no se está recalculando
	computePropagator(); // obtener con qué nodos voy a interpolar
	kernel_LSM(); // calcular el valor del nodo
	//requeue(); // reordenar el frente con los costes de menor a mayor
	// si estoy aquí, significa que se va a calcular la posición del gradiente nueva y no es un obstáculo
	//expanded_nodes++;
} // calcular el valor de un punto del grid

void FMM::computePropagator(){
	qx1=qx2=qy1=qy2=RAND_MAX;
	qc1=qc2=INF_VAL;	
	for(int i=0;i<4;i++){
		qx_=nx+dx[i];qy_=ny+dy[i];
		if(qx_>=0 && qy_>=0 && qx_<sx && qy_<sy){ // dentro del grid
			qc_=v[qx_][qy_];
			if(qc_<INF_VAL){ // sólo los vecinos que ya están calculados
				if(qc1==INF_VAL && qc_<qc1){ // qc1 no ha sido calculado todavía
					qx1=qx_;qy1=qy_;qc1=qc_;
				}else if(qc1<INF_VAL && qc2==INF_VAL && qc_<qc2){ // qc1 tiene valor y qc2 no
					if(qc_<qc1){ // el nuevo valor es menor, intercambio con qc1
						qx2=qx1;qy2=qy1;qc2=qc1;
						qx1=qx_;qy1=qy_;qc1=qc_;
					}else{
						qx2=qx_;qy2=qy_;qc2=qc_;
					}
				}else if (qc1<INF_VAL && qc2<INF_VAL){ // ambos ya estan calculados
					if (qc_<qc2 && qc_>qc1){ // menor que el segundo
						qx2=qx_;qy2=qy_;qc2=qc_;
					}else if (qc_<qc1){ // menor que el primero, intercambiar
						qx2=qx1;qy2=qy1;qc2=qc1;
						qx1=qx_;qy1=qy_;qc1=qc_;
					}
				}
			}
		}
	}
} // puntos para la interpolación

void FMM::kernel_LSM(){
	Ta=qc1;
	Tc=qx2<sx?qc2:INF_VAL;
	if (Tc-Ta>=tam_p/grid[nx][ny]){
		rhs[nx][ny]=Ta+tam_p/grid[nx][ny];
	}else{
		beta=-(Ta+Tc);
		gamma=.5*(pow(Ta,2)+pow(Tc,2)-(tam_p/pow(grid[nx][ny],2)));
		rhs[nx][ny]=.5*(-beta+sqrt(pow(beta,2)-4*gamma));
	}
} // calcular el valor del punto expandido

void FMM::requeue(){
	pos=presence(nx,ny);
	min=rhs[nx][ny]<v[nx][ny]?rhs[nx][ny]:v[nx][ny];
	if(pos<0){ // si no estaba en el frente, insertar
		// buscar donde insertar
		cont=0;
		diff=cw[cont]-min;
		while(diff>0 && cont<ws){
			diff=cw[cont]-min;
			cont++;
		}
		xw.insert(xw.begin()+cont,nx);
		yw.insert(yw.begin()+cont,ny);
		cw.insert(cw.begin()+cont,min);
		ws++;
	}else if(cw[pos]!=min){ // si ya estaba
		// primero lo borro
		xw.erase(xw.begin()+pos);
		yw.erase(yw.begin()+pos);
		cw.erase(cw.begin()+pos);
		// y añado uno nuevo
		cont=0;
		diff=cw[cont]-min;
		while(diff>0 && cont<ws){
			diff=cw[cont]-min;
			cont++;
		}
		if(!cont) cont++;
		xw.insert(xw.begin()+cont-1,nx);
		yw.insert(yw.begin()+cont-1,ny);
		cw.insert(cw.begin()+cont-1,min);
		// ERES TONTO, QUITAS UNO E INSERTAS OTRO, EL TAMAÑO SIGUE SIENDO EL MISMO
		//ws++;
	}
} // reordenar el frente

vector<vector<float> > FMM::get_rhs(){
	return rhs;
}
