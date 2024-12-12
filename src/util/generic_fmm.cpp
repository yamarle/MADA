#include <mada/util/generic_fmm.hpp>

int generic_FMM::presence(int x,int y)
{
	for(int i=0;i<ws;i++){
		if(x==xw[i] && y==yw[i]){
			return i;
		}
	}
	return -1;
}

generic_FMM::generic_FMM(vector<int> x, vector<int> y, vector<vector<vector<float> > > velocity_maps)
{
	dx={0,1,0,-1};dy={1,0,-1,0};
	tam_p = 1;
	// tamaño
	sx=velocity_maps[0].size();
	sy=velocity_maps[0][0].size();
	this->velocity_maps = velocity_maps;
	// Inicializar los "tamaños" de los frentes
	sizes.resize(x.size(),1);
	// Hasta aquí es lo mismo que lo anterior
	//initialize_sev(x,y);
	rhs.resize(sx,vector<float>(sy,INF_VAL));
	//for(i=0;i<sx;++i) rhs[i].resize(sy);
	v.resize(sx,vector<float>(sy,INF_VAL));
	sources_map.resize(sx,vector<int>(sy,-1));
	// Los puntos más lejanos desde las fuentes
	xf = x; yf = y; df.resize(x.size(),0);
	// menos el goal
	for(int i=0; i<x.size(); i++){
		rhs[x[i]][y[i]]=0;
		sources_map[x[i]][y[i]]=i;
	}
	// inicialización del frente
	//~===============================================
	// reservar espacio de antemano para el frente
	xw.reserve(sx*2+sy*2);
	yw.reserve(sx*2+sy*2);
	cw.reserve(sx*2+sy*2);
	sw.reserve(sx*2+sy*2);
	// el caso extremo
	//xw.reserve(sx*sy);
	//yw.reserve(sx*sy);
	//cw.reserve(sx*sy);
	//~===============================================
	for(int i=0; i<x.size(); i++){
		xw.push_back(x[i]);
		yw.push_back(y[i]);
		cw.push_back(0);
		sw.push_back(i);
	}

	ws=x.size(); // tamaño del frente de onda
	expanded_nodes = ws;

	// ??????????????????????????????????????????????????????????????
	// ESTO ES PARA LA PRUEBA DE ACUMULAR TODOS LOS PUTNOS DENTRO
	segs_poss.resize(x.size());
	for(int i=0; i<x.size(); i++){
		segs_poss[i].x.push_back(x[i]);
		segs_poss[i].y.push_back(y[i]);
	}
	// ??????????????????????????????????????????????????????????????

} // constructor

generic_FMM::generic_FMM(vector<int> x, vector<int> y, vector<vector<vector<float> > > velocity_maps, vector<vector<float> > distance_map)
{
	dx={0,1,0,-1};dy={1,0,-1,0};
	tam_p = 1;
	// tamaño
	sx=velocity_maps[0].size();
	sy=velocity_maps[0][0].size();
	this->velocity_maps = velocity_maps;
	// Inicializar los "tamaños" de los frentes
	sizes.resize(x.size(),1); // 1 celda por defecto
	// Hasta aquí es lo mismo que lo anterior
	//initialize_sev(x,y);
	rhs.resize(sx,vector<float>(sy,INF_VAL));
	//for(i=0;i<sx;++i) rhs[i].resize(sy);
	v.resize(sx,vector<float>(sy,INF_VAL));
	// Inicializar el mapa de los índices de los segmentos
	sources_map.resize(sx,vector<int>(sy,-1));
	// Distance Transform
	this->distance_map = distance_map;
	// Los puntos más lejanos desde las fuentes
	xf = x; yf = y; df.resize(x.size(),0);
	// menos el goal
	for(int i=0; i<x.size(); i++){
		rhs[x[i]][y[i]]=0;
		sources_map[x[i]][y[i]]=i;
	}
	// inicialización del frente
	//~===============================================
	// reservar espacio de antemano para el frente
	xw.reserve(sx*2+sy*2);
	yw.reserve(sx*2+sy*2);
	cw.reserve(sx*2+sy*2);
	sw.reserve(sx*2+sy*2);
	// el caso extremo
	//xw.reserve(sx*sy);
	//yw.reserve(sx*sy);
	//cw.reserve(sx*sy);
	//~===============================================
	for(int i=0; i<x.size(); i++){
		xw.push_back(x[i]);
		yw.push_back(y[i]);
		cw.push_back(0);
		sw.push_back(i);
	}

	ws=x.size(); // tamaño del frente de onda
	expanded_nodes = ws;

	// ??????????????????????????????????????????????????????????????
	// ESTO ES PARA LA PRUEBA DE ACUMULAR TODOS LOS PUTNOS DENTRO
	segs_poss.resize(x.size());
	for(int i=0; i<x.size(); i++){
		segs_poss[i].x.push_back(x[i]);
		segs_poss[i].y.push_back(y[i]);
	}
	// ??????????????????????????????????????????????????????????????

} // constructor

generic_FMM::generic_FMM(vector<int> x, vector<int> y, vector<vector<vector<float> > > velocity_maps, vector<vector<float> > distance_map, vector<int> sizes)
{
	dx={0,1,0,-1};dy={1,0,-1,0};
	tam_p = 1;
	// tamaño
	sx=velocity_maps[0].size();
	sy=velocity_maps[0][0].size();
	this->velocity_maps = velocity_maps;
	// Inicializar los "tamaños" de los frentes
	this->sizes = sizes;
	// Hasta aquí es lo mismo que lo anterior
	//initialize_sev(x,y);
	rhs.resize(sx,vector<float>(sy,INF_VAL));
	//for(i=0;i<sx;++i) rhs[i].resize(sy);
	v.resize(sx,vector<float>(sy,INF_VAL));
	// Inicializar el mapa de los índices de los segmentos
	sources_map.resize(sx,vector<int>(sy,-1));
	// Distance Transform
	this->distance_map = distance_map;
	// Los puntos más lejanos desde las fuentes
	xf = x; yf = y; df.resize(x.size(),0);
	// menos el goal
	for(int i=0; i<x.size(); i++){
		if(distance_map[x[i]][y[i]]>=sizes[i])
			rhs[x[i]][y[i]]=0;
		//rhs[x[i]][y[i]]=0;
		sources_map[x[i]][y[i]]=i;
	}
	// inicialización del frente
	//~===============================================
	// reservar espacio de antemano para el frente
	xw.reserve(sx*2+sy*2);
	yw.reserve(sx*2+sy*2);
	cw.reserve(sx*2+sy*2);
	sw.reserve(sx*2+sy*2);
	// el caso extremo
	//xw.reserve(sx*sy);
	//yw.reserve(sx*sy);
	//cw.reserve(sx*sy);
	//~===============================================
	for(int i=0; i<x.size(); i++){
		xw.push_back(x[i]);
		yw.push_back(y[i]);
		cw.push_back(0);
		sw.push_back(i);
	}

	ws=x.size(); // tamaño del frente de onda
	expanded_nodes = ws;

	// ??????????????????????????????????????????????????????????????
	// ESTO ES PARA LA PRUEBA DE ACUMULAR TODOS LOS PUTNOS DENTRO
	segs_poss.resize(x.size());
	for(int i=0; i<x.size(); i++){
		segs_poss[i].x.push_back(x[i]);
		segs_poss[i].y.push_back(y[i]);
	}
	// ??????????????????????????????????????????????????????????????

} // constructor

void generic_FMM::size_initialization()
{
	Poss<int> c;
	for(int i=0; i<sizes.size(); i++){
		// Todas las posiciones del círculo
		c = circle_poss(xw[i],yw[i],sizes[i],sx,sy);
		// Quitar el punto duplicado del frente (Por no tenerlo duplicado y p)
		//xw.erase(xw.begin()); yw.erase(yw.begin());
		//cw.erase(cw.begin()); sw.erase(sw.begin());
		//ws--;
		for(int j=0; j<c.x.size(); j++){
			// insertar en el frente
			xw.push_back(c.x[j]); yw.push_back(c.y[j]);
			//cw.push_back(0); sw.push_back(i); // MAL
			ws++;
			// Actualizar los mapas
			//rhs[c.x[j]][c.y[j]] = 0; // MAL 
			rhs[c.x[j]][c.y[j]] = sqrt(pow(xw[i]-c.x[j],2)-pow(yw[i]-c.y[j],2));
			sources_map[c.x[j]][c.y[j]] = i;

			cw.push_back(rhs[c.x[j]][c.y[j]]); sw.push_back(i);
		}
	}

	expanded_nodes = ws;
}

void generic_FMM::areas_initialization()
{
	areas.resize(sizes.size(),1); // El punto ya pertenece al área del segmento
	adjacency.resize(sizes.size(),vector<bool>(sizes.size(),false));
	xa.resize(sizes.size(),vector<int>(sizes.size()));
	ya.resize(sizes.size(),vector<int>(sizes.size()));
	da.resize(sizes.size(),vector<float>(sizes.size(),INF_VAL));
}

void generic_FMM::goals_initialization(vector<int> xgoals, vector<int> ygoals)
{
	ngoals.resize(sizes.size(),0); // Cantidad de goals dentro de cada segmento
	goals_map.resize(sx, vector<bool>(sy,false));
	for(int i=0; i<xgoals.size(); i++)
		goals_map[xgoals[i]][ygoals[i]] = true;

	xgoal_close.resize(sizes.size()); ygoal_close.resize(sizes.size());
	dgoal_close.resize(sizes.size(),INF_VAL);
	xgoal_far.resize(sizes.size()); ygoal_far.resize(sizes.size());
	dgoal_far.resize(sizes.size(),0);
}

generic_FMM::~generic_FMM()
{
	//cout<<"Se destruye el gradiente"<<endl;
} // destructor

void generic_FMM::compute_gradient()
{
	if(!distance_map.size())
		while (ws>0) propagate();
	else
		while (ws>0) propagate_dt();
} // calcular el gradiente sobre el escenario entero

vector<vector<float> > generic_FMM::compute_gradient_()
{
	if(!distance_map.size()){
		while (ws>0){
			propagate();
		} // la condición de parada del gradiente
	}else{
		while (ws>0){
			propagate_dt();
		} // la condición de parada del gradiente
	}
	return rhs;
} 

vector<vector<float> > generic_FMM::expand_nodes(int n)
{
	if(!distance_map.size()){
		while (expanded_nodes<n && ws>0){
			propagate();
		} // la condición de parada del gradiente
	}else{
		while (expanded_nodes<n && ws>0){
			propagate_dt();
		} // la condición de parada del gradiente
	}
	return rhs;
} // hasta haber expandido una cantidad de nodos determinada

vector<vector<float> > generic_FMM::expand2goal(int x, int y)
{
	if(!distance_map.size()){
		while (rhs[x][y]==INF_VAL && ws>0){
			propagate();
		} // la condición de parada del gradiente
	}else{
		while (rhs[x][y]==INF_VAL && ws>0){
			propagate_dt();
		} // la condición de parada del gradiente
	}
	return rhs;
} // hasta alcanzar el goal

vector<vector<float> > generic_FMM::expand_dist(float dist)
{
	if(!distance_map.size()){
		while(ws>0 && cw[0]<=dist){
			propagate();
		}
	}else{
		while(ws>0 && cw[0]<=dist){
			propagate_dt();
		}
	}
	return rhs;
} // hasta haber alcanzado una cierta distancia

vector<vector<float> > generic_FMM::expand_dist(float dist, Poss<int> &pos)
{
	if(!distance_map.size()){
		while (ws>0 && cw[0]<=dist){
			pos.x.push_back(xw[ws-1]); pos.y.push_back(yw[ws-1]);
			propagate();
		}
	}else{
		while (ws>0 && cw[0]<=dist){
			pos.x.push_back(xw[ws-1]); pos.y.push_back(yw[ws-1]);
			propagate_dt();
		}
	}
	return rhs;
} // hasta haber alcanzado una cierta distancia

vector<vector<float> > generic_FMM::pos_coverage(Poss<int> pos)
{
	vector<int> te;
	if(!distance_map.size()){
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
	}else{
		while (ws>0 && pos.x.size()){
			propagate_dt();
			for(int i=0; i<pos.x.size(); i++){
				if(rhs[pos.x[i]][pos.y[i]]<INF_VAL){
					te.push_back(i);
				}
			}
			pos = pos.erase(te);
			te.clear(); // Limpiar el vector
		}
	}
	return rhs;
} // hasta haber cubierto las posiciones "pos"

vector<vector<float> > generic_FMM::pos_coverage(int n, Poss<int> pos)
{
	vector<int> te;
	if(!distance_map.size()){
		while (ws>0 && n){
			propagate();
			for(int i=0; i<pos.x.size(); i++){
				if(rhs[pos.x[i]][pos.y[i]]<INF_VAL){
					te.push_back(i);
					n--;
				}
			}
			pos = pos.erase(te);
			te.clear(); // Limpiar el vector
		}
	}else{
		while (ws>0 && n){
			propagate_dt();
			for(int i=0; i<pos.x.size(); i++){
				if(rhs[pos.x[i]][pos.y[i]]<INF_VAL){
					te.push_back(i);
					n--;
				}
			}
			pos = pos.erase(te);
			te.clear(); // Limpiar el vector
		}
	}
	return rhs;
} // hasta haber cubierto "n" posiciones de "pos"

vector<vector<float> > generic_FMM::pos_coverage(int n, Poss<int> pos, Poss<int> &covered)
{
	vector<int> te;
	if(!distance_map.size()){
		while (ws>0 && n){
			propagate();
			for(int i=0; i<pos.x.size(); i++){
				if(rhs[pos.x[i]][pos.y[i]]<INF_VAL){
					te.push_back(i);
					n--;
					covered.x.push_back(pos.x[i]); covered.y.push_back(pos.y[i]);
				}
			}
			pos = pos.erase(te);
			te.clear(); // Limpiar el vector
		}
	}else{
		while (ws>0 && n){
			propagate_dt();
			for(int i=0; i<pos.x.size(); i++){
				if(rhs[pos.x[i]][pos.y[i]]<INF_VAL){
					te.push_back(i);
					n--;
					covered.x.push_back(pos.x[i]); covered.y.push_back(pos.y[i]);
				}
			}
			pos = pos.erase(te);
			te.clear(); // Limpiar el vector
		}
	}
	return rhs;
} // hasta haber cubierto "n" posiciones de "pos"

void generic_FMM::save_grad(string folder)
{
	int cont=0;
	if(!distance_map.size()){
		while (ws>0){
			propagate();
			save_matr((folder+"grad"+to_string(cont)+".txt").c_str(),rhs);
			cont++;
		} // la condición de parada del gradiente
	}else{
		while (ws>0){
			propagate_dt();
			save_matr((folder+"grad"+to_string(cont)+".txt").c_str(),rhs);
			cont++;
		} // la condición de parada del gradiente
	}
} // calcular el gradiente sobre el escenario entero

void generic_FMM::propagate()
{
	// se selecciona el último elemento del frente y se elimina de él
	ax = xw[ws-1]; xw.pop_back();
	ay = yw[ws-1]; yw.pop_back();
	ac = cw[ws-1]; cw.pop_back();
	asi = sw[ws-1]; sw.pop_back();
	ws--;
	if(v[ax][ay]>rhs[ax][ay]){ // seleccionar los vecinos todavía no calculados
		v[ax][ay]=rhs[ax][ay];
		for(int i=0;i<4;i++){ // para los vecinos
			nx=ax+dx[i];
			ny=ay+dy[i];
			if(nx>=0 && nx<sx && ny>=0 && ny<sy){ // si no me he salido del grid
				//if(grid[nx][ny] && rhs[nx][ny] && v[nx][ny]==INF_VAL){ // ---> ORIGINAL DE FMM BÁSICO
				if(velocity_maps[asi][nx][ny] && rhs[nx][ny] && v[nx][ny]==INF_VAL){
					update_vertex(); // actualizar si no estoy en un obstáculo, en un goal y no estoy recalculando
					requeue(); // reordenar el frente con los costes de menor a mayor
				}
				//~ if (rhs[nx][ny] && v[nx][ny]==INF_VAL) update_vertex(); // actualizar si no estoy en un obstáculo, en un goal y no estoy recalculando
			}
		}
	}
} // propagar el frente

void generic_FMM::propagate_dt()
{
	// se selecciona el último elemento del frente y se elimina de él
	ax = xw[ws-1]; xw.pop_back();
	ay = yw[ws-1]; yw.pop_back();
	ac = cw[ws-1]; cw.pop_back();
	asi = sw[ws-1]; sw.pop_back();
	ws--;
	if(v[ax][ay]>rhs[ax][ay]){ // seleccionar los vecinos todavía no calculados
		v[ax][ay]=rhs[ax][ay];
		for(int i=0;i<4;i++){ // para los vecinos
			nx=ax+dx[i];
			ny=ay+dy[i];
			if(nx>=0 && nx<sx && ny>=0 && ny<sy){ // si no me he salido del grid
				// No es obstáculo, no invado ningún obstáculo, no está calculado todavía
				//if(velocity_maps[asi][nx][ny] && distance_map[nx][ny]>sizes[asi] && rhs[nx][ny] && v[nx][ny]==INF_VAL){
				if(velocity_maps[asi][nx][ny] && rhs[nx][ny] && v[nx][ny]==INF_VAL){
					update_vertex(); // actualizar si no estoy en un obstáculo, en un goal y no estoy recalculando
					requeue(); // reordenar el frente con los costes de menor a mayor
				}
				//~ if (rhs[nx][ny] && v[nx][ny]==INF_VAL) update_vertex(); // actualizar si no estoy en un obstáculo, en un goal y no estoy recalculando
			}
		}
	}
} // propagar el frente

void generic_FMM::update_vertex()
{
	if(rhs[nx][ny]==INF_VAL) expanded_nodes++; // implica que no se está recalculando
	computePropagator(); // obtener con qué nodos voy a interpolar
	kernel_LSM(); // calcular el valor del nodo
} // calcular el valor de un punto del grid

void generic_FMM::computePropagator()
{
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

void generic_FMM::kernel_LSM()
{
	Ta=qc1;
	Tc=qx2<sx?qc2:INF_VAL;
	if (Tc-Ta>=tam_p/velocity_maps[asi][nx][ny]){
		rhs[nx][ny]=Ta+tam_p/velocity_maps[asi][nx][ny];
		//sources_map[nx][ny] = sources_map[qx1][qy1];
	}else{
		beta=-(Ta+Tc);
		gamma=.5*(pow(Ta,2)+pow(Tc,2)-(tam_p/pow(velocity_maps[asi][nx][ny],2)));
		rhs[nx][ny]=.5*(-beta+sqrt(pow(beta,2)-4*gamma));
		/*
		if(qc1<qc2){
			sources_map[nx][ny] = sources_map[qx1][qy1];
		}else if(qc1>qc2){
			sources_map[nx][ny] = sources_map[qx2][qy2];
		}else{
			if(sources_map[qx1][qy1]!=sources_map[qx2][qy2]){
				min = INF_VAL;
				for(int i=0; i<move_x.size(); i++){
					_x = nx+ move_x[i]; _y = ny+ move_y[i];
					if(_x>=0 && _x<sx && _y>=0 && _y<sy){
						if(sources_map[_x][_y]>=0 && min > rhs[_x][_y]){
							min = rhs[_x][_y];
							sources_map[nx][ny] = sources_map[_x][_y];
						}
					}
				}
			}
		}
		if(sources_map[qx1][qy1]!=sources_map[qx2][qy2]){
			min = INF_VAL;
			for(int i=0; i<move_x.size(); i++){
				_x = nx+ move_x[i]; _y = ny+ move_y[i];
				if(_x>=0 && _x<sx && _y>=0 && _y<sy){
					if(sources_map[_x][_y]>=0 && min > rhs[_x][_y]){
						min = rhs[_x][_y];
						sources_map[nx][ny] = sources_map[_x][_y];
					}
				}
			}
		}
		*/
		
	}
	/*
	// Quién será mi vecino
	min = INF_VAL;
	for(int i=0; i<move_x.size(); i++){
		_x = nx+ move_x[i]; _y = ny+ move_y[i];
		if(_x>=0 && _x<sx && _y>=0 && _y<sy){
			if(sources_map[_x][_y] >= 0 && min > rhs[_x][_y]){
				min = rhs[_x][_y];
				sources_map[nx][ny] = sources_map[_x][_y];
			}
		}
	}
	*/
	// Quién será mi vecino (Cuando calcule los caminos reales sobre el gradiente)
	min = INF_VAL;
	for(int i=0; i<move_x.size(); i++){
		_x = nx + move_x[i]; _y = ny + move_y[i];
		if(_x>=0 && _x<sx && _y>=0 && _y<sy){
			if(sources_map[_x][_y] >= 0){
				if(min > rhs[_x][_y]){
					min = rhs[_x][_y];
					sources_map[nx][ny] = sources_map[_x][_y];
				}
				// Actualizar la adyacencia
				if(areas.size()){
					adjacency[sources_map[_x][_y]][sources_map[nx][ny]] = true;
					adjacency[sources_map[nx][ny]][sources_map[_x][_y]] = true;
					if(da[sources_map[nx][ny]][sources_map[_x][_y]] > rhs[nx][ny]){
						xa[sources_map[nx][ny]][sources_map[_x][_y]] = nx;
						ya[sources_map[nx][ny]][sources_map[_x][_y]] = ny;
						da[sources_map[nx][ny]][sources_map[_x][_y]] = rhs[nx][ny];
						xa[sources_map[_x][_y]][sources_map[nx][ny]] = nx;
						ya[sources_map[_x][_y]][sources_map[nx][ny]] = ny;
						da[sources_map[_x][_y]][sources_map[nx][ny]] = rhs[nx][ny];
					}
				}
			}
		}
	}

	// Se supone que se ha seleccionado la fuente más cercana para calcular el punto, su valor debe ser menor 
	if(distance_map[nx][ny] < sizes[sources_map[nx][ny]]){
		// No es válido -> anular cálculo
		rhs[nx][ny] = INF_VAL;
		sources_map[nx][ny] = -1;
	}else{
		if(areas.size()){
			// Acumular el área del segmento
			areas[sources_map[nx][ny]]++;
			if(ngoals.size())
			if(goals_map[nx][ny]){ // La posición que se ha calculado es un goal alcanzable
				ngoals[sources_map[nx][ny]]++; // Se añade al segmento
				goals_map[nx][ny] = false; // Se quita para no volver a meterse aquí dentro
				if(dgoal_far[sources_map[nx][ny]] < rhs[nx][ny]){
					xgoal_far[sources_map[nx][ny]] = nx;
					ygoal_far[sources_map[nx][ny]] = ny;
					dgoal_far[sources_map[nx][ny]] = rhs[nx][ny];
				}
				if(dgoal_close[sources_map[nx][ny]] > rhs[nx][ny]){
					xgoal_close[sources_map[nx][ny]] = nx;
					ygoal_close[sources_map[nx][ny]] = ny;
					dgoal_close[sources_map[nx][ny]] = rhs[nx][ny];
				}
			}
		}
	}

	// Almacenar el más lejano de la fuente
	if(sources_map[nx][ny]>=0){
		if(df[sources_map[nx][ny]] < rhs[nx][ny]){
			df[sources_map[nx][ny]] = rhs[nx][ny];
			xf[sources_map[nx][ny]] = nx;
			yf[sources_map[nx][ny]] = ny;
		}
		// Insertar como punto del segmento
		segs_poss[sources_map[nx][ny]].x.push_back(nx);
		segs_poss[sources_map[nx][ny]].y.push_back(ny);
	}

} // calcular el valor del punto expandido

void generic_FMM::requeue()
{
	pos=presence(nx,ny);
	min=rhs[nx][ny]<v[nx][ny]?rhs[nx][ny]:v[nx][ny];
	if(rhs[nx][ny]<INF_VAL)
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
		sw.insert(sw.begin()+cont,asi);
		ws++;
	}else if(cw[pos]!=min){ // si ya estaba
		// primero lo borro
		xw.erase(xw.begin()+pos);
		yw.erase(yw.begin()+pos);
		cw.erase(cw.begin()+pos);
		sw.erase(sw.begin()+pos);
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
		sw.insert(sw.begin()+cont-1,asi);
		// ERES TONTO, QUITAS UNO E INSERTAS OTRO, EL TAMAÑO SIGUE SIENDO EL MISMO
		//ws++;
	}
} // reordenar el frente

vector<vector<float> > generic_FMM::get_rhs()
{
	return rhs;
}

vector<vector<int>> generic_FMM::get_sources_map()
{
	return sources_map;
}

Poss<int> generic_FMM::get_far_pos()
{
	Poss<int> res;
	res.x = xf; res.y = yf;
	return res;
}

Poss<int> generic_FMM::get_far_pos(vector<float> &df)
{
	Poss<int> res;
	res.x = xf; res.y = yf;
	df = this->df;
	return res;
}

vector<int> generic_FMM::get_areas()
{
	return areas;
}

vector<vector<bool>> generic_FMM::get_adjacency()
{
	return adjacency;
}

vector<int> generic_FMM::get_ngoals()
{
	return ngoals;
}

Poss<int> generic_FMM::get_goals_far()
{
	Poss<int> res;
	res.x = xgoal_far; res.y = ygoal_far;
	return res;
}

Poss<int> generic_FMM::get_goals_far(vector<float> &d)
{
	Poss<int> res;
	res.x = xgoal_far; res.y = ygoal_far;
	d = this->dgoal_far;
	return res;
}

Poss<int> generic_FMM::get_goals_close()
{
	Poss<int> res;
	res.x = xgoal_close; res.y = ygoal_close;
	return res;
}

Poss<int> generic_FMM::get_goals_close(vector<float> &d)
{
	Poss<int> res;
	res.x = xgoal_close; res.y = ygoal_close;
	d = this->dgoal_close;
	return res;
}

vector<Poss<int>> generic_FMM::get_adjacent_pos()
{
	vector<Poss<int>> res(sizes.size());
	for(int i=0; i<sizes.size(); i++){
		res[i].x = xa[i];
		res[i].y = ya[i];
	}
	return res;
}

vector<Poss<int>> generic_FMM::get_adjacent_pos(vector<vector<float>> &d)
{
	vector<Poss<int>> res(sizes.size());
	for(int i=0; i<sizes.size(); i++){
		res[i].x = xa[i];
		res[i].y = ya[i];
	}
	d = this->da;
	return res;
}

vector<Poss<int>> generic_FMM::get_segments_pos()
{
	return segs_poss;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
// FUNCIONES QUE NO PERTENECEN A LA CLASE FMM
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// UTILIDADES GENERALES PARA GFMM
//-----------------------------------------------------------------------------------

vector<vector<vector<float>>> GFMM_util::initialize_velocity_maps(vector<vector<float>> static_grid, vector<float> velocities)
{
	vector<vector<vector<float>>> velocity_maps(velocities.size(),static_grid);
	for(int i=0; i<velocity_maps[0].size(); i++)
        for(int j=0; j<velocity_maps[0][0].size(); j++)
        	for(int k=0; k<velocities.size(); k++)
            	velocity_maps[k][i][j] *= velocities[k];
    return velocity_maps;
} // Inicializar los mapas de velocidad de los fuentes

void GFMM_util::set_velocity_maps(vector<vector<vector<float>>> &velocity_maps, vector<float> velocities)
{
	for(int i=0; i<velocity_maps[0].size(); i++)
        for(int j=0; j<velocity_maps[0][0].size(); j++)
        	for(int k=0; k<velocities.size(); k++)
        		if(velocity_maps[k][i][j]) // No es un obstáculo
            		velocity_maps[k][i][j] = velocities[k];
} // Fijar las velocidades de las fuentes

void GFMM_util::set_velocity_maps(vector<vector<vector<float>>> &velocity_maps, Poss<int> free_poss, vector<float> velocities)
{
	for(int i=0; i<free_poss.x.size(); i++)
		for(int j=0; j<velocities.size(); j++)
			velocity_maps[j][free_poss.x[i]][free_poss.y[i]] = velocities[j];
} // Fijar las velocidades de las fuentes

vector<int> GFMM_util::goals_in_segs(Poss<int> goals, vector<vector<int>> segments, int nsegs)
{
    vector<int> res(nsegs,0);
    for(int i=0; i<goals.x.size(); i++)
    	if(segments[goals.x[i]][goals.y[i]] >= 0)
    		res[segments[goals.x[i]][goals.y[i]]]++;
    return res;
} // Cantidad de goals que hay dentro de los segmentos

vector<Poss<int>> GFMM_util::poss_in_segs(Poss<int> poss, vector<vector<int>> segments, int nsegs)
{
    vector<Poss<int>> res(nsegs);
    for(int i=0; i<poss.x.size(); i++)
    	if(segments[poss.x[i]][poss.y[i]] >= 0){
    		res[segments[poss.x[i]][poss.y[i]]].x.push_back(poss.x[i]);
    		res[segments[poss.x[i]][poss.y[i]]].y.push_back(poss.y[i]);
    	}
    return res;
} // Cantidad de goals que hay dentro de los segmentos

vector<float> GFMM_util::goals_dispersion(Poss<int> goals, vector<vector<int>> segments, vector<vector<float>> grad, int nsegs)
{
    vector<float> res(nsegs, 0);
    vector<int> ng(nsegs,0);
    for(int i=0; i<goals.x.size(); i++)
    	if(segments[goals.x[i]][goals.y[i]]>=0){
    		ng[segments[goals.x[i]][goals.y[i]]]++;
    		res[segments[goals.x[i]][goals.y[i]]] += grad[goals.x[i]][goals.y[i]];
    	}
	for(int i=0; i<nsegs; i++)
		if(ng[i]) res[i]/=ng[i];
    return res;
} // Dispersión "estimada" de los goals dentro de los segmentos

generic_FMM GFMM_util::obj_initialization(vector<int> x, vector<int> y, vector<vector<float>> grid)
{
	vector<vector<vector<float>>> velocity_maps(x.size(), grid);
	set_velocity_maps(velocity_maps, vector<float>(x.size(), 1));

	generic_FMM res(x, y, velocity_maps, grid, vector<int>(x.size(),0) );
	//generic_FMM *res; 
	//res = new generic_FMM(x, y, velocity_maps, grid, sizes);

	res.areas_initialization(); // Para poder calcular las adyacencias, sus puntos y las respectivas distancias
	res.goals_initialization(vector<int>(0),vector<int>(0)); // Hace falsta para redimensionar el mapa de goals

	return res;
} // Inicialización del objeto GFMM (con las variables de inicialización básicas: equipo homogéneo)

generic_FMM GFMM_util::obj_initialization(vector<int> x, vector<int> y, vector<float> velocities, vector<int> sizes, vector<vector<float>> grid)
{
	vector<vector<vector<float>>> velocity_maps(x.size(), grid);
	set_velocity_maps(velocity_maps, velocities);

	generic_FMM res(x, y, velocity_maps, grid, sizes);
	//generic_FMM *res; 
	//res = new generic_FMM(x, y, velocity_maps, grid, sizes);

	res.areas_initialization(); // Para poder calcular las adyacencias, sus puntos y las respectivas distancias
	res.goals_initialization(vector<int>(0),vector<int>(0)); // Hace falsta para redimensionar el mapa de goals

	return res;
} // Inicialización del objeto GFMM (SIN objetivos)

generic_FMM GFMM_util::obj_initialization(vector<int> x, vector<int> y, vector<float> velocities, vector<int> sizes, vector<vector<float>> grid, vector<int> xg, vector<int> yg)
{
	vector<vector<vector<float>>> velocity_maps(x.size(), grid);
	set_velocity_maps(velocity_maps, velocities);

	generic_FMM res(x, y, velocity_maps, grid, sizes);
	//generic_FMM res; 
	//res = new generic_FMM(x, y, velocity_maps, grid, sizes);

	res.areas_initialization(); // Para poder calcular las adyacencias, sus puntos y las respectivas distancias
	res.goals_initialization(xg, yg); // Hace falta para redimensionar el mapa de goals

	return res;
} // Inicialización del objeto GFMM (CON objetivos)

//-----------------------------------------------------------------------------------
// UTILIDADES PARA LOS SEGMENTOS OBTENIDOS
//-----------------------------------------------------------------------------------
vector<Poss<int>> fmm_segment::segments_points(vector<vector<int>> segments, int segs)
{
	// YA QUE ESTOY, METO LOS OBSTÁCULOS
	vector<Poss<int>> res(segs);
	for(int i=0; i<segments.size(); i++){
		for(int j=0; j<segments[0].size(); j++){
			if(segments[i][j]>=0){
				res[segments[i][j]].x.push_back(i);
				res[segments[i][j]].y.push_back(j);
			}
		}
	}
	return res;
}


vector<vector<int>> fmm_segment::generate_segments_graph(vector<vector<bool>> adjacency, vector<int> sizes)
{
    vector<vector<int>> res(adjacency.size(),vector<int>(adjacency.size(),0));
    for(int i=0; i<adjacency.size(); i++){
        for(int j=i+1; j<adjacency.size(); j++){
            if(adjacency[i][j]){
                if(!sizes[i]){
                    res[i][j] = 1;
                    res[j][i] = 1;
                }else{
                    res[i][j] = sizes[j];
                    res[j][i] = sizes[i];
                }
            }
        }
    }    

    return res;   
}

vector<vector<Poss<int>>> fmm_segment::frontier_points(vector<vector<int>> segments, vector<vector<bool>> graph)
{
    vector<vector<Poss<int>>> res(graph.size(), vector<Poss<int>>(graph.size()));
    int x, y;

    int sx = segments.size(), sy = segments[0].size();
    vector<vector<bool>> td(sx,vector<bool>(sy,true));
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            for(int k=0; k<move_x.size(); k++){
                x = i+move_x[k]; y = j+move_y[k];
                if(x>=0 && x<sx && y>=0 && y<sy){
                    // No son un obstáculo y no pertenecen al mismo segmento
                    if(segments[i][j]>=0 && segments[x][y]>=0 && segments[i][j] != segments[x][y]){
                        if(td[i][j]){
                            res[segments[i][j]][segments[x][y]].x.push_back(i); res[segments[i][j]][segments[x][y]].y.push_back(j);
                            res[segments[x][y]][segments[i][j]].x.push_back(i); res[segments[x][y]][segments[i][j]].y.push_back(j);
                        }
                        if(td[x][y]){
                            res[segments[i][j]][segments[x][y]].x.push_back(x); res[segments[i][j]][segments[x][y]].y.push_back(y);
                            res[segments[x][y]][segments[i][j]].x.push_back(x); res[segments[x][y]][segments[i][j]].y.push_back(y);
                        }
                        td[i][j]=false; td[x][y]=false;
                    }
                }
            }
        }
    }

    return res;
} // Obtener los puntos que conforman las fronteras entre los segmentos

vector<Poss<int>> fmm_segment::contour_points(vector<vector<int>> segments, vector<vector<bool>> graph)
{
    vector<Poss<int>> res(graph.size());
    int x, y;

    int sx = segments.size(), sy = segments[0].size();
    bool cont;
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            for(int k=0; k<move_x.size(); k++){
                x = i + move_x[k]; y = j + move_y[k];
                if(x>=0 && x<sx && y>=0 && y<sy){
                    // El punto no pertenece al mismo segmento
                    if(segments[i][j]>=0 && segments[i][j] != segments[x][y]){
                        res[segments[i][j]].push(i,j);
                        break;
                    }
                }
            }
        }
    }

    return res;
} // Obtener los puntos que conforman las fronteras entre los segmentos


// Obtener la distancia máxima de cada segmento
vector<float> fmm_segment::max_seg_dist(Poss<int> centroids, vector<Poss<int>> contour_poss)
{
	vector<float> res(centroids.x.size(),0);
	float cd;
	for(int i=0; i<centroids.x.size(); i++){
        for(int j=0; j<contour_poss[i].x.size(); j++){
            cd = sqrt(pow(centroids.x[i] - contour_poss[i].x[j], 2) + pow(centroids.y[i] - contour_poss[i].y[j] ,2));
            if(res[i]<cd){
                res[i] = cd;
            }
        }
    }
    return res;
}

vector<vector<float>> fmm_segment::distance_between_segments(Poss<int> centroids, vector<vector<bool>> graph)
{
	vector<vector<float>> res;
	if(!graph.size()) return res;
	res.resize(graph.size(), vector<float>(graph[0].size(), 0));
	for(int i=0; i<res.size(); i++)
		for(int j=0; j<res[i].size(); j++)
			if(graph[i][j]){
				res[i][j] = hypot(centroids.x[i] - centroids.x[j], centroids.y[i] - centroids.y[j]);
				res[j][i] = res[i][j];
			}
	return res;
}

fmm_segment::Segments fmm_segment::compute_segments(vector<vector<float>> grid, vector<vector<float>> grad)
{
	fmm_segment::Segments res;

	res.centroids = compute_gradient_centroids(grad, 1);
    vector<vector<vector<float>>> velocity_maps(res.centroids.x.size(), grad);
    vector<int> sizes(res.centroids.x.size(), 0);

    generic_FMM gen_gr(res.centroids.x, res.centroids.y, velocity_maps, grad, sizes);
    gen_gr.areas_initialization(); // Para poder calcular las adyacencias, sus puntos y las respectivas distancias
    //vector<vector<float>> rooms_grad = gen_gr.compute_gradient_();
    res.segs_grad = gen_gr.compute_gradient_();
    res.map = gen_gr.get_sources_map();
    res.graph = gen_gr.get_adjacency();
    res.frontier_poss = fmm_segment::frontier_points(res.map, res.graph);
    res.contour_poss = fmm_segment::contour_points(res.map, res.graph);

    res.poss = fmm_segment::segments_points(res.map, res.centroids.x.size());

    return res;
} // Calcular los segmentos y todas sus variables

fmm_segment::Segments fmm_segment::compute_segments(Poss<int> centroids, vector<vector<float>> grad)
{
	fmm_segment::Segments res;

	res.centroids = centroids;
    vector<vector<vector<float>>> velocity_maps(res.centroids.x.size(), grad);
    vector<int> sizes(res.centroids.x.size(), 0);

    generic_FMM gen_gr(res.centroids.x, res.centroids.y, velocity_maps, grad, sizes);
    gen_gr.areas_initialization(); // Para poder calcular las adyacencias, sus puntos y las respectivas distancias
    //vector<vector<float>> rooms_grad = gen_gr.compute_gradient_();
    res.segs_grad = gen_gr.compute_gradient_();
    res.map = gen_gr.get_sources_map();
    res.graph = gen_gr.get_adjacency();
    res.frontier_poss = fmm_segment::frontier_points(res.map, res.graph);
    res.contour_poss = fmm_segment::contour_points(res.map, res.graph);

    res.poss = fmm_segment::segments_points(res.map, res.centroids.x.size());

    return res;
} // Calcular los segmentos y todas sus variables
