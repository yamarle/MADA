
#include <mada/util/geometry.hpp>

	float geometry::distancef(float x1, float y1, float x2, float y2)
	{
		return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
	}

	Poss<int> geometry::allRayPoss(int x, int y, Poss<int> poss)
	{
		Poss<int> res;
	    for(int i=0; i<poss.x.size(); i++){
	        Poss<int> p = bresenham::points(x, y, poss.x[i], poss.y[i]);
	        for(int j=0; j<p.x.size(); j++){
	            res.push(p.x[j], p.y[j]);
	        }
	    }
	    return res;
	}

	Poss<int> geometry::allRayPoss(int x, int y, Poss<int> poss, double vr)
	{
		Poss<int> res;
	    for(int i=0; i<poss.x.size(); i++){
	        Poss<int> p = bresenham::points(x, y, poss.x[i], poss.y[i]);
	        for(int j=0; j<p.x.size(); j++){
	        	if(hypot(x - p.x[j], y - p.y[j]) <= vr){
	            	res.push(p.x[j], p.y[j]);
	            }
	        }
	    }
	    return res;
	}

	Poss<int> geometry::allRayPoss(int x, int y, Poss<int> poss, vector<vector<int>> &values_map)
	{
		Poss<int> res;
	    for(int i=0; i<poss.x.size(); i++){
	        Poss<int> p = bresenham::points(x, y, poss.x[i], poss.y[i]);
	        for(int j=0; j<p.x.size(); j++){
	            res.push(p.x[j], p.y[j]);
	            values_map[p.x[j]][p.y[j]]++;
	        }
	    }
	    return res;
	}

	//float angle(float x1, float y1, float x2, float y2, float x3, float y3)
	float geometry::angle(int x1, int y1, int x2, int y2, int x3, int y3)
	{
		float p12 = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
		float p13 = sqrt(pow(x1-x3,2)+pow(y1-y3,2));
		float p23 = sqrt(pow(x2-x3,2)+pow(y2-y3,2));
		float res = acos((pow(p12,2)+pow(p13,2)-pow(p23,2))/(2*p12*p13));
		if(isnan(res)) return 0; // Para que no devuelva NaN
		else return res;
	} // Ángulo entre 3 puntos

	float geometry::real_angle(float x1, float y1, float x2, float y2, float x3, float y3)
	{
		float p12x = x1-x2; float p12y = y1-y2;
		float p13x = x1-x3; float p13y = y1-y3;
		float dot = p12x*p13x + p12y*p13y;
		float cross = p12x*p13y + p12y*p13x;
		float alpha = atan2(cross, dot);
		return floor(alpha * 180. / M_PI + 0.5);
	} // Ángulo entre 3 puntos

	float geometry::ang_trans(float angle, int orient)
	{
		if(orient == 2) return angle;
        else if(orient == 1) return 2*M_PI - angle;
        else return angle;
	} // Transformar ángulo + orientación a ángulo sobre "la circunferencia"

	float geometry::rad2ang(float rad)
	{
		return rad*180/M_PI;
	}

	float geometry::ang2rad(float angle)
	{
		return angle*M_PI/180;
	}

	int geometry::choose_left(Poss<int> segment, Poss<int> points)
	{
		int res;
		int N = points.x.size();
		vector<float> ang(N); vector<int> orient(N);
    	vector<float> rang(N);
    	float min = M_PI*10;
    	for(int i=0; i<N; i++){
	        ang[i] = geometry::angle(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        orient[i] = geometry::orientation(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        rang[i] = geometry::ang_trans(ang[i], orient[i]);
	        if(min > rang[i]){
	        	min = rang[i];
	        	res = i;
	        }
	    }
		return res;
	} // Seleccionar el punto más a la izquierda

	int geometry::choose_left(Poss<int> segment, Poss<int> points, float lb, float ub)
	{
		int res = -1;
		int N = points.x.size();
		vector<float> ang(N); vector<int> orient(N);
    	vector<float> rang(N);
    	float min = M_PI*10;
    	for(int i=0; i<N; i++){
	        ang[i] = geometry::angle(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        orient[i] = geometry::orientation(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        rang[i] = geometry::ang_trans(ang[i], orient[i]);
	        if(rang[i]>=lb && rang[i]<=ub && min > rang[i]){
	        	min = rang[i];
	        	res = i;
	        }
	    }
		return res;
	} // Seleccionar el punto más a la izquierda conmrendido entre ángulos lb - ub

	int geometry::choose_right(Poss<int> segment, Poss<int> points)
	{
		int res;
		int N = points.x.size();
		vector<float> ang(N); vector<int> orient(N);
    	vector<float> rang(N);
    	float max = 0;
    	for(int i=0; i<N; i++){
	        ang[i] = geometry::angle(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        orient[i] = geometry::orientation(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        rang[i] = geometry::ang_trans(ang[i], orient[i]);
	        if(max < rang[i]){
	        	max = rang[i];
	        	res = i;
	        }
	    }
		return res;
	} // Seleccionar el punto más a la derecha

	int geometry::choose_right(Poss<int> segment, Poss<int> points, float lb, float ub)
	{
		int res = -1;
		int N = points.x.size();
		vector<float> ang(N); vector<int> orient(N);
    	vector<float> rang(N);
    	float max = 0;
    	for(int i=0; i<N; i++){
	        ang[i] = geometry::angle(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        orient[i] = geometry::orientation(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        rang[i] = geometry::ang_trans(ang[i], orient[i]);
	        if(rang[i]>=lb && rang[i]<=ub && max < rang[i]){
	        	max = rang[i];
	        	res = i;
	        }
	    }
		return res;
	} // Seleccionar el punto más a la derechas

	vector<float> geometry::compute_angles(Poss<int> segment, Poss<int> points)
	{
		int N = points.x.size();
		vector<float> ang(N), rang(N); vector<int> orient(N);
    	for(int i=0; i<N; i++){
	        ang[i] = geometry::angle(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        orient[i] = geometry::orientation(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	        rang[i] = geometry::ang_trans(ang[i], orient[i]);
	        //cout<<"("<<segment.x[0]<<", "<<segment.y[0]<<") - ("<<segment.x[1]<<", "<<segment.y[1]<<") - ("<<points.x[i]<<", "<<points.y[i]<<"): ";
	        //cout<<ang[i]<<" - "<<orient[i]<<" - "<<rang[i]<<endl;
	    }
	    return rang;
	}

	Poss<int> geometry::clockwise_sort(Poss<int> segment, Poss<int> points)
	{
		vector<float> angles = compute_angles(segment, points);
		vector<int> order;
		sort_vect(angles, 1, order);
		points.x = sort_vect(points.x, order); points.y = sort_vect(points.y, order);
		return points;
	}

	Poss<int> geometry::counter_clockwise_sort(Poss<int> segment, Poss<int> points)
	{
		vector<float> angles = compute_angles(segment, points);
		vector<int> order;
		sort_vect(angles, 0, order);
		points.x = sort_vect(points.x, order); points.y = sort_vect(points.y, order);
		return points;
	}

	Poss<int> geometry::bounding_box(Poss<int> points)
	{
		Poss<int> res;
		if(points.x.size()<3) return points;

		// Inicializar el contorno (extremos en cada dirección)
		vector<int> ind(4);
		// Arriba
		min_v(points.y, ind[0]);
		//res.x.push_back(points.x[ind[0]]); res.y.push_back(points.y[ind[0]]);
		// Derecha
		max_v(points.x, ind[1]);
		//res.x.push_back(points.x[ind[1]]); res.y.push_back(points.y[ind[1]]);
		// Izquierda
		min_v(points.x, ind[2]);
		//res.x.push_back(points.x[ind[2]]); res.y.push_back(points.y[ind[2]]);
		// Abajo
		max_v(points.y, ind[3]);
		/*
		for(int i=0; i<3; i++)
			if(ind[3] == ind[i]){
				return res;
			}
			*/
		//res.x.push_back(points.x[ind[3]]); res.y.push_back(points.y[ind[3]]);

		res.push(points.x[ind[2]], points.y[ind[3]]); // Arriba izquierda
		res.push(points.x[ind[1]], points.y[ind[3]]); // Arriba derecha
		res.push(points.x[ind[1]], points.y[ind[0]]); // Abajo derecha
		res.push(points.x[ind[2]], points.y[ind[0]]); // Abajo izquierda

		return res;
	}

	float geometry::angles(int x, int y, Poss<int> polygon)
	{
		float res = 0;
		for(int i=0; i<polygon.x.size()-1; i++){
			res += angle(x, y, polygon.x[i], polygon.y[i], polygon.x[i+1], polygon.y[i+1]);
		}
		res += angle(x, y, polygon.x[polygon.x.size()-1], polygon.y[polygon.y.size()-1], polygon.x[0], polygon.y[0]);
		return res;
	} // Sumatorio de ángulos entre un punto y un polígono

	Poss<int> geometry::polygon_edges(Poss<int> polygon)
	{
	    Poss<int> pos;
	    for(int j=0; j<polygon.x.size()-1; j++){
	        pos.append(bresenham::points(polygon.x[j], polygon.y[j], polygon.x[j+1], polygon.y[j+1]));
	    }
	    if(polygon.x.size()>2) // Por no duplicar los puntos de la arista, si el polígono en relidad es una línea
	        pos.append(bresenham::points(polygon.x[polygon.x.size()-1], polygon.y[polygon.y.size()-1], polygon.x[0], polygon.y[0]));
	    return pos;
	} // Obtener todos los puntos del contorno del poligono

	Poss<int> geometry::convex_hull(Poss<int> points)
	{
		if(points.x.size()<3) return points;
    
	    Poss<int> contour;

	    int ind;
	    // Seleccionar el punto más bajo
	    min_v(points.x, ind);
	    contour.x.push_back(points.x[ind]); contour.y.push_back(points.y[ind]);
	    int n = 0;

	    vector<bool> done(points.x.size(),false);
	    done[ind] = true;

	    // Ir seleccionando el vacino que quede más a la derecha
	    Poss<int> segment; segment(2);
	    float ang, bang;
	    int orient;
	    while(1){
	        bang = 0;
	        segment.x[0] = contour.x[n];
	        segment.y[0] = contour.y[n];
	        segment.x[1] = n ? contour.x[n-1] : contour.x[n];
	        segment.y[1] = n ? contour.y[n-1] : 0;

	        ind = points.x.size();
	        for(int i=0; i<points.x.size(); i++){
	            ang = angle(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	            orient = orientation(segment.x[0], segment.y[0], segment.x[1], segment.y[1], points.x[i], points.y[i]);
	            if((orient == 0 || orient == 2) && bang < ang){
	                bang = ang;
	                ind = i;
	            }
	        }

	        if(ind < points.x.size()){
	            contour.x.push_back(points.x[ind]); contour.y.push_back(points.y[ind]);
	            n++;
	            if(!done[ind]) done[ind] = true;
	            else{
	            	//cout<<"Cierre"<<endl;
	            	break; // Se ha cerrado el círculo de la vida
	            }
	        }else{
	        	//cout<<"no encontrado"<<endl;
	            break;
	        }
	    }

		return contour;
	}

	Poss<int> geometry::circle_poss_on_grid(int x, int y, int srange, int sx, int sy)
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

	Poss<int> geometry::circle_poss_on_grid(int x, int y, int srange, vector<vector<float>> grid)
	{   
	    // Reducir los puntos que se van a evaluar
	    int sx = grid.size(), sy = grid[0].size();
	    int xl, xu, yl, yu;
	    xl = (x-srange) > 0 ? (x-srange) : 0;
	    xu = (x+srange) < sx-1 ? (x+srange) : sx-1;
	    yl = (y-srange) > 0 ? (y-srange) : 0;
	    yu = (y+srange) < sy-1 ? (y+srange) : sy-1;
	    Poss<int> res;
	    for(int i=xl; i<=xu; i++){
	        for(int j=yl; j<=yu; j++){
	            if(sqrt(pow(x-i,2)+pow(y-j,2))<=srange && grid[i][j]){ // Dentro del rango y no es un obstáculo
	                res.x.push_back(i);
	                res.y.push_back(j);
	            }
	        }
	    }
	    return res;
	} // Todas las posiciones del círculo sobre el grid (sin incluir obstáculos)

	Poss<float> geometry::circle_contour(float x, float y, float radius, float dt)
	{
		Poss<float> res;
		for(float ang=0; ang<=2*M_PI; ang+=dt){
			res.x.push_back((float)x + radius * cos(ang));
			res.y.push_back((float)y + radius * sin(ang));
		}
		return res;
	} // Solo el contorno del círculo


	Poss<int> geometry::rectangle_poss_on_grid(vector<int> dimensions, vector<vector<float>> grid)
	{
		Poss<int> res;
        for(int i=dimensions[0]; i<dimensions[0]+dimensions[2]; i++){
            if(i>=0 && i<grid.size())
            for(int j=dimensions[1]; j<dimensions[1]+dimensions[3]; j++){
                if(j>=0 && j<grid[0].size())
                if(grid[i][j]){
                	res.x.push_back(i);	res.y.push_back(j);
                }
            }
    	}
    	return res;
	}// Todas las posiciones del rectángulo sobre el grid

	Poss<int> geometry::rectangle_poss_on_grid(vector<float> dimensions, vector<vector<float>> grid)
	{
		Poss<int> res;
        for(int i=dimensions[0]; i<dimensions[0]+dimensions[2]; i++){
            if(i>=0 && i<grid.size())
            for(int j=dimensions[1]; j<dimensions[1]+dimensions[3]; j++){
                if(j>=0 && j<grid[0].size())
                if(grid[i][j]){
                	res.x.push_back(i);	res.y.push_back(j);
                }
            }
    	}
    	return res;
	}// Todas las posiciones del rectángulo sobre el grid

	Poss<int> geometry::projectPoints(int xs, int ys, int xe, int ye, double dist)
	{
		Poss<int> line_poss;

        // Calcular la distancia total entre P1 y P2
        double total_dist = hypot(xe - xs, ye - ys);

        // Número de puntos a generar entre P1 y P2 (incluyendo ambos)
        int num_points = std::max(1, static_cast<int>(total_dist / dist));

        // Dirección (vector unitario) desde P1 hacia P2
        //double direction_x = (xe - xs) / total_dist;
        //double direction_y = (ye - ys) / total_dist;

        double x, y, t;

        // Generar los puntos a lo largo de la recta
        for (int i = 0; i <= num_points; ++i) {
            t = static_cast<double>(i) / num_points; // Factor de interpolación entre 0 y 1
            x = xs + t * (xe - xs);
            y = ys + t * (ye - ys);
            line_poss.push(x, y);
        }

        return line_poss;
	}

	Path geometry::refinePath(Path path, vector<vector<float>> grid, float radius)
	{
	    Path res;
	    if(path.tam == 0) return res;
	    bool waypointCollision;
	    int li, ps, _li;
	    // Elimino todos los puntos dentro del radio del robot
	    while(path.tam>1 && hypot(path.x[0]-path.x[1], path.y[0]-path.y[1])>radius){
	        path.pop();
	    }
	    ps = 0;
	    Poss<int> line_poss, _line_poss;
	    for(int i=1; i<path.tam-1; i++){
	        // Proyecto la línea hasta el punto siguiente
	        line_poss = geometry::projectPoints(path.x[ps], path.y[ps], path.x[i], path.y[i], radius);
	        // Compruebo si está en colisión
	        waypointCollision = false;
	        for(int j=0; j<line_poss.x.size(); j++){
	            if(grid[line_poss.x[j]][line_poss.y[j]] == 0){
	                waypointCollision = true;
	                li = j;
	                break;
	            }
	        }

	        // Colisión con obstáculo
	        if(waypointCollision == true){
	            ps = i;
	            if(_line_poss.x.size()){ // Hay una recta almacenada
	                // La inserto
	                for(int j=0; j<_line_poss.x.size(); j++){
	                    res.push_back(_line_poss.x[j], _line_poss.y[j]);
	                }
	            }
	        }
	        _line_poss = line_poss;
	    }

	    if(path.tam>1){
	        line_poss = geometry::projectPoints(path.x[ps], path.y[ps], path.x[path.tam-1], path.y[path.tam-1], radius);
	        for(int j=0; j<_line_poss.x.size(); j++){
	            res.push_back(line_poss.x[j], line_poss.y[j]);
	        }
	    }

	    return res;
	}
