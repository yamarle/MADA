
#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include "funciones.hpp"
#include "bresenham.hpp"
#include "path.hpp"

namespace geometry{

	static vector<int> nx4={0, 1, 0,-1};
	static vector<int> ny4={1, 0,-1, 0};

	static vector<int> nx8={0, 1, 1, 1, 0,-1,-1,-1};
	static vector<int> ny8={1, 1, 0,-1,-1,-1, 0, 1};

	static vector<int> nx={0, 1, 1, 1, 0,-1,-1,-1};
	static vector<int> ny={1, 1, 0,-1,-1,-1, 0, 1};

	template<typename T>
	Poss<T> sort_points(T xc, T yc, Poss<T> poss)
	{
	    // Obtener los ángulos
	    vector<float> angles(poss.x.size());
	    for(int i=0; i<poss.x.size(); i++){
	        angles[i] = atan2(poss.x[i] - xc, poss.y[i] - yc);
	    }

	    Poss<T> res; res(poss.x.size());
	    float min; int ind;
	    for(int i=0; i<poss.x.size(); i++){
	        min = INF;
	        for(int j=0; j<poss.x.size(); j++){
	            if(min > angles[j]){
	                min = angles[j];
	                ind = j;
	            }
	        }
	        res.x[i] = poss.x[ind]; res.y[i] = poss.y[ind];
	        angles[ind] = INF;
	    }
	    
	    return res;
	}

	template<typename T>
	Poss<T> sort_points(T xc, T yc, Poss<T> poss, bool cw)
	{
	    // Obtener los ángulos
	    vector<float> angles(poss.x.size());
	    for(int i=0; i<poss.x.size(); i++)
	        angles[i] = atan2(poss.x[i] - xc, poss.y[i] - yc);

	    // Ordernar los puntos
	    vector<int> order;
	    if(cw) sort_vect(angles, 0, order);
	    else sort_vect(angles, 1, order);

		poss.x = sort_vect(poss.x, order); poss.y = sort_vect(poss.y, order);
	    
	    return poss;
	}

	
	template<typename T>	
	float distance(T x1, T y1, T x2, T y2)
	{
		return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
	}

	template<typename T>	
	vector<T> distances(T x1, T y1, vector<T> x2, vector<T> y2)
	{
		vector<T> res(x2.size(),0);
		for(int i=0; i<x2.size(); i++)
			res[i] = distance(x1, y1, x2[i], y2[i]);
		return res;
	}

	float distancef(float x1, float y1, float x2, float y2);

	template <typename T>
	void centroid(Poss<T> points, T &x, T &y)
	{
	    if(!points.x.size()) return;
	    x = y = 0;
	    for(int i=0; i<points.x.size(); i++){
	        x += points.x[i]; y += points.y[i];
	    }
	    x /= points.x.size(); y /= points.y.size();
	}

	// Posiciones para el "laser"
	Poss<int> allRayPoss(int x, int y, Poss<int> poss);
	Poss<int> allRayPoss(int x, int y, Poss<int> poss, double vr);
	Poss<int> allRayPoss(int x, int y, Poss<int> poss, vector<vector<int>> &values_map);

	template<typename T>
	vector<Poss<int>> allRayPoss_ranges(int x, int y, Poss<int> poss, vector<T> vr)
	{
		vector<Poss<int>> res(vr.size());
		T dist;
	    for(int i=0; i<poss.x.size(); i++){
	        Poss<int> p = bresenham::points(x, y, poss.x[i], poss.y[i]);
	        for(int j=0; j<p.x.size(); j++){
	        	dist = hypot(x - p.x[j], y - p.y[j]);
	        	for(int r=0; r<vr.size(); r++){
		        	if(dist <= vr[r]){
		            	res[r].push(p.x[j], p.y[j]);
		            }
		        }
	        }
	    }
	    return res;
	}

	template<typename T>
	vector<Poss<int>> allRayPoss_ranges(int x, int y, const Poss<int>& poss, const vector<T>& vr, const vector<vector<float>>& grid)
	{
		vector<Poss<int>> res(vr.size());
		T dist;
	    for(int i=0; i<poss.x.size(); i++){
	        Poss<int> p = bresenham::points(x, y, poss.x[i], poss.y[i]);
	        for(int j=0; j<p.x.size(); j++){
	        	if(check_pos_in_grid(p.x[j], p.y[j], grid) && grid[p.x[j]][p.y[j]] == 0) break;
	        	dist = hypot(x - p.x[j], y - p.y[j]);
	        	for(int r=0; r<vr.size(); r++){
		        	if(dist <= vr[r]){
		            	res[r].push(p.x[j], p.y[j]);
		            }
		        }
	        }
	    }
	    return res;
	}


	// Funciones de ángulos
	//float angle(float x1, float y1, float x2, float y2, float x3, float y3);
	float angle(int x1, int y1, int x2, int y2, int x3, int y3);
	float real_angle(float x1, float y1, float x2, float y2, float x3, float y3);
	float ang_trans(float angle, int orient);
	float rad2ang(float rad);
	float ang2rad(float angle);
	int choose_left(Poss<int> segment, Poss<int> points);
	int choose_left(Poss<int> segment, Poss<int> points, float lb, float ub);
	int choose_right(Poss<int> segment, Poss<int> points);
	int choose_right(Poss<int> segment, Poss<int> points, float lb, float ub);

	vector<float> compute_angles(Poss<int> segment, Poss<int> points);
	Poss<int> clockwise_sort(Poss<int> segment, Poss<int> points);
	Poss<int> counter_clockwise_sort(Poss<int> segment, Poss<int> points);

	template <typename T>
	Poss<T> sort_points(Poss<T> segment, Poss<T> points, bool cw)
	{
		// Obtener ángulos
		vector<float> angles = compute_angles(segment, points);
		// Ordenar
		vector<int> order;
		if(cw) sort_vect(angles, 1, order);
		else sort_vect(angles, 0, order);
		points.x = sort_vect(points.x, order); points.y = sort_vect(points.y, order);
		return points;
	}

	template<typename T>
	bool onSegment(T px, T py, T qx, T qy, T rx, T ry) 
	{ 
	    if(qx <= max(px, rx) && qx >= min(px, rx) && qy <= max(py, ry) && qy >= min(py, ry)) 
	    	return true;
	    return false; 
	} // Comprobar si 3 puntos estan alineados

	template<typename T>
	int orientation(T px, T py, T qx, T qy, T rx, T ry)
	{
		// Recibe:
		// (px, py) - (qx, qy): posiciones del segmento
		// (rx, ry): punto
		// Devuelve
		// 0: puntos alineados
		// 1: sentido de las agujas del reloj
		// 2: sentido contrario de las agujas del reloj
		T val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy);
	    if (val == 0) return 0;  // colinear 
	    return (val > 0) ? 1 : 2; // clock or counterclock wise 
	} // Orientación entre 3 puntos

	template<typename T>
	bool doIntersect(T p1x, T p1y, T q1x, T q1y, T p2x, T p2y, T q2x, T q2y) 
	{ 
	    // Find the four orientations needed for general and 
	    // special cases 
	    int o1 = orientation(p1x, p1y, q1x, q1y, p2x, p2y); 
	    int o2 = orientation(p1x, p1y, q1x, q1y, q2x, q2y); 
	    int o3 = orientation(p2x, p2y, q2x, q2y, p1x, p1y); 
	    int o4 = orientation(p2x, p2y, q2x, q2y, q1x, q1y); 
	  
	    // General case 
	    if (o1 != o2 && o3 != o4) 
	        return true; 
	  
	    // Special Cases 
	    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
	    if (o1 == 0 && onSegment(p1x, p1y, p2x, p2y, q1x, q1y)) return true;
	  
	    // p1, q1 and q2 are colinear and q2 lies on segment p1q1 
	    if (o2 == 0 && onSegment(p1x, p1y, q2x, q2y, q1x, q1y)) return true;
	  
	    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
	    if (o3 == 0 && onSegment(p2x, p2y, p1x, p1x, q2x, q2y)) return true;
	  
	     // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
	    if (o4 == 0 && onSegment(p2x, p2y, q1x, q1y, q2x, q2y)) return true; 
	  
	    return false; // Doesn't fall in any of the above cases 
	} // Comprueba si dos rectas (formadas por los 4 puntos) se intersectan

	template <typename T>
	bool isInsidePolygon_(T x, T y, Poss<T> polygon)
	{ 
	    // There must be at least 3 vertices in polygon[] 
	    int n = polygon.x.size();
	    if (n < 3)  return false; 
	  
	    // Create a point for line segment from p to infinite 
	    //Point extreme = {INF, y}; 
	  
	    // Count intersections of the above line with sides of polygon 
	    int count = 0, i = 0; 
	    do
	    { 
	        int next = (i+1)%n; 
	  
	        // Check if the line segment from 'p' to 'extreme' intersects 
	        // with the line segment from 'polygon[i]' to 'polygon[next]' 
	        //if (doIntersect(polygon[i], polygon[next], p, extreme)) 
	    	if (doIntersect(polygon.x[i], polygon.y[i], polygon.x[next], polygon.y[next], x, y, (T)INF, y)) 
	        { 
	            // If the point 'p' is colinear with line segment 'i-next', 
	            // then check if it lies on segment. If it lies, return true, 
	            // otherwise false 
	            if(orientation(polygon.x[i], polygon.y[i], x, y, polygon.x[next], polygon.y[next]) == 0) 
	               return onSegment(polygon.x[i], polygon.y[i], x, y, polygon.x[next], polygon.y[next]); 
	  
	            count++; 
	        } 
	        i = next; 
	    } while (i != 0); 
	  
	    // Return true if count is odd, false otherwise 
	    return count&1;  // Same as (count%2 == 1) 
	} // Comprueba si un punto se encuentra dentro de un polígono


	template <typename T>
	bool isInsidePolygon(T x, T y, Poss<T> polygon)
	{
		Poss<int> points = polygon;
		int i, j, nvert = polygon.x.size();
		bool c = false;

		for(i = 0, j = nvert - 1; i < nvert; j = i++) {
			if( ( (points.y[i] >= y ) != (points.y[j] >= y) ) &&
			(x <= (points.x[j] - points.x[i]) * (y - points.y[i]) / (points.y[j] - points.y[i]) + points.x[i])
			)
			c = !c;
		}
		return c;
	}

	template <typename T>
	bool isConvex(vector<T> x, vector<T> y)
	{
		// Variables de la orientación
		int co = 0; // Actual
		int fo = 0; // Primer giro
		int p1,p2,p3; // Puntos
		for(int i=0; i<x.size()-3; i++){
			p1 = i; p2 = i+1; p3 = i+2;
			//co = orientation((float)x[p1],(float)y[p1],(float)x[p3],(float)y[p3],(float)x[p2],(float)y[p2]);
			co = orientation(x[p1],y[p1],x[p3],y[p3],x[p2],y[p2]);
			if(fo == 0){ // Todavía no ha habido ningún giro
				fo = co; 
			}else{ // Ya hay una dirección definida
				if(co && fo != co) return false; // No coincide -> no es una región conveza
			}
			//cout<<"("<<x[p1]<<", "<<y[p1]<<") - ("<<x[p2]<<", "<<y[p2]<<") - ("<<x[p3]<<", "<<y[p3]<<"): "<<co<<endl;
		}
		//cin.get();
		return true; // La regíon es convexa
	}

	Poss<int> bounding_box(Poss<int> points);

	float angles(int x, int y, Poss<int> polygon); // Sumatorio de ángulos entre un punto y un polígono
	Poss<int> polygon_edges(Poss<int> polygon); // Todas las posiciones de las aristas del polígono

	Poss<int> convex_hull(Poss<int> points);

	// Círculo
	Poss<int> circle_poss_on_grid(int x, int y, int srange, int sx, int sy); // Todas las posiciones del círculo sobre el grid
	Poss<int> circle_poss_on_grid(int x, int y, int srange, vector<vector<float>> grid);
	Poss<float> circle_contour(float x, float y, float radius, float dt); // Solo el contorno del círculo

	// Rectangulo
	Poss<int> rectangle_poss_on_grid(vector<int> dimensions, vector<vector<float>> grid);   // Todas las posiciones del rectángulo sobre el grid
	Poss<int> rectangle_poss_on_grid(vector<float> dimensions, vector<vector<float>> grid);   // Todas las posiciones del rectángulo sobre el grid


	template <typename T>
	Poss<int> contour(Poss<int> points, vector<vector<T>> graph)
	{
	    Poss<int> contour;

	    // Obtener la envolvente convexa
	    Poss<int> convex_poss = geometry::convex_hull(points);

	    if(convex_poss.x.size()<3) return contour;

	    contour.push(convex_poss.x[0],convex_poss.y[0]); // Seleccionar el primer punto de la envolvente convexa
	    convex_poss.pop(); // Eliminar de la lista

	    Poss<int> segment; segment(2);
	    float ang, cwbang, ccwbang;
	    float bang;
	    int orient;

	    int n = 0, ind = -1, _ind;

	    _ind = points.find(contour.x[0],contour.y[0]);

	    int it = 0, max_it = 10*points.x.size();

	    // Recorrer todos los puntos hasta que todos los puntos de la envolvente pertenezcan al contorno
	    while(true){
	        //cwbang = ccwbang = 0;
	        bang = 0;
	        segment.x[0] = contour.x[n];
	        segment.y[0] = contour.y[n];
	        segment.x[1] = n ? contour.x[n-1] : contour.x[n];
	        segment.y[1] = n ? contour.y[n-1] : 0;

	        //cout<<"*****************************************************************"<<endl;
	        //cout<<"it "<<it<<endl;

	        // Seleccionar los vecinos
	        Poss<int> neigh;
	        for(int i=0; i<points.x.size(); i++){
	            if(_ind != i && graph[_ind][i]){
	            //if(graph[_ind][i]){
	                neigh.push(points.x[i], points.y[i]);
	            }
	        }

	        //cout<<"Segmento"<<endl; segment.show_();
	        //cout<<"Vecinos"<<endl; neigh.show_();

	        // Seleccionar el de la izquierda
	        if(neigh.x.size()>1){
	            vector<float> angulos = geometry::compute_angles(segment, neigh);

	            //sh_vect_h(angulos, "angulos");
	            //cout<<"Seleccionando vecino ..."<<endl;
	            //ind = geometry::choose_left(segment, neigh, 0.01, 2*M_PI);
	            ind = geometry::choose_left(segment, neigh, 0.01, 2*M_PI-0.01); // Para no volver por donde he venido

	            //cout<<"ENCONTRADO "<<ind<< ": ("<<neigh.x[ind]<<", "<<neigh.y[ind]<<")"<<endl;
	            ind = points.find(neigh.x[ind], neigh.y[ind]);
	            //cout<<"ENCONTRADO "<<ind<< ": ("<<points.x[ind]<<", "<<points.y[ind]<<")"<<endl;
	        }else if(neigh.x.size()==1){
	            //cout<<"Hay un único vecino, voy por donde he venido"<<endl;
	            if(contour.x.size()>1) ind = points.find(segment.x[1],segment.y[1]);
	            else{
	                //cout<<"busco al vecino"<<endl;
	                ind = points.find(neigh.x[0],neigh.y[0]);
	            }
	            //cout<<"ENCONTRADO "<<ind<< ": ("<<points.x[ind]<<", "<<points.y[ind]<<")"<<endl;
	        }else{
	            //cout<<"No hay vecinos viables"<<endl;
	            ind = points.x.size();
	        }

	        if(ind < points.x.size()){ // Se ha encontrado un punto viable

	            //cout<<"SELECCIÓN: "<<_ind<<" ("<<points.x[_ind]<<", "<<points.y[_ind]<<") -> "<<ind<<" ("<<points.x[ind]<<", "<<points.y[ind]<<"): "<<cwbang<<"|"<<ccwbang<<endl;

	            //contour.x.push_back(points.x[ind]); contour.y.push_back(points.y[ind]);
	            contour.push(points.x[ind], points.y[ind]);
	            n++;
	            // Guardar el índice del punto anterior en la lista completa de puntos
	            _ind = ind;
	            
	            if(n == max_it){
	                //cout<<"Más iteraciones que puntos"<<endl;
	                break; // Se ha cerrado el círculo de la vida
	            }

	            //if(contour.x.size())
	            if(points.x[ind] == contour.x[0] && points.y[ind] == contour.y[0]){
	                //cout<<"Contorno completo"<<endl;
	                break;
	            }

	        }else{
	            //cout<<"NO SE HA ENCONTRADO UN VECINO VIABLE"<<endl;
	            break;
	        }

	        //cout<<"*****************************************************************"<<endl;
	        it++;
	        //cin.get();

	    }

	    //cout<<n<<" - "<<convex_poss.x.size()<<" - "<<points.x.size()<<endl;

	    return contour;   
	}


	Poss<int> projectPoints(int xs, int ys, int xe, int ye, double dist);
	Path refinePath(Path path, vector<vector<float>> grid, float radius);

}


#endif
