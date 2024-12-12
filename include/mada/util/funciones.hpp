#ifndef FUNCIONES_HPP
#define FUNCIONES_HPP

#define INF (float)RAND_MAX

#define _USE_MATH_DEFINES

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <queue>
#include <limits>

using namespace std;
/*
template < typename T > string to_string( const T& n )
{
    ostringstream stm ;
    stm << n ;
    return stm.str() ;
}*/

vector<bool> ind2bool(int s, vector<int> ind);
vector<int> bool2ind(vector<bool> ind);
vector<bool> invert_vector(vector<bool> v);

template <typename T>
struct Pos{
	T x = 0; T y = 0;
}; // posición 2Ds

template <typename T>
struct PosT{
	T x = 0; T y = 0; T t = 0;
}; // posición con tiempo

template <typename T>
struct Poss{
	vector<T> x; vector<T> y;
	void operator () (int n){
		x.resize(n);y.resize(n);
	} // Inicializar con n valores
	void operator () (int n, T xv, T yv){
		x.resize(n,xv); y.resize(n,yv);
	} // inicializar con n valores replicados
	void operator () (T xv, T yv){
		x.push_back(xv); y.push_back(yv);
	} // Inicializar con el punto (xv, yv)

	bool operator == (Poss<T> p){
		if(x.size() != p.x.size()) return false;
		for(int i=0; i<x.size(); i++)
			if(x[i] != p.x[i] || y[i] != p.y[i]) return false;
		return true;
	} // inicializar con n valores replicados

	bool operator != (Poss<T> p){
		if(x.size() != p.x.size()) return true;
		for(int i=0; i<x.size(); i++)
			if(x[i] != p.x[i] || y[i] != p.y[i]) return true;
		return false;
	} // inicializar con n valores replicados

	void append(Poss<T> p){
		x.insert(x.end(),p.x.begin(),p.x.end());
		y.insert(y.end(),p.y.begin(),p.y.end());
	}

	void append(vector<T> x_, vector<T> y_){
		x.insert(x.end(),x_.begin(),x_.end());
		y.insert(y.end(),y_.begin(),y_.end());
	}

	void push(T x_, T y_){
		//x.insert(x.end(),x_);
		//y.insert(y.end(),y_);
		x.push_back(x_);
		y.push_back(y_);
	}

	void push(Pos<T> pos){
		//x.insert(x.end(),x_);
		//y.insert(y.end(),y_);
		x.push_back(pos.x);
		y.push_back(pos.y);
	}

    void show(){
        cout<<"x: ";
        for(int i=0; i<x.size();i++) cout<<x[i]<<" ";
        cout<<endl<<"y: ";
        for(int i=0; i<y.size();i++) cout<<y[i]<<" ";
        cout<<endl;
    } // Mostrar

    void show_(){
    	int max=0;
    	string vx,vy;
    	for(int i=0; i<x.size(); i++){
    		if(max<to_string(x[i]).size()) max=to_string(x[i]).size();
    		if(max<to_string(y[i]).size()) max=to_string(y[i]).size();
    	}
    	vx="x:";
    	vy="y:";
    	string v;
    	for(int i=0; i<x.size(); i++){
    		v=to_string(x[i]);
    		while(v.size()<max) v.insert(v.begin(),' ');
    		vx+=" "+v;
    		v=to_string(y[i]);
    		while(v.size()<max) v.insert(v.begin(),' ');
    		vy+=" "+v;
    	}
    	cout<<vx<<endl<<vy<<endl;
    } // Mostrar alineado

    void show_precision(int length){
    	int max=0;
    	string vx,vy;
    	for(int i=0; i<x.size(); i++){
    		if(max<to_string(x[i]).size()) max=to_string(x[i]).size();
    		if(max<to_string(y[i]).size()) max=to_string(y[i]).size();
    	}
    	if(max > length) max = length;
    	vx="x:";
    	vy="y:";
    	string v;
    	for(int i=0; i<x.size(); i++){
    		v=to_string(x[i]); if(v.size()>length) v.erase(v.begin()+length, v.end());
    		while(v.size()<max) v.insert(v.begin(),' ');
    		vx+=" "+v;
    		v=to_string(y[i]); if(v.size()>length) v.erase(v.begin()+length, v.end());
    		while(v.size()<max) v.insert(v.begin(),' ');
    		vy+=" "+v;
    	}
    	cout<<vx<<endl<<vy<<endl;
    } // Mostrar alineado

    void show(vector<int> index){
    	cout<<"x: ";
    	for(int i=0; i<index.size(); i++) cout<<x[index[i]]<<" ";
		cout<<endl<<"y: ";
		for(int i=0; i<index.size(); i++) cout<<y[index[i]]<<" ";
		cout<<endl;
    } //mostrar los índices

    void pop_back(){
    	x.pop_back(); y.pop_back();
    }

    void pop(){
    	x.erase(x.begin()); y.erase(y.begin());
    }

    void erase(int i){
    	if(i>=0 && i<x.size()){
	    	x.erase(x.begin()+i); y.erase(y.begin()+i);
	    }
    }

    bool erase_point(int x, int y){
    	//x.erase(x.begin()+i); y.erase(y.begin()+i);
    	for(int i=0; i<this->x.size(); i++){
    		if(this->x[i]==x && this->y[i]==y){
    			this->erase(i);
    			return true;
    		}
    	}
    	return false;
    }

    int find(int xp, int yp){
    	for(int i=0; i<x.size(); i++)
    		if(xp == x[i] && yp == y[i])
    			return i;
		return -1;
    }

    Poss<int> erase(vector<int> ind){
		Poss<int> res;
		vector<bool> tr = ind2bool(x.size(),ind);
		for(int i=0; i<x.size(); i++){
			if(!tr[i]){
				res.x.push_back(x[i]);
				res.y.push_back(y[i]);
			}
		}
		return res;
	}

    void clear(){
    	x.clear(); y.clear();
    }
    void reverse(){
    	T v;
    	for(int i=0; i<x.size()/2; i++){
    		v=x[x.size()-i-1];
    		x[x.size()-i-1]=x[i];
    		x[i]=v;
    		v=y[y.size()-i-1];
    		y[y.size()-i-1]=y[i];
    		y[i]=v;
    	}
    }

    vector<int> repe(){
    	vector<int> res(x.size(),-1);
    	for(int i=0; i<x.size(); i++){
    		for(int j=i+1; j<x.size(); j++){
    			if(x[i]==x[j] && y[i]==y[j]){
    				res[j]=i;
    			}
    		}
    	}
    	return res;
    }

    Poss<T> select(int from, int to){
    	Poss<T> res;
    	res.x.insert(res.x.begin(),x.begin()+from, x.begin()+to);
    	res.y.insert(res.y.begin(),y.begin()+from, y.begin()+to);
    	return res;
    }

    Poss<T> select(vector<int> ind){
    	Poss<T> res;
    	for(int i=0; i<ind.size(); i++) res.push(x[ind[i]], y[ind[i]]);
    	return res;
    }

    Poss<T> centroid(){
    	Poss<T> res; res(1);
    	res.x[0] = 0; res.y[0] = 0;
    	if(!x.size()) return res;
		for(int i=0; i<this->x.size(); i++){
			res.x[0] += this->x[i]; res.y[0] += this->y[i];
		}
		res.x[0]/=this->x.size(); res.y[0]/=this->y.size();
		return res;
    }

    Poss<T> unique(){
    	bool c;
    	Poss<T> res;
    	for(int i=0; i<this->x.size(); i++){
    		c = false;
    		for(int j=0; j<res.x.size(); j++){
    			if(res.x[j] == this->x[i] && res.y[j] == this->y[i]){
    				c = true;
    				break;
    			}
    		}
    		if(!c) res.push(this->x[i], this->y[i]);
    	}
    	return res;
    }

    float distance(T x, T y){
    	float res = 0;
    	for(int i=0; i<this->x.size(); i++)
    		res += sqrt(pow(x-this->x[i],2) + pow(y-this->y[i],2));
    	return res/this->x.size();
    }

    float cum_distance(){
    	float res = 0;
    	for(int i=0; i<x.size()-1; i++) res += sqrt(pow(x[i]-x[i+1],2)+pow(y[i]-y[i+1],2));
    	return res;
    }

    float minDistToPoint(T x, T y){
    	float res = INF, d;
    	for(int i=0; i<this->x.size(); i++){
    		d = sqrt(pow(x-this->x[i],2) + pow(y-this->y[i],2));
    		if(res > d) res = d;
    	}
    	return res;
    }

    void swap(Poss<T> &poss)
    {
    	vector<T> v;
    	v = poss.x; poss.x = this->x; this->x = v;
    	v = poss.y; poss.y = this->y; this->y = v;
    }

    bool isEqual(Poss<int> poss)
    {
    	if(x.size() != poss.x.size()) return false;
    	for(int i=0; i<x.size(); i++)
    		if(x[i] != poss.x[i] || y[i] != poss.y[i])
    			return false;
		return true;
    }

    void save(string filename){
		string line; // línea
		ofstream file(filename); // abrir para escribir
		if(file.is_open()){
			// Esto es el camino
			file << "poss = [\n";
			int i;
			line.clear();
			file << "x: ";
			for(i=0;i<x.size();i++) line+=to_string(x[i])+" ";
			file << line << "\n";line.clear();
			file << "y: ";
			for(i=0;i<y.size();i++) line+=to_string(y[i])+" ";
			file << line << "\n";line.clear();
			file << "];";
			// Cerrar el fichero
			file.close();
		}else cout<<"No se puede guardar en ese fichero"<<endl;
    } // guardar en un fichero

	void save(const char* filename){
		string line; // línea
		ofstream file(filename); // abrir para escribir
		if(file.is_open()){
			// Esto es el camino
			file << "poss = [\n";
			int i;
			line.clear();
			file << "x: ";
			for(i=0;i<x.size();i++) line+=to_string(x[i])+" ";
			file << line << "\n";line.clear();
			file << "y: ";
			for(i=0;i<y.size();i++) line+=to_string(y[i])+" ";
			file << line << "\n";line.clear();
			file << "];";
			// Cerrar el fichero
			file.close();
		}else cout<<"No se puede guardar en ese fichero"<<endl;
    } // guardar en un fichero

    void save_(string filename){

    } // Guardar en el mismo fichero

};

Poss<int> agentPoss(int x, int y, int sx, int sy);
float distance2closest_point(int x, int y, Poss<int> points);

template <typename T>
float distance2closest_point(T x, T y, vector<T> xp, vector<T> yp)
{
	float res = INF, d;
	for(int i=0; i<xp.size(); i++){
		d = sqrt(pow(x - xp[i], 2) + pow(y - yp[i], 2));
		if(res > d) res = d;
	}
	return res;
}

template <typename T> 
void show_q(queue<T> q)
{
	while(q.size()){
		cout<<q.front(); q.pop(); cout<<" ";
	}
	cout<<endl;
}

template <typename T> 
void show_q(queue<T> q, string name)
{
	cout<<name<<": ";
	while(q.size()){
		cout<<q.front(); q.pop(); cout<<" ";
	}
	cout<<endl;
}

template <typename T> 
Poss<T> join_poss(vector<Poss<T>> poss){
	Poss<T> res;
	for(int i=0; i<poss.size(); i++)
		res.append(poss[i]);
	return res;
}

template <typename T> 
Poss<int> pos_obs_(vector<vector<T> > grid){
	Poss<int> res;
	for(int i=0; i<grid.size(); i++)
		for(int j=0; j<grid[i].size(); j++)
			if(!grid[i][j]){
				res.x.push_back(i);
				res.y.push_back(j);
			}
	return res;
} // posiciones donde hay obstáculos

Poss<int> load_poss(const char* filename);

template <typename T>
T floor_step(T val, T step){
	return step * floor(val/step);
}

template <typename T>
T ceil_step(T val, T step){
	return step * ceil(val/step);
}

template <typename T>
T round_step(T val, T step){
	return step * round(val/step);
}

//~ funciones sobre vectores (con templates)

template <typename T>
bool any(vector<T> v){
	for(int i=0; i<v.size(); i++)
		if(v[i]) return true;
	return false;
}

template <typename T>
bool exists_v(vector<T> v,T val){
	for (int i=0;i<v.size();i++){
		if (v[i]==val) return true;
	}
	return false;
} // comprobar si un valor está presente en un vector

template <typename T>
Poss<int> generic_random_positions(vector<vector<T> > grid, int n){
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

template <typename T>
void random_vector(vector<T> &v){
    for(int i=0;i<v.size();i++) v[i]=((T)rand()/INF);
}

template <typename T>
void random_vector(vector<T> &v, T lb, T ub){
    for(int i=0;i<v.size();i++) v[i]=((T)rand()/INF)*(ub-lb)+lb;
}

template <typename T>
void sh_mat(vector<vector<T> > mat){
	int i,j;
	for(i=0;i<mat.size();i++){
		for(j=0;j<mat[i].size();j++) cout<<mat[i][j]<<" ";
		cout<<endl;
	}
} // mostrar una matriz

template <typename T>
void sh_vect_v(vector<T> v){
	for (int i=0;i<v.size();i++) cout<<v[i]<<endl;
} // mostrar un vector en vertical

template <typename T>
void sh_vect_v(vector<T> v, const char* name){
	cout<<name<<": ";
	for (int i=0;i<v.size();i++) cout<<v[i]<<endl;
} // mostrar un vector en vertical

template <typename T>
void sh_vect_h(vector<T> v){
	for (int i=0;i<v.size();i++) cout<<v[i]<<" ";
	cout<<endl;
} // mostrar un vector en horizontal

template <typename T>
void sh_vect_h(vector<T> v, const char* name){
	cout<<name<<": ";
	for (int i=0;i<v.size();i++) cout<<v[i]<<" ";
	cout<<endl;
} // mostrar un vector en horizontal y ponerle un título por delante

template <typename T>
vector<T> select_values_from_index(vector<T> v, vector<int> ind){
	vector<T> res;
	if(!ind.size()){
		cout<<"VECTOR ÍNDICES VACÍO"<<endl;
		return res;
	}
	for(int i=0; i<ind.size(); i++){
		if(ind[i]<v.size()){
			res.push_back(v[ind[i]]);
		}else{
			cout<<"LOS ÍNDICES NO SON CORRECTOS"<<endl;
		}
	}
	return res;
}

template <typename T>
vector<T> select_values_from_index(vector<T> v, vector<bool> ind){
	return select_values_from_index(v, bool2ind(ind));
}

template <typename T>
void sh_vect_cons(vector<T> v, const char* name){
	cout<<name<<"={";
	for(int i=0; i<v.size()-1; i++) cout<<v[i]<<", ";
	cout<<v[v.size()-1]<<"};"<<endl;
} // mostrar el vector para hacer copy/paste

template <typename T>
T factorial(T val){
	T res = 1;
	for(int i=0; i<val; i++) res*=(val-i);
	return res;
}

template <typename T>
T sum_v(vector<T> v){
	T res = 0;
	for(int i=0; i<v.size(); i++) res+=v[i];
	return res;
}


template <typename T>
vector<T> prod_v(vector<T> v, T val){
	for(int i=0; i<v.size(); i++) v[i]*=val;
	return v;
}

template <typename T>
void prod_v_(vector<T> &v, T val){
	for(int i=0; i<v.size(); i++) v[i]*=val;
}

template <typename T>
vector<T> select_vector_values(vector<T> v, vector<int> values){
	vector<T> res(values.size());
	for(int i=0; i<values.size(); i++) res[i]=v[values[i]];
	return res;
}

template <typename T>
int find_value_in_vect(vector<T> v, T val){
	for(int i=0; i<v.size(); i++)
		if(v[i] == val) return i;
	return -1;
}

template <typename T>
T min_v(vector<T> v){
	T res = (T)INF;
	for(int i=0; i<v.size(); i++) res=v[i]<res?v[i]:res;
	return res;
}

template <typename T>
T min_v(vector<T> v, int &ind){
	T res = (T)INF;
	for(int i=0; i<v.size(); i++){
		if(res > v[i]){
			res = v[i];
			ind = i;
		}
	}
	return res;
}

template <typename T>
T nz_min_v(vector<T> v){
	T res = (T)INF;
	for(int i=0; i<v.size(); i++)
		if(v[i]) res=v[i]<res?v[i]:res;
	return res;
}

template <typename T>
T nz_min_v(vector<T> v, int &ind){
	T res = (T)INF;
	for(int i=0; i<v.size(); i++){
		if(v[i] && res > v[i]){
			res = v[i];
			ind = i;
		}
	}
	return res;
}

template <typename T>
T max_v(vector<T> v){
	T res = -(T)INF;
	for(int i=0; i<v.size(); i++) res=v[i]>res?v[i]:res;
	return res;
}

template <typename T>
T max_v(vector<T> v, int &ind){
	T res = -(T)INF;
	for(int i=0; i<v.size(); i++){
		if(res < v[i]){
			res = v[i];
			ind = i;
		}
	}
	return res;
}

template <typename T>
T nz_max_v(vector<T> v){
	T res = -(T)INF;
	for(int i=0; i<v.size(); i++)
		if(v[i]) res=v[i]>res?v[i]:res;
	return res;
}

template <typename T>
T nz_max_v(vector<T> v, int &ind){
	T res = -(T)INF;
	for(int i=0; i<v.size(); i++){
		if(v[i] && res < v[i]){
			res = v[i];
			ind = i;
		}
	}
	return res;
}

template <typename T>
T mean_v(vector<T> v){
	T res = 0;
	for(int i=0; i<v.size(); i++) res+=v[i];
	return res/v.size();
}

template <typename T>
T nz_mean_v(vector<T> v){
	T res=0;
	int nz = 0;
	for(int i=0; i<v.size(); i++){
		if(v[i]){
			res+=v[i];
			nz++;
		}
	}
	return res/nz;
}

template <typename T>
vector<T> norm_v(vector<T> v){
	T max=0;
	for(int i=0; i<v.size(); i++){
		if(max<v[i]) max = v[i];
	}
	for(int i=0; i<v.size(); i++){
		v[i]/=max;
	}
	return v;
}

template <typename T>
void norm_v_(vector<T> &v){
	T max=0;
	for(int i=0; i<v.size(); i++){
		if(max<v[i]) max = v[i];
	}
	for(int i=0; i<v.size(); i++){
		v[i]/=max;
	}
}

template <typename T>
vector<T> norm_v(vector<T> v, T val){
	T max=0;
	for(int i=0; i<v.size(); i++){
		v[i]/=val;
	}
	return v;
}

template <typename T>
void norm_v_(vector<T> &v, T val){
	T max=0;
	for(int i=0; i<v.size(); i++){
		v[i]/=val;
	}
}

template<typename T>
T var_v(vector<T> v){
	T res, mean=0, v_=0;
	for(int i=0; i<v.size(); i++){
		v_+=pow(v[i],2);
		mean+=v[i];
	}
	mean/=v.size();
	res=v_/v.size()-pow(mean,2);
	return res;
}

template<typename T>
T std_v(vector<T> v){
	return sqrt(var_v(v));
}

template <typename T>
string vect2str(vector<T> v){
	string res;
	for(int i=0; i<v.size(); i++) res+=(to_string(v[i])+" ");
	return res;
} // convertir a string

template <typename T>
vector<T> reverse(vector<T> v){
	int size=v.size();
	vector<T> res(size);
	for (int i=0; i<size; i++) res[size-i-1]=v[i];
	return res;
} // invertir vector entero

template <typename T>
void reverse_(vector<T> &v){
	int size=v.size();
	vector<T> res(size);
	for (int i=0; i<size; i++) res[size-i-1]=v[i];
	v=res;
} // invertir vector entero

template <typename T>
vector<T> reverse(vector<T> v, int start, int end){
	int size=v.size(), subsize=end-start;
	if (subsize==size-1) return reverse(v);
	vector<T> res=v;
	for (int i=start; i<end+1; i++){
		res[i]=v[start+end-i];
	}
	return res;
} // invertir parte del vector (de start a end)

template <typename T>
void reverse_(vector<T> &v, int start, int end){
	int size=v.size(), subsize=end-start;
	if (subsize==size-1) return reverse(v);
	vector<T> res=v;
	for (int i=start; i<end+1; i++){
		res[i]=v[start+end-i];
	}
	v=res;
} // invertir parte del vector (de start a end)

template <typename T>
vector<T> uniform_vector(T s, T e, T interv){
	float n = (float)(e-s)/(float)interv;
	if (n==floor(n)) n++;
	else n = ceil(n);
	vector<T> res(n);
	for (int i=0; i<res.size(); i++) res[i]=interv*(T)i+s;
	return res;
}

template<typename T>
vector<T> diff(vector<T> v){
	vector<T> res(v.size());
	for(int i=1; i<res.size(); i++) res[i]=v[i]-v[i-1];
	return res;
}

template<typename T>
vector<T> unique_sort(vector<T> v){
	vector<T> res;
	T min;
	int n = v.size();
	for(int i=0; i<n; i++){
		// Encontrar el valor mínimo del vector
		min = (T)numeric_limits<float>::max();
		for(int j=0; j<v.size(); j++){
			if(min > v[j]){
				min = v[j];
			}
		}
		if(min == (T)numeric_limits<float>::max()) break;
		// Insertar en el resultado
		res.push_back(min);
		// Eliminar ese valor del vector
		int k = 0;
		while(k<v.size()){
			if(v[k]==min) v.erase(v.begin()+k);
			else k++;
		}
	}
	return res;
}

template<typename T>
vector<T> unique_no_sort(vector<T> v){
	vector<T> res;
	if(v.size() == 0) return res;
	res.push_back(v[0]);
	bool esta;
	for(int i=1; i<v.size(); i++){
		esta = false;
		for(int j=0; j<res.size(); j++)
			if(res[j]==v[i]) esta = true;
		if (!esta) res.push_back(v[i]);
	}
	return res;
}

template<typename T>
vector<vector<T>> unique_vector(vector<T> vect)
{
    vector<vector<T>> res;
    bool ex;
    res.resize(1,vector<T>(2,0)); res[0][0] = vect[0]; res[0][1] = 1;
    for(int i=1; i<vect.size(); i++){
        ex = false;
        for(int j=0; j<res.size(); j++){
            if(res[j][0] == vect[i]){
                ex = true;
                res[j][1]++;
                break;
            }
        }
        if(!ex){
            ex = false;
            for(int j=0; j<res.size(); j++){
                if(res[j][0] > vect[i]){
                    res.insert(res.begin()+j, vector<T>(2,0));
                    res[j-1][0] = vect[i]; res[j-1][1] = 1;
                    ex = true;
                    break;
                }
            }
            if(!ex){
                //res.insert(res.begin(), vector<T>(2,0));
                res.push_back(vector<T>(2,0));
                res[res.size()-1][0] = vect[i]; res[res.size()-1][1] = 1;
            }
        }
    }
    return res;   
}

template<typename T>
vector<T> sort_vect(vector<T> v, int order){
	int n = v.size();
	vector<T> res(n);
	vector<bool> td(n,true);
	int k;
	if(!order){
		// Orden descendente
		for(int i=0; i<n; i++){
			res[i] = -(T)RAND_MAX;
			for(int j=0; j<n; j++){
				if(td[j] && v[j]>res[i]){
					k = j;
					res[i] = v[j];
				}
			}
			td[k] = false;
		}
	}else{
		// Orden ascendente
		for(int i=0; i<n; i++){
			res[i] = (T)RAND_MAX;
			for(int j=0; j<n; j++){
				if(td[j] && v[j]<res[i]){
					k = j;
					res[i] = v[j];
				}
			}
			td[k] = false;
		}
	}
	return res;
} // Ordenar vector en orden ascendente o descendente

template<typename T>
vector<T> sort_vect(vector<T> v, int order, vector<int> &o){
	int n = v.size();
	vector<T> res(n);
	vector<bool> td(n,true);
	if(o.size()!=n) o.resize(n, 0);
	int k;
	if(!order){
		// Orden descendente
		for(int i=0; i<n; i++){
			res[i] = -(T)RAND_MAX;
			for(int j=0; j<n; j++){
				if(td[j] && v[j]>res[i]){
					k = j;
					res[i] = v[j];
				}
			}
			o[k] = i;
			td[k] = false;
		}
	}else{
		// Orden ascendente
		for(int i=0; i<n; i++){
			res[i] = (T)RAND_MAX;
			for(int j=0; j<n; j++){
				if(td[j] && v[j]<res[i]){
					k = j;
					res[i] = v[j];
				}
			}
			o[k] = i;
			td[k] = false;
		}
	}
	return res;
} // Ordenar vector en orden ascendente o descendente, quedandome con el vector de los índices

template<typename T>
vector<T> sort_vect(vector<T> v, vector<int> order){
	vector<T> res(v.size());
	for(int i=0; i<v.size(); i++){
		for(int j=0; j<v.size(); j++){
			if(order[j] == i){
				res[i] = v[j];
			}
		}
	}
	return res;
} // Ordenar un vector con el vector orden

template<typename T>
vector<T> moving_average(vector<T> v, int n)
{
    vector<T> res = v;
    if(n<0) return res;
    int tam = res.size()-1;
    int s, e;
    T val;
    for(int i=0; i<=tam; i++){
        s = i-ceil(n/2) > 0 ? i-ceil(n/2) : 0;
        e = i+ceil(n/2) < tam ? i+ceil(n/2) : tam;
        val = 0;
        for(int j=s; j<=e; j++){
            val += v[j];
        }
        res[i] = val/((e-s)+1);
    }
    return res; 
}
//===================================================================
//===================================================================
//===================================================================
//          SOBRECARGA DE OPERADORES SOBRE VECTORES
//===================================================================
//===================================================================
//===================================================================
// SUMA
template <typename T>
vector<T> operator + (vector<T> v1, vector<T> v2){
    vector<T> res(v1.size());
    for(int i=0;i<v1.size();i++) res[i]=v1[i]+v2[i];
    return res;
} // Otro vector

template <typename T>
vector<T> operator + (vector<T> v1, T val){
    for(int i=0;i<v1.size();i++) v1[i]+=val;
    return v1;
} // Un valor

template <typename T>
void operator += (vector<T> &v1, vector<T> &v2){
    for(int i=0;i<v1.size();i++) v1[i]+=v2[i];
} // Otro vector

template <typename T>
void operator += (vector<T> &v1, T val){
    for(int i=0;i<v1.size();i++) v1[i]+=val;
} // Un valor

// RESTA
template <typename T>
vector<T> operator - (vector<T> v1, vector<T> v2){
    vector<T> res(v1.size());
    for(int i=0;i<v1.size();i++) res[i]=v1[i]-v2[i];
    return res;
} // Otro vector

template <typename T>
vector<T> operator - (vector<T> v1, T val){
    for(int i=0;i<v1.size();i++) v1[i]-=val;
    return v1;
} // Un valor

template <typename T>
void operator -= (vector<T> &v1, vector<T> &v2){
    for(int i=0;i<v1.size();i++) v1[i]-=v2[i];
} // Otro vector

template <typename T>
void operator -= (vector<T> &v1, T val){
    for(int i=0;i<v1.size();i++) v1[i]-=val;
} // Un valor

// PRODUCTO
template <typename T>
vector<T> operator * (vector<T> v1, vector<T> v2){
    vector<T> res(v1.size());
    for(int i=0;i<v1.size();i++) res[i]=v1[i]*v2[i];
    return res;
} // Otro vector

template <typename T>
vector<T> operator * (vector<T> v1, T val){
    for(int i=0;i<v1.size();i++) v1[i]*=val;
    return v1;
} // Un valor

template <typename T>
void operator *= (vector<T> &v1, vector<T> &v2){
    for(int i=0;i<v1.size();i++) v1[i]*=v2[i];
} // Otro vector

template <typename T>
void operator *= (vector<T> &v1, T val){
    for(int i=0;i<v1.size();i++) v1[i]*=val;
} // Un valor

// DIVISIÓN
template <typename T>
vector<T> operator / (vector<T> v1, vector<T> v2){
    vector<T> res(v1.size());
    for(int i=0;i<v1.size();i++) res[i]=v1[i]/v2[i];
    return res;
} // Otro vector

template <typename T>
vector<T> operator /= (vector<T> v1, T val){
    for(int i=0;i<v1.size();i++) v1[i]/=val;
} // Un valor

template <typename T>
void operator /= (vector<T> &v1, vector<T> &v2){
    for(int i=0;i<v1.size();i++) v1[i]/=v2[i];
} // Otro vector

template <typename T>
void operator /= (vector<T> &v1, T val){
    for(int i=0;i<v1.size();i++) v1[i]/=val;
} // Un valor

// COMPARACIÓN
template <typename T>
bool operator == (vector<T> v1, vector<T> v2){
    if(v1.size()!=v2.size()) return false;
    for(int i=0;i<v1.size();i++){
        if(v1[i]!=v2[i]) return false;
    }
    return true;
}

template <typename T>
int operator != (vector<T> v1, vector<T> v2){
    if(v1.size()!=v2.size()) return -1;
    for(int i=0;i<v1.size();i++){
        if(v1[i]!=v2[i]) return i;
    }
    return -1;
} // devuelve la posición donde hay dos valores que no coinciden

//===================================================================
//===================================================================
//===================================================================
//~ funciones para el grid
//~ MIRATE A VER SI EN UN FUTURO ES CONVENIENTE HACER UNA CLASE ENTERA PARA ESTO
vector<vector<float> > random_obs(int x, int y, float dens); // generar un grid aleatorio con cienrta densidad de obstáculos
void invertir_obs(vector<vector<float> > &grid); // cambia los valores de los obstáculos (simplen¡mente intercambio los valores)
void gen_pos(vector<vector<float> > grid, int& x, int& y); // generar una posición aleatoria, fuera de los obstáculos
vector<Pos<int> > gen_pos(vector<vector<float> > grid, int n); // generar n posiciones aleatorias
Poss<int> gen_pos2(vector<vector<float> > grid, int n);
Poss<int> pos_obs(vector<vector<float> > grid); // posiciones en las cuales hay obstáculos
Poss<int> pos_obs_wl(vector<vector<float> > grid);
Poss<int> gen_pos_dist(vector<vector<float> > grid, int n, float dist);

vector<Poss<int>> grid_positions(vector<vector<float> > grid);
vector<Poss<int>> grid_positions(vector<vector<float> > grid, vector<vector<float> > reference_grad);

vector<Poss<int>> fix_free_space(vector<vector<float> > &grid, vector<vector<float> > reference_grad);
vector<Poss<int>> fix_free_space_(vector<vector<float> > &grid, vector<vector<float> > reference_grad);

Poss<int> random_positions(vector<vector<float> > grid, int n);
Poss<int> bounded_random_positions(vector<vector<float> > grid, int n, int xlb, int xub, int ylb, int yub);

void inflate_pos(int x, int y, vector<vector<float>> &grid, int srange);
void inflate_pos(int x, int y, vector<vector<float>> &grid, int srange, float val);
void inflate_pos_iv(int x, int y, vector<vector<float>> &grid, int srange, float val);
Poss<int> inflated_poss(Poss<int> pos, int srange, int sx, int sy);
Poss<int> inflated_poss(Poss<int> pos, vector<int> srange, int sx, int sy);
Poss<int> generate_poss_with_srange(vector<vector<float> > grid, int n, int srange);
Poss<int> generate_poss_with_srange(vector<vector<float> > grid, vector<int> srange);
Poss<int> generate_bounded_poss_with_srange(vector<vector<float> > grid, int n, int srange, int xlb, int xub, int ylb, int yub);

Poss<int> generate_poss_with_sranges(vector<vector<float> > grid, vector<int> sranges);
vector<Poss<int>> generate_infl_poss_with_sranges(vector<vector<float> > grid, vector<int> sranges);
void generate_infl_poss_with_sranges(vector<vector<float> > grid, vector<int> sranges, Poss<int> &pos, vector<Poss<int>> &infl);

Poss<float> circle_poss(float x, float y, float radius, float dt); // Para dibujar
Poss<int> circle_poss(int x, int y, int srange, int sx, int sy); // Puntos sobre el grid
Poss<int> compute_centroid(Poss<int> pos);
Poss<int> compute_gradient_centroids(vector<vector<float>> grad, int cell_size);

vector<vector<float> > load_grid(const char* filename);
vector<vector<float> > load_matr(const char* filename);
vector<vector<float> > red_res(vector<vector<float> > grid, int res);
vector<vector<int> > red_res(vector<vector<int> > grid, int res);
vector<vector<float> > red_res_(vector<vector<float> > grid, int &res);
vector<vector<float> > red_res__(vector<vector<float> > grid, int &res);

vector<vector<float>> reverse_gradient(vector<vector<float>> grad);
vector<vector<float>> reverse_gradient(vector<vector<float>> grad, vector<vector<float>> grid);
vector<vector<float>> reverse_and_normalize_gradient(vector<vector<float>> grad);
vector<vector<float>> reverse_and_normalize_gradient(vector<vector<float>> grad, vector<vector<float>> grid);
vector<vector<float>> reverse_and_normalize_gradient(vector<vector<float>> grad, float val);
vector<vector<float>> reverse_and_normalize_gradient(vector<vector<float>> grad, vector<vector<float>> grid, float val);

vector<vector<float>> mapa_obstaculos_inflados(vector<vector<float>> static_grid, Poss<int> obst, int irange);
void accumulate_occupancy(vector<vector<float>> &occupancy_grid, vector<vector<float>> grid);
vector<vector<float>> accumulate_occupancy_(vector<vector<float>> occupancy_grid, vector<vector<float>> grid);

void anular_radio_f(vector<vector<float>> &grid, int x, int y, float r);
void anular_radio_i(vector<vector<int>> &grid, int x, int y, float r);

void anular_rango_f(vector<vector<float>> &grid, int x, int y, float r);
void anular_rango_i(vector<vector<int>> &grid, int x, int y, float r);

void set_value_precisionf(float v, int p);
void set_value_precisiond(double v, int p);

void set_vector_precisionf(vector<float> &v, int p);
void set_vector_precisiond(vector<double> &v, int p);

void set_matrix_precisionf(vector<vector<float>> &m, int p);
void set_matrix_precisiond(vector<vector<double>> &m, int p);

float angle(float x1, float y1, float x2, float y2, float x3, float y3);

void assign_value(Poss<int> poss, float val, vector<vector<float>> &grid);
void assign_values(Poss<int> poss, vector<vector<float>> val, vector<vector<float>> &grid);
void increase_value(Poss<int> poss, float val, vector<vector<float>> &grid);
void increase_values(Poss<int> poss, vector<vector<float>> val, vector<vector<float>> &grid);

bool eventTimeout(float &fixTime, float &time, float &_time, float currentTime);

template <typename T>
Poss<T> obstacle_free_poss(Poss<T> poss, vector<vector<float>> grid)
{
	Poss<T> res;
	for(int i=0; i<poss.x.size(); i++)
		if(grid[poss.x[i]][poss.y[i]]) res.push(poss.x[i],poss.y[i]);
	return res;	
}

template <typename T>
void set_values_in_grid(vector<vector<T>> &grid, Poss<int> poss, T val){
	for(int i=0; i<poss.x.size(); i++)
		grid[poss.x[i]][poss.y[i]] = val;
}

template <typename T>
void set_values_in_grid(vector<vector<T>> &grid, Poss<int> poss, vector<T> val){
	for(int i=0; i<poss.x.size(); i++)
		grid[poss.x[i]][poss.y[i]] = val[i];
}

template <typename T>
bool check_pos_in_grid(int x, int y, vector<vector<T>> grid){
	if(x>=0 && x<grid.size() && y>=0 && y<grid[0].size()) return true;
	else return false;
}

bool check_pos_in_grid(int x, int y, int sx, int sy);
template <typename T>
vector<T> get_values_from_map(Poss<int> pos, vector<vector<T>> map)
{
	vector<T> values(pos.x.size());
	for(int i=0; i<pos.x.size(); i++)
		values[i] = map[pos.x[i]][pos.y[i]];
	return values;
}

template <typename T>
vector<T> get_values_from_map(vector<int> x, vector<int> y, vector<vector<T>> map)
{
	vector<T> values(x.size());
	for(int i=0; i<x.size(); i++)
		values[i] = map[x[i]][y[i]];
	return values;
}

template <typename T>
Poss<T> fix_poss_in_map(Poss<T> pos, vector<vector<float>> map)
{
	int sx = map.size();
	if(sx == 0) return pos;
	int sy = map[0].size();
	int i=0;
	while(i<pos.x.size()){
		if(pos.x[i]<0 || pos.x[i]>=sx || pos.y[i]<0 || pos.y[i]>=sy)
			pos.erase(i);
		else
			i++;
	}
	return pos;
}

template <typename T>
Poss<T> fix_poss_in_map(Poss<T> pos, int sx, int sy)
{
	if(sx == 0) return pos;
	int i=0;
	while(i<pos.x.size()){
		if(pos.x[i]<0 || pos.x[i]>=sx || pos.y[i]<0 || pos.y[i]>=sy)
			pos.erase(i);
		else
			i++;
	}
	return pos;
}


template <typename T>
Poss<int> range_poss(int x, int y, int r, vector<vector<T>> grid)
{
	// Establecer los limites para no recorrer todo el grid
    int xl, xu, yl, yu;
    xl = (x-r) > 0 ? (x-r) : 0;
    xu = (x+r) < grid.size()-1 ? (x+r) : grid.size()-1;
    yl = (y-r) > 0 ? (y-r) : 0;
    yu = (y+r) < grid[0].size()-1 ? (y+r) : grid[0].size()-1;

	Poss<int> res;
	for(int i=xl; i<=xu; i++){
        for(int j=yl; j<=yu; j++){
            if(sqrt(pow(x-i,2)+pow(y-j,2))<=r){
                res.x.push_back(i);
            	res.y.push_back(j);
            }
        }
    }
	return res;
} // Devuelve todas las posiciones que están dentro del rango

template <typename T>
void valor_radio_f(vector<vector<T>> &grid, int x, int y, float r, T val)
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
                grid[i][j] = val;
        }
    }
} // Fijar con el valor "val" todas las posiciones que queden dentro del rango "r"

template <typename T>
void values_of_poss(Poss<int> pos, T val, vector<vector<T>> &grid)
{
	for(int i=0; i<pos.x.size(); i++)
		grid[pos.x[i]][pos.y[i]] = val;
}

template <typename T>
vector<vector<T>> round_matrix(vector<vector<T>> matrix)
{
    for(int i=0; i<matrix.size(); i++){
        for(int j=0; j<matrix[i].size(); j++){
            matrix[i][j] = round(matrix[i][j]);
        }
    }   
    return matrix;
}

template <typename T>
Pos<int> find_value_in_matr(vector<vector<T>> matr, T val){
	Pos<int> res;
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(matr[i][j] == val){
				res.x = i; res.y = j;
				return res;
			}
		}
	}
	return res;
} // Encontrar el valor en la matriz

template <typename T>
Poss<int> find_values_in_matrix(vector<vector<T>> matrix, T value)
{
    Poss<int> res;
    for(int i=0; i<matrix.size(); i++){
        for(int j=0; j<matrix[i].size(); j++){
            if(matrix[i][j]==value){
                res.push(i,j);
            }
        }
    }
    return res;   
} // Encontrar los valores en la matriz

template <typename T>
T find_matr_min(vector<vector<T>> matr, int &x, int &y){
	T min = (T)RAND_MAX;
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(min > matr[i][j]){
				min = matr[i][j];
				x = i; y = j;
			}
		}
	}
	return min;
} // Encontrar el máximo (NO INFINITO) de una matriz

template <typename T>
T find_matr_min_nz(vector<vector<T>> matr){
	T min = (T)RAND_MAX;
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(matr[i][j] && min > matr[i][j]){
				min = matr[i][j];
			}
		}
	}
	return min;
} // Encontrar el máximo (NO NULO) de una matriz

template <typename T>
T find_matr_min_nz(vector<vector<T>> matr, int &x, int &y){
	T min = (T)RAND_MAX;
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(matr[i][j] && min > matr[i][j]){
				min = matr[i][j];
				x = i; y = j;
			}
		}
	}
	return min;
} // Encontrar el máximo (NO NULO) de una matriz

template <typename T>
T find_matr_max(vector<vector<T>> matr){
	T max = -(T)RAND_MAX;
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(max<matr[i][j]){
				max = matr[i][j];
			}
		}
	}
	return max;
} // Encontrar el máximo (NO INFINITO) de una matriz

template <typename T>
T find_matr_max(vector<vector<T>> matr, int &x, int &y){
	T max = -(T)RAND_MAX;
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(max<matr[i][j]){
				max = matr[i][j];
				x = i; y = j;
			}
		}
	}
	return max;
} // Encontrar el máximo (NO INFINITO) de una matriz

template <typename T>
void find_matr_max_(vector<vector<T>> matr, int &x, int &y){
	T max = -(T)RAND_MAX;
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(max<matr[i][j]){
				max = matr[i][j];
				x = i; y = j;
			}
		}
	}
} // Encontrar el máximo (NO INFINITO) de una matriz

template <typename T>
T find_matr_max_nf(vector<vector<T>> matr){
	T max = 0, inf_ =T(RAND_MAX);
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(matr[i][j]<inf_ && max<matr[i][j]){
				max = matr[i][j];
			}
		}
	}
	return max;
} // Encontrar el máximo (NO INFINITO) de una matriz

template <typename T>
T find_matr_max_nf(vector<vector<T>> matr, int &x, int &y){
	T max = 0, inf_ =T(RAND_MAX);
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(matr[i][j]<inf_ && max<matr[i][j]){
				max = matr[i][j];
				x = i; y = j;
			}
		}
	}
	return max;
} // Encontrar el máximo (NO INFINITO) de una matriz

template <typename T>
void find_matr_max_nf_(vector<vector<T>> matr, int &x, int &y){
	T max = 0, inf_ =T(RAND_MAX);
	for(int i=0; i<matr.size(); i++){
		for(int j=0; j<matr[i].size(); j++){
			if(matr[i][j]<inf_ && max<matr[i][j]){
				max = matr[i][j];
				x = i; y = j;
			}
		}
	}
} // Encontrar el máximo (NO INFINITO) de una matriz

template<typename T>
bool symmetric_matrix(vector<vector<T>> m){
	int size = m.size();
	if(size == 0) return false;
	for(int i=0; i<size; i++){
		if(m[i].size() != size) return false;
		for(int j=0; j<size; j++)
			if(m[i][j] != m[j][i]) return false;
	}
	return true;
} // Devuelve si la matriz es simétrica o no

template<typename T>
vector<T> select_values_from_matr(vector<vector<T> > matr, vector<int> x, vector<int> y){
	vector<T> res(x.size());
	for(int i=0; i<x.size(); i++)
		res[i] = matr[x[i]][y[i]];
	return res;
} // Incrementar la resolución del grid

template<typename T>
vector<vector<T> > _red_res(vector<vector<T> > grid, int res){
    if(res%2){
        return grid;
    }
    int sx=grid.size(), sy=grid[0].size();
    int sx_=sx/res, sy_=sy/res; // tamaño del nuevo grid
    vector<vector<T> > grid_(sx_,vector<T>(sy_,1));
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

template<typename T>
vector<vector<T> > incr_res(vector<vector<T> > grid, int res){
	if(res<=1) return grid;
	int sx=grid.size(), sy=grid[0].size();
	int sx_=sx*res, sy_=sy*res;
	vector<vector<T> > grid_(sx_,vector<T>(sy_,0));
	for(int i=0; i<sx; i++){
		for(int j=0; j<sy; j++){
			for(int k=(i*res+1)-res/2; k<(i*res+1)+res/2; k++){
            	for(int l=(j*res+1)-res/2; l<(j*res+1)+res/2; l++){
            		if(k>=0 && k<sx_ && l>=0 && l<sy_){
            			grid_[k][l]=grid[i][j];
            		}
            	}
            }
		}
	}
	return grid_;
} // Incrementar la resolución del grid

template <typename T>
Poss<int> _pos_obs(vector<vector<T> > grid){
	Poss<int> res;
	for(int i=0; i<grid.size(); i++)
		for(int j=0; j<grid[i].size(); j++)
			if(!grid[i][j]){
				res.x.push_back(i);
				res.y.push_back(j);
			}
	return res;
} // posiciones donde hay obstáculos

//===================================================================
//===================================================================
//===================================================================

template<typename T>
vector<vector<T>> fliplr(vector<vector<T>> matr){
	vector<vector<T>> res(matr.size());
	for(int i=0; i<matr.size(); i++)
        for(int j=matr[i].size()-1; j>=0; j--)
            res[i].push_back(matr[i][j]);
	return res;
}

template<typename T>
void sh_matr(vector<vector<T> > matr){
    for(int i=0; i<matr.size(); i++){
        for(int j=0; j<matr[i].size(); j++) cout<<matr[i][j]<<" ";
        cout<<endl;
    }
}

template<typename T>
void sh_matr_al_(vector<vector<T> > matr){
    int sx=matr.size();
    if(!sx) return;
    int sy=matr[0].size();
    vector<vector<string> > s; // almacenará los strings
    vector<vector<int> > tam; // tamaño de las palabras
    s.resize(sx,vector<string>(sy,""));
    tam.resize(sx,vector<int>(sy,0));
    int maxs=0;
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            s[i][j]=to_string(matr[i][j]);
            tam[i][j]=s[i][j].size();
            maxs=tam[i][j]>maxs?tam[i][j]:maxs;
        }
    }
    for(int i=0; i<sx; i++){
        for(int j=0; j<sy; j++){
            for(int cont=0;cont<maxs-tam[i][j];cont++) s[i][j].insert(0," ");
            cout<<s[i][j]<<" ";
        }
        cout<<endl;
    }
} // representar alineada

template<typename T>
void sh_matr_al(vector<vector<T> > matr){
    int sx=matr.size();
    if(!sx) return;
    int sy = 0;
    vector<vector<string> > s; // almacenará los strings
    vector<vector<int> > tam; // tamaño de las palabras
    s.resize(sx);
    tam.resize(sx);
    int maxs=0;
    for(int i=0; i<sx; i++){
        for(int j=0; j<matr[i].size(); j++){
            s[i].push_back(to_string(matr[i][j]));
            tam[i].push_back(s[i][s[i].size()-1].size());
            maxs=tam[i][j]>maxs?tam[i][j]:maxs;
        }
    }
    for(int i=0; i<sx; i++){
        for(int j=0; j<matr[i].size(); j++){
            for(int cont=0;cont<maxs-tam[i][j];cont++) s[i][j].insert(0," ");
            cout<<s[i][j]<<" ";
        }
        cout<<endl;
    }
} // representar alineada

//~ template<typename T>
//~ vector<vector<T> > operator / (vector<vector<T> > matr1,vector<vector<T> > matr2){
	//~ int x1=matr1.size(),y1=matr1[0].size(),x2=matr2.size(),y2=matr2[0].size();
	//~ vector<vector<T> > res;
	//~ res.resize(x1,vector<T>(y1,0));
	//~ if (x1!=x2 || y1!=y2) return 0;
	//~ for(int i=0; i<x1; i++)
		//~ for(int j=0; j<y1; j++)
			//~ res[i][j]=matr1[i][j]/matr2[i][j];
	//~ return res;
//~ }

template<typename T>
void sh_matr_al(vector<vector<T> > matr, const char* titulo){
    int sx=matr.size();
    cout<<titulo<<": "<<endl;
    if(!sx) return;
    int sy=matr[0].size();
    vector<vector<string> > s; // almacenará los strings
    vector<vector<int> > tam; // tamaño de las palabras
    s.resize(sx,vector<string>(sy,""));
    tam.resize(sx,vector<int>(sy,0));
    int maxs=0;
    for(int i=0;i<sx;i++){
        for(int j=0;j<sy;j++){
            s[i][j]=to_string(matr[i][j]);
            tam[i][j]=s[i][j].size();
            maxs=tam[i][j]>maxs?tam[i][j]:maxs;
        }
    }
    for(int i=0; i<sx; i++){
        for(int j=0;j<sy;j++){
            for(int cont=0;cont<maxs-tam[i][j];cont++) s[i][j].insert(0," ");
            cout<<s[i][j]<<" ";
        }
        cout<<endl;
    }
} // representar alineada

template<typename T>
string matr2str(vector<vector<T> > m){
	string res,line;
	for(int i=0; i<m.size(); i++){
		line="";
		for(int j=0; j<m[i].size(); j++){
			line+=(to_string(m[i][j])+" ");
		}
		line+="\n";
		res+=line;
	}
	return res;
}

template<typename T>
void operator += (vector<vector<T> > &m, T val){
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]+=val;
}

template<typename T>
vector<vector<T> > operator + (vector<vector<T> > m, T val){
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]+=val;
	return m;
}

template<typename T>
void operator -= (vector<vector<T> > &m, T val){
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]-=val;
}

template<typename T>
vector<vector<T> > operator - (vector<vector<T> > m, T val){
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]-=val;
	return m;
}

template<typename T>
void operator *= (vector<vector<T> > &m, T val){
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]*=val;
}

template<typename T>
vector<vector<T> > operator * (vector<vector<T> > m, T val){
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]*=val;
	return m;
}

template<typename T>
void operator /= (vector<vector<T> > &m, T val){
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]/=val;
}

template<typename T>
vector<vector<T> > operator / (vector<vector<T> > m, T val){
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]/=val;
	return m;
}

//==================================
template<typename T>
void operator += (vector<vector<T> > &m, vector<vector<T> > val){
	if (m.size()!=val.size() || m[0].size()!=val[0].size()) return;
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]+=val[i][j];
}

template<typename T>
vector<vector<T> > operator + (vector<vector<T> > m, vector<vector<T> > val){
	if (m.size()!=val.size() || m[0].size()!=val[0].size()) return m;
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]+=val[i][j];
	return m;
}

template<typename T>
void operator -= (vector<vector<T> > &m, vector<vector<T> > val){
	if (m.size()!=val.size() || m[0].size()!=val[0].size()) return;
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]-=val[i][j];
}

template<typename T>
vector<vector<T> > operator - (vector<vector<T> > m, vector<vector<T> > val){
	if (m.size()!=val.size() || m[0].size()!=val[0].size()) return m;
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]-=val[i][j];
	return m;
}

template<typename T>
void operator *= (vector<vector<T> > &m, vector<vector<T> > val){
	if (m.size()!=val.size() || m[0].size()!=val[0].size()) return;
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]*=val[i][j];
}

template<typename T>
vector<vector<T> > operator * (vector<vector<T> > m, vector<vector<T> > val){
	if (m.size()!=val.size() || m[0].size()!=val[0].size()) return m;
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]*=val[i][j];
	return m;
}

template<typename T>
void operator /= (vector<vector<T> > &m, vector<vector<T> > val){
	if (m.size()!=val.size() || m[0].size()!=val[0].size()) return;
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]/=val[i][j];
}

template<typename T>
vector<vector<T> > operator / (vector<vector<T> > m, vector<vector<T> > val){
	if (m.size()!=val.size() || m[0].size()!=val[0].size()) return m;
	for(int i=0; i<m.size(); i++)
		for(int j=0; j<m[0].size(); j++)
			m[i][j]/=val[i][j];
	return m;
}

template<typename T>
T max_matr(vector<vector<T> > m){
	T res=-INF;
	for (int i=0; i<m.size(); i++)
		for (int j=0; j<m[0].size(); j++)
			if(m[i][j]<INF && m[i][j]>res) res=m[i][j];
	return res;
}

template<typename T>
T max_matr(vector<vector<T> > m, int &x, int &y){
	T res=-INF;
	for (int i=0; i<m.size(); i++)
		for (int j=0; j<m[0].size(); j++)
			if(m[i][j]<INF && m[i][j]>res){
				x=i; y=j;
				res=m[i][j];
			}
	return res;
}

template<typename T>
vector<vector<T> > max2matr(vector<vector<T> > m1, vector<vector<T> > m2){
	if(m1.size()!=m2.size() || m1[0].size()!=m2[0].size()) return m1;
	for (int i=0; i<m1.size(); i++)
		for (int j=0; j<m1[0].size(); j++)
			m1[i][j]=m1[i][j]>m2[i][j]?m1[i][j]:m2[i][j];
	return m1;
}

template<typename T>
void max2matr_(vector<vector<T> > &m1, vector<vector<T> > m2){
	if(m1.size()!=m2.size() || m1[0].size()!=m2[0].size()) return;
	for (int i=0; i<m1.size(); i++)
		for (int j=0; j<m1[0].size(); j++)
			m1[i][j]=m1[i][j]>m2[i][j]?m1[i][j]:m2[i][j];
}

template<typename T>
T min_matr(vector<vector<T> > m){
	T res=INF;
	for (int i=0; i<m.size(); i++)
		for (int j=0; j<m[0].size(); j++)
			if(m[i][j]<res) res=m[i][j];
	return res;
}

template<typename T>
T min_matr(vector<vector<T> > m, int &x, int &y){
	T res=INF;
	for (int i=0; i<m.size(); i++)
		for (int j=0; j<m[0].size(); j++)
			if(m[i][j]<res){
				x=i; y=j;
				res=m[i][j];
			}
	return res;
}

template<typename T>
vector<vector<T> > min2matr(vector<vector<T> > m1, vector<vector<T> > m2){
	if(m1.size()!=m2.size() || m1[0].size()!=m2[0].size()) return m1;
	for (int i=0; i<m1.size(); i++)
		for (int j=0; j<m1[0].size(); j++)
			m1[i][j]=m1[i][j]<m2[i][j]?m1[i][j]:m2[i][j];
	return m1;
}

template<typename T>
void min2matr_(vector<vector<T> > &m1, vector<vector<T> > m2){
	if(m1.size()!=m2.size() || m1[0].size()!=m2[0].size()) return;
	for (int i=0; i<m1.size(); i++)
		for (int j=0; j<m1[0].size(); j++)
			m1[i][j]=m1[i][j]<m2[i][j]?m1[i][j]:m2[i][j];
}

template<typename T>
T mean_matr(vector<vector<T> > m){
	T res = 0;
	for (int i=0; i<m.size(); i++)
		for (int j=0; j<m[0].size(); j++)
			res += m[i][j];
	return res/(m.size()*m[0].size());
}

template<typename T>
vector<vector<T>> sum_matr(vector<vector<T> > m1, vector<vector<T> > m2){
	if(m1.size()!=m2.size()) return m1;
	if(m1[0].size()!=m2[0].size()) return m1;
	for(int i=0; i<m1.size(); i++)
		for(int j=0; j<m1[0].size(); j++)
			m1[i][j] += m2[i][j];
	return m1;
}

template<typename T>
vector<vector<T> > norm_matr(vector<vector<T> > m){
	T max_val = find_matr_max(m);
	for (int i=0; i<m.size(); i++)
		for (int j=0; j<m[0].size(); j++)
			m[i][j]/=max_val;
	return m;
}

template<typename T>
vector<vector<T> > norm_matr(vector<vector<T> > m, T val){
	for (int i=0; i<m.size(); i++)
		for (int j=0; j<m[0].size(); j++)
			m[i][j]/=val;
	return m;
}

template<typename T>
vector<vector<T> > prod_matr(vector<vector<T> > m, T val){
	for (int i=0; i<m.size(); i++)
		for (int j=0; j<m[0].size(); j++)
			m[i][j]*=val;
	return m;
}

template<typename T>
vector<vector<T> > sort_matr(vector<vector<T> > m, int dim, int order){
	int x=m.size(), y=m[0].size();
	vector<vector<T> > res(x,vector<T>(y));
	if(!dim){
		// Por filas
		for(int i=0; i<x; i++){
			res[i]=sort_vect(m[i],order);
		}
	}else{
		// Por columnas
		vector<T> aux(x);
		for(int i=0; i<y; i++){
			// Formar el vector
			for(int j=0; j<x; j++) aux[j] = m[j][i];
			aux = sort_vect(aux,order);
			for(int j=0; j<x; j++) res[j][i] = aux[j];
		}
	}
	return res;
}

template <typename T>
bool matrix_equality(vector<vector<T>> m1, vector<vector<T>> m2)
{
    if(m1.size() != m2.size()) return false;
    for(int i=0; i<m1.size(); i++){
        if(m1[i].size() != m2[i].size()) return false;
        for(int j=0; j<m1[i].size(); j++){
            if(m1[i][j] != m2[i][j]) return false;
        }
    }
    return true;
}

template <typename T>
bool matrix_equality(vector<vector<T>> m1, vector<vector<T>> m2, vector<int> &xdiff, vector<int> &ydiff)
{
    if(m1.size() != m2.size()) return false;
    for(int i=0; i<m1.size(); i++){
        if(m1[i].size() != m2[i].size()) return false;
        for(int j=0; j<m1[i].size(); j++){
            if(m1[i][j] != m2[i][j]){
                xdiff.push_back(i);
                ydiff.push_back(j);
            }
        }
    }
    if(!xdiff.size()) return true;
    else return false;
}

template <typename T>
float compute_distance(T x1, T y1, T x2, T y2)
{
	return sqrt(pow((float)x1 - (float)x2, 2) + pow((float)y1 - (float)y2, 2));
}


//==========================================================================================
//==========================================================================================
//==========================================================================================

// Guardado de las cosas
vector<int> iteration_data(Pos<int> agent, Pos<int> goal);
vector<int> iteration_data(Poss<int> agents_poss, Poss<int> goals_poss);

template <typename T>
void save_vect(const char* filename, vector<T> v){
    string line; // línea
    ofstream file(filename); // abrir para escribr
    if(file.is_open()){
        file << "v=[ ";
        for(int i=0; i<v.size(); i++) line+=to_string(v[i])+ " ";
        file << line << "];\n";
        // Cerrar el fichero
        file.close();
    }else cout<<"No se puede guardar en ese fichero"<<endl;
} // guardar vector (nuevo)

template <typename T>
void save_vect_(const char* filename, vector<T> v){
    string line; // línea
    ofstream file(filename, ios::app); // abrir para escribir
    if(file.is_open()){
        file << "v=[ ";
        for(int i=0; i<v.size(); i++) line+=to_string(v[i])+ " ";
        file << line << "];\n";
        // Cerrar el fichero
        file.close();
    }else cout<<"No se puede guardar en ese fichero"<<endl;
} // guardar vector (append)

template <typename T>
void save_vect_(const char* filename, vector<T> v, string name){
    string line; // línea
    ofstream file(filename, ios::app); // abrir para escribir
    if(file.is_open()){
        file << (name+"=[ ");
        for(int i=0; i<v.size(); i++) line+=to_string(v[i])+ " ";
        file << line << "];\n";
        // Cerrar el fichero
        file.close();
    }else cout<<"No se puede guardar en ese fichero"<<endl;
} // guardar vector (append)

template <typename T>
void save_matr(const char* filename, vector<vector<T> > m){
    string line; // línea
    ofstream file(filename); // abrir para escribr
    int xc=0,yc=0; // contadores para recorrer la matriz
    int sx=m.size(),sy=m[0].size(); // tamaño de la matriz
    //~ int i;
    if(file.is_open()){
        // Esto es el grid
        file << "grid=[\n";
        while (yc<sy){
            line.clear();
            for(xc=0;xc<sx;xc++) line+=to_string(m[xc][yc])+ " ";
            file << line << "\n";
            yc++;
        }
        file << "];\n";
        // Cerrar el fichero
        file.close();
    }else cout<<"No se puede guardar en ese fichero"<<endl;
} // guardar matriz

template <typename T>
void save_pos(const char* filename, vector<T> x, vector<T> y){
    string line;
    ofstream file(filename); // abrir para escribir
    if(file.is_open()){
        // Esto es el camino
        file << "pos = [\n";
        line.clear();
        file << "x: ";
        for(int i=0;i<x.size();i++) line+=to_string(x[i])+" ";
        file << line << "\n";line.clear();
        file << "y: ";
        for(int i=0;i<y.size();i++) line+=to_string(y[i])+" ";
        file << line << "\n";line.clear();
        file << "];";
        // Cerrar el fichero
        file.close();
    }else cout<<"No se puede guardar en ese fichero"<<endl;
}

template <typename T>
void save_poss_(const char* filename, vector<T> x, vector<T> y){
    save_vect_(filename, x, "x");
	save_vect_(filename, y, "y");
}

vector<vector<float>> matrBool2Float(vector<vector<bool>> matr);
vector<vector<float>> matrInt2Float(vector<vector<int>> matr);


#endif
