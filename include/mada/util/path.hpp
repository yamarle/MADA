#ifndef PATH_HPP
#define PATH_HPP

#include "funciones.hpp"

class Path{
	public:
		int tam = 0;
		vector<int> x,y; // pueden ser int (de un grid) y float (un camino "no discreto")
		vector<float> dpp,tpp; // distancia y tiempo entre los puntos
		vector<float> dc,tc; // distancia y tiempo acumulados en cada punto
		float d=0,t=0; // distancia y tiempo totales
		
		Path(){};
		~Path(){};

		bool compare(Path p){
			if(p.x.size() != x.size()) return false;
			for(int i=0; i<x.size(); i++)
				if(x[i]!=p.x[i] || y[i] != p.y[i]) return false;
			return true;
		}

		void operator = (Path p){
			x=p.x; y=p.y;
			dpp=p.dpp; tpp=p.tpp;
			dc=p.dc; tc=p.tc;
			d=p.d; t=p.t;
			tam=p.tam;
		}
		
		void set(vector<int> x, vector<int> y){
			this->x=x;
			this->y=y;
			tam=x.size();
		} // si directamente recibo las posiciones

		void set(int x, int y){
			this->x={x};
			this->y={y};
			tam=1;
		} // si directamente recibo las posiciones
		
		void clear(){
			x.clear(); y.clear();
			dpp.clear(); tpp.clear();
			dc.clear(); tc.clear();
			d=0; t=0;
			tam=0;
		}

		void initialize(int x, int y){
			this->x.push_back(x); this->y.push_back(y);
			dc.push_back(0); dpp.push_back(0);
			tc.push_back(0); tpp.push_back(0);
			d=0; t=0;
			tam=1;
		}

		void invert(){
			if (!tam) return;
			x=reverse(x);
			y=reverse(y);
			if (dpp.size()) dpp=reverse(dpp);
			if (tpp.size()) tpp=reverse(tpp);
			if (dc.size()) dc=reverse(dc);
			if (tc.size()) tc=reverse(tc);
		}

		void pop(){
			if(!tam) return;
			x.erase(x.begin()); y.erase(y.begin());
			if(dpp.size()){
				d -= dpp[0];
				for(int i=0; i<tam; i++) dc[i]-=dpp[0];
				dpp.erase(dpp.begin());
				dc.erase(dc.begin());
			}
			if(tpp.size()){
				t -= tpp[0];
				for(int i=0; i<tam; i++) tc[i]-=tpp[0];
				tpp.erase(tpp.begin());
				tc.erase(tc.begin());
			}
			tam--;
		}

		Path back(){
			Path res;
			if(!tam) return res;
			res.d=d; res.t=t;
			for(int i=tam-2; i>=0; i--){
				res.x.push_back(x[i]); res.y.push_back(y[i]);
				res.tam++;
				if(d){
					res.dpp.push_back(dpp[i+1]);
					res.d+=dpp[i+1];
					res.dc.push_back(res.d);
				}
				if(t){
					res.tpp.push_back(tpp[i+1]);
					res.t+=tpp[i+1];
					res.tc.push_back(res.t);
				}
			}
			return res;
		} // Invertir el camino???????????

		void back_app(){
			if(!tam) return;
			for(int i=tam-1; i>=0; i--){
				x.push_back(x[i]); y.push_back(y[i]);
				tam++;
				if(d){
					dpp.push_back(dpp[i+1]);
					d+=dpp[tam-1];
					dc.push_back(d);
				}
				if(t){
					tpp.push_back(tpp[i+1]);
					t+=tpp[tam-1];
					tc.push_back(t);
				}
			}
		}
				
		void compute_distance(){
			dpp.resize(tam);
			dc.resize(tam);
			for(int i=1;i<tam;i++){
				dpp[i]=sqrt(pow(x[i-1]-x[i],2)+pow(y[i-1]-y[i],2));
				dc[i]=dc[i-1]+dpp[i];
			}
			d=dc[tam-1];
		}
		
		float compute_distance_(){
			compute_distance();
			return d;
		}
		
		void compute_time(float vmax){ // velocidad máxima del robot
			if(tpp.size()>0){ // reinicio los tiempos
				tpp.clear(); tc.clear();
			}
			tpp.resize(tam);
			tc.resize(tam);
			if(dc.size()>0){
				for(int i=1;i<tam;i++){
					tpp[i]=dpp[i]/vmax;
					tc[i]=tc[i-1]+tpp[i];
				}
			}else{
				dpp.resize(tam);
				dc.resize(tam);
				for(int i=1;i<tam;i++){
					dpp[i]=sqrt(pow(x[i-1]-x[i],2)+pow(y[i-1]-y[i],2));
					dc[i]=dc[i-1]+dpp[i];
					tpp[i]=dpp[i]/vmax;
					tc[i]=tc[i-1]+tpp[i];
				}
				d=dc[tam-1];
			}
			t=tc[tam-1];
		}
		
		float compute_time_(float vmax){
			compute_time(vmax);
			return t;
		}

		void naive_time(){
			if(!tpp.size()){
				tpp.resize(tam); dpp.resize(tam);
				tc.resize(tam); dc.resize(tam);
			}
			tpp[0] = 0; tc[0] = 0;
			dpp[0] = 0; dc[0] = 0;
			for(int i=1; i<tam; i++){
				tpp[i] = 1; tc[i] = i;
				dpp[i] = 1; dc[i] = i;
			}
			t = tam; d = tam;
		}
		
		void push_back(int x,int y){
			this->x.push_back(x);
			this->y.push_back(y);
			tam++;
		} // insertar al final (tonteria)

		void push_back(int x,int y,float t){
			this->x.push_back(x);
			this->y.push_back(y);
			dpp.push_back(sqrt(pow(x-this->x[tam-1],2)+pow(y-this->y[tam-1],2)));
			tpp.push_back(t-this->t);
			dc.push_back(d+dpp[dpp.size()-1]);
			this->t=t;
			tc.push_back(t);
			tam++;
		} // insertar al final (tonteria)

		void pop_back(){
			if(tam<=1) return;
			x.pop_back(); y.pop_back();
			tam--;
			if(dpp.size()>1){
				dc.pop_back(); dpp.pop_back();
				d = dc[tam-1];
			}
			if(tpp.size()>1){
				tc.pop_back(); tpp.pop_back();
				t = tc[tam-1];
			}
		} // Quitar el último punto del camino
		
		void insert(int w, int x, int y){
			this->x.insert(this->x.begin()+w,x);
			this->y.insert(this->y.begin()+w,y);
			tam++;
		}

		void push_front(int x, int y, float v){
			
			//cout<<"Tamaños: "<<this->x.size()<<", "<<this->y.size()<<", "<<dpp.size()<<", "<<dc.size()<<", "<<tpp.size()<<", "<<tc.size()<<", "<<tam<<endl;
			tam++;
			this->x.insert(this->x.begin(),x);
			this->y.insert(this->y.begin(),y);
			//cout<<"Tamaños: "<<this->x.size()<<", "<<this->y.size()<<", "<<dpp.size()<<", "<<dc.size()<<", "<<tpp.size()<<", "<<tc.size()<<", "<<tam<<endl;

			dpp.insert(dpp.begin()+1, sqrt(pow(this->x[1]-this->x[0],2)+pow(this->y[1]-this->y[0],2)));
			tpp.insert(tpp.begin()+1, dpp[1]/v);
			//cout<<"Tamaños: "<<this->x.size()<<", "<<this->y.size()<<", "<<dpp.size()<<", "<<dc.size()<<", "<<tpp.size()<<", "<<tc.size()<<", "<<tam<<endl;

			d=0; t=0;
			dc.resize(tam);
			tc.resize(tam);

			//cout<<"Tamaños: "<<this->x.size()<<", "<<this->y.size()<<", "<<dpp.size()<<", "<<dc.size()<<", "<<tpp.size()<<", "<<tc.size()<<", "<<tam<<endl;
			for(int i=1; i<tam; i++){
				dc[i]=dc[i-1]+dpp[i];
				tc[i]=tc[i-1]+tpp[i];
			}
			t=tc[tam-1];
			d=dc[tam-1];
		}

		void push_front(Path p, float v){
			for(int i=p.tam-1; i; i--){
				x.insert(x.begin(), p.x[i]); y.insert(y.begin(), p.y[i]);
				tam++;
			}

			if(!tc.size()){
				if(dc.size())
					compute_distance();
			}else if(tc.size()){
				compute_time(v);
			}
		}

		void push_front_(Path p){
			for(int i=p.tam-1; i>=0; i--){
				x.insert(x.begin(), p.x[i]); y.insert(y.begin(), p.y[i]);
				tam++;
			}

			if(dc.size())
				compute_distance();
			
		}
		
		void push_back(Path p){
			for(int i=0; i<p.tam; i++){
				this->x.push_back(p.x[i]);
				this->y.push_back(p.y[i]);
				if(p.dpp.size()>0) this->dpp.push_back(p.dpp[i]);
			}
			// NO ENTIENDO PORQUÉ, PERO DE LA OTRA MANERA INSERTA UN VALOR DEMÁS
			//~ this->tam+=p.tam;
			this->tam=x.size();
		} // insertar un camino al final (concatenar)
		
		void append_pos(Path p){
			for(int i=1; i<p.tam; i++){
				x.push_back(p.x[i]); y.push_back(p.y[i]);
				tam++;
			}
		} // insertar al final solo las posiciones (descartando la anterior)

		void append(Path p){
			for(int i=0; i<p.tam; i++){
				x.push_back(p.x[i]); y.push_back(p.y[i]);
				dpp.push_back(p.dpp[i]); tpp.push_back(p.tpp[i]);
				d+=p.dpp[i]; t+=p.tpp[i];
				dc.push_back(d); tc.push_back(t);
				tam++;
			}
		} // inserta al final

		void append_(Path p){
			if(tpp.size() && p.tpp.size()){
				for(int i=1; i<p.tam; i++){
					x.push_back(p.x[i]); y.push_back(p.y[i]);
					dpp.push_back(p.dpp[i]); tpp.push_back(p.tpp[i]);
					d+=p.dpp[i]; t+=p.tpp[i];
					dc.push_back(d); tc.push_back(t);
					tam++;
				}
			}else if(dpp.size() && p.dpp.size()){
				for(int i=1; i<p.tam; i++){
					x.push_back(p.x[i]); y.push_back(p.y[i]);
					d+=p.dpp[i];
					dc.push_back(d); dpp.push_back(p.dpp[i]);
					tam++;
				}
			}else{
				for(int i=1; i<p.tam; i++){
					x.push_back(p.x[i]); y.push_back(p.y[i]);
					tam++;
				}
			}
		} // lo mismo que la anterior, pero descarto el primer elemento

		void append_aux(Path p){
			if(tpp.size() && p.tpp.size()){
				for(int i=1; i<p.tam; i++){
					x.push_back(p.x[i]); y.push_back(p.y[i]);
					dpp.push_back(p.dpp[i]); tpp.push_back(p.tpp[i]);
					d+=p.dc[i]-p.dc[i-1]; t+=p.tc[i]-p.tc[i-1];
					dc.push_back(d); tc.push_back(t);
					tam++;
				}
			}else if(dpp.size() && p.dpp.size()){
				for(int i=1; i<p.tam; i++){
					x.push_back(p.x[i]); y.push_back(p.y[i]);
					d+=p.dpp[i];
					dc.push_back(d); dpp.push_back(p.dpp[i]);
					tam++;
				}
			}else{
				for(int i=1; i<p.tam; i++){
					x.push_back(p.x[i]); y.push_back(p.y[i]);
					tam++;
				}
			}
		} // lo mismo que la anterior, pero descarto el primer elemento

		void append2(Path p, float v){
			// Hay que mirar el enlace con el primer componente
			p.dpp[0]=sqrt(pow(p.x[0]-x[tam-1],2)+pow(p.y[0]-y[tam-1],2));
			p.tpp[0]=(p.dpp[0]/v)<p.tc[0]?(p.dpp[0]/v):p.tc[0];
			//p.dc[0]=dc[tam-1]+p.dpp[0];
			//p.tc[0]=t+p.tpp[0];
			for(int i=0; i<p.tam; i++){
				x.push_back(p.x[i]); y.push_back(p.y[i]);
				dpp.push_back(p.dpp[i]); tpp.push_back(p.tpp[i]);
				d+=p.dpp[i]; t+=tpp[i];
				dc.push_back(d); tc.push_back(t);
				tam++;
			}
		} // inserta el camino al final, mirando la distancia y el tiempo entre los dos caminos
		
		int find(int x, int y){
			for (int i=0; i<tam; i++)
				if(x==this->x[i] && y==this->y[i]) return i;
			return -1;
		} // encontrar el índice donde está situado un punto del camino (en base a la posición)

		int find_d(float dist){
			for(int i=1; i<tam; i++)
				if(dist>=dc[i-1] && dist<=dc[i]) return i-1;
			return -1;
		}

		int find_t(float time){
			for(int i=1; i<tam; i++)
				if(time>=tc[i-1] && time<=tc[i]) return i-1;
			return -1;
		}

		void initial_time(float t){
			for(int i=0; i<tam; i++) tc[i]+=t;
			this->t+=t;
		} // modificar el tiempo inicial del camino (En consecuencia cambian los tiempos acumulados y el final)

		void wait_until(float t){
			if(t>this->t){
				x.push_back(x[tam-1]); // Replicar el último punto
				y.push_back(y[tam-1]);
				dc.push_back(d); // La distancia recorrida sigue siendo la misma
				dpp.push_back(0); // No se ha movido
				tc.push_back(t); // El tiempo acumulado incrementa hasta t
				tpp.push_back(t-this->t); // El intervalo de espera es desde el último instante hasta terminar la espera
				this->t=t;
				tam++;
			}
		} // Se espera hasta el instante "t"

		void insert_wait(float t){
			x.push_back(x[tam-1]); // Replicar el último punto
			y.push_back(y[tam-1]);
			dc.push_back(d); // La distancia recorrida sigue siendo la misma
			dpp.push_back(0); // No se ha movido
			tc.push_back(t+this->t); // El tiempo acumulado incrementa hasta t
			tpp.push_back(t); // El intervalo de espera es desde el último instante hasta terminar la espera
			this->t+=t;
			tam++;
		} // Se inserta una espera de "t" segundos

		void interrupt(int pos){
			// pos: es el elemento desde el que se empieza a borrar
			if(pos<0) return;
			x.erase(x.begin()+pos,x.end());
			y.erase(y.begin()+pos,y.end());
			tam=x.size();
			if(dpp.size()){
				dpp.erase(dpp.begin()+pos,dpp.end());
				dc.erase(dc.begin()+pos,dc.end());
				d=dc[tam-1];
			}
			if(tpp.size()){
				tpp.erase(tpp.begin()+pos,tpp.end());
				tc.erase(tc.begin()+pos,tc.end());
				t=tc[tam-1];
			}
		} // Cortar el camino a partir del índice pos
		
		Path subPath(int s, int f){
			Path res;
			if (f>tam || s>tam){
				cerr<<"El camino es más pequeño"<<endl;
				return res;
			}
			res.x.insert(res.x.begin(),this->x.begin()+s,this->x.begin()+f);
			res.y.insert(res.y.begin(),this->y.begin()+s,this->y.begin()+f);
			if (dpp.size()>0) res.dpp.insert(res.dpp.begin(),this->dpp.begin()+s,this->dpp.begin()+f);
			if (tpp.size()>0) res.tpp.insert(res.tpp.begin(),this->tpp.begin()+s,this->tpp.begin()+f);
			if (dc.size()>0) res.dc.insert(res.dc.begin(),this->dc.begin()+s,this->dc.begin()+f);
			if (tc.size()>0) res.tc.insert(res.tc.begin(),this->tc.begin()+s,this->tc.begin()+f);
			if (dc.size()>0) res.d=this->dc[f];
			if (tc.size()>0) res.t=this->tc[f];
			res.tam=f-s;
			return res;
		} // extraer una parte del camino

		void subPath_(int s, int f){
			if (f>tam || s>tam){
				cerr<<"El camino es más pequeño"<<endl;
				return;
			}
			x.erase(this->x.begin(),this->x.begin()+s);
			x.erase(this->x.begin()+f+1,this->x.end());
			y.erase(this->y.begin(),this->y.begin()+s);
			y.erase(this->y.begin()+f+1,this->y.end());
			tam=x.size();
			compute_distance();
		} // extraer una parte del camino

		void subPath_(int s, int f, float v){
			if (f>tam || s>tam){
				cerr<<"El camino es más pequeño"<<endl;
				return;
			}
			if(s==0) s=1;
			x.erase(this->x.begin(),this->x.begin()+s-1);
			x.erase(this->x.begin()+f+1,this->x.end());
			y.erase(this->y.begin(),this->y.begin()+s-1);
			y.erase(this->y.begin()+f+1,this->y.end());
			tam=x.size();
			compute_time(v);
		} // extraer una parte del camino

		void subPath2(int s, int f, float v){
			if (f>tam || s>tam){
				cerr<<"El camino es más pequeño"<<endl;
				return;
			}
			if(s==0) s=1;
			x.erase(this->x.begin(),this->x.begin()+s-1);
			x.erase(this->x.begin()+f+1,this->x.end());
			y.erase(this->y.begin(),this->y.begin()+s-1);
			y.erase(this->y.begin()+f+1,this->y.end());
			tam=x.size();
			// Reiniciar los vectores de distancias tiempos
			dpp.clear(); tpp.clear(); dc.clear(); tc.clear();
			dpp.resize(tam); tpp.resize(tam); dc.resize(tam); tc.resize(tam);
			for(int i=1;i<tam;i++){
				dpp[i]=sqrt(pow(x[i-1]-x[i],2)+pow(y[i-1]-y[i],2));
				dc[i]=dc[i-1]+dpp[i];
				tpp[i]=dpp[i]/v;
				tc[i]=tc[i-1]+tpp[i];
			}
			d=dc[tam-1]; t=tc[tam-1];
		} // extraer una parte del camino
		
		void gradient_descent(vector<vector<float>> grad, int xi, int yi){
			// camino de [xs, ys] -> [xi, yi]
			int sx=grad.size(),sy=grad[0].size();
			// inserto el primero
			x.push_back(xi);
			y.push_back(yi);
			tam=1;
			//c.push_back(rhs[x][y]);
			// hay que mirar los vecinos en diagonal tambien
			//vector<int> nnx={0,1,1,1,0,-1,-1,-1},nny={-1,-1,0,1,1,1,0,-1};
			vector<int> nnx={0, 1, 1, 1, 0,-1,-1,-1};
			vector<int> nny={1, 1, 0,-1,-1,-1, 0, 1};
			int i; // has insertado el valor de partida
			int px=xi,py=yi; // índices previos
			int ix=xi,iy=yi; // índices instantáneos
			bool b;
			
			while(grad[px][py]){
				tam++;
				//x.resize(tam);y.resize(tam);
				x.push_back(px);y.push_back(py);
				b=false;
				for(i=0;i<8;i++){
					ix=px+nnx[i];iy=py+nny[i];
					if(ix>=0 && ix<sx && iy>=0 && iy<sy){ // si el vecino está dentro del grid
						if(grad[ix][iy]<grad[x[tam-1]][y[tam-1]]){ // seguríssimo que el vecino va a tener un valor menor (está más cerca del goal, según la formulación del FMM)
							x[tam-1]=ix; y[tam-1]=iy;
							b=true;
						}
					}
				}
				px=x[tam-1];py=y[tam-1];
				if(!b){
					//~ cout<<"Me temo que no existe un camino a ("<<xi<<", "<<yi<<")"<<endl;
					x.clear(); y.clear();
					tam=0;
					return;
				}
            }
		} // formación de un camino, descendiendo un gradiente
		
		void gradient_descent_(vector<vector<float>> grad, int xi, int yi){
			// camino de [xi, yi] -> [xs, ys]
			int sx=grad.size(),sy=grad[0].size();
			// inserto el primero
			x.push_back(xi);
			y.push_back(yi);
			tam=1;
			// hay que mirar los vecinos en diagonal tambien
			//vector<int> nnx={0,1,1,1,0,-1,-1,-1},nny={-1,-1,0,1,1,1,0,-1};
			vector<int> nnx={0, 1, 1, 1, 0,-1,-1,-1};
			vector<int> nny={1, 1, 0,-1,-1,-1, 0, 1};
			int i; // has insertado el valor de partida
			int px=xi,py=yi; // índices previos
			int ix=xi,iy=yi; // índices instantáneos
			bool b;
			
			while(grad[px][py]){
				tam++;
				x.insert(x.begin(),px);y.insert(y.begin(),py);
				b=false;
				for(i=0;i<8;i++){
					ix=px+nnx[i]; iy=py+nny[i];
					if(ix>=0 && ix<sx && iy>=0 && iy<sy){ // si el vecino está dentro del grid
						//~ if(grad[ix][iy]<grad[x[0]][y[0]]){ // seguríssimo que el vecino va a tener un valor menor (está más cerca del goal, según la formulación del FMM)
						if(grad[ix][iy]<grad[x[0]][y[0]]){ // seguríssimo que el vecino va a tener un valor menor (está más cerca del goal, según la formulación del FMM)
						//~ if(grad[ix][iy]<min){ // seguríssimo que el vecino va a tener un valor menor (está más cerca del goal, según la formulación del FMM)
							//~ x[tam-1]=ix; y[tam-1]=iy;
							x[0]=ix; y[0]=iy;
							b=true;
						}
					}
				}
				px=x[0]; py=y[0];
				if(!b){
					//~ cout<<"Me temo que no existe un camino a ("<<xi<<", "<<yi<<")"<<endl;
					x.clear(); y.clear();
					tam=0;
					return;
				}
            }
		} // formación de un camino, descendiendo un gradiente (se meten los valores al principio)
		
		void gradient_descent_comp(vector<vector<float>> grad, int xi, int yi, float vmax){
			int sx=grad.size(),sy=grad[0].size();
			// inserto el primero
			x.push_back(xi);
			y.push_back(yi);
			tam=1;
			// hay que mirar los vecinos en diagonal tambien
			//vector<int> nnx={0,1,1,1,0,-1,-1,-1},nny={-1,-1,0,1,1,1,0,-1};
			vector<int> nnx={0, 1, 1, 1, 0,-1,-1,-1};
			vector<int> nny={1, 1, 0,-1,-1,-1, 0, 1};
			int i; // has insertado el valor de partida
			int px=xi,py=yi; // índices previos
			int ix=xi,iy=yi; // índices instantáneos
			bool b;
			
			while(grad[px][py]){
				tam++;
				x.insert(x.begin(),0);y.insert(y.begin(),0);
				b=false;
				for(i=0;i<8;i++){
					ix=px+nnx[i];iy=py+nny[i];
					if(ix>=0 && ix<sx && iy>=0 && iy<sy){ // si el vecino está dentro del grid
						if(grad[ix][iy]<grad[x[0]][y[0]]){ // seguríssimo que el vecino va a tener un valor menor (está más cerca del goal, según la formulación del FMM)
							//~ x[tam-1]=ix; y[tam-1]=iy;
							x[0]=ix; y[0]=iy;
							b=true;
						}
					}
				}
				if (!b){
					//~ cout<<"Me temo que no existe un camino a ("<<xi<<", "<<yi<<")"<<endl;
					x.clear(); y.clear();
					tam=0;
					return;
				}
				
				// las variables
				dpp.push_back(sqrt(pow(px-x[0],2)+pow(py-y[0],2)));
				tpp.push_back(dpp[tam-1]/vmax);
				d+=dpp[tam-1];
				t+=tpp[tam-1];
				dc.push_back(d);
				tc.push_back(t);
				
				px=x[0]; py=y[0];
            }
		} // formación de un camino, descendiendo un gradiente (calculando a la vez las variables)

		bool inCollision(vector<vector<float>> grid){
			for(int i=0; i<x.size(); i++)
				if(grid[x[i]][y[i]] == 0) return true;
			return false;
		}
		
		//========================================================================
		//========================================================================
		//========================================================================
		// mostrar
		void show(){
			int i;
			cout<<"x: ";
			for(i=0;i<tam;i++) cout<<x[i]<<" ";
			cout<<endl;
			cout<<"y: ";
			for(i=0;i<tam;i++) cout<<y[i]<<" ";
			cout<<endl;
		} // mostrar camino entero
		
		void show(int i){
			cout<<i<<": ("<<x[i]<<", "<<y[i]<<")"<<endl;
		} // mostrar sólo un punto

		void show_sg(){
			if(!tam) return;
			if(!t)
				cout<<"("<<x[0]<<", "<<y[0]<<") - ("<<x[tam-1]<<", "<<y[tam-1]<<")"<<endl;
			else
				cout<<"("<<x[0]<<", "<<y[0]<<", "<<tc[0]<<") - ("<<x[tam-1]<<", "<<y[tam-1]<<", "<<t<<")"<<endl;
		} // mostrar punto de inicio y fin del camino
		
		void show(string variables){
			for(int i=0; i<variables.size();i++){
				if (variables[i]=='x') sh_vect_h(x,"x");
				if (variables[i]=='y') sh_vect_h(y,"y");
				if (variables[i]=='d') sh_vect_h(dc,"dc");
				if (variables[i]=='t') sh_vect_h(tc,"tc");
			}
		} // mostrar las variables que quiero
		
		//========================================================================
		//========================================================================
		//========================================================================		
		// devolver las variables del camino 
		// PA QUE, SI TODAS LAS VARIABLES SON PÚBLICAS, IDIOTA
		vector<int> getX(){return x;}
		vector<int> getY(){return y;}
		vector<float> getD(){return dpp;}
		vector<float> getT(){return tpp;}
        float getTD(){return d;}
        float getTT(){return t;}
        //========================================================================
		
		void save(const char* filename){
			if(!tam) return;
			string line; // línea
			ofstream file(filename); // abrir para escribir
			if(file.is_open()){
				// Esto es el camino
				file << "path = [\n";
				int i;
				line.clear();
				file << "x: ";
				for(i=0;i<tam;i++) line+=to_string(x[i])+" ";
				file << line << "\n";line.clear();
				file << "y: ";
				for(i=0;i<tam;i++) line+=to_string(y[i])+" ";
				file << line << "\n";line.clear();
				if (dc.size()>0){
					file << "d: ";
					for(i=0;i<tam;i++) line+=to_string(dc[i])+" ";
					file << line << "\n";
				}
                line.clear();
				if (tc.size()>0){
					file << "t: ";
					for(i=0;i<tam;i++) line+=to_string(tc[i])+" ";
					file << line << "\n";
				}
				file << "];";
				// Cerrar el fichero
				file.close();
			}else cout<<"No se puede guardar en ese fichero"<<endl;
        } // guardar en un fichero
		
        void load(const char* filename){

        	if(tam){
        		// Reiniciar el camino
        		tam=0;
        		x.clear(); y.clear();
        		tpp.clear(); tc.clear();
        		dpp.clear(); dc.clear();
        		d=0; t=0;
        	}

			string line, v;
			ifstream file(filename); // abrir para leer
			int value;
			float value_;
			if(file.is_open()){
				getline(file,line); // la primera lines lleva path=[
				// Cargar la x
				getline(file,line);
				for(int i=3; i<line.size(); i++){
					if(line[i]==' '){
						value=stoi(v);
						v.clear();
						x.push_back(value);
					}
					v+=line[i];
				}
				v.clear(); line.clear();
				// Cargar la x
				getline(file,line);
				for(int i=3; i<line.size(); i++){
					if(line[i]==' '){
						value=stoi(v);
						v.clear();
						y.push_back(value);
					}
					v+=line[i];
				}
				// Fijar el tamaño
				tam=x.size();
				v.clear(); line.clear();
				// Cargar la dc
				getline(file,line);
				if(line[0]=='d'){
					for(int i=3; i<line.size(); i++){
						if(line[i]==' '){
							value_=stod(v);
							v.clear();
							dc.push_back(value_);
						}
						v+=line[i];
					}
					v.clear(); line.clear();
				}
				// Cargar la tc
				getline(file,line);
				if(line[0]=='t'){
					for(int i=3; i<line.size(); i++){
						if(line[i]==' '){
							value_=stod(v);
							v.clear();
							tc.push_back(value_);
						}
						v+=line[i];
					}
					v.clear(); line.clear();
				}
			}
			
			
			if(dc.size()){
				d=dc[tam-1];
				dpp.resize(tam,0);
				for(int i=1; i<tam; i++){
					dpp[i]=dc[i]-dc[i-1];
				}
			}

			if(tc.size()){
				t=tc[tam-1];
				tpp.resize(tam,0);
				for(int i=1; i<tam; i++){
					tpp[i]=tc[i]-tc[i-1];
				}
			}

		}

};

#endif
