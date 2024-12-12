#ifndef FMM_2_H
#define FMM_2_H

#include "funciones.hpp"

class FMM{
	private:
		float INF_VAL = float(RAND_MAX);
		//float INF_VAL = numeric_limits<float>::max();
		// Varibles del grid
		float tam_p; // distancia entre los puntos (POR AHORA)
		vector<vector<float> > grid; // grid con los obstáculos (grid[x][y]==0: obstáculo)
		int sx,sy; // tamaño del grid
		// posición de inicio
		// variables del frente
		vector<int> xw, yw; // posiciones sobre el grid
		vector<float> cw; // costes
		int ws; // tamaño del frente
		
		// valor instantáneo del frente
		int ax,ay;
		float ac;
		vector<vector<float> > rhs,v; // gradiente y la matriz de nodos no "descubiertos"
		
		vector<int> dx, dy; // incrementos permitidos
		int nx, ny; // índices de los vecinos de los puntos del frente
		// los vecinos que se usan para interpolar (en este caso son 2)
		int qx1,qx2,qy1,qy2; // índices de los vecinos de los puntos para la interpolación
		float qc1,qc2;
		
		// Esta variable no sé si me va a hacer falta
		int qx_,qy_; // auxiliares para comparar con los de la interpolación
		float qc_;
		
		// Variables para la interpolación
		float Ta,Tc,beta,gamma;
		
		int presence(int x,int y); // comprobar la presencia de un punto en el frente
		//void initialize(); // inicializar las variables para el FMM
		//void initialize_sev(vector<int> x, vector<int> y); // inicializar ls variables del FMM, pero ahora hay distintos puntos de origen ("varios gradientes")
		
		// Variables auxiliares
		int pos,cont;
		float min,diff;

		int expanded_nodes;
		
	public:
		// Constructores
		FMM(int x,int y,vector<vector<float> > grid); // recibe directamente el grid
		FMM(vector<int> x, vector<int> y,vector<vector<float> > grid); // hay más de un punto de inicio

		//FMM();
		~FMM();

		// Calcular el gradiente de distintas maneras
		void compute_gradient(); // calcula pero no devuelve nada
		vector<vector<float> > compute_gradient_(); // calcula y devuelve el gradiente
		vector<vector<float> > expand_nodes(int n); // calcula (expande) el gradiente n puntos
		vector<vector<float> > expand2goal(int x, int y); // expande el gradiente hasta el goal (x,y)
		vector<vector<float> > expand_dist(float dist);
		vector<vector<float> > expand_dist(float dist, Poss<int> &pos);

		vector<vector<float> > pos_coverage(Poss<int> pos); // Hasta cubrir todas las posiciones
		vector<vector<float> > pos_coverage(int n, Poss<int> pos); // Hasta cubrir "n" posiciones de "pos"
		vector<vector<float> > pos_coverage(int n, Poss<int> pos, Poss<int> &covered); // Hasta cubrir "n" posiciones de "pos"
		vector<vector<float> > pos_coverage(vector<int> n, vector<Poss<int>> pos, vector<Poss<int>> &covered); // Hasta cubrir "n" posiciones de "pos"

		vector<vector<float> > expand2map_value(int n, int value, vector<vector<int>> map); // Hasta cubrir "n" de "map" que contienen el valor "value"
		vector<vector<float> > expand2map_value(int n, float value, vector<vector<float>> map); // Hasta cubrir "n" de "map" que contienen el valor "value"

		vector<vector<float> > expand2map_value(int n, int value, vector<vector<int>> map, Poss<int> &covered); // Hasta cubrir "n" de "map" que contienen el valor "value"
		vector<vector<float> > expand2map_value(int n, float value, vector<vector<float>> map, Poss<int> &covered); // Hasta cubrir "n" de "map" que contienen el valor "value"

		// Funciones propias del FMM
		void propagate(); // propagación del frente
		void update_vertex(); // calcular el valor de un punto del grid
		void computePropagator(); // puntos para la interpolación
		void kernel_LSM(); // calcular el valor del punto expandido
		void requeue(); // reordenar el frente

		void save_grad(string folder);

		vector<vector<float> > get_rhs();
};
#endif
