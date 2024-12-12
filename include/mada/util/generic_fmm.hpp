#ifndef GENERIC_FMM_H
#define GENERIC_FMM_H

#include "funciones.hpp"

static vector<int> move_x={0, 1, 1, 1, 0,-1,-1,-1};
static vector<int> move_y={1, 1, 0,-1,-1,-1, 0, 1};

class generic_FMM{
	private:
		float INF_VAL = float(RAND_MAX);
		//float INF_VAL = numeric_limits<float>::max();
		// Varibles del grid
		float tam_p; // distancia entre los puntos (POR AHORA)
		vector<vector<vector<float> > > velocity_maps; // mapas con las velocidades de propagación para cada fuente
		vector<vector<float> > distance_map; // mapas de las transformadas de distancia para cada agente (LO MÁS PROBABLE ES QUE SEA UN ÚNICO MAPA PARA TODOS)
		int sx,sy; // tamaño del grid
		
		vector<int> sizes; // Tamaños de los objetos/agentes para los que se está claculando el gradiente
		int _x, _y; // Auxiliar
		// Los lejanos de cada fuente
		vector<int> xf, yf; // Sus posiciones
		vector<float> df; // Distancia desde la fuente

		// Parámetros propios de la segmentación
		vector<int> areas;
		vector<vector<bool>> adjacency;

		// Variables propias de los goals
		vector<int> ngoals; // Cantidad de goals en cada segmento
		vector<vector<bool>> goals_map; // Mapa que contiene los goals

		// Posiciones de los "goals"
		vector<int> xgoal_close, ygoal_close; // Los goals más cercanos
		vector<float> dgoal_close;
		vector<int> xgoal_far, ygoal_far; // Los goals más lejanos
		vector<float> dgoal_far;
		vector<vector<int>> xa, ya; // Los adyacentes más cercanos
		vector<vector<float>> da;
		
		// variables del frente
		vector<int> xw,yw; // posiciones sobre el grid
		vector<float> cw; // costes
		vector<int> sw; // fuentes
		int ws; // tamaño del frente
		
		// valor instantáneo del frente
		int ax,ay; // Posición
		float ac; // Coste
		int asi; // índice de la fuente
		vector<vector<float> > rhs,v; // gradiente y la matriz de nodos no "descubiertos"

		vector<vector<int> > sources_map; // índices de los frentes
		
		vector<int> dx,dy; // incrementos permitidos
		int nx, ny; // índices de los vecinos de los puntos del frente
		int ns; // índice de la fuente de donde proviene el frente
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

		// ESTO ES PARA LA PRUEBA DE ACUMULAR TODOS LOS PUTNOS DENTRO
		vector<Poss<int>> segs_poss;
		
	public:
		// Constructores
		generic_FMM(vector<int> x, vector<int> y, vector<vector<vector<float> > > velocity_maps);
		generic_FMM(vector<int> x, vector<int> y, vector<vector<vector<float> > > velocity_maps, vector<vector<float> > distance_map);
		generic_FMM(vector<int> x, vector<int> y, vector<vector<vector<float> > > velocity_maps, vector<vector<float> > distance_map, vector<int> sizes);

		// Destructor
		~generic_FMM();

		void size_initialization();
		void areas_initialization();
		void goals_initialization(vector<int> xgoals, vector<int> ygoals);

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

		// Funciones propias del FMM
		void propagate(); // propagación del frente
		void update_vertex(); // calcular el valor de un punto del grid
		void computePropagator(); // puntos para la interpolación
		void kernel_LSM(); // calcular el valor del punto expandido
		void requeue(); // reordenar el frente

		void propagate_dt(); // propagación del frente considerando la distancia a los obstáculos

		void save_grad(string folder);

		// Devoluciones
		vector<vector<float> > get_rhs(); // Gradeinte
		vector<vector<int>> get_sources_map(); // Mapa de los segmentos

		Poss<int> get_far_pos(); // Posiciones de los lejanos
		Poss<int> get_far_pos(vector<float> &df); // Posiciones de los lejanos y sus respectivas distancias
		vector<int> get_areas(); // Áreas de los segmentos
		vector<vector<bool>> get_adjacency(); // Matriz de adyacencias
		vector<int> get_ngoals(); // Cantidad de goals dentro de cada segmento
		Poss<int> get_goals_far(); // Los goals más lejanos des cada segmento
		Poss<int> get_goals_far(vector<float> &d); // Los goals más lejanos des cada segmento y sus respectivas distancias
		Poss<int> get_goals_close(); // Los goals más cercanos des cada segmento
		Poss<int> get_goals_close(vector<float> &d); // Los goals más cercanos des cada segmento y sus respectivas distancias

		vector<Poss<int>> get_adjacent_pos(); // Posiciones de los adyacentes entre segmentos
		vector<Poss<int>> get_adjacent_pos(vector<vector<float>> &d); // Posiciones de los adyacentes entre segmentos y sus distancias

		vector<Poss<int>> get_segments_pos(); // Posiciones de los segmentos

};

namespace GFMM_util{
	vector<vector<vector<float>>> initialize_velocity_maps(vector<vector<float>> static_grid, vector<float> velocities);
	void set_velocity_maps(vector<vector<vector<float>>> &velocity_maps, vector<float> velocities);
	void set_velocity_maps(vector<vector<vector<float>>> &velocity_maps, Poss<int> free_poss, vector<float> velocities);
	vector<int> goals_in_segs(Poss<int> goals, vector<vector<int>> segments, int nsegs);
	vector<Poss<int>> poss_in_segs(Poss<int> poss, vector<vector<int>> segments, int nsegs);
	vector<float> goals_dispersion(Poss<int> goals, vector<vector<int>> segments, vector<vector<float>> grad, int nsegs);

	// Inicialización del objeto
	generic_FMM obj_initialization(vector<int> x, vector<int> y, vector<vector<float>> grid); // Lo básico
	generic_FMM obj_initialization(vector<int> x, vector<int> y, vector<float> velocities, vector<int> sizes, vector<vector<float>> grid); // Para equipo heterogeneo
	generic_FMM obj_initialization(vector<int> x, vector<int> y, vector<float> velocities, vector<int> sizes, vector<vector<float>> grid, vector<int> xg, vector<int> yg); // Teniendo unos objetivos
};

namespace fmm_segment{

	struct Segments{
		vector<vector<float>> segs_grad;
		vector<vector<int>> map;
		vector<vector<bool>> graph;
		Poss<int> centroids;
		vector<Poss<int>> poss;
		vector<vector<Poss<int>>> frontier_poss;
		vector<Poss<int>> contour_poss;
	}; // Todas las variables útiles de los segmentos

	vector<Poss<int>> segments_points(vector<vector<int>> segments, int segs);
	vector<vector<int>> generate_segments_graph(vector<vector<bool>> adjacency, vector<int> sizes);
	vector<vector<Poss<int>>> frontier_points(vector<vector<int>> segments, vector<vector<bool>> graph);
	vector<Poss<int>> contour_points(vector<vector<int>> segments, vector<vector<bool>> graph);

	vector<float> max_seg_dist(Poss<int> centroids, vector<Poss<int>> contour_poss);
	vector<vector<float>> distance_between_segments(Poss<int> centroids, vector<vector<bool>> graph);

	Segments compute_segments(vector<vector<float>> grid, vector<vector<float>> grad); // Calcular los segmentos y todas sus variables
	Segments compute_segments(Poss<int> centroids, vector<vector<float>> grad); // Calcular los segmentos y todas sus variables
};

#endif
