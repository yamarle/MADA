#ifndef VISIBILITY_FUNCTIONS_HPP
#define VISIBILITY_FUNCTIONS_HPP

#include "funciones.hpp"
#include "geometry.hpp"

namespace visibility{

	Poss<int> obtain_limits(int x, int y, int vrange, int sx, int sy)
	{
		int xl, xu, yl, yu;
        xl = (x-vrange) > 0 ? (x-vrange) : 0;
        xu = (x+vrange) < sx-1 ? (x+vrange) : sx-1;
        yl = (y-vrange) > 0 ? (y-vrange) : 0;
        yu = (y+vrange) < sy-1 ? (y+vrange) : sy-1;

        Poss<int> limits;
        for(int i=xl; i<=xu; i++){
            limits.x.push_back(i); limits.y.push_back(yu);
        }
        for(int i=yu; i>=yl; i--){
            limits.x.push_back(xu); limits.y.push_back(i);
        }
        for(int i=xu; i>=xl; i--){
            limits.x.push_back(i); limits.y.push_back(yl);
        }
        for(int i=yl; i<=yu; i++){
            limits.x.push_back(xl); limits.y.push_back(i);
        }
        return limits;
	} // Limites del rango de visión (Rectángulo)

	void update_positions(Pos<int> &pos1, Pos<int> &pos2, Pos<int> &delta, Pos<int> &incr)
    {
    	delta.x = (pos2.x - pos1.x);
        incr.x=((delta.x > 0) - (delta.x < 0));
        delta.x = abs(delta.x) << 1;

        delta.y=(pos2.y - pos1.y);
        incr.y=((delta.y > 0) - (delta.y < 0));
        delta.y = abs(delta.y) << 1;
    } // Actualización de las variables de la posiciones

    void update_x(Pos<int> &pos, Pos<int> &delta, int &error, Pos<int> incr)
    {
    	if ((error >= 0) && (error || (incr.x > 0))){
            error -= delta.x;
            pos.y += incr.y;
        }
        error += delta.y;
        pos.x += incr.x;
    } // Actaulización de las posiciones 

    void update_y(Pos<int> &pos, Pos<int> &delta, int &error, Pos<int> incr)
    {
    	if ((error >= 0) && (error || (incr.y > 0))){
            error -= delta.y;
            pos.x += incr.x;
        }
        error += delta.x;
        pos.y += incr.y;
    }

    bool update_visibility(int grid_value, bool &blind, bool &done_pos)
	{
		if(!grid_value){ // A partir de aquí no veo nada
            if(!blind){ // Antes veía
                blind = true; // A partir de aquí no se ve nada
                if(!done_pos){ // Para no insertarlo repetido (de distintos rayos)
                    done_pos = true;
                    return true;
                }else return false;
            }else return false;
        }else{
            if(!blind){
                if(!done_pos){ // Para no insertarlo repetido (de distintos rayos)
                    done_pos = true;
                    return true;
                }else return false;
            }else return false;
        }
	} // Actualización de las variables de la línea de visión

	Poss<int> obtain_visible_poss(int x, int y, vector<vector<float>> grid, int vrange)
    {
        // "grid" ES EL GRID DEL MUNDO REAL
        // "global_map" ES EL GRID QUE ALMACENA EL AGENTE

        Poss<int> visible_poss;

        Poss<int> limites = obtain_limits(x, y, vrange, grid.size(), grid[0].size());

        // Variables para bresenham
        Pos<int> pos1, pos2;
        Pos<int> incr, delta;
        int error;

        vector<vector<bool>> done(grid.size(), vector<bool>(grid[0].size(),false));

        bool blind;
        bool v;

        for(int i=0; i<limites.x.size(); i++){ // Cada posición del contorno
            pos1.x = x; pos1.y = y; // Inicio en la posición del robot
            pos2.x = limites.x[i]; pos2.y = limites.y[i]; // Limites del rango de visión

            blind = false;
            update_positions(pos1, pos2, delta, incr);

            if(!grid[pos1.x][pos1.y]){ // La posición es un obstáculo
                break;
            }

            if(!done[pos1.x][pos1.y]){
                done[pos1.x][pos1.y] = true;
                visible_poss.push(pos1);
            }

            // Recorrer las posiciones de la línea con bresenham
            if(delta.x >= delta.y){
                error = delta.y - (delta.x >> 1);
                while (pos1.x != pos2.x){
                    update_x(pos1, delta, error, incr);
                    
                    if(sqrt(pow(pos1.x-x,2)+pow(pos1.y-y,2))<=vrange){
	                    v = done[pos1.x][pos1.y];
	                    if(update_visibility(grid[pos1.x][pos1.y], blind, v)){
	                    	visible_poss.push(pos1);
	                    }
	                    done[pos1.x][pos1.y] = v;
	                }
                }
            }else{
                error = (delta.x - (delta.y >> 1));
                while (pos1.y != pos2.y){
                    update_y(pos1, delta, error, incr);

                    if(sqrt(pow(pos1.x-x,2)+pow(pos1.y-y,2))<=vrange){
	                    v = done[pos1.x][pos1.y];
	                    if(update_visibility(grid[pos1.x][pos1.y], blind, v)){
	                    	visible_poss.push(pos1);
	                    }
	                    done[pos1.x][pos1.y] = v;
	                }
                }
            }
        }
        return visible_poss;
    }

    struct view{
        Poss<int> poss;
        vector<float> values;
    }; // Variable de lo que se ve

    view visibility(int xreal, int yreal, float vrange, vector<vector<float>> real_map)
    {
        view res;
        res.poss = obtain_visible_poss(xreal, yreal, real_map, vrange);
        // Insertat la posición del agente (NO ENTIENDO POR QUE NO ESTA EN LOS PUNTOS VISIBLES)
        res.poss.x.insert(res.poss.x.begin(), xreal);
        res.poss.y.insert(res.poss.y.begin(), yreal);
        res.values.resize(res.poss.x.size(),0);
        for(int i=0; i<res.values.size(); i++)
            res.values[i] = real_map[res.poss.x[i]][res.poss.y[i]];
        return res;
    }

};

#endif