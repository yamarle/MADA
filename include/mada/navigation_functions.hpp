#ifndef NAVIGATION_FUNCTIONS_HPP
#define NAVIGATION_FUNCTIONS_HPP

#include <mada/util/fmm_2.hpp>

namespace navigation{
	
	// POSIBLES MOVIMIENTOS
    static vector<int> move_x={0, 0, 1, 1, 1, 0,-1,-1,-1};
    static vector<int> move_y={0, 1, 1, 0,-1,-1,-1, 0, 1};

    // POSIBLES MOVIMIENTOS (Sentido contrario a las agujas de reloj)
    static vector<int> cc_move_x={0,-1,-1,-1, 0, 1, 1, 1, 0};
    static vector<int> cc_move_y={0, 1, 0,-1,-1,-1, 0, 1, 1};

    vector<vector<float>> obtain_local_map(int x, int y, vector<vector<float>> grid, vector<vector<float>> static_map, vector<vector<float>> global_map, float vrange, Poss<int> &local_map_pos, Poss<int> &visible_pos, Poss<int> &new_obstacles)
    {
        // "grid" ES EL GRID DEL MUNDO REAL
        // "global_map" ES EL GRID QUE ALMACENA EL AGENTE
        
        // Inicialmente son todo obstáculos 
        vector<vector<float>> res(grid.size(), vector<float>(grid[0].size(), 0));

        // Límites del rango de visión
        int xl, xu, yl, yu;
        xl = (x-vrange) > 0 ? (x-vrange) : 0;
        xu = (x+vrange) < grid.size()-1 ? (x+vrange) : grid.size()-1;
        yl = (y-vrange) > 0 ? (y-vrange) : 0;
        yu = (y+vrange) < grid[0].size()-1 ? (y+vrange) : grid[0].size()-1;

        Poss<int> limites;
        for(int i=xl; i<=xu; i++){
            limites.x.push_back(i); limites.y.push_back(yu);
        }
        for(int i=yu; i>=yl; i--){
            limites.x.push_back(xu); limites.y.push_back(i);
        }
        for(int i=xu; i>=xl; i--){
            limites.x.push_back(i); limites.y.push_back(yl);
        }
        for(int i=yl; i<=yu; i++){
            limites.x.push_back(xl); limites.y.push_back(i);
        }

        // Varibales para bresenham
        int x1, x2, y1, y2;
        signed char ix, iy;
        int delta_x, delta_y, error;

        vector<vector<bool>> done(grid.size(), vector<bool>(grid[0].size(),false));

        bool blind;

        for(int i=0; i<limites.x.size(); i++){ // Cada posición del contorno
            x1 = x; y1 = y; // Inicio en la posición del robot
            x2 = limites.x[i]; y2 = limites.y[i]; // Limites del rango de visión

            blind = false;

            // BRESENHAM
            delta_x=(x2 - x1);
            ix=((delta_x > 0) - (delta_x < 0));
            delta_x = abs(delta_x) << 1;

            delta_y=(y2 - y1);
            iy=((delta_y > 0) - (delta_y < 0));
            delta_y = abs(delta_y) << 1;

            if(!grid[x1][y1]){
                break;
            }
            res[x1][y1] = 1;

            if(!done[x1][y1]){
                done[x1][y1] = true;
                visible_pos.x.push_back(x1);
                visible_pos.y.push_back(y1);
            }

            if(delta_x >= delta_y){
                error = delta_y - (delta_x >> 1);
                while (x1 != x2){
                    if ((error >= 0) && (error || (ix > 0))){
                        error -= delta_x;
                        y1 += iy;
                    }
                    error += delta_y;
                    x1 += ix;
                    /*
                    // Así no considero lo que hay detrás de los obstáculos
                    if(!grid[x1][y1] || sqrt(pow(x1-x,2)+pow(y1-y,2))>=vrange){
                        break;
                    }
                    res[x1][y1] = 1;
                    */
                    if(sqrt(pow(x1-x,2)+pow(y1-y,2))<=vrange){ // Solo si se está dentro del rango de visión
                        if(!grid[x1][y1]){ // A partir de aquí no veo nada
                            if(!blind){ // Antes veía
                                blind = true; // A partir de aquí no se ve nada
                                res[x1][y1] = 0;
                                //????????????????????????????????????
                                //done[x1][y1] = true; // <-------- ESTO ANTES NO ESTABA !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                // Aunque sea un obstáculo, lo veo
                                if(!done[x1][y1]){ // Para no insertarlo repetido (de distintos rayos)
                                    done[x1][y1] = true;
                                    visible_pos.x.push_back(x1);
                                    visible_pos.y.push_back(y1);
                                    // NUEVO
                                    if(static_map[x1][y1]){ // No es un obstáculo del mapa estático
                                        new_obstacles.x.push_back(x1);
                                        new_obstacles.y.push_back(y1);
                                    }
                                }
                                //????????????????????????????????????
                            }else{ // Ya NO veía nada, dejo lo que conocía previamente
                                res[x1][y1] = global_map[x1][y1];
                            }
                        }else{
                            if(blind){
                                res[x1][y1] = global_map[x1][y1];
                            }else{
                                res[x1][y1] = 1; // Posición libre
                                if(!done[x1][y1]){ // Para no insertarlo repetido (de distintos rayos)
                                    done[x1][y1] = true;
                                    visible_pos.x.push_back(x1);
                                    visible_pos.y.push_back(y1);
                                }
                            }
                        }
                        local_map_pos.x.push_back(x1);
                        local_map_pos.y.push_back(y1);
                    }
                    
                }
            }else{
                error = (delta_x - (delta_y >> 1));
                while (y1 != y2){
                    if ((error >= 0) && (error || (iy > 0))){
                        error -= delta_y;
                        x1 += ix;
                    }
                    error += delta_x;
                    y1 += iy;
                    if(sqrt(pow(x1-x,2)+pow(y1-y,2))<=vrange){ // Solo si se está dentro del rango de visión
                        
                        //cout<<"A: "<<blind<<" - "<<done[x1][y1]<<endl;

                        if(!grid[x1][y1]){ // A partir de aquí no veo nada
                            if(!blind){ // Antes veía
                                blind = true; // Dejo de ver
                                res[x1][y1] = 0;
                                //????????????????????????????????????
                                // Aunque sea un obstáculo, lo veo
                                if(!done[x1][y1]){
                                    done[x1][y1] = true;
                                    visible_pos.x.push_back(x1);
                                    visible_pos.y.push_back(y1);
                                    // NUEVO
                                    if(static_map[x1][y1]){ // No es un obstáculo del mapa estático
                                        new_obstacles.x.push_back(x1);
                                        new_obstacles.y.push_back(y1);
                                    }

                                }
                                //????????????????????????????????????

                            }else{ // Ya NO veía nada, dejo lo que conocía previamente
                                res[x1][y1] = global_map[x1][y1];
                            }
                        }else{
                            if(blind){
                                res[x1][y1] = global_map[x1][y1];
                            }else{
                                res[x1][y1] = 1;

                                if(!done[x1][y1]){
                                    done[x1][y1] = true;
                                    visible_pos.x.push_back(x1);
                                    visible_pos.y.push_back(y1);
                                }
                            }
                        }
                        local_map_pos.x.push_back(x1);
                        local_map_pos.y.push_back(y1);
                    }
                    
                }
            }
        }
        return res;
    }

    Poss<int> obtain_new_obstacles(Poss<int> visible_pos, vector<vector<float>> world_grid, vector<vector<float>> static_map)
    {
        Poss<int> res;
        for(int i=0; i<visible_pos.x.size(); i++)
            if(static_map[visible_pos.x[i]][visible_pos.y[i]] && !world_grid[visible_pos.x[i]][visible_pos.y[i]])
                res.push(visible_pos.x[i], visible_pos.y[i]);
        return res;
    } // Obtener obstáculos que no estaban en el mapa estático

    void update_global_map(int x, int y, vector<vector<float>> local_map, Poss<int> local_map_pos, vector<vector<float>> &global_map, float vrange)
    {
    	// "global_map" ES EL GRID QUE CONOCE EL AGENTE (todo lo que ha visto hasta entonces)
		for(int i=0; i<local_map_pos.x.size(); i++)
			if(local_map[local_map_pos.x[i]][local_map_pos.y[i]] && !global_map[local_map_pos.x[i]][local_map_pos.y[i]]){
				// Hay un obstáculo que ha desaparecido
				global_map[local_map_pos.x[i]][local_map_pos.y[i]] = 1;
			}else if(!local_map[local_map_pos.x[i]][local_map_pos.y[i]] && global_map[local_map_pos.x[i]][local_map_pos.y[i]]){
				// Hay un obstáculo que antes no estaba
				global_map[local_map_pos.x[i]][local_map_pos.y[i]] = 0;
			}
    } // Actualización del mapa global (insertar los obstáculos que he visto)

    Poss<int> new_obstacles_pos(vector<vector<float>> static_map, vector<vector<float>> local_map, Poss<int> visible_pos)
    {
        Poss<int> res;
        for(int i=0; i < visible_pos.x.size(); i++){
            if(static_map[visible_pos.x[i]][visible_pos.y[i]]!=local_map[visible_pos.x[i]][visible_pos.y[i]]){ // Es un obstáculo que no estaba en el mapa inicial
                res.x.push_back(visible_pos.x[i]); res.y.push_back(visible_pos.y[i]);
            }
        }
        return res;
    } // Posiciones de los obstáculos nuevos (que no estaban en static_map)

    void inflate(vector<vector<float>> &global_map, Poss<int> obst, int srange, vector<vector<float>> static_grid, Poss<int> visible_pos)
    {
        int sx = global_map.size(), sy = global_map[0].size();
        int xl, xu, yl, yu;
        // Fijar las posiciones que ve el agente (Para no inflar posiciones que no veo)
        for(int i=0; i<visible_pos.x.size(); i++) static_grid[visible_pos.x[i]][visible_pos.y[i]] = 3;
        for(int i=0; i<obst.x.size(); i++){
            xl = (obst.x[i]-srange) > 0 ? (obst.x[i]-srange) : 0;
            xu = (obst.x[i]+srange) < sx-1 ? (obst.x[i]+srange) : sx-1;
            yl = (obst.y[i]-srange) > 0 ? (obst.y[i]-srange) : 0;
            yu = (obst.y[i]+srange) < sy-1 ? (obst.y[i]+srange) : sy-1;

            for(int j=xl; j<=xu; j++){
                for(int k=yl; k<=yu; k++){
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    // PARA DISTINGUIR ENTRE EL INFLADO Y EL OBSTÁCULO REAL
                    //if(j==obst.x[i] && k==obst.y[i]) global_map[j][k] = 0;
                    //else global_map[j][k] = 0.5;
                    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    // TODO ES OBSTÁCULO
                    if(static_grid[j][k] == 3){ // Solo de las posiciones que ve el agente
                        global_map[j][k] = 0;
                    }
                }
            }
        }
    } // Inflado de los obastáculos

    void update_agent_information_no_infl(int x, int y, vector<vector<float>> world_grid, vector<vector<float>> static_map, vector<vector<float>> &global_map, vector<vector<float>> &local_map, Poss<int> &local_map_pos, Poss<int> &visible_pos, Poss<int> &new_obstacles, int vrange)
    {
        local_map_pos.clear();
        visible_pos.clear();
        new_obstacles.clear();
        // Actualizar los mapas
        local_map = obtain_local_map(x, y, world_grid, static_map, global_map, vrange, local_map_pos, visible_pos, new_obstacles);
        update_global_map(x, y, local_map, local_map_pos, global_map, vrange);
    } // Actualización de la información del entorno del agente (Sin el inflado de los obstáculos que ve el agente)

    void update_agents_information_no_infl(Poss<int> agents_poss, vector<vector<float>> world_grid, vector<vector<float>> static_map, vector<vector<float>> global_map, vector<vector<float>> local_map, Poss<int> &visible_poss, Poss<int> &visible_obstacles, float vrange)
    {
        Poss<int> _visible_poss, _visible_obstacles, local_map_poss;
        vector<vector<bool>> visible_poss_map, visible_obst_map;
        visible_poss_map.resize(static_map.size(),vector<bool>(static_map.size(),false));
        visible_obst_map = visible_poss_map;
        for(int i=0; i<agents_poss.x.size(); i++){
            navigation::update_agent_information_no_infl(agents_poss.x[i], agents_poss.y[i], world_grid, static_map, global_map, local_map, local_map_poss, _visible_poss, _visible_obstacles, vrange);
            for(int j=0; j<_visible_poss.x.size(); j++)
                if(!visible_poss_map[_visible_poss.x[j]][_visible_poss.y[j]]){
                    visible_poss_map[_visible_poss.x[j]][_visible_poss.y[j]] = true;
                    visible_poss.push(_visible_poss.x[j], _visible_poss.y[j]);
                }
            for(int j=0; j<_visible_obstacles.x.size(); j++)
                if(!visible_obst_map[_visible_obstacles.x[j]][_visible_obstacles.y[j]]){
                    visible_obst_map[_visible_obstacles.x[j]][_visible_obstacles.y[j]] = true;
                    visible_obstacles.push(_visible_obstacles.x[j], _visible_obstacles.y[j]);
                }
        }
    }

    void update_agent_information_infl(int x, int y, vector<vector<float>> world_grid, vector<vector<float>> static_map, vector<vector<float>> &global_map, vector<vector<float>> &local_map, Poss<int> &local_map_pos, Poss<int> &visible_pos, Poss<int> &new_obstacles, int vrange, int srange)
    {
    	update_agent_information_no_infl(x, y, world_grid, static_map, global_map, local_map, local_map_pos, visible_pos, new_obstacles, vrange);
        inflate(global_map, new_obstacles, srange, static_map, visible_pos);
    } // Actualización de la información del entorno del agente (Con el inflado de los obstáculos que ve el agente)

    Path path2goal(int xa, int ya, vector<vector<float>> navigation_map, int xgoal, int ygoal)
    {
        FMM gr(xa, ya, navigation_map);
        navigation_map = gr.expand2goal(xgoal, ygoal);
        Path res; res.gradient_descent_(navigation_map, xgoal, ygoal);
        return res;
    }

    Path path2goal(int xa, int ya, vector<vector<float>> navigation_map, int xgoal, int ygoal, vector<vector<float>> &gradient)
    {
        FMM gr(xa, ya, navigation_map);
        gradient = gr.expand2goal(xgoal, ygoal);
        Path res; res.gradient_descent_(gradient, xgoal, ygoal);
        return res;
    }

    Path path2goal_wr(int xa, int ya, vector<vector<float>> navigation_map, int xgoal, int ygoal, float radius)
    {
        return geometry::refinePath(path2goal(xa, ya, navigation_map, xgoal, ygoal), navigation_map, radius);
    }

    bool move_agent(int &x, int &y, vector<vector<float>> grad)
    {
        bool res = false;
        int x_, y_, xb = x, yb = y;
        for(int i=0; i<move_x.size(); i++){ // Analizar todos los posibles movimientos
            x_ = x + move_x[i]; y_ = y + move_y[i];
            if(x_>=0 && x_<grad.size() && y_>=0 && y_<grad[0].size()){
                if(grad[xb][yb] == INF){ // Si estaba en colisión
                    if(grad[x_][y_]<INF){ // y he dejado de estarlo
                        xb = x_; yb = y_;
                    }
                }else // No estaba en colisión
                if(grad[x_][y_]<INF && grad[xb][yb] > grad[x_][y_]){ // Sigo sin estarlo y me acerco más al objetivo
                    xb = x_; yb = y_;
                }
            }
        }
        // Se ha movido el agente
        if(x!=xb || y!=yb) res = true;
        // Actualizar la posición del agente
        x = xb; y = yb;
        return res;
    } // Movimiento simple, unicamente usando el gradiente

    bool move_agent(int &x, int &y, vector<vector<float>> grad, vector<vector<float>> grid)
    {
        // ESTA ES LA GÜENA
        bool res = false;
        int x_, y_, xb = x, yb = y;
        for(int i=0; i<move_x.size(); i++){ // Analizar todos los posibles movimientos
            x_ = x + move_x[i]; y_ = y + move_y[i];
            //cout<<"("<<x_<<", "<<y_<<"): "<<grad[x_][y_]<<endl;
            if(x_>=0 && x_<grad.size() && y_>=0 && y_<grad[0].size()){
                if(!grid[xb][yb]){ // Si estaba en colisión
                    if(grid[x_][y_]){ // y he dejado de estarlo
                        xb = x_; yb = y_;
                    }
                }else // No estaba en colisión
                if(grid[x_][y_] && grad[xb][yb] > grad[x_][y_]){ // Sigo sin estarlo y me acerco más al objetivo
                    xb = x_; yb = y_;
                    //cout<<"cc grad: "<<grad[xb][yb]<<endl;
                    //cout<<"MEJORO"<<endl;
                }
            }
        }
        // Se ha movido el agente
        if(x!=xb || y!=yb) res = true;
        // Actualizar la posición del agente
        x = xb; y = yb;
        return res;
    } // Movimiento simple con gradiente, pero considerando los obstáculos del "grid"

    Path move_agent_with_path(int &xa, int &ya, int xg, int yg, vector<vector<float>> goal_grad, vector<vector<float>> navigation_grid)
    {
        Path goal_path;
        if(goal_grad.size()){
            // Camino
            goal_path.gradient_descent_(goal_grad, xg, yg);
            // Mover el agente
            if(goal_path.tam > 1){
                if(navigation_grid[goal_path.x[1]][goal_path.y[1]]){
                    xa = goal_path.x[1]; ya = goal_path.y[1];
                }
            }
        }
        return goal_path;
    }

    bool move_agent_along_path(int &xa, int &ya, Path &goal_path, vector<vector<float>> navigation_grid)
    {
        if(goal_path.tam>1){
            if(navigation_grid[goal_path.x[1]][goal_path.y[1]]){
                xa = goal_path.x[1]; ya = goal_path.y[1];
                goal_path.pop(); // Eliminar el punto del camino (esto es solo para representar, el camino se recalcula en cada iteración)
                return true;
            }else{
                return false;
            }
        }else{
            return false;
        }
    }

    bool move_agent_cc(int &x, int &y, vector<vector<float>> grad, vector<vector<float>> grid)
    {
        // ESTA ES LA GÜENA
        bool res = false;
        int x_, y_, xb = x, yb = y;
        //for(int i=move_x.size()-1; i>=0; i--){ // Analizar todos los posibles movimientos
        for(int i=0; i<cc_move_x.size(); i++){ // Analizar todos los posibles movimientos
            x_ = x + cc_move_x[i]; y_ = y + cc_move_y[i];
            //cout<<"("<<x_<<", "<<y_<<"): "<<grad[x_][y_]<<endl;
            if(x_>=0 && x_<grad.size() && y_>=0 && y_<grad[0].size()){
                if(!grid[xb][yb]){ // Si estaba en colisión
                    if(grid[x_][y_]){ // y he dejado de estarlo
                        xb = x_; yb = y_;
                    }
                }else // No estaba en colisión
                if(grid[x_][y_] && grad[xb][yb] > grad[x_][y_]){ // Sigo sin estarlo y me acerco más al objetivo
                    xb = x_; yb = y_;
                    //cout<<"ccw grad: "<<grad[xb][yb]<<endl;
                    //cout<<"MEJORO"<<endl;
                }
            }
        }
        // Se ha movido el agente
        if(x!=xb || y!=yb) res = true;
        // Actualizar la posición del agente
        x = xb; y = yb;
        return res;
    } // Movimiento simple con gradiente, pero considerando los obstáculos del "grid"

    bool _move_agent(int &x, int &y, vector<vector<float>> grad, vector<vector<float>> grid)
    {
        bool res = false; // Se ha movido el agente?
        float min = numeric_limits<float>::max();
        bool col = !grid[x][y]; // ¿Estoy en colisión?
        int x_, y_, xb = x, yb = y;
        for(int i=0; i<move_x.size(); i++){ // Analizar todos los posibles movimientos
            x_ = x + move_x[i]; y_ = y + move_y[i];
            if(x_>=0 && x_<grad.size() && y_>=0 && y_<grad[0].size()){
                if(col){ // Si estaba en colisión
                    if(grid[x_][y_]){ // y he dejado de estarlo
                        xb = x_; yb = y_;
                        min = grad[xb][yb];
                        res = true;
                        col = false;
                    }
                }else // No estaba en colisión
                if(grid[x_][y_] && min > grad[x_][y_]){ // Sigo sin estarlo y me acerco más al objetivo
                    xb = x_; yb = y_;
                    min = grad[xb][yb];
                    res = true;
                }
            }
        }
        // Actualizar la posición del agente
        x = xb; y = yb;
        return res;
    } // ESTA ES LA MALA

    bool move_agent_(int &x, int &y, vector<vector<float>> grad, vector<vector<float>> grid)
    {
        bool res = false; // Se ha movido el agente?
        float min = grad[x][y];
        int x_, y_, xb = x, yb = y;
        for(int i=1; i<move_x.size(); i++){ // El primer moviemiento es quedarme quieto
            x_ = x + move_x[i]; y_ = y + move_y[i];
            if(x_>=0 && x_<grad.size() && y_>=0 && y_<grad[0].size()){
                if(grid[x_][y_] && min > grad[x_][y_]){
                    xb = x_; yb = y_;
                    min = grad[xb][yb];
                    res = true;
                }
            }
        }
        // Actualizar la posición del agente
        x = xb; y = yb;
        return res;
    }

    //*****************************************************************************************************
    // FUNCIONES PARA LA NAVEGACIÓN CON PRIORIDADES
    //*****************************************************************************************************
    void inflate_with_priorities(vector<vector<float>> &global_map, Poss<int> obst, vector<int> obstacles_priority, int srange, vector<vector<float>> static_grid, Poss<int> visible_pos)
    {
        int sx = global_map.size(), sy = global_map[0].size();
        int xl, xu, yl, yu;
        for(int i=0; i<visible_pos.x.size(); i++) static_grid[visible_pos.x[i]][visible_pos.y[i]] = -1;
        for(int i=0; i<obst.x.size(); i++){
            xl = (obst.x[i]-srange) > 0 ? (obst.x[i]-srange) : 0;
            xu = (obst.x[i]+srange) < sx-1 ? (obst.x[i]+srange) : sx-1;
            yl = (obst.y[i]-srange) > 0 ? (obst.y[i]-srange) : 0;
            yu = (obst.y[i]+srange) < sy-1 ? (obst.y[i]+srange) : sy-1;
            for(int j=xl; j<=xu; j++){
                for(int k=yl; k<=yu; k++){
                    // TODO ES OBSTÁCULO
                    if(static_grid[j][k] < 0){ // Posición que ve el agente
                        global_map[j][k] = obstacles_priority[i];
                    }
                }
            }
            // Fijar el propio obstáculo como obstáculo
            if(static_grid[obst.x[i]][obst.y[i]] < 0) global_map[obst.x[i]][obst.y[i]] = 0;
        }
    } // Inflado de los obastáculos con prioridades

    void update_agent_information_infl_priority(int x, int y, vector<vector<float>> world_grid, vector<vector<float>> static_map, vector<vector<float>> &global_map, vector<vector<float>> &local_map, Poss<int> &local_map_pos, Poss<int> &visible_pos, Poss<int> &new_obstacles, vector<int> obstacles_priority, int vrange, int srange)
    {
        update_agent_information_no_infl(x, y, world_grid, static_map, global_map, local_map, local_map_pos, visible_pos, new_obstacles, vrange);
        inflate_with_priorities(global_map, new_obstacles, obstacles_priority, srange, static_map, visible_pos);
    } // Actualización de la información del entorno del agente (Con el inflado de los obstáculos que ve el agente con sus respectivas prioridades)

    bool move_agent_with_priorities(int &x, int &y, int my_priority, vector<vector<float>> grad, vector<vector<float>> grid_with_priorities)
    {
        bool res = false;
        int x_, y_, xb = x, yb = y;
        for(int i=1; i<move_x.size(); i++){ // Analizar todos los posibles movimientos
            x_ = x + move_x[i]; y_ = y + move_y[i];
            if(x_>=0 && x_<grad.size() && y_>=0 && y_<grad[0].size()){
                if(!grid_with_priorities[xb][yb]){ // Si estaba en colisión
                    // y dejo de estarlo
                    if(grid_with_priorities[x_][y_]){ // No miro si invado a alguien
                        xb = x_; yb = y_;
                    }
                }else{ // No estaba en colisión
                    if(grid_with_priorities[xb][yb]==1){ // NO invadía a nadie
                        if(grid_with_priorities[x_][y_]==1 && grad[xb][yb] > grad[x_][y_]){
                            // Sigo sin invadir y además me acerco al objetivo
                            xb = x_; yb = y_;
                        }
                    }else{ // SI invadía a algien
                        if(grid_with_priorities[x_][y_] == 1){
                            // He dejado de invadir
                            xb = x_; yb = y_;
                        }else if(grid_with_priorities[x_][y_] > 1){
                            // Sigo invadiendo, pero no colisiono
                            if(grid_with_priorities[xb][yb] < my_priority){
                                // Invadía a alguien (alguien de mejor prioridad)
                                if(grid_with_priorities[x_][y_] > my_priority){
                                    // Prefiero invadir a alguien de peor prioridad que la mía (ya se apartará)
                                    xb = x_; yb = y_;
                                }
                            }else{
                                // Invadía a alguien (alguien de peor prioridad)
                                if(grid_with_priorities[xb][yb] < grid_with_priorities[x_][y_]){
                                    // Sigo invadiendo, pero a alguien de peor prioridad
                                    xb = x_; yb = y_;
                                }else if(grid_with_priorities[xb][yb] == grid_with_priorities[x_][y_]){
                                    if(grad[xb][yb] > grad[x_][y_]){
                                        xb = x_; yb = y_;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        // Se ha movido el agente?
        if(x!=xb || y!=yb) res = true;
        // Actualizar la posición del agente
        x = xb; y = yb;
        return res;
    } // Movimiento del agente considerando las prioridades de paso

    bool move_agent_with_priorities_(int &x, int &y, int my_priority, vector<vector<float>> grad, vector<vector<float>> grid_with_priorities)
    {
        bool res = false; // Se ha movido el agente?
        float min = numeric_limits<float>::max();
        //bool col = !grid_with_priorities[x][y]; // ¿Estoy en colisión?
        //bool col = (my_priority < grid_with_priorities[x][y]) || (grid_with_priorities[x][y]==0); // ¿Estoy en colisión?
        bool col = !grid_with_priorities[x][y]; // ¿Estoy en colisión?
        bool inv = grid_with_priorities[x][y] > 1; // ¿Estoy invadiendo a alguien?
        int x_, y_, xb = x, yb = y;
        for(int i=0; i<move_x.size(); i++){ // Analizar todos los posibles movimientos
            x_ = x + move_x[i]; y_ = y + move_y[i];
            if(x_>=0 && x_<grad.size() && y_>=0 && y_<grad[0].size()){
                if(col){ // Si estaba en colisión
                    if(grid_with_priorities[x_][y_] < my_priority || grid_with_priorities[x_][y_] == 1){ // y he dejado de estarlo y me he movido a un sitio donde tengo mayor prioridad
                        xb = x_; yb = y_;
                        min = grad[xb][yb];
                        res = true;
                        col = false;
                    }
                }else{ // No estaba en colisión
                    if((grid_with_priorities[x_][y_] < my_priority || grid_with_priorities[x_][y_] == 1) && min > grad[x_][y_]){ // Sigo sin estarlo y me acerco más al objetivo
                        xb = x_; yb = y_;
                        min = grad[xb][yb];
                        res = true;
                    }
                }
            }
        }
        // Actualizar la posición del agente
        /*
        if(grid_with_priorities[xb][yb] > grid_with_priorities[x][y]){
            //cout<<"Colisión en ("<<xb<<", "<<yb<<")"<<endl;
            cout<<my_priority<<endl;
            cout<<"Estaba en ("<<x<<", "<<y<<"): "<<grid_with_priorities[x][y]<<endl;
            cout<<"Movido a  ("<<xb<<", "<<yb<<"): "<<grid_with_priorities[xb][yb]<<endl;
            cin.get();
        }
        */
        x = xb; y = yb;
        return res;
    } // Movimiento del agente considerando las prioridades de paso


    // ============================================================================================================================================================

    // Funciones útiles para el movimiento de los agentes

    struct Agent_vars{
        int x, y;
        int vrange;
        Poss<int> visible_poss, visible_obstacles, local_map_poss;
        vector<vector<float>> global_map, local_map, static_map;
        vector<vector<float>> navigation_grid;
    };

    void agent_vars_update(Agent_vars &agent, vector<vector<float>> world_grid)
    {
        navigation::update_agent_information_no_infl(agent.x, agent.y, world_grid, agent.static_map, agent.global_map, agent.local_map, agent.local_map_poss, agent.visible_poss, agent.visible_obstacles, agent.vrange);
    }

    void simple_navigable_grid(Agent_vars &agent)
    {
        agent.navigation_grid = agent.static_map;
        for(int i=0; i<agent.visible_obstacles.x.size(); i++) agent.navigation_grid[agent.visible_obstacles.x[i]][agent.visible_obstacles.y[i]] = 0;  
    }

    void navigable_grid(Agent_vars &agent, vector<vector<float>> base_grid)
    {
        agent.navigation_grid = base_grid;
        for(int i=0; i<agent.visible_obstacles.x.size(); i++) agent.navigation_grid[agent.visible_obstacles.x[i]][agent.visible_obstacles.y[i]] = 0;
    }

    Path path2goal(Agent_vars agent, int xgoal, int ygoal)
    {
        FMM gr(agent.x, agent.y, agent.navigation_grid);
        vector<vector<float>> grad = gr.expand2goal(xgoal, ygoal);
        Path res; res.gradient_descent_(grad, xgoal, ygoal);
        return res;
    }

    void join_visibility(Agent_vars &agent, vector<Agent_vars> &agents)
    {
        // - agent: agente virtual (variable en la que se almacena la información conjunta)
        // - agents: variables de todos los agentes

        agent.visible_poss.clear();
        agent.visible_obstacles.clear();

        int sx = agent.static_map.size(), sy = agent.static_map[0].size();
        vector<vector<float>> visible_map(sx, vector<float>(sy, 0));
        vector<vector<float>> obstacles_map(sx, vector<float>(sy, 0));
        
        // Unificar las posiciones y obstáculos visibles
        for(int i=0; i<agents.size(); i++){
            // Posiciones visibles y mapa global (contiene lo que ven los agentes dentro de sus rangos de vissión respectivos)
            for(int j=0; j<agents[i].visible_poss.x.size(); j++){
                if(!visible_map[agents[i].visible_poss.x[j]][agents[i].visible_poss.y[j]]){
                    visible_map[agents[i].visible_poss.x[j]][agents[i].visible_poss.y[j]] = 1;
                    agent.visible_poss.push(agents[i].visible_poss.x[j], agents[i].visible_poss.y[j]);
                    agent.global_map[agents[i].visible_poss.x[j]][agents[i].visible_poss.y[j]] = agents[i].global_map[agents[i].visible_poss.x[j]][agents[i].visible_poss.y[j]];
                }
            }
            // Posiciones de los obstáculos
            for(int j=0; j<agents[i].visible_obstacles.x.size(); j++){
                if(!obstacles_map[agents[i].visible_obstacles.x[j]][agents[i].visible_obstacles.y[j]]){
                    obstacles_map[agents[i].visible_obstacles.x[j]][agents[i].visible_obstacles.y[j]] = 1;
                    agent.visible_obstacles.push(agents[i].visible_obstacles.x[j], agents[i].visible_obstacles.y[j]);
                }
            }
        }

        // Unificar el mapa de navegación
        agent.navigation_grid = agent.static_map;
        for(int i=0; i<agent.visible_obstacles.x.size(); i++) agent.navigation_grid[agent.visible_obstacles.x[i]][agent.visible_obstacles.y[i]] = 0;    
    }

}

#endif