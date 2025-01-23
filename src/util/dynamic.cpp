
#include <mada/util/dynamic.hpp>

void dynamic_fmm::random_movement(Poss<int> &robots, vector<vector<float>> &grid)
{
	int ind_move, x_, y_;
	for(int i=0; i<robots.x.size(); i++){
        ind_move = (int(rand())/INF)*move_x.size();
        x_ = robots.x[i] + move_x[ind_move]; y_ = robots.y[i] + move_y[ind_move];
        while(ind_move && !grid[x_][y_]){
        //while(!grid[x_][y_]){
            ind_move = (int(rand())/INF)*move_x.size();
            x_ = robots.x[i] + move_x[ind_move]; y_ = robots.y[i] + move_y[ind_move];
        }
        grid[robots.x[i]][robots.y[i]] = 1; // La posición que eja libre
        grid[x_][y_] = 0; // La posición que ocupa
        robots.x[i] = x_; robots.y[i] = y_; // Actualización de la posición
    }
}

void dynamic_fmm::random_movement(Poss<int> &robots, vector<vector<float>> &grid, Poss<int> others)
{
    int ind_move, x_, y_;
    // El grid con los otros obstáculos -> NO LOS MANEJO YO
    vector<vector<float>> grid_ = grid;
    for(int i=0; i<others.x.size(); i++) grid_[others.x[i]][others.y[i]] = 0;
    for(int i=0; i<robots.x.size(); i++){
        ind_move = (int(rand())/INF)*move_x.size();
        x_ = robots.x[i] + move_x[ind_move]; y_ = robots.y[i] + move_y[ind_move];
        while(ind_move && (!grid[x_][y_] || !grid_[x_][y_])){
            ind_move = (int(rand())/INF)*move_x.size();
            x_ = robots.x[i] + move_x[ind_move]; y_ = robots.y[i] + move_y[ind_move];
        }
        grid[robots.x[i]][robots.y[i]] = 1; // La posición que deja libre
        grid[x_][y_] = 0; // La posición que ocupa
        robots.x[i] = x_; robots.y[i] = y_; // Actualización de la posición
    }
}

void dynamic_fmm::swarm_random_movement(Poss<int> sources, vector<Poss<int>> &robots, vector<vector<float>> &grid, Poss<int> others)
{
    int ind_move, x_, y_, _x, _y;
    float min;
    // El grid con los otros obstáculos -> NO LOS MANEJO YO
    vector<vector<float>> grid_ = grid;
    vector<vector<float>> grad;
    for(int i=0; i<others.x.size(); i++) grid_[others.x[i]][others.y[i]] = 0;
    for(int i=0; i<sources.x.size(); i++){
        // El "tensor"
        FMM gr(sources.x[i], sources.y[i], grid_);
        grad = gr.pos_coverage(robots[i]);
        min = INF;
        for(int j=0; j<robots[i].x.size(); j++){
            min = INF;
            for(int k=0; k<move_x.size(); k++){
                x_ = robots[i].x[j] + move_x[k]; y_ = robots[i].y[j] + move_y[k];
                if(grid[x_][y_] && min > grad[x_][y_]){
                    ind_move = k;
                    min = grad[x_][y_];
                    _x = x_; _y = y_;
                }
            }
            grid[robots[i].x[j]][robots[i].y[j]] = 1; // La posición que deja libre
            grid[_x][_y] = 0; // La posición que ocupa
            robots[i].x[j] = _x; robots[i].y[j] = _y; // Actualización de la posición
        }
    }
}

void dynamic_fmm::generate_grads2move(Poss<int> &pos, vector<bool> &generate, vector<vector<float>> &grid, vector<vector<vector<float>>> &grads, vector<vector<float>> ref_grad, float deviation)
{
    Poss<int> goal; goal(1);

    int sx = grid.size(), sy = grid[0].size();
    int xl, xu, yl, yu;

    for(int i=0; i<pos.x.size(); i++){
        if(generate[i]){
            xl = (pos.x[i]-deviation) > 0 ? (pos.x[i]-deviation) : 0;
            xu = (pos.x[i]+deviation) < sx-1 ? (pos.x[i]+deviation) : sx-1;
            yl = (pos.y[i]-deviation) > 0 ? (pos.y[i]-deviation) : 0;
            yu = (pos.y[i]+deviation) < sy-1 ? (pos.y[i]+deviation) : sy-1;
            goal.x[0] = (int(rand())/INF)*(xu-xl)+xl;
            goal.y[0] = (int(rand())/INF)*(yu-yl)+yl;
            while((goal.x[0] == pos.x[i] && goal.y[0] == pos.y[i]) || ref_grad[goal.x[0]][goal.y[0]]==INF){
                goal.x[0] = (int(rand())/INF)*(xu-xl)+xl;
                goal.y[0] = (int(rand())/INF)*(yu-yl)+yl;
            }
            // Gradiente
            FMM gr(goal.x[0], goal.y[0], grid);
            grads[i] = gr.expand2goal(pos.x[i], pos.y[i]);
            generate[i] = false;
        }
    }
}

void dynamic_fmm::random_movement(Poss<int> &pos, vector<vector<vector<float>>> grads, vector<vector<float>> &grid, vector<bool> &generate, Poss<int> others)
{
    int _x, _y,__x,__y;
    float val_move;
    // El grid con los otros obstáculos -> NO LOS MANEJO YO
    vector<vector<float>> grid_ = grid;
    for(int i=0; i<others.x.size(); i++) grid_[others.x[i]][others.y[i]] = 0;
    for(int i=0; i<pos.x.size(); i++){
        val_move = INF;
        for(int k=0; k<move_x.size(); k++){
            _x = pos.x[i] + move_x[k]; _y = pos.y[i] + move_y[k];
            if(grid[_x][_y] && val_move > grads[i][_x][_y]){
                val_move = grads[i][_x][_y];
                __x = _x; __y = _y;
            }
        }
        grid[pos.x[i]][pos.y[i]] = 1; // La posición que deja libre
        grid[__x][__y] = 0; // La posición que ocupa
        pos.x[i] = __x; pos.y[i] = __y; // Actualización de la posición
        if(val_move == 0){
            generate[i] = true;
        }
    }
}

void dynamic_fmm::random_movement(Poss<int> &pos, vector<vector<float>> &grid, vector<int> &direction, vector<int> &time, Poss<int> others, int time_limit)
{
    int ind_move, x_, y_;
    // El grid con los otros obstáculos -> NO LOS MANEJO YO
    vector<vector<float>> grid_ = grid;
    for(int i=0; i<others.x.size(); i++) grid_[others.x[i]][others.y[i]] = 0;
    for(int i=0; i<pos.x.size(); i++){
        if(!time[i]){
            direction[i] = (int(rand())/INF)*move_x.size();
            time[i] = time_limit;
        }
        x_ = pos.x[i] + move_x[direction[i]]; y_ = pos.y[i] + move_y[direction[i]];
        if((!grid[x_][y_] || !grid_[x_][y_])){
            x_ = pos.x[i]; y_ = pos.y[i];
        }
        grid[pos.x[i]][pos.y[i]] = 1; // La posición que deja libre
        grid[x_][y_] = 0; // La posición que ocupa
        pos.x[i] = x_; pos.y[i] = y_; // Actualización de la posición
        time[i]--;
    }
}

void dynamic_fmm::random_movement(Poss<int> &pos, vector<vector<float>> &grid, vector<int> &direction, vector<int> &time, Poss<int> others, int time_limit, int seed)
{
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution(0,move_x.size()-1);

    int ind_move, x_, y_;
    // El grid con los otros obstáculos -> NO LOS MANEJO YO
    vector<vector<float>> grid_ = grid;
    for(int i=0; i<others.x.size(); i++) grid_[others.x[i]][others.y[i]] = 0;
    for(int i=0; i<pos.x.size(); i++){
        if(!time[i]){
            //direction[i] = (int(rand())/INF)*move_x.size();
            direction[i] = distribution(generator);
            time[i] = time_limit;
        }
        cout<<"Dirección: "<<direction[i]<<endl;
        x_ = pos.x[i] + move_x[direction[i]]; y_ = pos.y[i] + move_y[direction[i]];
        if((!grid[x_][y_] || !grid_[x_][y_])){
            x_ = pos.x[i]; y_ = pos.y[i];
        }
        grid[pos.x[i]][pos.y[i]] = 1; // La posición que deja libre
        grid[x_][y_] = 0; // La posición que ocupa
        pos.x[i] = x_; pos.y[i] = y_; // Actualización de la posición
        time[i]--;
    }
} // Lo mismo que la anterior pero con la semilla para generar el movimiento aleatorio

void dynamic_fmm::swarm_random_movement(Poss<int> sources, Poss<int> &robots, vector<vector<float>> &grid, Poss<int> others)
{
    int ind_move, x_, y_, _x, _y;
    int ng = robots.x.size()/sources.x.size()+1;
    float min;
    Poss<int> robots_;
    // El grid con los otros obstáculos -> NO LOS MANEJO YO
    vector<vector<float>> grid_ = grid;
    vector<vector<float>> grad;
    int s = 0, g = ng;
    for(int i=0; i<others.x.size(); i++) grid_[others.x[i]][others.y[i]] = 0;
    for(int i=0; i<sources.x.size(); i++){
        // El "tensor"
        FMM gr(sources.x[i], sources.y[i], grid_);
        grad = gr.compute_gradient_();
        min = INF;
        for(int j=s; j<g; j++){
            min = INF;
            for(int k=0; k<move_x.size(); k++){
                x_ = robots.x[j] + move_x[k]; y_ = robots.y[j] + move_y[k];
                if(grid[x_][y_] && min >= grad[x_][y_]){ // Al menos quedarse quieto
                    min = grad[x_][y_];
                    _x = x_; _y = y_;
                }
            }
            grid[robots.x[j]][robots.y[j]] = 1; // La posición que deja libre
            grid[_x][_y] = 0; // La posición que ocupa
            robots.x[j] = _x; robots.y[j] = _y; // Actualización de la posición
        }
        s = g + 1;
        g = (g + ng) > robots.x.size() ? robots.x.size() : (g + ng);
    }
}

void dynamic_fmm::include_obstacles(vector<vector<float>> &grid, Poss<int> robots, int xrobot, int yrobot, float vrange, int srange)
{
	for(int i=0; i<robots.x.size(); i++)
        if(sqrt(pow(robots.x[i]-xrobot,2)+pow(robots.y[i]-yrobot,2))<=vrange){
            // El robot solo
            if(!srange)
                grid[robots.x[i]][robots.y[i]] = 0;
            // El robot con el margen de seguridad
            else
                for(int j=robots.x[i]-srange; j<=robots.x[i]+srange; j++)
                    for(int k=robots.y[i]-srange; k<=robots.y[i]+srange; k++)
                        if(j>=0 && j<grid.size() && k>=0 && k<grid[0].size())
                            grid[j][k] = 0;
        }
}

void dynamic_fmm::move(int &xrobot, int &yrobot, int xgoal, int ygoal, vector<vector<float>> grid)
{
	Path robot_path;
	FMM gr_comp(xrobot, yrobot, grid);
    vector<vector<float>> robot_grad = gr_comp.compute_gradient_();
    if(robot_grad[xgoal][ygoal]<INF){
        robot_path.gradient_descent_(robot_grad, xgoal, ygoal);
        if(robot_path.tam>1){
            // Hay un camino posible al goal
            xrobot = robot_path.x[1]; yrobot = robot_path.y[1];
        }else{
            // No hay camino
            // Quedarse en el mismo sitio
        }
    }else{
        // El robot está encerrado
        // Huir al punto más alejado del espacio alcanzable
        // Punto más alejado
        float far_pos = 0;
        for(int i=0; i<grid.size(); i++){
            for(int j=0; j<grid[0].size(); j++){
                if(robot_grad[i][j]<INF && far_pos<robot_grad[i][j]){
                    far_pos = robot_grad[i][j];
                    xgoal = i; ygoal = j;
                }
            }
        }
        if(far_pos){
            robot_path.gradient_descent_(robot_grad, xgoal, ygoal);
            if(robot_path.tam>1){
                // Hay un camino posible al goal
                xrobot = robot_path.x[1]; yrobot = robot_path.y[1];
            }else{
                // No hay camino
                // Quedarse en el mismo sitio
            }
        }
    }
}

void dynamic_fmm::iterate(int xrobot, int yrobot, int xgoal, int ygoal, Poss<int> robots, vector<vector<float>> grid, float vrange, int srange, string results_folder)
{
	int cont = 0;
	Poss<int> pos2save; pos2save(1);
	vector<vector<float>> robots_grid, real_grid;
	int Nrobots = robots.x.size();
	//while((xrobot != xgoal && yrobot != ygoal) && cont<iter){
    while(xrobot != xgoal && yrobot != ygoal){
    	
    	// GUARDAR LAS POSICIONES DE LOS OBSTÁCULOS Y DEL AGENTE
        pos2save.x[0] = xrobot; pos2save.y[0] = yrobot;
        pos2save.save((results_folder+"agents_"+to_string(cont)+".txt").c_str()); // El robot
        robots.save((results_folder+"dyn_obst_"+to_string(cont)+".txt").c_str()); // Los robots

        // Fijar los obstáculos dinámicos
        robots_grid = grid;
        //robots_grid[xgoal][ygoal] = 0; // ???????????????????????
        for(int i=0; i<Nrobots; i++){
            robots_grid[robots.x[i]][robots.y[i]] = 0;
            //for(int j=0; j<move_x.size(); j++)
            //    robots_grid[robots.x[i]+move_x[j]][robots.y[i]+move_y[j]] = 0;  
        }

        // Mover aleatoriamente
        robots_grid = grid;
        random_movement(robots, robots_grid);

        // Meter los obstáculos en el grid
        real_grid = grid;
        include_obstacles(real_grid, robots, xrobot, yrobot, vrange, srange);

        // Mover
        move(xrobot, yrobot, xgoal, ygoal, real_grid);

        cont++;

    }
}

dynamic_fmm::navigable_area dynamic_fmm::generate_navigable_area(vector<vector<float>> grid, int nspaces, vector<int> shape)
{
    // Recibe:
    // - grid: grid vacío
    // - nsapces: cantidad de espacios a enerar
    // - shape: forma de los espacios
    //     - 0: tipo de forma
    //     - 1: límite inferior
    //     - 2: límite superior

    navigable_area res;
    
    if(shape[0] == 0){ // Espacios circulares
        // Generar las pociciones de los centros de los circulos
        Poss<int> centros = random_positions(grid, nspaces);
        // Generar los radios de los círculos
        vector<float> radios(nspaces);
        random_vector(radios, (float)shape[1], (float)shape[2]);
        // Obtener las posiciones
        res.grid.resize(grid.size(), vector<float>(grid[0].size(), 0));
        res.poss.resize(nspaces); res.contour_poss.resize(nspaces);
        for(int i=0; i<nspaces; i++){
            // Generar las posiciones del círculo para la representación en MatLab
            res.contour_poss[i] = geometry::circle_contour(centros.x[i], centros.y[i], radios[i], 0.05);
            // Generar las posiciones del círculo sobre el grid
            res.poss[i] = geometry::circle_poss_on_grid(centros.x[i], centros.y[i], radios[i], grid);
            // Delimitar el grid para la generación de los agentes
            for(int j=0; j<res.poss[i].x.size(); j++){
                res.grid[res.poss[i].x[j]][res.poss[i].y[j]] = 1;
            }
        }
    }else if(shape[0] == 1){ // Espacios cuadrados
        // Generar las posiciones de las posiciones de la esquina inferior izquierda de cada cuadrado
        Poss<int> posiciones = random_positions(grid, nspaces);
        // Generar las longitudes de los cuadrados
        vector<float> lados(nspaces);
        random_vector(lados, (float)shape[1], (float)shape[2]);
        // Obtener las posiciones
        res.grid.resize(grid.size(), vector<float>(grid[0].size(), 0));
        res.poss.resize(nspaces); res.contour_poss.resize(nspaces);
        vector<vector<float>> dim(nspaces, vector<float>(4,0));
        for(int i=0; i<nspaces; i++){
            dim[i][0] = posiciones.x[i]; dim[i][1] = posiciones.y[i]; // Posición
            dim[i][2] = lados[i]; dim[i][3] = lados[i]; // Dimensión
            // Almacenar los puntos del contorno del rectángulo
            res.contour_poss[i].x.push_back(dim[i][0]); res.contour_poss[i].y.push_back(dim[i][1]);
            res.contour_poss[i].x.push_back(dim[i][0]); res.contour_poss[i].y.push_back(dim[i][1]+lados[i]);
            res.contour_poss[i].x.push_back(dim[i][0]+lados[i]); res.contour_poss[i].y.push_back(dim[i][1]+lados[i]);
            res.contour_poss[i].x.push_back(dim[i][0]+lados[i]); res.contour_poss[i].y.push_back(dim[i][1]);
            // Generar las posiciones del cuadrado sobre el grid
            res.poss[i] = geometry::rectangle_poss_on_grid(dim[i], grid);
            // Delimitar el grid para la generación de los agentes
            for(int j=0; j<res.poss[i].x.size(); j++){
                res.grid[res.poss[i].x[j]][res.poss[i].y[j]] = 1;
            }
        }
    }else if(shape[0] == 2){ // Espacios rectangulares
        // Generar las posiciones de las posiciones de la esquina inferior izquierda de cada rectángulo
        Poss<int> posiciones = random_positions(grid, nspaces);
        // Generar las longitudes de los cuadrados
        vector<float> lados1(nspaces), lados2(nspaces);
        random_vector(lados1, (float)shape[1], (float)shape[2]);
        random_vector(lados2, (float)shape[1], (float)shape[2]);
        // Obtener las posiciones
        res.grid.resize(grid.size(), vector<float>(grid[0].size(), 0));
        res.poss.resize(nspaces); res.contour_poss.resize(nspaces);
        vector<vector<float>> dim(nspaces,vector<float>(4,0));
        for(int i=0; i<nspaces; i++){
            dim[i][0] = posiciones.x[i]; dim[i][1] = posiciones.y[i]; // Posición
            dim[i][2] = lados1[i]; dim[i][3] = lados2[i]; // Dimensión
            // Almacenar los puntos
            res.contour_poss[i].x.push_back(dim[i][0]); res.contour_poss[i].y.push_back(dim[i][1]);
            res.contour_poss[i].x.push_back(dim[i][0]); res.contour_poss[i].y.push_back(dim[i][1]+lados2[i]);
            res.contour_poss[i].x.push_back(dim[i][0]+lados1[i]); res.contour_poss[i].y.push_back(dim[i][1]+lados2[i]);
            res.contour_poss[i].x.push_back(dim[i][0]+lados1[i]); res.contour_poss[i].y.push_back(dim[i][1]);
            // Generar las posiciones del rectángulo sobre el grid
            res.poss[i] = geometry::rectangle_poss_on_grid(dim[i], grid);
            // Delimitar el grid para la generación de los agentes
            for(int j=0; j<res.poss[i].x.size(); j++){
                res.grid[res.poss[i].x[j]][res.poss[i].y[j]] = 1;
            }
        }
    }

    return res;
}

dynamic_fmm::navigable_area dynamic_fmm::generate_navigable_area(vector<vector<float>> grid, int nspaces, vector<int> shape, int points_in_area, int max_iter)
{
    //navigable_area res = dynamic_fmm::generate_navigable_area(grid, nspaces, shape);

    navigable_area res, aux;
    for(int i=0; i<nspaces; i++){
        // Generar un área
        aux = dynamic_fmm::generate_navigable_area(grid, 1, shape);
        int it = 0;
        while(it<max_iter){
            if(aux.poss[0].x.size() < points_in_area){
                aux = dynamic_fmm::generate_navigable_area(grid, 1, shape);
            }else{
                res.poss.push_back(aux.poss[0]);
                res.contour_poss.push_back(aux.contour_poss[0]);
                for(int i=0; i<aux.poss[0].x.size(); i++)
                    res.grid[aux.poss[0].x[i]][aux.poss[0].y[i]] = 1;
                break;
            }
            it++;
        }
    }

    return res;
}