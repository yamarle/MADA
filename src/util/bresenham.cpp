#include <mada/util/bresenham.hpp>

int bresenham::number_of_walls(vector<vector<float> > grid, int x1, int y1, int x2, int y2)
{
	int n=0; // cantidad de muros

    int delta_x(x2 - x1);
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = abs(delta_x) << 1;

    int delta_y(y2 - y1);
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = abs(delta_y) << 1;

    if(!grid[x1][y1]) n++;

    if (delta_x >= delta_y){
        int error(delta_y - (delta_x >> 1));
        while (x1 != x2){
            if ((error >= 0) && (error || (ix > 0))){
                error -= delta_x;
                y1 += iy;
            }
            error += delta_y;
            x1 += ix;

            if(!grid[x1][y1]) n++;
        }
    }else{
        int error(delta_x - (delta_y >> 1));
        while (y1 != y2){
            if ((error >= 0) && (error || (iy > 0))){
                error -= delta_y;
                x1 += ix;
            }
            error += delta_x;
            y1 += iy;
            if(!grid[x1][y1]) n++;
        }
    }
    return n;
}

int bresenham::ray_of_walls(vector<vector<float> > grid, int x1, int y1, int x2, int y2, vector<int> &x, vector<int> &y, vector<int> &n)
{
    int nw=0;
    if(x.size()){
        x.clear(); y.clear(); n.clear();
    }

    int delta_x(x2 - x1);
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = abs(delta_x) << 1;

    int delta_y(y2 - y1);
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = abs(delta_y) << 1;

    if(!grid[x1][y1]) nw++;

    x.push_back(x1);
    y.push_back(y1);
    n.push_back(nw);

    if (delta_x >= delta_y){
        int error(delta_y - (delta_x >> 1));
        while (x1 != x2){
            if ((error >= 0) && (error || (ix > 0))){
                error -= delta_x;
                y1 += iy;
            }
            error += delta_y;
            x1 += ix;

            if(!grid[x1][y1]) nw++;

            x.push_back(x1);
            y.push_back(y1);
            n.push_back(nw);
        }
    }else{
        int error(delta_x - (delta_y >> 1));
        while (y1 != y2){
            if ((error >= 0) && (error || (iy > 0))){
                error -= delta_y;
                x1 += ix;
            }
            error += delta_x;
            y1 += iy;

            if(!grid[x1][y1]) nw++;

            x.push_back(x1);
            y.push_back(y1);
            n.push_back(nw);
        }
    }
    return nw;
}


bool bresenham::exists_wall(vector<vector<float> > grid, int x1, int y1, int x2, int y2, int &x, int &y)
{
    int delta_x(x2 - x1);
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = abs(delta_x) << 1;

    int delta_y(y2 - y1);
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = abs(delta_y) << 1;

    if(!grid[x1][y1]){
		x=x1;y=y1;
		return true;
	}

    if (delta_x >= delta_y){
        int error(delta_y - (delta_x >> 1));
        while (x1 != x2){
            if ((error >= 0) && (error || (ix > 0))){
                error -= delta_x;
                y1 += iy;
            }
            error += delta_y;
            x1 += ix;

            if(!grid[x1][y1]){
				x=x1;y=y1;
				return true;
			}
        }
    }else{
        int error(delta_x - (delta_y >> 1));
        while (y1 != y2){
            if ((error >= 0) && (error || (iy > 0))){
                error -= delta_y;
                x1 += ix;
            }
            error += delta_x;
            y1 += iy;
            if(!grid[x1][y1]){
				x=x1;y=y1;
				return true;
			}
        }
    }
	return false;
}

bool bresenham::exists_wall(vector<vector<float> > grid, int x1, int y1, int x2, int y2)
{
    int delta_x(x2 - x1);
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = abs(delta_x) << 1;

    int delta_y(y2 - y1);
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = abs(delta_y) << 1;

    if(!grid[x1][y1]){
		return true;
	}

    if (delta_x >= delta_y){
        int error(delta_y - (delta_x >> 1));
        while (x1 != x2){
            if ((error >= 0) && (error || (ix > 0))){
                error -= delta_x;
                y1 += iy;
            }
            error += delta_y;
            x1 += ix;

            if(!grid[x1][y1]){
				return true;
			}
        }
    }else{
        int error(delta_x - (delta_y >> 1));
        while (y1 != y2){
            if ((error >= 0) && (error || (iy > 0))){
                error -= delta_y;
                x1 += ix;
            }
            error += delta_x;
            y1 += iy;
            if(!grid[x1][y1]){
				return true;
			}
        }
    }
	return false;
}

Poss<int> bresenham::points(int x1, int y1, int x2, int y2)
{
    Poss<int> res;

    int delta_x(x2 - x1);
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = abs(delta_x) << 1;

    int delta_y(y2 - y1);
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = abs(delta_y) << 1;
    
    res.x.push_back(x1);
    res.y.push_back(y1);

    if (delta_x >= delta_y){
        int error(delta_y - (delta_x >> 1));
        while (x1 != x2){
            if ((error >= 0) && (error || (ix > 0))){
                error -= delta_x;
                y1 += iy;
            }
            error += delta_y;
            x1 += ix;
            
            res.x.push_back(x1);
            res.y.push_back(y1);
        }
    }else{
        int error(delta_x - (delta_y >> 1));
        while (y1 != y2){
            if ((error >= 0) && (error || (iy > 0))){
                error -= delta_y;
                x1 += ix;
            }
            error += delta_x;
            y1 += iy;
            
            res.x.push_back(x1);
            res.y.push_back(y1);

        }
    }

    return res;
}

Poss<int> bresenham::points_wall_check(int x1, int y1, int x2, int y2, vector<vector<float> > grid)
{
    Poss<int> res;

    int delta_x(x2 - x1);
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = abs(delta_x) << 1;

    int delta_y(y2 - y1);
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = abs(delta_y) << 1;
    
    res.x.push_back(x1);
    res.y.push_back(y1);

    if(!grid[x1][y1]){
        return res;
    }

    if (delta_x >= delta_y){
        int error(delta_y - (delta_x >> 1));
        while (x1 != x2){
            if ((error >= 0) && (error || (ix > 0))){
                error -= delta_x;
                y1 += iy;
            }
            error += delta_y;
            x1 += ix;

            if(!grid[x1][y1]){
                return res;
            }
            
            res.x.push_back(x1);
            res.y.push_back(y1);
        }
    }else{
        int error(delta_x - (delta_y >> 1));
        while (y1 != y2){
            if ((error >= 0) && (error || (iy > 0))){
                error -= delta_y;
                x1 += ix;
            }
            error += delta_x;
            y1 += iy;

            if(!grid[x1][y1]){
                return res;
            }
            
            res.x.push_back(x1);
            res.y.push_back(y1);

        }
    }

    return res;
}

Poss<int> bresenham::all_rays_poss(int x, int y, Poss<int> poss)
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

Poss<int> bresenham::all_rays_poss(int x, int y, Poss<int> poss, vector<vector<int>> &seen_map)
{
    Poss<int> res;
    for(int i=0; i<poss.x.size(); i++){
        Poss<int> p = bresenham::points(x, y, poss.x[i], poss.y[i]);
        for(int j=0; j<p.x.size(); j++){
            res.push(p.x[j], p.y[j]);
            seen_map[p.x[j]][p.y[j]]++;
        }
    }
    return res;
}