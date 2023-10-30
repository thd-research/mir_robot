
from dataclasses import dataclass

from math_package.spline_math import *



@dataclass
class Route_param():
    on_route_ind: int
    on_route_curr_s_star: float


class PathMovement:
    def __init__(self, x_points, y_points):
        z_points = x_points*0
        waypoints =  np.vstack((x_points,y_points, z_points)).T
        self.__curr_param: Route_param  = Route_param(0, 0.0)
        self.__spline = M_spline_from_set(waypoints)
        self.dist_to_target: float = 1.5
        self.__delta: float  = 0
        self.__curr_point = np.array([0.0, 0.0, 0.0])
        self.__delta_to_target: float = 0.0
        
    def get_target_point(self):
        bias_param = self.__bias_Route_param(self.__curr_param, self.dist_to_target)
        target_point = self.__get_point(bias_param)
        self.__delta_to_target = np.linalg.norm(target_point[:2] - self.__curr_point[:2])
        return target_point
    

    def get_curr_point(self):
        point = self.__get_point(self.__curr_param)
        return point
    
    def __get_point(self, params: Route_param):
        cfg = self.__spline[params.on_route_ind]
        curr_projection = spline_find_point_state2d(cfg, params.on_route_curr_s_star)
        return curr_projection
    
    def close_to_goal(self) -> bool:
        return self.__curr_param.on_route_ind == len(self.__spline) - 1 and self.__curr_param.on_route_curr_s_star > 0.9 
    
    @property
    def delta(self):
        return self.__delta
        
    @property
    def delta2(self):
        return self.__delta_to_target
    

    @property
    def route_params(self):
        return self.__curr_param.on_route_ind, self.__curr_param.on_route_curr_s_star
    
    def estimate_curr_params(self, pos2d) -> None:
        pos = np.array([pos2d[0], pos2d[1], 0])
        curr_ind: int = self.__curr_param.on_route_ind - 1
        if(curr_ind <0):
            curr_ind = 0

        while(curr_ind - self.__curr_param.on_route_ind < 3):
            cfg = self.__spline[curr_ind]
            if(spline_content_point(pos, cfg)):
                s_star = spline_param_of_point_projection(pos, cfg)
                if(s_star < 0):
                    raise ValueError("Could not find projection")
                
                self.__curr_param.on_route_ind = curr_ind
                self.__curr_param.on_route_curr_s_star = s_star
                # print("estimate",  self.__curr_param)
                self.__curr_point = self.__get_point(self.__curr_param)[:2]
                self.__delta = np.linalg.norm(self.__curr_point[:2] - pos2d[:2])
                return 
            
            curr_ind += 1
            if(curr_ind > len(self.__spline)-1):
                break
        
        raise ValueError("Could not find cr params")
    
    def __bias_Route_param(self, start_pos: Route_param,  bias_dist: float):
        found_pos = Route_param(start_pos.on_route_ind, start_pos.on_route_curr_s_star)
        total_length: float = bias_dist
        while(total_length > 0):
            curr_s_star: float = found_pos.on_route_curr_s_star; 
            curr_spline_length: float = spline_length(self.__spline[found_pos.on_route_ind])
            dist_to_spline_end = np.linalg.norm(curr_spline_length*(1-curr_s_star))
            if(dist_to_spline_end > total_length):
                found_pos.on_route_curr_s_star = (curr_spline_length*curr_s_star + total_length) /curr_spline_length
                break
            else:
                total_length -= dist_to_spline_end
                found_pos.on_route_ind+=1
                found_pos.on_route_curr_s_star=0.0
            l: float = len(self.__spline)-1
            if(found_pos.on_route_ind > l):
                found_pos.on_route_ind  = l
                found_pos.on_route_curr_s_star  = 1.0
                break

            if(found_pos.on_route_ind == l and found_pos.on_route_curr_s_star >= 1.0):
                break

        return found_pos