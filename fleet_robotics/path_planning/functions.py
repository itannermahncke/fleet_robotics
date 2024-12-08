import random
import math

class PathPlanningNode():
    def __init__(self):
        self.current_pos = []

    
    def path_planning(current_pos, goal_pos, lidar, step_size, threshold_to_goal, threshold_to_obstacle):

        while distance(current_pos, goal_pos)>threshold_to_goal:
            
            vector_to_goal = normalize(goal_pos - current_pos)
            next_position = current_pos + step_size * vector_to_goal

            
            if is_obstacle(next_position, obstacles):
                #calculate all surrounding grid distance to goal and select the one that has the least distance
                left_detour = calculate_detour(current_pos, direction="left", step_size)
                right_detour = calculate_detour(current_pos, direction="right", step_size)
                best_position = choose_best_option(left_detour, right_detour, goal_pos)
            else:
                best_position = next_position

            current_pos = best_position
            redraw_path(current_pos, goal_pos)





def check_conflict(my_position, *other_positions, threshold):
    """
    check if there are immediate conflicts, if yes, return True
    """
    for position in other_positions:
        if math.dist(my_position, position)< threshold:
            return True

def generate_numbers():
    """
    generate random integer and publish their number to other neatos
    """
    random_int = random.randint(1, 100)
    
    return random_int

def move_or_not(my_num, *numbers):
    """
    intake themselves and other neato's random intergers and compare them to figure out a priority cue
    return true or false indicating if they can move or not
    """
    for number in numbers:
        if my_num > number:
            return True
    




