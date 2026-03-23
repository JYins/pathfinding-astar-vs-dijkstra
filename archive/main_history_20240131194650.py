import time
from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq

def dijkstra(map, start, goal):

    pq = [(0, start)]
    heapq.heapify(pq)
    distances = {(start.get_x(), start.get_y()): 0}
    visited = set()
    nodes_expanded = 0

    while pq:
        # Get the node with the smallest distance
        current_distance, current_node = heapq.heappop(pq)
        nodes_expanded += 1

        current_coords = (current_node.get_x(), current_node.get_y())

        # If the goal is reached, return the distance and nodes expanded
        if current_coords == (goal.get_x(), goal.get_y()):
            return current_distance, nodes_expanded

        # Mark the node as visited
        visited.add(current_coords)

        # Iterate over neighbours
        for neighbour in map.successors(current_node):
            neighbour_coords = (neighbour.get_x(), neighbour.get_y())
            distance = current_distance + map.cost(neighbour.get_x() - current_node.get_x(), neighbour.get_y() - current_node.get_y())

            # If the neighbour node has not been visited or found a shorter path
            if neighbour_coords not in visited and (neighbour_coords not in distances or distance < distances[neighbour_coords]):
                distances[neighbour_coords] = distance
                heapq.heappush(pq, (distance, neighbour))

    # If goal is not reachable, return -1
    return -1, nodes_expanded


def octile_distance(current, goal):
    dx = abs(current.get_x() - goal.get_x())
    dy = abs(current.get_y() - goal.get_y())
    return 1.5 * min(dx, dy) + abs(dx - dy)

def a_star(map, start, goal):

    start_coords = (start.get_x(), start.get_y())
    goal_coords = (goal.get_x(), goal.get_y())
    pq = [(octile_distance(start, goal), start)]
    heapq.heapify(pq)
    distances = {start_coords: 0}
    visited = set()
    nodes_expanded = 0

    while pq:
        # Get the node with the smallest priority
        current_priority, current_node = heapq.heappop(pq)
        current_coords = (current_node.get_x(), current_node.get_y())
        nodes_expanded += 1

        # If the goal is reached, return the distance and nodes expanded
        if current_coords == goal_coords:
            return distances[current_coords], nodes_expanded

        # Mark the node as visited
        visited.add(current_coords)

        # Iterate over neighbours
        for neighbour in map.successors(current_node):
            neighbour_coords = (neighbour.get_x(), neighbour.get_y())
            distance = distances[current_coords] + map.cost(neighbour.get_x() - current_coords[0], neighbour.get_y() - current_coords[1])

            # If the neighbour node has not been visited or found a shorter path
            if neighbour_coords not in visited and (neighbour_coords not in distances or distance < distances[neighbour_coords]):
                distances[neighbour_coords] = distance
                priority = distance + octile_distance(neighbour, goal)
                heapq.heappush(pq, (priority, neighbour))

    # If goal is not reachable, return -1
    return -1, nodes_expanded



def main():
    """
    Function for testing your A* and Dijkstra's implementation. 
    Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, a in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances and generate plots: main.py --plots")
            exit()
        elif o in ("--plots"):
            plots = True

    test_instances = "test-instances/testinstances.txt"
    
    # Dijkstra's algorithm and A* should receive the following map object as input
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []  
    nodes_expanded_astar = []

    time_dijkstra = []  
    time_astar = []

    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):    
        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        cost, expanded_diskstra = dijkstra(gridded_map, start, goal) # replace None, None with the call to your Dijkstra's implementation
        time_end = time.time()
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_dijkstra.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()    
         #print(cost,expanded_diskstra)
        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        cost, expanded_astar = a_star(gridded_map, start, goal) # replace None, None with the call to your A* implementation
        time_end = time.time()

        nodes_expanded_astar.append(expanded_astar)
        time_astar.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()
       #print(cost,expanded_astar)
    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
        plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")

if __name__ == "__main__":
    main()