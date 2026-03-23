import time
from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq

def dijkstra(map_path, start_state, goal_state):
    open_set = []  
    closed_set = {start_state.state_hash(): start_state}
    start_state.set_cost(start_state.get_g())
    heapq.heappush(open_set, start_state)
    nodes_expanded = 0  

    # Main loop for Dijkstra's algorithm.
    while open_set:
        current_state = heapq.heappop(open_set)  # Get the state with the lowest cost.
        nodes_expanded += 1

        if current_state == goal_state:  # Check if the goal state is reached.
            return current_state.get_g(), nodes_expanded, closed_set 

        # Process all neighbors of the current state.
        neighbors = [(neighbor, neighbor.state_hash()) for neighbor in map_path.successors(current_state)]
        for neighbor, neighbor_hash in neighbors:
            neighbor.set_cost(neighbor.get_g())
            # Add or update neighbor in the open and closed sets.
            if neighbor_hash not in closed_set or neighbor.get_g() < closed_set[neighbor_hash].get_g():
                closed_set[neighbor_hash] = neighbor
                heapq.heappush(open_set, neighbor)

    return -1, nodes_expanded, closed_set 

# Function to calculate octile distance, a heuristic for A*.
def octile_distance(current, goal):
    dx = abs(current.get_x() - goal.get_x())
    dy = abs(current.get_y() - goal.get_y())
    return 1.5 * min(dx, dy) + abs(dx - dy)

# Function to handle each neighbor node during the search.
def handle_neighbor(next_node, goal_node, open_heap, closed_set):
    node_hash = next_node.state_hash()
    next_g = next_node.get_g()
    next_f = next_g + octile_distance(next_node, goal_node)
    next_node.set_cost(next_f)

    if node_hash not in closed_set:
        heapq.heappush(open_heap, next_node)
        closed_set[node_hash] = next_node
    elif next_node.get_g() < closed_set[node_hash].get_g():
        heapq.heappush(open_heap, next_node)
        heapq.heapify(open_heap)
        closed_set[node_hash] = next_node

def a_star(map_graph, start_node, goal_node):
    open_heap = []
    start_node.set_g(0)
    start_node.set_cost(start_node.get_g())
    closed_set = {start_node.state_hash(): start_node}
    heapq.heappush(open_heap, start_node)
    nodes_expanded = 0

    while open_heap:
        current_node = heapq.heappop(open_heap)
        nodes_expanded += 1

        if current_node == goal_node:
            return current_node.get_g(), nodes_expanded, closed_set

        neighbors = map_graph.successors(current_node)
        for next_node in neighbors:
            handle_neighbor(next_node, goal_node, open_heap, closed_set)

    return -1, nodes_expanded, closed_set

def main():
    """
    Function for testing your A* and Dijkstra's implementation. 
    Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = True
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
        cost, expanded_diskstra, closlist = dijkstra(gridded_map, start, goal) # replace None, None with the call to your Dijkstra's implementation
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
        cost, expanded_astar, clist = a_star(gridded_map, start, goal) # replace None, None with the call to your A* implementation
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