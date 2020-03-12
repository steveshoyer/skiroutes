# -*- coding: utf-8 -*-
"""
Created on Thu Feb 13 02:35:59 2020

@author: steve shoyer
MET CS 664
Assignment 1

Version 3 - added tkinter user interface, more nodes and trails
Version 4 - added second agent, verify a-star route against Dijkstra
Version 5 - added ability to ignore trails that are closed
Version 6 - moved node and trail data to external json files
Version 7 - saved files to GitHub for easier updating

References:
    For the A* algorithm, Russell and Norvig, pages 84 and 93.  Also, much of
    the code for the graph search came from https://www.annytab.com/a-star-search-algorithm-in-python/.
    The method for determining the distance between two GPS coordinates came from
    https://janakiev.com/blog/gps-points-distance-python/.
    Dijkstra's algorithm is also from Russel and Norvig, page 84 and 110.
"""

import time
from geopy.distance import distance
import tkinter as tk
import json
import urllib.request
import os

"""
node data - this is a dictionary of nodes and their GPS coordinates

https://raw.githubusercontent.com/steveshoyer/skiroutes/master/nodes.json
"""

try:
    nodes_file = 'nodes.json'
    new_nodes = 'nodes.json.new'
    nodes_backup = 'nodes.json.save'
    url = "https://raw.githubusercontent.com/steveshoyer/skiroutes/master/nodes.json"
    print ("download start!")
    filename, headers = urllib.request.urlretrieve(url, filename=new_nodes)
    print ("download complete!")

    try:
        os.rename(nodes_file, nodes_backup)
    except WindowsError:
        os.remove(nodes_backup)
        os.rename(nodes_file, nodes_backup)
    try:
        os.rename(new_nodes, nodes_file)
    except WindowsError:
        os.remove(nodes_file)
        os.rename(new_nodes, nodes_file)
except :
    pass

node_data = json.load(open('nodes.json'))
    
"""
Trail data - dictionary of trails, with the format (start, end, length, rating, print name)

https://raw.githubusercontent.com/steveshoyer/skiroutes/master/nodes.json
"""

try:
    trail_file = 'trails.json'
    new_trails = 'trails.json.new'
    trail_backup = 'trails.json.save'
    url = "https://raw.githubusercontent.com/steveshoyer/skiroutes/master/trails.json"
    print ("download start!")
    filename, headers = urllib.request.urlretrieve(url, filename=new_trails)
    print ("download complete!")

    try:
        os.rename(trail_file, trail_backup)
    except WindowsError:
        os.remove(trail_backup)
        os.rename(trail_file, trail_backup)
    try:
        os.rename(new_trails, trail_file)
    except WindowsError:
        os.remove(trail_file)
        os.rename(new_trails, trail_file)
except :
    pass

trail_data = json.load(open("trails.json"))


# Closed trails list - a list of trails that should not be used for the graph
# Read closed trails from a file named closed_trails.txt
#closed_trails = ['Roadrunner','Lights Out']
# https://raw.githubusercontent.com/steveshoyer/skiroutes/master/closed_trails.txt
try:
    closed_trails = [line.rstrip() for line in open('closed_trails.txt')]
except:
    print("The 'closed_trails.txt' file was not found")
    closed_trails = []
   
    
    
    
LENGTH = 0  # index to path length in the graph list
RATINGS = ['lift','easy','intermediate','advanced','expert']  # trail ratings
NODELIST = list(node_data.keys())  # save the node names as a list

# set up agent classes for different search results
class Agent:
    def __init__(self, start_point, end_point, max_rating, ignore_closed_trails, algorithm='a-star'):
        self.algorithm = algorithm
        self.runtime = 0 # how long the search took
        self.nodes_visited = 0
        self.path = []
        self.start_point = start_point
        self.end_point = end_point
        self.max_rating = max_rating
        self.ignore_closed_trails = ignore_closed_trails
        
    def get_start_point(self):
        return self.start_point
    def get_end_point(self):
        return self.end_point
    def get_max_rating(self):
        return self.max_rating
    def get_ignore_closed_trails(self):
        return self.ignore_closed_trails
    def get_algorithm(self):
        return self.algorithm
    def get_runtime(self):
        return self.runtime
    def set_runtime(self, runtime):
        self.runtime = runtime
    def get_nodes_visited(self):
        return self.nodes_visited
    def set_nodes_visited(self, nodes_visited):
        self.nodes_visited = nodes_visited
    def get_path(self):
        return self.path
    def set_path(self, path):
        self.path = path
    def get_trails(self):
        return self.trails
    def set_trails(self, trails):
        self.trails = trails
    

# This class represent a node in either the frontier or explored queues
class Node:

    # Initialize the class
    def __init__(self, name:str, parent:str):
        self.name = name
        self.parent = parent
        self.g = 0 # Distance to start node
        self.h = 0 # Distance to goal node
        self.f = 0 # Total cost

    # Compare nodes
    def __eq__(self, other):
        return self.name == other.name

    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f

    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))


# Get children of a node
def get_children(graph, a, b=None):
    links = list(graph.setdefault(a, {}))
    if b is None:
        return links
    else:
        return links.get_children(graph, b)

"""
Graph search

This routine implements a uniform cost search.  If a dictionary with heuristic
values is passed to it, it will run an A* search.  If no heuristics are passed
(i.e., the dictionary values are all zero), it will perform Dijkstra's algorithm.
The routine keeps track of the number of nodes it tests to reach a solution as
a measure of performance, as the time it takes to execute either method is
small enough to not show significant differences.
"""        
def graph_search(graph, heuristics, start, end):
    
    # Create lists for frontier nodes and explored nodes
    frontier = []
    explored = []
    return_path = []

    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)

    # Add the start node
    frontier.append(start_node)
    vertex_count = 0 # keep track of how many nodes are added
    
    # Loop until the frontier list is empty
    while len(frontier) > 0:

        # Sort the frontier list to get the node with the lowest cost first
        frontier.sort()

        # Choose the lowest cost node in the frontier
        current_node = frontier.pop(0)
        vertex_count += 1

        # Add the current node to the explored list
        explored.append(current_node)
        
        # Check if we have reached the goal, return the path if we have
        if current_node == goal_node:
            return_path = []
            while current_node != start_node:
                return_path.append(current_node.name + ': ' + str(current_node.g))
                current_node = current_node.parent
            return_path.append(start_node.name + ': ' + str(start_node.g))
            # Return reversed path so it reads from first to last
            return(return_path[::-1], vertex_count)

        # Get children of the current node
        children = get_children(graph, current_node.name)

        # Loop through the children
        for key in children:

            # Create a child node
            child = Node(key, current_node)

            # Check if the child is in the explored list
            if (child in explored):
                continue

            # Calculate full path cost
            child.g = current_node.g + graph[current_node.name][child.name][LENGTH]
            child.h = heuristics.get(child.name)  # either the heuristic or 0, from input
            child.f = child.g + child.h

            # Check if child is in the frontier list and if it has a lower f value
            for node in frontier:
                if (child == node and child.f > node.f):
                    continue

            # Add the child to the frontier list
            frontier.append(child)

    # No path was found, so return an empty list
    return(return_path, vertex_count)


# Generate heuristic table (dictionary) - distance from the end point (or not)
def make_heuristic(end_point, algorithm):
    heuristic = {}
    for node, coord in node_data.items():
        if (algorithm == 'a-star'):
            heuristic[node] = distance(node_data[end_point], coord).m
        else:  # assume Dijkstra if not A-star 
            heuristic[node] = 0
    return(heuristic)


def find_route(agent:Agent):
# generate graph - start_point, end_point, max_rating, algorithm
#max_rating = 'expert'
    graph = {}
    if agent.get_ignore_closed_trails() == False:  # use the closed trails, so erase the list
        closed_trail_list = []
    else:  # ignore the closed trails, so copy the list
        closed_trail_list = closed_trails
    for a in trail_data:
    #    print(a, trail_data[a])
        start, end, length, rating, print_name = trail_data[a]
        if (RATINGS.index(rating) <= RATINGS.index(agent.get_max_rating())   # valid trail
            and (print_name not in closed_trail_list)):
            if start not in graph:  # new start node, so add this trail
                graph.setdefault(start, {})[end] = [length, print_name]
            else:
                routes_from = graph[start]  # get all routes
                if end not in routes_from:  # new end, so add
                    graph.setdefault(start, {})[end] = [length, print_name]
                elif routes_from[end][LENGTH] > length:  # existing trail is longer, so add
                    graph.setdefault(start, {})[end] = [length, print_name]

    heuristic = make_heuristic(agent.get_end_point(), agent.get_algorithm())
#    print(graph)

#    Use this to diagnose failed path verifications
#    if agent.get_algorithm() == 'a-star':
#        print(heuristic)

    start_time = time.time()
    path, v_count = graph_search(graph, heuristic, agent.get_start_point(), agent.get_end_point())
    end_time = time.time()
    
    # We found the path, so assemble the list of trails to follow that path.
    # A single trail can be made of multiple path segments.
    trails = []
    if (len(path) > 0):
        for i in range(0, len(path)-1):
            this_node = path[i].split(':')[0]
            next_node = path[i+1].split(':')[0]
            trail = graph[this_node][next_node][1]
            if len(trails) == 0 or trail != trails[-1]:
                    trails.append(trail)
#        print(trails)
    else:
#        print("Sorry, no path was found between" , start_point , "and" , end_point)
        trails = ["Sorry, no path was found between {0} and {1}".format(agent.get_start_point() ,
                                                                        agent.get_end_point())]
        
    agent.set_nodes_visited(v_count)
    agent.set_path(path)
    agent.set_trails(trails)
    agent.set_runtime((end_time - start_time) * 1000) # convert to ms
        
    return

"""
Main program - using tkinter as the GUI, the program gets user input for the starting and ending
points as well as the maximum trail difficulty and the graph search algorithm to use.

"""
def main():
    
    # return a formatted string with linked trails
    def format_trails(trails):
        output_string = ''
        if len(trails) > 0:  # something was returned, so start with the first element
            output_string = trails[0]
        for trail_name in trails[1::]:
            output_string = output_string + ' --> ' + trail_name 
        return(output_string)
        
        
  # "Find route" button action - get input values, process the search
    def search(*args):
        start_point = start_input.get()
        end_point = end_input.get()
        max_rating = max_rating_in.get()
        if ignore_closed_trails_in.get() == 1:
            ignore_closed_trails = True
        else:
            ignore_closed_trails = False
        
        # create two agents to run the search with different algorithms
        astar = Agent(start_point, end_point, max_rating, ignore_closed_trails, 'a-star')
        dijkstra = Agent(start_point, end_point, max_rating, ignore_closed_trails, 'dijkstra')
        
        # search the graph
        find_route(astar)
        find_route(dijkstra)
        
        # verification of route
        if astar.get_path() == dijkstra.get_path():  # found the same path
            compare_string = "Verification using Dijkstra\'s algorithm confirms the path; nodes visited: {0}, time: {1}".format(dijkstra.get_nodes_visited(), dijkstra.get_runtime())
            verification_output.configure({"background": 'light green'})
        else:
            compare_string = "Dijkstra\'s algorithm found a different path ({2}) on trails {3}; nodes visited: {0}, time: {1}".format(dijkstra.get_nodes_visited(), dijkstra.get_runtime(),dijkstra.get_path(),dijkstra.get_trails())
            verification_output.configure({"background": 'red'})
        print(compare_string)
 
       # set up and display the output
        node_output.delete(0.0, tk.END)
        node_output.insert(tk.END, astar.get_path())
        trail_output.delete(0.0, tk.END)
        trail_output.insert(tk.END, format_trails(astar.get_trails()))
        time_output.delete(0.0, tk.END)
        time_output.insert(tk.END, "Elapsed time for search: {0} ms".format(astar.get_runtime()))
        node_count_output.delete(0.0, tk.END)
        node_count_output.insert(tk.END, "Node count: {0}".format(astar.get_nodes_visited()))
        verification_output.delete(0.0, tk.END)
        verification_output.insert(tk.END, compare_string)
        
            
  # "Exit" button action - conclude the session
    def finish(*args):
        window.destroy()
    
    # create a tkinter window
    window = tk.Tk()
    window.title("Ski Trail Route Planner")
    window.geometry('500x600')
    
    # set up initial variable values    
    start_input = tk.StringVar(window)
    start_input.set(NODELIST[0])
    end_input = tk.StringVar(window)
    end_input.set(NODELIST[0])
    max_rating_in = tk.StringVar(window)
    max_rating_in.set(RATINGS[1])
    ignore_closed_trails_in = tk.IntVar(window)

    # create labels for the input fields
    tk.Label(window, relief=tk.RIDGE, text="Starting point:", bg="white", fg="black", font="none 12 bold").grid(row=0, column=0)
    tk.Label(window, relief=tk.RIDGE, text="Destination:", bg="white", fg="black", font="none 12 bold").grid(row=1, column=0)
    tk.Label(window, relief=tk.RIDGE, text="Maximum trail rating:", bg="white", fg="black", font="none 12 bold").grid(row=2, column=0)
    tk.Checkbutton(window, relief=tk.RIDGE, text="remove closed trails", variable=ignore_closed_trails_in).grid(row=3, column=1)    

    # create the drop-down menus
    tk.OptionMenu(window, start_input, *NODELIST).grid(row=0, column=1)
    tk.OptionMenu(window, end_input, *NODELIST).grid(row=1, column=1)
    tk.OptionMenu(window, max_rating_in, *RATINGS[1::]).grid(row=2, column=1)
    
    # create the output areas
    node_output = tk.Text(window, width=50, height=7, bg="white", wrap=tk.WORD)
    node_output.grid(row=6, column=0, columnspan=2)
    trail_output = tk.Text(window, width=50, height=4, bg="white", wrap=tk.WORD)
    trail_output.grid(row=7, column=0, columnspan=2)
    time_output = tk.Text(window, width=50, height=1, bg="white", wrap=tk.WORD)
    time_output.grid(row=8, column=0, columnspan=2)
    node_count_output = tk.Text(window, width=50, height=1, bg="white", wrap=tk.WORD)
    node_count_output.grid(row=9, column=0, columnspan=2)
    verification_output = tk.Text(window, width=50, height=3, bg="white", wrap=tk.WORD)
    verification_output.grid(row=10, column=0, columnspan=2)
    
    
# create buttons to search or exit
        
    tk.Button(window, text="Find route", width=12, command=search).grid(row=5, column=0, columnspan=2)
    tk.Button(window, text="Exit", width=12, command=finish).grid(row=12, column=0, columnspan=2)

# start the GUI
    window.mainloop()
       

# Tell python to run main method
if __name__ == "__main__": main()
