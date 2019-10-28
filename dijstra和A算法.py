import collections
#from implementation import *

class SimpleGraph:
	def __init__(self):
		self.edges = {}
		self.heuristic = {}
	
	def neighbors(self, id):
		return self.edges[id]
		
example_graph = SimpleGraph()
example_graph.edges = {
	'Arad': [['Zerind',75],['Sibiu',140],['Timisoara',118]],
	'Zerind': [['Oradea',71]],
	'Oradea': [['Sibiu',151]],
	'Sibiu': [['Fagaras',99], ['Riminicu Vilcea',80]],
	'Timisoara': [['Lugoj',111]],
	'Lugoj': [['Mehadia',70]],
	'Mehadia': [['Dobreta',75]],
	'Dobreta': [['Craiova',120]],
	'Riminicu Vilcea': [['Craiova',146],['Pitesti',97]],
	'Craiova': [['Pitesti',138]],
	'Pitesti': [['Bucharest',101]],
	'Fagaras': [['Bucharest',211]],
	'Bucharest': [['Giurgiu',90],['Urziceni',85]],
	'Urziceni': [['Vaslui',142],['Hirsova',86]],
	'Hirsova': [['Eforie',86]],
	'Vaslui': [['lasi',92]],
	'lasi': [['Neamt',87]]
}
example_graph.heuristic = {
	'Bucharest': [['Arad',366],['Bucharest',0],['Craiova',160],['Dobreta',242],['Eforie',161],['Fagaras',178],['Giurgiu',77],['Hirsova',151],['lasi',226],['Lugoj',244],['Mehadia',241],['Neamt',234],['Oradea',380],['Pitesti',98],['Rimnicu Vilcea',193],['Sibiu',253],['Timisoara',329],['Urziceni',80],['Vaslui',199],['Zerind',374]]
}

import heapq

class PriorityQueue:
	def __init__(self):
		self.elements = []
	
	def empty(self):
		return len(self.elements) == 0
	
	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))
	
	def get(self):
		return heapq.heappop(self.elements)[1]
		
def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + next[1]
            if next[0] not in cost_so_far or new_cost < cost_so_far[next[0]]:
                cost_so_far[next[0]] = new_cost
                priority = new_cost
                frontier.put(next[0], priority)
                came_from[next[0]] = current
    
    return came_from, cost_so_far


def a_star_search(graph, start, goal):
	frontier = PriorityQueue()
	frontier.put(start, 0)
	came_from = {}
	cost_so_far = {}
	came_from[start] = None
	cost_so_far[start] = 0
	
	while not frontier.empty():
		current = frontier.get()
		#print(current)
		if current == goal:
			break
		
		for next in graph.neighbors(current):
			new_cost = cost_so_far[current] + next[1]
			if next[0] not in cost_so_far or new_cost < cost_so_far[next[0]]:
				cost_so_far[next[0]] = new_cost
				for heu in graph.heuristic[goal]:
					if heu[0] == next [0]:
						heu_value = heu[1]
						break
				priority = new_cost + heu_value
				frontier.put(next[0], priority)
				#print(next[0], priority)
				came_from[next[0]] = current
	
	return came_from, cost_so_far

start, goal = 'Arad', 'Bucharest'
ans = 'Bucharest'
came_from_d, cost_so_far_d = dijkstra_search(example_graph, start, goal)

print('The result of dijkstra:')
while came_from_d[ans]:
	print(ans)
	print('|')
	ans = came_from_d[ans]
print(ans)

print('The cost is:',cost_so_far_d['Bucharest'])

came_from_a, cost_so_far_a = a_star_search(example_graph, start, goal)
ans = 'Bucharest'
print('The result of A*:')
while came_from_a[ans]:
	print(ans)
	print('|')
	ans = came_from_a[ans]
print(ans)

print('The cost is:',cost_so_far_a['Bucharest'])