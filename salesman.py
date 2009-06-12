# http://en.wikipedia.org/wiki/Ant_colony_optimization
 
# Travelling salesman problem
 
# distance matrix between cities
# matrix is symmetric and diagonal is zeros

import random
import itertools
from operator import mul, add

class Permutation:
    def fac(self, x): 
        return (1 if x==0 else x * self.fac(x-1))

    def permutations(self, plist):
        counter=0
        lenplist = len(plist)
        limit = self.fac(lenplist) / 2
        if len(plist) <=1:
            yield plist
        else:
            for perm in self.permutations(plist[1:]):
                for i in xrange(len(perm)+1):
                    if len(perm[:i] + plist[0:1] + perm[i:]) == lenplist:
                            if counter == limit:
                                 raise StopIteration
                            else:
                                 counter = counter + 1
                    yield perm[:i] + plist[0:1] + perm[i:]



class Salesman:

    def __init__(self, matrix, cities):
        self.matrix = matrix
        self.cities = cities

    def all_paths(self, cities):
        return self.all_paths_christoph(cities)
        #return self.all_paths_itertools(cities)
        
    def all_paths_christoph(self, cities):
        return Permutation().permutations(cities)
        
    def all_paths_itertools(self, cities):
        """
        Returns all permutations of list
        it is enough to return ceil(len(permutations)) elements
        """
        gen = itertools.permutations(cities)
        
        return gen
        

    def all_paths_old(self, str):
        """
        Returns all permutations of list or tuple.
        Actually only half of those are interesting
        """
        if len(str) <=1:
            yield str
        else:
            for perm in self.all_paths(str[1:]):
                for i in xrange(len(perm)+1):
                    yield perm[:i] + str[0:1] + perm[i:]

    def path_edges(self, path):
        return zip(path, path[1::])

    def all_edges(self):
        # TODO make this a generator
        edges = itertools.combinations(self.cities, 2)

        reversed_cities = self.cities[:]
        reversed_cities.reverse()
        
        reverse_edges = itertools.combinations(reversed_cities, 2)
        
        all_edges = list(edges)
        all_edges.extend( list(reverse_edges) )
        return all_edges

    def edge_list_to_path(self, edge_list):
        path = []
        for edge in edge_list:
            path.append(edge[0])
            
        path.append(edge_list[-1][1])
        return path

    def path_distance(self, path):
        sum = 0;
        edges = self.path_edges(path)
        for edge in edges:
            sum += self.matrix[ edge[0] ][ edge[1] ]
        return sum    


    def display_path(self, path):
        print "path:", path, "distance", self.path_distance(path)

    def find_min_distance(self, paths):
        raise Exception("implement is subclass")

class BruteForceSalesman(Salesman):
    def find_min_distance(self):
        all_paths = self.all_paths(self.cities)
        min_distance= None
        min_paths = []
        path_count = 0
        for path in all_paths:
            path_count += 1
            dist = self.path_distance(path)
            if min_distance == None or dist < min_distance:
                min_paths = [path]
                min_distance = dist
            elif min_distance == dist:
                min_paths.append(path)
        print "Path count: " + str(path_count) + ", cities " + str(len(min_paths[0]))
        return min_paths

class AntHillSalesman(Salesman):
    def __init__(self, matrix, cities, ant_count, iterations = 100, pheromone_control=0.1, distance_control=0.1, initial_pheromone=0.1):
        self.matrix = matrix
        self.cities = cities
        
        self.iterations = iterations
        self.ant_count = ant_count
        self.pheromone_control = pheromone_control
        self.distance_control = distance_control
        self.initial_pheromone = initial_pheromone
        
        self.evaporation = 0.01
        
        self.pheromone_map = {}
        self.initialize_pheromone_map()
        self.decay_pheromones()
        
    def initialize_pheromone_map(self):
        edges = self.all_edges()
        for edge in edges:
            self.pheromone_map[edge] = self.initial_pheromone
        
    def find_min_distance(self):
        available_edges = self.all_edges()

        best_paths = []
        for iteration in xrange(self.iterations):
            print "Iteration:" , iteration
            paths = []
            
            best_distance = None
            best_paths = []
            for ant_count in xrange(self.ant_count):
                path = self.ant_run()
                paths.append(path)
                distance = self.path_distance(path)
                
                if best_distance == None or distance < best_distance:
                    best_paths = [path]
                    best_distance = distance
                    
                elif distance == best_distance:
                    best_paths.append(path)
   
            print "best distance:", best_distance 
            self.decay_pheromones()
            for path in paths:
                pass
                self.leave_pheromones(path)
        
        print "Best paths for last iteration"
        for path in best_paths:
            print path, self.path_distance(path)
        return []
        
    def ant_run(self):
        available_cities = cities[:]
        selected_edges = []
        # TODO how to select first city?
        current_city = random.sample(available_cities, 1)[0]
        available_cities.remove(current_city)
        
        while available_cities:
            available_edges = [ (current_city, city) for city in available_cities] 
            selected_edge = self.select_edge(available_edges)
            current_city = selected_edge[1]
            available_cities.remove(current_city)
            available_edges.remove(selected_edge)
            selected_edges.append(selected_edge)

        path = self.edge_list_to_path(selected_edges)
        return path
        
    def select_edge(self, available_edges):
        # compute probs for every edge
        all_edges_value = self.compute_all_edges_value(available_edges)
        
        edge_probs = {}
        for edge in available_edges:
            edge_value= self.compute_edge_value(edge)

            prob = edge_value / all_edges_value
            edge_probs[edge] = prob

        # sort edges based on prob (highest first)
        sorted_edges = map(lambda x: x[1], 
                        sorted(
                        map(lambda edge: (1.0 - edge_probs[edge], edge), available_edges)))

        # select random edge based on probs
        selection_value = random.random()

        cumulation = 0.0
        for edge in sorted_edges:
            cumulation += edge_probs[edge]
            if cumulation > selection_value:
                return edge
                
        # return last edge
        return sorted_edge[-1]
    
    def compute_edge_value(self, edge):
        distance = self.matrix[edge[0]][edge[1]]
        pheromone = self.pheromone_map[edge]
        
        # TODO does not use *_control
        # TODO this can be zero
        prob =  ( 1.0 / distance ) * pheromone
        return prob

    def compute_all_edges_value(self, all_edges):
        prob = 0.0
        for edge in all_edges:
            prob += self.compute_edge_value(edge)
        return prob

    def compute_prob(self, edge, all_edges_value):
        pass

    def leave_pheromones(self, path):
        delta = 1.0 / self.path_distance(path)
        
        for edge in self.path_edges(path):
            existing_pheromone = self.pheromone_map[edge]
            existing_pheromone += delta
            self.pheromone_map[edge] = existing_pheromone
            

    def decay_pheromones(self):
        pmap = self.pheromone_map
        mult = (1 - self.evaporation)
        for key in pmap:
            pheromone = pmap[key]            
            pheromone *= mult



def create_cities(city_count, max_dist=100):
    """
    Create city distance matrix.
    Symmetric, diagonal is zeroes, other values are random between 1 and max_dist
    """
    matrix = []
    for i in xrange(city_count):
        matrix.append( [0] * city_count)

    for x in xrange(city_count):
        for y in xrange(x, city_count):
            if x == y : 
                # diagonal
                value = 0
            else:
                value = random.randint(1,max_dist)
            
            matrix[y][x] = matrix[x][y] = value
    return matrix

if __name__ == '__main__':
    
    import time
    random.seed(97531)

    city_count = 200
    cities = range(city_count)
    distance_matrix = create_cities(city_count, 15)
    
    salesman = AntHillSalesman(distance_matrix, cities, ant_count=10, iterations=200)

    start = time.time()
    all_paths = salesman.all_paths(cities)
    paths = salesman.find_min_distance()
    end = time.time()
    
    print "minimal paths, bruteforced, took " + str(end - start) + " sec"
    for path in paths:
        salesman.display_path(path)