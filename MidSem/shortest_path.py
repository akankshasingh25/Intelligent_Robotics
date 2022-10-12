# Dijkstra's shortest path algorithm
 
import sys
class Graph():
 
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]
 
    def printSolution(self, dist):
        print("Vertex \tDistance from Source")
        for node in range(self.V):
            print(node, "\t", dist[node])

    def minDistance(self, dist, sptSet): # finding the vertex with minimum distance value, from the set of vertices not yet included in shortest path tree
 
        # Initialize minimum distance for next node
        min = sys.maxsize
 
        # Search not nearest vertex not in the shortest path tree
        for u in range(self.V):
            if dist[u] < min and sptSet[u] == False:
                min = dist[u]
                min_index = u
 
        return min_index
 
 # main
    def dijkstra(self, src):
 
        dist = [sys.maxsize] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
 
        for cout in range(self.V):
            x = self.minDistance(dist, sptSet) # Picking the minimum distance vertex from the set of vertices not yet seen
            sptSet[x] = True # Putting the minimum distance vertex in theshortest path tree

            #Updating dist value of the adjacent vertices of the picked vertex only if 
            #the current distance is greater than new distance and 
            #the vertex in not in the shortest path tree
            for y in range(self.V):
                if self.graph[x][y] > 0 and sptSet[y] == False and \
                        dist[y] > dist[x] + self.graph[x][y]:
                    dist[y] = dist[x] + self.graph[x][y]
 
        self.printSolution(dist)
 
# Driver's code
if __name__ == "__main__":
    g = visible_graph
    g.dijkstra(0)
 
