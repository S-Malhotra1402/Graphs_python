class Graph:
    def __init__(self, vertices):
        self.V = vertices  # No. of vertices
        self.graph = []

    # function to add an edge to graph
    def addEdge(self, u, v, w):
        self.graph.append([u, v, w])
if __name__ == '__main__':
    g = Graph(5)
    g.addEdge(0, 1, -1)
    g.addEdge(0, 2, 4)
    g.addEdge(1, 2, 3)
    g.addEdge(1, 3, 2)
    g.addEdge(1, 4, 2)
    g.addEdge(3, 2, 5)
    g.addEdge(3, 1, 1)
    g.addEdge(4, 3, -3)

#DFS graph TC - O(V+E) ,SC - O(V)
class Solution:
    # Function to return a list containing the DFS traversal of the graph.
    def dfsOfGraph(self, V, adj):
        visited = []
        def dfs(curnode):
            if curnode not in visited:
                visited.append(curnode)
                for i in adj[curnode]:
                    dfs(i)
        dfs(0)
        return visited

#BFS graph  TC - O(V+2E) #2E because of the adjacent nodes ,SC - O(V)
#only one node can be at level zero ,the order is in increasing way of distance from source
from typing import List
from queue import Queue
class Solution:
    #Function to return Breadth First Traversal of given graph.
    def bfsOfGraph(self, V, adj):
        res = []
        vis = [0]*V
        q = [0]
        vis[0] = 1
        while len(q)!=0:
            temp = q.pop(0)
            res.append(temp)
            for i in adj[temp]:
                if vis[i] == 0:
                    q.append(i)
                    vis[i] = 1
        return res
    #alternate
    def bfsOfGraph(self, V, adj):
        q = []
        res = []
        q.append(0)
        res.append(0)
        while len(q)!=0:
            cur = q.pop(0)
            for i in adj[cur]:
                if i not in res:
                    res.append(i)
                    q.append(i)
        return res
#TOPOSORT using BFS (Kahn's algo)
#works only for directed acyclic graphs DAG
class Solution:
    def topoSort(self, V, adj):
        inDegree = [0] * V  #to store number of incoming edges to a vertex
        for i in range(V):
            for j in adj[i]:
                inDegree[j] += 1
        q = []
        res = []
        for i in range(V):
            if inDegree[i] == 0:#append the values to queue and res which have inDegree ==0
                res.append(i)   #as they have no incoming edges
                q.append(i)

        while len(q) != 0:
            cur = q.pop(0) #now start taking each value from queue
            for i in adj[cur]: #look for it's edge members and start decreasing inDegree value
                inDegree[i] -= 1
                if inDegree[i] == 0:   #as inDegree reaches zero append it to the result
                    res.append(i)
                    q.append(i)
        return res

#Dijkstra algorithm    TC - O(E*logV) as there are O(E) in priority queue

    #Function to find the shortest distance of all the vertices from the source vertex S.
import heapq
class Solution:
    def dijkstra(self, V, adj, S):
        dis= [int(100000000)]*V
        dis[S] = 0
        q = [] #minheap
        q.append((0,S)) #(dist,source)
        while q:
            l,u = heapq.heappop(q) #getting the minimum value index
            for i in adj[u]:
                v,weight = i
                if dis[v]>dis[u]+weight: #u is dist of shortest node + additional y
                    dis[v] = dis[u]+weight
                    heapq.heappush(q,(dis[v],v))
        return dis

#BELLMAN FORD   TC - O(V*E)
#works for negative edges and negative cycles
#does not work for negative edge undirected graph
#first iteration gaurantees to give all shortest path atmost 1 edge long
#second iteration gaurantees to give shortest path atmost 2 edges long
class Solution:
    # Function to construct and return cost of MST for a graph represented using adjacency matrix representation

    def bellman_ford(self, V, edges, S):
        dist = [int(1e8)]*V
        dist[S] = 0
        for i in range(V+1):
            for u,v,wt in edges: #[start , end , weight ]
                if dist[u]+wt<dist[v]:
                    if i==V: #if the value is still decreasing after n cycles then there is a negative cycle
                        return [-1]
                    dist[v] = dist[u]+wt
        return dist
#FlOYD WARSHALL  TC- O(V^3)
class Solution:
	def shortest_distance(self, mat):
        v= len(mat) #to get number of vertices
        for i in range(v):
            for j in range(v):
                if mat[i][j] == -1: #not accessible vertex
                    mat[i][j] = 10e9 #make it at infinite distance
        for via in range(v): #via will iterate from 0 to v-1
            for i in range(v):
                for j in range(v):
                    mat[i][j] = min(mat[i][j],mat[i][via]+mat[via][j]) #to go via every node between i and j
        for i in range(v):
            for j in range(v):
                if mat[i][j] == 10e9: #converting back the values
                    mat[i][j] = -1
        return mat


