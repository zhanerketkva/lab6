# lab6
# BreadthFirstSearch
# BreadthFirstSearch:
Initializes the BreadthFirstSearch object with the provided WeightedGraph
```
public BreadthFirstSearch(WeightedGraph<Integer> graph) {
        this.graph = graph;
    }
```
# search:
Performs the breadth-first search algorithm starting from the specified startVertex
```
    public void search(int startVertex) { //performs the breadth-first search algorithm starting from the specified startVertex
        graph.validateVertex(startVertex);
        boolean[] visited = new boolean[graph.numOfVertices];
        Queue<Integer> queue = new LinkedList<>();
        visited[startVertex] = true;
        queue.add(startVertex);

        while (!queue.isEmpty()) {
            int vertex = queue.poll();
            System.out.print(vertex + " ");

            for (Vertex<Integer> neighbor : graph.adjList[vertex]) {
                int neighborValue = neighbor.getValue();
                if (!visited[neighborValue]) {
                    visited[neighborValue] = true;
                    queue.add(neighborValue);
                }
            }
        }
    }
}
```
# DijkstraSearch
# DijkstraSearch:
Initializes the DijkstraSearch object with the provided WeightedGraph
```
public DijkstraSearch(WeightedGraph<Integer> graph) {
        this.graph = graph;
    }
```
# search:
Performs Dijkstra's algorithm starting from the specified startVertex
```
  public void search(int startVertex) { //performs Dijkstra's algorithm starting from the specified startVertex
        graph.validateVertex(startVertex);
        double[] distances = new double[graph.numOfVertices];
        boolean[] visited = new boolean[graph.numOfVertices];
        int[] previous = new int[graph.numOfVertices];

        for (int i = 0; i < graph.numOfVertices; i++) {
            distances[i] = Double.POSITIVE_INFINITY;
            visited[i] = false;
            previous[i] = -1;
        }

        distances[startVertex] = 0;

        PriorityQueue<VertexDistancePair> pq = new PriorityQueue<>();
        pq.offer(new VertexDistancePair(startVertex, 0));

        while (!pq.isEmpty()) {
            VertexDistancePair pair = pq.poll();
            int vertex = pair.getVertex();
            visited[vertex] = true;

            for (Vertex<Integer> neighbor : graph.adjList[vertex]) {
                int neighborValue = neighbor.getValue();
                double weight = getWeight(vertex, neighborValue);
                if (!visited[neighborValue]) {
                    double newDistance = distances[vertex] + weight;
                    if (newDistance < distances[neighborValue]) {
                        distances[neighborValue] = newDistance;
                        previous[neighborValue] = vertex;
                        pq.offer(new VertexDistancePair(neighborValue, newDistance));
                    }
                }
            }
        }

        printShortestPaths(startVertex, distances, previous);
    }
```
# getWeight:
Returns the weight of the edge between the source and destination vertices
```
 private double getWeight(int source, int destination) { //returns the weight of the edge
        for (Vertex<Integer> neighbor : graph.adjList[source]) {
            if (neighbor.getValue() == destination) {
                // Assume there is an associated weight with the edge
                // You can modify this based on your actual implementation
                return 1.0;
            }
        }
        return Double.POSITIVE_INFINITY;
    }
```
# printShortestPaths:
Prints the shortest paths and distances from the startVertex to all other vertices
```
  private void printShortestPaths(int startVertex, double[] distances, int[] previous) { //prints the shortest paths and distances
        for (int i = 0; i < graph.numOfVertices; i++) {
            if (i == startVertex) {
                continue;
            }

            System.out.print("Shortest path from " + startVertex + " to " + i + ": ");
            if (distances[i] == Double.POSITIVE_INFINITY) {
                System.out.println("No path exists");
            } else {
                printPath(i, previous);
                System.out.println(", distance: " + distances[i]);
            }
        }
    }
```
# printPath: 
Recursively prints the path from the startVertex to the given vertex
```
  private void printPath(int vertex, int[] previous) { //recursively prints the path
        if (vertex != -1) {
            printPath(previous[vertex], previous);
            System.out.print(vertex + " ");
        }
    }
 ```
 # Search:
 Defines the search algorithm that will be implemented by classes that implement this interface
 ```
 public interface Search {
    void search(int startVertex); //search algorithm starting from the specified startVertex
}
```
# Vertex
# Vertex:
Initializes a Vertex object with the provided value
```
public Vertex(T value) {
        this.value = value;
    }
```
# getValue:
Returns the value associated with the vertex
```
 public T getValue() {
        return value;
    }
```
# VertexDistancePair
# getVertex:
Returns the vertex value associated with the VertexDistancePair
```
  public int getVertex() { //returns the vertex value of the pair
        return vertex;
    }
```
# compareTo:
Implements the compareTo method from the Comparable interface
```
public int compareTo(VertexDistancePair other) { //implements the compareTo method from the Comparable interface
        return Double.compare(distance, other.distance);
    }
```
# WeightedGraph class
# WeightedGraph:
Initializes a WeightedGraph object with the provided number of vertices.
```
public WeightedGraph(int numOfVertices) {
        this.numOfVertices = numOfVertices;
        this.adjList = new LinkedList[numOfVertices];
        for (int i = 0; i < numOfVertices; i++) {
            this.adjList[i] = new LinkedList<>();
        }
    }
 ```
# addEdge:
Adds a weighted edge between the source and destination vertices
```
 public void addEdge(int source, int destination) { //adds a weighted edge
        validateVertex(source);
        validateVertex(destination);
        adjList[source].add((Vertex<T>) new Vertex<>(destination));
        adjList[destination].add((Vertex<T>) new Vertex<>(source));
    }
```
# removeEdge:
Removes the weighted edge between the source and destination vertices
```
 public void removeEdge(int source, int destination) { //removes the weighted edge
        validateVertex(source);
        validateVertex(destination);
        adjList[source].remove(new Vertex<>(destination));
        adjList[destination].remove(new Vertex<>(source));
    }
```
# hasEdge:
Checks if there is a weighted edge between the source and destination vertices
```
  public boolean hasEdge(int source, int destination) { //checks if there is a weighted edge
        validateVertex(source);
        validateVertex(destination);
        return adjList[source].contains(new Vertex<>(destination));
    }
```
# getNeighbor:
Returns a linked list of neighboring vertices for a given vertex
```
 public LinkedList<Vertex<T>> getNeighbor(int vertex) { //returns a linked list of neighboring vertices
        validateVertex(vertex);
        return adjList[vertex];
    }
```
# printGraph:
Prints the graph by iterating over each vertex and its neighboring vertices
```
  public void printGraph() { //prints the graph
        for (int i = 0; i < numOfVertices; i++) {
            System.out.print("Vertex " + i + " connected to: ");
            for (Vertex<T> neighbor : adjList[i]) {
                System.out.print(neighbor.getValue() + " ");
            }
            System.out.println();
        }
    }
```
# validateVertex:
Validates if the given vertex index is within the valid range
```
 void validateVertex(int index) { //validates if the given vertex index is within the valid range
        if (index < 0 || index >= numOfVertices) {
            throw new IllegalArgumentException("Vertex " + index + " is out of range!");
        }
    }
```
