import java.util.LinkedList;

public class WeightedGraph<T> {
    int numOfVertices;
    LinkedList<Vertex<T>>[] adjList;

    public WeightedGraph(int numOfVertices) {
        this.numOfVertices = numOfVertices;
        this.adjList = new LinkedList[numOfVertices];
        for (int i = 0; i < numOfVertices; i++) {
            this.adjList[i] = new LinkedList<>();
        }
    }

    public void addEdge(int source, int destination) { //adds a weighted edge
        validateVertex(source);
        validateVertex(destination);
        adjList[source].add((Vertex<T>) new Vertex<>(destination));
        adjList[destination].add((Vertex<T>) new Vertex<>(source));
    }

    public void removeEdge(int source, int destination) { //removes the weighted edge
        validateVertex(source);
        validateVertex(destination);
        adjList[source].remove(new Vertex<>(destination));
        adjList[destination].remove(new Vertex<>(source));
    }

    public boolean hasEdge(int source, int destination) { //checks if there is a weighted edge
        validateVertex(source);
        validateVertex(destination);
        return adjList[source].contains(new Vertex<>(destination));
    }

    public LinkedList<Vertex<T>> getNeighbor(int vertex) { //returns a linked list of neighboring vertices
        validateVertex(vertex);
        return adjList[vertex];
    }

    public void printGraph() { //prints the graph
        for (int i = 0; i < numOfVertices; i++) {
            System.out.print("Vertex " + i + " connected to: ");
            for (Vertex<T> neighbor : adjList[i]) {
                System.out.print(neighbor.getValue() + " ");
            }
            System.out.println();
        }
    }

    void validateVertex(int index) { //validates if the given vertex index is within the valid range
        if (index < 0 || index >= numOfVertices) {
            throw new IllegalArgumentException("Vertex " + index + " is out of range!");
        }
    }
}
