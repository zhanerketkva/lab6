import java.util.PriorityQueue;

public class DijkstraSearch implements Search {
    private WeightedGraph<Integer> graph;

    public DijkstraSearch(WeightedGraph<Integer> graph) {
        this.graph = graph;
    }

    @Override
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

    private void printPath(int vertex, int[] previous) { //recursively prints the path
        if (vertex != -1) {
            printPath(previous[vertex], previous);
            System.out.print(vertex + " ");
        }
    }
}
