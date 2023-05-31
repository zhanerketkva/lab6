import java.util.LinkedList;
import java.util.Queue;

public class BreadthFirstSearch implements Search {
    private WeightedGraph<Integer> graph;

    public BreadthFirstSearch(WeightedGraph<Integer> graph) {
        this.graph = graph;
    }

    @Override
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