public class Main {
    public static void main(String[] args) {
        int numOfVertices = 5;
        WeightedGraph<Integer> graph = new WeightedGraph<>(numOfVertices);
        graph.addEdge(0, 1);
        graph.addEdge(0, 2);
        graph.addEdge(1, 3);
        graph.addEdge(2, 4);

        System.out.println("Graph:");
        graph.printGraph();

        System.out.println("BFS:");
        Search bfs = new BreadthFirstSearch(graph);
        bfs.search(0);

        System.out.println("\nDijkstra:");
        Search dijkstra = new DijkstraSearch(graph);
        dijkstra.search(0);
    }
}