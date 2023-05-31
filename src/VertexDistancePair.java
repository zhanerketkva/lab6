public class VertexDistancePair implements Comparable<VertexDistancePair> {
    private int vertex;
    private double distance;

    public VertexDistancePair(int vertex, double distance) {
        this.vertex = vertex;
        this.distance = distance;
    }

    public int getVertex() { //returns the vertex value of the pair
        return vertex;
    }

    public double getDistance() { //returns the distance value of the pair
        return distance;
    }

    @Override
    public int compareTo(VertexDistancePair other) { //implements the compareTo method from the Comparable interface
        return Double.compare(distance, other.distance);
    }
}