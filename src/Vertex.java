import java.util.ArrayList;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;

public class Vertex<T> { //vertex object with a value of type T
    private T value;

    public Vertex(T value) {
        this.value = value;
    }

    public T getValue() {
        return value;
    }
}