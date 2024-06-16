import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

public class Graph<T> {
  static class Edge<T> {
    double weight;
    T value;

    Edge(T value, double weight) {
      this.weight = weight;
      this.value = value;
    }
  }

  private HashMap<T, LinkedList<Edge<T>>> adjacencyList = new HashMap<>();

  public void addVertex(T value) {
    adjacencyList.put(value, new LinkedList<>());
  }

  public void addEdge(T from, T to, double weight, boolean bidirectional) {
    if (!adjacencyList.containsKey(from)) {
      this.addVertex(from);
    }
    if (!adjacencyList.containsKey(to)) {
      this.addVertex(to);
    }
    this.adjacencyList.get(from).add(new Edge<T>(to, weight));
    if (bidirectional) {
      this.adjacencyList.get(to).add(new Edge<T>(from, weight));
    }
  }

  void BFS(T start) {
    HashSet<T> visited = new HashSet<>();
    LinkedList<T> queue = new LinkedList<>();
    visited.add(start);
    queue.add(start);
    while (!queue.isEmpty()) {
      T current = queue.poll();
      System.out.println(current);
      for (Edge<T> edge : this.adjacencyList.get(current)) {
        if (!visited.contains(edge.value)) {
          visited.add(edge.value);
          queue.add(edge.value);
        }
      }
    }
  }

  void DFSUtil(T start, HashSet<T> visited) {
    visited.add(start);
    System.out.println(start);
    for (Edge<T> edge : this.adjacencyList.get(start)) {
      if (!visited.contains(edge.value)) {
        DFSUtil(edge.value, visited);
      }
    }
  }

  void DFS(T start) {
    HashSet<T> visited = new HashSet<>();
    DFSUtil(start, visited);
  }

  void greedySearch(T start, T goal) {
    PriorityQueue<Edge<T>> pq = new PriorityQueue<>(Comparator.comparingDouble(edge -> edge.weight));
    HashSet<T> visited = new HashSet<>();
    pq.add(new Edge<>(start, 0.0));

    while (!pq.isEmpty()) {
      Edge<T> currentEdge = pq.poll();
      T current = currentEdge.value;

      if (!visited.contains(current)) {
        visited.add(current);
        System.out.println(current);

        if (current.equals(goal)) {
          System.out.println("Goal " + goal + " reached.");
          return;
        }

        for (Edge<T> edge : this.adjacencyList.get(current)) {
          if (!visited.contains(edge.value)) {
            pq.add(edge);
          }
        }
      }
    }

    System.out.println("No path found from " + start + " to " + goal);
  }

  public Map<T, Double> dijkstra(T start) {
    PriorityQueue<Edge<T>> pq = new PriorityQueue<>(Comparator.comparingDouble(edge -> edge.weight));
    Map<T, Double> distances = new HashMap<>();
    Set<T> visited = new HashSet<>();

    for (T vertex : adjacencyList.keySet()) {
      distances.put(vertex, Double.MAX_VALUE);
    }
    distances.put(start, 0.0);

    pq.add(new Edge<>(start, 0.0));

    while (!pq.isEmpty()) {
      Edge<T> currentEdge = pq.poll();
      T current = currentEdge.value;

      if (!visited.contains(current)) {
        visited.add(current);

        for (Edge<T> neighbor : adjacencyList.get(current)) {
          if (!visited.contains(neighbor.value)) {
            double newDist = distances.get(current) + neighbor.weight;

            if (newDist < distances.get(neighbor.value)) {
              distances.put(neighbor.value, newDist);
              pq.add(new Edge<>(neighbor.value, newDist));
            }
          }
        }
      }
    }

    return distances;
  }

  public static void main(String[] args) {
    Graph<String> graph = new Graph<>();
    graph.addEdge("A", "B", 1, true);
    graph.addEdge("A", "C", 3, true);
    graph.addEdge("B", "D", 1, true);
    graph.addEdge("C", "D", 1, true);

    System.out.println("\nBFS from A: \n");
    graph.BFS("A");

    System.out.println("\nDFS from A: \n");
    graph.DFS("A");

    System.out.println("\nGreedy Search from A to D: \n");
    graph.greedySearch("A", "D");

    System.out.println("\nDijkstra's Algorithm from A: ");
    Map<String, Double> distances = graph.dijkstra("A");
    for (String vertex : distances.keySet()) {
      System.out.println("Distance from A to " + vertex + " is " + distances.get(vertex));
    }
  }
}