#include <iostream>
#include <omp.h>
#include <queue>
#include <vector>

void bfs_parallel(const std::vector<std::vector<int>> &graph, int start_node) {
  int num_nodes = graph.size();
  std::vector<bool> visited(num_nodes, false);
  std::queue<int> bfs_queue;

  // Mark the start node as visited and enqueue it
  visited[start_node] = true;
  bfs_queue.push(start_node);

  while (!bfs_queue.empty()) {
    int current_node = bfs_queue.front();
    bfs_queue.pop();

    // Process the current node
    std::cout << "Visited Node: " << current_node << " (Thread "
              << omp_get_thread_num() << ")" << std::endl;

// Enqueue unvisited neighbors of the current node
#pragma omp parallel for
    for (int neighbor = 0; neighbor < num_nodes; ++neighbor) {
      if (graph[current_node][neighbor] && !visited[neighbor]) {
// Mark the neighbor as visited and enqueue it
#pragma omp critical
        {
          visited[neighbor] = true;
          bfs_queue.push(neighbor);
        }
      }
    }
  }
}

int main() {
  // Example graph represented as an adjacency matrix
  std::vector<std::vector<int>> graph = {{0, 1, 1, 0, 0},
                                         {1, 0, 0, 1, 1},
                                         {1, 0, 0, 0, 1},
                                         {0, 1, 0, 0, 0},
                                         {0, 1, 1, 0, 0}};

  int start_node = 0;

  // Set the number of OpenMP threads
  int num_threads = 4;
  omp_set_num_threads(num_threads);

  // Perform parallel BFS
  bfs_parallel(graph, start_node);

  return 0;
}
