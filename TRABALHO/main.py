from src.busca import dfs, bfs, branch_and_bound, a_star, dijkstra, read_graph

def main():
    # Leitura
    graph = read_graph('graph.txt')

    
    start = 0
    goal = 4

    # DFS
    nodes_analyzed, path_length, path = dfs(graph, start, goal)
    print(f"DFS: Nós analisados: {nodes_analyzed}, Comprimento do caminho: {path_length}, Caminho: {path}")

    # BFS
    nodes_analyzed, path_length, path = bfs(graph, start, goal)
    print(f"BFS: Nós analisados: {nodes_analyzed}, Comprimento do caminho: {path_length}, Caminho: {path}")

    # Branch and Bound
    nodes_analyzed, path_length, path = branch_and_bound(graph, start, goal)
    print(f"Branch and Bound: Nós analisados: {nodes_analyzed}, Comprimento do caminho: {path_length}, Caminho: {path}")

    # A*
    nodes_analyzed, path_length, path = a_star(graph, start, goal)
    print(f"A*: Nós analisados: {nodes_analyzed}, Comprimento do caminho: {path_length}, Caminho: {path}")

    # dijkstra
    nodes_analyzed, path_length, path = dijkstra(graph, start, goal)
    print(f"Dijkstra: Nós analisados: {nodes_analyzed}, Comprimento do caminho: {path_length}, Caminho: {path}")

if __name__ == '__main__':
    main()
