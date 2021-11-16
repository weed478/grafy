def to_adj_undirected(V, E):
    G = [[] for _ in range(V)]

    for (u, v, w) in E:
        G[u - 1].append((v - 1, w))
        G[v - 1].append((u - 1, w))

    return G
