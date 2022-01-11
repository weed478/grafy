from dimacs import loadWeightedGraph


class Node:
    def __init__(self, idx):
        self.idx = idx
        self.out = set()

    def connect_to(self, v):
        self.out.add(v)


class Graph:
    def __init__(self, V, L, correct_index=True):
        if not correct_index:
            self.nodes = [None] + [Node(i) for i in range(1, V + 1)]
            for u, v, w in L:
                self[u].connect_to(v)
                self[v].connect_to(u)
        else:
            self.nodes = [Node(i) for i in range(V)]
            for u, v, w in L:
                self[u - 1].connect_to(v - 1)
                self[v - 1].connect_to(u - 1)

    def __len__(self):
        return len(self.nodes)

    def __getitem__(self, i):
        return self.nodes[i]

    def __setitem__(self, i, v):
        self.nodes[i] = v


def load_and_build_graph(graph_file_path):
    V, L = loadWeightedGraph(graph_file_path)
    return Graph(V, L)
