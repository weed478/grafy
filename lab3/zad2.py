import os
from dimacs import loadWeightedGraph, readSolution
from queue import PriorityQueue
from math import inf
from random import shuffle, randint


class Node:
    def __init__(self, i):
        self.edges = {}
        self.merged = [i]
        self.active = True

    def __str__(self):
        return str(self.merged)

    def merge(self, u):
        self.merged.append(u)

    def add_edge(self, to, weight):
        self.edges[to] = self.edges.get(to, 0) + weight

    def del_edge(self, to):
        del self.edges[to]

    def kill(self):
        assert len(self.edges) == 0
        self.active = False


class Graph:
    def __init__(self, V, E):
        self.V = [Node(i) for i in range(V)]
        self.real_len = V
        for (u, v, w) in E:
            self.add_edge(u - 1, v - 1, w)

    def __str__(self):
        text = ""
        for node in self.V:
            text += "{}{}:\t{}\n".format("" if node.active else "!", node, list(node.edges.keys()))
            text += "\t\t{}\n".format(list(node.edges.values()))
        return text

    def merge_verts(self, x, y):
        for (yu, yuw) in self.V[y].edges.items():
            if yu != x:
                self.add_edge(x, yu, yuw)
        self.V[x].merge(y)
        self.kill_vert(y)

    def kill_vert(self, u):
        for v in list(self.V[u].edges.keys()):
            self.del_edge(u, v)
        self.V[u].kill()
        self.real_len -= 1

    def add_edge(self, u, v, w):
        self.V[u].add_edge(v, w)
        self.V[v].add_edge(u, w)

    def del_edge(self, u, v):
        self.V[u].del_edge(v)
        self.V[v].del_edge(u)


# def test_merge_verts():
#     E = [
#         (1, 2, 1),
#         (2, 3, 1),
#         (1, 4, 1),
#         (2, 4, 1),
#         (1, 5, 1),
#         (5, 4, 1)
#     ]
#     G = Graph(5, E)
#     print(G)
#     print("Merge 0 1")
#     G.merge_verts(0, 1)
#     print(G)


def minimum_cut_phase(G):
    V = len(G.V)

    s = None
    for u in range(len(G.V)):
        if G.V[u].active:
            s = u
            break
    assert s is not None
    t = None

    visited = [False] * V
    d = [0] * V
    Q = PriorityQueue()
    Q.put((-d[s], s))

    while not Q.empty():
        (_, u) = Q.get()

        if visited[u]:
            continue

        visited[u] = True

        for (v, uvw) in G.V[u].edges.items():
            d[v] += uvw
            Q.put((-d[v], v))

        t, s = s, u

    res = sum(G.V[s].edges.values())

    G.merge_verts(s, t)

    return res


def stoer_wagner(G):
    res = inf
    while G.real_len > 1:
        x = minimum_cut_phase(G)
        # print(res, x)
        res = min(res, x)
    return res


def main():
# if __name__ == "__main__":
    # test_merge_verts()
    # exit(0)

    for graph in os.listdir("./graphs-lab3"):
        if graph == "grid100x100": continue
        graph_path = "./graphs-lab3/" + graph
        V, E = loadWeightedGraph(graph_path)
        sol = int(readSolution(graph_path))

        G = Graph(V, E)
        ans = stoer_wagner(G)

        if sol != ans:
            print("ERR", graph_path, "expected", sol, "got", ans)
        else:
            print("OK", graph_path)


if __name__ == "__main__":
    main()
