from data import runtests
from queue import PriorityQueue
from math import inf


class ResNet:
    def __init__(self, V, E):
        self.connections = [set() for _ in range(V)]
        self.edge_losses = dict()
        self.current_flow = dict()

        for (u, v), losses in E:
            u -= 1
            v -= 1

            self.connections[u].add(v)
            self.connections[v].add(u)
            self.edge_losses[(u, v)] = losses
            self.current_flow[(u, v)] = 0
            self.current_flow[(v, u)] = 0

    def __len__(self):
        return len(self.connections)

    def get_losses(self):
        losses = 0
        for (u, v), flow in self.current_flow.items():
            assert flow == -self.current_flow[(v, u)]
            if flow > 0:
                losses += self.edge_losses[(u, v)][flow - 1]
        return losses

    def connections_and_current_weights_for_vertex(self, u):
        for v in self.connections[u]:
            flow = self.current_flow[(u, v)]

            if flow == 0 and (u, v) in self.edge_losses:
                yield v, self.edge_losses[(u, v)][0]
            elif 0 < flow:
                losses = self.edge_losses[(u, v)]
                if flow < len(losses):
                    old_w = losses[flow - 1]
                    new_w = losses[flow]
                    delta_w = new_w - old_w
                    yield v, delta_w
            # elif flow < 1:
            #     losses = self.edge_losses[(v, u)]
            #     old_w = losses[-flow - 1]
            #     new_w = losses[-flow - 2]
            #     delta_w = new_w - old_w
            #     yield v, -delta_w
            # elif flow < 0:
            #     losses = self.edge_losses[(v, u)]
            #     yield v, -losses[0]


def backtrack_path(parents, s):
    if parents[s] is None:
        return [s]
    return backtrack_path(parents, parents[s]) + [s]


def dijkstra(G, s, t):
    V = len(G)
    Q = PriorityQueue()
    d = [inf] * V
    visited = [False] * V
    parents = [None] * V
    d[s] = 0
    Q.put((d[s], s))

    while not Q.empty():
        _, u = Q.get()
        if visited[u]:
            continue
        visited[u] = True

        for (v, w_uv) in G.connections_and_current_weights_for_vertex(u):
            if d[u] + w_uv < d[v]:
                d[v] = d[u] + w_uv
                Q.put((d[v], v))
                parents[v] = u

    return backtrack_path(parents, t)


def find_expansion(G, s, t):
    return dijkstra(G, s, t)


def process_expansion(G, path):
    u = path[0]
    for v in path[1:]:
        G.current_flow[(u, v)] += 1
        G.current_flow[(v, u)] -= 1
        u = v


def find_min_losses(G, target_division_count):
    s = 0
    t = len(G) - 1

    division_count = 0
    while division_count < target_division_count:
        path = find_expansion(G, s, t)
        if path is None:
            print("rip")
            break
        process_expansion(G, path)
        division_count += 1

    return G.get_losses()


def my_solve(V, k, edges):
    return find_min_losses(ResNet(V, edges), k)


runtests(my_solve)


def tests():
    # Dijkstra

    G = ResNet(4, [
        ((1, 2), [1, 10, 50, 400]),
        ((1, 3), [3, 7, 12]),
        ((2, 4), [1, 2, 3]),
        ((3, 4), [1, 2, 3]),
    ])

    if dijkstra(G, 0, 3) == [0, 1, 3]:
        print("Dijkstra PASS")
    else:
        print("Dijkstra ERROR:" + str(dijkstra(G, 0, 3)))

    G.current_flow[(0, 1)] += 1
    G.current_flow[(1, 0)] -= 1
    G.current_flow[(1, 3)] += 1
    G.current_flow[(3, 1)] -= 1

    if dijkstra(G, 0, 3) == [0, 2, 3]:
        print("Dijkstra PASS")
    else:
        print("Dijkstra ERROR:" + str(dijkstra(G, 0, 3)))

    G.current_flow[(0, 2)] += 1
    G.current_flow[(2, 0)] -= 1
    G.current_flow[(2, 3)] += 1
    G.current_flow[(3, 2)] -= 1

    if dijkstra(G, 0, 3) == [0, 2, 3]:
        print("Dijkstra PASS")
    else:
        print("Dijkstra ERROR:" + str(dijkstra(G, 0, 3)))

    G.current_flow[(0, 2)] += 1
    G.current_flow[(2, 0)] -= 1
    G.current_flow[(2, 3)] += 1
    G.current_flow[(3, 2)] -= 1

    print(G.get_losses())


# tests()
