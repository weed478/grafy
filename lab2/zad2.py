from dimacs import loadWeightedGraph, readSolution
import os
from math import inf
from zad1 import to_dict, find_max_flow


if __name__ == "__main__":
    for graph in os.listdir("./connectivity"):
        if graph == "grid100x100": continue
        if graph == "clique200": continue
        graph_path = "./connectivity/" + graph
        print(graph_path)
        V, E = loadWeightedGraph(graph_path)
        sol = int(readSolution(graph_path))

        new_E = []
        for (u, v, _) in E:
            new_E.append((u, v, 1))
            new_E.append((v, u, 1))

        min_flow = inf

        for u in range(1, V):
            G = to_dict(V, new_E)
            max_flow = find_max_flow(G, u, 0)
            min_flow = min(min_flow, max_flow)

        if sol != min_flow:
            print("ERR", graph_path, "expected", sol, "got", min_flow)
        else:
            print("OK", graph_path)
