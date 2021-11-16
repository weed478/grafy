import os
from time import time
from dimacs import loadWeightedGraph, readSolution
from var1_findunion import findMaxMinEdge as var1
from var2_bisect import findMaxMinEdge as var2
from var3_dijkstra import find_max_min_edge as var3

if __name__ == "__main__":
    vars = [var1, var2, var3]

    passed = True

    for (var_i, var) in enumerate(vars):
        start = time()

        for graph in os.listdir("./graphs"):
            graphPath = "./graphs/" + graph
            V, E = loadWeightedGraph(graphPath)
            sol = int(readSolution(graphPath))
            ans = var(V, E)
            if sol != ans:
                passed = False
                print("[var\t{}]\t{}\texpected:\t{}\tgot:\t{}".format(var_i, graph, sol, ans))

        elapsed = time() - start
        print("var {} took {}s".format(var_i, elapsed))

    if passed:
        print("PASS")
