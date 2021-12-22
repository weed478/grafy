"""
Jakub Karbowski
Algorytmy Grafowe
Projekt 1 Przemarsz wojsk

Problem z tematu sprowadza się do zagadnienia
min-cost max-flow, gdzie koszt przepuszczenia
dodatkowej jednostki przepływu zależy
od aktualnej wartości przepływu na krawędzi.

Wprowadzamy dodatkowy wierzchołek 0
z krawędzią do wierzchołka 1
z zerowym kosztem i przepustowością
równą parametrowi wejściowemu będacym
liczbą oddziałów jakie mamy przeprowadzić.
Dzięki tej modyfikacji znalezienie
maksymalnego przepływu o najmniejszym
koszcie jest rozwiązaniem zadania.

Zaimplementowano algorytm cycle canceling.
Polega on na zainicjalizowaniu sieci residualnej
przepływem o maksymalnej wartości.
Następnie algorytm Bellmana-Forda
stosowany jest do znajdowania cykli
w sieci residualnej o ujemnym koszcie.
Koszt w sieci residualnej jest równy
kosztowi zwiększenia przepływu
w przypadku podążania wzdłuż krawędzi
i ujemnym koszcie zmniejszenia przepływu
w przypadku cofania się po krawędzi
(zmniejszamy przepływ więc 'zyskujemy' wydany koszt).
Kiedy ujemne cykle nie istnieją,
koszt jest optymalny.

Do inicjalizacji sieci residualnej
stosowany jest algorytm Edmondsa-Karpa.

Szukając ujemnych cykli algorytm
Bellmana-Forda puszczany jest
z wierzchołka będącym ujściem
sieci przepływowej.
Gwarantuje to znalezienie ewentualnych cykli.
W przeciwnym razie należałyby sprawdzić
każdy wierzchołek aby obsłużyć
grafy niespójne.
Rozpoczęcie z ujścia sieci daje
jednak gwarancję znalezienia cykli,
ponieważ wierzchołek ten będzie połączony
ścieżkami pomniejszającymi z resztą sieci.
Zródło jednak może nie być połączone z resztą grafu.
Ważną modyfikacją Bellmana-Forda jest
wykluczanie relaksacji krawędzi (u, v),
jeżeli parent[u] == v.
Gdyby taka krawędź została zrelaksowana
i parent[v] ustawiony na u,
koszt powstałego cyklu nie byłby poprawny
(koszt byłby niezerowy ale tak naprawdę
przepływ zostałby taki sam, bo wracamy po
tych samych krawędziach w drugą stronę).

Złożoność:
inicjalizacja - Edmonds-Karp O(VE^2)
główna pętla - O(CVE), gdzie C - koszt przepływu

Optymalizacja:
1. Po znalezieniu ujemnego cyklu
algorytm ponownie próbuje ten sam
cykl w nadziei, że również będzie
ujemny.
2. Algorytm Bellmana-Forda jest przerywany
jeśli nie nastąpi relaksacja żadnej krawędzi.

Heurystyka inicjalizacji przepływu:
Ponieważ algorytm cycle canceling jest algorytmem
iteracyjnym, rozpoczęcie od rozwiązania bliskiego
optymalnemu zmniejszy potrzebną liczbę iteracji
do osiągnięcia optymalnego kosztu.
W tym celu stosuje się uproszczony algorytm
successive shortest paths z algorytmem
Dijkstry do znajdowania ścieżek powiększających
o najmniejszym koszcie.
Algorytm ten to zasadniczo algorytm
Forda-Fulkersona, z pominięciem cofania przepływu
(Dijkstra nie obsługuje ujemnych wag).
Pierwszym etapem całego algorytmu jest
znalezienie "w miarę dobrego" rozwiązania
algorytmem SSP. Następnie algorytm Edmondsa-Karpa
maksymalizuje przepływ (uproszczony SSP może nie
znaleźć maksymalnego przepływu).
Po tej inicjalizacji spełnione są wszystkie
wymagania do puszczenia głównej pętli
algorytmu cycle canceling.
W praktyce, heurystyka ta zmniejsza czas działania
programu prawie o rząd wielkości
(1.9s do 0.22s na moim komputerze).

Inne rozważane algorytmy:
1. Successve shortest paths
Alogrytm SSP nie może być tutaj zastosowany
ze względu na zmienny koszt przepływu
w zależności od jego wartości na danej krawędzi.
2. Minimum mean cycle canceling
Ulepszona wersja algorytmu cycle canceling.
Trudniejsze w implementacji.
"""

from data import runtests
from math import inf
from collections import deque
from queue import PriorityQueue


class ResNet:
    def __init__(self, V, E, s, max_flow):
        V += 1

        self.connections = [set() for _ in range(V)]
        self.edge_losses = dict()
        self.current_flow = dict()

        # Wierzchołek 0 ograniczający przepływ maksymalny
        self.connections[0].add(s)
        self.edge_losses[(0, s)] = [0] * max_flow
        self.current_flow[(0, s)] = 0
        self.current_flow[(s, 0)] = 0

        for (u, v), losses in E:
            self.connections[u].add(v)
            self.connections[v].add(u)
            self.edge_losses[(u, v)] = losses
            self.current_flow[(u, v)] = 0
            self.current_flow[(v, u)] = 0

    def __len__(self):
        return len(self.connections)

    # Całkowity koszt aktualnego przepływu
    def get_losses(self):
        losses = 0
        for (u, v), flow in self.current_flow.items():
            if flow > 0:
                losses += self.edge_losses[(u, v)][flow - 1]
        return losses

    # residualny przepływ i koszt dla krawędzi (u, v)
    def residual_flow_and_cost(self, u, v):
        flow = self.current_flow[(u, v)]
        losses = self.edge_losses.get((u, v), None)

        if flow == 0 and losses is not None:
            capacity = len(losses)
            return capacity - flow, losses[0]
        elif 0 < flow < len(losses):
            capacity = len(losses)
            old_w = losses[flow - 1]
            new_w = losses[flow]
            delta_w = new_w - old_w
            return capacity - flow, delta_w
        elif flow < -1:
            losses = self.edge_losses[(v, u)]
            old_w = losses[-flow - 1]
            new_w = losses[-flow - 2]
            delta_w = new_w - old_w
            return -flow, delta_w
        elif flow < 0:
            losses = self.edge_losses[(v, u)]
            return -flow, -losses[0]

        return None

    # wszystkie krawędzie wychodzące z u
    def residual_flows_and_costs(self, u):
        for v in self.connections[u]:
            fc = self.residual_flow_and_cost(u, v)
            if fc is not None:
                yield v, fc[0], fc[1]

    # wszystkie krawędzie z ich kosztami
    def cost_edges(self):
        for u in range(len(self)):
            for v in self.connections[u]:
                fc = self.residual_flow_and_cost(u, v)
                if fc is not None:
                    yield u, v, fc[1]

    # krawędzie wychodzące z u z ich przepływami residualnymi
    def residual_flows(self, u):
        for v in self.connections[u]:
            fc = self.residual_flow_and_cost(u, v)
            if fc is not None:
                yield v, fc[0]

    # krawędzie dla algorytmu Dijkstry
    def residual_non_negative_costs(self, u):
        for v in self.connections[u]:
            fc = self.residual_flow_and_cost(u, v)
            if fc is not None and fc[1] >= 0:
                yield v, fc[1]


# odtwarza ścieżkę na podstawie tablicy parents
def backtrack_path(parents, s):
    if parents[s] is None:
        return [s]
    return backtrack_path(parents, parents[s]) + [s]


def bellman_ford(E, n, s):
    parent = [None] * n
    d = [inf] * n
    d[s] = 0

    for i in range(n - 1):
        updated = False
        for (u, v, w) in E:
            if d[u] + w < d[v] and parent[u] != v:
                updated = True
                d[v] = d[u] + w
                parent[v] = u
        if not updated:
            return None

    for (u, v, w) in E:
        if d[u] + w < d[v]:
            parent[v] = u
            # budowa cyklu
            C = v
            for _ in range(n):
                C = parent[C]
            cycle = []
            v = C
            while True:
                cycle.append(v)
                if v == C and len(cycle) > 1:
                    break
                v = parent[v]
            cycle.reverse()
            return cycle

    return None


def find_negative_cycle(G, s):
    return bellman_ford(list(G.cost_edges()), len(G), s)


def is_negative_cycle(G, cycle):
    cost = 0
    u = cycle[0]
    for v in cycle[1:]:
        fc = G.residual_flow_and_cost(u, v)
        if fc is None:
            return False
        cost += fc[1]
        u = v
    return cost < 0


def bfs(G, s, t):
    V = len(G)
    visited = [False] * V
    parent = [None] * V
    Q = deque()

    visited[s] = True
    Q.append(s)

    while len(Q) > 0:
        u = Q.popleft()
        for v, w in G.residual_flows(u):
            if w > 0 and not visited[v]:
                visited[v] = True
                parent[v] = u
                Q.append(v)

    if parent[t] is None:
        return None

    return backtrack_path(parent, t)


# szuka jaki przepływ może być puszczony po danej ścieżce
def find_path_flow(G, path):
    flow = inf
    u = path[0]
    for v in path[1:]:
        flow = min(flow, len(G.edge_losses[(u, v)]) - G.current_flow[(u, v)])
        u = v
    return flow


# zwiększa przepływ na ścieżce
def run_flow(G, path, flow):
    u = path[0]
    for v in path[1:]:
        G.current_flow[(u, v)] += flow
        G.current_flow[(v, u)] -= flow
        u = v


# zasadniczo Edmonds-Karp
def ford_fulkerson(G, s, t):
    max_flow = 0

    while True:
        path = bfs(G, s, t)
        if path is None:
            break
        flow = find_path_flow(G, path)
        run_flow(G, path, flow)
        max_flow += flow

    return max_flow


def find_max_flow(G, s, t):
    return ford_fulkerson(G, s, t)


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

        for v, w_uv in G.residual_non_negative_costs(u):
            if d[u] + w_uv < d[v]:
                d[v] = d[u] + w_uv
                Q.put((d[v], v))
                parents[v] = u

    if d[t] < inf:
        return backtrack_path(parents, t)
    else:
        return None


# uproszczony SSP
def init_flow_heuristic(G, s, t):
    while True:
        path = dijkstra(G, s, t)
        if path is None:
            break
        run_flow(G, path, 1)


def min_cost_max_flow(G, s, t):
    init_flow_heuristic(G, s, t)
    find_max_flow(G, s, t)

    while True:
        cycle = find_negative_cycle(G, t)
        if cycle is None:
            break
        while True:
            run_flow(G, cycle, 1)
            # próba ponownego wykorzystania tego samego cyklu
            if not is_negative_cycle(G, cycle):
                break

    return G.get_losses()


def my_solve(V, k, edges):
    G = ResNet(V, edges, 1, k)
    return min_cost_max_flow(G, 0, len(G) - 1)


runtests(my_solve)
