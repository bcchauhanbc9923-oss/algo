import heapq
import networkx as nx
import matplotlib.pyplot as plt

class CityMap:
    def __init__(self):
        self.graph = {}

    def add_road(self, a, b, d):
        self.graph.setdefault(a, []).append((b, d))
        self.graph.setdefault(b, []).append((a, d))

    def dijkstra(self, start):
        dist = {v: float('inf') for v in self.graph}
        prev = {v: None for v in self.graph}
        dist[start] = 0
        pq = [(0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            for v, w in self.graph[u]:
                if dist[v] > d + w:
                    dist[v] = d + w
                    prev[v] = u
                    heapq.heappush(pq, (dist[v], v))
        return dist, prev

def reconstruct_path(prev, start, target):
    path = []
    node = target
    while node:
        path.append(node)
        node = prev[node]
    path.reverse()
    return path if path and path[0] == start else []

def draw_graph(city):
    G = nx.Graph()
    for u in city.graph:
        for v, w in city.graph[u]:
            G.add_edge(u, v, weight=w)
    pos = nx.spring_layout(G, seed=42)
    nx.draw(G, pos, with_labels=True, node_color="#7bc8f6", node_size=900,
            font_size=10, font_weight="bold", edgecolors="black")
    nx.draw_networkx_edge_labels(G, pos, edge_labels=nx.get_edge_attributes(G, "weight"))
    return G, pos

def interactive_route_planner(city):
    G, pos = draw_graph(city)
    clicks = []

    def on_click(event):
        if not event.inaxes:
            return
        nearest_node = min(G.nodes, key=lambda n: (pos[n][0] - event.xdata) ** 2 + (pos[n][1] - event.ydata) ** 2)
        clicks.append(nearest_node)
        print(f" Selected: {nearest_node}")
        if len(clicks) == 2:
            start, target = clicks
            dist, prev = city.dijkstra(start)
            path = reconstruct_path(prev, start, target)
            if not path:
                print(f" No path found between {start} and {target}")
            else:
                total = sum(w for u, v in zip(path, path[1:]) 
                            for nbr, w in city.graph[u] if nbr == v)
                edges = list(zip(path, path[1:]))
                plt.cla()  
                nx.draw(G, pos, with_labels=True, node_color="#7bc8f6", node_size=900,
                        font_size=10, font_weight="bold", edgecolors="black")
                nx.draw_networkx_edge_labels(G, pos, edge_labels=nx.get_edge_attributes(G, "weight"))
                nx.draw_networkx_edges(G, pos, edgelist=edges, width=3, edge_color="red")
                plt.title(f"Shortest Path {start} → {target} : {path}\nTotal Distance = {total}", fontsize=10)
                plt.draw()
                print(f" Shortest Path {start} → {target}: {path}")
                print(f" Total Distance: {total}\n")
                clicks.clear()  

    plt.gcf().canvas.mpl_connect("button_press_event", on_click)
    plt.show()

def demo():
    city = CityMap()
    roads = [
        ("A", "B", 4), ("A", "C", 2), ("B", "C", 5), ("B", "D", 10),
        ("C", "E", 3), ("E", "D", 4), ("D", "F", 11), ("E", "F", 2),
    ]
    for a, b, d in roads:
        city.add_road(a, b, d)
    print(" SMART CITY ROUTE PLANNER")
    print("Click any TWO cities on the graph to find the shortest route.")
    print("The red line shows the path and total distance.\n")
    interactive_route_planner(city)

if __name__ == "__main__":
    demo()
