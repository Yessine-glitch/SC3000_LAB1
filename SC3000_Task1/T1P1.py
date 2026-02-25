import json
import heapq

def load_json(filepath):
    with open(filepath, 'r') as f:
        return json.load(f)

def dijkstra(G, Dist, source, target):
    """
    Dijkstra's algorithm with no energy constraint (part 1 of Task 1)
    """
    # Priority queue: (distance, node) - to choose greedily the closest neighbor
    pq = [(0, source)]

    # Best known distance to each node
    dist = {source: 0}

    # Previous node on the shortest path
    prev = {source: None}

    # Set to keep track of visited nodes
    visited = set()

    while pq:
        current_dist, u = heapq.heappop(pq)

        if u in visited:
            continue
        visited.add(u)

        #We reached the target so we're done
        if u == target:
            break

        # We need to relax the distance to all its neighbors
        for v in G.get(u, []):
            edge_key = f"{u},{v}"
            if edge_key not in Dist:
                continue
            weight = Dist[edge_key]
            new_dist = current_dist + weight

            #add to pq if first time exploring the node or we found a smaller distance
            if v not in dist or new_dist < dist[v]:
                dist[v] = new_dist
                prev[v] = u
                heapq.heappush(pq, (new_dist, v))

    # No path found
    if target not in dist:
        return None, float('inf')

    # Path reconstruction
    path = []
    node = target
    while node is not None:
        path.append(node)
        node = prev[node]
    path.reverse()

    return path, dist[target]

def compute_energy(path, Cost):
    total = 0
    for i in range(len(path) - 1):
        edge_key = f"{path[i]},{path[i+1]}"
        total += Cost.get(edge_key, 0)
    return total

def main():

    G    = load_json('G.json')
    Dist = load_json('Dist.json')
    Cost = load_json('Cost.json')

    source = '1'
    target = '50'

    path, shortest_dist = dijkstra(G, Dist, source, target)

    if path is None:
        print("No path found.")
        return

    total_energy = compute_energy(path, Cost)
    path_str = '->'.join(path)

    print(f"Shortest path: {path_str}.")
    print(f"Shortest distance: {shortest_dist}.")
    print(f"Total energy cost: {total_energy}.")

if __name__ == '__main__':
    main()