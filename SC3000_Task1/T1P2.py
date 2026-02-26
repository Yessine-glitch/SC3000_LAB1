import json
import heapq


def load_json(path):
    with open(path, "r") as f:
        return json.load(f)


def find_path(G, Dist, Cost, source, target, budget):
    """
    Find shortest path from start to end without exceeding energy budget.
    """

    # (distance, node, energy_used)
    pq = [(0, source, 0)]

    # best[node] = list of (distance, energy) seen so far
    best = {source: [(0, 0)]}

    # For rebuilding the path
    parent = {(source, 0): None}

    while pq:
        dist, node, energy = heapq.heappop(pq)

        # We foound the target
        if node == target:
            return build_path(parent, target, energy), dist, energy

        # discard path if we had found a better path in terms of distance and energy before for this node
        if not is_valid_state(best.get(node, []), dist, energy):
            continue

        for neighbor in G.get(node, []):
            key = f"{node},{neighbor}"

            if key not in Dist or key not in Cost:
                continue

            new_dist = dist + Dist[key]
            new_energy = energy + Cost[key]

            # Aborting the path if above budget
            if new_energy > budget:
                continue

            # If we already have a path to that neighbor that is better we discard
            if is_worse(best.get(neighbor, []), new_dist, new_energy):
                continue

            # Clean up and update the best path cost for this neighbor node
            best[neighbor] = remove_worse(
                best.get(neighbor, []), new_dist, new_energy
            )
            best[neighbor].append((new_dist, new_energy))

            parent[(neighbor, new_energy)] = (node, energy)
            heapq.heappush(pq, (new_dist, neighbor, new_energy))

    return None, float("inf"), float("inf")

def is_worse(records, dist, energy):
    for d, e in records:
        if d <= dist and e <= energy:
            return True
    return False


def is_valid_state(records, dist, energy):
    for d, e in records:
        if d == dist and e == energy:
            return True
        if d <= dist and e <= energy and (d < dist or e < energy):
            return False
    return True


def remove_worse(records, dist, energy):
    return [(d, e) for (d, e) in records
            if not (dist <= d and energy <= e)]

def build_path(parent, end, final_energy):
    path = []
    state = (end, final_energy)

    while state is not None:
        path.append(state[0])
        state = parent[state]

    return list(reversed(path))


def main():
    G = load_json("G.json")
    Dist = load_json("Dist.json")
    Cost = load_json("Cost.json")

    source = "1"
    target = "50"
    budget = 287932

    path, distance, energy = find_path(G, Dist, Cost, source, target, budget)

    if path is None:
        print("No path found within the energy budget.")
        return

    print(f"Shortest path: {'->'.join(path)}")
    print(f"Total distance: {distance}")
    print(f"Total energy: {energy}")


if __name__ == "__main__":
    main()