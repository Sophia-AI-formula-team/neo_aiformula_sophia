"""Synthetic fusion inputs; no previously recorded course data are loaded.

The single-branch helper isolates legacy numerical tests. The official-graph
fixture builds a hypothetical smooth loop with branch-normal tangents so the
real 80..95 degree rule can be tested with every supplied branch.
"""
import math


def single_synthetic_branch(official):
    ids, points = official.node_ids[:-1], official.points[:-1]
    p, prev, nxt = points[0], points[-1], points[1]
    a, b = (p[0] - prev[0], p[1] - prev[1]), (nxt[0] - p[0], nxt[1] - p[1])
    la, lb = math.hypot(*a), math.hypot(*b)
    if min(la, lb) <= 1e-8:
        raise ValueError('Synthetic fixture needs nonzero edges')
    tx, ty = a[0] / la + b[0] / lb, a[1] / la + b[1] / lb
    norm = math.hypot(tx, ty)
    if norm <= 1e-8:
        raise ValueError('Synthetic fixture cannot use a U-turn vertex')
    area = sum(points[i][0] * points[(i + 1) % len(points)][1] -
               points[i][1] * points[(i + 1) % len(points)][0] for i in range(len(points)))
    side = 1.0 if area >= 0.0 else -1.0
    inner = (p[0] - side * ty / norm * 3.0, p[1] + side * tx / norm * 3.0)
    return {ids[0]: [('test_inner_' + ids[0], inner)]}


def official_graph_branches(official, graph):
    outer = set(official.node_ids[:-1])
    return {key: [(neighbour, graph.nodes[neighbour]) for neighbour in graph.adjacency[key]
                  if neighbour not in outer]
            for key in official.node_ids[:-1]
            if any(neighbour not in outer for neighbour in graph.adjacency[key])}


def hypothetical_smooth_lap(official, graph, spacing=0.30):
    """Generate an artificial loop through the official nodes, not a real map."""
    ids, points = official.node_ids[:-1], official.points[:-1]
    branches = official_graph_branches(official, graph)

    def unit(x, y):
        length = math.hypot(x, y)
        if length <= 1e-8:
            raise ValueError('Synthetic tangent has zero length')
        return (x / length, y / length)

    tangents = []
    for index, identifier in enumerate(ids):
        p, prev, nxt = points[index], points[index - 1], points[(index + 1) % len(points)]
        approx = unit(nxt[0] - prev[0], nxt[1] - prev[1])
        if identifier.endswith('01') or identifier not in branches:
            tangent = approx
        else:
            inner = branches[identifier][0][1]
            tangent = unit(-(p[1] - inner[1]), p[0] - inner[0])
            if tangent[0] * approx[0] + tangent[1] * approx[1] < 0:
                tangent = (-tangent[0], -tangent[1])
        tangents.append(tangent)
    result = []
    for index, p in enumerate(points):
        q = points[(index + 1) % len(points)]
        length = math.dist(p, q)
        count = max(2, math.ceil(length / spacing))
        for step in range(count):
            t = step / count
            t2, t3 = t * t, t * t * t
            a, b, c, d = 2*t3 - 3*t2 + 1, t3 - 2*t2 + t, -2*t3 + 3*t2, t3 - t2
            result.append((a*p[0] + b*length*tangents[index][0] + c*q[0] + d*length*tangents[(index+1)%len(points)][0],
                           a*p[1] + b*length*tangents[index][1] + c*q[1] + d*length*tangents[(index+1)%len(points)][1]))
    return result
