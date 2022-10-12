from sympy import Polygon, Point, Line, Segment2D, Point2D, Line2D

# drawing visibility graph
def visibility_graph(source, goal, polygons):
    points = [source, goal]
    for p in polygons:
        points.extend(p.vertices) # storing vertices of polygon
    points = list(set(points)) # storing unique vertices only
    points.sort(key=lambda v: v.args[0]) # sorting by x-coordinate

    edges = []
    for p in polygons:
        edges.extend(p.sides) # storing polygon edges

    visibile_graph = []

    for p in range(len(points)-1):
        for q in range(p+1, len(points)):
            line = Line(points[p], points[q]) # drawing lines from one point to another including source, goal and unique polygon vertices

            valid = True
            for e in edges:
                intersection = e.intersect(line)
                if intersection.is_empty:   # the lines drawn doesn not intersect the polygons
                    continue

                elif type(intersection) is Segment2D:   # the lines drawn is coincident to the polygon edge
                    continue

                elif not list(intersection)[0] in e.args: # the x-coordinate of the intersection is not the x-coordinate of the edge end points
                    valid = False
                    break
                
            if valid:
                visibile_graph.append((points[p], points[q])) # storing the valid lines from p to q
    return set(visibile_graph)
