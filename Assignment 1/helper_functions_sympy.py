"""Helper functions through SymPy for Bug Simulations"""

import math
import sympy

#1. Equation of line from two points
def line_equation(p1, p2):
    l = Line(p1, p2)
    return l.equation()
 
# line from two points
def line_between_two_points(p1, p2):
    return Line(p1, p2)
  
#2. Distance between two points
def distance_between(p1,p2):
    p1 = Point(p1)
    p2 = Point(p2)
    return p1.distance(p2)
   
# Angle between two points
def angle_of(p1,p2):
  O = Point(0,0)
  l1 = Line(O,p1)
  l2 = Line(O,p2)
  return l2.angle_between(l1)

#3. Perpendicular distance between a point and line segment
def distance_from_line(x, p1, p2):
  s = Segment(p1, p2)
  return s.distance(x)

#4. Distance of a point and polygon
#Input = 'Point(X)'
def distance_from_polygon(x, poly):
  return poly.distance(x)
  
#5. Equation of tangents to polygon
def tangents_to_polygon(x, poly):
  line_from_x_to_polygon = []
  tangent = []

  # getting lines from x to vertices of polygon
  for i in range(0, len(poly.vertices)):
    line_from_x_to_polygon.append(Line(X, poly.vertices[i]))
  
  # if the line intersects the polygon only once it is a tangent
  for i in range(0, len(line_from_x_to_polygon)):
      if len(line_from_x_to_polygon[i].intersection(poly)) > 1:
        tangent.append(line_from_x_to_polygon[i])

  #removing duplicates
  tangent = [*set(tangent)]
  return tangent

#6. Intersection of two polygons
def intersection_polygon(poly1, poly2):
  return poly1.intersection(poly2)


# For Bug algorithms
def on_line(x, p1, p2, tolerance=0.02):   
    distance = distance_from_line(x, p1, p2)
    if distance > tolerance:
        return False
    else:
        return True

def get_bearing_in_degrees(north):
    rad = math.atan2(north[0], north[1])
    bearing = (rad - 1.5708) / 3.14 * 180.0
    bearing += 180
    if bearing < 0.0:
        bearing = bearing + 360.0

    return bearing
