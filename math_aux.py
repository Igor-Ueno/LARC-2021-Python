from math import sqrt, atan

# class Point:
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y
    
#     def set_coordinates(self, x, y):
#         self.x = x
#         self.y = y
    
#     def get_coordinates(self, x, y):
#         return self.x, self.y

class Polynomial:
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c
    
    def set_coefficients(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c
    
    def get_coefficients(self):
        return self.a, self.b, self.c
    
    def delta(self):
        return self.b ** 2 - 4 * self.a * self.c
    
    def roots(self):
        root1 = (-self.b + sqrt(self.delta())) / (2 * self.a)
        root2 = (-self.b - sqrt(self.delta())) / (2 * self.a)
        return root1, root2

def project_point_onto_circle(point_x, point_y, radius, center_x, center_y):
    """Input: a point's coordinates, and the circle's radius and center.
    Description: Calculates the coordinates of the pojetion of the point onto a certain circle.
    Output: Coordinates of the projected point."""
    # Calculation of the line that interconnect point and circle's center (y = m * x + c)
    m = (point_y - center_y) / (point_x - center_x)
    c = point_y - m * point_x

    polynomial = Polynomial(1 + m**2, 2 * (m * c - center_x - m * center_y), c ** 2 + center_x ** 2 + center_y ** 2 - radius ** 2 - 2 * c * center_y)

    x1, x2 = polynomial.roots()

    if(point_x > center_x):
        x = x1
    else:
        x = x2
    
    y = m * x + c

    return x, y

def angle_between_pair_lines2(point1_x, point1_y, point2_x, point2_y, intersection_point_x, intersection_point_y):
    """Input: Three pair of points that defines a pair of lines intersecting.
    Description: Calculates the smaller angle between the pair of lines.
    Output: Smaller angle between the pair of lines."""
    if point1_x > intersection_point_x:
        m1 = (point1_y - intersection_point_y) / (0.001 if point1_x - intersection_point_x == 0 else point1_x - intersection_point_x)
    else:
        m1 = (intersection_point_y - point1_y) / (0.001 if intersection_point_x - point1_x == 0 else intersection_point_x - point1_x)

    if point2_x > intersection_point_x:
        m2 = (point2_y - intersection_point_y) / (0.001 if point2_x - intersection_point_x == 0 else point2_x - intersection_point_x)
    else:
        m2 = (intersection_point_y - point2_y) / (0.001 if intersection_point_x - point2_x == 0 else intersection_point_x - point2_x)

    theta = abs(atan((m1 - m2) / (1 + m1 * m2)))

    return theta

def angle_between_pair_lines(point1_x, point1_y, point2_x, point2_y, intersection_point_x, intersection_point_y):
    """Input: Three pair of points that defines a pair of lines intersecting.
    Description: Calculates the smaller angle between the pair of lines.
    Output: Smaller angle between the pair of lines."""
    point1_x -= intersection_point_x * -1
    point1_y -= intersection_point_y
    point2_x -= intersection_point_x * -1
    point2_y -= intersection_point_y
    
    m1 = point1_y / (0.001 if point1_x == 0 else point1_x)
    m2 = point2_y / (0.001 if point2_x == 0 else point2_x)

    # theta = atan((m1 - m2) / (1 + m1 * m2))
    theta = abs(atan(m1))

    return theta