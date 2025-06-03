import math

# Square trajectory: list of (x, y)
RECTANGLE_POINTS = [
    (-0.64, -0.77),
    (0.64, -0.77),
    (0.64, 0.4),
   
    (-0.64, 0.4),
    (-0.64, -0.77),
]

TEST_POINTS = [
    (-0.64, -0.77),
]

# Circle trajectory: 8 points around a circle
def generate_circle_points(center_x=0.0, center_y=0.0, radius=0.5, num_points=8):
    return [
        (
            center_x + radius * math.cos(2 * math.pi * i / num_points),
            center_y + radius * math.sin(2 * math.pi * i / num_points)
        )
        for i in range(num_points)
    ]

CIRCLE_POINTS = generate_circle_points()

# Eight trajectory: Lissajous curve, 16 points
def generate_eight_points(num_points=16):
    a = 1.0
    b = 0.23
    points = []
    for i in range(num_points):
        t = 2 * math.pi * i / num_points
        x = a * math.sin(t)
        y = b * math.sin(2 * t)
        points.append((x, y))
    # Optionally, return to start
    points.append(points[0])
    return points

EIGHT_POINTS = generate_eight_points()
