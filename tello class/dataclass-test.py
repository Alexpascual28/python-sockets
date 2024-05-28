from dataclasses import dataclass, fields

@dataclass
class Point:
    x: float = 1.5
    y: float = 0.2
    z: float = 0.0

p = Point()

l = [["x", 0.5], ["y", 8.6], ["z", 9.9]]

i = 0

for field in fields(p):
    setattr(p, field.name, l[i][1])
    i = i + 1

# p = Point(item[1] for item in l)

print(p)  # Point(x=1.5, y=2.5, z=0.0)