from pygame.math import Vector2 as vec2
import time
from Rigidbody import Shape, Rigidbody
from Collisions import Collisions

vec = vec2(1,0)
vec.x -= 1.0

print(vec == vec2(0,0))

