from __future__ import annotations
from enum import Enum
from pygame.math import Vector2 as vec2
from pygame import draw, Surface
import math

def getRotationalInertia(mass: float, shape: Shape) -> float:
        if shape.shapeType == ShapeType.CIRCLE:
            return (1 / 2) * mass * (shape.radius  ** 2)
        elif shape.shapeType == ShapeType.BOX: 
            return (1 / 12) * mass * (shape.width ** 2 + shape.height ** 2)
        elif shape.shapeType == ShapeType.PENTAGON:
            return (1 / 8) * mass * (shape.width ** 2 + shape.height ** 2)
        else:
            return (1 / 24) * mass * (shape.width ** 2 + shape.height ** 2)

class ShapeType(Enum):
    CIRCLE = 0
    BOX = 1
    TRIANGLE = 2
    PENTAGON = 3

"""
======================================================================================================
AABB CLASS 
======================================================================================================
"""

class AABB:
    def __init__(self, min: vec2, max:vec2):
        self.__min = vec2(min)
        self.__max = vec2(max)

    @property
    def min(self): return self.__min
    @property
    def max(self): return self.__max

    @min.setter
    def min(self, newVal): self.__min = vec2(newVal)
    @max.setter
    def max(self, newVal): self.__max = vec2(newVal)

"""
======================================================================================================
SHAPE CLASS 
======================================================================================================
"""

class Shape:
    def __init__(self, shape: ShapeType | Shape):
        self.__radius = None
        self.__width = None
        self.__height = None
        self.__shapeType = None

        if isinstance(shape, ShapeType): 
            self.__shapeType = shape
        elif isinstance(shape, Shape):
            self.__radius = shape.radius
            self.__width = shape.width
            self.__height = shape.height
            self.__shapeType = shape.shapeType
        else: raise TypeError("shape argument must be a Shape or ShapeType type")

    def __eq__(self, obj: object):
        if isinstance(obj, Shape):
            if self.__shapeType == ShapeType.CIRCLE:
                return self.radius == obj.radius
            else:
                return self.height == obj.height and self.width == obj.width
        
        return False
    
    # GETTERS ===========================================================================

    @property
    def radius(self): return self.__radius
    @property
    def width(self): return self.__width
    @property
    def height(self): return self.__height
    @property
    def shapeType(self): return self.__shapeType

    # SETTERS ===========================================================================

    @radius.setter
    def radius(self, newVal): self.__radius = int(newVal)
    @width.setter
    def width(self, newVal): self.__width = int(newVal)
    @height.setter
    def height(self, newVal): self.__height = int(newVal)
    @shapeType.setter
    def shapeType(self, newVal): self.__shapeType = ShapeType(newVal)

    # CREATORS ===========================================================================

    @staticmethod
    def newCircle(radius: int):
        if int(radius) <= 0: raise ValueError("radius must be greater than zero")

        shapeRet = Shape(ShapeType.CIRCLE)
        shapeRet.radius = int(radius)
        return shapeRet

    @staticmethod
    def newBox(width: int, height: int):
        if int(width) <= 0: raise ValueError("width must be greater than zero")
        if int(height) <= 0: raise ValueError("height must be greater than zero")
        
        shapeRet = Shape(ShapeType.BOX)
        shapeRet.width = int(width)
        shapeRet.height = int(height)
        return shapeRet
    
    @staticmethod
    def newTriangle(width: int, height: int):
        if int(width) <= 0: raise ValueError("width must be greater than zero")
        if int(height) <= 0: raise ValueError("height must be greater than zero")
        
        shapeRet = Shape(ShapeType.TRIANGLE)
        shapeRet.width = int(width)
        shapeRet.height = int(height)
        return shapeRet
    
    @staticmethod
    def newPentagon(width: int, height: int):
        if int(width) <= 0: raise ValueError("width must be greater than zero")
        if int(height) <= 0: raise ValueError("height must be greater than zero")
        
        shapeRet = Shape(ShapeType.PENTAGON)
        shapeRet.width = int(width)
        shapeRet.height = int(height)
        return shapeRet

""" 
======================================================================================================
RIGIDBODY CLASS 
======================================================================================================
"""

class Rigidbody:
    def __init__(self, vecPos: vec2, mass: float, restitution: float, shape: Shape, isStatic: bool):
        if float(mass) <= 0: raise ValueError("mass must be greater than zero")
        if float(restitution) < 0: raise ValueError("restitution must be greater or equal than zero")
        
        self.__position = vec2(vecPos)
        self.__velocity = vec2()

        self.__rotation = 0.0
        self.__angularVelocity = 0.0
        self.__rotationalInertia = getRotationalInertia(mass, shape)

        self.__staticFriction = 0.5
        self.__dynamicFriction = 0.4

        self.__force = vec2()
        
        self.__mass = float(mass)
        self.__restitution = float(restitution)
        self.__isStatic = bool(isStatic)

        self.__shape = Shape(shape)

        self.__vertices = []
        self.__AABB = []
        self.__updatedVertices = False
        self.__updatedAABB = False

    # GETTERS ===========================================================================  

    @property
    def position(self): return self.__position
    @property
    def velocity(self): return self.__velocity
    @property
    def rotation(self): return self.__rotation
    @property
    def angularVelocity(self): return self.__angularVelocity
    @property
    def rotationalInertia(self): return self.__rotationalInertia
    @property
    def staticFriction(self): return self.__staticFriction
    @property
    def dynamicFriction(self): return self.__dynamicFriction
    @property
    def force(self): return self.__force
    @property
    def isStatic(self): return self.__isStatic
    @property 
    def shape(self): return self.__shape
    @property
    def mass(self): return self.__mass
    @property
    def restitution(self): return self.__restitution

    def getVertices(self) -> list[vec2]:
        if self.__updatedVertices: return self.__vertices

        if self.__shape.shapeType == ShapeType.BOX:
            left = -self.__shape.width / 2
            right = left + self.__shape.width
            top = -self.__shape.height / 2
            bottom = top + self.__shape.height

            vs = [vec2(left, top), vec2(right, top), vec2(right, bottom), vec2(left, bottom)]

            ret = [self.position + v.rotate(self.rotation) for v in vs]

            self.__vertices = ret
            self.__updatedVertices = True

            return ret
        elif self.__shape.shapeType == ShapeType.TRIANGLE:
            left = -self.__shape.width / 2
            right = left + self.__shape.width
            top = -self.__shape.height / 3 * 2
            bottom = top + self.__shape.height

            vs = [vec2(0, top), vec2(right, bottom), vec2(left, bottom)]

            ret = [self.position + v.rotate(self.rotation) for v in vs]

            self.__vertices = ret
            self.__updatedVertices = True

            return ret
        elif self.__shape.shapeType == ShapeType.PENTAGON:
            top = -self.__shape.height / 2

            vs = [vec2(0, top).rotate(72 * x) for x in range(5)]

            ret = [self.position + v.rotate(self.rotation) for v in vs]

            self.__vertices = ret
            self.__updatedVertices = True

            return ret
        else: print("A circle has no vertices")

    def getAABB(self) -> AABB:
        if self.__updatedAABB: return self.__AABB

        if self.__shape.shapeType == ShapeType.CIRCLE:
            minX = self.position.x - self.shape.radius
            minY = self.position.y - self.shape.radius
            maxX = self.position.x + self.shape.radius
            maxY = self.position.y + self.shape.radius
        else:
            minX = float('inf')
            minY = float('inf')
            maxX = float('-inf')
            maxY = float('-inf')

            verts = self.getVertices()
            for v in verts:
                if v.x < minX: minX = v.x
                if v.x > maxX: maxX = v.x
                if v.y < minY: minY = v.y
                if v.y > maxY: maxY = v.y

        ret = AABB(vec2(minX, minY), vec2(maxX, maxY))
        self.__AABB = ret
        self.__updatedAABB = True

        return ret

    # SETTERS ===========================================================================

    @position.setter
    def position(self, newVal): 
        self.__position = vec2(newVal)
        self.__updatedVertices = False
        self.__updatedAABB = False

    @velocity.setter
    def velocity(self, newVal): self.__velocity = vec2(newVal)

    @rotation.setter
    def rotation(self, newVal):
        self.__rotation = float(newVal)
        self.__updatedVertices = False
        self.__updatedAABB = False

    @angularVelocity.setter
    def angularVelocity(self, newVal): self.__angularVelocity = float(newVal)
    @force.setter
    def force(self, newVal): self.__force = vec2(newVal)
    @isStatic.setter
    def isStatic(self, newVal): self.__isStatic = bool(newVal)

    @shape.setter
    def shape(self, newVal):
        self.__shape = Shape(newVal)
        self.__rotationalInertia = getRotationalInertia(self.mass, self.shape)

    @mass.setter
    def mass(self, newVal):
        if float(newVal) <= 0: raise ValueError("mass must be greater than zero")
        self.__mass = float(newVal)
        self.__rotationalInertia = getRotationalInertia(self.mass, self.shape)

    @restitution.setter
    def restitution(self, newVal):
        if float(newVal) < 0: raise ValueError("restitution must be greater or equal than zero")
        self.__restitution = float(newVal)

    # METHODS ===========================================================================

    def update(self, deltaTime: float):
        acceleration = self.force / self.mass

        radianToDegree = 180 / math.pi

        self.velocity += acceleration * float(deltaTime)
        self.position += self.velocity * float(deltaTime)
        self.rotation += self.angularVelocity * radianToDegree * float(deltaTime)

        self.force = vec2() # Reset force

    def applyForce(self, vecForce: vec2):
        self.force += vec2(vecForce)

"""
======================================================================================================
Entity CLASS 
======================================================================================================
"""

class Entity:
    def __init__(self, body: Rigidbody, color: tuple = (0, 0, 0)) -> None:
        if not isinstance(body, Rigidbody): raise TypeError("body argument must be a Rigidbody type")
        if not isinstance(color, tuple) or len(color) != 3: raise TypeError("color argument must be a length 3 tuple")

        self.body = body
        self.color = color

    def draw(self, pantalla: Surface):
        if self.body.shape.shapeType == ShapeType.CIRCLE:
            draw.circle(pantalla, self.color, self.body.position, self.body.shape.radius)
            draw.circle(pantalla, (0, 0, 0), self.body.position, self.body.shape.radius, 3)
            draw.line(pantalla, (0, 0, 0), self.body.position, self.body.position + vec2(self.body.shape.radius, 0).rotate(self.body.rotation), 2)
        else:
            verts = self.body.getVertices()
            draw.polygon(pantalla, self.color, verts)
            draw.polygon(pantalla, (0, 0, 0), verts, 3)

class Spring:
    ropeStiffness = 50000

    def __init__(self, bodyA: Rigidbody, bodyB: Rigidbody, steadyLen: int, stiffness: int = ropeStiffness) -> None:
        if not isinstance(bodyA, Rigidbody): raise TypeError("bodyA argument must be a Rigidbody type")
        if not isinstance(bodyB, Rigidbody): raise TypeError("bodyB argument must be a Rigidbody type")

        self.bodyA = bodyA
        self.bodyB = bodyB
        self.steadyLen = steadyLen
        self.stiffness = stiffness
    
    def applyTension(self):
        vecRope = self.bodyB.position - self.bodyA.position
        dirTension = vecRope.normalize()
        diffSteadyLen =  self.steadyLen - vecRope.length()

        # HOOKE's law:
        #   k = spring stiffness constant
        #   x = difference between the current and the "relaxed" or "steady" length
        # Fvector = DirFNorm * x * k
        vecTension = dirTension * diffSteadyLen * self.stiffness

        if not self.bodyA.isStatic and not self.bodyB.isStatic:
            self.bodyA.applyForce(-vecTension/2)
            self.bodyB.applyForce(vecTension/2)   
        elif not self.bodyA.isStatic:
            self.bodyA.applyForce(-vecTension)
        elif not self.bodyB.isStatic:
            self.bodyB.applyForce(vecTension)

    def draw(self, pantalla: Surface):
        draw.line(pantalla, (0, 0, 0), self.bodyA.position, self.bodyB.position, 2)

        