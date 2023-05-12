from Rigidbody import *
from Collisions import Collisions, CollisionManifold

class World:
    airRes = 0.0005

    def __init__(self, screenSizes: tuple):
        self.entityList: list[Entity] = []
        self.springList: list[Spring] = []
        self.contactPairs: list[tuple[int]] = []
        # Because every pixel is like a meter, I will multiply gravity by 20 to make every 20 pixels a meter
        self.gravity = vec2(0, 9.81 * 20)
        self.screenSizes = screenSizes
        self.completionZone = (0, 0, 0, 0)

    def update(self, deltaTime: float, subIterations: int = 1):
        if subIterations < 1 or subIterations > 64: raise ValueError("subIterations argument must be between 1 and 64")
        subTime = deltaTime / subIterations
        
        for _ in range(int(subIterations)):
            self.contactPairs = []

            self.__broadPhace(subTime)
            self.__narrowPhace()

    def drawEntities(self, pantalla: Surface):
        draw.rect(pantalla, (0, 255, 0), self.completionZone)
        for e in self.entityList: e.draw(pantalla)
        for s in self.springList: s.draw(pantalla)
            
    def __narrowPhace(self):
        for contact in self.contactPairs:
            bodyA = self.entityList[contact[0]].body
            bodyB = self.entityList[contact[1]].body

            coll, normal, depth = Collisions.collide(bodyA, bodyB)
            if coll:                   
                self.__separateBodies(bodyA, bodyB, normal, depth)

                cp1, cp2, contactCount = Collisions.findContactPoints(bodyA, bodyB)
                collManifold = CollisionManifold(bodyA, bodyB, normal, depth, cp1, cp2, contactCount)

                Collisions.resolveCollisionWithRotationAndFriction(collManifold)

    def __broadPhace(self, deltaTime: float):
        for e in self.entityList:
            # GRAVITY
            if not e.body.isStatic: 
                e.body.applyForce(self.gravity * e.body.mass)

            # AIR RESISTANCE
            if not e.body.isStatic and e.body.velocity != vec2(0,0): 
                e.body.applyForce(self.airRes * -e.body.velocity.normalize() * e.body.velocity.length_squared())
            
            e.body.angularVelocity *= (1 - self.airRes * e.body.angularVelocity ** 2 * deltaTime) 

        for s in self.springList:
            s.applyTension()

        for i in range(len(self.entityList)):
            thisBody = self.entityList[i].body
            # UPDATE BODIES
            thisBody.update(deltaTime)

            for j in range(i + 1, len(self.entityList)):
                otherBody = self.entityList[j].body
                
                if thisBody.isStatic and otherBody.isStatic: continue

                thisAABB, otherAABB = thisBody.getAABB(), otherBody.getAABB()
                if not Collisions.intersectTwoAABBs(thisAABB, otherAABB): continue

                self.contactPairs.append((i, j))
                
    def __separateBodies(self, thisBody: Rigidbody, otherBody: Rigidbody, normal: vec2, depth: float):
        if thisBody.isStatic:
            otherBody.position += normal * depth
        elif otherBody.isStatic:
            thisBody.position -= normal * depth
        else:
            thisBody.position -= normal * depth/2
            otherBody.position += normal * depth/2
   
    """
    ======================================================================================================
    SCENE PREPARATION
    ======================================================================================================
    """

    def removeEntitiesOutOfScreen(self):
        listToRemove = []
        boundary = 50 # pixels

        for i in range(len(self.entityList)):
            pos = self.entityList[i].body.position
            if pos.x < -boundary or pos.y < -boundary or pos.x > self.screenSizes[0] + boundary or pos.y > self.screenSizes[1] + boundary:
                listToRemove.append(i)

        for index in listToRemove: del self.entityList[index]

    def setWalls(self):
        left = Rigidbody(vec2(0, self.screenSizes[1]/2), 1, 0.5, Shape.newBox(40, self.screenSizes[1]), True)
        right = Rigidbody(vec2(self.screenSizes[0], self.screenSizes[1]/2), 1, 0.5, Shape.newBox(40, self.screenSizes[1]), True)
        top = Rigidbody(vec2(self.screenSizes[0]/2, 0), 1, 0.5, Shape.newBox(self.screenSizes[0], 40), True)
        bottom = Rigidbody(vec2(self.screenSizes[0]/2, self.screenSizes[1]), 1, 0.5, Shape.newBox(self.screenSizes[0], 40), True)

        colorWall = (0, 0, 0)

        self.entityList.append(Entity(left, colorWall))
        self.entityList.append(Entity(right, colorWall))
        self.entityList.append(Entity(top, colorWall))
        self.entityList.append(Entity(bottom, colorWall))

    def setCompletionZone(self, pos: vec2, width, height):
        self.completionZone = (pos.x, pos.y, width, height)

    def setScene_1(self):
        baseRest = 0.25
        ground = Rigidbody(vec2(self.screenSizes[0]/2, self.screenSizes[1] - 100), 1, baseRest, Shape.newBox(600, 170), True)
        triangle = Rigidbody(vec2(self.screenSizes[0]/2, self.screenSizes[1] - 250), 1, baseRest, Shape.newTriangle(600, 200), True)

        self.entityList.append(Entity(ground, (0, 0, 0)))
        self.entityList.append(Entity(triangle, (0, 0, 0)))

        self.setCompletionZone(vec2(0, self.screenSizes[1] - 150), 200, 150)

    def setScene_2(self):
        baseRest = 0.25
        ground = Rigidbody(vec2(650, self.screenSizes[1] - 90), 1,baseRest, Shape.newBox(800, 150), True)
        triangle = Rigidbody(vec2(325, self.screenSizes[1] - 200), 1, baseRest, Shape.newTriangle(150, 100), True)
        topRight = Rigidbody(vec2(675, 80), 1, baseRest, Shape.newBox(700, 150), True)
        bouncyPanel = Rigidbody(vec2(20, 280), 1, 2, Shape.newCircle(50), True)

        self.entityList.append(Entity(ground, (0, 0, 0)))
        self.entityList.append(Entity(triangle, (0, 0, 0)))
        self.entityList.append(Entity(topRight, (0, 0, 0)))
        self.entityList.append(Entity(bouncyPanel, (200, 0, 0)))

        self.setCompletionZone(vec2(self.screenSizes[0] - 200, 300), 200, 150)