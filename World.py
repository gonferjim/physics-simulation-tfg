import random
from Rigidbody import *
from Collisions import Collisions, CollisionManifold

class World:
    airRes = 0.0005

    def __init__(self):
        self.entityList: list[Entity] = []
        self.springList: list[Spring] = []
        self.contactPairs: list[tuple[int]] = []
        # Because every pixel is like a meter, I will multiply gravity by 20 to make every 20 pixels a meter
        self.gravity = vec2(0, 9.81 * 10)
        self.completionZone = [0, 0, 0, 0]
        self.playableZone = [0, 0, 0, 0]
        self.usableObjects: list[Entity] = []
        self.completionEntityIndexes: list[int] = []
        self.currentLevel = 1

    def update(self, deltaTime: float, subIterations: int = 1):
        if subIterations < 1 or subIterations > 64: raise ValueError("subIterations argument must be between 1 and 64")
        subTime = deltaTime / subIterations
        
        for _ in range(int(subIterations)):
            self.contactPairs = []

            self.__broadPhace(subTime)
            self.__narrowPhace()
        
        # COMPLETION ZONE MOVABLE
        if self.currentLevel == 1:
            if self.completionZone[0] > 1000: self.completionZone[0] = -200
            self.completionZone[0] += 3
        
        if self.currentLevel == 2:
            if self.completionZone[1] > 500: self.completionZone[1] = -50
            self.completionZone[1] += 2

    def drawEntities(self, pantalla: Surface):
        draw.rect(pantalla, (0, 150, 0), self.completionZone)
        draw.rect(pantalla, (255, 255, 120), self.playableZone)
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

                Collisions.resolveCollisionWithRotation(collManifold)

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
            if pos.x < -boundary or pos.y < -boundary or pos.x > 1000 + boundary or pos.y > 500 + boundary:
                listToRemove.append(i)

        for index in listToRemove: del self.entityList[index]

    def setWalls(self):
        left = Rigidbody(vec2(0, 250), 1, 0.5, Shape.newBox(40, 500), True)
        right = Rigidbody(vec2(1000, 250), 1, 0.5, Shape.newBox(40, 500), True)
        top = Rigidbody(vec2(500, 0), 1, 0.5, Shape.newBox(1000, 150), True)
        bottom = Rigidbody(vec2(500, 500), 1, 0.5, Shape.newBox(1000, 40), True)

        colorWall = (0, 0, 0)

        self.entityList.append(Entity(left, colorWall))
        self.entityList.append(Entity(right, colorWall))
        self.entityList.append(Entity(top, colorWall))
        self.entityList.append(Entity(bottom, colorWall))

    def initScene(self):
        self.entityList.clear()
        self.springList.clear()
        self.usableObjects.clear()
        self.completionEntityIndexes.clear()
        self.setWalls()

    def setPlayableZone(self, pos: vec2, width, height):
        self.playableZone = [pos.x, pos.y, width, height]

    def addUsableObjectAtPos(self, index: int, pos: vec2, vel: vec2 = vec2(0,0)):
        if index < len(self.usableObjects) and not self.usableObjects[index] in self.entityList:
            self.usableObjects[index].body.position = pos
            self.usableObjects[index].body.velocity = vel
            self.usableObjects[index].body.angularVelocity = 0

            self.completionEntityIndexes.append(len(self.entityList))
            self.entityList.append(self.usableObjects[index])
  
            self.usableObjects.pop(index)

    """
    ======================================================================================================
    SCENE CREATIONS
    ======================================================================================================
    """

    def setScene_1(self):
        self.initScene()
        baseRest = 0.25

        #ground = Rigidbody(vec2(500, 400), 1, baseRest, Shape.newBox(600, 170), True)
        triangle = Rigidbody(vec2(500, 250), 1, baseRest, Shape.newTriangle(400, 150), True)

        #self.entityList.append(Entity(ground, (0, 0, 0)))
        self.entityList.append(Entity(triangle, (0, 0, 0)))

        self.completionZone = [0, 350, 200, 50]
        self.setPlayableZone(vec2(400, 0), 200, 300)

        newBody1 = Rigidbody(vec2(), 1, 0.25, Shape.newCircle(20), False)
        self.usableObjects.append(Entity(newBody1, (250,0,0)))

    def setScene_2(self):
        self.initScene()
        baseRest = 0.25

        ground = Rigidbody(vec2(250, 500), 1, baseRest, Shape.newBox(500, 200), True)
        ground2 = Rigidbody(vec2(900, 470), 1, baseRest, Shape.newBox(1000, 40), True)
        wallL = Rigidbody(vec2(150, 250), 1, baseRest, Shape.newBox(300, 500), True)
        rampL1 = Rigidbody(vec2(300, 350), 1, baseRest, Shape.newTriangle(100, 200), True)
        rampL2 = Rigidbody(vec2(330, 400), 1, baseRest, Shape.newTriangle(100, 100), True)
        rampL3 = Rigidbody(vec2(345, 400), 1, baseRest, Shape.newTriangle(100, 50), True)
        rampL4 = Rigidbody(vec2(345, 410), 1, baseRest, Shape.newTriangle(200, 50), True)
        
        rampR1 = Rigidbody(vec2(470, 405), 1, baseRest, Shape.newTriangle(50, 60), True)
        rampR2 = Rigidbody(vec2(485, 400), 1, baseRest, Shape.newTriangle(50, 50), True)
        
        ground2.rotation -= 2
        rampR1.rotation += 45
        rampR2.rotation += 28

        self.entityList.append(Entity(ground, (0, 0, 0)))
        self.entityList.append(Entity(ground2, (0, 0, 0)))
        self.entityList.append(Entity(wallL, (0, 0, 0)))
        self.entityList.append(Entity(rampL1, (0, 0, 0)))
        self.entityList.append(Entity(rampL2, (0, 0, 0)))
        self.entityList.append(Entity(rampL3, (0, 0, 0)))
        self.entityList.append(Entity(rampL4, (0, 0, 0)))
        self.entityList.append(Entity(rampR1, (0, 0, 0)))
        self.entityList.append(Entity(rampR2, (0, 0, 0)))

        self.completionZone = [960, 0, 20, 100]
        self.playableZone = [300, 0, 50, 400]

        newBody1 = Rigidbody(vec2(), 1, 0.25, Shape.newCircle(20), False)
        self.usableObjects.append(Entity(newBody1, (250,0,0)))

    def setScene_3(self):
        self.initScene()
        baseRest = 0.25

        pillar1 = Rigidbody(vec2(500 - 35, 400), 1, baseRest, Shape.newBox(10, 200), True)
        pillar2 = Rigidbody(vec2(500 + 35, 400), 1, baseRest, Shape.newBox(10, 200), True)
        
        roof = Rigidbody(vec2(500, 200), 1, baseRest, Shape.newTriangle(120, 100), False)

        self.entityList.append(Entity(pillar1, (0, 0, 0)))
        self.entityList.append(Entity(pillar2, (0, 0, 0)))
        self.entityList.append(Entity(roof, (180, 0, 0)))

        self.completionZone = [460, 400, 70, 100]
        self.setPlayableZone(vec2(400, 0), 200, 200)

        newBody1 = Rigidbody(vec2(), 5, 0.25, Shape.newCircle(20), False)
        newBody2 = Rigidbody(vec2(), 1, 0.25, Shape.newCircle(20), False)
        self.usableObjects.append(Entity(newBody1, (250,0,0)))
        self.usableObjects.append(Entity(newBody2, (250,0,0)))

    def setScene_4(self):
        self.initScene()
        baseRest = 0.25

        ground = Rigidbody(vec2(500 + 150, 500 - 75), 1, baseRest, Shape.newBox(700, 150), True)
    
        base = Rigidbody(vec2(500, 150), 1, baseRest, Shape.newCircle(2), True)
        ball1 = Rigidbody(vec2(650, 150), 10, baseRest, Shape.newCircle(20), False)
        spring1 = Spring(base, ball1)

        self.entityList.append(Entity(ground, (0, 0, 0)))
        self.entityList.append(Entity(ball1, (0, 0, 160)))
        self.springList.append(spring1)

        self.completionZone = [0, 400, 400, 200]
        self.setPlayableZone(vec2(300, 0), 400, 200)

        newBody1 = Rigidbody(vec2(), 1, 0.25, Shape.newCircle(20), False)
        self.usableObjects.append(Entity(newBody1, (250,0,0)))

    def setScene_5(self):
        self.initScene()
        baseRest = 0.25

        ground = Rigidbody(vec2(650, 500 - 90), 1,baseRest, Shape.newBox(800, 150), True)
        triangle = Rigidbody(vec2(325, 500 - 190), 1, baseRest, Shape.newTriangle(150, 70), True)
        topRight = Rigidbody(vec2(675, 80), 1, baseRest, Shape.newBox(700, 150), True)
        bouncyPanel = Rigidbody(vec2(20, 280), 1, 2, Shape.newCircle(50), True)

        self.entityList.append(Entity(ground, (0, 0, 0)))
        self.entityList.append(Entity(triangle, (0, 0, 0)))
        self.entityList.append(Entity(topRight, (0, 0, 0)))
        self.entityList.append(Entity(bouncyPanel, (200, 0, 0)))

        self.completionZone = [600, 250, 400, 200]
        self.setPlayableZone(vec2(0, 0), 325, 300)

        newBody1 = Rigidbody(vec2(), 3, 0.25, Shape.newCircle(20), False)
        self.usableObjects.append(Entity(newBody1, (250,0,0)))

    def setScene_6(self):
        self.initScene()
        baseRest = 0.25

        ground = Rigidbody(vec2(600, 520), 1, baseRest, Shape.newBox(300, 200), True)
        ramp = Rigidbody(vec2(280, 240), 1, baseRest, Shape.newBox(620, 30), True)

        ground.rotation -= 30
        ramp.rotation += 20

        self.entityList.append(Entity(ground, (0, 0, 0)))
        self.entityList.append(Entity(ramp, (0, 0, 0)))

        self.completionZone = [0, 400, 700, 150]
        self.setPlayableZone(vec2(0, 0), 300, 150)

        newBody1 = Rigidbody(vec2(), 1, 0.25, Shape.newCircle(20), False)
        newBody2 = Rigidbody(vec2(), 1, 0.25, Shape.newBox(40, 40), False)
        self.usableObjects.append(Entity(newBody1, (250,0,0)))
        self.usableObjects.append(Entity(newBody2, (250,0,0)))

    def setScene_7(self):
        self.initScene()
        baseRest = 0.25

        ground = Rigidbody(vec2(150, 450), 1, baseRest, Shape.newBox(620, 500), True)
        barrier = Rigidbody(vec2(500, 500 - 100), 1, baseRest, Shape.newBox(230, 15), True)

        ground.rotation += 30
        barrier.rotation -= 60

        self.entityList.append(Entity(ground, (0, 0, 0)))
        self.entityList.append(Entity(barrier, (0, 0, 0)))

        self.completionZone= [400, 400, 600, 150]
        self.setPlayableZone(vec2(0, 0), 300, 150)

        newBody1 = Rigidbody(vec2(), 1, 0.25, Shape.newCircle(20), False)
        newBody2 = Rigidbody(vec2(400, 200), 1, 0.25, Shape.newTriangle(120, 60), False)
        newBody2.rotation += 140
        self.usableObjects.append(Entity(newBody1, (250,0,0)))
        self.usableObjects.append(Entity(newBody2, (250,0,0)))

    def changeScene(self, sceneNum: int):
        if sceneNum <= 8 and sceneNum >= 1:
            self.currentLevel = sceneNum
        else: return

        if sceneNum == 1: self.setScene_1()   
        elif sceneNum == 2: self.setScene_2()
        elif sceneNum == 3: self.setScene_3()
        elif sceneNum == 4: self.setScene_4()
        elif sceneNum == 5: self.setScene_5()
        elif sceneNum == 6: self.setScene_6()
        elif sceneNum == 7: self.setScene_7()
        #elif sceneNum == 8: self.setScene_8()
        
        
