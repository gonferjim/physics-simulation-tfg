from pygame.math import Vector2 as vec2
from Rigidbody import Rigidbody, ShapeType, AABB
import math

class CollisionManifold:
    def __init__(self, bodyA: Rigidbody, bodyB: Rigidbody, normal: vec2, depth: float, 
                 contact1: vec2, contact2: vec2, contactCount: int):
        self.bodyA = bodyA
        self.bodyB = bodyB
        self.normal = normal
        self.depth = depth
        self.contact1 = contact1
        self.contact2 = contact2
        self.contactCount = contactCount

class Collisions:

    @staticmethod
    def intersectTwoAABBs(a: AABB, b: AABB):
        if a.max.x < b.min.x or b.max.x < a.min.x or a.max.y < b.min.y or b.max.y < a.min.y: return False  
        return True

    @staticmethod
    def intersectTwoCircles(centerA: vec2, radiusA: float, centerB: vec2, radiusB: float):
        AToB = centerB - centerA
        depthIntersection = radiusA + radiusB - AToB.length() # How much are colliding, negative = not colliding
        
        return depthIntersection > 0, AToB.normalize(), depthIntersection
    
    @staticmethod
    def intersectCirclePolygon(centerCircle: vec2, radiusCircle: float, centerPoly: vec2, verticesPoly: list[vec2]):
        normal = vec2()
        minDepth = float('inf')

        # SEPARATING AXIS THEOREM (SAT)
        for i in range(len(verticesPoly)):
            edge = verticesPoly[(i+1) % len(verticesPoly)] - verticesPoly[i]
            axis = vec2(-edge.y, edge.x).normalize()

            minA,  maxA = Collisions.projectVertices(verticesPoly, axis)
            minB,  maxB = Collisions.projectCircle(centerCircle, radiusCircle, axis)
            if minA >= maxB or minB >= maxA: return False, None, None

            axisDepth = min(maxB - minA, maxA - minB)

            if axisDepth < minDepth: minDepth, normal = axisDepth, axis

        axis = (Collisions.findClosestPointOfPolygon(centerCircle, verticesPoly) - centerCircle).normalize()

        minA, maxA = Collisions.projectVertices(verticesPoly, axis)
        minB, maxB = Collisions.projectCircle(centerCircle, radiusCircle, axis)
        if minA >= maxB or minB >= maxA: return False, None, None

        axisDepth = min(maxB - minA, maxA - minB)

        if axisDepth < minDepth: minDepth, normal = axisDepth, axis

        if (centerPoly - centerCircle).dot(normal) < 0: normal = -normal # Normal must points out of the polygon center

        return True, normal, minDepth


    @staticmethod
    def intersectTwoPolygons(centerPolyA: vec2, verticesPolyA: list[vec2], centerPolyB: vec2, verticesPolyB: list[vec2]):
        normal = vec2()
        minDepth = float('inf')

        # SEPARATING AXIS THEOREM (SAT)
        for i in range(len(verticesPolyA)):
            edge = verticesPolyA[(i+1) % len(verticesPolyA)] - verticesPolyA[i]
            axis = vec2(-edge.y, edge.x).normalize()

            minA,  maxA = Collisions.projectVertices(verticesPolyA, axis)
            minB,  maxB = Collisions.projectVertices(verticesPolyB, axis)

            if minA >= maxB or minB >= maxA: return False, None, None

            axisDepth = min(maxB - minA, maxA - minB)

            if axisDepth < minDepth: minDepth, normal = axisDepth, axis
        
        for i in range(len(verticesPolyB)):
            edge = verticesPolyB[(i+1) % len(verticesPolyB)] - verticesPolyB[i]
            axis = vec2(-edge.y, edge.x).normalize()

            minA,  maxA = Collisions.projectVertices(verticesPolyA, axis)
            minB,  maxB = Collisions.projectVertices(verticesPolyB, axis)

            if minA >= maxB or minB >= maxA: return False, None, None
            
            axisDepth = min(maxB - minA, maxA - minB)
            if axisDepth < minDepth: minDepth, normal = axisDepth, axis

        if (centerPolyB - centerPolyA).dot(normal) < 0: normal = -normal # Normal must points out of the polygon center
        
        return True, normal, minDepth

    @staticmethod
    def findClosestPointOfPolygon(center: vec2, vertices: list[vec2]):
        minDist = float('inf')
        ret = vec2()

        for v in vertices:
            dist = center.distance_to(v)
            if dist < minDist:
                minDist = dist
                ret = v
        
        return ret
    
    @staticmethod
    def findContactPoints(bodyA: Rigidbody, bodyB: Rigidbody):
        cp2 = None
        contactCount = 0

        if bodyA.shape.shapeType == ShapeType.CIRCLE: 
            if bodyB.shape.shapeType == ShapeType.CIRCLE:
                # BOTH CIRCLES
                cp1 = Collisions.findContactPointTwoCircles(bodyA.position, bodyA.shape.radius, bodyB.position)
                contactCount = 1 
            else:
                # BODYA CIRCLE and BODYB POLYGON
                cp1 = Collisions.findContactPointCirclePolygon(bodyA.position, bodyA.shape.radius, bodyB.position, bodyB.getVertices())
                contactCount = 1      
        elif bodyB.shape.shapeType == ShapeType.CIRCLE:
            # BODYA POLYGON and BODYB CIRCLE
            cp1 = Collisions.findContactPointCirclePolygon(bodyB.position, bodyB.shape.radius, bodyA.position, bodyA.getVertices())
            contactCount = 1
        else:
            # BOTH POLYGONS
            cp1, cp2, contactCount = Collisions.findContactPointsTwoPolygons(bodyA.getVertices(), bodyB.getVertices())      

        return cp1, cp2, contactCount
    
    @staticmethod
    def findContactPointsTwoPolygons(verticesA: list[vec2], verticesB: list[vec2]):
        minDistSq = float('inf')
        contact2 = None

        for i in range(len(verticesA)):
            for j in range(len(verticesB)):
                v1 = verticesB[j]
                v2 = verticesB[(j+1) % len(verticesB)]

                cp, distSq = Collisions.pointSegmentDistance(verticesA[i], v1, v2)

                if Collisions.nearlyEqual(distSq, minDistSq):
                    if not Collisions.closeEnough(cp, contact1):
                        contact2 = cp
                        contactCount = 2
                elif distSq < minDistSq:
                    minDistSq = distSq
                    contact1 = cp
                    contactCount = 1
        
        for i in range(len(verticesB)):
            for j in range(len(verticesA)):
                v1 = verticesA[j]
                v2 = verticesA[(j+1) % len(verticesA)]

                cp, distSq = Collisions.pointSegmentDistance(verticesB[i], v1, v2)

                if Collisions.nearlyEqual(distSq, minDistSq):
                    if not Collisions.closeEnough(cp, contact1):
                        contact2 = cp
                        contactCount = 2
                elif distSq < minDistSq:
                    minDistSq = distSq
                    contact1 = cp
                    contactCount = 1

        return contact1, contact2, contactCount


    @staticmethod
    def findContactPointCirclePolygon(centerCircle: vec2, radiusCircle: float, centerPoly: vec2, verticesPoly: list[vec2]):
        minDistSq = float('inf')
        
        for i in range(len(verticesPoly)):
            va = verticesPoly[i]
            vb = verticesPoly[(i + 1) % len(verticesPoly)]

            contact, distSq = Collisions.pointSegmentDistance(centerCircle, va, vb)

            if distSq < minDistSq:
                minDistSq = distSq
                retContact = contact

        return retContact

    @staticmethod
    def findContactPointTwoCircles(centerA: vec2, radiusA: float, centerB: vec2):
        ab = centerB - centerA
        dir = ab.normalize()
        return centerA + dir * radiusA

    @staticmethod
    def pointSegmentDistance(p: vec2, a: vec2, b: vec2):
        ab = b - a
        ap = p - a
        proj = ap.dot(ab)
        abLenSq = ab.length_squared()
        d = proj / abLenSq

        if d <= 0: contact = a
        elif d >= 1: contact = b
        else: contact = a + ab * d

        return contact, p.distance_squared_to(contact)

    @staticmethod
    def projectCircle(center: vec2, radius: float, axis: vec2):
        direction = axis.normalize() * radius
        v1 = center + direction
        v2 = center - direction
        min = v1.dot(axis)
        max = v2.dot(axis)

        if min > max: min, max = max, min # swap if choosen min is actually the max
        
        return min, max

    @staticmethod
    def projectVertices(vertices: list[vec2], axis: vec2):
        min = float("inf")
        max = float("-inf")

        for v in vertices:
            proj = v.dot(axis)

            if proj < min: min = proj
            if proj > max: max = proj
        
        return min, max
    
    @staticmethod
    def collide(bodyA: Rigidbody, bodyB: Rigidbody):
        if bodyA.shape.shapeType == ShapeType.CIRCLE: 
            if bodyB.shape.shapeType == ShapeType.CIRCLE:
                # BOTH CIRCLES
                coll, normal, depth = Collisions.intersectTwoCircles(bodyA.position, bodyA.shape.radius, bodyB.position, bodyB.shape.radius)
            else:
                # BODYA CIRCLE and BODYB POLYGON
                coll, normal, depth = Collisions.intersectCirclePolygon(bodyA.position, bodyA.shape.radius, bodyB.position, bodyB.getVertices())
        elif bodyB.shape.shapeType == ShapeType.CIRCLE:
            # BODYA POLYGON and BODYB CIRCLE
            coll, normal, depth = Collisions.intersectCirclePolygon(bodyB.position, bodyB.shape.radius, bodyA.position, bodyA.getVertices())
            if coll: normal *= -1
        else:
            # BOTH POLYGONS
            coll, normal, depth = Collisions.intersectTwoPolygons(bodyA.position, bodyA.getVertices(), bodyB.position, bodyB.getVertices())         
        
        return coll, normal, depth
    
    @staticmethod
    def resolveCollisionBasic(contact: CollisionManifold):
        bodyA = contact.bodyA
        bodyB = contact.bodyB
        normal = contact.normal

        rest = (bodyA.restitution + bodyB.restitution) / 2
        relativeVel = bodyB.velocity - bodyA.velocity

        contactVelMag = relativeVel.dot(normal)

        # If someone how their velocities are pointing away but they are inside dont change velocities
        # That way we dont get weird interaction where they get push in one to another
        if contactVelMag > 0: return

        # Being static is like having infinite mass
        invMassA = 0 if bodyA.isStatic else (1 / bodyA.mass)
        invMassB = 0 if bodyB.isStatic else (1 / bodyB.mass)

        j = -(1 + rest) * contactVelMag / (invMassA + invMassB)

        bodyA.velocity -= j * normal * invMassA
        bodyB.velocity += j * normal * invMassB 

    @staticmethod
    def resolveCollisionWithRotation(contact: CollisionManifold):
        bodyA = contact.bodyA
        bodyB = contact.bodyB
        normal = contact.normal
        contactList = [contact.contact1, contact.contact2]
        impulseList = [vec2(), vec2()]
        raList = [vec2(), vec2()]
        rbList = [vec2(), vec2()]

        rest = (bodyA.restitution + bodyB.restitution) / 2

        # Being static is like having infinite mass
        invMassA = 0 if bodyA.isStatic else (1 / bodyA.mass)
        invMassB = 0 if bodyB.isStatic else (1 / bodyB.mass)

        invInertiaA = 0 if bodyA.isStatic else (1 / bodyA.rotationalInertia)
        invInertiaB = 0 if bodyB.isStatic else (1 / bodyB.rotationalInertia)

        for i in range(contact.contactCount):
            ra = contactList[i] - bodyA.position
            raList[i] = ra
            rb = contactList[i] - bodyB.position
            rbList[i] = rb

            raPerp = vec2(-ra.y, ra.x)
            rbPerp = vec2(-rb.y, rb.x)

            angularLinearVelA = raPerp * bodyA.angularVelocity
            angularLinearVelB = rbPerp * bodyB.angularVelocity

            relativeVel = (bodyB.velocity + angularLinearVelB) - (bodyA.velocity + angularLinearVelA)

            contactVelMag = relativeVel.dot(normal)

            # If someone how their velocities are pointing away but they are inside dont change velocities
            # That way we dont get weird interaction where they get push in one to another
            if contactVelMag > 0: continue

            raPerpDotN = raPerp.dot(normal)
            rbPerpDotN = rbPerp.dot(normal)

            j = -(1 + rest) * contactVelMag 
            
            j /= invMassA + invMassB + (raPerpDotN ** 2) * invInertiaA  + (rbPerpDotN ** 2) * invInertiaB
            
            j /= contact.contactCount

            impulseList[i] = j * normal


        for i in range(contact.contactCount):
            impulse = impulseList[i]

            bodyA.velocity -= impulse * invMassA
            bodyA.angularVelocity -= raList[i].cross(impulse) * invInertiaA
            bodyB.velocity += impulse * invMassB 
            bodyB.angularVelocity += rbList[i].cross(impulse) * invInertiaB

    @staticmethod
    def resolveCollisionWithRotationAndFriction(contact: CollisionManifold):
        bodyA = contact.bodyA
        bodyB = contact.bodyB
        normal = contact.normal
        contactList = [contact.contact1, contact.contact2]
        impulseList = [vec2(), vec2()]
        frictionImpulseList = [vec2(), vec2()]
        raList = [vec2(), vec2()]
        rbList = [vec2(), vec2()]
        jList = [0, 0]

        rest = (bodyA.restitution + bodyB.restitution) / 2
        sf = (bodyA.staticFriction + bodyB.staticFriction) / 2
        df = (bodyA.dynamicFriction + bodyB.dynamicFriction) / 2

        # Being static is like having infinite mass
        invMassA = 0 if bodyA.isStatic else (1 / bodyA.mass)
        invMassB = 0 if bodyB.isStatic else (1 / bodyB.mass)

        invInertiaA = 0 if bodyA.isStatic else (1 / bodyA.rotationalInertia)
        invInertiaB = 0 if bodyB.isStatic else (1 / bodyB.rotationalInertia)

        for i in range(contact.contactCount):
            ra = contactList[i] - bodyA.position
            raList[i] = ra
            rb = contactList[i] - bodyB.position
            rbList[i] = rb

            raPerp = vec2(-ra.y, ra.x)
            rbPerp = vec2(-rb.y, rb.x)

            angularLinearVelA = raPerp * bodyA.angularVelocity
            angularLinearVelB = rbPerp * bodyB.angularVelocity

            relativeVel = (bodyB.velocity + angularLinearVelB) - (bodyA.velocity + angularLinearVelA)
            contactVelMag = relativeVel.dot(normal)

            # If someone how their velocities are pointing away but they are inside dont change velocities
            # That way we dont get weird interaction where they get push in one to another
            if contactVelMag > 0: continue

            raPerpDotN = raPerp.dot(normal)
            rbPerpDotN = rbPerp.dot(normal)

            j = -(1 + rest) * contactVelMag         
            j /= invMassA + invMassB + (raPerpDotN ** 2) * invInertiaA  + (rbPerpDotN ** 2) * invInertiaB           
            j /= contact.contactCount

            jList[i] = j

            impulseList[i] = j * normal


        for i in range(contact.contactCount):
            impulse = impulseList[i]

            bodyA.velocity -= impulse * invMassA
            bodyA.angularVelocity -= raList[i].cross(impulse) * invInertiaA
            bodyB.velocity += impulse * invMassB
            bodyB.angularVelocity += rbList[i].cross(impulse) * invInertiaB

        for i in range(contact.contactCount):
            ra = contactList[i] - bodyA.position
            raList[i] = ra
            rb = contactList[i] - bodyB.position
            rbList[i] = rb

            raPerp = vec2(-ra.y, ra.x)
            rbPerp = vec2(-rb.y, rb.x)

            angularLinearVelA = raPerp * bodyA.angularVelocity
            angularLinearVelB = rbPerp * bodyB.angularVelocity

            relativeVel = (bodyB.velocity + angularLinearVelB) - (bodyA.velocity + angularLinearVelA)
            
            tangent = relativeVel - relativeVel.dot(normal) * normal

            if Collisions.closeEnough(tangent, vec2()): continue

            tangent = tangent.normalize()

            raPerpDotT = raPerp.dot(tangent)
            rbPerpDotT = rbPerp.dot(tangent)

            jt = -(relativeVel.dot(tangent))
            jt /= invMassA + invMassB + (raPerpDotT ** 2) * invInertiaA  + (rbPerpDotT ** 2) * invInertiaB
            jt /= contact.contactCount

            if abs(jt) <= jList[i] * sf:
                frictionImpulseList[i] = jt * tangent
            else:
                frictionImpulseList[i] = -jList[i] * tangent * df

        for i in range(contact.contactCount):
            frictionImpulse = frictionImpulseList[i]

            bodyA.velocity -= frictionImpulse * invMassA
            bodyA.angularVelocity -= raList[i].cross(frictionImpulse) * invInertiaA
            bodyB.velocity += frictionImpulse * invMassB
            bodyB.angularVelocity += rbList[i].cross(frictionImpulse) * invInertiaB


    @staticmethod
    def nearlyEqual(a: float, b: float):
        return abs(a - b) < 0.05
    
    @staticmethod
    def closeEnough(v1: vec2, v2: vec2):
        return (v2 - v1).length() < 0.05
