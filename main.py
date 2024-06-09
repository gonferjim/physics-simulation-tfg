import pygame, sys, time, random
from pygame.locals import *
from Rigidbody import *
from World import World
from Collisions import Collisions

# CONFIG ================================================================================

#pygame.init()  # Init all (plus audio pop)
pygame.display.init()
pygame.font.init() 

# Pantalla, titulo e icono
pantalla = pygame.display.set_mode((1000, 500))
icono = pygame.image.load("imagenes/icono.png")
pygame.display.set_caption('Physics test')
pygame.display.set_icon(icono)

# Fondo y texto
""" fondo = aygame.image.load("imagenes/fondo.jpg").convert()
fondo = pygame.transform.scale(fondo, SCREEN_SIZES) """
my_font = pygame.font.SysFont('Arial', 20, True)

# HELPING FUNC ===================================================================================

def tuple4toAABB(tupla4: tuple):
    initAABB = vec2(tupla4[0], tupla4[1])
    endAABB = vec2(tupla4[0], tupla4[1]) + vec2(tupla4[2], tupla4[3])
    return AABB(initAABB, endAABB)

# GAME PROGRESS ===================================================================================

currentLevel = 1
unlockedLevel = 1
selectedCircle = True
speedObject = 5

world = World()
world.setScene_1()

# MAIN LOOP ====================================================================================

lastTime = time.time() # seconds

while True:
    pygame.time.Clock().tick(120)
    pygame.display.update()

    # Time Management

    currentTime = time.time()
    dt = currentTime - lastTime
    lastTime = currentTime

    # Background

    pantalla.fill((230, 230, 255))

    # World Update

    if world.currentLevel != currentLevel:
        world.changeScene(currentLevel)

    world.removeEntitiesOutOfScreen()

    world.update(dt, 10)
    world.drawEntities(pantalla)

    if unlockedLevel == currentLevel and world.completionEntityIndexes:
        for i in world.completionEntityIndexes:
            if Collisions.intersectTwoAABBs(world.entityList[i].body.getAABB(), tuple4toAABB(world.completionZone)):
                unlockedLevel += 1

    # GUI

    prevSceneButton = pygame.Rect(25, 20, 160, 40)
    nextSceneButton = pygame.Rect(210, 20, 160, 40)
    restartSceneButton = pygame.Rect(400, 20, 90, 40)
    pSBColor = (255, 255, 0)
    nSBColor = (255, 255, 0)
    rSBColor = (255, 255, 0)

    mX, mY = pygame.mouse.get_pos()

    if prevSceneButton.collidepoint((mX, mY)):
        pSBColor = (255, 150, 0)
        if leftClick and currentLevel > 1:
            currentLevel -= 1
            selectedCircle = True

    if unlockedLevel > currentLevel and nextSceneButton.collidepoint((mX, mY)):
        nSBColor = (255, 150, 0)
        if leftClick and currentLevel < 8:
            currentLevel += 1
            selectedCircle = True
    
    # RESET BUTTON
    if restartSceneButton.collidepoint((mX, mY)):
        rSBColor = (255, 150, 0)
        if leftClick:
            world.changeScene(currentLevel)
            selectedCircle = True
            speedObject = 5
    
    if currentLevel > 1:
        pygame.draw.rect(pantalla, pSBColor, prevSceneButton)
        pantalla.blit(my_font.render('Nivel Anterior', False, (0, 0, 0)), (35, 30))
    
    if currentLevel < 7:
        pygame.draw.rect(pantalla, nSBColor, nextSceneButton)
        pantalla.blit(my_font.render('Siguiente Nivel', False, (0, 0, 0)), (220, 30))

    pygame.draw.rect(pantalla, rSBColor, restartSceneButton)
    pantalla.blit(my_font.render('Restart', False, (0, 0, 0)), (410, 30))
    
    if unlockedLevel > currentLevel:
        pantalla.blit(my_font.render('¡Enhorabuena! Nivel Completado', False, (255, 150, 0)), (520, 30))

    # Special Level 2
    if currentLevel == 2:
        speedUPButton = pygame.Rect(940, 15, 35, 20)
        speedDOWNButton = pygame.Rect(940, 42, 35, 20)
        sUPColor = (0, 255, 255)
        sDOWNColor = (0, 255, 255)

        if speedUPButton.collidepoint((mX, mY)):
            sUPColor = (0, 190, 190)
            if leftClick and speedObject < 10:
                speedObject += 1

        if speedDOWNButton.collidepoint((mX, mY)):
            sDOWNColor = (0, 190, 190)
            if leftClick and speedObject > 0:
                speedObject -= 1

        string = "Vel: "+str(speedObject)

        pygame.draw.rect(pantalla, sUPColor, speedUPButton)
        pantalla.blit(my_font.render("+", False, (0, 0, 0)), (950, 15))
        pygame.draw.rect(pantalla, sDOWNColor, speedDOWNButton)
        pantalla.blit(my_font.render("-", False, (0, 0, 0)), (952, 40))
        pantalla.blit(my_font.render(string, False, (0, 255, 255)), (860, 30))

    # Special Level 6
    if currentLevel == 6 and len(world.usableObjects) > 1:
        changeObjectButton = pygame.Rect(850, 20, 120, 40)
        cOBColor = (0, 255, 255)

        if changeObjectButton.collidepoint((mX, mY)):
            cOBColor = (0, 190, 190)
            if leftClick:
                save = world.usableObjects.pop(0)
                world.usableObjects.append(save)
                selectedCircle = not selectedCircle

        string = "Circulo"
        if not selectedCircle: string = "Cuadrado"

        pygame.draw.rect(pantalla, cOBColor, changeObjectButton)
        pantalla.blit(my_font.render(string, False, (0, 0, 0)), (860, 30))

    # Special Level 7
    if currentLevel == 7 and world.usableObjects:
        changeObjectButton = pygame.Rect(850, 20, 120, 40)
        cOBColor = (0, 255, 255)

        if changeObjectButton.collidepoint((mX, mY)) and len(world.usableObjects) > 1:
            cOBColor = (0, 190, 190)
            if leftClick:
                save = world.usableObjects.pop(0)
                world.usableObjects.append(save)
                selectedCircle = not selectedCircle

        string = "Circulo"
        if not selectedCircle: string = "Triangulo"

        pygame.draw.rect(pantalla, cOBColor, changeObjectButton)
        pantalla.blit(my_font.render(string, False, (0, 0, 0)), (860, 30))

    
    # Inputs Processing

    leftClick = False

    for event in pygame.event.get():   

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
            mousePos = pygame.mouse.get_pos()
            if Collisions.intersectTwoAABBs(AABB(mousePos, mousePos), tuple4toAABB(world.playableZone)):
                if currentLevel == 2:
                    world.addUsableObjectAtPos(0, mousePos, vec2(0, 200*speedObject))
                elif currentLevel == 6:
                    if len(world.usableObjects) > 1:
                        world.addUsableObjectAtPos(0, mousePos)
                else:
                    world.addUsableObjectAtPos(0, mousePos)
                
                selectedCircle = not selectedCircle

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            leftClick = True

        if event.type == QUIT:
            pygame.quit()
            sys.exit()

