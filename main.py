import pygame, sys, time, random
from pygame.locals import *
from Rigidbody import *
from World import World

#pygame.init()  # Init all (plus audio pop)
pygame.display.init()
pygame.font.init() 

# config
SCREEN_SIZES = (1000, 600)
FPS = 120

# Pantalla, titulo e icono
pantalla = pygame.display.set_mode(SCREEN_SIZES)
icono = pygame.image.load("imagenes/icono.png")
pygame.display.set_caption('Physics test')
pygame.display.set_icon(icono)

# Fondo y texto
""" fondo = pygame.image.load("imagenes/fondo.jpg").convert()
fondo = pygame.transform.scale(fondo, SCREEN_SIZES) """
my_font = pygame.font.SysFont('Comic Sans MS', 15)

# WORLD

world = World(SCREEN_SIZES)

world.setScene_2()
world.setWalls()

def randomColor():
    return (random.randrange(30, 180), random.randrange(30, 180), random.randrange(30, 180))

lastTime = time.time() # seconds

while True:
    pygame.time.Clock().tick(FPS)
    pygame.display.update()

    # TIME MANAGEMENT
    currentTime = time.time()
    dt = currentTime - lastTime
    lastTime = currentTime

    # BACKGROUND AND TEXT

    pantalla.fill((230, 230, 255))
    pantalla.blit(my_font.render(f'Bodies: {len(world.entityList)}', False, (0, 0, 0)), (30, 30))

    # WORLD UPDATE

    world.removeEntitiesOutOfScreen()

    world.update(dt, 10)
    world.drawEntities(pantalla)

    # INPUTS PROCESS

    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONDOWN and (event.button == 1 or event.button == 3):
            if event.button == 1:
                base = random.randrange(40, 60)
                shape = Shape.newBox(base, base)
            elif event.button == 3:
                shape = Shape.newCircle(random.randrange(20, 30))
                
            newBody = Rigidbody(pygame.mouse.get_pos(), 1, 0.25, shape, False)
            world.entityList.append(Entity(newBody, randomColor()))
        
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 2:
            if world.springList: del world.springList[-1]

        if event.type == QUIT:
            pygame.quit()
            sys.exit()

