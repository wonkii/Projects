import sys
import pygame

from config import*
from water import Water1D
from sphere import Sphere
from render import draw_water, draw_spheres

# ============================
# 메인 루프
# ============================

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("1D Water Line + Falling Spheres (v2, Tuned)")

    clock = pygame.time.Clock()

    water = Water1D(NUM_POINTS)
    spheres = []

    running = True
    while running:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    # 전체 리셋
                    water = Water1D(NUM_POINTS)
                    spheres = []

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                gx = mx / float(CELL_SIZE)
                
                gz = (MID_Y - my) / Z_SCALE

                if event.button == 1:
                    spheres.append(Sphere(gx, gz))
                elif event.button == 3:
                    water.add_impulse(gx, strength=-0.2)

        for s in spheres:
            s.update(water)
        water.step()

        draw_water(screen, water)
        draw_spheres(screen, spheres, water)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
