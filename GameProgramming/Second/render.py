import pygame

from config import*
from water import Water1D

# ============================
# 렌더링 함수
# ============================

def draw_water(screen, water: Water1D):
    # 배경
    screen.fill((10, 10, 30))

    # 기준 수면(높이 0인 직선)
    pygame.draw.line(screen, (40, 40, 80),
                     (0, MID_Y), (SCREEN_WIDTH, MID_Y), 2)

    # 수면 실제 위치 (파동 포함)를 polyline으로 그림
    water_points = []
    for i in range(water.n):
        x = i * CELL_SIZE + CELL_SIZE * 0.5
        z = water.h[i]  # 기준에서 변위
        y = MID_Y - z * Z_SCALE
        water_points.append((int(x), int(y)))

    if len(water_points) >= 2:
        # 물 영역을 채우고 싶으면 polygon으로 아래쪽 채우기
        poly = [(0, MID_Y)] + water_points + [(SCREEN_WIDTH, MID_Y)]
        pygame.draw.polygon(screen, (0, 0, 80), poly)

        # 수면 선
        pygame.draw.lines(screen, (0, 130, 255), False, water_points, 3)


def draw_spheres(screen, spheres, water: Water1D):
    for s in spheres:
        sx, sy = s.get_screen_pos()

        # 색상: 물에 잠긴 정도에 따라 살짝 다르게
        water_h = water.sample_height(s.x)
        bottom_z = s.z - SPHERE_RADIUS
        depth = water_h - bottom_z

        if depth > 0:
            t = max(0.0, min(1.0, depth / (2.0 * SPHERE_RADIUS)))
            color = (int(255 * (1.0 - t)), int(90 * (1.0 - t)), 255)
        else:
            color = (255, 200, 0)

        radius_px = int(SPHERE_RADIUS * Z_SCALE)
        pygame.draw.circle(screen, color, (sx, sy), radius_px)
