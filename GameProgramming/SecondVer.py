import sys
import math
import numpy as np
import pygame

# ============================
# 기본 설정
# ============================

NUM_POINTS = 200          # 수면을 이루는 점 개수 (X 방향 분해능)
CELL_SIZE = 4             # 점 사이 화면 간격 (픽셀)

SCREEN_WIDTH = NUM_POINTS * CELL_SIZE
SCREEN_HEIGHT = 600
MID_Y = SCREEN_HEIGHT // 2  # 화면 가운데 수평선 = 기준 수면 높이

# 월드 좌표계에서 z(위/아래)를 화면 픽셀로 바꿀 때의 스케일
Z_SCALE = 30.0  # 1 단위 높이 = 30 픽셀 (예전 40보다 줄여서 시각적으로도 안정)

# 수면 높이 클램프 (수면이 이 범위를 넘지 않게 제한)
MAX_WATER_HEIGHT = 8.0  # h in [-3, 3] 안으로 제한



# ============================
# 물(1D Height Field) 물리 파라미터
# ============================

DX = 1.0           # 수평 격자 간격
DT = 0.03          # 시간 스텝
C = 10.0           # 파동 속도 (조금 줄임: 수면이 너무 빠르게 안 튀도록)
BASE_DAMPING = 0.01  # 감쇠 살짝 증가: 파동이 더 얌전히 죽도록

C2_DT2_DX2 = (C * C) * (DT * DT) / (DX * DX)

BOUNDARY_THICKNESS = 8     # 양 끝에서부터 몇 개의 점을 흡수 영역으로 볼지
MAX_BOUNDARY_DAMPING = 0.3 # 경계 쪽에서 추가로 더해질 감쇠 계수

IMPULSE_RADIUS = 4.0       # 강체가 줄 수면 교란 반경 (격자 index 단위)


# ============================
# 강체(구) 물리 파라미터
# ============================

SPHERE_RADIUS = 0.6   # 구 반지름 (월드 z축 단위)
SPHERE_MASS = 0.5

# 중력 / 부력 / 항력 계수를 낮춰서 수면 시간 스케일과 비슷하게 맞춤
GRAVITY = 1.5             # 예전보다 훨씬 작은 중력
BUOYANCY_STRENGTH = 8.0   # 부력 계수
DRAG_COEFF = 4.0          # 수중 항력

# 강체 <-> 수면 커플링 조정용
BUOY_HEIGHT_SMOOTH = 1.0        # 공이 느끼는 수면 높이 필터 강도 (0~1)
MAX_BUOY_DEPTH = 0.7 * SPHERE_RADIUS  # 부력 계산에 쓸 최대 잠김 깊이

# 수면에 주는 임펄스도 약하게 조정
IMPULSE_SCALE = 0.08      # 강체가 수면을 누를 때 생성하는 파동 세기

# 수면 '접촉' 모델 파라미터 (새로 추가)
MAX_PENETRATION = 0.3 * SPHERE_RADIUS   # 공이 수면 아래로 평형 시 허용할 최대 잠김 깊이
SURFACE_RESTITUTION = 0.2              # 튕길 때 속도 보존 비율 (0~1)

# ============================
# 물 Height Field (1D)
# ============================

class Water1D:
    def __init__(self, n_points):
        self.n = n_points
        self.h = np.zeros(self.n, dtype=np.float32)       # 현재 높이
        self.h_prev = np.zeros(self.n, dtype=np.float32)  # 이전 높이
        self.h_next = np.zeros(self.n, dtype=np.float32)  # 계산용 버퍼

        self.alpha = np.zeros(self.n, dtype=np.float32)   # 경계 흡수용 감쇠 계수
        self._init_boundary_damping()

    def _init_boundary_damping(self):
        # 경계에서 중앙으로 갈수록 0으로 줄어드는 감쇠 계수
        for i in range(self.n):
            dist_left = i
            dist_right = self.n - 1 - i
            dist_edge = min(dist_left, dist_right)

            if dist_edge < BOUNDARY_THICKNESS:
                t = 1.0 - (dist_edge / float(BOUNDARY_THICKNESS))
                self.alpha[i] = t * MAX_BOUNDARY_DAMPING
            else:
                self.alpha[i] = 0.0

    def step(self):
        """1D 파동 방정식 이산화로 한 스텝 진행."""
        h = self.h
        h_prev = self.h_prev
        h_next = self.h_next

        # 내부 점 업데이트 (양 끝 0, n-1은 경계로 취급)
        for i in range(1, self.n - 1):
            lap = h[i + 1] + h[i - 1] - 2.0 * h[i]
            local_damping = BASE_DAMPING + self.alpha[i]

            h_next[i] = (
                (2.0 - local_damping * DT) * h[i]
                - (1.0 - local_damping * DT) * h_prev[i]
                + C2_DT2_DX2 * lap
            )

        # 경계는 흡수: 높이를 0으로 고정
        h_next[0] = 0.0
        h_next[-1] = 0.0

        # 수면 높이 클램프 (폭주 방지 & 화면 밖으로 나가는 것 방지)
        np.clip(h_next, -MAX_WATER_HEIGHT, MAX_WATER_HEIGHT, out=h_next)

        # 버퍼 스왑
        self.h_prev, self.h, self.h_next = self.h, self.h_next, self.h_prev

    def sample_height(self, x):
        """
        연속 좌표 x (0 ~ n-1) 에서 선형 보간으로 높이 샘플링.
        이 값이 '기준 수면(0)'에서 얼마나 위/아래로 변위됐는지.
        """
        x = max(0.0, min(self.n - 1.001, x))
        i0 = int(math.floor(x))
        i1 = min(i0 + 1, self.n - 1)
        t = x - i0
        return self.h[i0] * (1.0 - t) + self.h[i1] * t

    def add_impulse(self, x, strength, radius=IMPULSE_RADIUS):
        """
        x 주변에 로컬 교란을 주어 파동 생성.
        strength > 0 이면 위로, < 0 이면 아래로 밀어냄.
        """
        cx = x
        r = radius

        i_min = max(1, int(math.floor(cx - r)))
        i_max = min(self.n - 2, int(math.ceil(cx + r)))

        for i in range(i_min, i_max + 1):
            dx = i - cx
            dist = abs(dx)
            if dist < r:
                w = 1.0 - (dist / r)
                self.h[i] += strength * w

        # 임펄스를 준 뒤에도 높이를 클램프
        np.clip(self.h, -MAX_WATER_HEIGHT, MAX_WATER_HEIGHT, out=self.h)


# ============================
# 강체(구)
# ============================

class Sphere:
    def __init__(self, x, z):
        self.x = float(x)
        self.z = float(z)
        self.vz = 0.0

        # 공이 힘 계산에 쓸 "부드러운 수면 높이"
        self.water_h_for_force = 0.0

    def update(self, water: Water1D):
        # 현재 위치에서 수면 높이 샘플
        water_h_raw = water.sample_height(self.x)

        # 2) 공이 "힘 계산용"으로 쓸 높이는 부드럽게 보간
        self.water_h_for_force += BUOY_HEIGHT_SMOOTH * (water_h_raw - self.water_h_for_force)
        water_h = self.water_h_for_force
        
        # 3) 구의 아래쪽 점 위치
        bottom_z = self.z - SPHERE_RADIUS
        

        # 실제 잠김 깊이 (지오메트리 기준)
        depth_raw = water_h_raw - bottom_z
        # 부력에 쓸 깊이는 살짝 클램프
        depth_for_buoy = max(0.0, min(depth_raw, MAX_BUOY_DEPTH))

        # 힘 계산
        F_gravity = -SPHERE_MASS * GRAVITY
        F_buoy = 0.0
        F_drag = 0.0

        if depth_raw > 0.0:
            # 부력: 클램프된 깊이 사용
            F_buoy = BUOYANCY_STRENGTH * depth_for_buoy

            # 잠긴 비율은 실제 깊이 기준 (수중 항력, 임펄스에는 그대로 반영)
            submergence = max(0.0, min(1.0, depth_raw / (2.0 * SPHERE_RADIUS)))

            # 수중 항력
            F_drag = -DRAG_COEFF * submergence * self.vz

            # 아래로 들어갈 때 수면에 임펄스
            if self.vz < 0.0:
                impulse = IMPULSE_SCALE * (-self.vz) * submergence
                water.add_impulse(self.x, -impulse)

        Fz = F_gravity + F_buoy + F_drag

        az = Fz / SPHERE_MASS
        self.vz += az * DT
        self.z += self.vz * DT

        # 너무 아래로 떨어지는 것 방지용 바닥:
        # 수면이 내려갈 수 있는 최소 높이보다 조금만 더 아래까지만 허용
        min_z = -MAX_WATER_HEIGHT - SPHERE_RADIUS * 0.5
        if self.z < min_z:
            self.z = min_z
            if self.vz < 0.0:
                self.vz *= -0.2  # 아주 약한 튕김

        # --- 수면 복원 & 튕김 처리 (추가 부분) ---
        water_h_after = water.sample_height(self.x)
        bottom_z_after = self.z - SPHERE_RADIUS
        depth_after = water_h_after - bottom_z_after

        # 너무 깊게 잠겨 있으면 수면 쪽으로 밀어 올리고, 약간 튕기게
        if depth_after > MAX_PENETRATION:
            penetration = depth_after - MAX_PENETRATION

            # 강체를 수면 쪽으로 되밀기 (관통량만큼 올려줌)
            self.z += penetration

            # 아래로 향하던 속도가 있으면 위로 반사 (복원 / 반발 계수 적용)
            if self.vz < 0.0:
                self.vz = -self.vz * SURFACE_RESTITUTION

    def get_screen_pos(self):
        # x: 격자 index -> 화면 x
        sx = int(self.x * CELL_SIZE + CELL_SIZE * 0.5)
        # z: 월드 -> 화면 y (위가 0, 아래로 증가)
        sy = int(MID_Y - self.z * Z_SCALE)
        return sx, sy


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
        # --- 이벤트 처리 ---
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
                gx = mx / float(CELL_SIZE)   # 격자 index 좌표
                # 화면 y를 월드 z로 역변환 (기준선=0)
                gz = (MID_Y - my) / Z_SCALE

                if event.button == 1:
                    # 왼쪽 클릭: 구 하나 생성해서 그 위치에 '놓기'
                    spheres.append(Sphere(gx, gz))
                elif event.button == 3:
                    # 오른쪽 클릭: 수면에 수동 파동
                    water.add_impulse(gx, strength=-0.2)

        # --- 시뮬레이션 업데이트 ---
        for s in spheres:
            s.update(water)
        water.step()

        # --- 렌더링 ---
        draw_water(screen, water)
        draw_spheres(screen, spheres, water)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
