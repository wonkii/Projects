import numpy as np
import math

from config import *

# ============================
# 물 Height Field (1D)
# ============================

class Water1D:
    def __init__(self, n_points):
        self.n = n_points # 수면을 이루는 점의 개수
        self.h = np.zeros(self.n, dtype=np.float32)       # 수면의 현재 높이
        self.h_prev = np.zeros(self.n, dtype=np.float32)  # 수면의 이전 높이
        self.h_next = np.zeros(self.n, dtype=np.float32)  # 수면의 다음 높이
        
        self.alpha = np.zeros(self.n, dtype=np.float32)   # 경계 흡수용 감쇠 계수
        self._init_boundary_damping()

    # 경계에서 중앙으로 갈수록 0으로 줄어드는 감쇠 계수
    def _init_boundary_damping(self):
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
        # 1D 파동 방정식 이산화로 한 스텝 진행
        h = self.h
        h_prev = self.h_prev
        h_next = self.h_next

        # 내부 점 업데이트
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
        # 수면 높이 제한 (화면 밖으로 나가는 것 방지)
        np.clip(h_next, -MAX_WATER_HEIGHT, MAX_WATER_HEIGHT, out=h_next)
        # 버퍼 스왑
        self.h_prev, self.h, self.h_next = self.h, self.h_next, self.h_prev

    """
        연속 좌표 x (0 ~ n-1) 에서 선형 보간으로 높이 샘플링.
        이 값이 '기준 수면(0)'에서 얼마나 위/아래로 변위됐는지.
    """
    
    def sample_height(self, x):
        x = max(0.0, min(self.n - 1.001, x))
        i0 = int(math.floor(x))
        i1 = min(i0 + 1, self.n - 1)
        t = x - i0
        return self.h[i0] * (1.0 - t) + self.h[i1] * t


    """
        x 주변에 로컬 교란을 주어 파동 생성.
        strength > 0 이면 위로, < 0 이면 아래로 밀어냄.
    """

    def add_impulse(self, x, strength, radius=IMPULSE_RADIUS):
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
        np.clip(self.h, -MAX_WATER_HEIGHT, MAX_WATER_HEIGHT, out=self.h)