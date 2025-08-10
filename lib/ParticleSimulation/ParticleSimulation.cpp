#include "ParticleSimulation.hpp"
#include <math.h>
#include <string.h>

// ──────────────────────────────────────── 工具
// Carmack's fast inverse square root
static inline float rsqrt(float n) {
  static_assert(sizeof(float) == sizeof(int32_t), "float is not 32-bit");
  int32_t i;
  float x2, y;
  const float threehalfs = 1.5f;

  x2 = n * 0.5f;
  y = n;
  memcpy(&i, &y, sizeof(float));  // evil floating point bit level hacking
  i = 0x5f3759df - (i >> 1);      // what the fuck?
  memcpy(&y, &i, sizeof(float));
  y = y * (threehalfs - (x2 * y * y));  // 1st iteration
  return y;
}

static inline FluidType classifyCell(int n, float v) {
  if (n < FLUID_PARTICLE_THRESHOLD)
    return FLUID_EMPTY;
  if (v > FOAM_SPEED_THRESHOLD)
    return FLUID_FOAM;
  return FLUID_LIQUID;
}

// ──────────────────────────────────────── 初始化
void ParticleSimulation::begin(QMI8658C* imu) {
  m_imu = imu;
  m_numParticles = PC_MAX;
  seedParticles();
  initGrid();
}

void ParticleSimulation::seedParticles() {
  for (int i = 0; i < m_numParticles; ++i) {
    // 0.2 ~ 0.8 区域内随机
    m_particles[i].x = random(20, 80) / 100.0f;
    m_particles[i].y = random(20, 80) / 100.0f;
    m_particles[i].vx = random(-50, 50) / 100.0f * CELL;  // 速度尺度≈单元
    m_particles[i].vy = random(-50, 50) / 100.0f * CELL;
    m_particles[i].r = 0.2f;
    m_particles[i].g = 0.4f;
    m_particles[i].b = 1.0f;
  }
}

void ParticleSimulation::initGrid() {
  for (int i = 0; i < GC; ++i) {
    int gx = i / GS, gy = i % GS;
    float cx = (gx + 0.5f) * CELL - 0.5f,
          cy = (gy + 0.5f) * CELL - 0.5f;  // 以(0,0)~(1,1)中心
    float rad = 0.5f - CELL;               // 圆容器半径
    m_cellType[i] = (cx * cx + cy * cy <= rad * rad) ? FLUID_CELL : SOLID_CELL;
    m_s[i] = 1.0f;
  }
}

// ──────────────────────────────────────── 主循环
void ParticleSimulation::simulate(float dt) {
  /* ───── 计时用变量 ─────────────────── */
  static uint32_t accIMU = 0;
  static uint32_t accIntg = 0;
  static uint32_t accPush = 0;
  static uint32_t accTvG = 0;  // transfer → grid
  static uint32_t accSolve = 0;
  static uint32_t accTvP = 0;  // transfer → particle
  static uint32_t accStat = 0;
  static uint32_t frames = 0;
  static uint32_t tLastPrint = millis();

  /* ───── 阶段 1：IMU ─────────────────── */
  uint32_t t0 = micros();
  updateIMU();
  uint32_t t1 = micros();

  /* ───── 阶段 2：积分 & 碰撞 ──────────── */
  integrateParticles(dt);
  uint32_t t2 = micros();

  /* ───── 阶段 3：粒子推开  ─────────────── */
  pushParticlesApart(SEPARATE_ITERS_P);
  uint32_t t3 = micros();

  /* ───── 阶段 4：粒子 → 网格 (PIC) ─────── */
  transferVelocities(true, 0.0f);
  uint32_t t4 = micros();

  /* ───── 阶段 5：压力求解 ──────────────── */
  solveIncompressibility(SOLVER_ITERS_P, dt);
  uint32_t t5 = micros();

  /* ───── 阶段 6：网格 → 粒子 (FLIP/PIC) ─ */
  transferVelocities(false, FLIP_RATIO);
  uint32_t t6 = micros();

  /* ───── 阶段 7：统计/卷积 ─────────────── */
  classifyGridCells();  // <── 二合一的函数
  updateFluidCells();
  uint32_t t7 = micros();

  /* ───── 累加 ─────────────────────────── */
  accIMU += t1 - t0;
  accIntg += t2 - t1;
  accPush += t3 - t2;
  accTvG += t4 - t3;
  accSolve += t5 - t4;
  accTvP += t6 - t5;
  accStat += t7 - t6;
  ++frames;

  /* ───── 每秒打印一次 ─────────────────── */
  if (millis() - tLastPrint >= 1000) {
    Serial.printf(
        "[%3u fps]  IMU:%4lu  Intg:%4lu  Push:%4lu  ToG:%4lu  Solve:%4lu  "
        "ToP:%4lu  Stat:%4lu (µs per frame)\r\n",
        frames, accIMU / frames, accIntg / frames, accPush / frames,
        accTvG / frames, accSolve / frames, accTvP / frames, accStat / frames);
    /* 清零 */
    accIMU = accIntg = accPush = accTvG = accSolve = accTvP = accStat = 0;
    frames = 0;
    tLastPrint = millis();
  }
}

// ──────────────────────────────────────── 动态网格分类
void ParticleSimulation::classifyGridCells() {
  /* 0️⃣ 备份上一帧状态 */
  memcpy(m_prevFluid, m_currFluid, sizeof(m_currFluid));

  /* 1️⃣ 统计粒子覆盖半径 & 速度平方：cnt[], acc[] */
  static uint16_t cnt[GC];
  static float acc[GC];
  memset(cnt, 0, sizeof(cnt));
  memset(acc, 0, sizeof(acc));

  const float r = PARTICLE_RADIUS, r2 = r * r, cell = CELL;
  for (int p = 0; p < m_numParticles; ++p) {
    const float px = m_particles[p].x, py = m_particles[p].y;
    const float speedSq =
        m_particles[p].vx * m_particles[p].vx + m_particles[p].vy * m_particles[p].vy;
    int gx0 = clampF(int((px - r) * GS), 0, GS - 1),
        gy0 = clampF(int((py - r) * GS), 0, GS - 1);
    int gx1 = clampF(int((px + r) * GS), 0, GS - 1),
        gy1 = clampF(int((py + r) * GS), 0, GS - 1);
    for (int gx = gx0; gx <= gx1; ++gx)
      for (int gy = gy0; gy <= gy1; ++gy) {
        // float cx = (gx + 0.5f) * cell, cy = (gy + 0.5f) * cell;
        // float dx = cx - px, dy = cy - py;
        // if (dx * dx + dy * dy > r2) continue;
        int id = idx(gx, gy);
        ++cnt[id];
        acc[id] += speedSq;
      }
  }

  /* 2️⃣ 根据统计值，分类物理网格 (m_simCellType) 和渲染网格 (m_currFluid) */
  for (int i = 0; i < GC; ++i) {
    const int n = cnt[i];
    const float v2 = n ? acc[i] / n : 0.f;
    // 物理
    if (m_cellType[i] == SOLID_CELL) {
      m_simCellType[i] = SOLID_CELL;
    } else if (n >= FLUID_PARTICLE_THRESHOLD) {
      m_simCellType[i] = FLUID_CELL;
    } else {
      m_simCellType[i] = AIR_CELL;
    }
    // 渲染
    if (n >= FLUID_PARTICLE_THRESHOLD)
      m_currFluid[i] = (v2 > FOAM_SPEED_THRESHOLD_SQ) ? FLUID_FOAM : FLUID_LIQUID;
    else if (n >= FLUID_RIM_PARTICLE_THRESHOLD)
      m_currFluid[i] = FLUID_RIM_TRANSPARENT;
    else
      m_currFluid[i] = FLUID_EMPTY;
  }
}

// ──────────────────────────────────────── IMU
void ParticleSimulation::updateIMU() {
  if (!m_imu)
    return;
  float ax, ay, az;
  if (m_imu->readAccelerometer(&ax, &ay, &az)) {
    m_ax = -ax * 10.f * GRAVITY_MODIFIER;  // 缩放到归一化空间
    m_ay = -ay * 10.f * GRAVITY_MODIFIER;
  }
}

// ──────────────────────────────────────── 积分+碰撞
void ParticleSimulation::integrateParticles(float dt) {
  constexpr float CX = 0.5f, CY = 0.5f, R = 0.5f - CELL - PARTICLE_RADIUS;
  for (int i = 0; i < m_numParticles; ++i) {
    Particle& p = m_particles[i];
    p.vx += m_ax * dt;
    p.vy += m_ay * dt;
    p.x += p.vx * dt;
    p.y += p.vy * dt;

    // 边界盒
    p.x = clampF(p.x, PARTICLE_RADIUS, 1.0f - PARTICLE_RADIUS);
    p.y = clampF(p.y, PARTICLE_RADIUS, 1.0f - PARTICLE_RADIUS);

    // 圆容器碰撞
    float dx = p.x - CX, dy = p.y - CY;
    float d2 = dx * dx + dy * dy;
    if (d2 > R * R) {
      // ---------- 推回圆内 ----------
      float inv_d = rsqrt(d2);
      float d = 1.0f / inv_d;
      float nx = dx * inv_d;  // 法向单位向量
      float ny = dy * inv_d;

      float s = R - d;  // 穿透深度
      p.x += nx * s;    // 直接平移回边界
      p.y += ny * s;

      // ---------- 速度分解 ----------
      float vn = p.vx * nx + p.vy * ny;  // 法向分量
      float vx_n = vn * nx;              // 法向速度向量
      float vy_n = vn * ny;
      float vx_t = p.vx - vx_n;  // 切向速度向量
      float vy_t = p.vy - vy_n;

      vx_n = -REST_N * vx_n;
      vy_n = -REST_N * vy_n;
      vx_t = (1.0f - FRIC_T) * vx_t;
      vy_t = (1.0f - FRIC_T) * vy_t;

      p.vx = vx_n + vx_t;
      p.vy = vy_n + vy_t;
    }
  }
}

// ──────────────────────────────────────── Push-Apart
void ParticleSimulation::pushParticlesApart(int iters) {
  const float min2 = (2 * PARTICLE_RADIUS) * (2 * PARTICLE_RADIUS);
  static int particle_cell_idx[PC_MAX];

  memset(m_numPartCell, 0, sizeof(m_numPartCell));
  for (int i = 0; i < m_numParticles; ++i) {
    int xi = clampF(m_particles[i].x * P_INV_SP, 0, PNX - 1);
    int yi = clampF(m_particles[i].y * P_INV_SP, 0, PNY - 1);
    int cell_idx = xi * PNY + yi;
    particle_cell_idx[i] = cell_idx;
    ++m_numPartCell[cell_idx];
  }
  int pref = 0;
  for (int i = 0; i < PNC; ++i) {
    int n = m_numPartCell[i];
    m_firstPart[i] = pref;
    pref += n;
  }
  m_firstPart[PNC] = pref;
  for (int i = 0; i < m_numParticles; ++i) {
    m_cellPartIds[--m_firstPart[particle_cell_idx[i]]] = i;
  }

  for (int it = 0; it < iters; ++it) {
    for (int i = 0; i < m_numParticles; ++i) {
      Particle& a = m_particles[i];
      int cx = a.x * P_INV_SP, cy = a.y * P_INV_SP;

      // ─── 5点邻域检查（替代 3x3 九宫格） ───
      int cells_to_check[5][2] = {{cx, cy},
                                  {cx - 1, cy},
                                  {cx + 1, cy},
                                  {cx, cy - 1},
                                  {cx, cy + 1}};

      for (int i_cell = 0; i_cell < 5; ++i_cell) {
        int xi = cells_to_check[i_cell][0];
        int yi = cells_to_check[i_cell][1];
        if (xi < 0 || xi >= PNX || yi < 0 || yi >= PNY) continue;

        int cell = xi * PNY + yi;
        for (int k = m_firstPart[cell]; k < m_firstPart[cell + 1]; ++k) {
          int j = m_cellPartIds[k];
          if (j <= i) continue;
          Particle& b = m_particles[j];
          float dx = b.x - a.x, dy = b.y - a.y, d2 = dx * dx + dy * dy;
          if (d2 > min2 || d2 < 1e-9f) continue;
          float inv_d = rsqrt(d2);
          float d = 1.0f / inv_d;
          float s = 0.5f * ((2 * PARTICLE_RADIUS) - d) * inv_d;
          s *= m_pushApartFactor;
          dx *= s;
          dy *= s;
          a.x -= dx;
          a.y -= dy;
          b.x += dx;
          b.y += dy;
        }
      }
    }
  }
}

// ──────────────────────────────────────── 速度搬运
void ParticleSimulation::transferVelocities(bool toGrid, float flipRatio) {
  const float hInv = float(GS);  // 1/H
  if (toGrid) {
    memcpy(m_prevU, m_u, sizeof(m_u));
    memcpy(m_prevV, m_v, sizeof(m_v));
    memset(m_u, 0, sizeof(m_u));
    memset(m_v, 0, sizeof(m_v));
    memset(m_du, 0, sizeof(m_du));
    memset(m_dv, 0, sizeof(m_dv));
  }

  for (int comp = 0; comp < 2; ++comp) {
    float dx = comp ? 0.5f * CELL : 0.0f;
    float dy = comp ? 0.0f : 0.5f * CELL;
    float *f = comp ? m_v : m_u, *fp = comp ? m_prevV : m_prevU,
          *dw = comp ? m_dv : m_du;

    for (int p = 0; p < m_numParticles; ++p) {
      Particle& pr = m_particles[p];
      float fx = (pr.x - dx) * hInv, fy = (pr.y - dy) * hInv;
      int x0 = (int)fx, y0 = (int)fy;
      float tx = fx - x0, ty = fy - y0, sx = 1 - tx, sy = 1 - ty;
      int x1 = x0 + 1 < GS ? x0 + 1 : GS - 1,
          y1 = y0 + 1 < GS ? y0 + 1 : GS - 1;
      float w0 = sx * sy, w1 = tx * sy, w2 = tx * ty, w3 = sx * ty;
      int n0 = idx(x0, y0), n1 = idx(x1, y0), n2 = idx(x1, y1),
          n3 = idx(x0, y1);

      if (toGrid) {
        float pv = comp ? pr.vy : pr.vx;
        f[n0] += pv * w0;
        dw[n0] += w0;
        f[n1] += pv * w1;
        dw[n1] += w1;
        f[n2] += pv * w2;
        dw[n2] += w2;
        f[n3] += pv * w3;
        dw[n3] += w3;
      } else {
        // G2P: 只更新在流体中的粒子
        int gx = clampF(pr.x * GS, 0, GS - 1);
        int gy = clampF(pr.y * GS, 0, GS - 1);
        if (m_simCellType[idx(gx, gy)] == FLUID_CELL) {
          float pic = w0 * f[n0] + w1 * f[n1] + w2 * f[n2] + w3 * f[n3];
          float corr = w0 * (f[n0] - fp[n0]) + w1 * (f[n1] - fp[n1]) +
                       w2 * (f[n2] - fp[n2]) + w3 * (f[n3] - fp[n3]);
          float flip = (comp ? pr.vy : pr.vx) + corr;
          float val = (1.f - flipRatio) * pic + flipRatio * flip;
          if (comp)
            pr.vy = val;
          else
            pr.vx = val;
        }
      }
    }
    if (toGrid)
      for (int i = 0; i < GC; ++i)
        if (dw[i] > 0)
          f[i] /= dw[i];
  }
}

// ──────────────────────────────────────── 压力求解
void ParticleSimulation::solveIncompressibility(int iters, float dt) {
  const float cp = FLUID_DENSITY * CELL / dt;
  for (int k = 0; k < iters; ++k) {
    for (int gx = 1; gx < GS - 1; ++gx)
      for (int gy = 1; gy < GS - 1; ++gy) {
        int c = idx(gx, gy);
        if (m_simCellType[c] != FLUID_CELL)
          continue;
        int l = idx(gx - 1, gy), r = idx(gx + 1, gy), b = idx(gx, gy - 1),
            t = idx(gx, gy + 1);
        float div = m_u[r] - m_u[c] + m_v[t] - m_v[c];
        float p = -div / 4.f * 1.9f;
        m_pressure[c] += cp * p;
        m_u[c] -= p;
        m_u[r] += p;
        m_v[c] -= p;
        m_v[t] += p;
      }
  }
}

// ──────────────────────────────────────── 状态统计
void ParticleSimulation::updateFluidCells() {
  /* 卷积：EMPTY → RIM_LIGHT（若邻接液体边缘） */
  memcpy(convTmp, m_currFluid, sizeof(convTmp));  // 保留原状态

  auto neighborFilled = [&](int id) -> bool {
    // 从当前帧的初始状态（卷积前）判断
    return (m_currFluid[id] == FLUID_RIM_TRANSPARENT) ||
           (m_currFluid[id] == FLUID_LIQUID) || (m_currFluid[id] == FLUID_FOAM);
  };

  for (int gx = 0; gx < GS; ++gx) {
    for (int gy = 0; gy < GS; ++gy) {
      int id = idx(gx, gy);
      if (m_currFluid[id] != FLUID_EMPTY)
        continue;  // 只有 EMPTY 需要判断

      int touch = 0;
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
          if (!dx && !dy) continue;  // 跳过自身
          if (dx && dy) continue;    // 只看上下左右
          int nx = gx + dx, ny = gy + dy;
          if (nx < 0 || nx >= GS || ny < 0 || ny >= GS) continue;
          if (neighborFilled(idx(nx, ny))) ++touch;
        }

      // 根据触边数量决定亮度等级（写到临时数组 convTmp）
      if (touch >= 4)
        convTmp[id] = FLUID_LIQUID;
      else if (touch >= 2)
        convTmp[id] = FLUID_RIM_TRANSPARENT;
      else if (touch >= 1)
        convTmp[id] = FLUID_RIM_LIGHT;
    }
  }

  /* ---------- 写回并生成变化表 ---------- */
  m_changedCnt = 0;
  for (int i = 0; i < GC; ++i) {
    if (convTmp[i] != m_prevFluid[i])
      m_changedIdx[m_changedCnt++] = i;
    m_currFluid[i] = convTmp[i];
  }
}
