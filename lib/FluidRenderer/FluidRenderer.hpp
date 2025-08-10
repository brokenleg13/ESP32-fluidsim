#pragma once
#include <Arduino_GFX_Library.h>
#include "ParticleSimulation.hpp"

#define RENDER_GRID_SIZE 48
#define RENDER_PIXEL_PER_CELL SCREEN_HEIGHT / RENDER_GRID_SIZE

// 渲染器使用的流体类型定义
enum RenderFluidType : uint8_t {
  RENDER_FLUID_EMPTY,
  RENDER_FLUID_LIQUID,
  RENDER_FLUID_FOAM,
  RENDER_FLUID_RIM_TRANSPARENT,
  RENDER_FLUID_RIM_LIGHT
};

class FluidRenderer {
 public:
  enum Mode { BALLS, GRID, PARTIAL_GRID, PARTIAL_BALLS };

  FluidRenderer(Arduino_GFX* disp, const ParticleSimulation* sim)
      : m_disp(disp), m_sim(sim) {
    // 初始化状态数组
    memset(m_prevFluid, 0, sizeof(m_prevFluid));
    memset(m_currFluid, 0, sizeof(m_currFluid));
  }

  void render(Mode mode);

  // 原有接口
  void renderBalls();
  void renderGrid();
  void renderPartialGrid();

  // 新增：更新流体状态（从粒子数据计算渲染网格状态）
  void updateFluidCells();

  // 配色
  void setBallBaseColor(uint16_t c) { m_ballBase = c; }
  void setGridSolidColor(uint16_t c) { m_gridSolid = c; }
  void setGridFluidColor(uint16_t c) { m_gridFluid = c; }

  // 设置渲染网格大小（默认与模拟网格相同）
  void setRenderGridSize(int size) {
    m_renderGridSize = size;
    m_renderCellSize = SCREEN_WIDTH / size;
  }

 private:
  Arduino_GFX* m_disp;
  const ParticleSimulation* m_sim;

  // 渲染网格参数
  int m_renderGridSize = RENDER_GRID_SIZE;
  int m_renderCellSize = RENDER_PIXEL_PER_CELL;

  // 状态追踪（使用最大可能的网格大小）
  static constexpr int MAX_GRID_CELLS = RENDER_GRID_SIZE * RENDER_GRID_SIZE;

  RenderFluidType m_prevFluid[MAX_GRID_CELLS];
  RenderFluidType m_currFluid[MAX_GRID_CELLS];
  int m_changedIdx[MAX_GRID_CELLS];
  int m_changedCnt = 0;

  // 临时缓冲区
  RenderFluidType m_convTmp[MAX_GRID_CELLS];

  // 颜色配置
  uint16_t m_ballBase = CYAN;
  uint16_t m_gridSolid = DARKGREY;
  uint16_t m_gridFluid = BLUE;
  uint16_t m_gridFoam = WHITE;

  // 辅助函数
  inline uint16_t lerp565(uint16_t c1, uint16_t c2, float t) const;
  inline int idx(int x, int y) const { return x * m_renderGridSize + y; }

  // 获取渲染网格对应的流体类型颜色
  uint16_t getFluidColor(RenderFluidType type) const;

  // 检查模拟网格中的固体单元（需要坐标转换）
  bool isSimSolid(int renderGx, int renderGy) const;
};