#include <Arduino.h>
#include <Wire.h>
#include "Arduino_GFX_Library.h"
#include "FluidRenderer.hpp"
#include "LowPowerESP32.h"  // ★ 低功耗库
#include "ParticleSimulation.hpp"
#include "pin_config.h"
#include "qmi8658c.hpp"

using namespace low_power;

/* ────── 硬件/模块 ─────────────────────────── */
Arduino_DataBus* bus = new Arduino_ESP32QSPI(LCD_CS /* CS */,
                                             LCD_SCLK /* SCK */,
                                             LCD_SDIO0 /* SDIO0 */,
                                             LCD_SDIO1 /* SDIO1 */,
                                             LCD_SDIO2 /* SDIO2 */,
                                             LCD_SDIO3 /* SDIO3 */);

Arduino_GFX* display = new Arduino_CO5300(bus,
                                          LCD_RESET /* RST */,
                                          0 /* rotation */,
                                          false /* IPS */,
                                          LCD_WIDTH,
                                          LCD_HEIGHT,
                                          6 /* col_offset1 */,
                                          0 /* row_offset1 */,
                                          0 /* col_offset2 */,
                                          0 /* row_offset2 */
);
static QMI8658C imu;
static ParticleSimulation sim;
static FluidRenderer renderer(display, &sim);
static ArduinoLowPowerESP32 lp;  // ★ 低功耗对象

/* ────── 运行参数 ──────────────────────────── */
static constexpr float TARGET_FPS = 40.f;
static constexpr float FIXED_DT = 1.f / TARGET_FPS;

/* ────── 运动检测参数 ──────────────────────── */
static constexpr float GYRO_EPS = 10.0f;     // Δ阈值
static constexpr uint32_t STILL_MS = 30000;  // 判静止时间
static constexpr uint32_t DETECT_MS = 1000;  // 判静止时间

/* ────── 状态机枚举 ────────────────────────── */
enum class AppState { RUNNING, GO_SLEEP, SLEEP_POLL };
static AppState state = AppState::RUNNING;

/* ────── 陀螺仪监视变量 ────────────────────── */
static float prevGx = 0, prevGy = 0;
static uint32_t stillTimer = 0;

/* ────── 辅助：读取陀螺仪 Δ ─────────────────── */
static bool gyroMoving(float& dxdy) {
  float gx, gy, gz;
  if (!imu.readGyroscope(&gx, &gy, &gz))
    return true;  // 读失败视为运动

  dxdy = hypotf(gx - prevGx, gy - prevGy);  // √Δx²+Δy²
  prevGx = gx;
  prevGy = gy;
  return dxdy > GYRO_EPS;
}

/* ────── 初始化 ────────────────────────────── */
void setup() {
  Serial.begin(115200);
#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif
  // Init Display
  if (!display->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  display->setRotation(0);
  display->fillScreen(BLACK);

  Wire1.begin(PIN_IMU_SDA, PIN_IMU_SCL, 400000);
  imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);
  imu.configureGyro(QMI8658C::GyroScale::GYRO_SCALE_256DPS,
                    QMI8658C::GyroODR::GYRO_ODR_250HZ);

  sim.begin(&imu);
  renderer.setGridSolidColor(DARKGREY);
  renderer.setGridFluidColor(BLUE);

  lp.setSleepMode(sleep_mode_enum_t::lightSleep);
  // lp.setRestart(false);  // 醒来不复位
}

/* ────── 主循环 ────────────────────────────── */
void loop() {
  float dG = 0.f;  // 本帧 gyro Δ

  switch (state) {
    /* ――― 正常运行 ――― */
    case AppState::RUNNING: {
      /* 物理 & 渲染 */
      sim.simulate(FIXED_DT);
      renderer.render(FluidRenderer::PARTIAL_GRID);

      /* 运动检测 */
      bool moving = gyroMoving(dG);
      if (moving) {
        stillTimer = millis();  // 重置
      } else if (millis() - stillTimer > STILL_MS) {
        state = AppState::GO_SLEEP;
      }
      break;
    }

    /* ――― 进入休眠前的一次性收尾 ――― */
    case AppState::GO_SLEEP: {
      display->fillScreen(BLACK);  // 可选：关闭背光
      // display->setBrightness(0);  // 若驱动支持
      state = AppState::SLEEP_POLL;
      break;
    }

    /* ――― Deep-sleep 轮询 ――― */
    case AppState::SLEEP_POLL: {
      /* 进入 deep-sleep（约 1 s） */
      lp.sleepFor(DETECT_MS, time_unit_t::ms);

      /* 醒来后只读 gyro 做判断 */
      bool moving = gyroMoving(dG);
      if (moving) {
        stillTimer = millis();
        state = AppState::RUNNING;  // 恢复正常
        // display->setBrightness(255);  // 恢复背光（如有）
      }
      /* 若仍静止则下一轮 loop() 会再次 lp.sleep() */
      break;
    }
  }
}