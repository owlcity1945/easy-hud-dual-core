#include <TFT_eSPI.h>
#include <SPI.h>

#include "easy_hud.h"

#include <Arduino.h>

//增加I2C姿态传感器
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno(55);
TaskHandle_t TaskSensorHandle;  // 传感器任务句柄

// 共享变量（主循环只读，任务只写）
volatile float global_pitch = 0.0f;
volatile float global_roll = 0.0f;
volatile float global_heading = 0.0f;

// 互斥锁，防止读写冲突
portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

// --- Core 0 任务：专门负责读取传感器和滤波 ---
void TaskSensorCode(void* pvParameters) {
  Serial.print("传感器任务运行在 Core: ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    // 1. 读取原始数据 (耗时操作)
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float raw_heading = euler.x();
    float raw_pitch = euler.y();
    float raw_roll = euler.z();

    // 2. 滤波逻辑 (原代码逻辑移到这里)
    static float filtered_pitch = 0.0f;
    static float filtered_roll = 0.0f;
    const float alpha = 0.05f;  // 滤波系数

    filtered_pitch = filtered_pitch * (1 - alpha) + raw_pitch * alpha;
    filtered_roll = filtered_roll * (1 - alpha) + raw_roll * alpha;

    // 3. 更新共享变量 (加锁保护)
    portENTER_CRITICAL(&sensorMux);
    global_heading = raw_heading;
    global_pitch = filtered_pitch;
    global_roll = filtered_roll;
    portEXIT_CRITICAL(&sensorMux);

    // 4. 任务调度延迟 (非常重要，给看门狗喂狗)
    vTaskDelay(pdMS_TO_TICKS(5));  // 5ms更新一次，约200Hz，足够平滑
  }
}

void setup() {
  Serial.begin(115200);


  Wire.begin(8, 9);       // BHO055姿态传感器 I2C 引脚：SDA = GPIO18, SCL = GPIO19
  Wire.setClock(400000);  // 400kHz，提高兼容性

  if (!bno.begin()) {
    Serial.println("未找到 BNO055");
    while (1)
      ;
  }
  delay(1000);
  bno.setExtCrystalUse(true);  // 使用外部晶振（如果有）
  Serial.println("BNO055 初始化完成");

  tft.initDMA();  // ✅ 开启 DMA 支持，提高1帧，位置不能放在tft初始化后面
  tft.init();
  tft.setRotation(0);

  //采用寄存器方式镜像屏幕，提升7帧
  tft.writecommand(0x36);      // MADCTL 内存数据访问控制命令 
  tft.writedata(0x00 | 0x40 | 0x08);// 0x00是典型的竖屏值，0x40是MX位(镜像X轴)，加上 0x08 (BGR位)，0x08 (Bit 3) = 颜色反转 (RGB <-> BGR)

  spr.setColorDepth(16);  // ✅ 改为16位色深（RGB565）  // 使用 8 位颜色模式（256色）
  spr.createSprite(WIDTH, HEIGHT);


  tft.fillScreen(TFT_BLACK);  // 清空屏幕

  // --- 启动双核任务 ---
  xTaskCreatePinnedToCore(
    TaskSensorCode,     // 任务函数
    "TaskSensor",       // 任务名称
    4096,               // 堆栈大小
    NULL,               // 参数
    1,                  // 优先级 (1是低优先级，但足够了)
    &TaskSensorHandle,  // 句柄
    0);                 // 📌 核心 0 (后台核心)
}




void loop() {

  words_display();
}
