//自定义特殊颜色
uint16_t TFT_BRIGHT_RED = tft.color565(255, 60, 60);      // 亮红色（高警示/战斗提示）
uint16_t TFT_BRIGHT_PINK = tft.color565(255, 100, 180);   // 亮粉红（柔和警示/女性风格）
uint16_t TFT_NEON_PINK = tft.color565(255, 20, 147);      // 荧光粉红（紫偏粉，夜视感强）
uint16_t TFT_LIGHT_YELLOW = tft.color565(255, 255, 180);  // 浅黄色（柔和提示/高亮区域）
uint16_t TFT_SKY_BLUE = tft.color565(135, 206, 235);      // 天蓝色（高度线/背景信息）
uint16_t TFT_MINT_GREEN = tft.color565(152, 255, 152);    // 薄荷绿（系统正常/辅助线）

// ===== 锁定目标全局变量定义 =====
struct Target {
  float x, y;    // 当前坐标
  float vx, vy;  // 当前速度
  bool active;   // 是否存在目标
  bool locked;   // 是否锁定成功
};

Target target = { 0 };

enum HudState { HUD_IDLE,
                HUD_REACT_DELAY,
                HUD_CHASE,
                HUD_LOCKED };

// 📌 全局变量（放在外部，保留状态）
// --- HUD 状态变量 ---
float hud_angle = 0.0f;     // 当前 HUD 旋转角度（度）
float hud_velocity = 0.0f;  // 当前 HUD 旋转角速度
int last_counter_hud = 0;

float hud_offset_x = 0.0f;  // 当前漂移位置
float hud_offset_y = 0.0f;
float hud_vx = 0.0f;  // 漂移速度
float hud_vy = 0.0f;


uint16_t bg_colors[8] = {
  TFT_YELLOW,       // 黄色（夜视风格/军事）
  TFT_WHITE,        // 白色（高亮/主文字）
  TFT_GREEN,        // 绿色
  TFT_MAGENTA,      // 洋红
  TFT_ORANGE,       // 橙色
  TFT_BRIGHT_RED,   // 亮红色（高警示/战斗提示）
  TFT_BRIGHT_PINK,  // 亮粉红（柔和警示）
  TFT_NEON_PINK,    // 荧光粉红（紫偏粉，夜视感强）

};

// 🎨 全局HUD颜色索引 & 当前颜色
int ladder_color_index = 0;
uint16_t ladder_color = TFT_YELLOW;  // 默认绿色或你喜欢的起始颜色

// 引用外部共享变量
extern volatile float global_pitch;
extern volatile float global_roll;
extern volatile float global_heading;
extern portMUX_TYPE sensorMux;

//姿态传感器数据获取，优化后的数据获取函数，提升1帧
void target_from_bno055(float &x, float &y) {
  float pitch, roll;

  // 快速读取共享变量 (加锁极快)
  portENTER_CRITICAL(&sensorMux);
  pitch = global_pitch;
  roll  = global_roll;
  // heading = global_heading; // 如果需要 heading
  portEXIT_CRITICAL(&sensorMux);

  // === 小球坐标更新 ===
  // 滤波已经在 Core 0 完成了，这里直接用
  const float scale_x = 2.2f;
  const float scale_y = 2.5f;
  
  x = WIDTH / 2 + roll * scale_x;
  y = HEIGHT / 2 + pitch * scale_y;
}
// void target_from_bno055(float &x, float &y) {
//   // === 姿态传感器输入 ===
//   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//   float heading = euler.x();  // 偏航角
//   float pitch = euler.y();    // 俯仰角
//   float roll = euler.z();     // 翻滚角
//   // === 低通滤波 ===
//   static float filtered_pitch = 0.0f, filtered_roll = 0.0f;  // 用于保存滤波后的值
//   const float alpha = 0.05f;                                 // 滤波因子（可以根据需要调整，通常在 0.05 到 0.2 之间）✅ 重要惯性响应参数

//   // 一阶低通滤波器
//   filtered_pitch = filtered_pitch * (1 - alpha) + pitch * alpha;
//   filtered_roll = filtered_roll * (1 - alpha) + roll * alpha;
//   // === 小球坐标更新（姿态接管） ===
//   const float scale_x = 2.2f;
//   const float scale_y = 2.5f;

//   x = WIDTH / 2 + filtered_roll * scale_x;    // 更新 x
//   y = HEIGHT / 2 + filtered_pitch * scale_y;  // 更新 y
// }

//俯仰梯纵向数字打印函数(不用此函数会出现数字跟随重力悬挂于刻度线，不能保持整体水平跟随)
void draw_rot_string(int value, int x, int y, float angle_deg, uint16_t color) {
  char buf[8];
  sprintf(buf, "%d", abs(value));  // 自动 abs 转字符串

  float angle = angle_deg * DEG_TO_RAD;
  float cos_a = cos(angle);
  float sin_a = sin(angle);

  const int char_w = 16;

  int len = strlen(buf);
  int max_col = 0;

  // ✅ 自动计算最大“右边缘列”
  for (int i = 0; i < len; i++) {
    char ch = buf[i];
    if (ch < '0' || ch > '9') continue;
    int idx = ch - '0';

    int rightmost = 0;
    for (int col = char_w - 1; col >= 0; col--) {
      bool has_pixel = false;
      for (int row = 0; row < 16; row++) {
        uint16_t bits = (digit_16x16[idx][row * 2] << 8) | digit_16x16[idx][row * 2 + 1];
        if ((bits >> (15 - col)) & 1) {
          has_pixel = true;
          break;
        }
      }
      if (has_pixel) {
        rightmost = col + 1;
        break;
      }
    }
    if (rightmost > max_col) max_col = rightmost;
  }

  int spacing = max_col + 1;  // 自动间距（宽度 + 缓冲）

  float total_w = spacing * (len - 1);
  float fx = x - cos_a * (total_w / 2);
  float fy = y - sin_a * (total_w / 2);

  for (int i = 0; i < len; i++) {
    char ch = buf[i];
    if (ch < '0' || ch > '9') continue;
    int idx = ch - '0';

    for (int row = 0; row < 16; row++) {
      uint16_t bits = (digit_16x16[idx][row * 2] << 8) | digit_16x16[idx][row * 2 + 1];
      for (int col = 0; col < 16; col++) {
        if ((bits >> (15 - col)) & 1) {
          int dx = col - 8;
          int dy = row - 8;
          int rx = round(cos_a * dx - sin_a * dy);
          int ry = round(sin_a * dx + cos_a * dy);
          spr.drawPixel(fx + rx, fy + ry, color);
        }
      }
    }

    fx += cos_a * spacing;
    fy += sin_a * spacing;
  }
}



//俯仰梯打印
void draw_dial(TFT_eSprite &spr, int x, int y, float capture_x) {
  // === [静态变量] 用于X轴变化检测和惯性旋转动画 ===
  static float x_prev = 0;              // 上一帧原始 x 坐标
  static float tilt_spin_angle = 0;     // 当前旋转动画角度（叠加项）
  static float tilt_spin_velocity = 0;  // 当前旋转速度
  static bool tilt_spinning = false;    // 是否处于旋转动画状态

  // === 设置 HUD 中心位置 ===
  int xc = WIDTH / 2;   // 水平中心点
  int yc = HEIGHT / 2;  // 垂直中心点

  // === 计算原始 X 轴速度：用于判断是否触发旋转动画 ===
  float x_speed = capture_x - x_prev;
  x_prev = capture_x;

  // float pitch_angle_base = (x - xc) * 0.25f;  // 当前基础倾斜角度（原始写法改为 0.25 更明显）
  float pitch_angle_base = (x - xc) * 0.4f;  // 适中的倾斜灵敏度
  if (abs(x_speed) > 6 && !tilt_spinning) {
    tilt_spinning = true;
    tilt_spin_velocity = (x_speed > 0 ? +1 : -1) * 4.50f;  // 初始旋转速度：方向+强度
  }

  if (tilt_spinning) {
    tilt_spin_angle += tilt_spin_velocity;  // 累加角度（旋转）
    tilt_spin_velocity *= 0.985f;           // 衰减（阻尼）
    if (abs(tilt_spin_velocity) < 0.5f) {
      tilt_spin_velocity = 0;
      tilt_spin_angle += (0 - tilt_spin_angle) * 0.05f;  // 俯仰梯旋转完缓慢回正
      // 如果角度也回到接近 0，停止运动
      if (abs(tilt_spin_angle) < 0.2f) {
        tilt_spin_angle = 0;
        tilt_spinning = false;
      }
    }
  }
  // === 最终倾斜角度：基础倾斜 + 惯性叠加 ===
  float pitch_angle = pitch_angle_base + tilt_spin_angle;  // 最终倾斜角度
  float angle_rad = pitch_angle * DEG_TO_RAD;              // 转换为弧度
  float cos_a = cos(angle_rad);                            // 倾斜角度的余弦，用于旋转变换
  float sin_a = sin(angle_rad);                            // 倾斜角度的正弦，用于旋转变换

  // === 俯仰梯显示参数设置 ===
  int pitch_spacing = 60;  // 每 5° 所对应的垂直像素间距
  int pitch_step = 5;      // 每个刻度代表 5 度
  int pitch_range = 90;    // 俯仰显示范围 ±90 度

  // === 左右导轨的横坐标位置 ===
  int rail_left = xc - 10;   // 左侧导轨位置
  int rail_right = xc + 10;  // 右侧导轨位置

  // === 局部函数：将点(x0, y0)绕中心(xc, yc)旋转 pitch_angle ===
  auto rotate_point = [&](int x0, int y0, int &x_out, int &y_out) {
    x_out = xc + (x0 - xc) * cos_a - (y0 - yc) * sin_a;
    y_out = yc + (x0 - xc) * sin_a + (y0 - yc) * cos_a;
  };

  // === 绘制从 -90° 到 +90° 的俯仰刻度线 ===
  for (int i = -pitch_range; i <= pitch_range; i += pitch_step) {
    int pitch_val = i;  // 当前刻度的角度值（-90 ~ +90）

    // === 计算当前刻度相对于目标位置的差值，并做 wrap 环绕处理 ===
    float diff = (yc - y) * 0.4f - pitch_val;  //🔺放大垂直滑动感 从 0.2 改成 0.35~0.5，上下飘逸增强
    diff = fmod(diff + 90.0f, 180.0f);         // wrap 到 0~180
    if (diff < 0) diff += 180.0f;
    diff -= 90.0f;  // 再映射回 -90~+90

    // === 根据差值计算该刻度线应显示在屏幕上的 y 位置 ===
    float y = yc - diff * pitch_spacing / pitch_step;
    if (y < 5 || y > HEIGHT - 5) continue;  // 剪裁不可见区域,y < 20上边剪裁高度,HEIGHT-10	下边剪裁范围

    int len = 36;                  // 每根主线长度
    if (pitch_val == 0) len = 88;  // ✅ 0° 特别加长（视觉基准线）
    // uint16_t color = spr.color565(0, 255, 0);  // 绿色线条颜色

    // === 计算钩子方向向量（垂直于主线） ===
    int hook_len = 10;
    float hook_rad = (pitch_val > 0 ? pitch_angle - 90.0f : pitch_angle + 90.0f) * DEG_TO_RAD;  // ✅ 实线钩子向上，虚线钩子向下
    float dx = hook_len * cos(hook_rad);                                                        // 钩子的 x 偏移
    float dy = hook_len * sin(hook_rad);                                                        // 钩子的 y 偏移

    // === 计算当前线段倾斜角度（受 pitch_val 大小时影响） ===
    float abs_pitch = abs(pitch_val);
    float slope_angle = (abs_pitch <= 85.0f) ? (10.0f * abs_pitch / 85.0f) : 10.0f;  //10度内倾斜角
    float slope_rad = slope_angle * DEG_TO_RAD;
    float slope_dx = tan(slope_rad) * len;    // 水平倾斜距离
    if (pitch_val > 0) slope_dx = -slope_dx;  // ✅ 反转方向使虚线向上倾斜
    else slope_dx = slope_dx;

    // === 左侧主线段：起点(x1,y1) → 终点(x2,y2) ===
    int x1, y1, x2, y2;
    rotate_point(rail_left - len, y - slope_dx, x1, y1);
    rotate_point(rail_left, y + slope_dx, x2, y2);
    if (pitch_val > 0) {
      // 画虚线（每段 2 像素）
      for (int i = 0; i < len; i += 6) {
        float fx1 = float(i) / len;
        float fx2 = float(i + 2) / len;
        int px1 = x1 + (x2 - x1) * fx1;
        int py1 = y1 + (y2 - y1) * fx1;
        int px2 = x1 + (x2 - x1) * fx2;
        int py2 = y1 + (y2 - y1) * fx2;
        spr.drawLine(px1, py1, px2, py2, ladder_color);
      }
    } else {
      spr.drawLine(x1, y1, x2, y2, ladder_color);
    }

    // === 左钩子线段 ===
    rotate_point(rail_left - len, y - slope_dx, x1, y1);
    spr.drawLine(x1, y1, x1 + dx, y1 + dy, ladder_color);

    // === 右侧主线段 ===
    rotate_point(rail_right, y + slope_dx, x1, y1);
    rotate_point(rail_right + len, y - slope_dx, x2, y2);
    if (pitch_val > 0) {
      for (int i = 0; i < len; i += 6) {
        float fx1 = float(i) / len;
        float fx2 = float(i + 2) / len;
        int px1 = x1 + (x2 - x1) * fx1;
        int py1 = y1 + (y2 - y1) * fx1;
        int px2 = x1 + (x2 - x1) * fx2;
        int py2 = y1 + (y2 - y1) * fx2;
        spr.drawLine(px1, py1, px2, py2, ladder_color);
      }
    } else {
      spr.drawLine(x1, y1, x2, y2, ladder_color);
    }

    // === 右钩子线段 ===
    rotate_point(rail_right + len, y - slope_dx, x2, y2);
    spr.drawLine(x2, y2, x2 + dx, y2 + dy, ladder_color);

    // === 绘制数字（左右各一），使用旋转文本函数 ===
    int tx, ty;
    float angle_deg = angle_rad * 180.0 / PI;
    rotate_point(rail_left - len - 8, y, tx, ty);
    draw_rot_string(pitch_val, tx, ty, angle_deg, ladder_color);
    rotate_point(rail_right + len + 14, y, tx, ty);
    draw_rot_string(pitch_val, tx, ty, angle_deg, ladder_color);
  }
}


//锁定器机制与横向滑轨函数
void draw_target_box(TFT_eSprite &spr, float center_x, float center_y) {
  const int TRAIL_LEN = 20;
  static bool target_exists = false;
  static float target_x = 0, target_y = 0;
  static float target_angle = 0, target_speed = 0.5f;
  static unsigned long ballSpawnTime = 0, eliminateTime = 0;
  static float trailX[TRAIL_LEN], trailY[TRAIL_LEN];

  static float hud_x = center_x, hud_y = center_y;
  static float hud_vx = 0, hud_vy = 0;
  static float hud_scale = 1.0f, hud_alpha = 0;
  static HudState hud_state = HUD_IDLE;
  static unsigned long hudDetectStart = 0, hudReactStart = 0;
  static unsigned long lastEvadeTurn = 0, lastBeep = 0;
  static unsigned long evadeStart = 0, lockHoldStart = 0, captureStart = 0;

  unsigned long last_encoder_time = 0;
  unsigned long now = millis();
  bool encoder_idle = (now - last_encoder_time > 1000);

  const unsigned long RESPAWN_MS = 1000;         // 等待 1 秒后刷新敌机
  const unsigned long DETECT_DELAY_MS = 500;     //HUD探测等待时间
  const unsigned long REACT_DELAY_MS = 300;      //HUD启动追逐前延迟反应时间 300
  const unsigned long LOCK_HOLD_MS = 1000;       //HUD 1 秒持续包裹判定锁定成功
  const unsigned long CAPTURE_MS = 800;          // 锁定成功后的过渡时长（例如缩放 HUD）
  const unsigned long EVADE_DURATION_MS = 1000;  // 小球进入逃逸状态后维持 1 秒再恢复巡航

  const float PRE_DETECT_SPEED = 0.5f;  // 巡航速度，低速匀速移动
  const float EVADE_SPEED = 2.5f;       // 逃逸速度，进入 CHASE 状态后加速

  const float HUD_SPRING_K = 0.025f;                     // HUD 追踪响应速度 0.03
  const float HUD_DAMPING = 0.85f;                       // HUD 追踪阻尼（惯性缓冲）0.85f
  const float BREATHE_AMP = 0.01f, BREATHE_SP = 0.005f;  // HUD 缩放呼吸动画的振幅和速度

  const float LOCK_DIST = 12.0f;  // 判断是否可以锁定目标的小球与 HUD 中心的距离阈值（单位：像素）

  static float angle_current = 0;  // ✅小球真实转向角度

  // — AI  小球生成 —
  if (!target_exists && now - eliminateTime >= RESPAWN_MS) {
    target_x = random(20, WIDTH - 20);
    target_y = random(20, HEIGHT - 20);
    target_angle = random(0, 360) * DEG_TO_RAD;
    target_speed = PRE_DETECT_SPEED;
    target_exists = true;
    ballSpawnTime = now;
    evadeStart = lockHoldStart = captureStart = lastEvadeTurn = 0;
    for (int i = 0; i < TRAIL_LEN; i++) {
      trailX[i] = target_x;
      trailY[i] = target_y;
    }
    hud_state = HUD_IDLE;
    hudDetectStart = now;
  }

  // — B. 小球运动与逃逸 —
  if (target_exists) {
    for (int i = TRAIL_LEN - 1; i > 0; i--) {
      trailX[i] = trailX[i - 1];
      trailY[i] = trailY[i - 1];
    }
    trailX[0] = target_x;
    trailY[0] = target_y;

    bool detected = (hud_state == HUD_REACT_DELAY || hud_state == HUD_CHASE || hud_state == HUD_LOCKED);
    float dx = hud_x - target_x, dy = hud_y - target_y;
    float halfw = 44 * hud_scale, halfh = 23 * hud_scale;
    float dist = sqrtf(dx * dx + dy * dy);

    //小球触发的随机扰动曲线
    if (!detected) {
      if (now - lastEvadeTurn > 2000 && random(100) < 20) {
        target_angle += random(20, 40) * (random(2) ? 1 : -1) * DEG_TO_RAD;
        lastEvadeTurn = now;
      }
      target_speed = PRE_DETECT_SPEED;
    } else {
      if (dist < halfw && dist < halfh && evadeStart == 0) {
        evadeStart = now;
        target_angle = atan2f(dy, dx);
      }
      if (evadeStart > 0 && now - evadeStart < EVADE_DURATION_MS) {
        float t = dist / max(halfw, halfh);
        target_speed = EVADE_SPEED * (1.0f + (1.0f - t) * 0.5f);
      } else {
        target_speed = PRE_DETECT_SPEED;
        evadeStart = 0;
      }
      if (now - lastEvadeTurn > 300) {                 // 每 500ms 微调一次角度
        target_angle += random(-80, 80) * DEG_TO_RAD;  // 每次微转 1~2 度
        lastEvadeTurn = now;
      }
    }

    // 转向缓动过渡
    float angle_diff = target_angle - angle_current;
    angle_diff = atan2f(sinf(angle_diff), cosf(angle_diff));  // 防止角度跳变
    angle_current += angle_diff * 0.03f;                      //“更粘滞”（更慢转），把 0.08f 改成 0.04f 或 0.02f。

    // if (click_flag == 1) {
    //   // 使用平滑后的 angle_current 计算位置
    //   target_x += cosf(angle_current) * target_speed;
    //   target_y += sinf(angle_current) * target_speed;
    // }

    // else {

    //   target_from_bno055(target_x, target_y);
    // }

    // 小球运动轨迹
    target_from_bno055(target_x, target_y);


    if (target_x < -4) target_x = WIDTH + 4;
    else if (target_x > WIDTH + 4) target_x = -4;
    if (target_y < -4) target_y = HEIGHT + 4;
    else if (target_y > HEIGHT + 4) target_y = -4;
  }


  // — D. HUD 状态控制 —
  bool wrapped = target_exists && (target_x < 0 || target_x > WIDTH || target_y < 0 || target_y > HEIGHT);
  if (wrapped && hud_state != HUD_LOCKED) {
    hud_state = HUD_IDLE;
    hudDetectStart = now;
    hud_vx = hud_vy = 0;
  }

  if (hud_state == HUD_IDLE) {
    float fx = (center_x - hud_x) * HUD_SPRING_K;
    float fy = (center_y - hud_y) * HUD_SPRING_K;
    hud_vx = (hud_vx + fx) * HUD_DAMPING;
    hud_vy = (hud_vy + fy) * HUD_DAMPING;
    hud_x += hud_vx;
    hud_y += hud_vy;
    hud_scale = 1.0f + BREATHE_AMP * sinf(now * BREATHE_SP);
    if (target_exists && !wrapped && encoder_idle && now - hudDetectStart >= DETECT_DELAY_MS) {
      hud_state = HUD_REACT_DELAY;
      hudReactStart = now;
    }
  } else if (hud_state == HUD_REACT_DELAY) {
    if (now - hudReactStart >= REACT_DELAY_MS) {
      hud_state = HUD_CHASE;
      captureStart = now;
      lockHoldStart = 0;
    }
  } else if (hud_state == HUD_CHASE) {
    float predx = target_x + cosf(target_angle) * target_speed * 4;
    float predy = target_y + sinf(target_angle) * target_speed * 4;
    float fx = (predx - hud_x) * HUD_SPRING_K;
    float fy = (predy - hud_y) * HUD_SPRING_K;
    hud_vx = (hud_vx + fx) * HUD_DAMPING;
    hud_vy = (hud_vy + fy) * HUD_DAMPING;
    hud_x += hud_vx;
    hud_y += hud_vy;

    hud_scale += (0.5f - hud_scale) * 0.08f + BREATHE_AMP * 0.5f * sinf(now * BREATHE_SP);

    if (random(1000) < 4) {
      hud_state = HUD_REACT_DELAY;
      hudReactStart = now;
      hud_vx = hud_vy = 0;
    }

    float d = hypotf(hud_x - target_x, hud_y - target_y);
    if (d < LOCK_DIST) {
      if (!lockHoldStart) lockHoldStart = now;
      else if (now - lockHoldStart >= LOCK_HOLD_MS) {
        hud_state = HUD_LOCKED;
        captureStart = now;
      }
    } else lockHoldStart = 0;
  } else if (hud_state == HUD_LOCKED) {
    float t = constrain((now - captureStart) / float(CAPTURE_MS), 0.0f, 1.0f);
    float ease = 1.0f - powf(1.0f - t, 3.0f);

    // 并行飞行：HUD 跟随但不吸附
    float dx = target_x - hud_x;
    float dy = target_y - hud_y;
    float k = 0.02f + 0.08f * ease;
    hud_vx = (hud_vx + dx * k) * HUD_DAMPING;
    hud_vy = (hud_vy + dy * k) * HUD_DAMPING;
    hud_x += hud_vx;
    hud_y += hud_vy;

    hud_scale += (0.3f - hud_scale) * 0.12f + BREATHE_AMP * 0.5f * sinf(now * BREATHE_SP);

    int lines = int(ease * 6);
    for (int i = 1; i <= lines; i++) {
      int sx = int(hud_x) - int(44 * hud_scale) + i * int(88 * hud_scale / (lines + 1));
      int sy = int(hud_y) - int(23 * hud_scale) + i * int(46 * hud_scale / (lines + 1));
      spr.drawFastVLine(sx, int(hud_y) - int(23 * hud_scale), int(46 * hud_scale), spr.color565(0, 200, 200));
      spr.drawFastHLine(int(hud_x) - int(44 * hud_scale), sy, int(88 * hud_scale), spr.color565(0, 200, 200));
    }

    if (now - lastBeep > 300) {
      // Turn_Beep();
      lastBeep = now;
    }

    if (now - captureStart >= CAPTURE_MS) {
      target_exists = false;
      eliminateTime = now;
      hud_state = HUD_IDLE;
    }
  }

  // — E. 敌机尾迹 —
  if (target_exists) {

    uint16_t ballC = spr.color565(255, 50, 50);  // 小球红色
    for (int i = TRAIL_LEN - 1; i >= 0; i--) {
      int tx = int(trailX[i]);
      int ty = int(trailY[i]);

      // ① 读取背景颜色（当前像素颜色）
      uint16_t bgC = spr.readPixel(tx, ty);  // ✅ 背景为 HUD 绿、线条、数字等皆可兼容

      // ② 根据透明度混合颜色（尾部越淡）
      uint8_t a = (255 * (TRAIL_LEN - i)) / TRAIL_LEN / 2;
      uint16_t c = spr.alphaBlend(a, ballC, bgC);

      // ③ 绘制混合颜色尾迹点
      spr.fillCircle(tx, ty, 3, c);
    }


    // ④ 绘制当前小球主体
    spr.fillCircle(int(target_x), int(target_y), 4, ballC);
  }


  // 缓动惯性跟随，✅ 让俯仰梯跟随小球轨迹变化
  float slow_k = 0.05f;  //惯性系数，越小越飘逸	0.05 ~ 0.1✅ 重要惯性响应参数
  // float slow_k = 0.4f;  // 更快速的传感器响应
  static float slow_x = WIDTH / 2;
  static float slow_y = HEIGHT / 2;
  // static float hudf_x = 0;
  // static float hudf_y = 0;
  //不再追踪小球，直接引用姿态传感器数据
  // target_from_bno055(hudf_x, hudf_y);
  slow_x += (hud_x - slow_x) * slow_k;
  slow_y += (hud_y - slow_y) * slow_k;

  draw_dial(spr, slow_x, slow_y, hud_x);


  // — F. HUD追踪器锁定框 绘制 —
  hud_alpha += (180 - hud_alpha) * 0.2f;
  hud_alpha = min(hud_alpha, 180.0f);
  uint16_t col = spr.alphaBlend((uint8_t)hud_alpha, TFT_CYAN, TFT_BLACK);
  int w = int(88 * hud_scale), h = int(46 * hud_scale), len = 8;
  int x0 = int(hud_x) - w / 2, y0 = int(hud_y) - h / 2;

  spr.drawFastHLine(x0, y0, len, col);
  spr.drawFastVLine(x0, y0, len, col);
  spr.drawFastHLine(x0 + w - len, y0, len, col);
  spr.drawFastVLine(x0 + w - 1, y0, len, col);
  spr.drawFastHLine(x0, y0 + h - 1, len, col);
  spr.drawFastVLine(x0, y0 + h - len, len, col);
  spr.drawFastHLine(x0 + w - len, y0 + h - 1, len, col);
  spr.drawFastVLine(x0 + w - 1, y0 + h - len, len, col);

  spr.drawFastVLine(int(hud_x), y0 - 20, h + 40, col);
  spr.drawFastHLine(int(hud_x) - w / 2 - 20, int(hud_y), w + 40, col);


  // === 🔺小三角形自由追踪系统（非绕中心） ===
  static float tri_x = center_x, tri_y = center_y;
  static float tri_vx = 0, tri_vy = 0;
  static float tri_angle = 0;
  static unsigned long tri_idle_since = 0;
  static bool tracking_mode = false;

  float radius = 68.0f;
  float spring_k = 0.02f;
  float damping = 0.85f;
  float max_speed = 4.0f;

  encoder_idle = (now - last_encoder_time > 500);

  if (encoder_idle) {
    if (!tracking_mode) {
      tracking_mode = true;
      tri_idle_since = now;
    }
  } else {
    tracking_mode = false;
  }

  // === 更新小三角位置 ===
  if (tracking_mode && target_exists) {
    // 自动追踪小球（不会触发逃逸）
    float dx = target_x - tri_x;
    float dy = target_y - tri_y;
    tri_vx = (tri_vx + dx * spring_k);
    tri_vy = (tri_vy + dy * spring_k);
  } else {
    // 编码器控制状态：回归指示角度位置
    float target_tx = center_x;
    float target_ty = center_y - radius;
    float dx = target_tx - tri_x;
    float dy = target_ty - tri_y;
    tri_vx = (tri_vx + dx * spring_k);
    tri_vy = (tri_vy + dy * spring_k);
  }

  // 限制最大速度（可选）
  float speed = sqrtf(tri_vx * tri_vx + tri_vy * tri_vy);
  if (speed > max_speed) {
    float scale = max_speed / speed;
    tri_vx *= scale;
    tri_vy *= scale;
  }

  // 施加阻尼 & 移动
  tri_vx *= damping;
  tri_vy *= damping;
  tri_x += tri_vx;
  tri_y += tri_vy;

  // 更新朝向
  float dx = (target_exists ? target_x : center_x) - tri_x;
  float dy = (target_exists ? target_y : center_y) - tri_y;
  tri_angle = atan2f(dy, dx);

  // === 绘制小三角形 ===
  float tip_len = 18.0f;    // 原10 → 放大为18
  float side_len = 12.0f;   // 原6 → 放大为12
  float angle_span = 0.3f;  // 张角保持

  float tip_xf = tri_x + cosf(tri_angle) * tip_len;
  float tip_yf = tri_y + sinf(tri_angle) * tip_len;
  float left_xf = tri_x + cosf(tri_angle - angle_span) * side_len;
  float left_yf = tri_y + sinf(tri_angle - angle_span) * side_len;
  float right_xf = tri_x + cosf(tri_angle + angle_span) * side_len;
  float right_yf = tri_y + sinf(tri_angle + angle_span) * side_len;


  uint16_t tri_color = spr.color565(255, 50, 50);
  spr.drawLine(tip_xf, tip_yf, left_xf, left_yf, tri_color);
  spr.drawLine(tip_xf, tip_yf, right_xf, right_yf, tri_color);
  spr.drawLine(left_xf, left_yf, right_xf, right_yf, tri_color);



  // === 航向滑轨（HUD 顶部）===
  static float heading_display = 0;

  float heading_target = fmodf(hud_x * 0.6f, 360.0f);
  // float heading_target = fmodf(slow_x * 0.6f, 360.0f);
  if (heading_target < 0) heading_target += 360.0f;

  // 插值缓动：让显示值逐渐逼近目标
  heading_display += (heading_target - heading_display) * 0.06f;  //中惯性（真实感）  0.06f ~ 0.09f

  // 归一化回 0~360 范围
  float heading = fmodf(heading_display + 360.0f, 360.0f);


  int xc = WIDTH / 2;
  int y_base = 55;          // 滑轨垂直位置
  int tick_spacing = 5;     // 每 5° 对应的像素间距
  int tick_len_short = 5;   //每 5° 短刻度线长度（无数字）
  int tick_len_long = 10;   // 每 10° 长刻度线长度（带数字）
  int visible_range = 60;   // 显示 ±60°
  int desired_width = 120;  // ✅ 控制滑轨视觉宽度（像素）

  int base_deg = ((int)(heading + 360 - visible_range) / 5) * 5;
  // uint16_t green = spr.color565(0, 255, 0);

  for (int d = -visible_range; d <= visible_range; d += 5) {
    int deg = (base_deg + d + 360) % 360;
    float dx = (d - (int)(heading) % 5) * tick_spacing;
    int x = xc + dx;

    // ✅ 限制滑轨只显示 desired_width 像素宽
    if (x < xc - desired_width / 2 || x > xc + desired_width / 2) continue;

    int tick_len = (deg % 10 == 0) ? tick_len_long : tick_len_short;
    spr.drawLine(x, y_base, x, y_base + tick_len, ladder_color);

    if (deg % 10 == 0) {
      spr.setTextDatum(MC_DATUM);
      spr.setTextFont(2);
      spr.setTextColor(ladder_color);
      spr.drawNumber(deg, x, y_base - 10);
    }
  }

  // 中心固定空心半三角指示器
  int arrow_y = y_base + tick_len_long + 6;
  spr.drawLine(xc - 6, arrow_y, xc, arrow_y - 6, ladder_color);
  spr.drawLine(xc + 6, arrow_y, xc, arrow_y - 6, ladder_color);

  // 小十字标记（在三角正下方）
  int cross_y = arrow_y + 8;
  spr.drawLine(xc - 4, cross_y, xc + 4, cross_y, ladder_color);
  spr.drawLine(xc, cross_y - 4, xc, cross_y + 4, ladder_color);
}


//aircraft状态指示器
void draw_status_block(TFT_eSprite &spr, int x, int y) {
  static uint32_t last_update = 0;
  uint32_t now = millis();
  float t = now / 2000.0f;  // 秒为单位

  // === 模拟数据生成（智能仿真模式） ===
  float alpha = 15.0f + 5.0f * sinf(t * 1.1f);      // α 迎角 10~20
  float Ma = 0.9f + 0.2f * cosf(t * 1.0f);          // Mach 0.6 ~ 1.2，真实巡航感
  float g = 3.5f + 0.5f * sinf(t * 1.8f);           // G 值 1~6
  float fuel = 9.0f - 4.0f * fabs(sinf(t * 0.1f));  // 模拟消耗再回升
  int altitude_ft = 1800 + 200 * sinf(t * 1.3f);    // 高度 1600~2000 ft

  // 🟢 缓动值（每帧平滑变化）
  static float spd = 150;    // 空速
  static float alt = 13000;  // 高度

  // 🔄 模拟目标值（正弦轻微波动）
  float tgt_spd = 155 + 5 * sinf(t * 0.7f);      // 目标空速
  float tgt_alt = 12800 + 150 * cosf(t * 0.5f);  // 目标高度

  // 🧮 缓动追踪（越小越稳）
  spd += (tgt_spd - spd) * 0.05f;
  alt += (tgt_alt - alt) * 0.05f;

  // 🎯 显示用整数
  int v_spd = round(spd);  // 显示用空速
  int v_alt = round(alt);  // 显示用高度


  // === 获取时间戳 ===
  uint32_t seconds = now / 1000;
  int h = (seconds / 3600) % 24;
  int m = (seconds / 60) % 60;
  int s = seconds % 60;
  char time_str[16];
  snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", h, m, s);

  // === 绘制 HUD 样式文字 ===
  // uint16_t green = spr.color565(0, 255, 0);
  spr.setTextFont(2);
  spr.setTextColor(ladder_color);  // ✅ 不指定背景色，透明叠加
  spr.setTextDatum(ML_DATUM);

  int line_h = 16;
  spr.drawString("a", x, y + line_h * 0);
  spr.drawFloat(alpha, 1, x + 20, y + line_h * 0);
  spr.drawString("M", x, y + line_h * 1);
  spr.drawFloat(Ma, 1, x + 20, y + line_h * 1);
  spr.drawString("G", x, y + line_h * 2);
  spr.drawFloat(g, 1, x + 20, y + line_h * 2);
  spr.drawFloat(fuel, 1, x + 20, y + line_h * 3);
  spr.drawString(time_str, x, y + line_h * 4);
  // ✅ 新增一行：FT 高度，整齐排在最后
  spr.drawString(String(altitude_ft) + " FT", x + 110, y + line_h * 0);

  int hx = WIDTH / 2;  // HUD 中心点
  int hy = HEIGHT / 2;

  int dx = 70;   // 横向偏移
  int dy = -20;  // 纵向偏移

  int bw = 48;  // 框宽
  int bh = 16;  // 框高

  int xl = hx - dx - 10;  // 左框位置
  int xr = hx + dx - 30;  // 右框位置
  int yb = hy + dy;       // 纵向对齐

  spr.setTextDatum(MC_DATUM);
  // 左侧空速框
  spr.drawRect(xl, yb, bw - 10, bh, ladder_color);
  spr.drawNumber(v_spd, xl + bw / 2, yb + bh / 2);

  // 右侧雷达高度框
  spr.drawRect(xr, yb, bw, bh, ladder_color);
  spr.drawNumber(v_alt, xr + bw / 2, yb + bh / 2);
}

//帧率刷新函数
void show_fps(TFT_eSprite &spr, int x = 10, int y = 10) {
  static uint32_t last_ms = 0;
  static uint16_t frames = 0;
  static float fps = 0.0;

  frames++;
  uint32_t now = millis();
  if (now - last_ms >= 1000) {
    fps = frames * 1000.0 / (now - last_ms);
    last_ms = now;
    frames = 0;
  }

  // 显示在屏幕左上角
  char buf[16];
  sprintf(buf, "FPS: %.1f", fps);

  spr.setTextColor(TFT_WHITE);  // 白色字，黑底
  spr.setTextFont(1);           // 小号字体
  spr.setTextDatum(TL_DATUM);
  spr.drawString(buf, x, y);
}

//屏幕极速镜像推送函数
void pushSpriteMirrorX(TFT_eSprite &spr, TFT_eSPI &tft, int x, int y) {
  if (spr.getColorDepth() != 16) return;

  int w = spr.width();
  int h = spr.height();
  uint16_t *buf = (uint16_t *)spr.getPointer();
  if (!buf) return;

  for (int yy = 0; yy < h; yy++) {
    uint16_t *row = &buf[yy * w];
    int left = 0;
    int right = w - 1;
    while (left < right) {
      uint16_t tmp = row[left];
      row[left++] = row[right];
      row[right--] = tmp;
    }
  }

  spr.pushSprite(x, y);
  // ❌ 删除了原本在这里的“Swap Back”循环，提升6帧
  // 因为下一帧开始时 spr.fillSprite(TFT_BLACK) 会重置所有像素
  // 所以不需要把 buffer 还原回去，这能省下一半的 CPU 时间！
  // for (int yy = 0; yy < h; yy++) {
  //   uint16_t *row = &buf[yy * w];
  //   int left = 0;
  //   int right = w - 1;
  //   while (left < right) {
  //     uint16_t tmp = row[left];
  //     row[left++] = row[right];
  //     row[right--] = tmp;
  //   }
  // }
}



//TFT显示
void words_display() {
  spr.fillSprite(TFT_BLACK);// 这里会清除上一帧的内容，所以镜像后的乱序无所谓

  ladder_color_index = 2;  //每4步更换一种颜色，限定颜色索引在 0~8 之间,避免负数导致越界
  ladder_color = bg_colors[ladder_color_index];


  draw_target_box(spr, WIDTH / 2, HEIGHT / 2);

  draw_status_block(spr, 8, HEIGHT - 5 * 16 - 10);  // ⬅️ 距底部预留约10像素

  show_fps(spr);
  spr.pushSprite(tft.width() / 2 - WIDTH / 2, tft.height() / 2 - HEIGHT / 2);
  // pushSpriteMirrorX(spr, tft, tft.width() / 2 - WIDTH / 2, tft.height() / 2 - HEIGHT / 2);  //HUD显示镜像
}
