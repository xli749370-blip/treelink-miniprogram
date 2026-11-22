const { buildFrame, buildActionFrame, parseTelemetry, stateText, ACTION } = require('../../utils/protocol.js');
const DroneSocket = require('../../utils/websocket.js');
const THEME = require('../../utils/theme.js'); // 主题色唯一来源（Canvas 取色用；CSS 侧见 app.wxss）
const sfx = require('../../utils/sfx.js');     // 按键反馈音效（WebAudio 合成，见 2026-11-15 预览稿 01 节）

// 与 firmware/control 的最大指令量保持一致
const VX_MAX = 1.5;   // 前后最大速度 m/s
const VY_MAX = 1.5;   // 左右最大速度 m/s
const YAW_MAX = 2.0;  // 最大偏航角速度 rad/s

const RAD2DEG = 57.2958;
const TELEM_MIN_MS = 100; // 遥测刷新节流：最多 10Hz，避免 setData/重绘过频
const GYRO_FULL_DEG = 30; // 相对基准姿态倾斜多少度 → 小球到达边缘（满量程）
// 约定基准握持姿态：屏幕「上仰 30°」（用户确认，横竖屏一致）。
// 注意：零位与转轴都由「开启陀螺仪那一帧」自适应得到，不依赖这个角度；
// 它只用于：①平放告警文案 ②给用户的握持提示。
const GYRO_REF_TILT_DEG = 30;
// 方向符号（x=左右，y=前后）。真机若发现某个方向仍是反的，只改这两个符号即可。
// 2026-10-16 用户实测：左右与实际相反 → x 取 -1（按要求把左右逻辑互换）
const GYRO_DIR = { x: -1, y: 1 };
// 用「时间常数」而非固定系数：与帧率/采样率无关，90/120Hz 高刷屏同样丝滑，
// 也不会因设备帧率不同而手感不一致。数值越小越跟手（越接近实时）。
const GYRO_SAMPLE_DT = 0.02;   // 传感器回调间隔(s)，interval:'game' ≈ 20ms（微信最快档）
const GYRO_ACC_TAU = 0.006;    // 原始值 → 目标位置(s)：只留必要抗噪，近乎直通（原 30ms 明显拖尾）
// —— 显示连续性（互补滤波，2026-10-23）——
// 旧方案 GYRO_RENDER_TAU=0.0015：τ 远小于帧间隔 → k≈1，等于零平滑，
// 50Hz 采样台阶 / 外推跳变 / 回中瞬移全部裸露 → 改为下面这对时间常数。
const GYRO_ANCHOR_TAU = 0.120;     // 锚定时间常数(s)：陀螺仪可用时。新样本跳变被摊到 ~120ms，肉眼不可见
const GYRO_ANCHOR_TAU_FB = 0.030;  // 锚定时间常数(s)：无陀螺仪回退路径（无可信速度，锚得紧一点保持跟手）
const GYRO_RATE_SHOW_TAU = 0.025;  // 显示专用角速度平滑(s)：只喂小球积分的角速度再轻滤一道，
                                   // 抹掉 50Hz 量化噪声在积分路径上的高频毛刺；控制用的 gyroRate 不动
const GYRO_DEAD = 0.012;       // 死区（归一化，≈0.35°）：只滤静止噪声，不再吃掉小幅度操作

// —— 实时性增强 ①：外推（补偿 50Hz 采样固有的滞后）——
// 微信加速度计最快只有 50Hz（≈20ms 一个样本），若直接显示，画面永远滞后约半个采样周期。
// 渲染时用「速度 × 已过去的时间」把目标外推到"现在"，小球跟的是手机**当前**姿态。
const GYRO_PREDICT_MAX = 0.035; // 最多外推 35ms：陀螺仪可用时速度是实测真值，可放得更远
// 额外前引（ms）：微信「传感器 → JS 逻辑层」管线本身有几十毫秒延迟，只补"样本陈旧时间"补不回来。
// 角速度是实测真值、停手即归零，所以静止时速率≈0、不产生任何偏移；只有真运动时才把画面/指令提前一点。
// 【怎么调】仿真扫描结论：最优前引 ≈ 该管线的实际延迟。取值参考——
//   0ms：管线很快时才合适；20~25ms：多数安卓机较优；40ms+：画面很跟手，但快速摆到头会略微过冲。
// 调法：觉得还不够跟手就加大（每次 +10ms），觉得"甩一下会弹一下"就减小。仅在陀螺仪可用时启用。
const GYRO_LEAD_EXTRA = 0.020;
const GYRO_VEL_TAU = 0.010;     // 回退路径（无陀螺仪）用：位置差分的平滑(s)
const GYRO_VEL_MAX = 12;        // 速度上限(1/s)，抑制噪声放大造成的过冲

// —— 实时性增强 ②：融合陀螺仪角速度（关键）——
// 加速度计是"位置"类信号：芯片内部低通 + 只能 50Hz 采样 → 天生滞后。
// 陀螺仪是"速度"类信号：直接给出此刻的瞬时角速度 → 没有位置信号的滞后，
// 且停手时立刻归零（外推不会过冲）。用它替代"位置差分估速度"来驱动外推，
// 既更准又能推得更远；同时两路 50Hz 数据互相填补，姿态更新率实际翻倍。
const GYRO_RSIN = Math.sin(GYRO_FULL_DEG / RAD2DEG); // 满量程倾角对应的 |r|
const GYRO_RATE_TAU = 0.012;    // 角速度轻平滑(s)：只去量化噪声，不引入可感知滞后
const GYRO_RATE_MIN = 0.20;     // 触发符号自检所需的最小角速度(1/s)
const GYRO_RATE_CONF = 0.35;    // 符号自检的相关度阈值：越大越稳、越小越快判定

// 指令下传周期(ms)：与传感器同速(50Hz) 才能让飞机实时跟上手机；原为 50ms(20Hz)。
// 若 UART/WiFi 链路吃紧，把这里改回 50 即可。
const SEND_MS = 20;

// ---- 按钮交互参数（2026-10-17 起：按下反馈与点击派发都在 JS 里自己维护）----
// 为什么不用 hover-class + bindtap：
//   ① hover-class 由基础库在原生侧实现，手指按下的时间短于一帧时（快速连点）类来不及挂上
//      → 「偶尔看不到按下反馈」；
//   ② bindtap 也在原生侧识别，连点时偶尔会漏掉一次 → 「按太快有时候不响应」。
// 改成：touchstart 立即挂 .btn-down，touchend 按位移判定是否算一次点击并派发动作。
const PRESS_MIN_MS = 90;    // 按下态最短可见时间(ms)：再快的点按也能看到键帽沉下去
const TAP_MOVE_MAX = 14;    // 手指位移超过这个值(px)就不算点击（滑动/滑开时不误触发）
const ARM_HOLD_MS = 500;    // 上锁需长按(ms)

// ---- 特技动作（落叶飘 / 空翻）----
// 触发型：抬手确认是点击后发 3 帧（同一个 seq），覆盖 ~80ms，抗一次 WiFi 丢包。
// 固件按 seq 去重，重复帧只会被执行一次 —— 所以多发不危险，漏发才危险。
const ACT_RESEND_MS = 40;
const ACT_RESEND_N = 3;

// ---- 垂直升降 ----
// 按住「爬升 / 下降」时下发的垂直速度(m/s)。0.4 / 0.3 是室内 12cm 小机的温和速度：
// 从 1.2m 爬到 2.0m 约 2s，既不会窜，也不至于让人等得不耐烦。
// 下降比上升更慢（0.3）：落地的最后 20cm 是最容易磕桨的一段，宁可慢。
// 松手立刻发 0 → 飞控的定高环把飞机停在**当前高度**（不是落回 1.2m），
// 所以这两个键永远不会造成"松手就掉/就窜"，最坏情况只是停在原地。
// ⚠️ 飞控侧必须做高度保护：上限建议 2.5m（到顶忽略 vz>0），
//    下限建议 0.25m（到地忽略 vz<0 并收油门）。这两条写在 control/ 的高度环里 ——
//    本按键不限制按住时长，只靠手指抬起来兜底是不可接受的。
const VZ_UP = 0.4;
const VZ_DOWN = 0.3;

// 摇杆位移的「按帧合并」窗口(ms)。
// touchmove 在双指同时拖动时可能每秒 200+ 次；若每次移动都 setData，桥接和视图层会被打满 → 掉帧。
// 16ms ≈ 一帧：一帧内所有变化攒成一次 setData（左右摇杆同时动也只发一次），观感仍是 60fps 跟手。
// 下发的是**一个 style 字符串**（不是 {x,y} 对象），视图层只改 transform，不触发重排。
const STICK_FRAME_MS = 16;

// 数字格式化：无效值统一显示为 '--'
function fx(v, digits) {
  if (v === null || v === undefined || v === '' || isNaN(Number(v))) return '--';
  return Number(v).toFixed(digits);
}

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

// 死区整形：|v| ≤ d 视为 0，超出部分线性映射回 0~1（满量程不被压缩）
function deadzone(v, d) {
  const a = Math.abs(v);
  if (a <= d) return 0;
  return (v > 0 ? 1 : -1) * Math.min(1, (a - d) / (1 - d));
}

// ============================================================================
// 启动动画说明（2026-10-23）：本页原「四点起飞」启动层已整体移除 ——
// 启动动画统一由模式选择页的 splash 承担（无人机=A 轨道停靠），
// 本页入场只保留根节点 scale .96→1 长入（.page-enter，配方同平衡车页）。
// ============================================================================

Page({
  data: {
    ip: '192.168.4.1',
    port: 81,
    connected: false,
    armed: false,
    gyroMode: false,
    // 垂直升降：按住 = 持续上升/下降，松手 = 停在当前高度（不是"点一下走一段"）。
    // 选按住式而不是点击式，是因为点击式必须回答"走多少"，而按住式的停止条件
    // 天然就是"手指抬起"—— 这是唯一不会让飞机自己跑掉的语义。
    // 用一个字符串而不是两个布尔量（climbing/descending）：方向是**互斥**的，
    // 两个布尔量一旦同时为 true 就会出现"一边升一边降"的无意义指令，
    // 而且视图层要检查两遍。空串 = 悬停（vz 发 0）。
    vzDir: '',       // '' | 'up' | 'down'
    // 自动模式开关（陀螺仪正上方两枚键）。不是"按住型"也不是"一次性动作"，
    // 而是**状态**：按下翻转一次，键帽持续高亮表示"当前开着"。
    // 状态放在 data（要驱动高亮），下发仍走动作帧（见 _fireMode / protocol.ACTION）。
    traceOn: false,  // 自动循迹
    avoidOn: false,  // 自动避障
    // 个性档位（1..3 循环）。个性切换系统尚未接固件 —— 目前只是本地档位：
    // 键上文字显示当前档位，点一下 +1 循环并给一次 0.64s 的一次性反馈（.tick）。
    // personaTick 驱动 .mode.persona.tick（外扩 + 键面柔光 + 双箭头错身 + 档位文字落回），
    // 连点重播见 _firePersona（先摘类、隔一帧再挂回，CSS 动画才能从头跑）。
    persona: 1,
    personaTick: false,
    // ---- 动效反馈（方案：design/visual-spec.html 05 动效层）----
    // 时长只有三档：0.6s 一次性 / 0.6s 按住循环 / 2s 系统在跑。见 index.wxss 动效层一节。
    busyKey: '',       // 特技/起飞动作执行中（0.6s 一次性动效）：'leaf' | 'flip' | 'takeoff' | ''
    takeoffShake: false, // 一键起飞未解锁点击的水平轻晃（0.34s）
    armFx: '',         // 解锁/上锁的一次性反馈：'burst'（外扩）| 'suck'（内收）| ''
    flip: false,       // 横竖屏切换瞬间的全屏扫光（0.55s 一次性）
    connecting: false, // 连接进行中（连接键 0.9s 扫描循环）
    // 升降标尺的实时演算值（_rlStep 每 16ms 算一次、差分 setData）：
    // h = 充能高度%（|v|×43）、top = 游标位置%（50 − v×43）、off = 刻度累计位移 px、
    // a = 三角/拖尾透明度、dir = 充能方向（跟 v 的符号，松手后随衰减保持、过零才消失）。
    // 松手后 v 衰减：刻度惯性减速停在哪算哪、游标平滑滑回中点 —— 解决"小球乱跳/刻度跳变"。
    rl: { h: 0, top: 50, off: 0, a: 0, dir: '' },
    // 跟随环的位移/缩放：与摇杆帽同一节奏（_flushKnob 里同帧写出）。
    leftRingStyle: '',
    rightRingStyle: '',
    isLand: false,   // 两种方向均为「陀螺仪居中放大 + 机身姿态右上」改版布局；横屏=三列左右摇杆，竖屏=垂直流式
    padTop: 0,       // 自定义导航栏下避让右上角「胶囊」的顶部内边距(px)
    status: '未连接',
    // 摇杆帽的位移：由这两个 style 字符串驱动（transform），只写这一个字段。
    // 更新节奏见 STICK_FRAME_MS —— 一帧最多一次，左右摇杆同时动也只发一次 setData。
    leftKnobStyle: '',
    rightKnobStyle: '',
    // 摇杆圆心与可动半径(px)：首次渲染/方向切换后量一次，之后每次按下直接用。
    leftCenter: { x: 0, y: 0, R: 0 },
    rightCenter: { x: 0, y: 0, R: 0 },
    // 注意：摇杆的归一化值（-1~1）**不放 data**。视图只用像素位移，
    // 归一化值只喂控制回路（_emit），放实例上即可 —— 每次下发能少一半数据。
    // 摇杆是否被按住：驱动"凹陷加深 + 内圈亮起 + 帽子微鼓"的按下反馈。
    leftActive: false,
    rightActive: false,
    // 当前被按下的按钮 key（'conn' / 'orient' / 'arm' / 'gyro' / 'climb' / 'descend'
    // / 'leaf' / 'flip'），空串=没有按钮按下。
    // 各按钮的 class 里写 {{pressed == 'x' ? 'btn-down' : ''}} —— 单一变量驱动全部按钮，
    // 一次 setData 只改一个字符串，比每个按钮一个布尔量便宜。
    pressed: '',
    telemetry: '等待连接…',

    // ---- 仪表数据 ----
    tm: { vx: '--', vy: '--', vyaw: '--', alt: '--', temp: '--', vbat: '--', yaw: '--' },
    roll: '0.0',
    pitch: '0.0',
    stateText: '待机',
    stateClass: 'st-idle',
    motors: [
      { i: 1, pct: 0 }, { i: 2, pct: 0 }, { i: 3, pct: 0 }, { i: 4, pct: 0 }
    ],
    rpmText: '',
    gyroTilt: { x: '0.0', y: '0.0' }
  },

  onLoad(options) {
    // —— 个性主题档位 ——
    // 冷启恒为个性1；从模式选择页进入时带 ?persona=N（用户在选页翻面选定的档）。
    // theme-N 类由 wxml 根节点按 persona 直挂，
    // canvas 配色走 THEME.applyTheme（同步改写 THEME 导出对象，绘制引用零改动）。
    const p = Math.min(3, Math.max(1, parseInt(options && options.persona, 10) || 1));
    if (p !== this.data.persona) this.setData({ persona: p });
    THEME.applyTheme(p);
    this._applyThemeChrome();

    this.socket = new DroneSocket();
    this.gyroV = null;
    this.gyroPos = { x: 0, y: 0 };     // 陀螺仪小球当前显示位置（归一化）
    this.gyroTarget = { x: 0, y: 0 };  // 陀螺仪小球目标位置（归一化）
    this.gyroAnim = false;
    this.gyroStopping = false;
    this._lastTelem = 0;
    this._lastLog = '';                // 上一条纯文本日志（相同就不重复 setData）
    // 按钮按下的时间线（见 onBtnDown/onBtnUp）：按 key 分开的按下记录 + 上锁长按定时器
    this._presses = {};
    this._holdTimer = null;
    this._connecting = false;          // 连接请求进行中（防连点重复建链）
    // 摇杆：归一化值（控制回路用）+ 待下发的像素位移 + 上次已下发的值（量化后去重）
    this._norm = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } };
    // 每个摇杆当前锁定的触摸 identifier（双指同时拖动时按它认领各自那根手指）
    this._touchId = { left: null, right: null };
    this._knobPend = {};
    this._knobSent = {};
    this._knobTimer = null;
    // 升降标尺实时演算（_rlStart/_rlStep）：v=归一化速率、tgt=目标(按住±1/松手0)、
    // off=刻度累计位移。泵只在有速率时跑，彻底停稳自动停。
    this._vzV = 0;
    this._vzTgt = 0;
    this._rlOff = 0;
    this._rlTimer = null;
    this._rlLast = 0;
    this._rlSent = '';
    // 遥测差分用的上一帧缓存：只把真正变了的字段塞进 setData（见 _onMessage）
    this._mc = {};
    // 指令帧复用同一个 ArrayBuffer：50Hz 下每秒省 50 次分配（见 protocol.buildFrame）
    this._frame = new ArrayBuffer(12);
    // 动作帧的序号：每次触发 +1（固件按 seq 去重，连发 3 帧只执行一次）
    this._actSeq = 0;
    // 陀螺仪角速度融合：轴向符号由「与加速度计对拍」自动判定，全页面生命周期只判一次
    this.gyroRateOn = false;
    this.gyroRate = { x: 0, y: 0 };    // 归一化角速度(1/s)，已带符号修正
    this.gyroRateShow = { x: 0, y: 0 };// 显示专用角速度（gyroRate 的轻平滑副本，只喂小球积分）
    this._rateSign = { x: 1, y: 1 };   // 陀螺仪轴向约定修正（设备属性，判定后不再改）
    this._rateDone = { x: false, y: false }; // 每轴是否已判定
    this._rateCommitted = false;       // 符号是否已判定完成
    this._rateCorr = { x: 0, y: 0 };   // 判定用的累计相关度
    this._rateTilt = null;             // 上一次「未死区/未平滑」的加速度计倾角
    // 自定义导航栏：先按当前方向算出避让「胶囊」的顶部内边距，避免首帧跳动
    this.setData({ padTop: this._safeTop(this._isLand()) });
    // 周期发送，保证摇杆/陀螺保持时持续下传；STM32 有断连停桨保护。
    // 与传感器同速(50Hz) 才能让飞机实时跟上手机姿态
    this.timer = setInterval(() => this._emit(), SEND_MS);
  },

  onReady() {
    this._applyOrientation(this._isLand());
  },

  // 屏幕旋转（横竖屏切换）时，重新判定方向并重建布局
  onResize(e) {
    const s = e && e.size;
    const land = s ? (s.windowWidth > s.windowHeight) : this._isLand();
    this._applyOrientation(land);
  },

  // 「切换横屏 / 竖屏模式」按钮：调用 API 尝试真正旋转屏幕；
  // 手机端 setPageOrientation 可能不被支持，但布局仍按 isLand 切换（onResize 会再次校正）
  onToggleOrientation() {
    // 连点防护：切一次方向要重建两套 Canvas（selectorQuery 重新取尺寸 + dpr 缩放）。
    // 前一次还没画完就再切，第二次查询可能拿到旧方向的尺寸 → 画布拉伸/空白。
    const now = Date.now();
    if (now - (this._oriAt || 0) < 350) return;
    this._oriAt = now;
    const target = !this.data.isLand;
    if (wx.setPageOrientation) {
      try {
        wx.setPageOrientation({ orientation: target ? 'landscape' : 'portrait' });
      } catch (e) { /* 不支持则仅切换布局 */ }
    }
    this._applyOrientation(target);
    // 动效反馈：全屏斜向扫光盖掉"啪"地跳变（0.55s 一次性，见 .page.flip::after）
    clearTimeout(this._flipTimer);
    this.setData({ flip: true });
    this._flipTimer = setTimeout(() => this.setData({ flip: false }), 600);
  },

  _isLand() {
    try {
      const info = (wx.getWindowInfo ? wx.getWindowInfo() : wx.getSystemInfoSync());
      return info.windowWidth > info.windowHeight;
    } catch (err) {
      return false;
    }
  },

  // 自定义导航栏 → 内容需避让右上角「胶囊」按钮，返回顶部安全内边距(px)
  _safeTop(land) {
    let bottom = 0;
    let winH = 0;
    try {
      const r = wx.getMenuButtonBoundingClientRect ? wx.getMenuButtonBoundingClientRect() : null;
      if (r && r.bottom) bottom = r.bottom;
    } catch (e) { /* ignore */ }
    try {
      const info = (wx.getWindowInfo ? wx.getWindowInfo() : wx.getSystemInfoSync());
      winH = info.windowHeight || 0;
      if (!bottom) bottom = (info.statusBarHeight || 20) + 32;
    } catch (e) { /* ignore */ }
    let pad = bottom + 6;
    // 横屏时部分机型该接口仍返回竖屏坐标，按视口高度比例封顶，避免吃掉过多纵向空间
    if (land && winH) pad = Math.min(pad, Math.round(winH * 0.14));
    return Math.max(0, Math.round(pad));
  },

  // 根据方向切换布局：两种方向均为「陀螺仪居中放大 + 机身姿态右上角」改版布局，
  // 仅排布不同（竖屏垂直流式、横屏三列左右摇杆）。两种方向都含 #hud 与 #gyro 画布，故全部重建
  _applyOrientation(land) {
    const changed = (land !== this.data.isLand);
    this.setData({ isLand: land, padTop: this._safeTop(land) }, () => {
      this._queryCenter('leftStick', 'leftCenter');
      this._queryCenter('rightStick', 'rightCenter');
      this._initHud();
      this._initGyroCanvas();
    });
    // 屏幕方向变化 ⇔ 使用者把手机拧了一下（横竖屏互转）。这次「拧动」本身就是一次
    // 很大的横滚，若沿用旧零位会让小球瞬间钉到边上、并给飞机一个满速侧移指令。
    // 因此：立即清空指令并丢弃零位，让下一帧按新的握持姿态重新标定（≈20~40ms）。
    // 只在方向真的变了时做，避免键盘/窗口微调触发的 resize 把正在飞的指令打断。
    if (changed && this.data.gyroMode) {
      this.gyroRef = null;
      this.gyroAxes = null;
      this.gyroTarget = { x: 0, y: 0 };
      this.gyroV = { vx: 0, vy: 0 };
      this._gyroSample = null;          // 别用旧姿态的样本/速度继续外推
      this._gyroVel = { x: 0, y: 0 };
      this.gyroRate = { x: 0, y: 0 };   // 拧手机本身是很大的转动，旧角速度不能留
      this.gyroRateShow = { x: 0, y: 0 };
      this._rateTilt = null;
    }
  },

  onUnload() {
    clearInterval(this.timer);
    clearTimeout(this._connTimer);      // 别让 8s 兜底定时器在页面销毁后 setData
    clearTimeout(this._gyroInitT);      // 画布尺寸重试定时器（防空白盘的兜底）也别漏
    clearTimeout(this._hudInitT);
    this._stopKnobTimer();
    this._clearHold();
    // 动效反馈的收尾定时器（特技 busy / 上锁 armFx / 切屏扫光 flip / 个性 tick / 起飞轻晃）
    clearTimeout(this._busyTimer);
    clearTimeout(this._tkShakeTimer);
    clearTimeout(this._armFxTimer);
    clearTimeout(this._flipTimer);
    clearTimeout(this._ptOn);
    clearTimeout(this._ptOff);
    clearInterval(this._rlTimer);       // 标尺演算泵
    this.socket.close();
    this._stopGyro();
  },

  // 量一次摇杆的圆心与可动半径。结果同时写实例字段（触摸计算用，零开销）
  // 和 data（仅在初始化/方向切换时发生一次，方便调试查看）。
  // 触摸过程中不再读布局：读一次 boundingClientRect 会强制同步布局，正好落在拖动的关键路径上。
  _queryCenter(id, key) {
    wx.createSelectorQuery().select('#' + id).boundingClientRect((rect) => {
      if (rect) {
        // R 为摇杆帽中心可移动半径：取基圆宽度的 0.30，保证行程够大、帽不溢出
        const geo = {
          x: rect.left + rect.width / 2,
          y: rect.top + rect.height / 2,
          R: rect.width * 0.30
        };
        this[key] = geo;
        this.setData({ [key]: geo });
      }
    }).exec();
  },

  // ---- 水平仪（人工地平仪）Canvas 2D 初始化 ----
  _initHud() {
    // 与 _initGyroCanvas 同一套防空白兜底：方向切换后布局未落定时延迟重试
    clearTimeout(this._hudInitT);
    this._hudInitTry = 0;
    wx.createSelectorQuery().select('#hud')
      .fields({ node: true, size: true })
      .exec((res) => {
        const r = res && res[0];
        if (!r || !r.node || !(r.width >= 2) || !(r.height >= 2)) {
          this._hudInitTry = (this._hudInitTry || 0) + 1;
          if (this._hudInitTry <= 10) {
            this._hudInitT = setTimeout(() => this._initHud(), 80);
          }
          return;
        }
        this._hudInitTry = 0;
        const canvas = r.node;
        const ctx = canvas.getContext('2d');
        let dpr = 2;
        try {
          dpr = (wx.getWindowInfo ? wx.getWindowInfo().pixelRatio
                                  : wx.getSystemInfoSync().pixelRatio) || 2;
        } catch (e) { dpr = 2; }
        canvas.width = r.width * dpr;
        canvas.height = r.height * dpr;
        ctx.scale(dpr, dpr);
        this.hud = { canvas, ctx, w: r.width, h: r.height };
        this._drawHud(Number(this.data.roll) || 0, Number(this.data.pitch) || 0);
      });
  },

  // roll: 右滚为正(度)；pitch: 抬头为正(度)。方向相反时把符号取反即可
  _drawHud(roll, pitch) {
    const hud = this.hud;
    if (!hud) return;
    const { ctx, w, h } = hud;
    if (!(w >= 2) || !(h >= 2)) return;   // 退化尺寸不画（同 _drawGyro）
    const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 2;
    const ppd = R / 45; // 每度对应像素：45° 填满半径

    ctx.clearRect(0, 0, w, h);

    // 天地背景（圆形裁剪）
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.clip();

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(roll * Math.PI / 180);
    ctx.translate(0, pitch * ppd);

    const BIG = R * 3;
    ctx.fillStyle = THEME.canvas.sky;          // 天空
    ctx.fillRect(-BIG, -BIG, BIG * 2, BIG);
    ctx.fillStyle = THEME.canvas.ground;       // 地面
    ctx.fillRect(-BIG, 0, BIG * 2, BIG);

    ctx.strokeStyle = THEME.canvas.hudLine;
    ctx.lineWidth = Math.max(1, R * 0.05);
    ctx.beginPath();
    ctx.moveTo(-BIG, 0);
    ctx.lineTo(BIG, 0);
    ctx.stroke();

    ctx.strokeStyle = THEME.canvas.hudTicks;
    ctx.fillStyle = THEME.canvas.hudNums;
    ctx.font = Math.max(6, Math.round(R * 0.2)) + 'px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let d = -30; d <= 30; d += 10) {
      if (d === 0) continue;
      const y = -d * ppd;
      const half = (d % 20 === 0) ? R * 0.55 : R * 0.32;
      ctx.beginPath();
      ctx.moveTo(-half, y);
      ctx.lineTo(half, y);
      ctx.stroke();
      ctx.fillText(String(Math.abs(d)), half + R * 0.24, y);
    }
    ctx.restore();

    // 固定机体符号（黄色）——尺寸按半径比例，保证横屏小尺寸仪表也不溢出
    ctx.strokeStyle = THEME.gyro;
    ctx.lineWidth = Math.max(1.5, R * 0.07);
    const a1 = R * 0.62, a2 = R * 0.22, tri = R * 0.16;
    ctx.beginPath();
    ctx.moveTo(cx - a1, cy); ctx.lineTo(cx - a2, cy);
    ctx.moveTo(cx + a2, cy); ctx.lineTo(cx + a1, cy);
    ctx.moveTo(cx - a2, cy); ctx.lineTo(cx, cy + tri);
    ctx.lineTo(cx + a2, cy);
    ctx.stroke();
    ctx.fillStyle = THEME.gyro;
    ctx.beginPath();
    ctx.arc(cx, cy, Math.max(1.5, R * 0.06), 0, Math.PI * 2);
    ctx.fill();

    ctx.restore(); // 解除裁剪

    // 外圈
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.strokeStyle = THEME.border;
    ctx.lineWidth = Math.max(1.5, R * 0.08);
    ctx.stroke();
  },

  // ---- 陀螺仪控制显示：大圆套小圆（小圆点 = 手机倾斜 → 无人机移动方向） ----
  // ⚠️ 方向切换后立刻查询，原生窗口重排（vh/vw 解析、新节点布局）可能尚未落定，
  // 会拿到 width/height=0 → canvas 尺寸置 0 → 永远画不出东西（空白盘）。
  // 浅色主题（T3）下空白盘直接读成"白色圆盘"——已修：尺寸非法就延迟重试。
  _initGyroCanvas() {
    clearTimeout(this._gyroInitT);
    this._gyroInitTry = 0;              // 新一轮初始化：重试计数从头算
    wx.createSelectorQuery().select('#gyro')
      .fields({ node: true, size: true })
      .exec((res) => {
        const r = res && res[0];
        if (!r || !r.node || !(r.width >= 2) || !(r.height >= 2)) {
          // 节点未挂载或布局未落定：80ms 后重试（最多 10 次 ≈ 0.8s）
          this._gyroInitTry = (this._gyroInitTry || 0) + 1;
          if (this._gyroInitTry <= 10) {
            this._gyroInitT = setTimeout(() => this._initGyroCanvas(), 80);
          }
          return;
        }
        this._gyroInitTry = 0;
        const canvas = r.node;
        const ctx = canvas.getContext('2d');
        let dpr = 2;
        try {
          dpr = (wx.getWindowInfo ? wx.getWindowInfo().pixelRatio
                                  : wx.getSystemInfoSync().pixelRatio) || 2;
        } catch (e) { dpr = 2; }
        canvas.width = r.width * dpr;
        canvas.height = r.height * dpr;
        ctx.scale(dpr, dpr);
        this.gyroCv = { canvas, ctx, w: r.width, h: r.height };
        // 碗形底盘的径向渐变只跟「圆心 + 半径」有关，这两个值在同一个画布生命周期里是死的 → 建一次就够。
        // 原来每帧都 createRadialGradient（60~120 次/秒），是开着陀螺仪时最贵的一笔开销。
        this.gyroCv.bowl = this._gyroBowl(ctx, r.width, r.height);
        // 静态底图（碗/沿/十字/死区/方向字）+ 小球精灵：每帧只剩 2 次贴图（见 _drawGyro 注）
        this._buildGyroBase();
        // 初始：关闭状态（小圆点居中灰）
        this._drawGyro(false, 0, 0);
      });
  },

  // 碗形底盘渐变：中心亮到边缘暗，形如凹进去的仪表盘。
  // 内圈中心放在盘心正上方 → 与全局「光从正上方来」的材质假设一致。
  _gyroBowl(ctx, w, h) {
    const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 3;
    const bowl = ctx.createRadialGradient(cx, cy - R * 0.35, R * 0.06, cx, cy, R * 1.04);
    bowl.addColorStop(0, THEME.canvas.panCenter);
    bowl.addColorStop(1, THEME.canvas.panEdge);
    return bowl;
  },

  // on: 是否开启陀螺仪；nx,ny ∈[-1,1]，来自手机横滚/俯仰归一化
  // 约定：上=前飞、下=后飞、左=左移、右=右移（与摇杆一致）
  //
  // 性能架构（2026-10-21 响应速度拉满）：
  //   每帧原本 = clearRect + 碗渐变填充 + 描边 + 十字 + 死区圈 + 4×fillText + 球(shadowBlur)。
  //   其中文本光栅化和 shadowBlur 是 canvas 最贵的两笔 —— 全部挪出每帧路径：
  //     · 静态底图（碗/沿/十字/死区/方向字）画进离屏 canvas 建一次，每帧一次 drawImage 贴图；
  //     · 小球（辉光+高光）预渲染成精灵，每帧一次 drawImage 平移贴到位置。
  //   每帧只剩 2 次贴图 + 1 次 clearRect，60/120fps 下都跑得动；不支持离屏画布的环境
  //   自动回退到旧的全量绘制（baseOk=0 / 精灵为 null），行为不变只是慢一点。
  _drawGyro(on, nx, ny) {
    const g = this.gyroCv;
    if (!g) return;
    const { ctx, w, h } = g;
    if (!(w >= 2) || !(h >= 2)) return;   // 布局未落定时拿到的退化尺寸：不画（等重试初始化）
    const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 3;

    // —— 背景：贴静态底图（回退：直接画静态层）——
    if (g.baseOk === 1 && g.base) {
      ctx.clearRect(0, 0, w, h);
      ctx.drawImage(g.base, 0, 0, g.canvas.width, g.canvas.height, 0, 0, w, h);
    } else {
      this._paintGyroStatic(ctx, w, h);
    }

    // —— 小球：贴预渲染精灵（回退：直接画）——
    const px = cx + nx * (R * 0.78);
    const py = cy - ny * (R * 0.78); // 上=前 → py 减小
    const spr = on ? (g.sprOn || this._buildGyroBall(true))
                   : (g.sprOff || this._buildGyroBall(false));
    if (spr) {
      ctx.drawImage(spr.cv, 0, 0, spr.phys, spr.phys, px - spr.half, py - spr.half, spr.size, spr.size);
    } else {
      this._paintGyroBall(ctx, on, px, py, Math.max(9, R * 0.16), R);
    }
  },

  // 静态层：碗 + 碗沿 + 十字 + 死区圈 + 方向字。同一画布生命周期内全部不变，
  // 且主题（颜色/虚线）切换时由 _retintCanvas 置 g.base=null 触发重建。
  _paintGyroStatic(ctx, w, h) {
    const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 3;
    const g = this.gyroCv;

    // 底盘：径向渐变的「碗」。纯色块在深色界面上永远是"贴纸"；
    // 碗形从中心亮到边缘暗，才像凹进去的仪表底盘。渐变已缓存在 g.bowl。
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.fillStyle = (g && g.bowl) || this._gyroBowl(ctx, w, h);
    ctx.fill();
    // 碗沿描边：渐变到边缘已经很暗，与页面底几乎同色（对比 1.1），
    // 必须靠一圈明确的边才看得出盘子的范围。用 stickEdge（对页面底 3.03，与摇杆底盘同一个色）。
    ctx.lineWidth = 3;
    ctx.strokeStyle = THEME.stickEdge;
    ctx.stroke();

    // 十字参考线
    ctx.strokeStyle = THEME.canvas.cross;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
    ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
    ctx.stroke();

    // 死区圈（倾斜小于该值视为回中）。虚线与否随主题性格：紫电=虚线（电感），其余实线
    ctx.beginPath();
    ctx.arc(cx, cy, R * 0.33, 0, Math.PI * 2);
    if (THEME.canvas.deadzoneDash) ctx.setLineDash([R * 0.055, R * 0.05]);
    ctx.strokeStyle = THEME.canvas.deadzone;
    ctx.stroke();
    if (THEME.canvas.deadzoneDash) ctx.setLineDash([]);

    // 方向标注：前/后/左/右（位置与字号按半径比例，横竖屏通用）
    ctx.fillStyle = THEME.canvas.dirLabel;
    ctx.font = Math.max(8, Math.round(R * 0.17)) + 'px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('前', cx, cy - R + R * 0.2);
    ctx.fillText('后', cx, cy + R - R * 0.07);
    ctx.fillText('左', cx - R + R * 0.16, cy + R * 0.05);
    ctx.fillText('右', cx + R - R * 0.16, cy + R * 0.05);
  },

  // 静态底图：把 _paintGyroStatic 画进离屏画布（物理分辨率 = 主画布，1:1 不重采样）。
  // 主题切换 / 画布重建时 g.base=null → 下一帧重建，重建成本一次性 ~1ms。
  _buildGyroBase() {
    const g = this.gyroCv;
    if (!g) return;
    g.base = null;
    g.baseOk = 0;
    try {
      if (typeof wx.createOffscreenCanvas !== 'function') return;
      const dpr = g.canvas.width / g.w;
      const off = wx.createOffscreenCanvas({ type: '2d', width: g.canvas.width, height: g.canvas.height });
      const octx = off.getContext('2d');
      octx.scale(dpr, dpr);
      this._paintGyroStatic(octx, g.w, g.h);
      g.base = off;
      g.baseOk = 1;
    } catch (e) { g.baseOk = 0; }
  },

  // 小球精灵：辉光 + 球体 + 高光一次渲染成小图，每帧只做一次 drawImage 平移。
  // shadowBlur 每帧执行是 canvas 最贵操作之一，精灵化后每帧 0 次。
  // 精灵烧着主题色（glowAccent/gyro/ballHi/gyroOff）→ 主题切换由 _retintCanvas 置空重建。
  _buildGyroBall(on) {
    const g = this.gyroCv;
    if (!g || typeof wx.createOffscreenCanvas !== 'function') return null;
    try {
      const R = Math.min(g.w, g.h) / 2 - 3;
      const r = Math.max(9, R * 0.16);
      const blur = on ? Math.max(8, R * (THEME.canvas.glowK || 0.16)) : 0;
      const pad = Math.ceil(blur * 1.6 + 4);          // 辉光在球外，留足余量防止被裁
      const size = Math.ceil((r + pad) * 2);          // 逻辑尺寸
      const dpr = g.canvas.width / g.w;
      const phys = Math.ceil(size * dpr);             // 物理尺寸（与主画布同密度，1:1 不重采样）
      const cv = wx.createOffscreenCanvas({ type: '2d', width: phys, height: phys });
      const c = cv.getContext('2d');
      c.scale(phys / size, phys / size);
      const cx = size / 2;
      if (on) {
        c.shadowColor = THEME.glowAccent;
        c.shadowBlur = blur;
        c.beginPath();
        c.arc(cx, cx, r, 0, Math.PI * 2);
        c.fillStyle = THEME.gyro;
        c.fill();
        c.shadowBlur = 0;
        // 左上高光：凸起的球必须有高光，否则和"贴在盘上的圆片"没区别
        c.beginPath();
        c.arc(cx - r * 0.3, cx - r * 0.34, r * 0.32, 0, Math.PI * 2);
        c.fillStyle = THEME.canvas.ballHi;
        c.fill();
      } else {
        // 关闭态：哑光灰球，不加高光、不加发光 —— 视觉上就是"没在工作"
        c.beginPath();
        c.arc(cx, cx, r, 0, Math.PI * 2);
        c.fillStyle = THEME.canvas.gyroOff;
        c.fill();
      }
      const spr = { cv, size, phys, half: size / 2 };
      if (on) g.sprOn = spr; else g.sprOff = spr;
      return spr;
    } catch (e) { return null; }
  },

  // 球的回退直画路径（环境不支持离屏画布时才走，行为与旧版一致）
  _paintGyroBall(ctx, on, px, py, r, R) {
    if (on) {
      ctx.save();
      ctx.shadowColor = THEME.glowAccent;
      ctx.shadowBlur = Math.max(8, R * (THEME.canvas.glowK || 0.16));
      ctx.beginPath();
      ctx.arc(px, py, r, 0, Math.PI * 2);
      ctx.fillStyle = THEME.gyro;
      ctx.fill();
      ctx.restore();
      ctx.beginPath();
      ctx.arc(px - r * 0.3, py - r * 0.34, r * 0.32, 0, Math.PI * 2);
      ctx.fillStyle = THEME.canvas.ballHi;
      ctx.fill();
    } else {
      ctx.beginPath();
      ctx.arc(px, py, r, 0, Math.PI * 2);
      ctx.fillStyle = THEME.canvas.gyroOff;
      ctx.fill();
    }
  },

  onIp(e) { this.setData({ ip: e.detail.value }); },
  onPort(e) { this.setData({ port: e.detail.value }); },

  onConnect() {
    if (this.data.connected) {
      this._connecting = false;
      this._connSeq = (this._connSeq || 0) + 1;   // 让在途回调失效
      this.socket.close();
      // 断开时同时停升降：断链后 _emit 直接 return，vzDir 若留着，
      // 重连的第一帧就会把飞机顶上去 / 按下去。
      this.setData({ connected: false, status: '已断开', vzDir: '' });
      this._vzRelease();   // 断链瞬间若正按着升降：标尺也要跟着衰减回中
      return;
    }
    // 连点防护：上一次"连接中"还没结束时不再重复建链。
    // DroneSocket.connect 会先 close 掉旧链，旧链的 onClose 随后又把新的「已连接」改回去，
    // 表现出来就是「按快了没反应」——所以这里必须挡掉，而不是让它重建。
    if (this._connecting) return;
    const { ip, port } = this.data;
    this._connecting = true;
    const seq = (this._connSeq = (this._connSeq || 0) + 1);
    const alive = () => seq === this._connSeq;    // 只认最新一次连接的回调
    this.setData({ status: '连接中…', connecting: true });  // connecting 驱动连接键的 0.9s 扫描
    this.socket.connect(ip, port, {
      onOpen: () => {
        if (!alive()) return;
        this._connecting = false;
        this.setData({ connected: true, status: '已连接 ✅', connecting: false });
      },
      onClose: () => {
        if (!alive()) return;
        this._connecting = false;
        this.setData({ connected: false, status: '连接关闭', vzDir: '', connecting: false });
      },
      onError: (err) => {
        if (!alive()) return;
        this._connecting = false;
        this.setData({ connected: false, status: '连接失败：' + ((err && err.errMsg) || '未知'), vzDir: '', connecting: false });
      },
      onMessage: (res) => this._onMessage(res)
    });
    // 兜底：部分机型连接迟迟不回调，别让按钮就此"永远没反应"，8s 后允许重新点
    clearTimeout(this._connTimer);
    this._connTimer = setTimeout(() => {
      if (!alive() || !this._connecting) return;
      this._connecting = false;
      if (!this.data.connected) this.setData({ status: '连接超时，可再点一次', connecting: false });
    }, 8000);
  },

  // ---- 接收：遥测文本 → 解析 JSON → 刷新仪表；非 JSON 则当日志 ----
  // 性能约定（摇杆跟手的另一半前提）：
  //   ① 节流放在最前面 —— 被丢掉的样本连 JSON.parse 都不做（链路 50Hz 时省掉 80% 解析）；
  //   ② 只 setData 真正变了的字段（差分），桥接载荷从「7+ 字段 + 4 个电机对象」降到通常 2~3 项；
  //   ③ 姿态角几乎没变就不重画 HUD（悬停微调时省掉整块画布的重绘）。
  // 三条都不改显示语义：变了的照样刷，没变的天生不需要刷。
  _onMessage(res) {
    if (!res || typeof res.data !== 'string') return;
    const now = Date.now();
    if (now - this._lastTelem < TELEM_MIN_MS) return; // 节流（含纯文本日志）
    this._lastTelem = now;

    const o = parseTelemetry(res.data);
    if (!o) {
      if (res.data !== this._lastLog) {               // 同一行日志被反复推送时不必重刷
        this._lastLog = res.data;
        this.setData({ telemetry: res.data });
      }
      return;
    }

    const roll = (o.roll !== undefined) ? Number(o.roll) : 0;
    const pitch = (o.pitch !== undefined) ? Number(o.pitch) : 0;

    const stCode = o.state || (o.armed ? 'ARMED' : 'IDLE');
    const arr = (o.mot && o.mot.length) ? o.mot : [0, 0, 0, 0];
    const pcts = [0, 1, 2, 3].map((i) =>
      Math.max(0, Math.min(100, Math.round(Number(arr[i]) || 0))));

    // 差分：_mc 里存上一帧「已经发出去」的值，只有变了才进 patch
    const mc = this._mc;
    const patch = {};
    const put = (k, v) => { if (mc[k] !== v) { mc[k] = v; patch[k] = v; } };

    put('tm.vx', fx(o.vx, 2));
    put('tm.vy', fx(o.vy, 2));
    put('tm.vyaw', fx(o.vyaw !== undefined ? o.vyaw * RAD2DEG : null, 0));
    put('tm.alt', fx(o.alt, 2));
    put('tm.temp', fx(o.temp, 1));
    put('tm.vbat', fx(o.vbat, 2));
    put('tm.yaw', fx(o.yaw, 0));
    put('roll', fx(roll, 1));
    put('pitch', fx(pitch, 1));
    put('stateText', stateText(stCode) || '待机');
    put('stateClass', 'st-' + String(stCode).toLowerCase());
    put('telemetry', '');
    // 电机柱：数组每次都是新对象（引用永远不等），用百分比串做指纹，没变就整组不重渲染
    const motKey = pcts.join(',');
    if (mc.mot !== motKey) {
      mc.mot = motKey;
      patch.motors = pcts.map((p, i) => ({ i: i + 1, pct: p }));
    }
    put('rpmText', (o.rpm && o.rpm.length >= 4)
      ? o.rpm.slice(0, 4).map((r, i) => 'M' + (i + 1) + ' ' + Math.round(Number(r) || 0)).join('  ')
      : '');

    if (Object.keys(patch).length) this.setData(patch);

    // 姿态角变化小于 0.05° 肉眼不可辨 → 不重画
    if (Math.abs(roll - (this._hudR || 0)) > 0.05 || Math.abs(pitch - (this._hudP || 0)) > 0.05) {
      this._hudR = roll;
      this._hudP = pitch;
      this._drawHud(roll, pitch);
    }
  },

  // ===== 摇杆 =====
  // 事件挂在 .stick-layer 上（catch 前缀 → 顺便掐掉拖动时的页面滚动/下拉刷新）。
  // 一次 touchmove 只做三件事：算偏移、写归一化值（实例字段，不进 data）、把像素位移
  // 交给下面的「按帧合并」。拖动过程每个摇杆每帧最多 1 次 setData，且只发一个 style 字符串。
  //
  // 双指同时拖动：两根手指按 identifier 各自认领一个摇杆，互不抢点。
  // （早前试过把位移整体搬到 WXS 视图层，但那条链路一旦在某机型上取不到节点就**没有回退**，
  //   表现为摇杆完全不响应。跟手收益不值得拿可用性去换，已撤回。）
  onStickStart(e) {
    const side = this._sideOf(e);
    if (!side) return;
    const t = (e.changedTouches && e.changedTouches[0]) || (e.touches && e.touches[0]);
    if (!t) return;
    this._touchId[side] = t.identifier;
    this._ensureCenter(side);      // 首次进页面/刚切完横竖屏时补量一次（量到后不再读布局）
    sfx.play('grab');              // 摇杆抓住：极轻 tick（-24dB 档，有则手感更实）
    // 按下帧的位置不要前沿直发（会拆成两次 setData）：_holdFlush 让 _track 只入队，
    // 随后 _flushKnob(extra) 把「位置 + 按下反馈」并进同一次 setData
    this._holdFlush = true;
    this._track(side, t);
    this._holdFlush = false;
    this._stopKnobTimer();
    this._flushKnob({ [side + 'Active']: true });
  },

  onStickMove(e) {
    const side = this._sideOf(e);
    if (!side) return;
    const t = this._touchOf(e, side);
    if (!t) return;                // 这一批触点里没有自己那根手指 → 不是我的事
    this._track(side, t);
  },

  onStickEnd(e) {
    const side = this._sideOf(e);
    if (!side) return;
    const id = this._touchId[side];
    if (id !== null && id !== undefined) {
      const c = e.changedTouches || [];
      let mine = false;
      for (let i = 0; i < c.length; i++) if (c[i].identifier === id) mine = true;
      if (!mine) return;           // 双指先后松开：另一根手指抬起不该把这个摇杆归零
    }
    this._touchId[side] = null;
    this._release(side);
    sfx.play('release');
  },

  _sideOf(e) {
    const d = e.currentTarget && e.currentTarget.dataset;
    const side = d && d.side;
    return (side === 'left' || side === 'right') ? side : '';
  },

  // 取「这一路摇杆自己那根手指」：先按 identifier 在 touches 里找，
  // 找不到再看 changedTouches（touchend 时那根手指只出现在这里）。
  _touchOf(e, side) {
    const id = this._touchId[side];
    const list = e.touches || [];
    const changed = e.changedTouches || [];
    if (id !== null && id !== undefined) {
      for (let i = 0; i < list.length; i++) if (list[i].identifier === id) return list[i];
      for (let i = 0; i < changed.length; i++) if (changed[i].identifier === id) return changed[i];
      return null;
    }
    return changed[0] || list[0] || null;
  },

  // 圆心没量到（首次进页面、刚切完横竖屏）就补一次；量到后不再重复读布局。
  _ensureCenter(side) {
    const c = this[side + 'Center'];
    if (!c || !(c.R > 0)) this._queryCenter(side + 'Stick', side + 'Center');
  },

  // 位移 → 视觉 + 控制量。
  //   视觉：交给「按帧合并」（_queueKnob），一帧只发一次 style 字符串；
  //   控制量：直接写实例字段，50Hz 的 _emit 每个周期自取，不需要 setData 参与。
  _track(side, t) {
    const c = this[side + 'Center'];
    if (!c || !(c.R > 0)) return;      // 圆心还没量到：这一帧不动，也绝不发算错的指令
    const px = (t.clientX !== undefined) ? t.clientX : t.pageX;
    const py = (t.clientY !== undefined) ? t.clientY : t.pageY;
    let dx = (Number(px) || 0) - c.x;
    let dy = (Number(py) || 0) - c.y;
    const R = c.R;
    const dist = Math.sqrt(dx * dx + dy * dy);
    // 限幅到圆内：斜推 45° 时两轴仍是满量程，不会因为"角上更远"变成超量程
    if (dist > R) { dx = dx / dist * R; dy = dy / dist * R; }
    const n = this._norm[side];
    n.x = dx / R;
    n.y = dy / R;
    this._queueKnob(side, dx, dy);
  },

  // 松手：控制量立刻归零（不等下一帧），回中 + 摘掉按下反馈合并成一次 setData。
  _release(side) {
    const n = this._norm[side];
    if (n) { n.x = 0; n.y = 0; }
    this._stopKnobTimer();
    this._knobPend[side] = { x: 0, y: 0 };
    this._flushKnob({ [side + 'Active']: false });
  },

  // ---- 摇杆位移：前沿即时 + 16ms 抑制窗（2026-10-24 响应提速）----
  // 旧配方每笔位移都等 16ms 定时器（平均 +8ms 延迟）；现在窗口空闲时第一笔
  // **立即下发（零延迟）**，窗口内的后续位移攒进 pend、窗口到期尾随补发——
  // 常态触控（60Hz）每笔都即时，高频触控（200Hz+）仍被压到每窗 ≤2 次 setData。
  // 位置量化到 0.5px —— 亚像素差异看不出来，却能省掉大量"其实没动"的下发。
  _queueKnob(side, x, y) {
    const qx = Math.round(x * 2) / 2;
    const qy = Math.round(y * 2) / 2;
    const last = this._knobSent[side];
    if (last && last.x === qx && last.y === qy) return;   // 与上次相同 → 不占用这一帧
    this._knobPend[side] = { x: qx, y: qy };
    if (this._knobTimer) return;                          // 抑制窗内：攒着，尾随统一发
    if (this._holdFlush) return;                          // 按下帧：并入 _flushKnob(extra) 一次发
    this._flushKnob();                                    // 前沿：零延迟直发
    this._knobTimer = setTimeout(() => {
      this._knobTimer = null;
      if (Object.keys(this._knobPend).length) this._flushKnob();   // 尾随：补发窗内积压
    }, STICK_FRAME_MS);
  },

  // 把攒下的位移一次性下发（左右摇杆同帧只发一次，每条只带一个字符串字段）。
  // 归零 = 松手回中：这一帧才挂上 transition（拖动帧绝不能有，否则帽子会追着手指拖尾）。
  // 跟随环与键帽同一帧、同一个位移：推得越远环收得越紧（scale 1.14 → 0.76），
  // 读作「行程余量」—— 连续输入配连续反馈，不做脉冲（呼吸会让人以为它在自己动）。
  _flushKnob(extra) {
    const pend = this._knobPend;
    const sides = Object.keys(pend);
    if (!sides.length && !extra) return;
    const d = extra || {};
    for (let i = 0; i < sides.length; i++) {
      const side = sides[i];
      const p = pend[side];
      const zero = !(p.x || p.y);
      d[side + 'KnobStyle'] = (p.x || p.y)
        ? 'transform: translate3d(' + p.x + 'px, ' + p.y + 'px, 0)'
        : 'transform: translate3d(0px, 0px, 0); transition: transform 140ms ease-out';
      // 环的缩放按归一化行程收：n = 位移/半径。归零帧（松手）回到松手态并挂过渡。
      // 收放幅度随个性主题走（theme.js motion）：紫电收得更紧，极昼更松。
      const mo = THEME.motion || {};
      const ringHi = mo.ringHi || 1.14;
      const ringSpan = mo.ringSpan || 0.38;
      const c = this[side + 'Center'];
      const R = (c && c.R > 0) ? c.R : 1;
      const n = Math.min(1, Math.sqrt(p.x * p.x + p.y * p.y) / R);
      d[side + 'RingStyle'] = 'transform: translate3d(' + p.x + 'px, ' + p.y + 'px, 0) scale('
        + (zero ? String(ringHi) : (ringHi - ringSpan * n).toFixed(3)) + ')'
        + (zero ? '; transition: transform 150ms ease-out' : '');
      this._knobSent[side] = p;
      delete pend[side];
    }
    // 按下/松手帧（extra 分支）：环的初态/回中态也要写，不能只靠 pend
    if (extra) {
      const mo = THEME.motion || {};
      const ringHi = mo.ringHi || 1.14;
      for (const k in extra) {
        const m = /^(\w+)Active$/.exec(k);
        if (!m) continue;
        const side = m[1];
        if (!d[side + 'RingStyle']) {
          d[side + 'RingStyle'] = extra[k]
            ? 'transform: translate3d(0px, 0px, 0) scale(' + ringHi + ')'
            : 'transform: translate3d(0px, 0px, 0) scale(' + ringHi + '); transition: transform 150ms ease-out';
        }
      }
    }
    this.setData(d);
  },

  _stopKnobTimer() {
    if (this._knobTimer) {
      clearTimeout(this._knobTimer);
      this._knobTimer = null;
    }
  },

  // ===== 按钮：按下反馈 + 点击派发（自己维护，不依赖 hover-class / bindtap）=====
  // 三个入口对应 wxml 里每个 button 的三个绑定：
  //   bindtouchstart="onBtnDown"  bindtouchend="onBtnUp"  bindtouchcancel="onBtnCancel"
  // 动作在 touchend 时才派发（抬手生效）：这是「按太快偶尔不响应」的根治办法 ——
  // 原生 tap 的识别逻辑会和 hover 态、长按、滑动判定互相抢；touchend 则一定会到。
  // 按下记录按 key 分开存（_presses[key]）：两根手指先后按不同按钮时互不覆盖。
  onBtnDown(e) {
    const key = this._btnKey(e);
    if (!key) return;
    const t = (e.touches && e.touches[0]) || (e.changedTouches && e.changedTouches[0]) || {};
    this._presses[key] = {
      at: Date.now(),
      x: Number(t.clientX) || 0,
      y: Number(t.clientY) || 0,
      long: false
    };
    if (this.data.pressed !== key) this.setData({ pressed: key });

    // 按键音（2026-11-15）：全部键按下给 16ms tick；升降是按住型，
    // 按下即"开始出力" → 用轻 servo 上扫（holdStart），松手对应 holdEnd
    if (key === 'climb' || key === 'descend') {
      if (this.data.armed) sfx.play('holdStart');
    } else {
      sfx.play('press');
    }

    // 垂直升降：按下当帧就进入"上升/下降"，不等抬手 —— 它们是按住型按钮（语义是"按住多久走多久"）。
    // 未解锁时给一次提示而不是静默失败：手指压着看不到文字，静默会让人以为程序卡了。
    if (key === 'climb' || key === 'descend') {
      if (!this.data.armed) {
        sfx.play('refuse');
        wx.showToast({ title: '请先解锁', icon: 'none', duration: 1000 });
        return;
      }
      const dir = key === 'climb' ? 'up' : 'down';
      if (this.data.vzDir !== dir) {
        // 后按的接管方向：单变量 = 天然互斥，不会同时"升又降"
        this.setData({ vzDir: dir });
        try { wx.vibrateShort({ type: 'light' }); } catch (err) {}  // 触觉确认：开始上升/下降
      }
      // 标尺实时演算：目标速率 ±1，泵启动后逐帧逼近（攻击快 = 跟手）
      this._vzTgt = dir === 'up' ? 1 : -1;
      this._rlStart();
      return;
    }

    // 上锁按钮：解锁态下按住 ARM_HOLD_MS = 上锁。
    // 用定时器自己实现（原 bindlongpress）→ 与按下态共用同一条时间线，不会再出现
    // "长按已经上锁、松手又被补发的 tap 解锁回去" 这种打架。
    if (key === 'arm' && this.data.armed) {
      this._clearHold();
      this._holdTimer = setTimeout(() => {
        this._holdTimer = null;
        const p = this._presses.arm;
        if (!p) return;                 // 中途松手/被打断，定时器已被清掉
        p.long = true;                  // 松手时不再补一次"点击"
        this._setArmed(false);
        sfx.play('lock');               // 上锁 thunk：危险操作听感要沉（与 heavy 振动同步）
        try { wx.vibrateShort({ type: 'heavy' }); } catch (err) {} // 触觉确认：机已停桨
      }, ARM_HOLD_MS);
    }
  },

  onBtnUp(e) {
    const key = this._btnKey(e);
    const p = key ? this._presses[key] : null;
    if (key) delete this._presses[key];
    if (key === 'arm') this._clearHold();
    // 按下态：不足 PRESS_MIN_MS 就延后摘掉，保证极快的点按也看得到反馈
    const elapsed = p ? Date.now() - p.at : PRESS_MIN_MS;
    const clear = () => {
      if (this.data.pressed !== key) return;        // 已被别的键接管
      if (this._presses[key]) return;               // 同一个键又被按下了
      this.setData({ pressed: '' });
    };
    if (elapsed < PRESS_MIN_MS) setTimeout(clear, PRESS_MIN_MS - elapsed);
    else clear();

    // 升降是按住型：抬手就停，不参与"点击判定/动作派发"。
    // 放在位移判定之前 —— 手指按住后轻微滑动（很常见）也必须停，不能漏。
    // 只停自己那一路：两指分别按着升降时，先松开的那个不许把另一个也停掉。
    if (key === 'climb' || key === 'descend') {
      this._stopVz(key === 'climb' ? 'up' : 'down');
      sfx.play('holdEnd');            // 升降松手：轻下扫收尾（卸力），比 holdStart 更轻
      return;
    }

    if (!p) return;
    // 位移过大 = 手指在滑动（或从按钮上滑开），不算点击
    const t = (e.changedTouches && e.changedTouches[0]) || {};
    if (t.clientX !== undefined) {
      const dx = (Number(t.clientX) || 0) - p.x;
      const dy = (Number(t.clientY) || 0) - p.y;
      if (Math.hypot(dx, dy) > TAP_MOVE_MAX) return;
    }
    if (p.long) return;             // 长按已生效，松手不再触发点击
    this._fireBtn(key);
  },

  // 触摸被系统打断（来电、手势返回、被父级滚动抢走）：只清状态，绝不派发动作
  onBtnCancel(e) {
    const key = this._btnKey(e);
    const kill = (k) => {
      delete this._presses[k];
      if (k === 'arm') this._clearHold();
      if (this.data.pressed === k) this.setData({ pressed: '' });
    };
    if (key) kill(key);
    else {                                  // 拿不到 key（防御）：全部清掉
      Object.keys(this._presses).forEach(kill);
      this._clearHold();
      if (this.data.pressed) this.setData({ pressed: '' });
    }
    // 触摸被系统打断（来电、手势返回）时，手指已经不在屏幕上了，
    // 但"升降"是按住型 —— 不能让它一直升/降下去，必须在这里收掉（不带方向 = 全停）。
    this._stopVz('');
  },

  // 停止升降：把意图交回定高环（vz=0 → 停在当前高度，不是落回 1.2m）
  // 传方向 = 只停那一路（另一个键还按着就继续）；传空 = 无条件全停（被打断/上锁/断连）。
  _stopVz(dir) {
    if (dir && this.data.vzDir !== dir) return;
    if (!this.data.vzDir) return;
    this.setData({ vzDir: '' });
    this._vzRelease();   // 标尺松手：速率衰减回中，不是瞬时归零
  },

  // ---- 升降标尺实时演算（2026-10-21 用户反馈：松手小球乱跳、刻度跳变）----
  // 旧实现是纯 CSS 类驱动：按住 = 0.6s 循环动画，松手 = 摘类 —— 循环动画在相位处
  // 瞬间归零（刻度跳一下）、游标 transition 的 1.42 过冲又会弹过头（读作乱跳）。
  // 现在改成速度模型：v 逐帧向目标逼近，位置/高度/偏移全部由 v 实时算出。
  _vzRelease() {
    this._vzTgt = 0;
    this._rlStart();
  },

  // 演算泵：16ms 一帧，只在有速率（按住中或衰减未完）时跑
  _rlStart() {
    if (this._rlTimer) return;
    if (this._vzV === 0 && this._vzTgt === 0) return;
    this._rlLast = Date.now();
    this._rlTimer = setInterval(() => this._rlStep(), 16);
  },

  _rlStep() {
    const now = Date.now();
    let dt = now - this._rlLast;
    this._rlLast = now;
    if (dt > 100) dt = 100;               // 后台切回/掉帧：一次别追太大步
    const tgt = this._vzTgt;
    // 攻击快（按住跟手）/ 释放慢（电机卸力的衰减感），系数按 dt 归一，掉帧不"追得慢"。
    // 系数随个性主题走（theme.js motion）：钢穹=机械克制、紫电=更脆、极昼=更绵。
    const prof = THEME.motion || { hold: 0.70, rel: 0.865 };
    const k = 1 - Math.pow(tgt !== 0 ? prof.hold : prof.rel, dt / 16.7);
    this._vzV += (tgt - this._vzV) * k;
    if (Math.abs(this._vzV) < 0.004 && tgt === 0) this._vzV = 0;
    const v = this._vzV;
    const av = Math.abs(v);
    // 刻度累计位移：满速 ≈100px/s（≈4 个主刻度周期/秒）。向上取负 = 刻度朝上流（与原方向一致）
    this._rlOff -= v * 1.7 * (dt / 16.7);
    if (av === 0 && tgt === 0) {          // 彻底停稳：写最终态并停泵
      clearInterval(this._rlTimer);
      this._rlTimer = null;
    }
    const rl = {
      h: Math.round(av * 430) / 10,             // 充能高度 %（|v|×43）
      top: Math.round((50 - v * 43) * 10) / 10, // 游标位置 %
      off: Math.round(this._rlOff * 2) / 2,     // 刻度位移 px（停在任意位置，不归零）
      a: Math.round(Math.min(1, av * 2.2) * 100) / 100,
      dir: v > 0.02 ? 'up' : v < -0.02 ? 'dn' : ''
    };
    const sig = rl.h + '|' + rl.top + '|' + rl.off + '|' + rl.a + '|' + rl.dir;
    if (sig !== this._rlSent) {         // 差分：值没变就不发
      this._rlSent = sig;
      this.setData({ rl });
    }
  },

  _btnKey(e) {
    return (e && e.currentTarget && e.currentTarget.dataset && e.currentTarget.dataset.key) || '';
  },

  _clearHold() {
    if (this._holdTimer) {
      clearTimeout(this._holdTimer);
      this._holdTimer = null;
    }
  },

  _fireBtn(key) {
    switch (key) {
      case 'conn': sfx.play('confirm'); this.onConnect(); break;
      case 'orient': sfx.play('confirm'); this.onToggleOrientation(); break;
      case 'gyro': this.onGyroToggle(); break;   // 开关音在 onGyroToggle 里按新状态给
      case 'arm': this.onArmTap(); break;        // 解锁 confirm / 上锁 lock 已在 _setArmed 与长按定时器
      // 特技动作是一次性指令：抬手确认是点击（不是滑开）之后才发
      case 'leaf':
      case 'flip': this._fireAction(key); break;
      // 一键起飞：与特技同一套处理（守卫 + 抬手触发 + busy 编排），码位 TAKEOFF=7 预留
      case 'takeoff': this._fireTakeoff(); break;
      // 自动模式是开关型：抬手确认是点击后，翻转状态并下发「开/关」
      case 'trace':
      case 'avoid': this._fireMode(key); break;
      // 个性键：换页面皮肤，与连接/解锁无关（全页唯一无飞行守卫的键，见 _firePersona）
      case 'persona': this._firePersona(); break;
      // climb / descend 是按住型：起停都在 onBtnDown / onBtnUp 里做完了，
      // 这里保留空分支只是为了 _fireBtn 覆盖全部 data-key（守护脚本会查）。
      case 'climb':
      case 'descend': break;
      default: break;
    }
  },

  // ---- 特技动作：落叶飘 / 空翻 ----
  // 与升降的分工：升降是「按住多久走多久」的连续量，走 50Hz 的 rc 帧；
  // 这里是「抖一下就完事」的一次性动作，走独立的动作帧（0x57），抬手时才发。
  //
  // 连发 3 帧、同一个 seq：WiFi 下丢一个包不至于让动作白按；固件按 seq 去重，
  // 多发不会翻两次 —— 所以"多发"是安全的，"漏发"才危险。
  // 5 字节的帧每帧新建一次，不做缓冲复用（怕底层异步持有同一块内存）。
  _fireAction(key) {
    if (!this._needArmed()) return;
    this._sendActionCode(key === 'leaf' ? ACTION.LEAF : ACTION.FLIP);
    sfx.play('whoosh');   // 动作发出：噪声上扫 + 亮 tick，与 busy 0.6s 编排同步
    // 动效反馈：0.6s 一次性「动作发出去了」（外扩 + 冲击波 + 三层光带 + 图标随动）。
    // 动画本身纯 CSS（.act.busy），这里只负责把 busyKey 挂上去、600ms 后摘下来。
    clearTimeout(this._busyTimer);
    this.setData({ busyKey: key });
    this._busyTimer = setTimeout(() => this.setData({ busyKey: '' }), 600);
  },

  // ---- 一键起飞：与 _fireAction 同一套处理，只有动作码和反馈编排不同 ----
  // 守卫拒绝时不弹错误沉默返回，而是让键水平轻晃一下（视觉禁用也要有反馈路径）。
  _fireTakeoff() {
    if (!this._needArmed()) { this._takeoffShake(); return; }
    this._sendActionCode(ACTION.TAKEOFF);   // 固件待接：ACTION 7 码位预留
    sfx.play('whoosh');   // 与特技同一支"动作发出"音色（守卫拒绝时 _needArmed 已给 refuse）
    // 起飞编排（0.6s）：双环外扩 + 蓄力光收束 + 光斑下扫 + 图标离地 + 跑道延伸 + 火花
    clearTimeout(this._busyTimer);
    this.setData({ busyKey: 'takeoff' });
    this._busyTimer = setTimeout(() => this.setData({ busyKey: '' }), 600);
  },

  _takeoffShake() {
    clearTimeout(this._tkShakeTimer);
    this.setData({ takeoffShake: true });
    this._tkShakeTimer = setTimeout(() => this.setData({ takeoffShake: false }), 340);
  },

  // ---- 自动模式：自动循迹 / 自动避障 ----
  // 与落叶飘/空翻的差别只有语义：那两个是"抖一下就完事"，这两个是**开关**，
  // 所以每按一次翻转本地状态，并把"开/关"编进 action 码 ——
  // 动作帧里只有 1 个 action 字节，没有第二个参数字节可用（见 protocol.ACTION）。
  // 下发通道完全一致（独立 0x57 帧、连发 3 帧同 seq、固件按 seq 去重）。
  _fireMode(key) {
    if (!this._needArmed()) return;
    const on = !this.data[key + 'On'];
    const patch = {};
    patch[key + 'On'] = on;
    this.setData(patch);
    sfx.play(on ? 'toggleOn' : 'toggleOff');   // 开关可听辨：上滑=开 / 下滑=关
    const code = key === 'trace'
      ? (on ? ACTION.TRACE_ON : ACTION.TRACE_OFF)
      : (on ? ACTION.AVOID_ON : ACTION.AVOID_OFF);
    this._sendActionCode(code);
  },

  // ---- 个性键：皮肤档位 3 档循环（1 → 2 → 3 → 1）----
  // 个性切换 = 换操作页面的皮肤（app.wxss 的 --t-* 主题变量整套换色），
  // 与连接/解锁完全无关 —— 没连上飞机也要能换肤，所以**不走 _needArmed() 守卫**，
  // 是全页唯一没有飞行守卫的键（见 wxml：persona 键也不挂 .off 灰化）。
  //
  // 同一枚键连点要能重启动画：CSS 动画只有"类被摘掉 → 重新挂上"才会从头跑，
  // 而 setData 的渲染合并没有强制回流的钩子 —— 所以先摘 tick，隔一帧（32ms）再挂回去。
  // 两个定时器都要可清：连点时上一次的"挂回/摘除"全部作废，以最新一次为准。
  _firePersona() {
    // 个性键 = 换肤：1 钢穹 → 2 紫电 → 3 极昼 → 1 循环。与连接/解锁无关，
    // 是全页唯一没有飞行守卫的键（守卫已移除）。档位同时是主题编号，
    // wxml 根节点据此挂 theme-N 类（CSS 侧换 token），canvas 侧走 THEME.applyTheme。
    const next = this.data.persona % 3 + 1;
    THEME.applyTheme(next);
    // 2026-10-23：不再持久化档位——每次进入小程序恒为个性1（onLoad 不读 storage）
    this.setData({ persona: next });
    sfx.play('persona', next);   // 玻璃 ping 三档音高：听得出换到了哪一格
    clearTimeout(this._ptOn);
    clearTimeout(this._ptOff);
    this.setData({ personaTick: false });
    this._ptOn = setTimeout(() => this.setData({ personaTick: true }), 32);
    this._ptOff = setTimeout(() => this.setData({ personaTick: false }), 32 + 640);
    this._retintCanvas();
    this._applyThemeChrome();
    // 触觉确认：换肤是"换了一格"，用最轻的一档
    try { wx.vibrateShort({ type: 'light' }); } catch (e) {}
  },

  // 换肤后强制重画两块 canvas：它们都带变化检测，静置时不会自己重绘，
  // 不推一把的话旧配色的圆盘/HUD 会一直挂着，直到下一次触摸/遥测才翻过来。
  _retintCanvas() {
    // 陀螺圆盘：底图与球精灵都烧着主题色 → 全部作废重建后重画。
    // 不设 gyroRef 条件 —— 圆盘的关闭态绘制不依赖它，换肤必须无条件刷新。
    const g = this.gyroCv;
    if (g) {
      g.base = null;
      g.bowl = null;      // 碗渐变烧着旧主题的 panCenter/panEdge，不清会带旧色重建底图
      g.sprOn = null;
      g.sprOff = null;
      this._buildGyroBase();
      try { this._drawGyro(false, 0, 0); } catch (e) {}
    }
    try { this._drawHud(Number(this.data.roll) || 0, Number(this.data.pitch) || 0); } catch (e) {}
  },

  // 导航栏 / 窗口底色在 page 元素之外（app.json 固定深色），换肤要跟着走；
  // 极昼（浅色）下导航栏文字必须换深色，白字在浅底上看不见。
  _applyThemeChrome() {
    const id = THEME.id || 1;
    const bg = id === 3 ? '#dde2ea' : id === 2 ? '#141221' : '#141821';
    const front = id === 3 ? '#1b2230' : '#ffffff';
    try { wx.setNavigationBarColor({ backgroundColor: bg, color: front }); } catch (e) {}
    try { wx.setBackgroundColor({ backgroundColor: bg }); } catch (e) {}
  },

  // 连接 + 解锁的守卫：两枚动作键、两枚模式键共用同一套提示。
  _needArmed() {
    if (!this.data.connected) {
      sfx.play('refuse');   // 低频双 buzz 与 toast 同步：读作"不行"
      wx.showToast({ title: '请先连接', icon: 'none', duration: 1000 });
      return false;
    }
    if (!this.data.armed) {
      sfx.play('refuse');
      wx.showToast({ title: '请先解锁', icon: 'none', duration: 1000 });
      return false;
    }
    return true;
  },

  // 动作帧连发：WiFi 下丢一个包不至于让动作白按；固件按 seq 去重，多发不会翻两次。
  _sendActionCode(code) {
    this._actSeq = (this._actSeq + 1) & 0xFF;
    const seq = this._actSeq;
    let left = ACT_RESEND_N;
    const shoot = () => {
      if (left <= 0) return;
      left--;
      if (!this.data.connected) return;      // 中途断链就别再挤了
      this.socket.send(buildActionFrame(code, seq));
      if (left > 0) setTimeout(shoot, ACT_RESEND_MS);
    };
    shoot();
    // 触觉确认：动作发出去的那一下能"摸到"。比 toast 轻，也不挡画面。
    try { wx.vibrateShort({ type: 'medium' }); } catch (e) {}
  },

  // ---- 上锁 / 解锁 ----
  // 上锁是危险操作：空中上锁 = 立即停桨坠落，而它与「开陀螺仪」等宽等高相邻。
  // 因此：解锁（上锁→解锁）短按即可；上锁（解锁→上锁）必须长按 ARM_HOLD_MS + 一次重振动
  // （长按本身由 onBtnDown 的定时器处理，这里只处理"点一下"的情况）。
  onArmTap() {
    if (this.data.armed) {
      wx.showToast({ title: '长按才能上锁', icon: 'none', duration: 1200 });
      return;
    }
    this._setArmed(true);
  },
  _setArmed(armed) {
    if (armed) sfx.play('confirm');   // 解锁 = 动作发出去了（双 tick 上行）；上锁的 lock 音在长按定时器里
    const patch = {
      armed,
      status: armed ? '已解锁，谨慎操作！' : '已上锁',
      // 动效反馈：解锁 = 一次性外扩（动作发出去了）；上锁 = 一次性内收（收拢）。
      armFx: armed ? 'burst' : 'suck'
    };
    // 上锁 = 立即停桨：升降意图必须同时清零。
    // 否则下次解锁的瞬间，上一轮残留的 vzDir 会让飞机自己升/降起来。
    // 自动模式同理：上锁后飞机不再执行任何自主机动，界面上的"开启中"高亮也必须灭掉，
    // 否则再解锁时人以为还开着，实际固件已经复位。
    if (!armed) {
      patch.vzDir = ''; patch.traceOn = false; patch.avoidOn = false;
      this._vzRelease();   // 上锁瞬间若正按着升降：标尺跟着衰减回中
    }
    this.setData(patch);
    clearTimeout(this._armFxTimer);
    this._armFxTimer = setTimeout(() => this.setData({ armFx: '' }), 600);
  },

  onGyroToggle() {
    const gyroMode = !this.data.gyroMode;
    this.setData({ gyroMode });
    sfx.play(gyroMode ? 'toggleOn' : 'toggleOff');   // 开关音色与循迹/避障同族
    if (gyroMode) this._startGyro();
    else this._stopGyro();
    // 触觉反馈：开启瞬间即代表"零位已标定"，给一次轻振动确认
    try { wx.vibrateShort({ type: 'light' }); } catch (e) {}
  },

  // ===== 陀螺仪姿态解算 =====
  // 目标（用户确认）：手机往前倒 → 小球向上、飞机前进；往后倒 → 小球向下、飞机后退；
  //                   往左倾 → 小球向左、飞机左移；往右倾 → 小球向右、飞机右移。
  // 做法与握持角度、横竖屏都无关：
  //   ① 开启首帧把重力方向 û 记为参考 ū → 那一刻小球居中、飞机悬停（零位标定）；
  //   ② 之后用旋转矢量 r = û × ū 表示「相对基准转了多少、绕哪根轴转」（|r| = sin 倾角）；
  //   ③ 把 r 分解到**两根世界水平轴**上——它们都与重力垂直，所以「转多少度就得到多少量」：
  //        ŵ_t：世界横向水平轴（= 世界上「右手方向」）→ 绕它转即为前后倒（远端下沉）
  //        ŵ_f：世界上「朝使用者」的水平轴（= 屏幕法线去掉竖直分量）→ 绕它转即为左右倾
  //   ④ 偏航（绕竖直轴转）与两轴都垂直 → 不产生任何输出，天然去耦。
  //
  // 为什么不用「把 r 投到屏幕上轴/右轴」的写法：左右倾的转轴就是屏幕法线方向，
  // 投影到屏幕平面恒为 0（竖持时左右倾完全失灵），而且灵敏度随握持仰角变化（屏面越平越钝）。
  _gyroAxes(gRef) {
    // ŵ_f = 屏幕法线去掉竖直分量后归一化。前提是手持时屏幕朝向使用者（无偏航），
    // 摇杆式操控天然满足；手机接近平放时该向量退化 → 用极限默认值（屏幕朝上、机顶朝远）
    const fm = Math.hypot(-gRef.z * gRef.x, -gRef.z * gRef.y, 1 - gRef.z * gRef.z);
    if (!(fm > 0.08)) {
      return { f: { x: 0, y: -1, z: 0 }, t: { x: 1, y: 0, z: 0 }, flat: true };
    }
    const f = {
      x: -gRef.z * gRef.x / fm,
      y: -gRef.z * gRef.y / fm,
      z: (1 - gRef.z * gRef.z) / fm
    };
    const t = { // ŵ_t = ĝ × ŵ_f（右手系：世界 x = y × z）
      x: gRef.y * f.z - gRef.z * f.y,
      y: gRef.z * f.x - gRef.x * f.z,
      z: gRef.x * f.y - gRef.y * f.x
    };
    return { f, t, flat: false };
  },

  // ---- 陀螺仪角速度：取「此刻的瞬时转动速率」，供外推使用 ----
  // 物理关系：绕 ŵ_f 转 = 左右倾速率，绕 ŵ_t 转 = 前后倒速率（rad/s → 归一化 1/s）
  _startRate() {
    if (typeof wx.startGyroscope !== 'function' || typeof wx.onGyroscopeChange !== 'function') {
      console.warn('[gyro] 当前环境不支持陀螺仪角速度，回退为「位置差分估速度」路径');
      return;
    }
    if (!this._rateSign) this._rateSign = { x: 1, y: 1 };        // 防御：正常由 onLoad 初始化
    if (!this._rateDone) this._rateDone = { x: false, y: false };
    if (!this._rateCorr) this._rateCorr = { x: 0, y: 0 };
    if (!this.gyroRate) this.gyroRate = { x: 0, y: 0 };
    this._gyroRateCb = (res) => this._onRate(res);
    try {
      wx.startGyroscope({
        interval: 'game',
        fail: () => { this.gyroRateOn = false; console.warn('[gyro] 陀螺仪启动失败，回退'); }
      });
      wx.onGyroscopeChange(this._gyroRateCb);
      this.gyroRateOn = true;
      console.log('[gyro] 陀螺仪角速度已开启（50Hz），用于实时外推');
    } catch (e) {
      this.gyroRateOn = false;
      console.warn('[gyro] 陀螺仪不可用，回退:', e && e.errMsg);
    }
  },

  _onRate(res) {
    const A = this.gyroAxes;
    if (!A || !this.gyroRef) return;          // 尚未零位标定 → 先丢弃
    const rf = res.x * A.f.x + res.y * A.f.y + res.z * A.f.z; // 左右倾速率
    const rt = res.x * A.t.x + res.y * A.t.y + res.z * A.t.z; // 前后倒速率
    const k = 1 - Math.exp(-GYRO_SAMPLE_DT / GYRO_RATE_TAU);
    const nx = GYRO_DIR.x * -rf / GYRO_RSIN * this._rateSign.x;
    const ny = GYRO_DIR.y * -rt / GYRO_RSIN * this._rateSign.y;
    this.gyroRate.x += (nx - this.gyroRate.x) * k;
    this.gyroRate.y += (ny - this.gyroRate.y) * k;
    // 显示专用副本：再轻滤一道（τ=25ms），只喂小球积分——50Hz 量化噪声在
    // 积分路径上表现为高频毛刺，这里抹掉它；控制用的 gyroRate 保持原样
    if (!this.gyroRateShow) this.gyroRateShow = { x: 0, y: 0 };
    const kS = 1 - Math.exp(-GYRO_SAMPLE_DT / GYRO_RATE_SHOW_TAU);
    this.gyroRateShow.x += (this.gyroRate.x - this.gyroRateShow.x) * kS;
    this.gyroRateShow.y += (this.gyroRate.y - this.gyroRateShow.y) * kS;
  },

  // 角速度轴向自检：刚体转动下「倾角变化率」必须与「角速度」同号。
  // 累加 倾角增量 × 角速度 的相关度，显著为负说明该轴约定相反 → 自动翻转。
  // 只在有真实动作时累加，判定一次后永久生效（设备属性）。
  _rateSignCheck(x, y) {
    const p = this._rateTilt;
    this._rateTilt = { x, y };
    if (!p || !this.gyroRateOn || this._rateCommitted) return;
    const decay = 0.97;
    const D = this._rateDone;
    if (!D.x && Math.abs(this.gyroRate.x) > GYRO_RATE_MIN) {
      this._rateCorr.x = this._rateCorr.x * decay + (x - p.x) * this.gyroRate.x;
    }
    if (!D.y && Math.abs(this.gyroRate.y) > GYRO_RATE_MIN) {
      this._rateCorr.y = this._rateCorr.y * decay + (y - p.y) * this.gyroRate.y;
    }
    for (const ax of ['x', 'y']) {
      if (D[ax] || Math.abs(this._rateCorr[ax]) < GYRO_RATE_CONF) continue;
      const s = this._rateCorr[ax] > 0 ? 1 : -1;
      D[ax] = true;                  // 每轴只判一次，锁定后不再反悔（避免来回翻）
      this._rateCorr[ax] = 0;
      if (s !== this._rateSign[ax]) {
        this._rateSign[ax] = s;
        this.gyroRate[ax] = 0;
        console.log('[gyro] 角速度轴 ' + ax + ' 与加速度计反号 → 已自动翻转');
      }
    }
    if (D.x && D.y) {
      this._rateCommitted = true;
      console.log('[gyro] 角速度符号自检完成 sign=', JSON.stringify(this._rateSign));
    }
  },

  // 零位标定：把当前姿态定为「小球居中 / 飞机悬停」的参考姿态
  _calibrateGyro(u) {
    this.gyroRef = { x: u.x, y: u.y, z: u.z };
    this.gyroAxes = this._gyroAxes(this.gyroRef);
    // 新参考点：清掉外推历史，避免沿用旧姿态算出巨大的速度而让小球窜出去
    this._gyroSample = null;
    this._gyroVel = { x: 0, y: 0 };
    this.gyroRate = { x: 0, y: 0 };
    this.gyroRateShow = { x: 0, y: 0 };// 旧姿态的显示角速度同样不能喂给小球
    this._rateTilt = null;
    const A = this.gyroAxes;
    const tilt = Math.acos(clamp(u.z, -1, 1)) * RAD2DEG; // 屏幕法线与竖直方向的夹角
    console.log('[gyro] 零位标定：屏幕法线与竖直夹角 ' + tilt.toFixed(0) + '° 平放=' + A.flat);
    if (A.flat) {
      console.warn('[gyro] 手机接近平放 → 无法判定屏幕「朝使用者」方向。请按约定「屏幕上仰约 ' +
        GYRO_REF_TILT_DEG + '°」拿稳后，把陀螺仪开关关闭再打开重新标定。');
      wx.showToast({ title: '请把屏幕上仰约 ' + GYRO_REF_TILT_DEG + '° 再开陀螺仪', icon: 'none' });
    }
  },

  // 把最新样本按「当前速度」外推到此刻，补偿 50Hz 采样固有的滞后。
  // 小球显示与下传指令**共用**这个值 → 二者始终同源（小球朝哪走，飞机就朝哪走）。
  // 速度优先取陀螺仪实测角速度（真值、停手即归零）；无陀螺仪时退回位置差分估计。
  // 未标定 / 非陀螺模式 / 回中过程中不外推，直接返回当前目标。
  _gyroLead() {
    const t = this.gyroTarget;
    const s = this._gyroSample;
    if (!s || !this.data.gyroMode || this.gyroStopping) return t;
    const age = (Date.now() - s.t) / 1000;
    if (!(age > 0)) return t;
    const gyro = this.gyroRateOn && this.gyroRate;
    const v = gyro ? gyro : this._gyroVel;
    // 外推量 = 「样本已陈旧的时间」+「额外前引」。
    // 额外前引只在有实测角速度时启用：它用来抵消微信「传感器→JS 逻辑层」的固有延迟
    // （只靠补陈旧时间补不回来）。角速度是实测真值且停手立刻归零，静止时不会产生偏移。
    const lead = Math.min(age, GYRO_PREDICT_MAX) + (gyro ? GYRO_LEAD_EXTRA : 0);
    return {
      x: clamp(s.x + v.x * lead, -1, 1),
      y: clamp(s.y + v.y * lead, -1, 1)
    };
  },

  // 手机往哪边倒，小球就往哪边走、飞机也往哪边走：
  //   前倒 → 小球上移 / 前进    后倒 → 小球下移 / 后退
  //   左倾 → 小球左移 / 左平移  右倾 → 小球右移 / 右平移
  _startGyro() {
    this.gyroV = { vx: 0, vy: 0 };
    this.gyroTarget = { x: 0, y: 0 };   // 目标位置（归一化；x=右为正，y=前为正）
    this.gyroPos = { x: 0, y: 0 };      // 实际显示位置，由动画循环平滑趋近
    this.gyroStopping = false;
    this.gyroRef = null;                // 待首帧采样后做零位标定
    this.gyroAxes = null;
    this._gyroSample = null;            // 最近一次传感器样本 {t,x,y}（供渲染外推）
    this._gyroVel = { x: 0, y: 0 };     // 目标变化速度(1/s)：无陀螺仪时的回退速度
    this.gyroRate = { x: 0, y: 0 };     // 陀螺仪实测角速度(1/s)：优先用它外推
    this.gyroRateShow = { x: 0, y: 0 };// 显示专用副本（见 _onRate）
    this._rateCorr = { x: 0, y: 0 };    // 符号自检相关度清零（未判定完才有意义）

    // 回调固定 50Hz('game') 触发 → 用固定 dt 换算，不依赖真实时间戳
    const kAcc = 1 - Math.exp(-GYRO_SAMPLE_DT / GYRO_ACC_TAU);
    const RSIN = GYRO_RSIN; // 满量程倾角对应的 |r|

    this._accCb = (res) => {
      // 归一化 → 兼容 iOS(g) 与 Android(m/s²) 的单位差异
      const mag = Math.sqrt(res.x * res.x + res.y * res.y + res.z * res.z) || 1;
      const u = { x: res.x / mag, y: res.y / mag, z: res.z / mag };
      this._lastU = u;
      if (!this.gyroRef) { this._calibrateGyro(u); return; } // 首帧只做零位标定

      const w = this.gyroRef, A = this.gyroAxes;
      // 旋转矢量 r = û × ū：|r| = sin(相对基准的倾角)，方向 = 绕哪根轴转
      const rx = u.y * w.z - u.z * w.y;
      const ry = u.z * w.x - u.x * w.z;
      const rz = u.x * w.y - u.y * w.x;
      const rF = rx * A.f.x + ry * A.f.y + rz * A.f.z; // 绕「朝使用者」水平轴 → 左右倾
      const rT = rx * A.t.x + ry * A.t.y + rz * A.t.z; // 绕「横向」水平轴   → 前后倒

      // 前倒（远端下沉）→ r·ŵ_t 为负 → ty 为正 → 小球上移、飞机前进
      // 右倾（右侧下沉）→ r·ŵ_f 为负 → tx 为正 → 小球右移、飞机右移
      const tyr = clamp(GYRO_DIR.y * -rT / RSIN, -1, 1); // 未死区/未平滑 → 供角速度符号自检
      const txr = clamp(GYRO_DIR.x * -rF / RSIN, -1, 1);
      this._rateSignCheck(txr, tyr);
      const ty = deadzone(tyr, GYRO_DEAD);
      const tx = deadzone(txr, GYRO_DEAD);

      // 极轻平滑（τ≈6ms）：几乎直通，只抹掉传感器毛刺，不引入可感知的滞后
      this.gyroTarget.x += (tx - this.gyroTarget.x) * kAcc;
      this.gyroTarget.y += (ty - this.gyroTarget.y) * kAcc;

      // 记录样本并估计变化速度 → 供渲染循环把目标外推到"现在"
      const now = Date.now();
      const prev = this._gyroSample;
      if (prev) {
        const dtS = Math.max(0.004, (now - prev.t) / 1000);
        const kv = 1 - Math.exp(-dtS / GYRO_VEL_TAU);
        const ivx = clamp((this.gyroTarget.x - prev.x) / dtS, -GYRO_VEL_MAX, GYRO_VEL_MAX);
        const ivy = clamp((this.gyroTarget.y - prev.y) / dtS, -GYRO_VEL_MAX, GYRO_VEL_MAX);
        this._gyroVel.x += (ivx - this._gyroVel.x) * kv;
        this._gyroVel.y += (ivy - this._gyroVel.y) * kv;
      }
      this._gyroSample = { t: now, x: this.gyroTarget.x, y: this.gyroTarget.y };

      // 控制量与小球**同源**：小球朝哪走，飞机就朝哪走，二者永远一致
      this.gyroV.vx = this.gyroTarget.y * VX_MAX; // 前倒 → 前进
      this.gyroV.vy = this.gyroTarget.x * VY_MAX; // 右倾 → 右移

      // 调试：约 2Hz 打印（只进控制台，不动界面），便于真机核对方向与量值
      if (now - (this._gyroDbg || 0) > 500) {
        this._gyroDbg = now;
        console.log('[gyro] raw=', res.x.toFixed(2), res.y.toFixed(2), res.z.toFixed(2),
          '| x右=', tx.toFixed(2), 'y前=', ty.toFixed(2),
          '| 指令 vx=', this.gyroV.vx.toFixed(2), 'vy=', this.gyroV.vy.toFixed(2),
          '| 角速度(1/s) rX=', this.gyroRate.x.toFixed(2), 'rY=', this.gyroRate.y.toFixed(2),
          this.gyroRateOn ? '融合中' : '回退(无陀螺仪)');
      }
    };

    wx.startAccelerometer({ interval: 'game' }); // 微信最快档（≈20ms / 50Hz）
    wx.onAccelerometerChange(this._accCb);
    this._startRate();                           // 叠加陀螺仪角速度（50Hz），用于实时外推
    this._gyroT = 0;                             // 供动画循环计算真实帧间隔
    // 预热开启态球精灵：第一次开陀螺的那一帧不做精灵构建（~1ms），避免可感知的首帧顿挫
    if (this.gyroCv && !this.gyroCv.sprOn) this._buildGyroBall(true);
    if (!this.gyroAnim) { this.gyroAnim = true; this._gyroLoop(); }
  },

  // 平滑动画循环：按「真实帧间隔」做指数逼近 → 高刷屏(90/120Hz)同样丝滑、连续、无瞬移
  _gyroLoop() {
    if (!this.gyroAnim) return;
    const now = Date.now();
    let dt = this._gyroT ? (now - this._gyroT) / 1000 : GYRO_SAMPLE_DT;
    this._gyroT = now;
    if (!(dt > 0) || dt > 0.1) dt = GYRO_SAMPLE_DT;   // 掉帧 / 后台休眠保护

    const on = this.data.gyroMode && !this.gyroStopping;
    const p = this.gyroPos;
    // —— 互补滤波（显示连续性核心）——
    // ① 连续驱动：角速度按真实帧间隔积分。样本间隔(20ms)里小球也在走，
    //    50Hz 采样在 60/120Hz 屏上的"台阶"被抹平；角速度停手即归零，不会过冲。
    // ② 低增益锚定：缓慢校正到外推目标（_gyroLead 与下传指令同源）。
    //    新样本到达时目标的小跳变（采样噪声 + 速度噪声×前引）被摊到
    //    GYRO_ANCHOR_TAU 时间常数里；角速度的积分漂移也由它兜底。
    // 回退路径（无陀螺仪）：没有可信速度，只留锚定（常数取小值，保持跟手）。
    // 关陀螺回中同样走锚定 → 真正平滑回中（旧 τ 下 k≈1 是瞬移）。
    const l = this._gyroLead();
    const rate = this.gyroRateOn ? this.gyroRateShow : null;  // 显示积分用轻平滑副本（更丝滑）
    if (rate) {
      p.x += rate.x * dt;
      p.y += rate.y * dt;
    }
    const kC = 1 - Math.exp(-dt / (rate ? GYRO_ANCHOR_TAU : GYRO_ANCHOR_TAU_FB));
    p.x += (l.x - p.x) * kC;
    p.y += (l.y - p.y) * kC;
    if (p.x > 1.05 || p.x < -1.05 || p.y > 1.05 || p.y < -1.05) {
      p.x = clamp(p.x, -1.05, 1.05);   // 防御：异常角速度也不飞出碗（锚定会立刻拉回）
      p.y = clamp(p.y, -1.05, 1.05);
    }
    this._drawGyro(on, p.x, p.y);
    // 关闭后平滑回中，到位即收尾（小球变灰居中）
    if (this.gyroStopping && Math.abs(p.x) < 0.01 && Math.abs(p.y) < 0.01) {
      p.x = 0; p.y = 0;
      this._drawGyro(false, 0, 0);
      this.gyroAnim = false;
      this.gyroStopping = false;
      return;
    }
    const cv = this.gyroCv;
    const next = () => this._gyroLoop();
    if (cv && cv.canvas && cv.canvas.requestAnimationFrame) {
      cv.canvas.requestAnimationFrame(next);   // 跟随屏幕刷新率（60/90/120fps）
    } else {
      setTimeout(next, 8);                     // 回退：尽量高频
    }
  },

  _stopGyro() {
    if (this._accCb) wx.offAccelerometerChange(this._accCb);
    wx.stopAccelerometer();
    if (this._gyroRateCb && wx.offGyroscopeChange) {
      try { wx.offGyroscopeChange(this._gyroRateCb); } catch (e) { /* ignore */ }
    }
    this._gyroRateCb = null;
    try { if (wx.stopGyroscope) wx.stopGyroscope(); } catch (e) { /* ignore */ }
    this.gyroRateOn = false;
    this.gyroRate = { x: 0, y: 0 };
    this.gyroRateShow = { x: 0, y: 0 };
    this._rateTilt = null;
    this.gyroV = { vx: 0, vy: 0 };
    if (this.gyroTarget) { this.gyroTarget.x = 0; this.gyroTarget.y = 0; }
    // 清掉零位与转轴：下次开启时按当时的握持姿态重新标定（需重新居中就直接关再开）
    this.gyroRef = null;
    this.gyroAxes = null;
    this._lastU = null;
    this._gyroSample = null;                                // 停外推，回中就是纯平滑
    if (this._gyroVel) { this._gyroVel.x = 0; this._gyroVel.y = 0; }
    this.gyroStopping = true;          // 让小球先平滑回中，再由循环收尾
    if (!this.gyroAnim) {              // 异常：未运行时直接复位
      this.gyroPos = { x: 0, y: 0 };
      this._drawGyro(false, 0, 0);
      this.gyroStopping = false;
    }
    this.setData({ gyroTilt: { x: '0.0', y: '0.0' } });
  },

  _emit() {
    if (!this.data.connected) return;
    let vx, vy;
    if (this.data.gyroMode) {
      // 与外推后的小球显示同源，且取到的是最新姿态（而非上一个 20ms 采样点）
      const l = this._gyroLead();
      vx = l.y * VX_MAX;
      vy = l.x * VY_MAX;
    } else {
      const ln = this._norm.left;
      vx = -ln.y * VX_MAX; // 上推 = 前进
      vy = ln.x * VY_MAX;  // 右推 = 右移
    }
    const rn = this._norm.right;
    const vyaw = rn.x * YAW_MAX; // 右推 = 顺时针偏航
    // 升降：按住给 ±速率，松手立即回 0（飞控据此停在当前高度）。
    // 上锁时强制 0 —— 即使 UI 状态因某种异常没清干净，也不许带着升降意图停放。
    const vz = !this.data.armed ? 0
      : this.data.vzDir === 'up' ? VZ_UP
      : this.data.vzDir === 'down' ? -VZ_DOWN
      : 0;
    // 复用同一个 ArrayBuffer：50Hz 下每秒少 50 次分配/回收，也少 50 次 GC 机会。
    // ⚠️ 复用要求 send 是「拷贝」语义（微信 WebSocket 是拷贝，安全）；若换成零拷贝的实现需改回新建。
    const buf = buildFrame(vx, vy, vyaw, this.data.armed ? 1 : 0, vz, this._frame);
    this.socket.send(buf);
  }
});
