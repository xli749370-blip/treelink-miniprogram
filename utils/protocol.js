// 指令帧编码，与 firmware/protocol.h 的 rc_frame_t 完全一致
// 帧格式（小端）：AA 55 | vx(i16,*100) | vy(i16,*100) | vyaw(i16,*1000) | vz(i16,*100) | arm(u8) | chk(u8)
// vx: 机体前后速度 m/s（正=前）；vy: 机体左右速度 m/s（正=右）
// vyaw: 偏航角速度 rad/s（正=顺时针）
// vz: 期望垂直速度 m/s（正=上升，0=保持当前高度）。由「爬升」按键按住时给正值，
//     飞控把它叠加在 ToF 定高环上：松手即回 0，飞机停在当前高度，不会掉。
// arm: 0=上锁停桨, 1=解锁

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

// ---------------------------------------------------------------------------
// 动作帧（手机 → 飞控）：一次性特技动作
//   AA 57 | action(u8) | seq(u8) | chk(u8)     共 5 字节，chk = 前 4 字节累加 & 0xFF
//   action: 1 = 落叶飘，2 = 空翻，3/4 = 自动循迹开/关，5/6 = 自动避障开/关
//           （与 firmware/protocol.h 的 ACT_* 一一对应）
//   seq   : 每次触发 +1；连发 3 帧用同一个 seq，固件按 seq 去重 → 只执行一次
//
// 为什么不复用 rc 帧：rc 帧是 50Hz 连发的连续控制量，长度 12 字节，
// 字段与校验被「小程序 / ESP32 / STM32 / test_chain.py」四处锁死。
// 往里加字段，只要有一端没同步就整体失步（2026-10-17 加 vz 时踩过这个坑）。
//
// 独立帧头 0x57 对旧固件是安全的：drv_rc_uart 的解析状态机在 hdr1 不认识时
// 直接 idx=0 重新找 0xAA → 动作帧被当噪声丢掉，rc 帧一帧不少。
// ---------------------------------------------------------------------------
// 动作码。1/2 是「抖一下就完事」的一次性动作；3~6 是「模式开关」，成对出现 ——
// 模式必须能表达开/关两态，而帧里只有 1 个 action 字节，所以用两个码表示同一个开关。
// 3~6 原为飞控侧预留（2026-10-24 起智能车页直接复用：循迹/避障就是这两个开关，
// 同一套车控固件）；7~10 是平衡车侧的预留码位（2026-10-23 落地 balance 页时分配，固件待接 ——
// 同 TAKEOFF=7 的处理：小程序先按约定发，车控固件跟上后即生效）：
//   平衡启停不走动作码 —— rc 帧的 arm 字节就是平衡开关（arm=1 平衡启动），
//   50Hz 连发自带状态确认，比一次性动作帧更可靠。
const ACTION = {
  LEAF: 1, FLIP: 2,              // 落叶飘 / 空翻
  TRACE_ON: 3, TRACE_OFF: 4,     // 自动循迹 开 / 关
  AVOID_ON: 5, AVOID_OFF: 6,     // 自动避障 开 / 关
  TAKEOFF: 7,                    // 一键起飞（固件待接：码位预留）
  HORN: 8,                       // 平衡车鸣笛（车控待接）
  LIGHT_ON: 9, LIGHT_OFF: 10     // 平衡车灯光 开 / 关（车控待接）
};
const ACTION_LEN = 5;

function buildActionFrame(action, seq, buf) {
  const b = (buf && buf.byteLength >= ACTION_LEN) ? buf : new ArrayBuffer(ACTION_LEN);
  const dv = new DataView(b);
  dv.setUint8(0, 0xAA);
  dv.setUint8(1, 0x57);
  dv.setUint8(2, action & 0xFF);
  dv.setUint8(3, seq & 0xFF);
  let chk = 0;
  for (let i = 0; i <= 3; i++) chk += dv.getUint8(i);
  dv.setUint8(4, chk & 0xFF);
  return b;
}

// buf 可选：传入一个已有的 12 字节 ArrayBuffer 就原地复用（50Hz 下省掉每秒 50 次分配）。
// 不传则照旧新建 —— 对调用方完全向后兼容。
function buildFrame(vx, vy, vyaw, arm, vz, buf) {
  const b = (buf && buf.byteLength >= 12) ? buf : new ArrayBuffer(12);
  const dv = new DataView(b);
  dv.setUint8(0, 0xAA);
  dv.setUint8(1, 0x55);
  dv.setInt16(2, clamp(Math.round(vx * 100), -32768, 32767), true);
  dv.setInt16(4, clamp(Math.round(vy * 100), -32768, 32767), true);
  dv.setInt16(6, clamp(Math.round(vyaw * 1000), -32768, 32767), true);
  dv.setInt16(8, clamp(Math.round((vz || 0) * 100), -32768, 32767), true);
  dv.setUint8(10, arm ? 1 : 0);
  let chk = 0;
  for (let i = 0; i <= 10; i++) chk += dv.getUint8(i);
  dv.setUint8(11, chk & 0xFF);
  return b;
}

// ---------------------------------------------------------------------------
// 遥测解析
// 飞控（STM32）经 ESP32 透传回发的文本消息，约定为单行 JSON，字段均可选：
//   state : 运行状态字符串，见 STATE_TEXT 键；armed: true/false
//   vx,vy,vz : 实际速度 m/s（正=前/右/上）
//   vyaw  : 偏航角速度 rad/s
//   roll,pitch,yaw : 姿态角（度）
//   alt   : 高度 m（光流/气压/ToF）
//   dist  : 前方障碍距离 cm（智能车超声波，选发；smartcar 页读数用）
//   temp  : MCU/IMU 内部温度 °C
//   vbat  : 电池电压 V
//   mot   : [4] 电机输出百分比 0–100
//   rpm   : [4] 电机转速 RPM（有转速反馈时才发，如双向 DShot）
// 示例：
//   {"state":"FLY","armed":true,"vx":0.35,"vy":-0.12,"vyaw":0.5,
//    "roll":-3.2,"pitch":1.8,"yaw":92.4,"alt":1.20,"temp":42.5,
//    "vbat":7.85,"mot":[45,46,44,45],"rpm":[12000,12050,11980,12010]}
// ---------------------------------------------------------------------------

const STATE_TEXT = {
  IDLE: '待机',
  CAL: '校准中',
  ARMED: '已解锁',
  FLY: '飞行中',
  HOLD: '定点悬停',
  AVOID: '避障中',
  LOWBAT: '低电告警',
  FAILSAFE: '失控保护',
  LAND: '降落中'
};

function stateText(code) {
  if (!code) return '';
  return STATE_TEXT[code] || String(code);
}

// 若消息不是合法 JSON 对象则返回 null（调用方按纯文本日志处理）
function parseTelemetry(str) {
  if (typeof str !== 'string') return null;
  const s = str.trim();
  if (!s || s.charAt(0) !== '{') return null;
  try {
    const o = JSON.parse(s);
    if (!o || typeof o !== 'object') return null;
    return o;
  } catch (e) {
    return null;
  }
}

module.exports = {
  buildFrame, buildActionFrame,
  parseTelemetry, stateText, STATE_TEXT,
  ACTION, ACTION_LEN
};
