// ============================================================================
// 主题 —— 界面配色与动效性格的唯一来源
// ============================================================================
// 小程序运行期读不到 .wxss，所以 Canvas（ctx.fillStyle 只接受具体色值）必须从
// 这里取色；而 CSS 侧的同一批颜色以 CSS 变量形式定义在 app.wxss 的 page{} 里。
//
//   命名规则：本文件 key 的 camelCase → app.wxss 里的 --t-kebab-case
//             例：bgBase  ⇄  --t-bg-base
//
//   【个性主题（2026-10-21）】
//     个性键切换的三套皮肤 = token 组 + 动效性格组。CSS 侧的换色在 app.wxss
//     （.page.theme-2 / .page.theme-3 覆盖块），动效形态覆盖在 index.wxss
//     「主题动效差异层」；Canvas 与 JS 系数在这里：
//     applyTheme(id) 把对应组的值**原地写进导出对象** —— index.js 里所有
//     THEME.xxx 引用（含 module 级缓存）不用改一行就拿到新值。
//
//   【配色纪律】状态色（ok/danger/warn/info/gyro）三套恒定 —— 「绿=正常」
//     这类语义不随皮肤漂移；换的只是中性面 + 自由强调色（accent 及其派生）。
//     T3 浅色主题的状态色单独加深，保住浅底上的白字对比。
//
//   【动效纪律】节奏三档（60ms 按下 / 0.6s 手指压着 / ≥2s 系统在跑）三套皮肤
//     共同遵守 —— 换的是"形态与包络"，不是节奏档位：
//       T1 钢穹 · 机械克制 —— 单脉冲、1.42 过冲（现行默认，动效无覆盖）
//       T2 紫电 · 电弧脉冲 —— 双闪、更陡的缓动、更脆的收放
//       T3 极昼 · 柔光长扩 —— 更淡更绵、幅度更大但透明度更低
// ============================================================================

// —— 第一套「钢穹」= 现役默认（app.wxss page{} 里的值逐条一致）——
const T1 = {
  bgBase:    '#141821',            // 页面 / 窗口底色
  // 页面背景三段渐变（仅 CSS 侧使用）。2026-10-24 与 T2 拉开色相：T1 往钢蓝推
  bgHi:      '#153050',            // 渐变亮端（页面上部）
  bgMd:      '#0d1e33',            // 渐变中段（52% 处）
  bgLo:      '#081322',            // 渐变暗端（页面下部）
  bgSurface: '#232a38',            // 卡片、摇杆底、圆盘底、输入框
  border:    '#2f3a4d',            // 描边 / 边框 / 分隔线
  text:      '#e6e9ef',            // 主文字
  textDim:   '#9aa4b2',            // 次级文字（标签、单位、说明）
  onAccent:  '#ffffff',            // 强调色底之上的反白文字

  // ---- 四种语义色：2026-10-17「包 A」统一压深（白字 4.83~4.84）----
  // ⚠️ 改这里的任何一个，都要检查白字对比度
  accent:    '#326fd2',            // 蓝 · 主操作（连接、摇杆帽、解锁态、按住中的爬升）
  ok:        '#12833c',            // 绿 · 解锁按钮 / 正常飞行
  danger:    '#dc2626',            // 红 · 上锁按钮 / 低电 / 失控
  warn:      '#ab5e05',            // 橙 · 避障警示
  info:      '#077c98',            // 青 · 降落
  gyro:      '#facc15',            // 黄 · 陀螺仪小球 / 地平仪机体符号
  shadow:    'rgba(0,0,0,0.5)',    // 摇杆帽投影

  // 摇杆底盘的描边：比通用 border 略亮（对页面底 3.03，过非文本图形 3.0）
  stickEdge: '#5d6574',
  // 电机柱体（中性钢灰，不是蓝）：蓝色专指「可操作」，转速是数据不是控件
  steel:     '#6b7a92',

  // ---- 材质结构色：质感层（凸起 / 凹陷 / 键帽厚度）----
  // 统一光源假设：光从【正上方】来。一律半透明黑/白，套在任何底色上不跑偏。
  edgeLight:  'rgba(255,255,255,0.12)',
  edgeDark:   'rgba(0,0,0,0.35)',
  shadeIn:    'rgba(0,0,0,0.5)',
  shadeOut:   'rgba(0,0,0,0.45)',
  keyCap:     'rgba(0,0,0,0.45)',
  gripOn:     'rgba(50,111,210,0.35)',
  glowAccent: 'rgba(50,111,210,0.5)',
  glowInfo:   'rgba(7,124,152,0.5)'
};

// —— 仅 Canvas 使用（CSS 用不到，故只在这里定义）——
const CANVAS = {
  hudLine:  '#ffffff',                // 地平仪地平线
  hudTicks: 'rgba(255,255,255,0.75)', // 地平仪刻度线
  hudNums:  'rgba(255,255,255,0.85)', // 地平仪刻度数字
  sky:      '#1d4ed8',                // 地平仪「天空」（语义色，三套恒定）
  ground:   '#7c4a1e',                // 地平仪「地面」（同上）
  cross:    'rgba(154,164,178,0.5)',  // 圆盘十字参考线
  deadzone: 'rgba(154,164,178,0.4)',  // 圆盘死区圈
  dirLabel: '#8c919c',                // 圆盘 前 / 后 / 左 / 右（对盘底 4.55）
  gyroOff:  '#74736f',                // 小球关闭态（对盘底 3.03）

  // ---- 陀螺仪圆盘：径向渐变的「碗」----
  // 碗心比 bgSurface 亮一档、碗沿比页面底还暗（把盘"压"进去）—— 随主题走
  panCenter: '#2a3242',
  panEdge:   '#191f28',
  ballHi:    'rgba(255,255,255,0.55)',

  // ---- 陀螺仪圆盘的绘制性格（_drawGyro 按字段分支）----
  deadzoneDash: false,   // 死区圈虚线（紫电=电感虚线，其余实线）
  glowK: 0.16            // 开启态小球的辉光半径系数（×盘半径；紫电更炸、极昼更含蓄）
};

// —— 动效性格（JS 侧系数；CSS 侧形态见 index.wxss「主题动效差异层」）----
// hold: 标尺速率逼近系数（按住时，越接近 1 越跟手）
// rel:  释放衰减系数（松手后，越大收得越快）
// ringHi / ringSpan: 摇杆跟随环的松手态 scale 与收放幅度（scale = ringHi − ringSpan×行程）
const MOTION_T1 = { hold: 0.70, rel: 0.865, ringHi: 1.14, ringSpan: 0.38 };

// —— 第二套「紫电」：自由色相换紫（#6d55d8）。
//    紫是状态色谱里唯一没被占用的色相，语义零冲突；面板偏紫炭。
const T2 = {
  bgBase:    '#141221',
  bgHi:      '#251746',   // 渐变底：「紫电」往紫罗兰推，与 T1 钢蓝拉开区分度
  bgMd:      '#160f2c',
  bgLo:      '#0b0718',
  bgSurface: '#221f38',
  border:    '#3a3560',
  text:      '#eae8f4',
  textDim:   '#a49ec0',
  onAccent:  '#ffffff',
  accent:    '#6d55d8',
  ok:        '#12833c',   // 状态色恒定
  danger:    '#dc2626',
  warn:      '#ab5e05',
  info:      '#077c98',
  gyro:      '#facc15',
  shadow:    'rgba(0,0,0,0.5)',
  stickEdge: '#6a6490',
  steel:     '#7b74a0',
  edgeLight:  'rgba(255,255,255,0.12)',
  edgeDark:   'rgba(0,0,0,0.35)',
  shadeIn:    'rgba(0,0,0,0.5)',
  shadeOut:   'rgba(0,0,0,0.45)',
  keyCap:     'rgba(0,0,0,0.45)',
  gripOn:     'rgba(109,85,216,0.35)',
  glowAccent: 'rgba(109,85,216,0.5)',
  glowInfo:   'rgba(7,124,152,0.5)'
};
const CANVAS_T2 = {
  panCenter: '#322a58',            // 紫调碗心（比 T1 更明显地"紫"）
  panEdge:   '#151221',            // 碗沿压进底色
  cross:     'rgba(164,158,192,0.5)',
  deadzone:  'rgba(164,158,192,0.42)',
  dirLabel:  '#a49ec0',            // 对紫炭盘底保正文对比
  gyroOff:   '#8a84a8',            // 关闭态球偏紫灰（与盘同族）
  deadzoneDash: true,              // 死区圈改虚线 —— 电感
  glowK: 0.24                      // 开启球辉光更炸
};
const MOTION_T2 = { hold: 0.80, rel: 0.905, ringHi: 1.10, ringSpan: 0.44 };   // 更跟手、环收得更紧

// —— 第三套「极昼」：明度反转浅色（户外强光下浅底可读）。
//    状态色整体加深保住浅底上的白字对比；材质阴影减淡。
const T3 = {
  bgBase:    '#dde2ea',
  bgHi:      '#f2f6fb',   // 渐变底：延用平衡车「冰川」页背景
  bgMd:      '#e2e9f3',
  bgLo:      '#d0d9e7',
  bgSurface: '#eef1f6',
  border:    '#c3cad6',
  text:      '#1b2230',
  textDim:   '#5a6472',
  onAccent:  '#ffffff',
  accent:    '#2456b3',
  ok:        '#0e7a34',
  danger:    '#c81e1e',
  warn:      '#96520a',
  info:      '#056a84',
  gyro:      '#efb800',            // 亮金黄 · 2026-10-23 用户反馈暗赭 #a97e08 太暗，提亮；
                                   // 浅碗（#f6f8fb）上靠 glowAccent 蓝晕 + 白高光勾出轮廓
  shadow:    'rgba(0,0,0,0.25)',
  stickEdge: '#8a93a3',
  steel:     '#7d8797',
  edgeLight:  'rgba(255,255,255,0.55)',
  edgeDark:   'rgba(0,0,0,0.22)',
  shadeIn:    'rgba(0,0,0,0.25)',
  shadeOut:   'rgba(0,0,0,0.18)',
  keyCap:     'rgba(0,0,0,0.22)',
  gripOn:     'rgba(36,86,179,0.3)',
  glowAccent: 'rgba(36,86,179,0.4)',
  glowInfo:   'rgba(5,106,132,0.45)'
};
const CANVAS_T3 = {
  panCenter: '#f6f8fb',            // 浅碗心（比 bgSurface 亮一档）
  panEdge:   '#ccd2dc',            // 碗沿比页面底暗一档（把盘"压"进去）
  cross:     'rgba(90,100,115,0.45)',
  deadzone:  'rgba(90,100,115,0.38)',
  dirLabel:  '#5a6472',
  gyroOff:   '#9aa0aa',
  ballHi:    'rgba(255,255,255,0.8)',
  deadzoneDash: false,
  glowK: 0.12                      // 浅底上光晕本就显淡，再收一档 = 含蓄
};
const MOTION_T3 = { hold: 0.58, rel: 0.82, ringHi: 1.20, ringSpan: 0.30 };    // 更绵软、环更松

const THEMES = {
  1: { shared: T1, canvas: {},        motion: MOTION_T1 },
  2: { shared: T2, canvas: CANVAS_T2, motion: MOTION_T2 },
  3: { shared: T3, canvas: CANVAS_T3, motion: MOTION_T3 }
};

// —— 导出对象：SHARED 平铺 + canvas / motion 子对象 ——
// applyTheme 把主题组的值**原地写进这个对象**，已有 THEME.xxx 引用零改动。
const CANVAS_BASE = Object.assign({}, CANVAS);   // T1 基准副本：切回默认时先重置，防性格字段残留
const exported = Object.assign({}, T1, { canvas: CANVAS, motion: MOTION_T1 });

function applyTheme(id) {
  const t = THEMES[id] || THEMES[1];
  for (const k of Object.keys(t.shared)) exported[k] = t.shared[k];
  for (const k of Object.keys(CANVAS_BASE)) CANVAS[k] = CANVAS_BASE[k];
  Object.assign(CANVAS, t.canvas);
  exported.motion = t.motion;
  exported.id = id;
}
exported.applyTheme = applyTheme;

module.exports = exported;
