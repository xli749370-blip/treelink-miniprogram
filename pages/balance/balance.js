// ===========================================================================
// 平衡车控制页（2026-10-23 落地自 design/balance-car-preview.html 定稿）
// ---------------------------------------------------------------------------
// 架构移植自无人机页已验证体系，差异点：
//   · 单摇杆（行驶：前后速度 + 左右转向）→ rc 帧的 vx / vyaw，vy 恒 0（两轮车无横移）
//   · 平衡启停 = rc 帧 arm 字节（50Hz 连发自带状态确认，不占用动作码）
//   · 灯光 / 鸣笛 = 独立动作帧（ACTION.LIGHT_ON/OFF / HORN，码位已预留，车控固件待接）
//   · 档位纯本地（缓行 4 / 标准 8 / 运动 12 km/h）：它只是vx的缩放上限，
//     固件收到的永远是绝对速度，无需下发档位
//   · 断连 / 车控未回传时仪表走本地演示物理（与预览页同一套模型），链路通了自动切换真值
// 性能约定（与无人机页同源）：
//   摇杆 16ms 按帧合并 + 0.5px 量化；速度/泡 rAF + setInterval(16) 双路；
//   读数 10Hz、泡 60Hz（差分 setData）；速度弧 canvas 直绘带变化检测。
// ===========================================================================
const { buildFrame, buildActionFrame, parseTelemetry, ACTION } = require('../../utils/protocol.js');
const DroneSocket = require('../../utils/websocket.js');
const sfx = require('../../utils/sfx.js');     // 按键反馈音效（三页共用，见 utils/sfx.js 头注）

const SEND_MS = 20;          // 指令下传周期(ms)，与无人机页一致
const TELEM_MIN_MS = 100;    // 遥测刷新节流：最多 10Hz
const YAW_MAX = 2.0;         // 最大偏航角速度 rad/s（与无人机 rc 帧口径一致）
const BATT_FULL_V = 12.6;    // 3S 满电电压：电量条按 vbat/12.6 折算

// ---- 按钮交互（与无人机页同一套：touchstart 挂按下态、touchend 派发）----
const PRESS_MIN_MS = 90;     // 按下态最短可见时间(ms)
const TAP_MOVE_MAX = 14;     // 手指位移超过这个值(px)不算点击
const MAIN_HOLD_MS = 800;    // 平衡主键长按关闭(ms)
const MAIN_TAP_MIN_MS = 120; // 短于这个时长视为误触 → 拒绝闪烁

// ---- 档位（名称 / 极速 km/h / 表针转角）----
const GEAR = [
  { name: '缓行', max: 4,  deg: -40 },
  { name: '标准', max: 8,  deg: 0 },
  { name: '运动', max: 12, deg: 40 }
];

// ---- 速度弧几何（预览 viewBox 258：r=122，弧从左下经顶到右下）----
// canvas 直绘按 dial 实测尺寸等比缩放，横竖屏同一套代码
const ARC_A0_DEG = 138.53;   // 起始角（左下，y 向下坐标系）
const ARC_SPAN_DEG = 262.94; // 弧总张角（经顶部到右下）

// 摇杆位移的按帧合并窗口(ms)：一帧最多一次 setData（无人机页经验）
const STICK_FRAME_MS = 16;

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

Page({
  data: {
    isLand: false,
    padTop: 0,          // 自定义导航栏避让(px)
    flip: false,        // 切屏瞬间的全屏扫光
    persona: 1,         // 个性档位 1/2/3（皮肤 + 背景随根节点 theme-N 类切换）

    // ---- 连接（复用 DroneSocket：车控 ESP32 与飞控同款 WebSocket 透传）----
    ip: '192.168.4.1',
    port: 81,
    connected: false,
    connecting: false,
    connText: '未连接',

    // ---- 状态 ----
    running: false,     // 平衡是否启动（绿键面 / 状态字 / 速度弧颜色都吃这个态）
    lightOn: false,
    gear: 1,
    gearName: '标准',
    gearDeg: 0,
    balStateText: '平衡未启动',

    // ---- 仪表显示（低频 setData 值；高频值走实例字段 + canvas/transform）----
    spdText: '0.0',
    tiltText: '+0.0',
    tiltOver: false,    // 倾角超限（|tilt|>10°）：泡与读数变红
    bubbleStyle: '',    // 泡的 translate（60Hz 差分）
    tm: { vbat: '--', temp: '--' },
    battPct: 0,
    telemetryText: '等待连接…',

    // ---- 摇杆视图 ----
    knobStyle: '',
    stickActive: false,
    stickFwd: false,
    stickBack: false,

    // ---- 按钮按下态（单一变量驱动全部键，同无人机页）----
    pressed: '',
    // 一次性特效类（外扩 burst / 内收 suck），挂在各键的 .fx-ring 上
    lightFx: '', gearFx: '', hornFx: '', mainFx: '',
    mainRefuse: false   // 主键拒绝闪烁（红框零位移）
  },

  onLoad(options) {
    // 个性档位：冷启恒 1；从模式选择页进入时带 ?persona=N（选页翻面选定的档）
    const p = Math.min(3, Math.max(1, parseInt(options && options.persona, 10) || 1));
    if (p !== this.data.persona) this.setData({ persona: p });
    this.socket = new DroneSocket();
    // 摇杆：归一化值（控制回路用，不进 data）+ 触摸 identifier + 几何缓存
    this._n = { x: 0, y: 0 };
    this._touchId = null;
    this._center = { x: 0, y: 0, R: 0 };
    this._knobPend = null;
    this._knobSent = null;
    this._knobTimer = null;
    // 按钮时间线
    this._presses = {};
    this._holdTimer = null;
    this._fxTimers = {};
    // 仪表演示物理：spd=显示速度(km/h)、tilt=显示倾角(°)、phase=空闲呼吸相位
    this._spd = 0;
    this._tilt = 0;
    this._phase = 0;
    this._vy = 0;              // 摇杆纵向意图 [-1,1]，上推为正
    // 车控回传真值缓存（600ms 内有效，过期回退本地演示）
    this._telemSpd = null;     // km/h
    this._telemTilt = null;    // °
    this._telemAt = 0;
    this._lastTelem = 0;
    this._mc = {};             // 遥测差分缓存
    // 指令帧复用同一 ArrayBuffer
    this._frame = new ArrayBuffer(12);
    this._actSeq = 0;
    // 渲染差分指纹
    this._arcSig = '';
    this._bubbleSig = '';
    this._spdSent = '';
    this._tiltSent = '';
    // 自定义导航栏避让
    this.setData({ padTop: this._safeTop(this._isLand()) });
    // 指令周期下传（未连接时 _emit 直接返回）
    this.timer = setInterval(() => this._emit(), SEND_MS);
  },

  onReady() {
    this._applyOrientation(this._isLand());
  },

  onResize(e) {
    const s = e && e.size;
    const land = s ? (s.windowWidth > s.windowHeight) : this._isLand();
    this._applyOrientation(land);
  },

  onUnload() {
    clearInterval(this.timer);
    clearInterval(this._pumpTimer);
    clearTimeout(this._connTimer);
    clearTimeout(this._dialInitT);
    clearTimeout(this._flipTimer);
    clearTimeout(this._refuseTimer);
    clearTimeout(this._hintTimer);
    clearTimeout(this._holdTimer);
    clearTimeout(this._knobTimer);
    Object.keys(this._fxTimers).forEach(k => clearTimeout(this._fxTimers[k]));
    this.socket.close();
  },

  // ================= 方向 =================
  _isLand() {
    try {
      const info = (wx.getWindowInfo ? wx.getWindowInfo() : wx.getSystemInfoSync());
      return info.windowWidth > info.windowHeight;
    } catch (err) { return false; }
  },

  _safeTop(land) {
    let bottom = 0, winH = 0;
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
    if (land && winH) pad = Math.min(pad, Math.round(winH * 0.14));
    return Math.max(0, Math.round(pad));
  },

  onToggleOrientation() {
    const now = Date.now();
    if (now - (this._oriAt || 0) < 350) return;   // 连点防护（要重建 canvas）
    this._oriAt = now;
    const target = !this.data.isLand;
    if (wx.setPageOrientation) {
      try { wx.setPageOrientation({ orientation: target ? 'landscape' : 'portrait' }); }
      catch (e) { /* 不支持则仅切换布局 */ }
    }
    this._applyOrientation(target);
    clearTimeout(this._flipTimer);
    this.setData({ flip: true });
    this._flipTimer = setTimeout(() => this.setData({ flip: false }), 600);
  },

  _applyOrientation(land) {
    this.setData({ isLand: land, padTop: this._safeTop(land) }, () => {
      this._queryCenter();
      this._initDial();
    });
    // 几何变了：摇杆帽回中，行驶意图清零（vzRelease 同款处理）
    this._n.x = 0; this._n.y = 0; this._vy = 0;
    this._stopKnobTimer();
    this._flushKnob({ stickActive: false });
    this.setData({ stickFwd: false, stickBack: false });
  },

  // ================= 速度弧 canvas =================
  // 方向切换后布局未落定时可能拿到 0 尺寸 → 80ms 重试 ×10（无人机页空白盘教训）
  _initDial() {
    clearTimeout(this._dialInitT);
    this._dialTry = 0;
    wx.createSelectorQuery().in(this).select('#dial')
      .fields({ node: true, size: true })
      .exec((res) => {
        const r = res && res[0];
        if (!r || !r.node || !(r.width >= 2) || !(r.height >= 2)) {
          if (++this._dialTry <= 10) {
            this._dialInitT = setTimeout(() => this._initDial(), 80);
          }
          return;
        }
        this._dialTry = 0;
        const canvas = r.node;
        const ctx = canvas.getContext('2d');
        let dpr = 2;
        try {
          dpr = (wx.getWindowInfo ? wx.getWindowInfo().pixelRatio : 2) || 2;
        } catch (e) { dpr = 2; }
        canvas.width = r.width * dpr;
        canvas.height = r.height * dpr;
        ctx.scale(dpr, dpr);
        this._dial = { canvas, ctx, w: r.width, h: r.height };
        // 泡的行程按实测玻璃管宽算：±12° → ±(管宽/2 − 泡半径)，贴边不溢出
        this._bubbleTravel = (r.width * (344 / 516)) / 2 - r.width * (48 / 516) / 2 - 2;
        this._arcSig = '';   // 强制下一帧重画
        this._pumpStart();
      });
  },

  // 速度弧：frac ∈ [0,1]。变化检测：fraction 量化到 0.001 + 颜色态 + 主题，没变不重画
  _drawArc(frac, running) {
    const cv = this._dial;
    if (!cv) return;
    const { ctx, w, h } = cv;
    if (!(w >= 2) || !(h >= 2)) return;
    const sig = Math.round(frac * 1000) + '|' + (running ? 1 : 0) + '|' + this.data.persona;
    if (sig === this._arcSig) return;
    this._arcSig = sig;
    const cx = w / 2, cy = h / 2;
    const R = w * 122 / 258;
    const a0 = ARC_A0_DEG * Math.PI / 180;
    const span = ARC_SPAN_DEG * Math.PI / 180;
    ctx.clearRect(0, 0, w, h);
    ctx.lineWidth = w * 7 / 258;
    ctx.lineCap = 'round';
    // 底弧（T3 冰川浅底换深石板灰蓝，预览 .phone.t3 .dial-ring-bg 同值）
    ctx.beginPath();
    ctx.arc(cx, cy, R, a0, a0 + span);
    ctx.strokeStyle = this.data.persona === 3 ? 'rgba(60, 90, 140, 0.22)' : 'rgba(140, 170, 210, 0.16)';
    ctx.stroke();
    // 速度弧（蓝 = 常规 / 绿 = 平衡运行中），零速不画（linecap 圆点残留）
    if (frac > 0.004) {
      ctx.beginPath();
      ctx.arc(cx, cy, R, a0, a0 + span * frac);
      ctx.strokeStyle = running ? '#4fe0a8' : '#3a94dd';
      ctx.shadowColor = running ? 'rgba(79, 224, 168, .55)' : 'rgba(58, 148, 221, .55)';
      ctx.shadowBlur = w * 5 / 258;
      ctx.stroke();
      ctx.shadowBlur = 0;
    }
  },

  // ================= 仪表泵：rAF + setInterval(16) 双路 =================
  // rAF 不保证会跑（后台/省电/headless），数据变就必须跟着变的显示不能把命交给渲染节拍。
  // 系数全部按真实 dt 归一（τ=时间常数），与调用频率解耦 —— 双路下双倍调用率也不失真。
  _pumpStart() {
    if (this._pumpOn) return;
    this._pumpOn = true;
    this._pumpLast = Date.now();
    this._pumpTimer = setInterval(() => this._pump(), 16);
    this._pumpRaf();
  },

  _pumpRaf() {
    if (!this._pumpOn) return;
    this._pump();
    const cv = this._dial;
    if (cv && cv.canvas.requestAnimationFrame) {
      cv.canvas.requestAnimationFrame(() => this._pumpRaf());
    } else {
      setTimeout(() => this._pumpRaf(), 16);
    }
  },

  _pump() {
    const now = Date.now();
    let dt = now - this._pumpLast;
    this._pumpLast = now;
    if (dt <= 0) return;
    if (dt > 50) dt = 50;          // 后台切回：一次别追太大步
    const dts = dt / 1000;
    this._phase += dts;

    // ---- 速度：目标 = 摇杆 y × 档位上限；未启动平衡 = 0（推杆无效）----
    // 车控回传 600ms 内有实速就用真值；否则本地演示（与预览页同一套模型）
    const g = GEAR[this.data.gear];
    const telemFresh = (this._telemSpd !== null) && (now - this._telemAt < 600);
    let vTgt = 0;
    if (this.data.running) {
      if (telemFresh) vTgt = this._telemSpd;
      else if (Math.abs(this._vy) > 0.04) vTgt = this._vy * g.max;
    }
    const vPrev = this._spd;
    this._spd += (vTgt - this._spd) * (1 - Math.exp(-dt / 180));

    // ---- 倾角：加速度 → 前倾（动量），速度平滑回正；空闲 ±0.6° 微呼吸（自平衡在跑）----
    // 系数 0.15：起步/急停瞬间泡摆到 ±8° 内，红区 >10° 留给真实异常（被撞/搬起）
    let tiltTgt;
    const telemTiltFresh = (this._telemTilt !== null) && (now - this._telemAt < 600);
    if (telemTiltFresh) {
      tiltTgt = this._telemTilt;
    } else {
      const accel = (this._spd - vPrev) / dts;
      tiltTgt = clamp(accel * 0.15, -8, 8) + Math.sin(this._phase * 2.1) * 0.6;
    }
    this._tilt += (tiltTgt - this._tilt) * (1 - Math.exp(-dt / 110));

    this._render(g);
  },

  _render(g) {
    // 速度弧（canvas 直绘，变化检测）
    this._drawArc(clamp(this._spd / g.max, 0, 1), this.data.running);

    // 泡：60Hz 差分 setData（0.5px 量化，没动不发）
    const bx = Math.round(this._tilt / 12 * (this._bubbleTravel || 0) * 2) / 2;
    const bsig = String(bx);
    if (bsig !== this._bubbleSig) {
      this._bubbleSig = bsig;
      this.setData({ bubbleStyle: 'transform: translateX(' + bx + 'px)' });
    }

    // 数字读数 10Hz（跳太快读不清）
    const now = Date.now();
    if (now - (this._spdSentAt || 0) >= 100) {
      this._spdSentAt = now;
      const st = this._spd.toFixed(1);
      if (st !== this._spdSent) { this._spdSent = st; this.setData({ spdText: st }); }
      const tt = (this._tilt >= 0 ? '+' : '-') + Math.abs(this._tilt).toFixed(1);
      if (tt !== this._tiltSent) {
        this._tiltSent = tt;
        const over = Math.abs(this._tilt) > 10;
        this.setData({ tiltText: tt, tiltOver: over });
      }
    }
  },

  // ================= 连接 =================
  onIp(e) { this.setData({ ip: e.detail.value }); },
  onPort(e) { this.setData({ port: e.detail.value }); },

  onConnectToggle() {
    if (this.data.connected) {
      this._connSeq = (this._connSeq || 0) + 1;
      this.socket.close();
      // 断链同时停平衡：断链后 _emit 直接 return，running 留着会读到假"运行中"
      this.setData({ connected: false, connText: '未连接', connecting: false, running: false, balStateText: '平衡未启动' });
      this._telemSpd = this._telemTilt = null;
      return;
    }
    if (this.data.connecting) return;    // 连点防护
    const { ip, port } = this.data;
    const seq = (this._connSeq = (this._connSeq || 0) + 1);
    const alive = () => seq === this._connSeq;
    this.setData({ connText: '连接中…', connecting: true });
    this.socket.connect(ip, port, {
      onOpen: () => {
        if (!alive()) return;
        this.setData({ connected: true, connText: '已连接', connecting: false });
      },
      onClose: () => {
        if (!alive()) return;
        this.setData({ connected: false, connText: '连接关闭', connecting: false, running: false, balStateText: '平衡未启动' });
        this._telemSpd = this._telemTilt = null;
      },
      onError: (err) => {
        if (!alive()) return;
        this.setData({ connected: false, connText: '连接失败', connecting: false });
        wx.showToast({ title: (err && err.errMsg) || '连接失败', icon: 'none', duration: 1200 });
      },
      onMessage: (res) => this._onMessage(res)
    });
    // 兜底：部分机型迟迟不回调，8s 后允许再点
    clearTimeout(this._connTimer);
    this._connTimer = setTimeout(() => {
      if (!alive() || !this.data.connecting) return;
      this.setData({ connText: '连接超时', connecting: false });
    }, 8000);
  },

  // ---- 接收：车控遥测（单行 JSON，字段与飞控同族）----
  _onMessage(res) {
    if (!res || typeof res.data !== 'string') return;
    const now = Date.now();
    if (now - this._lastTelem < TELEM_MIN_MS) return;   // 节流（含纯文本）
    this._lastTelem = now;
    const o = parseTelemetry(res.data);
    if (!o) return;
    this._telemAt = now;
    if (o.vx !== undefined) this._telemSpd = Math.abs(Number(o.vx) || 0) * 3.6;   // m/s → km/h
    if (o.pitch !== undefined) this._telemTilt = Number(o.pitch) || 0;
    if (o.vbat === undefined && o.temp === undefined) return;

    // 差分 setData：只发真变了的面板字段
    const mc = this._mc;
    const patch = {};
    if (o.vbat !== undefined) {
      const v = (Number(o.vbat) || 0).toFixed(2);
      if (mc.vbat !== v) {
        mc.vbat = v;
        patch['tm.vbat'] = v;
        patch.battPct = clamp(Math.round((Number(o.vbat) || 0) / BATT_FULL_V * 100), 0, 100);
      }
    }
    if (o.temp !== undefined) {
      const t = (Number(o.temp) || 0).toFixed(1);
      if (mc.temp !== t) { mc.temp = t; patch['tm.temp'] = t; }
    }
    if (Object.keys(patch).length) this.setData(patch);
  },

  // ================= 指令下传（20Hz）=================
  _emit() {
    if (!this.data.connected || !this.socket) return;
    const g = GEAR[this.data.gear];
    // vx = 前后速度(m/s，正=前)；两轮车无横移 vy 恒 0；arm 字节 = 平衡开关
    const vx = this.data.running ? this._vy * (g.max / 3.6) : 0;
    const vyaw = this._n.x * YAW_MAX;
    this.socket.send(buildFrame(vx, 0, vyaw, this.data.running ? 1 : 0, 0, this._frame));
  },

  // 一次性动作帧（灯光/鸣笛）：连发 3 帧同 seq，车控按 seq 去重（无人机页同款）
  _sendActionCode(code) {
    if (!this.data.connected) return;
    const seq = this._actSeq = (this._actSeq + 1) & 0xFF;
    for (let i = 0; i < 3; i++) {
      setTimeout(() => this.socket.send(buildActionFrame(code, seq)), i * 40);
    }
  },

  // ================= 摇杆（单杆：行驶）=================
  // 事件挂在 .stick-layer（catch 前缀 → 掐掉页面滚动/下拉刷新）。
  // touchmove 只做三件事：算偏移、写归一化值（实例字段）、像素位移交给按帧合并。
  onStickStart(e) {
    const t = (e.changedTouches && e.changedTouches[0]) || (e.touches && e.touches[0]);
    if (!t) return;
    this._touchId = t.identifier;
    this._ensureCenter();
    sfx.play('grab');   // 摇杆抓住：极轻 tick
    // 按下帧的位置不要前沿直发（会拆成两次 setData）：_holdFlush 让 _track 只入队，
    // 随后 _flushKnob(extra) 把「位置 + 按下反馈」并进同一次 setData
    this._holdFlush = true;
    this._track(t);
    this._holdFlush = false;
    this._stopKnobTimer();
    this._flushKnob({ stickActive: true });
  },

  onStickMove(e) {
    const t = this._touchOf(e);
    if (!t) return;
    this._track(t);
  },

  onStickEnd(e) {
    if (this._touchId !== null && this._touchId !== undefined) {
      const c = e.changedTouches || [];
      let mine = false;
      for (let i = 0; i < c.length; i++) if (c[i].identifier === this._touchId) mine = true;
      if (!mine) return;
    }
    this._touchId = null;
    sfx.play('release');
    // 松手：控制量立刻归零，回中 + 摘掉按下反馈合并成一次 setData
    this._n.x = 0; this._n.y = 0; this._vy = 0;
    this._stopKnobTimer();
    this._knobPend = { x: 0, y: 0 };
    this._flushKnob({ stickActive: false });
    this.setData({ stickFwd: false, stickBack: false });
  },

  _touchOf(e) {
    const id = this._touchId;
    const list = e.touches || [];
    const changed = e.changedTouches || [];
    if (id !== null && id !== undefined) {
      for (let i = 0; i < list.length; i++) if (list[i].identifier === id) return list[i];
      for (let i = 0; i < changed.length; i++) if (changed[i].identifier === id) return changed[i];
      return null;
    }
    return changed[0] || list[0] || null;
  },

  _ensureCenter() {
    if (!this._center || !(this._center.R > 0)) this._queryCenter();
  },

  // 量一次圆心与可动半径（首次/切方向后）。拖动过程不再读布局（强制同步布局在关键路径上）
  _queryCenter() {
    wx.createSelectorQuery().in(this).select('#driveStick').boundingClientRect((rect) => {
      if (!rect) return;
      // R = 基圆半径 − 帽半径（帽心可动范围），与预览一致：帽 116/基圆 256 → 0.273
      this._center = {
        x: rect.left + rect.width / 2,
        y: rect.top + rect.height / 2,
        R: rect.width * 0.27
      };
    }).exec();
  },

  _track(t) {
    const c = this._center;
    if (!c || !(c.R > 0)) return;
    const px = (t.clientX !== undefined) ? t.clientX : t.pageX;
    const py = (t.clientY !== undefined) ? t.clientY : t.pageY;
    let dx = (Number(px) || 0) - c.x;
    let dy = (Number(py) || 0) - c.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > c.R) { dx = dx / dist * c.R; dy = dy / dist * c.R; }   // 限幅到圆内
    this._n.x = dx / c.R;
    this._n.y = dy / c.R;
    this._vy = -this._n.y;      // 上推 = 前进 = 正
    // 方向微光（推杆方向亮起），只在越过阈值时 setData
    const fwd = this._vy > 0.45, back = this._vy < -0.45;
    if (fwd !== this.data.stickFwd || back !== this.data.stickBack) {
      this.setData({ stickFwd: fwd, stickBack: back });
    }
    this._queueKnob(dx, dy);
  },

  // 摇杆位移下发：前沿即时 + 16ms 抑制窗（2026-10-24 响应提速，无人机页同配方）。
  // 窗口空闲时第一笔位移零延迟直发；窗口内的攒进 pend、到期尾随补发。
  // 0.5px 量化保留 —— 亚像素差异看不出来，却能省掉大量"其实没动"的下发。
  _queueKnob(x, y) {
    const qx = Math.round(x * 2) / 2;
    const qy = Math.round(y * 2) / 2;
    const last = this._knobSent;
    if (last && last.x === qx && last.y === qy) return;
    this._knobPend = { x: qx, y: qy };
    if (this._knobTimer) return;                    // 抑制窗内：攒着，尾随统一发
    if (this._holdFlush) return;                    // 按下帧：并入 _flushKnob(extra) 一次发
    this._flushKnob();                              // 前沿：零延迟直发
    this._knobTimer = setTimeout(() => {
      this._knobTimer = null;
      if (this._knobPend) this._flushKnob();        // 尾随：补发窗内积压
    }, STICK_FRAME_MS);
  },

  _flushKnob(extra) {
    const d = extra || {};
    const p = this._knobPend;
    if (p) {
      const zero = !(p.x || p.y);
      d.knobStyle = zero
        ? 'transform: translate3d(0px, 0px, 0); transition: transform 140ms ease-out'
        : 'transform: translate3d(' + p.x + 'px, ' + p.y + 'px, 0)';
      this._knobSent = p;
      this._knobPend = null;
    }
    this.setData(d);
  },

  _stopKnobTimer() {
    if (this._knobTimer) {
      clearTimeout(this._knobTimer);
      this._knobTimer = null;
    }
  },

  // ================= 按钮（按下反馈 + 点击派发，同无人机页）=================
  onBtnDown(e) {
    const key = this._btnKey(e);
    if (!key) return;
    const t = (e.touches && e.touches[0]) || (e.changedTouches && e.changedTouches[0]) || {};
    this._presses[key] = { at: Date.now(), x: Number(t.clientX) || 0, y: Number(t.clientY) || 0, long: false };
    if (this.data.pressed !== key) this.setData({ pressed: key });
    sfx.play('press');   // 全键按下 tick（16ms，最低音量档）

    // 平衡主键：运行中按住 MAIN_HOLD_MS = 关闭（定时器自己实现，bindlongpress 会打架）
    if (key === 'main' && this.data.running) {
      this._clearHold();
      this._holdTimer = setTimeout(() => {
        this._holdTimer = null;
        const p = this._presses.main;
        if (!p) return;
        p.long = true;                 // 松手不再补一次"点击"
        this._setRunning(false);
        sfx.play('lock');              // 停机 thunk：与 heavy 振动同步，危险操作听感要沉
        try { wx.vibrateShort({ type: 'heavy' }); } catch (err) {}
      }, MAIN_HOLD_MS);
    }
  },

  onBtnUp(e) {
    const key = this._btnKey(e);
    const p = key ? this._presses[key] : null;
    // 按下时刻先存到实例字段：_fireBtn 派发时 _presses 已删，
    // 主键的误触判定（按住是否够 120ms）要从这里读
    this._lastPressAt = p ? p.at : 0;
    if (key) delete this._presses[key];
    if (key === 'main') this._clearHold();
    // 按下态：不足 PRESS_MIN_MS 就延后摘掉，极快点按也看得到反馈
    const elapsed = p ? Date.now() - p.at : PRESS_MIN_MS;
    const clear = () => {
      if (this.data.pressed !== key) return;
      if (this._presses[key]) return;
      this.setData({ pressed: '' });
    };
    if (elapsed < PRESS_MIN_MS) setTimeout(clear, PRESS_MIN_MS - elapsed);
    else clear();

    if (!p) return;
    // 位移过大 = 滑开，不算点击
    const t = (e.changedTouches && e.changedTouches[0]) || {};
    if (t.clientX !== undefined) {
      const dx = (Number(t.clientX) || 0) - p.x;
      const dy = (Number(t.clientY) || 0) - p.y;
      if (Math.hypot(dx, dy) > TAP_MOVE_MAX) return;
    }
    if (p.long) return;
    this._fireBtn(key);
  },

  onBtnCancel(e) {
    const key = this._btnKey(e);
    const kill = (k) => {
      delete this._presses[k];
      if (k === 'main') this._clearHold();
      if (this.data.pressed === k) this.setData({ pressed: '' });
    };
    if (key) kill(key);
    else {
      Object.keys(this._presses).forEach(kill);
      this._clearHold();
      if (this.data.pressed) this.setData({ pressed: '' });
    }
  },

  _fireBtn(key) {
    switch (key) {
      case 'conn': sfx.play('confirm'); this.onConnectToggle(); break;
      case 'orient': sfx.play('confirm'); this.onToggleOrientation(); break;
      case 'persona': this._firePersona(); break;
      case 'main': this._fireMain(); break;
      case 'light': this._fireLight(); break;
      case 'gear': this._fireGear(); break;
      case 'horn': this._fireHorn(); break;
      default: break;
    }
  },

  // ---- 个性键：1→2→3→1 循环。token 换档全在 CSS（.page.theme-N），
  //      canvas 只需清签名让底弧按新主题重画一次 ----
  _firePersona() {
    const p = this.data.persona % 3 + 1;
    this.setData({ persona: p });
    this._arcSig = '';   // 强制下一帧重画（底弧颜色随主题）
    sfx.play('persona', p);   // 玻璃 ping 三档音高（三页同族）
    try { wx.vibrateShort({ type: 'light' }); } catch (e) {}
  },

  // ---- 平衡主键：点按启动 / 拒绝零位移闪烁 / 长按已在 onBtnDown 处理 ----
  _fireMain() {
    if (!this.data.connected) { this._refuse(); return; }       // 未连接：拒绝
    if (!this.data.running) {
      // 误触（按住不足 120ms）→ 拒绝闪烁，不启动
      if (this._lastPressAt && Date.now() - this._lastPressAt < MAIN_TAP_MIN_MS) { this._refuse(); return; }
      this._setRunning(true);
      this._fx('mainFx', 'burst');
      sfx.play('confirm');   // 平衡启动 = 动作发出去了（长按关闭的 lock 在按下定时器里）
      try { wx.vibrateShort({ type: 'light' }); } catch (e) {}
    } else {
      // 运行中点按：应该长按 —— 红框 + 状态字提示（1.2s 后回正）
      this._refuse();
      this.setData({ balStateText: '长按主键才能关闭' });
      clearTimeout(this._hintTimer);
      this._hintTimer = setTimeout(() => {
        if (this.data.running) this.setData({ balStateText: '平衡运行中' });
      }, 1200);
    }
  },

  _refuse() {
    sfx.play('refuse');   // 低频双 buzz：未连接 / 误触 / 运行中点按，与红框闪烁同步
    // CSS 动画要重播：先摘类、隔一帧再挂回（personaTick 同款）
    clearTimeout(this._refuseTimer);
    this.setData({ mainRefuse: false });
    setTimeout(() => {
      this.setData({ mainRefuse: true });
      this._refuseTimer = setTimeout(() => this.setData({ mainRefuse: false }), 520);
    }, 32);
  },

  _setRunning(on) {
    this.setData({ running: on, balStateText: on ? '平衡运行中' : '平衡未启动' });
    if (!on) {
      this._fx('mainFx', 'suck');
      // 关平衡 = 立即停：行驶意图清零（vzRelease 经验），帽回中
      this._n.x = 0; this._n.y = 0; this._vy = 0;
      this._stopKnobTimer();
      this._knobPend = { x: 0, y: 0 };
      this._flushKnob({ stickActive: false });
      this.setData({ stickFwd: false, stickBack: false });
    }
  },

  // ---- 功能键：灯光 / 档位 / 鸣笛 ----
  _fireLight() {
    const on = !this.data.lightOn;
    this.setData({ lightOn: on });
    sfx.play(on ? 'toggleOn' : 'toggleOff');   // 开/关可听辨
    this._sendActionCode(on ? ACTION.LIGHT_ON : ACTION.LIGHT_OFF);
    this._fx('lightFx', on ? 'burst' : 'suck');
  },

  _fireGear() {
    const gear = (this.data.gear + 1) % 3;
    this.setData({ gear, gearName: GEAR[gear].name, gearDeg: GEAR[gear].deg });
    sfx.play('confirm');   // 换档确认（三档循环，音高不随档变——档位有表针可视化）
    this._fx('gearFx', 'burst');
    try { wx.vibrateShort({ type: 'light' }); } catch (e) {}
  },

  _fireHorn() {
    sfx.play('confirm');   // 鸣笛 = 一次性动作发出；车端喇叭声才是主声，这里只做按键确认
    this._sendActionCode(ACTION.HORN);
    this._fx('hornFx', 'burst');
    try { wx.vibrateShort({ type: 'light' }); } catch (e) {}
  },

  // 一次性特效：先摘类隔一帧再挂回（重播），620ms 后自动摘。
  // 两个定时器分键存放（'xxx'/'xxxon'），onUnload 一并清掉
  _fx(field, kind) {
    clearTimeout(this._fxTimers[field]);
    clearTimeout(this._fxTimers[field + 'on']);
    const patchOff = {}; patchOff[field] = '';
    this.setData(patchOff);
    this._fxTimers[field + 'on'] = setTimeout(() => {
      const patchOn = {}; patchOn[field] = kind;
      this.setData(patchOn);
      this._fxTimers[field] = setTimeout(() => {
        const patchEnd = {}; patchEnd[field] = '';
        this.setData(patchEnd);
      }, 620);
    }, 32);
  },

  _btnKey(e) {
    return (e && e.currentTarget && e.currentTarget.dataset && e.currentTarget.dataset.key) || '';
  },

  _clearHold() {
    if (this._holdTimer) {
      clearTimeout(this._holdTimer);
      this._holdTimer = null;
    }
  }
});
