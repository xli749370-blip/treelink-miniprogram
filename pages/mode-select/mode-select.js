// ===========================================================================
// 控制模式选择页（2026-10-23 落地自 design/mode-select-preview.html 定稿）
// 流程：选模式 → 启动动画（无人机=A 轨道停靠 / 平衡车=B 碎片积木 /
//       智能车=C 循迹进站）→ 图形先收 240ms → navigateTo 对应控制页
// 交棒说明：预览页的"幕布淡出露底层"跨页面做不到 —— 这里改为 splash 自身
// 收尾后跳转，目标页根节点自带 scale .96→1 长入动画（见各控制页 .page-enter）
// ===========================================================================
const HANDOFF_MS = { drone: 1800, car: 1400, smart: 1500 };   // 交棒点随变体表演时长
const CARD_MOVE_MAX = 14;   // 手指位移超过这个值(px)不算点击（滑卡不误触）
const PERSONA_COUNT = 3;    // 个性皮肤档数（theme-1/2/3）
const sfx = require('../../utils/sfx.js');   // 按键反馈音效（合成，零音频资源）

// 翻面检测阈值（g）：z 轴 +1g=正面朝上 / -1g=背面朝上。±0.55 以内视为"侧立"不计。
const FLIP_Z = 0.55;
const FLIP_GAP_MS = 800;    // 两次翻转最小间隔（抖动/连翻节流）
// 误触防线（2026-10-24）：从控制页返回时转场动画 + 手部动作会让 z 轴瞬时窜过阈值，
// 若 onShow 一上来就按"正面"判定会当场误切。三道防线：
//   1) WARMUP：回场缓冲期只读不判（转场/放下手机的动作都发生在这一窗内）
//   2) 武装：缓冲期结束后先按真实读数认当前面（认面不触发），再启用判定
//   3) 确认：进入背面需连续 ACC_DOWN_TICKS 拍低于阈值（20Hz 下 2 拍=100ms，
//      甩一下手机的瞬时尖峰凑不满），真正的翻面动作 300ms+ 绰绰有余
const ACC_WARMUP_MS = 600;
const ACC_DOWN_TICKS = 2;

Page({
  data: {
    splashShow: false,     // 启动动画层（wx:if 控制挂载，动画靠它重启）
    splashMode: 'drone',   // 当前启动动画变体
    handoff: false,        // 交棒中：图形收 + 幕布退场
    splashName: '',
    splashSub: '',
    pressedCard: '',       // 按下中的模式卡（'drone' | 'car' | 'smart'，touchstart 即时挂下陷态）
    persona: 1,            // 个性皮肤档（冷启恒 1；翻面循环切换；选定后带给控制页）
    personaFx: '',         // 翻面换肤的全屏扫光（'go' 一次）
  },

  // ---- 模式卡点按（2026-10-24）：接在不动的外壳 .mode-hit 上，touchend 派发 ----
  // 不用 bindtap 的原因：卡体按下会 translateY 下沉，tap 依赖 touchend 落点命中
  // 变形后的卡体 —— 点得快/点得靠边时落点已在外 → 丢触发。这里只看位移容差。
  // 下陷反馈不用 hover-class：真机链路延迟可见、快点点按来不及画一帧就被摘。
  // 改为 touchstart 即时 setData 挂 .pressed（与两个控制页的按键同套）。
  onCardDown(e) {
    const mode = (e.currentTarget.dataset && e.currentTarget.dataset.mode) || '';
    const t = (e.touches && e.touches[0]) || (e.changedTouches && e.changedTouches[0]) || {};
    this._card = {
      mode,
      at: Date.now(),
      x: Number(t.clientX) || 0,
      y: Number(t.clientY) || 0
    };
    if (mode && this._state !== 'splash') {
      this.setData({ pressedCard: mode });
      sfx.play('press');               // 按下反馈：与三张控制页的按键同款 tick
    }
  },

  onCardUp(e) {
    const c = this._card;
    this._card = null;
    if (this.data.pressedCard) this.setData({ pressedCard: '' });
    if (!c || !c.mode) return;
    const t = (e.changedTouches && e.changedTouches[0]) || {};
    if (t.clientX !== undefined) {
      const dx = (Number(t.clientX) || 0) - c.x;
      const dy = (Number(t.clientY) || 0) - c.y;
      if (Math.hypot(dx, dy) > CARD_MOVE_MAX) return;   // 滑动，不算点击（无声，防误滑吵）
    }
    sfx.play('confirm');               // 选中派发：双 tick 上行（连接/启动同族）
    this.onPickMode({ currentTarget: { dataset: { mode: c.mode } } });
  },

  onCardCancel() {
    this._card = null;
    if (this.data.pressedCard) this.setData({ pressedCard: '' });
  },

  onPickMode(e) {
    const mode = e.currentTarget.dataset.mode;
    if (this._state === 'splash') return;            // 防连点：动画进行中无效
    this._state = 'splash';
    this._clearTimers();
    // 启动动画文案（三模式查表；卡片顺序 2026-10-24 调整为 智能车→平衡车→无人机）
    const NAMES = { drone: '无人机模式', car: '平衡车模式', smart: '智能车模式' };
    const SUBS  = {
      drone: '正在连接飞控 · 建立遥测链路',
      car:   '正在连接车控 · 自平衡待命',
      smart: '正在连接车控 · 循迹避障待命'
    };
    this.setData({
      splashShow: true,
      splashMode: mode,
      handoff: false,
      splashName: NAMES[mode] || '',
      splashSub:  SUBS[mode] || '',
    }, () => {
      // wx:if 刚插入 DOM：B 变体在布局落定后初始化进度环 canvas
      // （尺寸非法 <2px 时 80ms 重试 ×10，见 _initRingCanvas —— 布局竞态兜底）
      if (mode === 'car' || mode === 'smart') this._initRingCanvas();
      // 交棒：图形先收 240ms → 跳转目标页（目标页根节点 scale .96→1 长入）
      this._t1 = setTimeout(() => this.setData({ handoff: true }), HANDOFF_MS[mode]);
      this._t2 = setTimeout(() => this._go(mode), HANDOFF_MS[mode] + 240);
    });
  },

  _go(mode) {
    this._state = 'done';
    // 选定的个性皮肤一并交给控制页（三控制页 onLoad 接收；不带参时控制页恒为 1）
    const PAGES = { drone: '/pages/index/index', car: '/pages/balance/balance', smart: '/pages/smartcar/smartcar' };
    const url = (PAGES[mode] || PAGES.drone) + '?persona=' + this.data.persona;
    wx.navigateTo({
      url,
      complete: () => {           // 无论成败都择机复位（navigateBack 回来时 onShow 兜底）
        this._t3 = setTimeout(() => this._reset(), 600);
      },
    });
  },

  // 复位到选择态：清动画定时器、撤 splash 层
  _reset() {
    if (this._state === 'splash') return;   // 动画进行中不复位（onShow 首载触发用）
    this._clearTimers();
    this._state = 'sel';
    if (this.data.splashShow || this.data.handoff) {
      this.setData({ splashShow: false, handoff: false });
    }
  },

  onShow() {
    // 从控制页返回选择页：回到初始选择态（splash 已随跳转播完）
    if (this._state === 'done') this._reset();
    // 翻面换肤手势只在选择态监听（进控制页 onHide 会停掉）
    this._startFlip();
  },

  onHide() { this._stopFlip(); },
  onUnload() { this._clearTimers(); this._stopFlip(); },

  /* ==========================================================================
     翻面切换个性（2026-10-24）：翻转手机，**屏幕朝下那一刻就完成切换**，
     翻回正面时看到的已经是新个性（切换动画发生在看不见的背面，无感换肤）。
     加速度计判 z 轴：正面 z≈+1g、背面 z≈-1g ——
     up →(z<-0.55 连续2拍)→ 背面：立即换肤并记为 down →(z>+0.55)→ 回到 up（不再触发）。
     界面本身不旋转（选页没有横屏布局）。
     onShow 启监听，onHide/onUnload 停（省电；控制页内不误触发）。
     误触三防线（ACC_WARMUP_MS / 武装认面 / 连拍确认）见顶部常量区注释——
     解决"从控制页返回时转场+手部动作瞬时窜过阈值被当成翻面"的问题。
     ========================================================================== */
  _startFlip() {
    if (this._accelOn) return;
    this._accelOn = true;
    this._face = null;                 // 未武装：缓冲期后按真实读数认面（认面不触发）
    this._downTicks = 0;
    this._armAt = Date.now() + ACC_WARMUP_MS;
    this._lastFlipAt = 0;
    wx.startAccelerometer({ interval: 'ui', fail: () => {} });   // 20Hz 足够判翻面
    if (!this._onAccel) {
      this._onAccel = (res) => this._onAccelTick(res);
      wx.onAccelerometerChange(this._onAccel);
    }
  },

  _stopFlip() {
    if (!this._accelOn) return;
    this._accelOn = false;
    if (this._onAccel) {
      wx.offAccelerometerChange(this._onAccel);
      this._onAccel = null;
    }
    wx.stopAccelerometer({ fail: () => {} });
  },

  _onAccelTick(res) {
    const z = Number(res && res.z) || 0;
    // 防线 1：回场缓冲期只丢抖动读数，不判定不认面
    if (Date.now() < this._armAt) { this._downTicks = 0; return; }
    // 防线 2：未武装 → 先按读数认当前面（手机若本来就趴着，认成 down，翻回正面只复位）
    if (this._face === null) {
      if (Math.abs(z) >= FLIP_Z) this._face = z > 0 ? 'up' : 'down';
      return;
    }
    if (this._face === 'up') {
      if (z < -FLIP_Z) {
        // 防线 3：连续多拍低于阈值才算真翻面（瞬时尖峰/甩动凑不满）
        if (++this._downTicks < ACC_DOWN_TICKS) return;
        this._downTicks = 0;
        this._face = 'down';                              // 翻到屏幕朝下 = 此刻切换
        const now = Date.now();
        if (now - this._lastFlipAt < FLIP_GAP_MS) return; // 抖动/连翻节流
        this._lastFlipAt = now;
        this._cyclePersona();
      } else {
        this._downTicks = 0;
      }
    } else if (z > FLIP_Z) {
      this._face = 'up';                                  // 翻回正面，不再触发
    }
  },

  _cyclePersona() {
    if (this._state === 'splash' || this._state === 'done') return;   // 动画/跳转中不切
    const next = this.data.persona % PERSONA_COUNT + 1;
    sfx.play('swish');                 // 翻面换肤：干净的「咻」（纯噪声扫频，无 tick；2026-11-15 用户定）
    // 先摘扫光类隔一帧再挂回（重播）；换肤本身即时生效
    this.setData({ persona: next, personaFx: '' });
    clearTimeout(this._fxT);
    this._fxT = setTimeout(() => {
      this.setData({ personaFx: 'go' });
      this._fxT = setTimeout(() => this.setData({ personaFx: '' }), 640);
    }, 32);
  },

  _clearTimers() {
    clearTimeout(this._t1); clearTimeout(this._t2); clearTimeout(this._t3);
    clearTimeout(this._ringInitT);
    clearInterval(this._ringTimer);
    this._t1 = this._t2 = this._t3 = this._ringInitT = this._ringTimer = null;
  },

  // ---- B 变体进度环：canvas 2d 描线 0→100%（0.55s 起、0.5s ease-out）----
  // 颜色取 theme-1 常量（本页恒为个性1；换肤开放时改读 THEME.applyTheme 导出值）
  _initRingCanvas() {
    clearTimeout(this._ringInitT);
    wx.createSelectorQuery().in(this).select('#ringCv')
      .fields({ node: true, size: true })
      .exec((res) => {
        if (!res || !res[0] || !res[0].node) return;
        const { node: canvas, width, height } = res[0];
        if (!width || width < 2) {
          // 布局竞态（wx:if 插入后首帧尺寸未落定）→ 延迟重试，最多 10 次
          if (++this._ringRetry > 10) return;
          this._ringInitT = setTimeout(() => this._initRingCanvas(), 80);
          return;
        }
        this._ringRetry = 0;
        const ctx = canvas.getContext('2d');
        let dpr = 2;
        try {
          dpr = (wx.getWindowInfo ? wx.getWindowInfo().pixelRatio : 2) || 2;
        } catch (e) { dpr = 2; }
        canvas.width = width * dpr;
        canvas.height = height * dpr;
        ctx.scale(dpr, dpr);
        this._ring = { canvas, ctx, w: width, h: height };
        this._startRingDraw();
      });
  },

  _startRingDraw() {
    clearInterval(this._ringTimer);
    const t0 = Date.now();                    // 时间轴锚定 splash 起播
    this._ringTimer = setInterval(() => {
      const r = this._ring;
      if (!r) { clearInterval(this._ringTimer); return; }
      let p = (Date.now() - t0 - 550) / 500;  // 0.55s 起扫、0.5s 时长
      if (p <= 0) { this._drawRing(0); return; }
      if (p >= 1) { this._drawRing(1); clearInterval(this._ringTimer); return; }
      this._drawRing(1 - Math.pow(1 - p, 3)); // ease-out cubic
    }, 16);
  },

  _drawRing(p) {
    const r = this._ring;
    if (!r) return;
    const { ctx, w, h } = r;
    const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 3;
    if (R < 2) return;                        // 退化尺寸守卫
    ctx.clearRect(0, 0, w, h);
    ctx.lineWidth = 3;
    ctx.lineCap = 'round';
    // 底环（border 同族灰）
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(255,255,255,0.10)';
    ctx.stroke();
    // 进度弧：从顶部顺时针
    if (p > 0) {
      ctx.beginPath();
      ctx.arc(cx, cy, R, -Math.PI / 2, -Math.PI / 2 + p * Math.PI * 2);
      ctx.strokeStyle = '#4aa3ff';
      ctx.shadowColor = 'rgba(74,163,255,0.45)';
      ctx.shadowBlur = 6;
      ctx.stroke();
    }
  },
});
