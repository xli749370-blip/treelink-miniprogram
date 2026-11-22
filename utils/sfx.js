/* ===========================================================================
   按键反馈音效引擎（2026-11-15，三张控制页共用）—— WebAudio 实时合成，零音频资源
   ---------------------------------------------------------------------------
   音色分层与动效三档纪律对齐（design/drone-texture-preview.html 01 节验收稿）：
     press      16ms 干涩 tick          —— 60ms「按下」档的听觉对应物
     confirm    双 tick 上行            —— 「动作发出去了」的一次性确认
     toggleOn/Off  正弦上滑/下滑        —— 开关可听辨（循迹/避障/陀螺仪/灯光）
     whoosh     带通噪声上扫 + 亮 tick  —— 特技/一键起飞，与 busy 0.6s 编排同步
     swish      干净的「咻」（纯噪声上扫+下扫尾） —— 模式选择页翻面换肤专用
     lock       低频 thunk + 咔点       —— 长按上锁 / 长按停机，危险操作要沉
     refuse     方波双 buzz             —— 未连接/未解锁拒绝，与 toast/shake 同步
     holdStart/End  轻 servo 上/下扫    —— 升降按住型键的起止
     persona(n) 玻璃 ping 三档音高      —— 钢穹 1980 / 紫电 2240 / 极昼 2520Hz
     grab/release   极轻 tick           —— 摇杆抓住/松手
   实现：wx.createWebAudioContext（基础库 ≥2.19.0）。所有出口包 try/catch ——
   音效是锦上添花，任何失败（低版本基础库/系统拒签）都不允许影响控制主流程。
   =========================================================================== */

let ctx = null;        // WebAudioContext 单例（懒创建：首次用户触摸时才有声）
let master = null;     // 主音量（0.5，与预览稿一致）
let noiseBuf = null;   // 白噪声缓冲：whoosh/lock/press 的气感成分共用
let broken = false;    // 创建失败标记（低版本基础库等）：此后全部静默跳过

/* 兜底白噪声（0.5s · 8kHz · 8bit mono WAV）：仅当 WebAudioContext.createBuffer
   不可用时经 decodeAudioData 解码使用（≈4KB，换取全部环境都有气感声部）。 */
const NOISE_B64 = [
'UklGRsQPAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YaAPAAClTcoYJTC7HW0TLN7WI3su2R4/ch/LGXEXRJTWSTyd',
'XDRgvjEgHmn+2qDu6LmZf1x8KZn9r+WTJTzWVK9N+tcUJ6Cus/7pIy+K8iEfnuSRxbEL7LVWO/web5NCfsvI/ilV5c2ORtyO1LfC',
'dk0qWk12dwb4XYaQAkrWvaNAG+nIy8zJNfbNH2EiauFTOK4aNABNM7oNJGrATIGxuvI+O/nu9fefK0k0r4f1UgtpuUsNmC6Fu1W2',
'cqhyY3rNdGb8tg4Oj/GEY7DksropcDR08GSsaPcA9bArPcZm9FveqizK7c0rUVdBDk3uSvKzT0MKBzRH3mNsDoBslXumhNZDH7Xq',
'10JNCeFdAkxYSPI9H6b3Nh1/YY0VMucOIOKmZo3n9H6EZ+VG1T7I4qEle9slbJs+T7tJgUbvcDDL+VNyUtzOrddktqMvuwmt6uEJ',
'xKmXIDl1NSuHixRcikLYhM9M/actjh1d2SWJCC2FKnEihz7oBa3ViUIWejhShhlcZ5+caZTkW4qxCYASBwlh833kNt39yZ1uda9l',
'R8+xG0IHJILcUxwrw5B8lhfrXlCJ5AGGuqilfRGeb7ZdAKvDKvOOZn8CLoctScwVyQuZm3crT8em/UyRShbbRwh1Kw8VRLg1wOcZ',
'CX36hwHpIy8h8oEmh3hpduv8wyf1kxdlJ0upgptEBvYf+Ikyb/qUku3u7jxmnyvyCJTqJ+aJxmtrJi5IhrhDjzm6dv74yQxRAfvm',
'z5pI1bDAoT2pAKatyz1kBpSBviHJxye424wYjzQakkx/iN+hYb/bDsxoKRnS5kaS+BlBV/HUr5CYgoXPepr3yT1VUiZq/nDnquba',
'R2J8LlmvLqN6vIRnCtPE02vAiq0f/464QG4vin/EzOTdnwtBENny+gAlyO/lfzdyT0036isUAEB3E5tBgN85MiSZYsaFcgAFmuuO',
'oXzzeH4O0p0cC2P/1ymDdNm9dPwRrde5ymUDlSJp/WafY3bucYeXN/1fcvjVHErJG20MSNQaHl7J5qA5KFSoYV7vEJ/Bv6niVjcB',
'KI8ps9c/asK2nt0sGfJkvuRipbryD9J+zxTAEe0gH4NjIK25i6sWhqKNmAEhDHc28+7FgNz8Q/5dBJtNeKej67koZchRftAhEfam',
'Uto1JIcrajHX/+RYd0TV63g+lpaPib6ChWXgfl99eE6QYKchyoB9djPtEjQC83blvxSWdz0ZYWMmvlvlhQM2s28TvK5IFmiCE2gF',
'p9G+Xp8naBD99yDQM8pPLlPLitGRndUan7bU1Qm6ZMjPaAPeUNg6Ls+661NCBxpIyy29V0qykVJXIjfE+2WaQBb3oRvGLFJxz2Ty',
'XW8VzFDEtz9MfmIVE6U8x+mc151/2ce85OBbCwH67njk6lvyzDYiQbfcuy7iFBRCKqAoG8FFDSE4Y0P7k1RxIbOBUaWM6UmC9WqG',
'eaO+EmVdzlKOp8BWhzoYuOc1gcm+h8C8SripKeJ1WhiXgZ6gABFxTJTd1boYQ/p0FwsbAbWbNrZy05pEaLvzUUQHfEzmMSBKis2H',
'BRyz4/x/VAAWHwzPX3lRHTUGZEjTZtRZniCZGPQDwN/uKedZczWFdhM/q4YaiN+Hl28rB1aFeGdRp2LHqHrC8PEDDd93nWzIJ1dK',
'EA05NlKwSA4PFUYVIhchumYhxDZ+aWg5EREsk/QzQzJolqOs2IUKs4OQGLyk85MP0w/fMrHwGG4uk1ffAGeTGwKy+zD7Xv2xhVGR',
'bXb/VDgp+zWntjDNyizYDL5pm4bbV8J360ARsqdP5qVW7eCDdkCr7HliiJpPT36nslJ4p2CENFQ0ZMRNS5qY3oxkNzaPacbtEQbM',
'33GX7QtIg88CfNzXdXVcP+jdoIUy1nzMUIDY9+kK0V2nBcf6NhOAb1JmsjPpaPMIva/S6WteyD62HIGMw8wfBibW17SHN3KbzXDI',
'7GxUQiNi8HNKtNPvlkDwtXWIwIHaX/YBj7d9mqT1+NsruU6bxR0rpkewBwVrJJaAM0l3X+exTmrOVS6YZf1tKOA7PIfWd0fy/B33',
'70n7fv9UA1Kk7/6X7r/a1iZcuA4KF6kw9/hJEW3UQK0wu67ya5Her9iAGpSVtfzOqouwaPw8qWKimUEsFMzPGcyZNwMXYfMewEsq',
'bBTqWTNcEtczBrxHnoSaXtcRowrcG/4UPNfP5CIHxk/z0zQq8WxNB9oCBD4tbz5C8QmNfOZfGbtKK5b/64IaEAUfByjHn59U+R6h',
'vODwVUo7uVPV9MXni6qVjx+qB02e237AxsB355EApIaJ2FAVk0hLjP+xK/jDZneeHcruaYIExesstSB3y4Sk9GdgbGIvXJS5t85M',
'fhb8vza+7SlPoQ+wjwowEWj4bYWP2jHkQ4ITrWZcwSoOGhG96vkgyz0ug6N3Lcld5VG9eHFYE4O0Hg4YhPccM0qiAmWY4TXxpb6D',
'xz+/9sJW4XpJBu9jElBwJ79H5DHFCybnraV39Du7SalxHVznSuBMiNbSfk8NiperVYX7N6Lp9zpOHWz0kj2DZ7rdhXp5MceU1FMd',
'lkkI4q5H4gCSX7jeFNFvjVxGXHVZZCgs/YxZaUZinWcFIdAcsauQ/C4H0fREiH9fuxJTvgK25CQ9tn2kwx+VN/3kDUQKfC1yXVU0',
'n4APCTFjhQnteuM0szBbF4s/7vyPOD4+z0Z0dEvsy1QJx9cSyhq5rc17q9+kzRumS7R/2AW6N18jpt1mCnNH18voFxQRiIsSM4A+',
'Bt55FJM5nLFVPR6JK+5L4T9DltCTjHwsk+hxxWe765v08J4PfKpxYMTKBrRTeqWm+4qRbpcdC1EisuEfxuG1N3NP1ay0R2eNMPOJ',
'QdM0AtI8/stM1Y84wufqk7SVtMjEpAP/wuOZXptK38F2LamlfKZo2gUNGIP+mZ/f3MfttxSz5wUidTLRv81OYNf5zeGvL1e5orsm',
'n1k4lq/XUJRqYNNdHja0FdIFAZ0Cm8syBw9kWf6ISWXSPkpQNg4zJlf779wfBqVJebWNVhCIMiCyYubFChtwyhbhG3p/chZRWKED',
'6ZvWgf0ifMdx057M+At8LFhXt8JfA5TKuTqrxavOIT/Ys33GYe+RsHnfEY4Mrk97Qi9kikHi73pRvLRuz8BqmPNodOdDheG8fs5s',
'QD4uisUOSp8HxyxadqRgNyK5mGIhny1zk0DMkLbO7UONWg+7s9MM7H/NtDJdlTqKcBTPFFLcZZtPwhSfW3T+gt6yADmSFRh9OBOj',
'a7As1clxjy6y2eKu5xtp20H6YBaFWVN4hX8eVrex0i9nn0ZF+fd5ewPjRLOZREh7qjzZVk/sz2k6lAa4+WkWHo+bZDie5TlSpuPv',
'uZRWJBcF7/gqqYc3+t76YaQEty6SgH0oRg4MykqXvF9WNJ6nwl62o3W8Rb2Beh0VNs4Zbv3Y/1CZKUh0U0bizS0U4fVhb74BENlJ',
'kSQc160g4ARaVMGXAuKyZPArpevbT80pHqmY17z2RpmvDmBx5StLvtW4e+HKhTp0XGc5cYEwYID6dOpzOSnQJeFEOjTryFdi8y9G',
'vx3PeRi+FQdt65k9RdosZzq1VruuBYI+er62+ha0M7anORF8grVi5ArhOgr5OCWEXkyUwkmAieMHDK9N+fcQEiZdyPNR5cl1Jrio',
'bp9DFmxWuO+p78a1oAOr96p0Cn/rF0pJi8SLIIa2RxEwZtoyuZB5SCSbrrl9s8+rHqyl9rx8eLJNRWkD6M/kyppWIUmanYGuJWEo',
'W5u077bbIvijWY2DC1SJeQpvGMzlZpAyZHsdQhgoJa5FAmCKB6UObKSnDfjPrFkd1Bcsq/3Mg+0GDaKgHNSoUC8JT2tJLre52LBO',
'qXWE9BCe6I65jEOBBPMzuU10zS4ORD4eaF2Eu0xaUg6zfOL/bbDH62ylDTcHIc2zHnTA0cByD4AKht57drVoptmOmP9uUPSIRZmQ',
'LakC+H9So+dsGmu4F+Bd3keYDDlNBESaTbQxVu3LLtSty6sQeGcHE0V23DUKGKIhOD35RdsBW3JLObX+J7JuciWLWgeHiSMWZBjQ',
'uYgFphXokKnSiczYotbETcbF0UkCeoLBe2U7LBEZz6bioekA8vCvwnjBtSDJiKQkcoeG8rL0cUghumhWu3pYTutaFqTDuds+0U6A',
'wDS6tprnLYzKlOQ55vRZTANCu/p5va7DgQlmAIQdW5yMpYJ7h+Au/C1nQdiUvhbiwLsVl9Dcg7R6xUJiviBoqCQo5MLJ1P4NN+zs',
'39TyWiHhy/tFBHZmzRSWqcbrPC5xJwc0/i1u6Bxmq/cc1UfQGUqkq2EDX4yGLKDEgpjK1xqdm3/C34OcZ0Maar/t+ki7rmbpGqAE',
'ItGlEoxw4JVma+jP42hoHVzePxlGJP5cB1T/cZZsUUppM+4wZy4Z1HKD4tlPHUQVUeSWd6NOnoSmbU12yBCnwk+Vci9l7Uxe3KrN',
'OhO0PmsllPqyCf4vZviPmy1nR/CKdJkQMwCwY02ZGViqs+b2fqi6WziYI+gwOVLJ7BIRFDHTQ9S0J79TuFYuqQL1m0yFMDZ6O07+',
'ijym731TFYO7ZZHOaEF6ejAHNhv6a3UsV06HD9nJOJU9K293fB99JawyFW5Zm68r7F0FotLQEC19S1VNsEdoZXCpIgH1E/6oIyBl',
'GbvSL7JT/P5FhJsb7lTexZk7IoF2emXqefwZyMqvws8sdK3anAKZ+gg489bSmepKq20qtcnuEJWrLYpf4tB7PW4VwF7HiqpNuVVy',
's8md/6NgU8gEAFk1feiAtDPARYHVJqnjiJe5nMAe//y6CR08weWfTeoRpvdGA4pJYBfIWI97lQ3X0CvC/LiOpVL9GLFHZh9TnVef',
'G5jEuF+LnvNlpODON4W5yaPF8Yg5aObRUaEWTY7w0ieMyLnKkz6E5gYVnLW4h3wjMdM4nVRaPM7JrszI/6yzX0nTk0RtrSHTIgF4',
'3c5tjENNcXo/kBHDk0PEjCKLbXKeMLgouAskPqZvAepH5Iwe5BAU7zj3cpauqXVvapAPclgOidm/IIwtOczH0XMcvqiAJPRE3Ojo',
'Ya5hOc5UkGMnCOBlZIdnlwsIILVp1QaHtVOhtZw1Flm11w/oNK82Trrx+Cqso/NBN4DHa7WACmKO38RS30RGBjhtwg4ELO0WaCSl',
'rez4aQN8aLXDNTJAZuHp4SIb8FbMevDxSDz+wyB6dQLIchN8MGYAE+4YzXtwFtOGFU7vCfU1MV9JU6U2wwEkDysnG5TqywNqDF/q',
'aj5q2zgstDAsejMtvIyanpdL/KtiAygmFjptxenQaygLHg9F3BxcluKCRIGZsg6mwzBT4lPypox/BtMKrna2qAB6ryhS'
].join('');

function ac() {
  if (broken) return null;
  if (!ctx) {
    if (!wx.createWebAudioContext) { broken = true; return null; }
    try {
      ctx = wx.createWebAudioContext();
      master = ctx.createGain();
      master.gain.value = 0.5;
      master.connect(ctx.destination);
    } catch (e) {
      broken = true;
      return null;
    }
  }
  return ctx;
}

/* 单音：f0→f1 指数滑音 + 快起慢落包络（4ms 起音，杜绝"咔"的爆点） */
function tone(f0, f1, dur, type, vol, when) {
  const c = ac();
  if (!c) return;
  const t = c.currentTime + (when || 0);
  const o = c.createOscillator();
  const g = c.createGain();
  o.type = type;
  o.frequency.setValueAtTime(f0, t);
  if (f1 && f1 !== f0) o.frequency.exponentialRampToValueAtTime(f1, t + dur);
  g.gain.setValueAtTime(0.0001, t);
  g.gain.exponentialRampToValueAtTime(vol, t + 0.004);
  g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
  o.connect(g);
  g.connect(master);
  o.start(t);
  o.stop(t + dur + 0.02);
}

/* 白噪声缓冲：优先 createBuffer 现场生成；不可用时解码内嵌的 8bit WAV 兜底 */
function getNoise() {
  const c = ac();
  if (!c) return null;
  if (noiseBuf) return noiseBuf;
  try {
    if (c.createBuffer) {
      noiseBuf = c.createBuffer(1, Math.floor(c.sampleRate * 0.5), c.sampleRate);
      const d = noiseBuf.getChannelData(0);
      for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1;
      return noiseBuf;
    }
  } catch (e) { /* 落到解码兜底 */ }
  try {
    const arr = wx.base64ToArrayBuffer(NOISE_B64);
    // decodeAudioData 是异步的：首次先返回 null（这一声少个气感垫底，可接受），
    // 解码完成后 noiseBuf 就位，后续调用立刻可用
    c.decodeAudioData(arr, (buf) => { noiseBuf = buf; }, () => { broken = false; });
  } catch (e) { /* 兜底也失败：噪声声部直接跳过 */ }
  return noiseBuf;
}

/* 带通噪声：f0→f1 扫频（whoosh 的气感上扫 / lock 的咔点垫底） */
function noise(dur, vol, f0, f1, q, when) {
  const c = ac();
  if (!c) return;
  const buf = getNoise();
  if (!buf) return;
  const t = c.currentTime + (when || 0);
  const s = c.createBufferSource();
  s.buffer = buf;
  s.loop = true;
  const bp = c.createBiquadFilter();
  bp.type = 'bandpass';
  bp.Q.value = q || 2;
  bp.frequency.setValueAtTime(f0, t);
  if (f1 && f1 !== f0) bp.frequency.exponentialRampToValueAtTime(f1, t + dur);
  const g = c.createGain();
  g.gain.setValueAtTime(0.0001, t);
  g.gain.exponentialRampToValueAtTime(vol, t + dur * 0.25);
  g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
  s.connect(bp);
  bp.connect(g);
  g.connect(master);
  s.start(t);
  s.stop(t + dur + 0.02);
}

/* 音色库：参数与 design/drone-texture-preview.html 验收稿逐值一致 */
const SFX = {
  press()    { tone(1600, 1450, 0.02, 'triangle', 0.16); noise(0.012, 0.06, 3000, 3000, 3); },
  confirm()  { tone(1500, 1500, 0.03, 'triangle', 0.16); tone(2100, 2100, 0.035, 'triangle', 0.18, 0.045); },
  toggleOn() { tone(660, 990, 0.09, 'sine', 0.2); tone(990, 990, 0.05, 'sine', 0.1, 0.09); },
  toggleOff(){ tone(880, 560, 0.09, 'sine', 0.18); },
  whoosh()   { noise(0.15, 0.22, 420, 2300, 2.5); tone(2100, 2100, 0.03, 'triangle', 0.14, 0.15); },
  /* 干净的「咻」：纯带通噪声上扫（500→3200Hz，Q 收窄到 1.4 出气感），
     不叠 tick —— whoosh 头上那声 2100Hz 硬 tick 会把它读成"咔咻"（2026-11-15 用户反馈）。
     后半段叠一层向下扫的弱噪声（1200→400Hz）做"甩过去"的尾音，整体 ~0.3s。 */
  swish()    { noise(0.28, 0.3, 500, 3200, 1.4); noise(0.18, 0.12, 1200, 400, 1.8, 0.12); },
  lock()     { tone(190, 150, 0.06, 'square', 0.2); noise(0.02, 0.1, 900, 900, 2, 0.05); },
  refuse()   { tone(135, 130, 0.05, 'square', 0.22); tone(135, 128, 0.06, 'square', 0.22, 0.075); },
  holdStart(){ tone(300, 520, 0.07, 'sawtooth', 0.1); },
  holdEnd()  { tone(430, 270, 0.06, 'sawtooth', 0.08); },
  /* n = 个性档位 1/2/3：钢穹 1980 / 紫电 2240 / 极昼 2520Hz，主音 + 1.5 倍频泛音 */
  persona(n) {
    const f = [1980, 2240, 2520][n - 1] || 1980;
    tone(f, f * 0.995, 0.13, 'sine', 0.18);
    tone(f * 1.5, f * 1.5, 0.06, 'sine', 0.06, 0.02);
  },
  grab()     { tone(1200, 1150, 0.012, 'triangle', 0.07); },
  release()  { tone(900, 860, 0.012, 'triangle', 0.06); }
};

/* 唯一出口：sfx.play('press') / sfx.play('persona', 2)。
   每次调用独立 try/catch —— 某个音色失败不影响后续按键。 */
function play(name, arg) {
  try {
    if (broken || !SFX[name]) return;
    SFX[name](arg);
  } catch (e) { /* 音效永远不阻塞控制流程 */ }
}

module.exports = { play };
