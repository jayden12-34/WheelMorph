import asyncio
import json
import os
import signal
import sys
import threading
import time
import http.server

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

# ── Constants (mirrors keyboard.py Steam Deck section) ───────────────────────
SD_DEADZONE  = 0.12
SD_LEG_STEP  = 15

HTTP_PORT = 8080
WS_PORT   = 8765

# ── Embedded web UI ───────────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>WHEEL TELEOP ◈ ROS2</title>
<style>
:root{
  --bg:#0a1a35;--panel:#122448;--dark:#060e20;
  --cyan:#40dcff;--cdim:#205878;
  --green:#00f5bc;--gdim:#005a48;
  --red:#ff4060;--rdim:#550020;
  --purple:#7aabff;--pdim:#1e3888;
  --white:#e8f8ff;--gray:#4888d0;
}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
html,body{width:100%;height:100vh;background:var(--bg);color:var(--white);
  font-family:'Courier New',monospace;overflow:hidden;touch-action:none;user-select:none;}
#app{display:flex;flex-direction:column;height:100vh;}

/* ── Header ── */
#hdr{background:var(--bg);border-bottom:1px solid var(--cdim);padding:5px 12px;
  display:flex;align-items:center;gap:10px;flex-shrink:0;}
#hb{width:8px;height:8px;border-radius:50%;background:var(--cdim);}
.title{color:var(--cyan);font-size:13px;font-weight:bold;flex:1;}
#gp-lbl{color:var(--gray);font-size:10px;}
#link-lbl{color:var(--red);font-size:10px;font-weight:bold;}

/* ── Speed bar ── */
#spd-bar{background:var(--panel);border-bottom:1px solid var(--cdim);
  padding:5px 12px;display:flex;align-items:center;gap:10px;flex-shrink:0;flex-wrap:wrap;}
#spd-bar label{color:var(--cyan);font-size:10px;white-space:nowrap;}
#spd-range{flex:1;max-width:260px;accent-color:var(--cyan);}
#spd-num{color:var(--white);font-size:11px;min-width:32px;}

/* ── Main 3-column ── */
#main{display:flex;flex:1;overflow:hidden;}
.side{width:250px;flex-shrink:0;background:var(--panel);display:flex;
  flex-direction:column;align-items:center;padding:7px 6px;gap:6px;overflow:hidden;}
#left{border-right:1px solid var(--cdim);}
#right{border-left:1px solid var(--cdim);}
.center{flex:1;display:flex;flex-direction:column;align-items:center;
  padding:6px 8px;gap:5px;overflow:hidden;}

/* ── Section headers ── */
.shdr{color:var(--cyan);font-size:9px;font-weight:bold;letter-spacing:1px;
  border-bottom:1px solid var(--cdim);width:100%;text-align:center;padding-bottom:2px;}

/* ── Buttons ── */
.btn{background:var(--dark);border:1px solid var(--cdim);color:var(--white);
  font-family:'Courier New',monospace;font-size:10px;font-weight:bold;
  border-radius:5px;cursor:pointer;display:flex;align-items:center;justify-content:center;
  text-align:center;line-height:1.2;transition:background .08s,border-color .08s;}
.btn.pressed,.btn:active{background:var(--cdim);border-color:var(--cyan);color:var(--cyan);}
.btn-g{color:var(--green);border-color:var(--gdim);}
.btn-g.pressed,.btn-g:active{background:var(--gdim);border-color:var(--green);}
.btn-r{color:var(--red);border-color:var(--rdim);}
.btn-r.pressed,.btn-r:active{background:var(--rdim);border-color:var(--red);}
.btn-p{color:var(--purple);border-color:var(--pdim);}
.btn-p.pressed,.btn-p:active{background:var(--pdim);border-color:var(--purple);}

/* ── Paddle row ── */
.pad-row{display:flex;gap:5px;width:100%;}
.pad-btn{flex:1;height:34px;}

/* ── Shoulder ── */
.sh-btn{width:100%;height:38px;}

/* ── D-pad ── */
#dpad{display:grid;grid-template-columns:repeat(3,42px);
  grid-template-rows:repeat(3,42px);gap:3px;}
.dp{width:42px;height:42px;font-size:14px;}

/* ── Face buttons ── */
#face{display:grid;grid-template-columns:repeat(3,44px);
  grid-template-rows:repeat(3,44px);gap:3px;}
.fb{width:44px;height:44px;border-radius:50%;font-size:12px;}

/* ── Joystick canvas ── */
.js-wrap{display:flex;flex-direction:column;align-items:center;gap:3px;}
.js-lbl{color:var(--cyan);font-size:9px;font-weight:bold;}
canvas.js{border-radius:50%;touch-action:none;cursor:crosshair;}

/* ── Wheel/leg grid ── */
.wg{display:grid;grid-template-columns:repeat(2,1fr);gap:4px;width:100%;}
.wc{background:var(--dark);border:1px solid var(--cdim);border-radius:4px;
  padding:3px 6px;text-align:center;}
.wc .wl{color:var(--gray);font-size:8px;}
.wc .wv{font-size:13px;font-weight:bold;color:var(--gray);}

/* ── Robot diagram ── */
#diag-wrap{flex:1;width:100%;background:var(--dark);border:1px solid var(--cdim);
  border-radius:4px;position:relative;min-height:60px;}
#diag{position:absolute;top:0;left:0;width:100%;height:100%;}

/* ── Current bar ── */
#curr-bar{width:100%;background:var(--dark);border:1px solid var(--cdim);
  border-radius:4px;padding:5px 8px;font-size:9px;flex-shrink:0;}
.curr-row{display:flex;gap:14px;margin-top:3px;}
.curr-sect{}
.curr-sect .sh{color:var(--cyan);font-size:8px;font-weight:bold;margin-bottom:2px;}
.cg{display:grid;grid-template-columns:repeat(4,1fr);gap:3px;}
.cc{text-align:center;}
.cc .cl{color:var(--gray);font-size:8px;}
.cc .cv{font-size:11px;font-weight:bold;}
#tot-curr{font-size:15px;font-weight:bold;color:var(--white);
  display:flex;align-items:center;margin-left:auto;}
</style>
</head>
<body>
<div id="app">

  <!-- Header -->
  <div id="hdr">
    <div id="hb"></div>
    <div class="title">WHEEL TELEOP ◈ ROS2</div>
    <div id="gp-lbl">GAMEPAD: —</div>
    <div id="link-lbl">● NO LINK</div>
  </div>

  <!-- Speed / quick-action bar -->
  <div id="spd-bar">
    <label>SPEED %</label>
    <input type="range" id="spd-range" min="0" max="100" value="20">
    <span id="spd-num">20%</span>
    <button class="btn btn-r" id="btn-l2"
      style="width:80px;height:28px;font-size:10px;">L2 RESET</button>
    <button class="btn" id="btn-estop"
      style="width:90px;height:28px;color:#ff0040;border-color:#550020;font-size:11px;">⚠ E-STOP</button>
  </div>

  <!-- Main area -->
  <div id="main">

    <!-- LEFT: paddles · L1 · left-stick · dpad -->
    <div class="side" id="left">
      <div class="shdr">LEFT CONTROLS</div>

      <div class="pad-row">
        <button class="btn btn-p pad-btn" id="btn-l4">L4<br><span style="font-size:8px">FL wheel</span></button>
        <button class="btn btn-p pad-btn" id="btn-l5">L5<br><span style="font-size:8px">BL wheel</span></button>
      </div>

      <button class="btn btn-g sh-btn" id="btn-l1">L1 — EXTEND ALL LEGS</button>

      <div class="js-wrap">
        <div class="js-lbl">LEFT STICK — ARCADE DRIVE</div>
        <canvas class="js" id="js-left" width="130" height="130"></canvas>
      </div>

      <div class="js-lbl">D-PAD — EXTEND LEGS</div>
      <div id="dpad">
        <div></div>
        <button class="btn btn-p dp" id="dp-up">▲<br><span style="font-size:8px">FL</span></button>
        <div></div>
        <button class="btn btn-p dp" id="dp-left">◄<br><span style="font-size:8px">BL</span></button>
        <div></div>
        <button class="btn btn-p dp" id="dp-right">►<br><span style="font-size:8px">FR</span></button>
        <div></div>
        <button class="btn btn-p dp" id="dp-down">▼<br><span style="font-size:8px">BR</span></button>
        <div></div>
      </div>
    </div>

    <!-- CENTER: readings · diagram · currents -->
    <div class="center">
      <div class="shdr">WHEEL SPEEDS</div>
      <div class="wg">
        <div class="wc"><div class="wl">FL (0)</div><div class="wv" id="ws0">0</div></div>
        <div class="wc"><div class="wl">FR (2)</div><div class="wv" id="ws2">0</div></div>
        <div class="wc"><div class="wl">BL (1)</div><div class="wv" id="ws1">0</div></div>
        <div class="wc"><div class="wl">BR (3)</div><div class="wv" id="ws3">0</div></div>
      </div>

      <div class="shdr">LEG ANGLES</div>
      <div class="wg">
        <div class="wc"><div class="wl">FL (0)</div><div class="wv" id="la0" style="color:var(--purple)">0°</div></div>
        <div class="wc"><div class="wl">FR (2)</div><div class="wv" id="la2" style="color:var(--purple)">0°</div></div>
        <div class="wc"><div class="wl">BL (1)</div><div class="wv" id="la1" style="color:var(--purple)">0°</div></div>
        <div class="wc"><div class="wl">BR (3)</div><div class="wv" id="la3" style="color:var(--purple)">0°</div></div>
      </div>

      <div class="shdr">ROBOT DIAGRAM</div>
      <div id="diag-wrap"><canvas id="diag"></canvas></div>

      <div id="curr-bar">
        <div class="shdr" style="border:none;padding:0;margin-bottom:2px;">CURRENT DRAW</div>
        <div class="curr-row">
          <div class="curr-sect">
            <div class="sh">WHEELS (mA)</div>
            <div class="cg">
              <div class="cc"><div class="cl">FL</div><div class="cv" id="wc0">0</div></div>
              <div class="cc"><div class="cl">BL</div><div class="cv" id="wc1">0</div></div>
              <div class="cc"><div class="cl">FR</div><div class="cv" id="wc2">0</div></div>
              <div class="cc"><div class="cl">BR</div><div class="cv" id="wc3">0</div></div>
            </div>
          </div>
          <div class="curr-sect">
            <div class="sh">LEGS (mA)</div>
            <div class="cg">
              <div class="cc"><div class="cl">FL</div><div class="cv" id="lc0" style="color:var(--purple)">0</div></div>
              <div class="cc"><div class="cl">BL</div><div class="cv" id="lc1" style="color:var(--purple)">0</div></div>
              <div class="cc"><div class="cl">FR</div><div class="cv" id="lc2" style="color:var(--purple)">0</div></div>
              <div class="cc"><div class="cl">BR</div><div class="cv" id="lc3" style="color:var(--purple)">0</div></div>
            </div>
          </div>
          <div id="tot-curr">0 mA</div>
        </div>
      </div>
    </div>

    <!-- RIGHT: paddles · R1 · face buttons · right-stick (throttle) -->
    <div class="side" id="right">
      <div class="shdr">RIGHT CONTROLS</div>

      <div class="pad-row">
        <button class="btn btn-p pad-btn" id="btn-r4">R4<br><span style="font-size:8px">FR wheel</span></button>
        <button class="btn btn-p pad-btn" id="btn-r5">R5<br><span style="font-size:8px">BR wheel</span></button>
      </div>

      <button class="btn btn-r sh-btn" id="btn-r1">R1 — RETRACT ALL LEGS</button>

      <div class="js-lbl">FACE BUTTONS — SNAP LEG TO 0°</div>
      <div id="face">
        <div></div>
        <button class="btn fb" id="btn-y" style="color:#ffff40;border-color:#666600;background:#1a1a00;">Y<br><span style="font-size:8px">FL</span></button>
        <div></div>
        <button class="btn fb" id="btn-x" style="color:#4080ff;border-color:#002288;background:#000a20;">X<br><span style="font-size:8px">BL</span></button>
        <div></div>
        <button class="btn fb" id="btn-b" style="color:#ff4040;border-color:#880000;background:#1a0000;">B<br><span style="font-size:8px">FR</span></button>
        <div></div>
        <button class="btn fb" id="btn-a" style="color:#40ff40;border-color:#006600;background:#001a00;">A<br><span style="font-size:8px">BR</span></button>
        <div></div>
      </div>

      <div class="js-wrap">
        <div class="js-lbl">RIGHT STICK — THROTTLE (Y)</div>
        <canvas class="js" id="js-right" width="130" height="130"></canvas>
      </div>
    </div>

  </div><!-- #main -->
</div><!-- #app -->

<script>
// ════════════════════════════════════════════════
// WebSocket
// ════════════════════════════════════════════════
const WS_URL = `ws://${location.hostname}:8765`;
let ws = null, wsReady = false;

function connect() {
  ws = new WebSocket(WS_URL);
  ws.onopen  = () => { wsReady = true;  setLink(true); };
  ws.onclose = () => { wsReady = false; setLink(false); setTimeout(connect, 2000); };
  ws.onerror = () => ws.close();
  ws.onmessage = e => { try { const d = JSON.parse(e.data); if (d.type==='state') applyState(d); } catch{} };
}
connect();

function setLink(on) {
  const el = document.getElementById('link-lbl');
  el.textContent = on ? '● LINKED' : '● NO LINK';
  el.style.color  = on ? '#00ff44' : '#ff4060';
}

// ════════════════════════════════════════════════
// Controller raw state (not all fields sent)
// ════════════════════════════════════════════════
const raw = {
  lx:0, ly:0, ry:0,
  l2:false, l1:false, r1:false,
  l4:false, l5:false, r4:false, r5:false,
  btn_a:false, btn_b:false, btn_x:false, btn_y:false,
  dp_up:false, dp_down:false, dp_left:false, dp_right:false,
};

function buildMsg() {
  return JSON.stringify({
    type:'ctrl',
    lx: raw.lx, ly: raw.ly, ry: raw.ry,
    l2: raw.l2, l1: raw.l1, r1: raw.r1,
    l4: raw.l4, l5: raw.l5, r4: raw.r4, r5: raw.r5,
    btn_a: raw.btn_a, btn_b: raw.btn_b,
    btn_x: raw.btn_x, btn_y: raw.btn_y,
    dpad: [
      (raw.dp_right?1:0)-(raw.dp_left?1:0),
      (raw.dp_up?1:0)-(raw.dp_down?1:0),
    ],
  });
}

setInterval(() => { if (wsReady) ws.send(buildMsg()); }, 50);

// ════════════════════════════════════════════════
// Display update
// ════════════════════════════════════════════════
function applyState(d) {
  if (d.speed_pct !== undefined) {
    sldEl.value = d.speed_pct;
    document.getElementById('spd-num').textContent = d.speed_pct + '%';
  }
  for (let i=0;i<4;i++) {
    const v = d.wheel_speed[i];
    const el = document.getElementById('ws'+i);
    if (el) { el.textContent=v; el.style.color=v>0?'#00f5bc':v<0?'#ff4060':'#4888d0'; }
    const la = document.getElementById('la'+i);
    if (la) la.textContent = d.leg_angles[i]+'°';
    const wc = document.getElementById('wc'+i);
    if (wc) { wc.textContent=d.wheel_currents[i]; wc.style.color=Math.abs(d.wheel_currents[i])>20?'#00f5bc':'#4888d0'; }
    const lc = document.getElementById('lc'+i);
    if (lc) { lc.textContent=d.leg_currents[i]; lc.style.color=Math.abs(d.leg_currents[i])>20?'#7aabff':'#1e3888'; }
  }
  const tot = d.wheel_currents.reduce((a,b)=>a+Math.abs(b),0)+d.leg_currents.reduce((a,b)=>a+Math.abs(b),0);
  const te = document.getElementById('tot-curr');
  if (te) { te.textContent=tot>=1000?(tot/1000).toFixed(2)+' A':tot+' mA'; te.style.color=tot>5000?'#ff4060':tot>500?'#40dcff':'#e8f8ff'; }
  if (d.linked) setLink(true);
  drawDiagram(d.wheel_speed, d.leg_angles);
}

// ════════════════════════════════════════════════
// Robot diagram
// ════════════════════════════════════════════════
const dcanvas = document.getElementById('diag');
const dctx = dcanvas.getContext('2d');
let lastSpd = null;

function resizeDiag() {
  const wrap = document.getElementById('diag-wrap');
  dcanvas.width  = wrap.clientWidth  || 200;
  dcanvas.height = wrap.clientHeight || 100;
  drawDiagram(lastSpd||[0,0,0,0],[0,0,0,0]);
}

function drawDiagram(speeds, legs) {
  lastSpd = speeds;
  const c=dctx, w=dcanvas.width, h=dcanvas.height;
  if (w<30||h<30) return;
  c.fillStyle='#060e20'; c.fillRect(0,0,w,h);
  c.strokeStyle='#102840'; c.lineWidth=1;
  for(let x=0;x<w;x+=40){c.beginPath();c.moveTo(x,0);c.lineTo(x,h);c.stroke();}
  for(let y=0;y<h;y+=40){c.beginPath();c.moveTo(0,y);c.lineTo(w,y);c.stroke();}
  const cx=w/2,cy=h/2;
  // HUD corners
  c.strokeStyle='#40dcff';c.lineWidth=2;
  [[8,8,1,1],[w-8,8,-1,1],[8,h-8,1,-1],[w-8,h-8,-1,-1]].forEach(([bx,by,sx,sy])=>{
    c.beginPath();c.moveTo(bx,by);c.lineTo(bx+sx*14,by);c.stroke();
    c.beginPath();c.moveTo(bx,by);c.lineTo(bx,by+sy*14);c.stroke();
  });
  c.fillStyle='#205878';c.font='9px Courier New';c.textAlign='center';
  c.fillText('▲ FORWARD',cx,14);
  const bw=Math.min(38,w*.08),bh=Math.min(52,h*.18);
  c.fillStyle='#0a2240';c.strokeStyle='#40dcff';c.lineWidth=2;
  c.beginPath();c.rect(cx-bw,cy-bh,bw*2,bh*2);c.fill();c.stroke();
  c.strokeStyle='#205878';c.lineWidth=1;
  c.beginPath();c.moveTo(cx,cy-bh);c.lineTo(cx,cy+bh);c.stroke();
  c.beginPath();c.moveTo(cx-bw,cy);c.lineTo(cx+bw,cy);c.stroke();
  c.fillStyle='#40dcff';c.font='bold 8px Courier New';c.fillText('ROBOT',cx,cy+3);
  const wxo=Math.min(65,w*.17),wyo=Math.min(60,h*.21);
  const wpos=[[cx-wxo,cy-wyo],[cx-wxo,cy+wyo],[cx+wxo,cy-wyo],[cx+wxo,cy+wyo]];
  const wlbls=['FL','BL','FR','BR'];
  wpos.forEach(([wx,wy],i)=>{
    const s=speeds[i];
    const col=s>0?'#00f5bc':s<0?'#ff4060':'#4888d0';
    const dim=s>0?'#005a48':s<0?'#550020':'#152e48';
    c.strokeStyle=dim;c.lineWidth=5;c.beginPath();c.arc(wx,wy,16,0,Math.PI*2);c.stroke();
    c.fillStyle='#060e20';c.strokeStyle=col;c.lineWidth=2;c.beginPath();c.arc(wx,wy,10,0,Math.PI*2);c.fill();c.stroke();
    c.fillStyle=col;c.beginPath();c.arc(wx,wy,3,0,Math.PI*2);c.fill();
    c.fillStyle='#e8f8ff';c.font='8px Courier New';c.fillText(String(s),wx,wy+24);
    c.fillStyle=col;c.font='bold 8px Courier New';c.fillText(wlbls[i],wx,wy-19);
  });
}

new ResizeObserver(resizeDiag).observe(document.getElementById('diag-wrap'));
setTimeout(resizeDiag,80);

// ════════════════════════════════════════════════
// Virtual joystick
// ════════════════════════════════════════════════
class VJoy {
  constructor(id, opts={}) {
    this.c   = document.getElementById(id);
    this.ctx = this.c.getContext('2d');
    this.sz  = this.c.width;
    this.r   = this.sz/2;
    this.tr  = this.sz*.18;  // thumb radius
    this.md  = this.r-this.tr-5;  // max displacement
    this.x=0; this.y=0;
    this.tx=this.r; this.ty=this.r;
    this.on=false;
    this.yOnly = opts.yOnly||false;
    this.color = opts.color||'#40dcff';
    this.c.addEventListener('touchstart', e=>{e.preventDefault();this.on=true;const t=e.touches[0];this._mv(...this._rel(t.clientX,t.clientY));},{passive:false});
    this.c.addEventListener('touchmove',  e=>{e.preventDefault();if(this.on){const t=e.touches[0];this._mv(...this._rel(t.clientX,t.clientY));}},{passive:false});
    this.c.addEventListener('touchend',   e=>{e.preventDefault();this._rst();});
    this.c.addEventListener('mousedown',  e=>{this.on=true;this._mv(...this._rel(e.clientX,e.clientY));});
    window.addEventListener('mousemove',  e=>{if(this.on) this._mv(...this._rel(e.clientX,e.clientY));});
    window.addEventListener('mouseup',    ()=>{if(this.on) this._rst();});
    this._draw();
  }
  _rel(cx,cy){const r=this.c.getBoundingClientRect(),sx=this.sz/r.width,sy=this.sz/r.height;return[(cx-r.left)*sx,(cy-r.top)*sy];}
  _mv(px,py){
    let dx=px-this.r, dy=py-this.r;
    if(this.yOnly) dx=0;
    const d=Math.sqrt(dx*dx+dy*dy);
    if(d>this.md){dx=dx/d*this.md;dy=dy/d*this.md;}
    this.tx=this.r+dx; this.ty=this.r+dy;
    this.x=dx/this.md; this.y=dy/this.md;
    this._draw();
  }
  _rst(){this.on=false;this.x=0;this.y=0;this.tx=this.r;this.ty=this.r;this._draw();}
  _draw(){
    const c=this.ctx,s=this.sz,r=this.r;
    c.clearRect(0,0,s,s);
    c.fillStyle='#060e20';c.strokeStyle=this.on?this.color:'#205878';c.lineWidth=2;
    c.beginPath();c.arc(r,r,r-3,0,Math.PI*2);c.fill();c.stroke();
    c.strokeStyle='#102840';c.lineWidth=1;
    c.beginPath();c.moveTo(r,3);c.lineTo(r,s-3);c.stroke();
    c.beginPath();c.moveTo(3,r);c.lineTo(s-3,r);c.stroke();
    if(this.yOnly){c.strokeStyle='#205878';c.lineWidth=1;c.beginPath();c.moveTo(r-12,3);c.lineTo(r-12,s-3);c.stroke();c.beginPath();c.moveTo(r+12,3);c.lineTo(r+12,s-3);c.stroke();}
    c.fillStyle=this.on?this.color:'#205878';c.strokeStyle=this.color;c.lineWidth=2;
    c.beginPath();c.arc(this.tx,this.ty,this.tr,0,Math.PI*2);c.fill();c.stroke();
    c.fillStyle=this.on?'#060e20':this.color;c.beginPath();c.arc(this.tx,this.ty,4,0,Math.PI*2);c.fill();
  }
  syncFrom(nx,ny){this.x=nx;this.y=ny;this.tx=this.r+nx*this.md;this.ty=this.r+ny*this.md;this._draw();}
}

const jsL = new VJoy('js-left',  {color:'#40dcff'});
const jsR = new VJoy('js-right', {color:'#00f5bc', yOnly:true});

setInterval(()=>{raw.lx=jsL.x;raw.ly=jsL.y;raw.ry=jsR.y;},25);

// ════════════════════════════════════════════════
// Button wiring
// ════════════════════════════════════════════════
function holdBtn(id, key) {
  const el = document.getElementById(id);
  if (!el) return;
  const dn = e=>{e.preventDefault();raw[key]=true; el.classList.add('pressed');};
  const up = e=>{e.preventDefault();raw[key]=false;el.classList.remove('pressed');};
  el.addEventListener('touchstart',dn,{passive:false});
  el.addEventListener('touchend',  up,{passive:false});
  el.addEventListener('mousedown', dn);
  el.addEventListener('mouseup',   up);
  el.addEventListener('mouseleave',up);
}

function tapBtn(id, key) {
  const el = document.getElementById(id);
  if (!el) return;
  const dn = e=>{e.preventDefault();raw[key]=true;el.classList.add('pressed');setTimeout(()=>{raw[key]=false;el.classList.remove('pressed');},150);};
  el.addEventListener('touchstart',dn,{passive:false});
  el.addEventListener('mousedown', dn);
}

holdBtn('btn-l1','l1'); holdBtn('btn-r1','r1');
holdBtn('btn-l4','l4'); holdBtn('btn-l5','l5');
holdBtn('btn-r4','r4'); holdBtn('btn-r5','r5');
holdBtn('btn-l2','l2');
holdBtn('dp-up',  'dp_up');  holdBtn('dp-down', 'dp_down');
holdBtn('dp-left','dp_left');holdBtn('dp-right','dp_right');
tapBtn('btn-y','btn_y'); tapBtn('btn-x','btn_x');
tapBtn('btn-b','btn_b'); tapBtn('btn-a','btn_a');

// E-STOP
function doEstop(e){
  e.preventDefault();
  if(wsReady) ws.send(JSON.stringify({type:'estop'}));
  jsL._rst(); jsR._rst();
  Object.keys(raw).forEach(k=>{ if(typeof raw[k]==='boolean') raw[k]=false; else raw[k]=0; });
}
document.getElementById('btn-estop').addEventListener('touchstart',doEstop,{passive:false});
document.getElementById('btn-estop').addEventListener('mousedown', doEstop);

// Speed slider
const sldEl=document.getElementById('spd-range');
sldEl.addEventListener('input',()=>{
  const v=parseInt(sldEl.value);
  document.getElementById('spd-num').textContent=v+'%';
  if(wsReady) ws.send(JSON.stringify({type:'speed_pct',value:v}));
});

// ════════════════════════════════════════════════
// Gamepad API (physical Steam Deck buttons)
// ════════════════════════════════════════════════
let gpIdx=null;
window.addEventListener('gamepadconnected', e=>{
  gpIdx=e.gamepad.index;
  document.getElementById('gp-lbl').textContent='GAMEPAD: '+e.gamepad.id.substring(0,18);
  document.getElementById('gp-lbl').style.color='#00f5bc';
});
window.addEventListener('gamepaddisconnected',()=>{
  gpIdx=null;
  document.getElementById('gp-lbl').textContent='GAMEPAD: —';
  document.getElementById('gp-lbl').style.color='#4888d0';
});

function b(gp,i){return gp.buttons[i]&&gp.buttons[i].pressed||false;}

function pollGP(){
  if(gpIdx!==null){
    const gp=navigator.getGamepads()[gpIdx];
    if(gp){
      // Axes: 0=LX,1=LY,2=RX,3=RY (standard mapping)
      raw.lx = gp.axes[0]||0;
      raw.ly = gp.axes[1]||0;
      raw.ry = gp.axes[3]||0;
      // L2 trigger (button 6, analog)
      raw.l2 = gp.buttons[6]?gp.buttons[6].value>0.3:false;
      raw.l1 = b(gp,4); raw.r1=b(gp,5);
      raw.btn_a=b(gp,0); raw.btn_b=b(gp,1);
      raw.btn_x=b(gp,2); raw.btn_y=b(gp,3);
      // D-pad (standard: 12=up,13=down,14=left,15=right)
      raw.dp_up=b(gp,12); raw.dp_down=b(gp,13);
      raw.dp_left=b(gp,14); raw.dp_right=b(gp,15);
      // Back paddles (Steam Deck: varies; try 11,13,12,14 first; adjust if needed)
      raw.l4=b(gp,11)||false;
      raw.l5=b(gp,17)||false;
      raw.r4=b(gp,15)||false;
      raw.r5=b(gp,16)||false;
      // Sync virtual stick visuals
      jsL.syncFrom(raw.lx,raw.ly);
      jsR.syncFrom(0,raw.ry);
    }
  }
  requestAnimationFrame(pollGP);
}
requestAnimationFrame(pollGP);

// ════════════════════════════════════════════════
// Heartbeat blink
// ════════════════════════════════════════════════
let hbOn=false;
setInterval(()=>{hbOn=!hbOn;document.getElementById('hb').style.background=hbOn?'#40dcff':'#205878';},600);
</script>
</body>
</html>"""


# ── HTTP server (serves the HTML page) ───────────────────────────────────────
class _HTTPHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        body = HTML.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass  # suppress per-request log noise


def _run_http(port: int):
    srv = http.server.HTTPServer(('0.0.0.0', port), _HTTPHandler)
    srv.serve_forever()


# ── ROS2 node ─────────────────────────────────────────────────────────────────
class WebTeleop(Node):

    def __init__(self):
        super().__init__('web_teleop')
        self.pub       = self.create_publisher(Int32MultiArray, 'wheel_commands', 10)
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)

        self.wheel_speed    = [0, 0, 0, 0]
        self.leg_angles     = [0, 0, 0, 0]
        self.wheel_currents = [0, 0, 0, 0]
        self.leg_currents   = [0, 0, 0, 0]
        self.wheel_max      = 50
        self.speed_pct      = 20
        self.lock           = threading.Lock()

        self._ctrl = dict(lx=0.0, ly=0.0, ry=0.0,
                          l2=False, l1=False, r1=False,
                          l4=False, l5=False, r4=False, r5=False,
                          btn_a=False, btn_b=False, btn_x=False, btn_y=False,
                          dpad=[0, 0])

        self.create_subscription(
            Int32MultiArray, 'wheel_currents', self._wheel_cb, 10)
        self.create_subscription(
            Int32MultiArray, 'leg_currents', self._leg_cb, 10)

        self._ws_clients: set = set()
        self._ws_clients_lock = threading.Lock()

    # ── ROS subscribers ──────────────────────────────────────────────────────

    def _wheel_cb(self, msg):
        with self.lock:
            self.wheel_currents = list(msg.data[:4])

    def _leg_cb(self, msg):
        with self.lock:
            self.leg_currents = list(msg.data[:4])

    # ── Controller processing ─────────────────────────────────────────────────

    def process_ctrl(self, ctrl: dict):
        """Apply one 20 Hz frame of controller state."""
        with self.lock:
            angles = list(self.leg_angles)

        if ctrl.get('l2'):
            with self.lock:
                self.wheel_speed = [0, 0, 0, 0]
                self.leg_angles  = [0, 0, 0, 0]
            return

        lx = float(ctrl.get('lx', 0))
        ly = float(ctrl.get('ly', 0))
        ry = float(ctrl.get('ry', 0))
        if abs(lx) < SD_DEADZONE: lx = 0.0
        if abs(ly) < SD_DEADZONE: ly = 0.0

        # Right stick Y directly sets speed_pct (push up = faster).
        throttle = max(0.0, -ry)
        if throttle < SD_DEADZONE:
            throttle = 0.0
        else:
            self.speed_pct = int(throttle * 100)
        effective = self.speed_pct / 100.0 * self.wheel_max
        forward   = -ly * effective
        turn      =  lx * effective

        def clamp(v):
            return int(max(-self.wheel_max, min(self.wheel_max, v)))

        ws = [
            clamp(forward + turn),  # FL
            clamp(forward + turn),  # BL
            clamp(forward - turn),  # FR
            clamp(forward - turn),  # BR
        ]

        dpad = ctrl.get('dpad', [0, 0])
        dx, dy = int(dpad[0]), int(dpad[1])
        if dy ==  1: angles[0] = min(180, angles[0] + SD_LEG_STEP)
        if dx == -1: angles[1] = min(180, angles[1] + SD_LEG_STEP)
        if dx ==  1: angles[2] = min(180, angles[2] + SD_LEG_STEP)
        if dy == -1: angles[3] = min(180, angles[3] + SD_LEG_STEP)

        if ctrl.get('l1'): angles = [min(180, a + SD_LEG_STEP) for a in angles]
        if ctrl.get('r1'): angles = [max(0,   a - SD_LEG_STEP) for a in angles]

        if ctrl.get('btn_y'): angles[0] = max(0, angles[0] - SD_LEG_STEP)
        if ctrl.get('btn_x'): angles[1] = max(0, angles[1] - SD_LEG_STEP)
        if ctrl.get('btn_b'): angles[2] = max(0, angles[2] - SD_LEG_STEP)
        if ctrl.get('btn_a'): angles[3] = max(0, angles[3] - SD_LEG_STEP)

        paddle_spd = max(1, int(self.wheel_max * self.speed_pct / 100))
        if ctrl.get('l4'): ws[0] = paddle_spd
        if ctrl.get('l5'): ws[1] = paddle_spd
        if ctrl.get('r4'): ws[2] = paddle_spd
        if ctrl.get('r5'): ws[3] = paddle_spd

        with self.lock:
            self.wheel_speed = ws
            self.leg_angles  = angles

    def emergency_stop(self):
        with self.lock:
            self.wheel_speed = [0, 0, 0, 0]
            self.leg_angles  = [0, 0, 0, 0]
        msg = Int32MultiArray()
        msg.data = [0] * 8
        self.pub.publish(msg)
        estop = Bool()
        estop.data = True
        self.estop_pub.publish(estop)

    # ── Publish loop (background thread) ────────────────────────────────────

    def publish_loop(self):
        dt = 1.0 / 20
        while rclpy.ok():
            with self.lock:
                ctrl = dict(self._ctrl)
            self.process_ctrl(ctrl)
            msg = Int32MultiArray()
            with self.lock:
                msg.data = self.wheel_speed + self.leg_angles
            self.pub.publish(msg)
            time.sleep(dt)

    # ── State snapshot for broadcast ─────────────────────────────────────────

    def state_json(self) -> str:
        with self.lock:
            return json.dumps({
                'type':           'state',
                'wheel_speed':    list(self.wheel_speed),
                'leg_angles':     list(self.leg_angles),
                'wheel_currents': list(self.wheel_currents),
                'leg_currents':   list(self.leg_currents),
                'speed_pct':      self.speed_pct,
                'linked':         self.pub.get_subscription_count() > 0,
            })


# ── WebSocket handlers ────────────────────────────────────────────────────────
_CTRL_FIELDS = ('lx','ly','ry','l2','l1','r1',
                'l4','l5','r4','r5',
                'btn_a','btn_b','btn_x','btn_y','dpad')


async def ws_client(node: WebTeleop, websocket):
    with node._ws_clients_lock:
        node._ws_clients.add(websocket)
    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            t = msg.get('type')
            if t == 'ctrl':
                with node.lock:
                    for k in _CTRL_FIELDS:
                        if k in msg:
                            node._ctrl[k] = msg[k]
            elif t == 'estop':
                node.emergency_stop()
            elif t == 'speed_pct':
                node.speed_pct = max(0, min(100, int(msg.get('value', 20))))
    finally:
        with node._ws_clients_lock:
            node._ws_clients.discard(websocket)


async def broadcast_loop(node: WebTeleop):
    while True:
        await asyncio.sleep(0.05)
        js = node.state_json()
        with node._ws_clients_lock:
            clients = set(node._ws_clients)
        if clients:
            await asyncio.gather(
                *[c.send(js) for c in clients],
                return_exceptions=True,
            )


async def run_ws(node: WebTeleop, host: str = '0.0.0.0', port: int = WS_PORT):
    from websockets.asyncio.server import serve
    async with serve(lambda ws: ws_client(node, ws), host, port):
        asyncio.create_task(broadcast_loop(node))
        node.get_logger().info(
            f'WebTeleop ready — '
            f'UI: http://0.0.0.0:{HTTP_PORT}  WS: ws://0.0.0.0:{WS_PORT}'
        )
        await asyncio.Future()  # run until cancelled


# ── ESC key watcher ──────────────────────────────────────────────────────────
def _esc_watcher():
    """Watch stdin for ESC and send SIGINT so the process shuts down cleanly."""
    try:
        import tty, termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
    except Exception:
        return  # stdin is not a real tty (piped/redirected) — skip silently
    try:
        tty.setcbreak(fd)
        while True:
            ch = os.read(fd, 1)
            if ch == b'\x1b':
                os.kill(os.getpid(), signal.SIGINT)
                break
    except Exception:
        pass
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except Exception:
            pass


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = WebTeleop()

    threading.Thread(target=_run_http, args=(HTTP_PORT,), daemon=True).start()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=node.publish_loop, daemon=True).start()
    threading.Thread(target=_esc_watcher, daemon=True).start()

    try:
        asyncio.run(run_ws(node))
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
