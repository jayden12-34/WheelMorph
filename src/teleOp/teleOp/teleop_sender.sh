#!/bin/bash
# Wheel Teleop sender — add this file to Steam as a Non-Steam Game.
# In Steam game properties → Compatibility: ensure "Force Proton" is OFF.
# Set the robot IP in Launch Options: --host 192.168.1.XXX

PYTHON=$(command -v python3 2>/dev/null || echo /usr/bin/python3)

if ! "$PYTHON" -c "import pygame" 2>/dev/null; then
    "$PYTHON" -m pip install --user pygame
fi

exec "$PYTHON" - "$@" << 'PYTHON_EOF'
import argparse, json, socket, threading, time
import pygame

CTRL_PORT  = 7700
STATE_PORT = 7701

BG         = (10,  26,  53)
PANEL      = (18,  36,  72)
DARK       = (6,   14,  32)
CYAN       = (64,  220, 255)
CYAN_DIM   = (32,  88,  120)
GREEN      = (0,   245, 188)
GREEN_DIM  = (0,   90,  72)
RED        = (255, 64,  96)
RED_DIM    = (85,  0,   32)
PURPLE     = (122, 171, 255)
PURPLE_DIM = (30,  56,  136)
WHITE      = (232, 248, 255)
GRAY       = (72,  136, 208)
YELLOW     = (255, 255, 64)
BLUE_BTN   = (64,  128, 255)
RED_BTN    = (255, 64,  64)
GREEN_BTN  = (64,  255, 64)

SD_AXIS_LX  = 0
SD_AXIS_LY  = 1
SD_AXIS_L2  = 2
SD_AXIS_RY  = 4
SD_DEADZONE = 0.12

SD_BTN_A  = 0
SD_BTN_B  = 1
SD_BTN_X  = 2
SD_BTN_Y  = 3
SD_BTN_L1 = 4
SD_BTN_R1 = 5
SD_BTN_L4 = 11
SD_BTN_L5 = 13
SD_BTN_R4 = 12
SD_BTN_R5 = 14

WIN_W, WIN_H = 1280, 720
SIDE_W       = 265
HDR_H        = 36
BAR_H        = 48


class TeleopSender:

    def __init__(self, host, ctrl_port, state_port):
        self.host       = host
        self.ctrl_port  = ctrl_port
        self.state_port = state_port

        self.ctrl = dict(lx=0.0, ly=0.0, ry=0.0,
                         l2=False, l1=False, r1=False,
                         l4=False, l5=False, r4=False, r5=False,
                         btn_a=False, btn_b=False, btn_x=False, btn_y=False,
                         dpad=[0, 0])

        self.state = dict(wheel_speed=[0]*4, leg_angles=[0]*4,
                          wheel_currents=[0]*4, leg_currents=[0]*4,
                          speed_pct=20)
        self._state_lock = threading.Lock()
        self._last_recv  = 0.0
        self.speed_pct   = 20

        self._ctrl_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._state_sock.bind(('0.0.0.0', state_port))
        self._state_sock.settimeout(0.3)
        threading.Thread(target=self._recv_loop, daemon=True).start()

        pygame.init()
        pygame.joystick.init()
        self.joy = None
        self._connect_joy()

        self.screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
        pygame.display.set_caption('WHEEL TELEOP  ◈  ROS2')
        self.clock = pygame.time.Clock()

        self.font_sm  = pygame.font.SysFont('Courier New', 10, bold=True)
        self.font_med = pygame.font.SysFont('Courier New', 13, bold=True)
        self.font_lg  = pygame.font.SysFont('Courier New', 17, bold=True)

        self.hb_on    = False
        self.hb_timer = 0.0
        self.running  = True
        self._speed_dragging = False
        self._send_timer     = 0.0
        self._estop_rect = pygame.Rect(0, 0, 0, 0)
        self._reset_rect = pygame.Rect(0, 0, 0, 0)
        self._spd_track  = pygame.Rect(0, 0, 0, 0)
        self._reset_flash = 0.0

    def _recv_loop(self):
        while True:
            try:
                data, _ = self._state_sock.recvfrom(4096)
                msg = json.loads(data.decode())
                if msg.get('type') == 'state':
                    with self._state_lock:
                        self.state.update(msg)
                    self._last_recv = time.monotonic()
            except socket.timeout:
                pass
            except Exception:
                pass

    def _send_ctrl(self):
        msg = {'type': 'ctrl', 'speed_pct': self.speed_pct}
        msg.update(self.ctrl)
        try:
            self._ctrl_sock.sendto(json.dumps(msg).encode(), (self.host, self.ctrl_port))
        except Exception:
            pass

    def _send_special(self, t):
        try:
            self._ctrl_sock.sendto(json.dumps({'type': t}).encode(), (self.host, self.ctrl_port))
        except Exception:
            pass

    def do_estop(self):
        self._send_special('estop')
        self.ctrl.update(lx=0.0, ly=0.0, ry=0.0,
                         l2=False, l1=False, r1=False,
                         l4=False, l5=False, r4=False, r5=False,
                         btn_a=False, btn_b=False, btn_x=False, btn_y=False,
                         dpad=[0, 0])

    def do_motor_reset(self):
        self._send_special('motor_reset')
        self._reset_flash = 0.8

    def _connect_joy(self):
        if pygame.joystick.get_count() > 0:
            try:
                self.joy = pygame.joystick.Joystick(0)
                self.joy.init()
            except Exception:
                self.joy = None

    def _poll_gamepad(self):
        if self.joy is None:
            return
        try:
            n_axes = self.joy.get_numaxes()
            n_btns = self.joy.get_numbuttons()

            def axis(i): return self.joy.get_axis(i) if n_axes > i else 0.0
            def btn(i):  return bool(self.joy.get_button(i)) if n_btns > i else False

            l2_raw = axis(SD_AXIS_L2)
            self.ctrl['l2'] = l2_raw > 0.0
            if self.ctrl['l2']:
                self.ctrl.update(lx=0.0, ly=0.0, ry=0.0)
                return

            lx = axis(SD_AXIS_LX)
            ly = axis(SD_AXIS_LY)
            self.ctrl['lx'] = lx if abs(lx) >= SD_DEADZONE else 0.0
            self.ctrl['ly'] = ly if abs(ly) >= SD_DEADZONE else 0.0

            ry = axis(SD_AXIS_RY)
            self.ctrl['ry'] = ry if abs(ry) >= SD_DEADZONE else 0.0
            throttle = max(0.0, -self.ctrl['ry'])
            if throttle >= SD_DEADZONE:
                self.speed_pct = int(throttle * 100)

            self.ctrl['l1']    = btn(SD_BTN_L1)
            self.ctrl['r1']    = btn(SD_BTN_R1)
            self.ctrl['btn_a'] = btn(SD_BTN_A)
            self.ctrl['btn_b'] = btn(SD_BTN_B)
            self.ctrl['btn_x'] = btn(SD_BTN_X)
            self.ctrl['btn_y'] = btn(SD_BTN_Y)
            self.ctrl['l4']    = btn(SD_BTN_L4)
            self.ctrl['l5']    = btn(SD_BTN_L5)
            self.ctrl['r4']    = btn(SD_BTN_R4)
            self.ctrl['r5']    = btn(SD_BTN_R5)

            if self.joy.get_numhats() > 0:
                hx, hy = self.joy.get_hat(0)
                self.ctrl['dpad'] = [hx, hy]
            else:
                up, down, left, right = btn(12), btn(13), btn(14), btn(15)
                self.ctrl['dpad'] = [
                    (1 if right else 0) - (1 if left else 0),
                    (1 if up   else 0) - (1 if down else 0),
                ]
        except Exception:
            pass

    def run(self):
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            self._process_events()
            self._poll_gamepad()

            self.hb_timer += dt
            if self.hb_timer >= 0.6:
                self.hb_on = not self.hb_on
                self.hb_timer = 0.0
            if self._reset_flash > 0:
                self._reset_flash = max(0.0, self._reset_flash - dt)

            self._send_timer += dt
            if self._send_timer >= 0.05:
                self._send_ctrl()
                self._send_timer = 0.0

            self._draw()
            pygame.display.flip()

        self.do_estop()
        time.sleep(0.12)
        pygame.quit()

    def _process_events(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.running = False
            elif ev.type == pygame.JOYDEVICEADDED:
                pygame.joystick.quit(); pygame.joystick.init(); self._connect_joy()
            elif ev.type == pygame.JOYDEVICEREMOVED:
                self.joy = None; pygame.joystick.quit(); pygame.joystick.init()
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE: self.running = False
                elif ev.key == pygame.K_e:    self.do_estop()
                elif ev.key == pygame.K_r:    self.do_motor_reset()
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                p = ev.pos
                if self._estop_rect.collidepoint(p):  self.do_estop()
                elif self._reset_rect.collidepoint(p): self.do_motor_reset()
                elif self._spd_track.collidepoint(p):
                    self._speed_dragging = True; self._set_speed_from_x(p[0])
            elif ev.type == pygame.MOUSEMOTION:
                if self._speed_dragging: self._set_speed_from_x(ev.pos[0])
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                self._speed_dragging = False

    def _set_speed_from_x(self, mx):
        r = self._spd_track
        self.speed_pct = max(0, min(100, int((mx - r.left) / max(r.width, 1) * 100)))

    def _draw(self):
        W, H = self.screen.get_size()
        self.screen.fill(BG)
        y = self._draw_header(0, W)
        y = self._draw_control_bar(y, W)
        self._draw_main(y, W, H - y)

    def _draw_header(self, y, W):
        linked = (time.monotonic() - self._last_recv) < 0.5
        pygame.draw.rect(self.screen, BG, (0, y, W, HDR_H))
        pygame.draw.line(self.screen, CYAN_DIM, (0, y+HDR_H-1), (W, y+HDR_H-1))
        mid = y + HDR_H // 2
        pygame.draw.circle(self.screen, CYAN if self.hb_on else CYAN_DIM, (14, mid), 5)
        self._blit_cy(self.font_lg.render('WHEEL TELEOP  ◈  ROS2', True, CYAN), 28, mid)
        gp = ('GAMEPAD: ' + self.joy.get_name()[:22]) if self.joy else 'GAMEPAD: —'
        self._blit_cy(self.font_sm.render(gp, True, GREEN if self.joy else GRAY), W-400, mid)
        lnk = self.font_sm.render('● LINKED' if linked else '● NO LINK', True, GREEN if linked else RED)
        self.screen.blit(lnk, (W - lnk.get_width() - 10, mid - lnk.get_height()//2))
        return y + HDR_H

    def _draw_control_bar(self, y, W):
        pygame.draw.rect(self.screen, PANEL, (0, y, W, BAR_H))
        pygame.draw.line(self.screen, CYAN_DIM, (0, y+BAR_H-1), (W, y+BAR_H-1))
        mid = y + BAR_H // 2
        cx = 12

        lbl = self.font_sm.render('SPEED %', True, CYAN)
        self._blit_cy(lbl, cx, mid); cx += lbl.get_width() + 10

        TW, TH = 210, 8
        pygame.draw.rect(self.screen, DARK, (cx, mid-TH//2, TW, TH), border_radius=4)
        fill = int(TW * self.speed_pct / 100)
        if fill > 0:
            pygame.draw.rect(self.screen, CYAN, (cx, mid-TH//2, fill, TH), border_radius=4)
        pygame.draw.circle(self.screen, CYAN, (cx+fill, mid), 9)
        self._spd_track = pygame.Rect(cx-9, y, TW+18, BAR_H)
        cx += TW + 14

        pct = self.font_med.render(f'{self.speed_pct}%', True, WHITE)
        self._blit_cy(pct, cx, mid); cx += pct.get_width() + 20
        self._blit_cy(self.font_sm.render(f'HOST: {self.host}:{self.ctrl_port}', True, GRAY), cx, mid)

        BW, BH = 140, 30
        mr_x = W - BW*2 - 30; mr_y = mid - BH//2
        flash = self._reset_flash > 0
        mr_r = pygame.Rect(mr_x, mr_y, BW, BH)
        pygame.draw.rect(self.screen, CYAN_DIM if flash else DARK, mr_r, border_radius=4)
        pygame.draw.rect(self.screen, CYAN if flash else CYAN_DIM, mr_r, 1, border_radius=4)
        ms = self.font_sm.render('⟳ MOTOR RESET  [R]', True, CYAN if flash else GRAY)
        self.screen.blit(ms, (mr_x + BW//2 - ms.get_width()//2, mr_y + BH//2 - ms.get_height()//2))
        self._reset_rect = mr_r

        es_x = W - BW - 10
        es_r = pygame.Rect(es_x, mr_y, BW, BH)
        pygame.draw.rect(self.screen, (42,0,16), es_r, border_radius=4)
        pygame.draw.rect(self.screen, RED_DIM, es_r, 2, border_radius=4)
        es = self.font_sm.render('⚠ E-STOP  [E]', True, RED)
        self.screen.blit(es, (es_x + BW//2 - es.get_width()//2, mr_y + BH//2 - es.get_height()//2))
        self._estop_rect = es_r
        return y + BAR_H

    def _draw_main(self, y, W, H):
        pygame.draw.line(self.screen, CYAN_DIM, (SIDE_W, y), (SIDE_W, y+H))
        pygame.draw.line(self.screen, CYAN_DIM, (W-SIDE_W, y), (W-SIDE_W, y+H))
        self._draw_left_panel(pygame.Rect(0, y, SIDE_W, H))
        self._draw_center_panel(pygame.Rect(SIDE_W, y, W-2*SIDE_W, H))
        self._draw_right_panel(pygame.Rect(W-SIDE_W, y, SIDE_W, H))

    def _draw_left_panel(self, rect):
        pygame.draw.rect(self.screen, PANEL, rect)
        x, y, w = rect.x, rect.y, rect.width
        cy = y + 6
        cy = self._shdr('LEFT CONTROLS', x, cy, w)
        js_r = 52; js_cx = x+w//2; js_cy = cy+js_r+14
        self._joystick(js_cx, js_cy, js_r, self.ctrl['lx'], self.ctrl['ly'], CYAN, 'LEFT STICK  —  ARCADE DRIVE')
        cy = js_cy + js_r + 20
        cy = self._shdr('D-PAD  —  EXTEND LEGS', x, cy, w); cy += 4
        dp = self.ctrl['dpad']
        for arrow, corner, active in [
            ('▲ UP','FL leg',dp[1]==1), ('◄ LEFT','BL leg',dp[0]==-1),
            ('► RIGHT','FR leg',dp[0]==1), ('▼ DOWN','BR leg',dp[1]==-1)]:
            self._mbtn(f'{arrow}  →  {corner}', x+8, cy, w-16, 22, active, GREEN if active else GRAY); cy += 26
        cy += 4
        self._mbtn('L1  —  EXTEND ALL LEGS',  x+8, cy, w-16, 24, self.ctrl['l1'], GREEN); cy += 28
        self._mbtn('R1  —  RETRACT ALL LEGS', x+8, cy, w-16, 24, self.ctrl['r1'], RED)

    def _draw_right_panel(self, rect):
        pygame.draw.rect(self.screen, PANEL, rect)
        x, y, w = rect.x, rect.y, rect.width
        cy = y + 6
        cy = self._shdr('RIGHT CONTROLS', x, cy, w)
        js_r = 52; js_cx = x+w//2; js_cy = cy+js_r+14
        self._joystick(js_cx, js_cy, js_r, 0, self.ctrl['ry'], GREEN, 'RIGHT STICK  —  THROTTLE', y_only=True)
        cy = js_cy + js_r + 20
        cy = self._shdr('FACE BUTTONS  —  SNAP LEG 0°', x, cy, w); cy += 4
        for lbl, corner, active, col in [
            ('Y','FL',self.ctrl['btn_y'],YELLOW), ('X','BL',self.ctrl['btn_x'],BLUE_BTN),
            ('B','FR',self.ctrl['btn_b'],RED_BTN), ('A','BR',self.ctrl['btn_a'],GREEN_BTN)]:
            self._mbtn(f'{lbl}  →  {corner} leg → 0°', x+8, cy, w-16, 22, active, col); cy += 26
        cy += 4
        cy = self._shdr('PADDLES  —  SPIN WHEEL', x, cy, w); cy += 4
        for lbl, desc, active in [
            ('L4','FL wheel',self.ctrl['l4']), ('L5','BL wheel',self.ctrl['l5']),
            ('R4','FR wheel',self.ctrl['r4']), ('R5','BR wheel',self.ctrl['r5'])]:
            self._mbtn(f'{lbl}  →  {desc}', x+8, cy, w-16, 22, active, PURPLE); cy += 26

    def _draw_center_panel(self, rect):
        x, y, w, h = rect.x, rect.y, rect.width, rect.height
        with self._state_lock:
            ws = list(self.state['wheel_speed']); la = list(self.state['leg_angles'])
            wc = list(self.state['wheel_currents']); lc = list(self.state['leg_currents'])
        cy = y + 6

        cy = self._shdr('WHEEL SPEEDS', x, cy, w); cy += 4
        cy = self._4cell(x, cy, w, 38, ['FL','FR','BL','BR'], [ws[0],ws[2],ws[1],ws[3]],
                         lambda v: GREEN if v>0 else (RED if v<0 else GRAY), str, CYAN_DIM); cy += 4

        cy = self._shdr('LEG ANGLES', x, cy, w); cy += 4
        cy = self._4cell(x, cy, w, 38, ['FL','FR','BL','BR'], [la[0],la[2],la[1],la[3]],
                         lambda _: PURPLE, lambda v: f'{v}°', PURPLE_DIM); cy += 4

        cy = self._shdr('ROBOT DIAGRAM', x, cy, w)
        dh = max(80, h - (cy-y) - 70)
        self._diagram(pygame.Rect(x+4, cy, w-8, dh), ws); cy += dh + 6

        cy = self._shdr('CURRENT DRAW', x, cy, w); cy += 4
        hw = w//2; q = hw//4
        lbls = ['FL','FR','BL','BR']
        vw = [wc[0],wc[2],wc[1],wc[3]]; vl = [lc[0],lc[2],lc[1],lc[3]]
        wh = self.font_sm.render('WHEELS (mA)', True, CYAN); self.screen.blit(wh, (x+4, cy))
        lh = self.font_sm.render('LEGS (mA)',   True, CYAN); self.screen.blit(lh, (x+hw+4, cy))
        cy += wh.get_height() + 3
        for i in range(4):
            ls = self.font_sm.render(lbls[i], True, GRAY)
            self.screen.blit(ls, (x+i*q+2, cy)); self.screen.blit(ls, (x+hw+i*q+2, cy))
            ws_ = self.font_sm.render(str(vw[i]), True, GREEN if abs(vw[i])>20 else GRAY)
            ls_ = self.font_sm.render(str(vl[i]), True, PURPLE if abs(vl[i])>20 else PURPLE_DIM)
            self.screen.blit(ws_, (x+i*q+2, cy+12)); self.screen.blit(ls_, (x+hw+i*q+2, cy+12))
        cy += 28
        total = sum(abs(v) for v in wc+lc)
        tt = f'TOTAL: {total/1000:.2f} A' if total>=1000 else f'TOTAL: {total} mA'
        tc = RED if total>5000 else (CYAN if total>500 else WHITE)
        ts = self.font_med.render(tt, True, tc)
        self.screen.blit(ts, (x+w//2-ts.get_width()//2, cy))

    def _diagram(self, rect, speeds):
        pygame.draw.rect(self.screen, DARK, rect, border_radius=4)
        pygame.draw.rect(self.screen, CYAN_DIM, rect, 1, border_radius=4)
        rw, rh = rect.width, rect.height
        cx = rect.x + rw//2; cy = rect.y + rh//2
        for gx in range(rect.x, rect.right, 40):
            pygame.draw.line(self.screen, (16,40,64), (gx,rect.top), (gx,rect.bottom))
        for gy in range(rect.top, rect.bottom, 40):
            pygame.draw.line(self.screen, (16,40,64), (rect.left,gy), (rect.right,gy))
        fw = self.font_sm.render('▲  FORWARD', True, CYAN_DIM)
        self.screen.blit(fw, (cx-fw.get_width()//2, rect.top+4))
        bw, bh = min(38,rw//7), min(54,rh//3)
        pygame.draw.rect(self.screen, (10,34,64), (cx-bw,cy-bh,bw*2,bh*2))
        pygame.draw.rect(self.screen, CYAN, (cx-bw,cy-bh,bw*2,bh*2), 2)
        pygame.draw.line(self.screen, CYAN_DIM, (cx,cy-bh),(cx,cy+bh))
        pygame.draw.line(self.screen, CYAN_DIM, (cx-bw,cy),(cx+bw,cy))
        rl = self.font_sm.render('ROBOT', True, CYAN)
        self.screen.blit(rl, (cx-rl.get_width()//2, cy-rl.get_height()//2))
        wxo, wyo = min(72,rw//4), min(62,rh//3)
        for wx, wy, lbl, i in [(cx-wxo,cy-wyo,'FL',0),(cx-wxo,cy+wyo,'BL',1),
                                (cx+wxo,cy-wyo,'FR',2),(cx+wxo,cy+wyo,'BR',3)]:
            s = speeds[i]
            col = GREEN if s>0 else (RED if s<0 else GRAY)
            dim = GREEN_DIM if s>0 else (RED_DIM if s<0 else (21,46,72))
            pygame.draw.circle(self.screen, dim, (wx,wy), 18, 5)
            pygame.draw.circle(self.screen, DARK,(wx,wy), 12)
            pygame.draw.circle(self.screen, col, (wx,wy), 12, 2)
            pygame.draw.circle(self.screen, col, (wx,wy), 3)
            ls = self.font_sm.render(lbl, True, col)
            self.screen.blit(ls, (wx-ls.get_width()//2, wy-26))
            ss = self.font_sm.render(str(s), True, WHITE)
            self.screen.blit(ss, (wx-ss.get_width()//2, wy+20))

    def _joystick(self, cx, cy, r, jx, jy, col, label, y_only=False):
        pygame.draw.circle(self.screen, DARK,     (cx,cy), r)
        pygame.draw.circle(self.screen, CYAN_DIM, (cx,cy), r, 2)
        pygame.draw.line(self.screen, (16,40,64), (cx,cy-r+3),(cx,cy+r-3))
        pygame.draw.line(self.screen, (16,40,64), (cx-r+3,cy),(cx+r-3,cy))
        md = r - 13
        tx = cx+(0 if y_only else int(jx*md)); ty = cy+int(jy*md)
        pygame.draw.circle(self.screen, col,  (tx,ty), 13)
        pygame.draw.circle(self.screen, DARK, (tx,ty), 5)
        ls = self.font_sm.render(label, True, CYAN)
        self.screen.blit(ls, (cx-ls.get_width()//2, cy-r-16))

    def _4cell(self, x, cy, w, rh, labels, values, color_fn, fmt_fn, border_col):
        cw = w//4
        for i, (lbl, val) in enumerate(zip(labels, values)):
            px = x+i*cw; cell = pygame.Rect(px+2, cy, cw-4, rh)
            pygame.draw.rect(self.screen, DARK, cell, border_radius=3)
            pygame.draw.rect(self.screen, border_col, cell, 1, border_radius=3)
            ls = self.font_sm.render(lbl, True, GRAY)
            self.screen.blit(ls, (px+cw//2-ls.get_width()//2, cy+4))
            vs = self.font_med.render(fmt_fn(val), True, color_fn(val))
            self.screen.blit(vs, (px+cw//2-vs.get_width()//2, cy+rh-vs.get_height()-4))
        return cy + rh

    def _shdr(self, text, x, y, w):
        s = self.font_sm.render(text, True, CYAN)
        self.screen.blit(s, (x+w//2-s.get_width()//2, y))
        ly = y+s.get_height()+3
        pygame.draw.line(self.screen, CYAN_DIM, (x+4,ly),(x+w-4,ly))
        return ly+5

    def _mbtn(self, text, x, y, w, h, active, col):
        bg = tuple(max(0,c//4) for c in col) if active else DARK
        r = pygame.Rect(x,y,w,h)
        pygame.draw.rect(self.screen, bg, r, border_radius=3)
        pygame.draw.rect(self.screen, col if active else CYAN_DIM, r, 1, border_radius=3)
        s = self.font_sm.render(text, True, col if active else GRAY)
        self.screen.blit(s, (x+w//2-s.get_width()//2, y+h//2-s.get_height()//2))

    def _blit_cy(self, surf, x, cy):
        self.screen.blit(surf, (x, cy-surf.get_height()//2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host',       default='127.0.0.1')
    parser.add_argument('--ctrl-port',  type=int, default=CTRL_PORT)
    parser.add_argument('--state-port', type=int, default=STATE_PORT)
    args = parser.parse_args()
    TeleopSender(args.host, args.ctrl_port, args.state_port).run()

main()
PYTHON_EOF
