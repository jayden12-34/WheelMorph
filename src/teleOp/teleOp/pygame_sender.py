#!/usr/bin/env python3
"""Pygame teleop sender — runs standalone (no ROS required).

Usage:
    teleop_sender                        # connect to localhost
    teleop_sender --host 192.168.1.100   # connect to remote receiver
"""

import argparse
import json
import math
import socket
import sys
import threading
import time

import pygame
try:
    import cv2
    import numpy as np
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

CTRL_PORT  = 7700
STATE_PORT = 7701
CAM_PORT   = 7702   # UDP port for JPEG frames streamed from the Jetson

# ── Colours ──────────────────────────────────────────────────────────────────
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
ORANGE     = (255, 165, 0)

# ── Gamepad axis / button mapping (Steam Deck SDL2, no Steam Input) ──────────
SD_AXIS_LX  = 0   # left  stick X
SD_AXIS_LY  = 1   # left  stick Y  (-1=fwd)
SD_AXIS_L2  = 2   # left  trigger  (-1=rest, +1=pressed)
SD_AXIS_RY  = 4   # right stick Y  (-1=up/more torque)
SD_DEADZONE = 0.12

SD_BTN_A  = 0    # south  — snap BR leg to 0°
SD_BTN_B  = 1    # east   — snap FR leg to 0°
SD_BTN_X  = 2    # west   — snap BL leg to 0°
SD_BTN_Y  = 3    # north  — snap FL leg to 0°
SD_BTN_L1 = 4    # extend all legs
SD_BTN_R1 = 5    # retract all legs
SD_BTN_L4 = 11   # upper-left  paddle → BL wheel
SD_BTN_L5 = 13   # lower-left  paddle → FL wheel
SD_BTN_R4 = 12   # upper-right paddle → FR wheel
SD_BTN_R5 = 14   # lower-right paddle → BR wheel

SMOOTH_ALPHA  = 0.25   # axis low-pass per 60fps tick (~167 ms to 94% of target)
THROTTLE_RATE = 60.0   # % per second at full joystick deflection

# ── Base layout dimensions at 1280×720 ───────────────────────────────────────
WIN_W, WIN_H = 1280, 720
SIDE_W       = 265
HDR_H        = 36
BAR_H        = 48


class TeleopSender:

    def __init__(self, host: str, ctrl_port: int, state_port: int):
        self.host       = host
        self.ctrl_port  = ctrl_port
        self.state_port = state_port

        self.ctrl = dict(
            lx=0.0, ly=0.0, ry=0.0,
            l2=False, l1=False, r1=False,
            l4=False, l5=False, r4=False, r5=False,
            btn_a=False, btn_b=False, btn_x=False, btn_y=False,
            dpad=[0, 0],
        )

        self.state = dict(
            wheel_torque=[0]*4, leg_angles=[0]*4,
            wheel_currents=[0]*4, leg_currents=[0]*4,
            wheel_temps=[0]*4,
            speed_pct=20,
        )
        self._state_lock = threading.Lock()
        self._last_recv  = 0.0

        self.speed_pct  = 20
        self.drive_mode = 0      # 0 = torque/current, 1 = velocity

        # UDP sockets
        self._ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._state_sock.bind(('0.0.0.0', state_port))
        self._state_sock.settimeout(0.3)

        threading.Thread(target=self._recv_loop, daemon=True).start()

        # Pygame init
        pygame.init()
        pygame.joystick.init()
        self.joy = None
        self._connect_joy()

        self.screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
        pygame.display.set_caption('WHEEL TELEOP  ◈  ROS2')
        self.clock = pygame.time.Clock()

        # Scale state — updated whenever the window is resized
        self._scale_x  = 1.0
        self._scale_y  = 1.0
        self._scale    = 1.0
        self._last_size = (WIN_W, WIN_H)
        self._rebuild_fonts()

        self.hb_on    = False
        self.hb_timer = 0.0
        self.running  = True

        self._speed_dragging = False
        self._send_timer     = 0.0
        self._raw_btns_pressed: set[int] = set()
        self._kb_paddles = {'l4': False, 'l5': False, 'r4': False, 'r5': False}

        # Clickable rects (populated each frame)
        self._estop_rect = pygame.Rect(0, 0, 0, 0)
        self._reset_rect = pygame.Rect(0, 0, 0, 0)
        self._spd_track  = pygame.Rect(0, 0, 0, 0)
        self._mode_rect  = pygame.Rect(0, 0, 0, 0)

        # Motor reset flash state
        self._reset_flash = 0.0

        self._smooth_lx = 0.0
        self._smooth_ly = 0.0

        # Camera feed
        self._cam_view  = False
        self._cam_frame = None   # latest pygame Surface from capture thread
        self._cam_lock  = threading.Lock()
        if _CV2_AVAILABLE:
            threading.Thread(target=self._cam_loop, daemon=True).start()

    # ── Scale helpers ─────────────────────────────────────────────────────────

    def _sw(self, v: float) -> int:
        return int(v * self._scale_x)

    def _sh(self, v: float) -> int:
        return int(v * self._scale_y)

    def _ss(self, v: float) -> int:
        return max(1, int(v * self._scale))

    def _rebuild_fonts(self):
        s = self._scale
        self.font_sm  = pygame.font.SysFont('Courier New', max(8,  int(10 * s)), bold=True)
        self.font_med = pygame.font.SysFont('Courier New', max(10, int(13 * s)), bold=True)
        self.font_lg  = pygame.font.SysFont('Courier New', max(12, int(17 * s)), bold=True)

    # ── Networking ───────────────────────────────────────────────────────────

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
        msg = {'type': 'ctrl', 'speed_pct': self.speed_pct,
               'drive_mode': self.drive_mode}
        msg.update(self.ctrl)
        try:
            self._ctrl_sock.sendto(json.dumps(msg).encode(),
                                   (self.host, self.ctrl_port))
        except Exception:
            pass

    def _send_special(self, msg_type: str):
        try:
            self._ctrl_sock.sendto(json.dumps({'type': msg_type}).encode(),
                                   (self.host, self.ctrl_port))
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

    # ── Gamepad ──────────────────────────────────────────────────────────────

    def _connect_joy(self):
        if pygame.joystick.get_count() > 0:
            try:
                self.joy = pygame.joystick.Joystick(0)
                self.joy.init()
            except Exception:
                self.joy = None

    def _poll_gamepad(self, dt: float = 1 / 60):
        for k in ('l4', 'l5', 'r4', 'r5'):
            self.ctrl[k] = self._kb_paddles[k]
        if self.joy is None:
            return

        try:
            n_axes = self.joy.get_numaxes()
            n_btns = self.joy.get_numbuttons()

            def axis(i):
                return self.joy.get_axis(i) if n_axes > i else 0.0

            def btn(i):
                return bool(self.joy.get_button(i)) if n_btns > i else False

            l2_raw = axis(SD_AXIS_L2)
            self.ctrl['l2'] = l2_raw > 0.0
            if self.ctrl['l2']:
                self._smooth_lx = 0.0
                self._smooth_ly = 0.0
                self.ctrl.update(lx=0.0, ly=0.0, ry=0.0)
                return

            lx_raw = axis(SD_AXIS_LX)
            ly_raw = axis(SD_AXIS_LY)
            lx_raw = lx_raw if abs(lx_raw) >= SD_DEADZONE else 0.0
            ly_raw = ly_raw if abs(ly_raw) >= SD_DEADZONE else 0.0
            self._smooth_lx += SMOOTH_ALPHA * (lx_raw - self._smooth_lx)
            self._smooth_ly += SMOOTH_ALPHA * (ly_raw - self._smooth_ly)
            self.ctrl['lx'] = self._smooth_lx
            self.ctrl['ly'] = self._smooth_ly

            ry = axis(SD_AXIS_RY)
            ry_val = ry if abs(ry) >= SD_DEADZONE else 0.0
            self.ctrl['ry'] = ry_val
            if abs(ry_val) > 0:
                delta = -ry_val * THROTTLE_RATE * dt
                self.speed_pct = max(0, min(100, int(self.speed_pct + delta)))

            self.ctrl['l1'] = btn(SD_BTN_L1)
            self.ctrl['r1'] = btn(SD_BTN_R1)

            self.ctrl['btn_a'] = btn(SD_BTN_A)
            self.ctrl['btn_b'] = btn(SD_BTN_B)
            self.ctrl['btn_x'] = btn(SD_BTN_X)
            self.ctrl['btn_y'] = btn(SD_BTN_Y)

            self.ctrl['l4'] = btn(SD_BTN_L4) or self._kb_paddles['l4']
            self.ctrl['l5'] = btn(SD_BTN_L5) or self._kb_paddles['l5']
            self.ctrl['r4'] = btn(SD_BTN_R4) or self._kb_paddles['r4']
            self.ctrl['r5'] = btn(SD_BTN_R5) or self._kb_paddles['r5']

            if self.joy.get_numhats() > 0:
                hx, hy = self.joy.get_hat(0)
                self.ctrl['dpad'] = [hx, hy]
            else:
                up    = btn(12)
                down  = btn(13)
                left  = btn(14)
                right = btn(15)
                self.ctrl['dpad'] = [
                    (1 if right else 0) - (1 if left else 0),
                    (1 if up   else 0) - (1 if down else 0),
                ]

            self._raw_btns_pressed = {
                i for i in range(n_btns) if self.joy.get_button(i)
            }

        except Exception:
            pass

    # ── Main loop ────────────────────────────────────────────────────────────

    def run(self):
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            self._process_events()
            self._poll_gamepad(dt)

            self._hb_timer_tick(dt)
            if self._reset_flash > 0:
                self._reset_flash = max(0.0, self._reset_flash - dt)

            self._send_timer += dt
            if self._send_timer >= 0.05:
                self._send_ctrl()
                self._send_timer = 0.0

            self._draw()
            pygame.display.flip()

        self._shutdown()

    def _process_events(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.running = False

            elif ev.type == pygame.JOYDEVICEADDED:
                pygame.joystick.quit()
                pygame.joystick.init()
                self._connect_joy()

            elif ev.type == pygame.JOYDEVICEREMOVED:
                self.joy = None
                pygame.joystick.quit()
                pygame.joystick.init()

            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    self.running = False
                elif ev.key == pygame.K_e:
                    self.do_estop()
                elif ev.key == pygame.K_r:
                    self.do_motor_reset()
                elif ev.key == pygame.K_t:
                    self.drive_mode = 1 - self.drive_mode
                elif ev.key == pygame.K_c:
                    self._cam_view = not self._cam_view
                elif ev.key == pygame.K_v:
                    self._kb_paddles['l4'] = True
                elif ev.key == pygame.K_b:
                    self._kb_paddles['l5'] = True
                elif ev.key == pygame.K_n:
                    self._kb_paddles['r4'] = True
                elif ev.key == pygame.K_m:
                    self._kb_paddles['r5'] = True

            elif ev.type == pygame.KEYUP:
                if ev.key == pygame.K_v:
                    self._kb_paddles['l4'] = False
                elif ev.key == pygame.K_b:
                    self._kb_paddles['l5'] = False
                elif ev.key == pygame.K_n:
                    self._kb_paddles['r4'] = False
                elif ev.key == pygame.K_m:
                    self._kb_paddles['r5'] = False

            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                p = ev.pos
                if self._estop_rect.collidepoint(p):
                    self.do_estop()
                elif self._reset_rect.collidepoint(p):
                    self.do_motor_reset()
                elif self._mode_rect.collidepoint(p):
                    self.drive_mode = 1 - self.drive_mode
                elif self._spd_track.collidepoint(p):
                    self._speed_dragging = True
                    self._set_speed_from_x(p[0])

            elif ev.type == pygame.MOUSEMOTION:
                if self._speed_dragging:
                    self._set_speed_from_x(ev.pos[0])

            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                self._speed_dragging = False

    def _set_speed_from_x(self, mx: int):
        r = self._spd_track
        pct = (mx - r.left) / max(r.width, 1)
        self.speed_pct = max(0, min(100, int(pct * 100)))

    def _hb_timer_tick(self, dt: float):
        self.hb_timer += dt
        if self.hb_timer >= 0.6:
            self.hb_on = not self.hb_on
            self.hb_timer = 0.0

    def _shutdown(self):
        self.running = False
        self.do_estop()
        time.sleep(0.12)
        try:
            self._ctrl_sock.close()
        except Exception:
            pass
        try:
            self._state_sock.close()
        except Exception:
            pass
        pygame.quit()

    # ── Camera capture ────────────────────────────────────────────────────────

    def _cam_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', CAM_PORT))
        sock.settimeout(1.0)

        while self.running:
            try:
                data, _ = sock.recvfrom(65536)
            except socket.timeout:
                with self._cam_lock:
                    self._cam_frame = None
                continue
            except Exception:
                continue

            try:
                buf   = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                surf = pygame.surfarray.make_surface(frame_rgb.swapaxes(0, 1))
                with self._cam_lock:
                    self._cam_frame = surf
            except Exception:
                pass

        sock.close()

    # ── Drawing ──────────────────────────────────────────────────────────────

    def _draw(self):
        W, H = self.screen.get_size()
        if (W, H) != self._last_size:
            self._scale_x  = W / WIN_W
            self._scale_y  = H / WIN_H
            self._scale    = min(self._scale_x, self._scale_y)
            self._rebuild_fonts()
            self._last_size = (W, H)

        self.screen.fill(BG)
        y = self._draw_header(0, W)
        y = self._draw_control_bar(y, W)
        self._draw_main(y, W, H - y)

    def _draw_header(self, y: int, W: int) -> int:
        hdr_h = self._sh(HDR_H)
        linked = (time.monotonic() - self._last_recv) < 0.5
        pygame.draw.rect(self.screen, BG, (0, y, W, hdr_h))
        pygame.draw.line(self.screen, CYAN_DIM, (0, y + hdr_h - 1), (W, y + hdr_h - 1))

        mid_y = y + hdr_h // 2

        hb_col = CYAN if self.hb_on else CYAN_DIM
        pygame.draw.circle(self.screen, hb_col, (self._sw(14), mid_y), self._ss(5))

        self._blit_center_y(self.font_lg.render('WHEEL TELEOP  ◈  ROS2', True, CYAN),
                            self._sw(28), mid_y)

        gp_txt = ('GAMEPAD: ' + self.joy.get_name()[:22]) if self.joy else 'GAMEPAD: —'
        self._blit_center_y(self.font_sm.render(gp_txt, True, GREEN if self.joy else GRAY),
                            W - self._sw(400), mid_y)

        lnk_txt = '● LINKED' if linked else '● NO LINK'
        lnk_col = GREEN if linked else RED
        lnk_s = self.font_sm.render(lnk_txt, True, lnk_col)
        self.screen.blit(lnk_s, (W - lnk_s.get_width() - self._sw(10),
                                  mid_y - lnk_s.get_height() // 2))

        return y + hdr_h

    def _draw_control_bar(self, y: int, W: int) -> int:
        bar_h = self._sh(BAR_H)
        pygame.draw.rect(self.screen, PANEL, (0, y, W, bar_h))
        pygame.draw.line(self.screen, CYAN_DIM, (0, y + bar_h - 1), (W, y + bar_h - 1))

        mid_y = y + bar_h // 2
        cx    = self._sw(12)

        bar_lbl = 'SPEED %' if self.drive_mode == 1 else 'TORQUE %'
        lbl = self.font_sm.render(bar_lbl, True, CYAN)
        self._blit_center_y(lbl, cx, mid_y)
        cx += lbl.get_width() + self._sw(10)

        TW    = self._sw(210)
        TH    = self._sh(8)
        knob_r = self._ss(9)
        tr = pygame.Rect(cx, mid_y - TH // 2, TW, TH)
        pygame.draw.rect(self.screen, DARK, tr, border_radius=self._ss(4))
        fill = int(TW * self.speed_pct / 100)
        if fill > 0:
            pygame.draw.rect(self.screen, CYAN,
                             pygame.Rect(cx, mid_y - TH // 2, fill, TH),
                             border_radius=self._ss(4))
        pygame.draw.circle(self.screen, CYAN, (cx + fill, mid_y), knob_r)
        self._spd_track = pygame.Rect(cx - knob_r, y, TW + knob_r * 2, bar_h)
        cx += TW + self._sw(14)

        pct_s = self.font_med.render(f'{self.speed_pct}%', True, WHITE)
        self._blit_center_y(pct_s, cx, mid_y)
        cx += pct_s.get_width() + self._sw(20)

        host_s = self.font_sm.render(f'HOST: {self.host}:{self.ctrl_port}', True, GRAY)
        self._blit_center_y(host_s, cx, mid_y)

        BH  = self._sh(30)
        GAP = self._sw(8)
        btn_y = mid_y - BH // 2

        BW_ES = self._sw(140)
        es_x  = W - BW_ES - self._sw(8)
        es_rect = pygame.Rect(es_x, btn_y, BW_ES, BH)
        pygame.draw.rect(self.screen, (42, 0, 16), es_rect, border_radius=self._ss(4))
        pygame.draw.rect(self.screen, RED_DIM, es_rect, 2, border_radius=self._ss(4))
        es_s = self.font_sm.render('⚠ E-STOP  [E]', True, RED)
        self.screen.blit(es_s, (es_x + BW_ES // 2 - es_s.get_width() // 2,
                                 btn_y + BH // 2 - es_s.get_height() // 2))
        self._estop_rect = es_rect

        BW_MR = self._sw(140)
        mr_x  = es_x - BW_MR - GAP
        mr_rect = pygame.Rect(mr_x, btn_y, BW_MR, BH)
        flash  = self._reset_flash > 0
        mr_bg  = CYAN_DIM if flash else DARK
        mr_fg  = CYAN     if flash else CYAN_DIM
        pygame.draw.rect(self.screen, mr_bg, mr_rect, border_radius=self._ss(4))
        pygame.draw.rect(self.screen, mr_fg, mr_rect, 1, border_radius=self._ss(4))
        mr_s = self.font_sm.render('⟳ MOTOR RESET  [R]', True, CYAN if flash else GRAY)
        self.screen.blit(mr_s, (mr_x + BW_MR // 2 - mr_s.get_width() // 2,
                                 btn_y + BH // 2 - mr_s.get_height() // 2))
        self._reset_rect = mr_rect

        BW_MD = self._sw(110)
        md_x  = mr_x - BW_MD - GAP
        md_rect = pygame.Rect(md_x, btn_y, BW_MD, BH)
        if self.drive_mode == 0:
            md_bg, md_fg, md_col = DARK, PURPLE_DIM, PURPLE
            md_txt = '⚡ TORQUE [T]'
        else:
            md_bg, md_fg, md_col = (0, 30, 20), GREEN_DIM, GREEN
            md_txt = '◎ VELOCITY [T]'
        pygame.draw.rect(self.screen, md_bg, md_rect, border_radius=self._ss(4))
        pygame.draw.rect(self.screen, md_fg, md_rect, 1, border_radius=self._ss(4))
        md_s = self.font_sm.render(md_txt, True, md_col)
        self.screen.blit(md_s, (md_x + BW_MD // 2 - md_s.get_width() // 2,
                                 btn_y + BH // 2 - md_s.get_height() // 2))
        self._mode_rect = md_rect

        return y + bar_h

    def _draw_main(self, y: int, W: int, H: int):
        side_w = self._sw(SIDE_W)
        cx_x   = side_w
        cx_w   = W - 2 * side_w

        pygame.draw.line(self.screen, CYAN_DIM, (side_w,     y), (side_w,     y + H))
        pygame.draw.line(self.screen, CYAN_DIM, (W - side_w, y), (W - side_w, y + H))

        self._draw_left_panel(pygame.Rect(0,           y, side_w, H))
        self._draw_center_panel(pygame.Rect(cx_x,      y, cx_w,   H))
        self._draw_right_panel(pygame.Rect(W - side_w, y, side_w, H))

    def _draw_left_panel(self, rect: pygame.Rect):
        pygame.draw.rect(self.screen, PANEL, rect)
        x, y, w = rect.x, rect.y, rect.width
        cy = y + self._sh(6)

        cy = self._section_hdr('LEFT CONTROLS', x, cy, w)

        js_r  = self._ss(52)
        js_cx = x + w // 2
        js_cy = cy + js_r + self._sh(14)
        self._draw_joystick(js_cx, js_cy, js_r,
                            self.ctrl['lx'], self.ctrl['ly'],
                            CYAN, 'LEFT STICK  —  ARCADE DRIVE')
        cy = js_cy + js_r + self._sh(20)

        cy = self._section_hdr('D-PAD  —  RETRACT LEGS', x, cy, w)
        cy += self._sh(4)
        dp = self.ctrl['dpad']
        dpad_rows = [
            ('▲ UP',    'FL leg ▼', dp[1] == 1),
            ('◄ LEFT',  'BL leg ▼', dp[0] == -1),
            ('► RIGHT', 'FR leg ▼', dp[0] == 1),
            ('▼ DOWN',  'BR leg ▼', dp[1] == -1),
        ]
        bh22 = self._sh(22)
        bh26 = self._sh(26)
        pad  = self._sw(8)
        pad2 = self._sw(16)
        for arrow, corner, active in dpad_rows:
            col = RED if active else GRAY
            self._mini_btn(f'{arrow}  →  {corner}', x + pad, cy, w - pad2, bh22, active, col)
            cy += bh26

        cy += self._sh(4)
        self._mini_btn('L1  —  RETRACT ALL LEGS', x + pad, cy, w - pad2, self._sh(24),
                       self.ctrl['l1'], RED)
        cy += self._sh(28)
        self._mini_btn('R1  —  EXTEND ALL LEGS',  x + pad, cy, w - pad2, self._sh(24),
                       self.ctrl['r1'], GREEN)

    def _draw_right_panel(self, rect: pygame.Rect):
        pygame.draw.rect(self.screen, PANEL, rect)
        x, y, w = rect.x, rect.y, rect.width
        cy = y + self._sh(6)

        cy = self._section_hdr('RIGHT CONTROLS', x, cy, w)

        js_r  = self._ss(52)
        js_cx = x + w // 2
        js_cy = cy + js_r + self._sh(14)
        rs_lbl = 'RIGHT STICK  —  SPEED LIMIT' if self.drive_mode == 1 else 'RIGHT STICK  —  TORQUE LIMIT'
        self._draw_joystick(js_cx, js_cy, js_r,
                            0, self.ctrl['ry'],
                            GREEN, rs_lbl,
                            y_only=True)
        cy = js_cy + js_r + self._sh(20)

        cy = self._section_hdr('FACE BUTTONS  —  EXTEND LEGS', x, cy, w)
        cy += self._sh(4)
        face_rows = [
            ('Y', 'FL', self.ctrl['btn_y'], YELLOW),
            ('X', 'BL', self.ctrl['btn_x'], BLUE_BTN),
            ('B', 'FR', self.ctrl['btn_b'], RED_BTN),
            ('A', 'BR', self.ctrl['btn_a'], GREEN_BTN),
        ]
        bh22 = self._sh(22)
        bh26 = self._sh(26)
        pad  = self._sw(8)
        pad2 = self._sw(16)
        for lbl, corner, active, col in face_rows:
            self._mini_btn(f'{lbl}  →  {corner} leg ▲', x + pad, cy, w - pad2, bh22, active, col)
            cy += bh26

        cy += self._sh(4)
        cy = self._section_hdr('PADDLES  —  SPIN WHEEL', x, cy, w)
        cy += self._sh(4)
        paddle_rows = [
            ('L4 [V]', 'BL wheel', self.ctrl['l4']),
            ('L5 [B]', 'FL wheel', self.ctrl['l5']),
            ('R4 [N]', 'FR wheel', self.ctrl['r4']),
            ('R5 [M]', 'BR wheel', self.ctrl['r5']),
        ]
        for lbl, desc, active in paddle_rows:
            self._mini_btn(f'{lbl}  →  {desc}', x + pad, cy, w - pad2, bh22, active, PURPLE)
            cy += bh26

        cy += self._sh(4)
        cy = self._section_hdr('RAW BTNS PRESSED', x, cy, w)
        cy += self._sh(2)
        if self.joy:
            n_total = self.joy.get_numbuttons()
            pressed = sorted(self._raw_btns_pressed)
            dbg_txt = f'n={n_total}  pressed={pressed if pressed else "none"}'
        else:
            dbg_txt = 'no gamepad'
        dbg_s = self.font_sm.render(dbg_txt, True, YELLOW)
        self.screen.blit(dbg_s, (x + w // 2 - dbg_s.get_width() // 2, cy))

    def _draw_center_panel(self, rect: pygame.Rect):
        x, y, w, h = rect.x, rect.y, rect.width, rect.height

        with self._state_lock:
            wt   = list(self.state['wheel_torque'])
            la   = list(self.state['leg_angles'])
            wc   = list(self.state['wheel_currents'])
            lc   = list(self.state['leg_currents'])
            wtmp = list(self.state['wheel_temps'])

        cy    = y + self._sh(6)
        row_h = self._sh(38)
        gap4  = self._sh(4)

        mode_hdr = 'WHEEL VEL CMD' if self.drive_mode == 1 else 'WHEEL TORQUE CMD'
        cy = self._section_hdr(mode_hdr, x, cy, w)
        cy += gap4
        cy = self._draw_4cell_row(x, cy, w, row_h,
                                  ['FL', 'FR', 'BL', 'BR'], [wt[0], wt[2], wt[1], wt[3]],
                                  lambda v: GREEN if v > 0 else (RED if v < 0 else GRAY),
                                  lambda v: str(v), CYAN_DIM)
        cy += gap4

        cy = self._section_hdr('LEG ANGLES', x, cy, w)
        cy += gap4
        cy = self._draw_4cell_row(x, cy, w, row_h,
                                  ['FL', 'FR', 'BL', 'BR'], [la[0], la[2], la[1], la[3]],
                                  lambda _: PURPLE,
                                  lambda v: f'{v}°', PURPLE_DIM)
        cy += gap4

        cam_hdr = 'CAMERA FEED [C]' if self._cam_view else 'ROBOT DIAGRAM [C]'
        cy = self._section_hdr(cam_hdr, x, cy, w)
        remaining = h - (cy - y) - h // 4
        diag_h    = max(self._sh(80), remaining)
        diag_rect = pygame.Rect(x + self._sw(4), cy, w - self._sw(8), diag_h)
        if self._cam_view:
            self._draw_camera(diag_rect)
        else:
            self._draw_robot_diagram(diag_rect, wt, wc)
        cy += diag_h + self._sh(6)

        cy = self._section_hdr('CURRENT DRAW (mA)', x, cy, w)
        cy += gap4
        hw = w // 2
        q  = hw // 4
        labels_order = ['FL', 'FR', 'BL', 'BR']
        vals_w = [wc[0], wc[2], wc[1], wc[3]]
        vals_l = [lc[0], lc[2], lc[1], lc[3]]

        wc_hdr = self.font_sm.render('WHEELS actual (mA)', True, CYAN)
        self.screen.blit(wc_hdr, (x + self._sw(4), cy))
        lc_hdr = self.font_sm.render('LEGS actual (mA)', True, CYAN)
        self.screen.blit(lc_hdr, (x + hw + self._sw(4), cy))
        cy += wc_hdr.get_height() + self._sh(3)

        val_off = self._sh(12)
        px2     = self._sw(2)
        for i in range(4):
            px_w = x + i * q
            px_l = x + hw + i * q
            col_w = GREEN if abs(vals_w[i]) > 20 else GRAY
            col_l = PURPLE if abs(vals_l[i]) > 20 else PURPLE_DIM

            lbl_s = self.font_sm.render(labels_order[i], True, GRAY)
            self.screen.blit(lbl_s, (px_w + px2, cy))
            self.screen.blit(lbl_s, (px_l + px2, cy))

            v_w = self.font_sm.render(str(vals_w[i]), True, col_w)
            v_l = self.font_sm.render(str(vals_l[i]), True, col_l)
            self.screen.blit(v_w, (px_w + px2, cy + val_off))
            self.screen.blit(v_l, (px_l + px2, cy + val_off))

        cy += self._sh(28)
        total = sum(abs(v) for v in wc + lc)
        tot_txt = f'TOTAL: {total/1000:.2f} A' if total >= 1000 else f'TOTAL: {total} mA'
        tot_col = RED if total > 5000 else (CYAN if total > 500 else WHITE)
        tot_s = self.font_med.render(tot_txt, True, tot_col)
        self.screen.blit(tot_s, (x + w // 2 - tot_s.get_width() // 2, cy))
        cy += tot_s.get_height() + self._sh(6)

        cy = self._section_hdr('WHEEL TEMPS (°C)', x, cy, w)
        cy += gap4
        tmp_labels = ['FL', 'FR', 'BL', 'BR']
        tmp_vals   = [wtmp[0], wtmp[2], wtmp[1], wtmp[3]]
        q_t     = w // 4
        tmp_off = self._sh(12)
        for i, (lbl, t) in enumerate(zip(tmp_labels, tmp_vals)):
            if   t >= 70: tc = RED
            elif t >= 60: tc = ORANGE
            elif t >= 50: tc = YELLOW
            else:         tc = GREEN
            px = x + i * q_t
            ls = self.font_sm.render(lbl, True, GRAY)
            self.screen.blit(ls, (px + q_t // 2 - ls.get_width() // 2, cy))
            vs = self.font_med.render(f'{t}°', True, tc)
            self.screen.blit(vs, (px + q_t // 2 - vs.get_width() // 2, cy + tmp_off))

    def _draw_camera(self, rect: pygame.Rect):
        br = self._ss(4)
        pygame.draw.rect(self.screen, DARK, rect, border_radius=br)
        pygame.draw.rect(self.screen, CYAN_DIM, rect, 1, border_radius=br)

        with self._cam_lock:
            frame = self._cam_frame

        if frame is not None:
            scaled = pygame.transform.scale(frame, (rect.width, rect.height))
            self.screen.blit(scaled, rect.topleft)
            pygame.draw.rect(self.screen, CYAN_DIM, rect, 1, border_radius=br)
        else:
            if not _CV2_AVAILABLE:
                msg, sub = 'cv2 NOT INSTALLED', 'pip install opencv-python numpy'
            else:
                msg, sub = 'NO VIDEO STREAM', f'waiting for Jetson on UDP:{CAM_PORT}'
            ms = self.font_lg.render(msg, True, RED_DIM)
            ss = self.font_sm.render(sub, True, GRAY)
            self.screen.blit(ms, (rect.centerx - ms.get_width() // 2,
                                   rect.centery - ms.get_height()))
            self.screen.blit(ss, (rect.centerx - ss.get_width() // 2,
                                   rect.centery + self._sh(4)))

    def _draw_robot_diagram(self, rect: pygame.Rect, torques: list, currents: list):
        br = self._ss(4)
        pygame.draw.rect(self.screen, DARK, rect, border_radius=br)
        pygame.draw.rect(self.screen, CYAN_DIM, rect, 1, border_radius=br)

        rw, rh = rect.width, rect.height
        cx = rect.x + rw // 2
        cy = rect.y + rh // 2

        grid = max(1, self._ss(40))
        for gx in range(rect.x, rect.right, grid):
            pygame.draw.line(self.screen, (16, 40, 64), (gx, rect.top), (gx, rect.bottom))
        for gy in range(rect.top, rect.bottom, grid):
            pygame.draw.line(self.screen, (16, 40, 64), (rect.left, gy), (rect.right, gy))

        fwd = self.font_sm.render('▲  FORWARD', True, CYAN_DIM)
        self.screen.blit(fwd, (cx - fwd.get_width() // 2, rect.top + self._sh(4)))

        bw = min(self._ss(38), rw // 7)
        bh = min(self._ss(54), rh // 3)
        pygame.draw.rect(self.screen, (10, 34, 64), (cx - bw, cy - bh, bw * 2, bh * 2))
        pygame.draw.rect(self.screen, CYAN, (cx - bw, cy - bh, bw * 2, bh * 2), 2)
        pygame.draw.line(self.screen, CYAN_DIM, (cx, cy - bh), (cx, cy + bh))
        pygame.draw.line(self.screen, CYAN_DIM, (cx - bw, cy), (cx + bw, cy))
        r_lbl = self.font_sm.render('ROBOT', True, CYAN)
        self.screen.blit(r_lbl, (cx - r_lbl.get_width() // 2, cy - r_lbl.get_height() // 2))

        wxo  = min(self._ss(72), rw // 4)
        wyo  = min(self._ss(62), rh // 3)
        wr   = self._ss(18)
        ir   = self._ss(12)
        dr   = self._ss(3)
        rw5  = self._ss(5)
        loff = self._ss(26)
        moff = self._ss(20)
        wheel_positions = [
            (cx - wxo, cy - wyo, 'FL', 0),
            (cx - wxo, cy + wyo, 'BL', 1),
            (cx + wxo, cy - wyo, 'FR', 2),
            (cx + wxo, cy + wyo, 'BR', 3),
        ]
        for wx, wy, lbl, i in wheel_positions:
            t   = torques[i]
            ma  = currents[i]
            col = GREEN if t > 0 else (RED if t < 0 else GRAY)
            dim = GREEN_DIM if t > 0 else (RED_DIM if t < 0 else (21, 46, 72))
            stall    = t != 0 and abs(ma) < 50
            ring_col = ORANGE if stall else col
            pygame.draw.circle(self.screen, dim,      (wx, wy), wr, rw5)
            pygame.draw.circle(self.screen, DARK,     (wx, wy), ir)
            pygame.draw.circle(self.screen, ring_col, (wx, wy), ir, 2)
            pygame.draw.circle(self.screen, ring_col, (wx, wy), dr)
            lbl_s = self.font_sm.render(lbl, True, col)
            self.screen.blit(lbl_s, (wx - lbl_s.get_width() // 2, wy - loff))
            ma_s = self.font_sm.render(f'{ma}mA', True, WHITE)
            self.screen.blit(ma_s, (wx - ma_s.get_width() // 2, wy + moff))

    def _draw_joystick(self, cx: int, cy: int, r: int,
                       jx: float, jy: float, col, label: str,
                       y_only: bool = False):
        pygame.draw.circle(self.screen, DARK, (cx, cy), r)
        pygame.draw.circle(self.screen, CYAN_DIM, (cx, cy), r, 2)
        cr = self._ss(3)
        pygame.draw.line(self.screen, (16, 40, 64), (cx, cy - r + cr), (cx, cy + r - cr))
        pygame.draw.line(self.screen, (16, 40, 64), (cx - r + cr, cy), (cx + r - cr, cy))

        dot_r = self._ss(13)
        max_d = max(1, r - dot_r)
        tx = cx + (0 if y_only else int(jx * max_d))
        ty = cy + int(jy * max_d)
        pygame.draw.circle(self.screen, col,  (tx, ty), dot_r)
        pygame.draw.circle(self.screen, DARK, (tx, ty), self._ss(5))

        lbl_s = self.font_sm.render(label, True, CYAN)
        self.screen.blit(lbl_s, (cx - lbl_s.get_width() // 2, cy - r - self._ss(16)))

    def _draw_4cell_row(self, x, cy, w, row_h, labels, values, color_fn, fmt_fn, border_col):
        cw = w // 4
        br = self._ss(3)
        p2 = self._sw(2)
        p4 = self._sh(4)
        for i, (lbl, val) in enumerate(zip(labels, values)):
            px   = x + i * cw
            cell = pygame.Rect(px + p2, cy, cw - p2 * 2, row_h)
            pygame.draw.rect(self.screen, DARK, cell, border_radius=br)
            pygame.draw.rect(self.screen, border_col, cell, 1, border_radius=br)
            ls = self.font_sm.render(lbl, True, GRAY)
            self.screen.blit(ls, (px + cw // 2 - ls.get_width() // 2, cy + p4))
            vs = self.font_med.render(fmt_fn(val), True, color_fn(val))
            self.screen.blit(vs, (px + cw // 2 - vs.get_width() // 2,
                                   cy + row_h - vs.get_height() - p4))
        return cy + row_h

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _section_hdr(self, text: str, x: int, y: int, w: int) -> int:
        s = self.font_sm.render(text, True, CYAN)
        self.screen.blit(s, (x + w // 2 - s.get_width() // 2, y))
        line_y = y + s.get_height() + self._sh(3)
        pygame.draw.line(self.screen, CYAN_DIM,
                         (x + self._sw(4), line_y), (x + w - self._sw(4), line_y))
        return line_y + self._sh(5)

    def _mini_btn(self, text: str, x: int, y: int, w: int, h: int,
                  active: bool, col):
        bg = tuple(max(0, c // 4) for c in col) if active else DARK
        r  = pygame.Rect(x, y, w, h)
        br = self._ss(3)
        pygame.draw.rect(self.screen, bg, r, border_radius=br)
        pygame.draw.rect(self.screen, col if active else CYAN_DIM, r, 1, border_radius=br)
        s = self.font_sm.render(text, True, col if active else GRAY)
        self.screen.blit(s, (x + w // 2 - s.get_width() // 2,
                              y + h // 2 - s.get_height() // 2))

    def _blit_center_y(self, surf, x: int, cy: int):
        self.screen.blit(surf, (x, cy - surf.get_height() // 2))


def main():
    parser = argparse.ArgumentParser(description='Wheel Teleop pygame sender')
    parser.add_argument('--host',       default='127.0.0.1',
                        help='IP address of the receiver (default: 127.0.0.1)')
    parser.add_argument('--ctrl-port',  type=int, default=CTRL_PORT,
                        help=f'UDP port for control packets (default: {CTRL_PORT})')
    parser.add_argument('--state-port', type=int, default=STATE_PORT,
                        help=f'UDP port for state feedback (default: {STATE_PORT})')
    args = parser.parse_args()

    sender = TeleopSender(args.host, args.ctrl_port, args.state_port)
    sender.run()


if __name__ == '__main__':
    main()
