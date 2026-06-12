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

CTRL_PORT  = 7700
STATE_PORT = 7701

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

# ── Gamepad axis / button mapping (Steam Deck SDL2, no Steam Input) ──────────
SD_AXIS_LX  = 0   # left  stick X
SD_AXIS_LY  = 1   # left  stick Y  (-1=fwd)
SD_AXIS_L2  = 2   # left  trigger  (-1=rest, +1=pressed)
SD_AXIS_RY  = 4   # right stick Y  (-1=up/throttle)
SD_DEADZONE = 0.12

SD_BTN_A  = 0    # south  — snap BR leg to 0°
SD_BTN_B  = 1    # east   — snap FR leg to 0°
SD_BTN_X  = 2    # west   — snap BL leg to 0°
SD_BTN_Y  = 3    # north  — snap FL leg to 0°
SD_BTN_L1 = 4    # extend all legs
SD_BTN_R1 = 5    # retract all legs
SD_BTN_L4 = 11   # upper-left  paddle → FL wheel
SD_BTN_L5 = 13   # lower-left  paddle → BL wheel
SD_BTN_R4 = 12   # upper-right paddle → FR wheel
SD_BTN_R5 = 14   # lower-right paddle → BR wheel

SMOOTH_ALPHA  = 0.25   # axis low-pass per 60fps tick (~167 ms to 94% of target)
THROTTLE_RATE = 60.0   # % per second at full joystick deflection

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
            wheel_speed=[0]*4, leg_angles=[0]*4,
            wheel_currents=[0]*4, leg_currents=[0]*4,
            speed_pct=20,
        )
        self._state_lock = threading.Lock()
        self._last_recv  = 0.0

        self.speed_pct = 20

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

        self.font_sm  = pygame.font.SysFont('Courier New', 10, bold=True)
        self.font_med = pygame.font.SysFont('Courier New', 13, bold=True)
        self.font_lg  = pygame.font.SysFont('Courier New', 17, bold=True)

        self.hb_on    = False
        self.hb_timer = 0.0
        self.running  = True

        self._speed_dragging = False
        self._send_timer     = 0.0

        # Clickable rects (populated each frame)
        self._estop_rect = pygame.Rect(0, 0, 0, 0)
        self._reset_rect = pygame.Rect(0, 0, 0, 0)
        self._spd_track  = pygame.Rect(0, 0, 0, 0)

        # Motor reset flash state
        self._reset_flash = 0.0

        self._smooth_lx     = 0.0
        self._smooth_ly     = 0.0
        self.compliant_mode = False
        self._compliant_rect = pygame.Rect(0, 0, 0, 0)

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
        msg = {'type': 'ctrl', 'speed_pct': self.speed_pct}
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

    def do_compliant_toggle(self):
        self.compliant_mode = not self.compliant_mode
        try:
            self._ctrl_sock.sendto(
                json.dumps({'type': 'compliant', 'value': self.compliant_mode}).encode(),
                (self.host, self.ctrl_port))
        except Exception:
            pass

    # ── Gamepad ──────────────────────────────────────────────────────────────

    def _connect_joy(self):
        if pygame.joystick.get_count() > 0:
            try:
                self.joy = pygame.joystick.Joystick(0)
                self.joy.init()
            except Exception:
                self.joy = None

    def _poll_gamepad(self, dt: float = 1 / 60):
        if self.joy is None:
            return

        try:
            n_axes = self.joy.get_numaxes()
            n_btns = self.joy.get_numbuttons()

            def axis(i):
                return self.joy.get_axis(i) if n_axes > i else 0.0

            def btn(i):
                return bool(self.joy.get_button(i)) if n_btns > i else False

            # L2 trigger — any press resets all motion
            l2_raw = axis(SD_AXIS_L2)
            self.ctrl['l2'] = l2_raw > 0.0
            if self.ctrl['l2']:
                self._smooth_lx = 0.0
                self._smooth_ly = 0.0
                self.ctrl.update(lx=0.0, ly=0.0, ry=0.0)
                return

            # Left stick arcade drive — low-pass smoothed
            lx_raw = axis(SD_AXIS_LX)
            ly_raw = axis(SD_AXIS_LY)
            lx_raw = lx_raw if abs(lx_raw) >= SD_DEADZONE else 0.0
            ly_raw = ly_raw if abs(ly_raw) >= SD_DEADZONE else 0.0
            self._smooth_lx += SMOOTH_ALPHA * (lx_raw - self._smooth_lx)
            self._smooth_ly += SMOOTH_ALPHA * (ly_raw - self._smooth_ly)
            self.ctrl['lx'] = self._smooth_lx
            self.ctrl['ly'] = self._smooth_ly

            # Right stick Y — rate-based throttle: up increases, down decreases
            ry = axis(SD_AXIS_RY)
            ry_val = ry if abs(ry) >= SD_DEADZONE else 0.0
            self.ctrl['ry'] = ry_val
            if abs(ry_val) > 0:
                delta = -ry_val * THROTTLE_RATE * dt
                self.speed_pct = max(0, min(100, int(self.speed_pct + delta)))

            # Shoulder buttons
            self.ctrl['l1'] = btn(SD_BTN_L1)
            self.ctrl['r1'] = btn(SD_BTN_R1)

            # Face buttons
            self.ctrl['btn_a'] = btn(SD_BTN_A)
            self.ctrl['btn_b'] = btn(SD_BTN_B)
            self.ctrl['btn_x'] = btn(SD_BTN_X)
            self.ctrl['btn_y'] = btn(SD_BTN_Y)

            # Back paddles
            self.ctrl['l4'] = btn(SD_BTN_L4)
            self.ctrl['l5'] = btn(SD_BTN_L5)
            self.ctrl['r4'] = btn(SD_BTN_R4)
            self.ctrl['r5'] = btn(SD_BTN_R5)

            # D-pad (hat preferred, buttons as fallback for standard mapping)
            if self.joy.get_numhats() > 0:
                hx, hy = self.joy.get_hat(0)
                self.ctrl['dpad'] = [hx, hy]
            else:
                # Standard gamepad mapping: 12=up 13=down 14=left 15=right
                up    = btn(12)
                down  = btn(13)
                left  = btn(14)
                right = btn(15)
                self.ctrl['dpad'] = [
                    (1 if right else 0) - (1 if left else 0),
                    (1 if up   else 0) - (1 if down else 0),
                ]

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
                elif ev.key == pygame.K_c:
                    self.do_compliant_toggle()

            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                p = ev.pos
                if self._estop_rect.collidepoint(p):
                    self.do_estop()
                elif self._reset_rect.collidepoint(p):
                    self.do_motor_reset()
                elif self._compliant_rect.collidepoint(p):
                    self.do_compliant_toggle()
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

    # ── Drawing ──────────────────────────────────────────────────────────────

    def _draw(self):
        W, H = self.screen.get_size()
        self.screen.fill(BG)

        y = self._draw_header(0, W)
        y = self._draw_control_bar(y, W)
        self._draw_main(y, W, H - y)

    def _draw_header(self, y: int, W: int) -> int:
        linked = (time.monotonic() - self._last_recv) < 0.5
        pygame.draw.rect(self.screen, BG, (0, y, W, HDR_H))
        pygame.draw.line(self.screen, CYAN_DIM, (0, y + HDR_H - 1), (W, y + HDR_H - 1))

        mid_y = y + HDR_H // 2

        hb_col = CYAN if self.hb_on else CYAN_DIM
        pygame.draw.circle(self.screen, hb_col, (14, mid_y), 5)

        self._blit_center_y(self.font_lg.render('WHEEL TELEOP  ◈  ROS2', True, CYAN),
                            28, mid_y)

        gp_txt = ('GAMEPAD: ' + self.joy.get_name()[:22]) if self.joy else 'GAMEPAD: —'
        self._blit_center_y(self.font_sm.render(gp_txt, True, GREEN if self.joy else GRAY),
                            W - 400, mid_y)

        lnk_txt = '● LINKED' if linked else '● NO LINK'
        lnk_col = GREEN if linked else RED
        lnk_s = self.font_sm.render(lnk_txt, True, lnk_col)
        self.screen.blit(lnk_s, (W - lnk_s.get_width() - 10, mid_y - lnk_s.get_height() // 2))

        return y + HDR_H

    def _draw_control_bar(self, y: int, W: int) -> int:
        pygame.draw.rect(self.screen, PANEL, (0, y, W, BAR_H))
        pygame.draw.line(self.screen, CYAN_DIM, (0, y + BAR_H - 1), (W, y + BAR_H - 1))

        mid_y = y + BAR_H // 2
        cx = 12

        # Speed label
        lbl = self.font_sm.render('SPEED %', True, CYAN)
        self._blit_center_y(lbl, cx, mid_y)
        cx += lbl.get_width() + 10

        # Slider track
        TW, TH = 210, 8
        tr = pygame.Rect(cx, mid_y - TH // 2, TW, TH)
        pygame.draw.rect(self.screen, DARK, tr, border_radius=4)
        fill = int(TW * self.speed_pct / 100)
        if fill > 0:
            pygame.draw.rect(self.screen,
                             CYAN, pygame.Rect(cx, mid_y - TH // 2, fill, TH),
                             border_radius=4)
        pygame.draw.circle(self.screen, CYAN, (cx + fill, mid_y), 9)
        self._spd_track = pygame.Rect(cx - 9, y, TW + 18, BAR_H)
        cx += TW + 14

        pct_s = self.font_med.render(f'{self.speed_pct}%', True, WHITE)
        self._blit_center_y(pct_s, cx, mid_y)
        cx += pct_s.get_width() + 20

        host_s = self.font_sm.render(f'HOST: {self.host}:{self.ctrl_port}', True, GRAY)
        self._blit_center_y(host_s, cx, mid_y)

        # Compliant mode toggle button
        BW, BH = 140, 30
        cm_x = W - BW * 3 - 50
        mr_y = mid_y - BH // 2
        cm_rect = pygame.Rect(cm_x, mr_y, BW, BH)
        cm_on  = self.compliant_mode
        cm_bg  = (0, 50, 20) if cm_on else DARK
        cm_fg  = GREEN       if cm_on else CYAN_DIM
        pygame.draw.rect(self.screen, cm_bg, cm_rect, border_radius=4)
        pygame.draw.rect(self.screen, cm_fg, cm_rect, 1, border_radius=4)
        cm_lbl = '◎ COMPLIANT  [C]' if cm_on else '○ COMPLIANT  [C]'
        cm_s = self.font_sm.render(cm_lbl, True, GREEN if cm_on else GRAY)
        self.screen.blit(cm_s, (cm_x + BW // 2 - cm_s.get_width() // 2,
                                 mr_y + BH // 2 - cm_s.get_height() // 2))
        self._compliant_rect = cm_rect

        # Motor Reset button
        mr_x = W - BW * 2 - 30
        mr_rect = pygame.Rect(mr_x, mr_y, BW, BH)
        flash = self._reset_flash > 0
        mr_bg  = CYAN_DIM if flash else DARK
        mr_fg  = CYAN     if flash else CYAN_DIM
        pygame.draw.rect(self.screen, mr_bg, mr_rect, border_radius=4)
        pygame.draw.rect(self.screen, mr_fg, mr_rect, 1, border_radius=4)
        mr_s = self.font_sm.render('⟳ MOTOR RESET  [R]', True, CYAN if flash else GRAY)
        self.screen.blit(mr_s, (mr_x + BW // 2 - mr_s.get_width() // 2,
                                 mr_y + BH // 2 - mr_s.get_height() // 2))
        self._reset_rect = mr_rect

        # E-STOP button
        es_x = W - BW - 10
        es_rect = pygame.Rect(es_x, mr_y, BW, BH)
        pygame.draw.rect(self.screen, (42, 0, 16), es_rect, border_radius=4)
        pygame.draw.rect(self.screen, RED_DIM, es_rect, 2, border_radius=4)
        es_s = self.font_sm.render('⚠ E-STOP  [E]', True, RED)
        self.screen.blit(es_s, (es_x + BW // 2 - es_s.get_width() // 2,
                                 mr_y + BH // 2 - es_s.get_height() // 2))
        self._estop_rect = es_rect

        return y + BAR_H

    def _draw_main(self, y: int, W: int, H: int):
        cx_x = SIDE_W
        cx_w = W - 2 * SIDE_W

        pygame.draw.line(self.screen, CYAN_DIM, (SIDE_W, y), (SIDE_W, y + H))
        pygame.draw.line(self.screen, CYAN_DIM, (W - SIDE_W, y), (W - SIDE_W, y + H))

        self._draw_left_panel(pygame.Rect(0, y, SIDE_W, H))
        self._draw_center_panel(pygame.Rect(cx_x, y, cx_w, H))
        self._draw_right_panel(pygame.Rect(W - SIDE_W, y, SIDE_W, H))

    def _draw_left_panel(self, rect: pygame.Rect):
        pygame.draw.rect(self.screen, PANEL, rect)
        x, y, w = rect.x, rect.y, rect.width
        cy = y + 6

        cy = self._section_hdr('LEFT CONTROLS', x, cy, w)

        # Left joystick
        js_r  = 52
        js_cx = x + w // 2
        js_cy = cy + js_r + 14
        self._draw_joystick(js_cx, js_cy, js_r,
                            self.ctrl['lx'], self.ctrl['ly'],
                            CYAN, 'LEFT STICK  —  ARCADE DRIVE')
        cy = js_cy + js_r + 20

        # D-pad
        cy = self._section_hdr('D-PAD  —  EXTEND LEGS', x, cy, w)
        cy += 4
        dp = self.ctrl['dpad']
        dpad_rows = [
            ('▲ UP',    'FL leg', dp[1] == 1),
            ('◄ LEFT',  'BL leg', dp[0] == -1),
            ('► RIGHT', 'FR leg', dp[0] == 1),
            ('▼ DOWN',  'BR leg', dp[1] == -1),
        ]
        for arrow, corner, active in dpad_rows:
            col = GREEN if active else GRAY
            self._mini_btn(f'{arrow}  →  {corner}', x + 8, cy, w - 16, 22, active, col)
            cy += 26

        cy += 4
        self._mini_btn('L1  —  EXTEND ALL LEGS',   x + 8, cy, w - 16, 24,
                       self.ctrl['l1'], GREEN)
        cy += 28
        self._mini_btn('R1  —  RETRACT ALL LEGS',  x + 8, cy, w - 16, 24,
                       self.ctrl['r1'], RED)

    def _draw_right_panel(self, rect: pygame.Rect):
        pygame.draw.rect(self.screen, PANEL, rect)
        x, y, w = rect.x, rect.y, rect.width
        cy = y + 6

        cy = self._section_hdr('RIGHT CONTROLS', x, cy, w)

        # Right stick (throttle, Y-only)
        js_r  = 52
        js_cx = x + w // 2
        js_cy = cy + js_r + 14
        self._draw_joystick(js_cx, js_cy, js_r,
                            0, self.ctrl['ry'],
                            GREEN, 'RIGHT STICK  —  THROTTLE',
                            y_only=True)
        cy = js_cy + js_r + 20

        # Face buttons
        cy = self._section_hdr('FACE BUTTONS  —  SNAP LEG 0°', x, cy, w)
        cy += 4
        face_rows = [
            ('Y', 'FL', self.ctrl['btn_y'], YELLOW),
            ('X', 'BL', self.ctrl['btn_x'], BLUE_BTN),
            ('B', 'FR', self.ctrl['btn_b'], RED_BTN),
            ('A', 'BR', self.ctrl['btn_a'], GREEN_BTN),
        ]
        for lbl, corner, active, col in face_rows:
            self._mini_btn(f'{lbl}  →  {corner} leg → 0°', x + 8, cy, w - 16, 22, active, col)
            cy += 26

        cy += 4
        cy = self._section_hdr('PADDLES  —  SPIN WHEEL', x, cy, w)
        cy += 4
        paddle_rows = [
            ('L4', 'FL wheel', self.ctrl['l4']),
            ('L5', 'BL wheel', self.ctrl['l5']),
            ('R4', 'FR wheel', self.ctrl['r4']),
            ('R5', 'BR wheel', self.ctrl['r5']),
        ]
        for lbl, desc, active in paddle_rows:
            self._mini_btn(f'{lbl}  →  {desc}', x + 8, cy, w - 16, 22, active, PURPLE)
            cy += 26

    def _draw_center_panel(self, rect: pygame.Rect):
        x, y, w, h = rect.x, rect.y, rect.width, rect.height

        with self._state_lock:
            ws  = list(self.state['wheel_speed'])
            la  = list(self.state['leg_angles'])
            wc  = list(self.state['wheel_currents'])
            lc  = list(self.state['leg_currents'])

        cy = y + 6

        # Wheel speeds
        cy = self._section_hdr('WHEEL SPEEDS', x, cy, w)
        cy += 4
        cy = self._draw_4cell_row(x, cy, w, 38,
                                  ['FL', 'FR', 'BL', 'BR'], [ws[0], ws[2], ws[1], ws[3]],
                                  lambda v: GREEN if v > 0 else (RED if v < 0 else GRAY),
                                  lambda v: str(v), CYAN_DIM)
        cy += 4

        # Leg angles
        cy = self._section_hdr('LEG ANGLES', x, cy, w)
        cy += 4
        cy = self._draw_4cell_row(x, cy, w, 38,
                                  ['FL', 'FR', 'BL', 'BR'], [la[0], la[2], la[1], la[3]],
                                  lambda _: PURPLE,
                                  lambda v: f'{v}°', PURPLE_DIM)
        cy += 4

        # Robot diagram
        cy = self._section_hdr('ROBOT DIAGRAM', x, cy, w)
        remaining = h - (cy - y) - 70
        diag_h    = max(80, remaining)
        diag_rect = pygame.Rect(x + 4, cy, w - 8, diag_h)
        self._draw_robot_diagram(diag_rect, ws)
        cy += diag_h + 6

        # Current draw
        cy = self._section_hdr('CURRENT DRAW', x, cy, w)
        cy += 4
        hw = w // 2
        q  = hw // 4
        labels_order = ['FL', 'FR', 'BL', 'BR']
        vals_w = [wc[0], wc[2], wc[1], wc[3]]
        vals_l = [lc[0], lc[2], lc[1], lc[3]]

        wc_hdr = self.font_sm.render('WHEELS (mA)', True, CYAN)
        self.screen.blit(wc_hdr, (x + 4, cy))
        lc_hdr = self.font_sm.render('LEGS (mA)', True, CYAN)
        self.screen.blit(lc_hdr, (x + hw + 4, cy))
        cy += wc_hdr.get_height() + 3

        for i in range(4):
            px_w = x + i * q
            px_l = x + hw + i * q
            col_w = GREEN if abs(vals_w[i]) > 20 else GRAY
            col_l = PURPLE if abs(vals_l[i]) > 20 else PURPLE_DIM

            lbl_s = self.font_sm.render(labels_order[i], True, GRAY)
            self.screen.blit(lbl_s, (px_w + 2, cy))
            self.screen.blit(lbl_s, (px_l + 2, cy))

            v_w = self.font_sm.render(str(vals_w[i]), True, col_w)
            v_l = self.font_sm.render(str(vals_l[i]), True, col_l)
            self.screen.blit(v_w, (px_w + 2, cy + 12))
            self.screen.blit(v_l, (px_l + 2, cy + 12))

        cy += 28
        total = sum(abs(v) for v in wc + lc)
        tot_txt = f'TOTAL: {total/1000:.2f} A' if total >= 1000 else f'TOTAL: {total} mA'
        tot_col = RED if total > 5000 else (CYAN if total > 500 else WHITE)
        tot_s = self.font_med.render(tot_txt, True, tot_col)
        self.screen.blit(tot_s, (x + w // 2 - tot_s.get_width() // 2, cy))

    def _draw_robot_diagram(self, rect: pygame.Rect, speeds: list):
        pygame.draw.rect(self.screen, DARK, rect, border_radius=4)
        pygame.draw.rect(self.screen, CYAN_DIM, rect, 1, border_radius=4)

        rw, rh = rect.width, rect.height
        cx = rect.x + rw // 2
        cy = rect.y + rh // 2

        # Grid
        for gx in range(rect.x, rect.right, 40):
            pygame.draw.line(self.screen, (16, 40, 64), (gx, rect.top), (gx, rect.bottom))
        for gy in range(rect.top, rect.bottom, 40):
            pygame.draw.line(self.screen, (16, 40, 64), (rect.left, gy), (rect.right, gy))

        fwd = self.font_sm.render('▲  FORWARD', True, CYAN_DIM)
        self.screen.blit(fwd, (cx - fwd.get_width() // 2, rect.top + 4))

        bw = min(38, rw // 7)
        bh = min(54, rh // 3)
        pygame.draw.rect(self.screen, (10, 34, 64), (cx - bw, cy - bh, bw * 2, bh * 2))
        pygame.draw.rect(self.screen, CYAN, (cx - bw, cy - bh, bw * 2, bh * 2), 2)
        pygame.draw.line(self.screen, CYAN_DIM, (cx, cy - bh), (cx, cy + bh))
        pygame.draw.line(self.screen, CYAN_DIM, (cx - bw, cy), (cx + bw, cy))
        r_lbl = self.font_sm.render('ROBOT', True, CYAN)
        self.screen.blit(r_lbl, (cx - r_lbl.get_width() // 2, cy - r_lbl.get_height() // 2))

        wxo = min(72, rw // 4)
        wyo = min(62, rh // 3)
        wheel_positions = [
            (cx - wxo, cy - wyo, 'FL', 0),
            (cx - wxo, cy + wyo, 'BL', 1),
            (cx + wxo, cy - wyo, 'FR', 2),
            (cx + wxo, cy + wyo, 'BR', 3),
        ]
        for wx, wy, lbl, i in wheel_positions:
            s   = speeds[i]
            col = GREEN if s > 0 else (RED if s < 0 else GRAY)
            dim = GREEN_DIM if s > 0 else (RED_DIM if s < 0 else (21, 46, 72))
            pygame.draw.circle(self.screen, dim, (wx, wy), 18, 5)
            pygame.draw.circle(self.screen, DARK, (wx, wy), 12)
            pygame.draw.circle(self.screen, col,  (wx, wy), 12, 2)
            pygame.draw.circle(self.screen, col,  (wx, wy), 3)
            lbl_s = self.font_sm.render(lbl, True, col)
            self.screen.blit(lbl_s, (wx - lbl_s.get_width() // 2, wy - 26))
            spd_s = self.font_sm.render(str(s), True, WHITE)
            self.screen.blit(spd_s, (wx - spd_s.get_width() // 2, wy + 20))

    def _draw_joystick(self, cx: int, cy: int, r: int,
                       jx: float, jy: float, col, label: str,
                       y_only: bool = False):
        pygame.draw.circle(self.screen, DARK, (cx, cy), r)
        pygame.draw.circle(self.screen, CYAN_DIM, (cx, cy), r, 2)
        pygame.draw.line(self.screen, (16, 40, 64), (cx, cy - r + 3), (cx, cy + r - 3))
        pygame.draw.line(self.screen, (16, 40, 64), (cx - r + 3, cy), (cx + r - 3, cy))

        max_d = r - 13
        tx = cx + (0 if y_only else int(jx * max_d))
        ty = cy + int(jy * max_d)
        pygame.draw.circle(self.screen, col, (tx, ty), 13)
        pygame.draw.circle(self.screen, DARK, (tx, ty), 5)

        lbl_s = self.font_sm.render(label, True, CYAN)
        self.screen.blit(lbl_s, (cx - lbl_s.get_width() // 2, cy - r - 16))

    def _draw_4cell_row(self, x, cy, w, row_h, labels, values, color_fn, fmt_fn, border_col):
        cw = w // 4
        for i, (lbl, val) in enumerate(zip(labels, values)):
            px   = x + i * cw
            cell = pygame.Rect(px + 2, cy, cw - 4, row_h)
            pygame.draw.rect(self.screen, DARK, cell, border_radius=3)
            pygame.draw.rect(self.screen, border_col, cell, 1, border_radius=3)
            ls = self.font_sm.render(lbl, True, GRAY)
            self.screen.blit(ls, (px + cw // 2 - ls.get_width() // 2, cy + 4))
            vs = self.font_med.render(fmt_fn(val), True, color_fn(val))
            self.screen.blit(vs, (px + cw // 2 - vs.get_width() // 2,
                                   cy + row_h - vs.get_height() - 4))
        return cy + row_h

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _section_hdr(self, text: str, x: int, y: int, w: int) -> int:
        s = self.font_sm.render(text, True, CYAN)
        self.screen.blit(s, (x + w // 2 - s.get_width() // 2, y))
        line_y = y + s.get_height() + 3
        pygame.draw.line(self.screen, CYAN_DIM, (x + 4, line_y), (x + w - 4, line_y))
        return line_y + 5

    def _mini_btn(self, text: str, x: int, y: int, w: int, h: int,
                  active: bool, col):
        bg = tuple(max(0, c // 4) for c in col) if active else DARK
        r  = pygame.Rect(x, y, w, h)
        pygame.draw.rect(self.screen, bg, r, border_radius=3)
        pygame.draw.rect(self.screen, col if active else CYAN_DIM, r, 1, border_radius=3)
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
