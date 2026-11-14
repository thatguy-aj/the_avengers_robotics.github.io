#!/usr/bin/env python3
"""INAV 7 RealFlight-protocol experimental quadcopter + waypoint planner.

This is a lightweight, original teaching simulator, not RealFlight, Gazebo,
or a validated real-aircraft dynamics model. It exchanges genuine INAV SITL
motor outputs for synthetic sensors/RC input over 127.0.0.1:18083.

The drone indicator only moves from motor outputs returned by INAV SITL.
Route overlays and progress tracking do NOT auto-pilot the aircraft.
No flight commands are sent to real hardware. No external packages required.
"""
from __future__ import annotations

import math
import re
import socketserver
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field

HOST, PORT = '127.0.0.1', 18083
HEADER = 'm-channelValues-0to1 xsi:type="SOAP-ENC:Array" SOAP-ENC:arrayType="xsd:double[12]"'



@dataclass(frozen=True)
class Waypoint:
    name: str
    east_m: float
    north_m: float
    planned_alt_m: float = 5.0


# Local virtual coordinates in meters, not latitude/longitude or real GPS.
# The last waypoint returns to the start above HOME; it does not trigger a landing.
SQUARE_ROUTE = (
    Waypoint('WP1', 0.0, 15.0),
    Waypoint('WP2', 20.0, 15.0),
    Waypoint('WP3', 20.0, 0.0),
    Waypoint('HOME', 0.0, 0.0),
)

ARRIVAL_RADIUS_M = 2.0
MIN_ARRIVAL_HEIGHT_M = 3.0


@dataclass
class MissionTracker:
    """Compare measured simulated position with the plan; never alter flight input."""
    active: bool = False
    next_index: int = 0
    completed: bool = False
    last_event: str = 'Plan loaded. Press Track Route when ready.'

    def start(self) -> None:
        if self.completed:
            self.reset()
        self.active = True
        self.last_event = 'Tracking enabled. Fly manually toward WP1.'

    def pause(self) -> None:
        self.active = False
        if not self.completed:
            self.last_event = 'Tracking paused.'

    def reset(self) -> None:
        self.active = False
        self.next_index = 0
        self.completed = False
        self.last_event = 'Route reset. No waypoints marked complete.'

    def update(self, east_m: float, north_m: float, alt_m: float, armed: bool) -> bool:
        """Return True only when a new waypoint is reached from real simulated position."""
        if not self.active or self.completed or not armed:
            return False
        if alt_m < MIN_ARRIVAL_HEIGHT_M:
            return False
        waypoint = SQUARE_ROUTE[self.next_index]
        if math.hypot(east_m - waypoint.east_m, north_m - waypoint.north_m) > ARRIVAL_RADIUS_M:
            return False
        self.next_index += 1
        self.last_event = f'{waypoint.name} reached at {alt_m:.1f} m AGL.'
        if self.next_index == len(SQUARE_ROUTE):
            self.completed = True
            self.active = False
            self.last_event = 'Square route COMPLETE (above home; landing not verified).'
        return True

    def target(self) -> Waypoint | None:
        return None if self.completed else SQUARE_ROUTE[self.next_index]


@dataclass
class State:
    lock: threading.RLock = field(default_factory=threading.RLock)
    requests: int = 0
    exchanges: int = 0
    motors: list[float] = field(default_factory=lambda: [0.0] * 4)
    motor_spool: list[float] = field(default_factory=lambda: [0.0] * 4)
    zero_motor_packets: int = 0
    last_seen: float = 0.0
    last_update: float = field(default_factory=time.monotonic)
    throttle: float = 0.0
    armed_switch: bool = False
    roll_cmd: float = 0.0
    pitch_cmd: float = 0.0
    yaw_cmd: float = 0.0
    aux6: bool = False
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    trace: list[tuple[float,float]] = field(default_factory=list)
    mission: MissionTracker = field(default_factory=MissionTracker)

    def tick(self, motor_vals: list[float]) -> None:
        # Coarse, deliberately bounded dynamics. Movement requires INAV motors.
        now = time.monotonic()
        dt = max(0.001, min(0.04, now - self.last_update))
        self.last_update = now
        self.exchanges += 1
        self.last_seen = now
        self.motors = [max(0, min(1, m)) for m in motor_vals[:4]]
        # Model motor/propeller inertia instead of treating each command as instant thrust.
        # Keep the original INAV outputs in self.motors for honest diagnostics.
        blend = 1.0 - math.exp(-dt / 0.15)
        self.motor_spool = [s + blend * (target - s)
                            for s, target in zip(self.motor_spool, self.motors)]
        if self.armed_switch and all(v < 0.001 for v in self.motors):
            self.zero_motor_packets += 1
        m = self.motor_spool
        avg = sum(m) / 4
        # Approximate quad-X torque signs (not hardware calibrated).
        roll_torque = (m[0] + m[3]) - (m[1] + m[2])
        pitch_torque = (m[0] + m[1]) - (m[2] + m[3])
        yaw_torque = (m[0] + m[2]) - (m[1] + m[3])
        self.roll += ((roll_torque * 35 - 2.0*self.roll) * dt)
        self.pitch += ((pitch_torque * 35 - 2.0*self.pitch) * dt)
        self.yaw = (self.yaw + yaw_torque * 25 * dt) % 360
        self.roll = max(-25, min(25, self.roll))
        self.pitch = max(-25, min(25, self.pitch))
        # Experimental visual model: motor output around 0.20 roughly hovers.
        # This is NOT a calibrated airframe thrust curve.
        az = (avg / 0.20 - 1.0) * 9.81 - self.vz * 0.65
        self.vz = max(-8, min(8, self.vz + az*dt))
        self.z = max(0.0, self.z + self.vz*dt)
        if self.z <= 0 and self.vz < 0: self.vz = 0
        if self.z > 0.025:
            # Horizontal motion from motor imbalance, not from planned WP directly.
            self.vx += (math.sin(math.radians(self.pitch)) * 9.81 - self.vx*.7)*dt
            self.vy += (math.sin(math.radians(self.roll)) * 9.81 - self.vy*.7)*dt
            self.x += self.vx*dt
            self.y += self.vy*dt
        else:
            self.vx *= .9
            self.vy *= .9
        self.x = max(-90, min(90, self.x))
        self.y = max(-90, min(90, self.y))
        self.mission.update(self.x, self.y, self.z, self.armed_switch)
        if self.exchanges % 20 == 0:
            self.trace.append((self.x, self.y))
            if len(self.trace)>400: self.trace.pop(0)

    def response(self) -> str:
        ch = [0.5+self.roll_cmd*.5, 0.5+self.pitch_cmd*.5, self.throttle,
              0.5+self.yaw_cmd*.5, 1.0 if self.armed_switch else 0.0,
              1.0 if self.aux6 else 0.0] + [0.0]*6
        values = ''.join(f'<item>{max(0,min(1,v)):.4f}</item>' for v in ch)
        status = 'CAS-FLYING' if self.z > 0.1 else 'CAS-WAITINGTOLAUNCH'
        gs = math.hypot(self.vx,self.vy)
        # Names and units match INAV 7's RealFlight simulator parser.
        data = {
            'm-airspeed-MPS': gs,
            'm-altitudeASL-MTR': 200.0+self.z,
            'm-altitudeAGL-MTR': self.z,
            'm-groundspeed-MPS': gs,
            'm-pitchRate-DEGpSEC': 0,
            'm-rollRate-DEGpSEC': 0,
            'm-yawRate-DEGpSEC': 0,
            'm-azimuth-DEG': (360-self.yaw-90)%360,
            'm-inclination-DEG': -self.pitch,
            'm-roll-DEG': self.roll,
            'm-aircraftPositionX-MTR': self.x,
            'm-aircraftPositionY-MTR': self.y,
            'm-velocityWorldU-MPS': self.vx,
            'm-velocityWorldV-MPS': self.vy,
            'm-velocityWorldW-MPS': self.vz,
            'm-accelerationBodyAX-MPS2': 0,
            'm-accelerationBodyAY-MPS2': 0,
            'm-accelerationBodyAZ-MPS2': -9.81,
            'm-batteryVoltage-VOLTS': 16.4,
            'm-batteryCurrentDraw-AMPS': 0,
        }
        fields = ''.join(f'<{k}>{v:.5f}</{k}>' for k,v in data.items())
        return (f'<soap:Envelope xmlns:soap="http://schemas.xmlsoap.org/soap/envelope/" '
                f'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                f'xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/">'
                f'<soap:Body><ExchangeDataResponse><ExchangeDataResult>'
                f'{fields}<m-currentAircraftStatus>{status}</m-currentAircraftStatus>'
                f'<{HEADER}>{values}</m-channelValues-0to1>'
                f'</ExchangeDataResult></ExchangeDataResponse></soap:Body></soap:Envelope>')

STATE = State()

class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(1.2)
        buf = bytearray()
        try:
            # INAV 7 has a nonstandard Content-Length for requests: it counts
            # inner SOAP data but not the envelope. Read to closing envelope.
            while len(buf) < 12000 and b'</soap:Envelope>' not in buf:
                chunk = self.request.recv(4096)
                if not chunk: break
                buf.extend(chunk)
            if b'\r\n\r\n' not in buf: return
            head, body = bytes(buf).split(b'\r\n\r\n', 1)
            m = re.search(br'(?im)^soapaction:\s*([^\r\n]+)', head)
            action = m.group(1).decode('ascii','replace').strip() if m else ''
            with STATE.lock:
                STATE.requests += 1
                if action == 'ExchangeData':
                    items = re.findall(br'<item>([^<]+)</item>',body)
                    if len(items) >= 4:
                        try: motor = [float(v) for v in items[:4]]
                        except ValueError: motor = [0.0]*4
                        STATE.tick(motor)
                    xml = STATE.response()
                else:
                    xml = ('<soap:Envelope xmlns:soap="http://schemas.xmlsoap.org/soap/envelope/">'
                           f'<soap:Body><{action or "Response"}Response /></soap:Body></soap:Envelope>')
            payload = xml.encode('utf-8')
            # INAV 7 creates a NEW TCP connection for each RealFlight SOAP request.
            # Responding instantly can generate hundreds of short-lived sockets per
            # second and exhaust Windows' ephemeral TCP ports (TIME_WAIT).
            # Pace the controller by delaying each ExchangeData response rather
            # than sleeping after sending, which did not regulate request rate.
            if action == 'ExchangeData':
                time.sleep(0.030)   # target <= ~33 physics/sensor updates per second
            self.request.sendall((f'HTTP/1.1 200 OK\r\nContent-Length: {len(payload)}\r\n'
                                  'Content-Type: text/xml; charset=UTF-8\r\n'
                                  'Connection: close\r\n\r\n').encode()+payload)
        except (ConnectionError, TimeoutError, OSError):
            pass

class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

class App:
    """Ground-station-like VIEW of virtual state: waypoints are not autopilot commands."""
    MAP_SCALE = 8.0       # pixels per virtual meter
    MAP_CENTER_X = 10.0  # center the square in the window
    MAP_CENTER_Y = 7.5

    def __init__(self, root):
        self.root = root
        root.title('SAR Drone | INAV SITL Flight + Square Waypoint Planner')
        root.geometry('970x780')
        root.minsize(800, 650)
        root.configure(bg='#101923')

        self.info = tk.Label(root, bg='#101923', fg='white', font=('Arial', 12),
            text='Waiting for INAV 7 SITL RealFlight connection on 127.0.0.1:18083')
        self.info.pack(pady=(9, 3))
        self.mission_label = tk.Label(root, bg='#101923', fg='#f3d588', font=('Arial', 11),
            text='PLANNED ROUTE: HOME -> WP1 -> WP2 -> WP3 -> HOME | 5 m target altitude',
            wraplength=900)
        self.mission_label.pack(pady=(0, 8))
        self.canvas = tk.Canvas(root, width=910, height=465, bg='#172c27', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True, padx=15)
        self.status = tk.Label(root, bg='#101923', fg='#b8d4eb', font=('Consolas',10),
            text='INAV must connect and output four motor values before the quad can move',
            wraplength=950)
        self.status.pack(pady=7)

        controls = tk.Frame(root, bg='#101923')
        controls.pack(pady=(4, 1))
        self.arm = tk.Button(controls, text='ARM switch: OFF (A)', command=self.toggle_arm,
                            width=21, bg='#354f66', fg='white')
        self.arm.pack(side='left', padx=4)
        tk.Button(controls, text='Throttle + (W)', command=lambda: self.throttle(+.05), width=17).pack(side='left', padx=4)
        tk.Button(controls, text='Throttle - (S)', command=lambda: self.throttle(-.05), width=17).pack(side='left', padx=4)
        tk.Button(controls, text='Reset position (R)', command=self.reset_pos, width=18).pack(side='left', padx=4)

        mission_controls = tk.Frame(root, bg='#101923')
        mission_controls.pack(pady=(4, 2))
        self.track_button = tk.Button(mission_controls, text='Start route tracking (manual flight)',
                                      command=self.toggle_tracking, width=31,
                                      bg='#235c69', fg='white')
        self.track_button.pack(side='left', padx=7)
        tk.Button(mission_controls, text='Reset route progress', command=self.reset_route,
                  width=24).pack(side='left', padx=7)
        tk.Label(root, bg='#101923', fg='#9bb3ae', font=('Arial', 10),
                 text='A = ARM CH5   W/S = throttle   Arrows = manual pitch/roll   R = reset position (disarmed only)').pack(pady=(3, 1))
        tk.Label(root, bg='#101923', fg='#f3d588', font=('Arial', 10),
                 text='ROUTE DISPLAY + ARRIVAL TRACKING ONLY: no autonomous guidance, GPS, or actual landing command').pack(pady=(0, 5))

        root.bind_all('<KeyPress>', self.keydown)
        root.bind_all('<KeyRelease>', self.keyup)
        self.draw()

    def toggle_arm(self):
        with STATE.lock:
            STATE.armed_switch = not STATE.armed_switch
            STATE.throttle = 0.0  # avoid unexpected simulated acceleration
            on = STATE.armed_switch
            if not on:
                STATE.roll_cmd = 0.0
                STATE.pitch_cmd = 0.0
                STATE.mission.pause()
        self.arm.configure(text=f'ARM switch: {"ON" if on else "OFF"} (A)',
                           bg='#9f552a' if on else '#354f66')
        if not on:
            self.track_button.configure(text='Start route tracking (manual flight)')

    def throttle(self, delta):
        with STATE.lock:
            STATE.throttle = max(0.0, min(1.0, STATE.throttle + delta))

    def reset_pos(self):
        with STATE.lock:
            if STATE.armed_switch:
                STATE.mission.last_event = 'Disarm before resetting position.'
                return
            STATE.x = STATE.y = STATE.z = 0.0
            STATE.vx = STATE.vy = STATE.vz = 0.0
            STATE.pitch = STATE.roll = STATE.yaw = 0.0
            STATE.trace.clear()
            STATE.throttle = 0.0
            STATE.motor_spool = [0.0]*4
            STATE.mission.reset()
        self.track_button.configure(text='Start route tracking (manual flight)')

    def toggle_tracking(self):
        with STATE.lock:
            m = STATE.mission
            if m.active:
                m.pause()
            else:
                m.start()
            running = m.active
        self.track_button.configure(text='Pause route tracking' if running
                                    else 'Start route tracking (manual flight)')

    def reset_route(self):
        with STATE.lock:
            STATE.mission.reset()
        self.track_button.configure(text='Start route tracking (manual flight)')

    def keydown(self, event):
        key = event.keysym.lower()
        if key == 'a' and not getattr(self, 'a_held', False):
            self.a_held = True
            self.toggle_arm()
        elif key == 'w':
            self.throttle(+.04)
        elif key == 's':
            self.throttle(-.04)
        elif key == 'r':
            self.reset_pos()
        elif event.keysym in ('Left','Right','Up','Down'):
            with STATE.lock:
                if event.keysym == 'Left': STATE.roll_cmd = -.4
                if event.keysym == 'Right': STATE.roll_cmd = .4
                if event.keysym == 'Up': STATE.pitch_cmd = -.4
                if event.keysym == 'Down': STATE.pitch_cmd = .4

    def keyup(self, event):
        key = event.keysym.lower()
        if key == 'a':
            self.a_held = False
        elif event.keysym in ('Left', 'Right', 'Up', 'Down'):
            with STATE.lock:
                STATE.roll_cmd = STATE.pitch_cmd = 0.0

    def xy(self, east_m: float, north_m: float, cx: float, cy: float) -> tuple[float,float]:
        return (cx + (east_m - self.MAP_CENTER_X)*self.MAP_SCALE,
                cy - (north_m - self.MAP_CENTER_Y)*self.MAP_SCALE)

    def draw(self):
        c = self.canvas
        c.delete('all')
        w = max(650, c.winfo_width())
        h = max(300, c.winfo_height())
        cx, cy = w/2, h/2

        for x in range(-100, 101, 10):
            px,_ = self.xy(x, 0, cx,cy)
            if 0 <= px <= w: c.create_line(px,0,px,h,fill='#294138')
        for y in range(-100, 101, 10):
            _,py = self.xy(0,y,cx,cy)
            if 0 <= py <= h: c.create_line(0,py,w,py,fill='#294138')
        c.create_text(14, 15, anchor='nw', fill='#bfdad1',
                      text='TOP VIEW | local virtual meters | grid: 10 m | N up, E right',
                      font=('Arial', 11))
        c.create_text(w-75, 20, anchor='ne', fill='#f3d588',
                      text='N  ^', font=('Arial', 12, 'bold'))

        with STATE.lock:
            motors=list(STATE.motors)
            x,y,z=STATE.x, STATE.y, STATE.z
            trace=list(STATE.trace)
            exchanges=STATE.exchanges
            last_seen=STATE.last_seen
            throttle=STATE.throttle
            armed=STATE.armed_switch
            zero_packets=STATE.zero_motor_packets
            mission=STATE.mission
            target=mission.target()
            visited=mission.next_index
            completed=mission.completed
            tracking=mission.active
            event=mission.last_event
            yaw=STATE.yaw

        home = self.xy(0,0,cx,cy)
        mission_points=[home]+[self.xy(wp.east_m,wp.north_m,cx,cy) for wp in SQUARE_ROUTE]
        c.create_line(*[v for pt in mission_points for v in pt],
                      fill='#e3bc65', width=3, dash=(7, 4), joinstyle='round')

        # Draw all planned stops; completed stops are green.
        for idx,wp in enumerate(SQUARE_ROUTE):
            px,py = self.xy(wp.east_m,wp.north_m,cx,cy)
            radius=16 if idx == visited and not completed else 13
            fill='#74d1a5' if idx < visited else ('#f8cf73' if idx == visited else '#96cae8')
            if wp.name == 'HOME':
                # Last marker sits over launch point; outline indicates home.
                c.create_oval(px-radius-4,py-radius-4,px+radius+4,py+radius+4,
                              outline='#f8cf73',width=2)
            c.create_oval(px-radius,py-radius,px+radius,py+radius,fill=fill,outline='#152b2b',width=2)
            c.create_text(px,py, text=wp.name.replace('WP',''),
                          fill='#102924',font=('Arial',10,'bold'))
            c.create_text(px,py-26, text=f'{wp.name} ({wp.east_m:g} E, {wp.north_m:g} N)',
                          fill='#f4eee0',font=('Arial',9))
        if len(trace) > 1:
            pts=[]
            for tx,ty in trace:
                pts.extend(self.xy(tx,ty,cx,cy))
            c.create_line(*pts, fill='#70e9f4',width=2)
        dx,dy = self.xy(x,y,cx,cy)
        c.create_oval(dx-11,dy-11,dx+11,dy+11,outline='white',fill='#f36e56',width=3)
        c.create_line(dx,dy,dx+20*math.sin(math.radians(yaw)),
                      dy-20*math.cos(math.radians(yaw)),fill='white',width=2)
        c.create_text(dx+20,dy+18,text=f'{z:.1f} m AGL', fill='white',font=('Arial',10))

        # Altitude display is actual (synthetic-physics) altitude, NOT desired altitude.
        c.create_rectangle(w-56,58,w-28,h-42,outline='#adbaae')
        available=max(1,h-102)
        v=max(0,min(1,z/20))
        c.create_rectangle(w-55,h-42-v*available,w-29,h-43,fill='#71c4e7',outline='')
        c.create_text(w-42,h-23,fill='white',text='ALT')

        connected=time.monotonic()-last_seen < 2
        self.info.configure(text=('CONNECTED: INAV SITL is exchanging motor signals'
                            if connected else 'Waiting for INAV SITL RealFlight connection on 127.0.0.1:18083'),
                            fg=('#7be4ad' if connected else '#fff2ac'))
        if completed:
            mission_status='COMPLETE (position returned above HOME; no landing commanded)'
        elif target:
            distance=math.hypot(target.east_m-x,target.north_m-y)
            mission_status=f'{visited}/4 reached | next {target.name} {distance:.1f} m away | ' + ('TRACKING' if tracking else 'PLAN ONLY')
        else:
            mission_status='PLAN ONLY'
        self.mission_label.configure(text=f'{mission_status} | {event}')
        self.status.configure(text=(
            f'Motors: {[round(v,2) for v in motors]}   packets: {exchanges}   zero packets: {zero_packets}   '
            f'CH5 ARM switch: {"ON" if armed else "OFF"}   throttle: {throttle:.2f}\n'
            f'Position: E={x:.1f} m  N={y:.1f} m  AGL={z:.1f} m (synthetic physics)'))
        self.root.after(100,self.draw)

def self_test():
    # All route state logic is tested separately from a live SITL process.
    tracker=MissionTracker()
    assert tracker.target().name == 'WP1'
    assert not tracker.update(0,15,5,True), 'A stopped tracker must not advance'
    tracker.start()
    assert not tracker.update(0,15,0,True), 'On-ground arrival must not count'
    assert not tracker.update(0,15,5,False), 'Disarmed arrival must not count'
    assert not tracker.update(5,15,5,True), 'Out-of-radius arrival must not count'
    for index,wp in enumerate(SQUARE_ROUTE):
        assert tracker.update(wp.east_m,wp.north_m,5,True), f'{wp.name} should count'
        assert tracker.next_index == index+1
    assert tracker.completed and not tracker.active and tracker.target() is None
    tracker.reset()
    assert tracker.next_index == 0 and not tracker.completed
    print('PASS: square mission route, armed/altitude proximity and reset')
    s=State()
    s.tick([.65,.65,.65,.65])
    assert s.z >= 0
    xml=s.response()
    assert HEADER in xml and len(re.findall(r'<item>.*?</item>',xml)) == 12
    assert len(SQUARE_ROUTE) == 4
    print('PASS: RealFlight response contains 12 RC channels')
    print('PASS: drone position updates only from INAV motor outputs (no scripted travel)')
    print('OFFLINE SELF-TEST ONLY: live SITL flight/manual waypoint reach not verified')

def main():
    if '--self-test' in sys.argv: self_test(); return
    try:
        server=Server((HOST,PORT),Handler)
    except OSError as err:
        print(f'ERROR: TCP port {PORT} unavailable: {err}',file=sys.stderr)
        raise SystemExit(1)
    thread=threading.Thread(target=server.serve_forever,daemon=True)
    thread.start()
    print(f'INAV RealFlight-protocol lab listening on {HOST}:{PORT}')
    print('Rate-limited to roughly 33 sensor updates/s to reduce Windows TCP socket exhaustion.')
    print('Keep this window running; start INAV 7 SITL with RealFlight simulation enabled.')
    try:
        root=tk.Tk(); App(root); root.mainloop()
    finally:
        server.shutdown();server.server_close()

if __name__ == '__main__': main()
