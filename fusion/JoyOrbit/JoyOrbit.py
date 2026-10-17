# JoyOrbit: Fusion add-in that orbits/pans the active viewport from an
# M5StickC Plus running this repo's firmware (x,y,sw CSV, 115200bps, USB serial).
#
#   stick              -> move the view in the current mode
#   sw short press     -> toggle mode: orbit <-> pan (on release)
#   sw long press      -> home view (fires once while still held)
#                         (sw = stick push; the view doesn't move while it's held)
#   BtnA on the device -> re-center (the "# calibrated center" line is picked up)

import json
import os
import sys
import threading
import time
import traceback

import adsk.core

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lib'))
import serial  # noqa: E402  (bundled pyserial in ./lib)
import serial.tools.list_ports  # noqa: E402

# ---- settings ---------------------------------------------------------------
PORT = None                 # e.g. 'COM5'; None = auto-detect
BAUD = 115200
USB_VIDS = (0x0403, 0x1A86, 0x10C4)  # FTDI (StickC Plus), CH9102 (Plus2), CP210x
DEADZONE = 250              # raw counts around center (0-4095 scale) that are ignored
ORBIT_DEG_PER_SEC = 90.0    # orbit speed at full deflection
PAN_PER_SEC = 0.3          # pan speed at full deflection, x eye-target distance
SIGN_X = 1                  # flip to -1 if left/right is reversed
SIGN_Y = -1                 # flip to -1 if up/down is reversed
DEBOUNCE_SEC = 0.3          # ignore sw pushes closer together than this
LONG_PRESS_SEC = 0.8        # hold sw this long to go to the home view
# -----------------------------------------------------------------------------

EVENT_ID = 'JoyOrbitInput'
MODES = ('orbit', 'pan')

app = adsk.core.Application.get()
handlers = []
custom_event = None
reader = None


def log(msg):
    try:
        app.log('[JoyOrbit] ' + msg)
    except Exception:
        pass


def find_port():
    if PORT:
        return PORT
    for p in serial.tools.list_ports.comports():
        if p.vid in USB_VIDS:
            return p.device
    return None


class SerialReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.stop_flag = threading.Event()
        self.ser = None
        self.center = None
        self.boot_samples = []
        self.mode = 0
        self.prev_sw = 1
        self.last_push = 0.0
        self.press_t = None       # when the current (debounced) press started
        self.long_fired = False   # home already sent for the current press

    def run(self):
        while not self.stop_flag.is_set():
            port = find_port()
            if not port:
                self.stop_flag.wait(2)
                continue
            try:
                self.ser = serial.Serial()
                self.ser.port = port
                self.ser.baudrate = BAUD
                self.ser.timeout = 0.5
                self.ser.dtr = False  # don't reset the ESP32 on open
                self.ser.rts = False
                self.ser.open()
                log('connected: ' + port)
                self.send_mode()
                self.read_loop()
            except Exception as e:
                if not self.stop_flag.is_set():
                    log('serial error: {} (retrying)'.format(e))
            finally:
                if self.ser:
                    self.send_mode('')  # clear the mode on the device's LCD
                    self.ser.close()
                    self.ser = None
            self.stop_flag.wait(2)

    def read_loop(self):
        while not self.stop_flag.is_set():
            line = self.ser.readline().decode('ascii', 'ignore').strip()
            if not line:
                continue
            if line.startswith('# calibrated center:'):
                # "# calibrated center: x=2050 y=2041"
                try:
                    parts = dict(kv.split('=') for kv in line.split(':', 1)[1].split())
                    self.center = (int(parts['x']), int(parts['y']))
                    log('center: {}'.format(self.center))
                except Exception:
                    pass
                continue
            try:
                x, y, sw = (int(v) for v in line.split(','))
            except ValueError:
                continue
            self.handle_sw(sw)
            if sw == 0:
                continue  # don't move the view while the stick is pushed in
            if self.center is None:
                # Initial center = average of the first samples (device is at rest)
                self.boot_samples.append((x, y))
                if len(self.boot_samples) >= 10:
                    n = len(self.boot_samples)
                    self.center = (sum(s[0] for s in self.boot_samples) // n,
                                   sum(s[1] for s in self.boot_samples) // n)
                    log('center: {}'.format(self.center))
                continue
            nx = normalize(x - self.center[0])
            ny = normalize(y - self.center[1])
            app.fireCustomEvent(EVENT_ID, json.dumps(
                {'action': 'move', 'nx': nx, 'ny': ny, 'mode': MODES[self.mode]}))

    def handle_sw(self, sw):
        """sw is 0 while pressed. Short press toggles the mode on release;
        holding for LONG_PRESS_SEC goes home once, and its release is ignored."""
        now = time.perf_counter()
        if sw == 0 and self.prev_sw != 0:
            if now - self.last_push >= DEBOUNCE_SEC:
                self.last_push = now
                self.press_t = now
                self.long_fired = False
        elif sw == 0 and self.press_t is not None and not self.long_fired:
            if now - self.press_t >= LONG_PRESS_SEC:
                self.long_fired = True
                log('home')
                app.fireCustomEvent(EVENT_ID, json.dumps({'action': 'home'}))
        elif sw != 0 and self.prev_sw == 0:
            if self.press_t is not None and not self.long_fired:
                self.mode = (self.mode + 1) % len(MODES)
                log('mode: ' + MODES[self.mode])
                self.send_mode()
            self.press_t = None
        self.prev_sw = sw

    def send_mode(self, mode=None):
        """Tell the device which mode to show on its LCD ("M:<mode>\\n")."""
        if mode is None:
            mode = MODES[self.mode]
        try:
            self.ser.write(('M:' + mode + '\n').encode('ascii'))
        except Exception:
            pass

    def stop(self):
        self.stop_flag.set()
        if self.ser:
            try:
                self.ser.cancel_read()
            except Exception:
                pass


def normalize(d):
    """Raw offset -> -1..1 with the dead zone removed."""
    if abs(d) <= DEADZONE:
        return 0.0
    span = 2047 - DEADZONE
    v = (abs(d) - DEADZONE) / span
    return max(-1.0, min(1.0, v if d > 0 else -v))


class InputHandler(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()
        self.last_t = None

    def notify(self, args):
        try:
            now = time.perf_counter()
            dt = 0.0 if self.last_t is None else min(now - self.last_t, 0.1)
            self.last_t = now
            data = json.loads(args.additionalInfo)
            if data['action'] == 'home':
                vp = app.activeViewport
                if vp:
                    vp.goHome()
                return
            nx = data['nx'] * SIGN_X
            ny = data['ny'] * SIGN_Y
            if dt == 0.0 or (nx == 0.0 and ny == 0.0):
                return
            vp = app.activeViewport
            if not vp:
                return
            if data['mode'] == 'pan':
                pan(vp, nx, ny, dt)
            else:
                orbit(vp, nx, ny, dt)
            # Setting vp.camera alone doesn't redraw until the next mouse event
            vp.refresh()
        except Exception:
            log(traceback.format_exc())


def camera_axes(cam):
    view = cam.eye.vectorTo(cam.target)
    up = cam.upVector.copy()
    right = view.crossProduct(up)
    right.normalize()
    up = right.crossProduct(view)  # re-orthogonalize
    up.normalize()
    return view, up, right


def orbit(vp, nx, ny, dt):
    cam = vp.camera
    _, up, right = camera_axes(cam)
    yaw = adsk.core.Matrix3D.create()
    yaw.setToRotation(-nx * ORBIT_DEG_PER_SEC * dt * 3.14159265 / 180, up, cam.target)
    pitch = adsk.core.Matrix3D.create()
    pitch.setToRotation(ny * ORBIT_DEG_PER_SEC * dt * 3.14159265 / 180, right, cam.target)
    yaw.transformBy(pitch)
    eye = cam.eye
    eye.transformBy(yaw)
    up.transformBy(yaw)
    cam.eye = eye
    cam.upVector = up
    cam.isSmoothTransition = False
    vp.camera = cam


def pan(vp, nx, ny, dt):
    cam = vp.camera
    view, up, right = camera_axes(cam)
    k = view.length * PAN_PER_SEC * dt
    right.scaleBy(-nx * k)
    up.scaleBy(-ny * k)
    right.add(up)
    eye = cam.eye
    target = cam.target
    eye.translateBy(right)
    target.translateBy(right)
    cam.eye = eye
    cam.target = target
    cam.isSmoothTransition = False
    vp.camera = cam


def run(context):
    global custom_event, reader
    try:
        custom_event = app.registerCustomEvent(EVENT_ID)
        handler = InputHandler()
        custom_event.add(handler)
        handlers.append(handler)
        reader = SerialReader()
        reader.start()
        log('started')
    except Exception:
        app.userInterface.messageBox('JoyOrbit failed:\n' + traceback.format_exc())


def stop(context):
    global custom_event, reader
    try:
        if reader:
            reader.stop()
            reader.join(2)
            reader = None
        if custom_event:
            for h in handlers:
                custom_event.remove(h)
            app.unregisterCustomEvent(EVENT_ID)
            custom_event = None
        handlers.clear()
        log('stopped')
    except Exception:
        app.userInterface.messageBox('JoyOrbit stop failed:\n' + traceback.format_exc())
