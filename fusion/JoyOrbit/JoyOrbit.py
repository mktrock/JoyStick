# JoyOrbit: Fusion add-in that orbits/pans the active viewport from an
# XIAO ESP32C3 running this repo's firmware (x,y,sw CSV, 115200bps, USB serial).
# Without USB it connects over BLE instead (Nordic UART Service, same lines).
#
#   stick up/down      -> orbit up/down (pivots on the selection's center, if any)
#   stick left/right   -> zoom (right = in)
#   rotary encoder     -> spin around the model's Z axis like a turntable
#                         (same pivot as the orbit; clockwise = the model
#                         turns clockwise seen from +Z)
#                         (from the "E:<detents>" lines)
#   left switch short  -> snap to the nearest standard view (on release): the
#                         nearest of the 6 faces and 8 iso corners, screen-up
#                         to the nearest model axis; target and distance are kept
#   right switch short -> fit the whole model in view, like F6 (on release)
#   right switch long  -> home view (as soon as it has been held LONG_PRESS_SEC;
#                         nothing more on release)
#                         (switches come from the "B:<n>,<state>" lines)
#   encoder push or stick push held -> the stick pans and the encoder does
#                         nothing while either is down (the encoder push is
#                         the CSV's sw column, the stick push is switch 3)
#   both switches held -> the device recalibrates with the stick where it is now
#                         (the view doesn't move while both are held)
#   BOOT on the device -> re-center (the "# calibrated center" line is picked up)

import asyncio
import json
import math
import os
import sys
import threading
import time
import traceback

import adsk.core

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lib'))
import serial  # noqa: E402  (bundled pyserial in ./lib)
import serial.tools.list_ports  # noqa: E402
try:
    from bleak import BleakClient, BleakScanner  # noqa: E402  (bundled bleak in ./lib)
    BLE_IMPORT_ERROR = None
except Exception as e:  # BLE is optional; USB still works
    BleakClient = BleakScanner = None
    BLE_IMPORT_ERROR = e

# ---- settings ---------------------------------------------------------------
PORT = None                 # e.g. 'COM5'; None = auto-detect
BAUD = 115200
USB_VIDS = (0x303A,)        # Espressif USB Serial/JTAG (XIAO ESP32C3)
USE_BLE = True              # fall back to BLE when no USB port is found
BLE_NAME = 'JoyStick-XIAO'
BLE_SCAN_SEC = 5.0          # scan this long per attempt
DEADZONE = 80              # raw counts from center (radius, 0-4095 scale) that are ignored
EXPO = 1.5                # response curve exponent: 1 = linear, larger = finer near center
ORBIT_DEG_PER_SEC = 90.0    # orbit speed at full deflection
ORBIT_AROUND_SELECTION = True  # orbit around the selection's center when something is selected
MAX_UPDATE_HZ = 30          # view updates per second at most (more makes the mouse sluggish)
PAN_PER_SEC = 0.075        # pan speed at full deflection, x eye-target distance
SIGN_X = 1                  # flip to -1 if left/right is reversed
SIGN_Y = -1                 # flip to -1 if up/down is reversed
LONG_PRESS_SEC = 0.8        # a switch press this long or longer isn't a short press
SPIN_DEG_PER_DETENT = 15.0  # Z-axis spin angle per encoder click
SPIN_SIGN = 1               # flip to -1 if clockwise turns the model counterclockwise
STICK_ZOOM_PER_SEC = 2.0    # zoom factor per second with the stick fully left/right
STICK_ZOOM_SIGN = 1         # flip to -1 if tilting right zooms out
HOME_SWITCH = 2             # 1 = left, 2 = right: long press -> home view
SNAP_SWITCH = 1             # short press -> snap to the nearest view
FIT_SWITCH = 2              # short press -> fit the whole model in view (like F6)
# -----------------------------------------------------------------------------

EVENT_ID = 'JoyOrbitInput'
NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'  # add-in -> device
NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'  # device -> add-in

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
        self.write = None         # bytes -> device, for the current connection
        self.center = None
        self.boot_samples = []
        self.switch = {1: 1, 2: 1}  # switch states (0 = pressed)
        self.press_t = {}           # switch -> when its current press started
        self.combo = {}             # switch -> its current press is used up (both were
                                    # down, or the long press already acted): no action on release
        self.enc_sw = 1             # encoder push, from the CSV's sw column (0 = pressed)
        self.stick_sw = 1           # stick push, switch 3 (0 = pressed)
        self.latest_move = None   # (nx, ny, mode) of the newest sample
        self.move_pending = False # a move event is queued and not yet handled
        self.last_fire = 0.0      # when the last move event was fired
        self.moving = False       # the last fired move was off center

    def run(self):
        if USE_BLE and BLE_IMPORT_ERROR:
            log('BLE unavailable ({}); USB only'.format(BLE_IMPORT_ERROR))
        while not self.stop_flag.is_set():
            port = find_port()
            if port:
                self.serial_session(port)
            elif USE_BLE and BleakClient:
                try:
                    asyncio.run(self.ble_session())
                except Exception as e:
                    if not self.stop_flag.is_set():
                        log('BLE error: {} (retrying)'.format(e))
                finally:
                    self.write = None
            self.stop_flag.wait(2)

    def serial_session(self, port):
        try:
            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = BAUD
            self.ser.timeout = 0.5
            self.ser.dtr = False  # don't reset the ESP32 on open
            self.ser.rts = False
            self.ser.open()
            self.write = self.ser.write
            log('connected: ' + port)
            self.send_mode()
            while not self.stop_flag.is_set():
                line = self.ser.readline().decode('ascii', 'ignore').strip()
                if line:
                    self.handle_line(line)
        except Exception as e:
            if not self.stop_flag.is_set():
                log('serial error: {} (retrying)'.format(e))
        finally:
            if self.ser:
                self.send_mode('')  # tell the device the add-in stopped
                self.ser.close()
                self.ser = None
            self.write = None

    async def ble_session(self):
        device = None

        def on_detect(d, ad):
            nonlocal device
            if device is None and (d.name == BLE_NAME or NUS_SERVICE in ad.service_uuids):
                device = d

        # Poll instead of find_device_by_filter() so stop() isn't held up by the scan
        async with BleakScanner(on_detect):
            t0 = time.perf_counter()
            while (device is None and not self.stop_flag.is_set()
                   and time.perf_counter() - t0 < BLE_SCAN_SEC):
                await asyncio.sleep(0.2)
        if device is None or self.stop_flag.is_set():
            return

        loop = asyncio.get_running_loop()
        buf = bytearray()
        pending = set()  # keep write tasks referenced until they finish

        def on_notify(_, data):
            buf.extend(data)
            while b'\n' in buf:
                i = buf.index(b'\n')
                line = buf[:i].decode('ascii', 'ignore').strip()
                del buf[:i + 1]
                if line:
                    self.handle_line(line)

        async with BleakClient(device, timeout=10) as client:
            def spawn_write(data):
                task = loop.create_task(client.write_gatt_char(NUS_RX, data, response=False))
                pending.add(task)
                task.add_done_callback(pending.discard)

            await client.start_notify(NUS_TX, on_notify)
            # Callable from any thread (and from on_notify, which runs on this loop)
            self.write = lambda data: loop.call_soon_threadsafe(spawn_write, bytes(data))
            log('connected: BLE ' + (device.name or device.address))
            self.send_mode()
            try:
                while client.is_connected and not self.stop_flag.is_set():
                    await asyncio.sleep(0.2)
            finally:
                self.write = None
                if client.is_connected:
                    try:  # tell the device the add-in stopped
                        await client.write_gatt_char(NUS_RX, b'M:\n', response=False)
                    except Exception:
                        pass
        log('BLE disconnected')

    def handle_line(self, line):
        if line.startswith('# calibrated center:'):
            # "# calibrated center: x=2050 y=2041"
            try:
                parts = dict(kv.split('=') for kv in line.split(':', 1)[1].split())
                self.center = (int(parts['x']), int(parts['y']))
                log('center: {}'.format(self.center))
            except Exception:
                pass
            return
        if line.startswith('E:'):
            # "E:<detents>"
            try:
                steps = int(line[2:])
            except ValueError:
                return
            # Ignored while a push is held
            if steps and self.active_mode() == 'orbit':
                app.fireCustomEvent(EVENT_ID, json.dumps({'action': 'spin', 'steps': steps}))
            return
        if line.startswith('B:'):
            # "B:<n>,<state>" (state 0 = pressed; the device debounces)
            try:
                n, state = (int(v) for v in line[2:].split(','))
            except ValueError:
                return
            if n == 3:
                self.stick_sw = state
            else:
                self.handle_switch(n, state)
            return
        try:
            x, y, self.enc_sw = (int(v) for v in line.split(','))
        except ValueError:
            return
        # The CSV keeps coming at 100 Hz while a switch is held, so the long
        # press is timed here instead of waiting for the release.
        self.check_long_press()
        if self.switch[1] == 0 and self.switch[2] == 0:
            return  # both switches = recalibrating on the device: hold the view
        if self.center is None:
            # Initial center = average of the first samples (device is at rest)
            self.boot_samples.append((x, y))
            if len(self.boot_samples) >= 10:
                n = len(self.boot_samples)
                self.center = (sum(s[0] for s in self.boot_samples) // n,
                               sum(s[1] for s in self.boot_samples) // n)
                log('center: {}'.format(self.center))
            return
        nx, ny = normalize(x - self.center[0], y - self.center[1])
        # Samples arrive at 100 Hz, but every move redraws the viewport on
        # Fusion's main thread, which also handles the mouse. So at rest only
        # the first centered sample is sent (to reset the pivot), moves are
        # sent at most MAX_UPDATE_HZ, and if the main thread falls behind only
        # the newest sample is kept instead of letting events queue up.
        moving = nx != 0.0 or ny != 0.0
        self.latest_move = (nx, ny, self.active_mode())
        if not moving and not self.moving:
            return
        now = time.perf_counter()
        if moving and now - self.last_fire < 1.0 / MAX_UPDATE_HZ:
            return
        if not self.move_pending:
            self.move_pending = True
            self.last_fire = now
            self.moving = moving
            app.fireCustomEvent(EVENT_ID, json.dumps({'action': 'move'}))

    def active_mode(self):
        """'pan' while the encoder push or the stick push is held (the stick
        pans, the encoder is ignored), otherwise 'orbit'."""
        if self.enc_sw == 0 or self.stick_sw == 0:
            return 'pan'
        return 'orbit'

    def check_long_press(self):
        """HOME_SWITCH held for LONG_PRESS_SEC -> home, right away (once per
        press; the release then does nothing)."""
        n = HOME_SWITCH
        t = self.press_t.get(n)
        if (self.switch[n] == 0 and t is not None and not self.combo.get(n)
                and time.perf_counter() - t >= LONG_PRESS_SEC):
            self.combo[n] = True
            log('home')
            app.fireCustomEvent(EVENT_ID, json.dumps({'action': 'home'}))

    def handle_switch(self, n, state):
        """A short press (released within LONG_PRESS_SEC) acts on release:
        SNAP_SWITCH snaps to the nearest view, FIT_SWITCH fits the model in
        view. (HOME_SWITCH's long press is handled by check_long_press().)
        Presses during which both switches were down are the recalibrate combo
        (done by the device) and do nothing here."""
        if n not in self.switch:
            return
        now = time.perf_counter()
        self.switch[n] = state
        if state == 0:
            self.press_t[n] = now
            self.combo[n] = False
        if self.switch[1] == 0 and self.switch[2] == 0:
            for b in self.switch:
                self.combo[b] = True
        if state == 0:
            return
        t = self.press_t.pop(n, None)
        if t is None or self.combo.get(n) or now - t >= LONG_PRESS_SEC:
            return
        if n == SNAP_SWITCH:
            log('snap')
            app.fireCustomEvent(EVENT_ID, json.dumps({'action': 'snap'}))
        elif n == FIT_SWITCH:
            log('fit')
            app.fireCustomEvent(EVENT_ID, json.dumps({'action': 'fit'}))

    def send_mode(self, mode='orbit'):
        """Tell the device the mode ("M:<mode>\\n"): 'orbit' on connect, ''
        when the add-in stops."""
        try:
            self.write(('M:' + mode + '\n').encode('ascii'))
        except Exception:
            pass

    def stop(self):
        self.stop_flag.set()
        if self.ser:
            try:
                self.ser.cancel_read()
            except Exception:
                pass


def normalize(dx, dy):
    """Raw offset -> (nx, ny), each -1..1. The dead zone is a circle and the
    EXPO curve is applied to the length, so the direction is kept as is."""
    r = math.hypot(dx, dy)
    if r <= DEADZONE:
        return 0.0, 0.0
    m = min(1.0, (r - DEADZONE) / (2047 - DEADZONE)) ** EXPO
    return dx / r * m, dy / r * m


class InputHandler(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()
        self.last_t = None
        self.pivot = None  # orbit center, fixed from tilt until back to center

    def notify(self, args):
        try:
            data = json.loads(args.additionalInfo)
            if data['action'] == 'home':
                vp = app.activeViewport
                if vp:
                    vp.goHome()
                return
            if data['action'] == 'fit':
                vp = app.activeViewport
                if vp:
                    vp.fit()
                    self.pivot = None
                return
            if data['action'] == 'snap':
                vp = app.activeViewport
                if vp:
                    snap_view(vp)
                    self.pivot = None
                return
            if data['action'] == 'spin':
                vp = app.activeViewport
                if vp:
                    # Same pivot as the stick's orbit
                    pivot = self.pivot or selection_center() or vp.camera.target
                    spin_z(vp, data['steps'] * SPIN_SIGN * SPIN_DEG_PER_DETENT, pivot)
                    vp.refresh()
                return
            # Only move events count for dt (others would shorten the next step)
            now = time.perf_counter()
            dt = 0.0 if self.last_t is None else min(now - self.last_t, 0.1)
            self.last_t = now
            if not reader:
                return
            reader.move_pending = False  # clear first: a newer sample fires again
            nx, ny, mode = reader.latest_move
            nx *= SIGN_X
            ny *= SIGN_Y
            if nx == 0.0 and ny == 0.0:
                self.pivot = None
                self.last_t = None  # no events at rest: don't count that time as dt
                return
            if dt == 0.0:
                return
            vp = app.activeViewport
            if not vp:
                return
            if mode == 'pan':
                pan(vp, nx, ny, dt)
            else:
                # Up/down orbits (pitch only), left/right zooms (right = in)
                if ny != 0.0:
                    if self.pivot is None:
                        self.pivot = selection_center() or vp.camera.target
                    orbit(vp, 0.0, ny, dt, self.pivot)
                if nx != 0.0:
                    zoom(vp, STICK_ZOOM_PER_SEC ** (-nx * dt * STICK_ZOOM_SIGN))
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


def selection_center():
    """Center of the selected entities' bounding boxes (model space), or None
    when nothing is selected or ORBIT_AROUND_SELECTION is off."""
    if not ORBIT_AROUND_SELECTION:
        return None
    box = None
    for sel in app.userInterface.activeSelections:
        ent = sel.entity
        bb = None
        # Sketch entities report their box in sketch space; use the picked point
        if '::Sketch' not in ent.objectType:
            try:
                bb = ent.boundingBox
            except Exception:
                pass
        if bb is None:
            bb = adsk.core.BoundingBox3D.create(sel.point, sel.point)
        if box is None:
            box = bb.copy()
        else:
            box.combine(bb)
    if box is None:
        return None
    lo, hi = box.minPoint, box.maxPoint
    return adsk.core.Point3D.create((lo.x + hi.x) / 2, (lo.y + hi.y) / 2, (lo.z + hi.z) / 2)


def orbit(vp, nx, ny, dt, pivot):
    deg = ORBIT_DEG_PER_SEC * dt
    rotate_view(vp, nx * deg, ny * deg, pivot)


def zoom(vp, factor):
    """Scale the view by factor (< 1 = zoom in) toward the camera target.
    Perspective views zoom by the eye's distance, orthographic ones by their
    extents, so both are scaled."""
    cam = vp.camera
    eye = cam.eye
    target = cam.target
    back = target.vectorTo(eye)
    back.scaleBy(factor)
    eye = target.copy()
    eye.translateBy(back)
    cam.eye = eye
    if cam.cameraType == adsk.core.CameraTypes.OrthographicCameraType:
        cam.viewExtents = cam.viewExtents * factor
    cam.isSmoothTransition = False
    vp.camera = cam


AXES = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
_C = 1 / math.sqrt(3)
SNAP_DIRS = AXES + tuple((sx * _C, sy * _C, sz * _C)
                         for sx in (1, -1) for sy in (1, -1) for sz in (1, -1))


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def snap_view(vp):
    """Turn the camera to the nearest of the 6 faces and 8 iso corners
    (SNAP_DIRS, in model space), keeping the target and the distance. Screen
    up goes to the model axis nearest the current up (projected onto the
    screen, so iso views get the usual tilted axis)."""
    cam = vp.camera
    target = cam.target
    v = cam.eye.vectorTo(target)  # viewing direction
    dist = v.length
    if dist == 0:
        return
    v = (v.x / dist, v.y / dist, v.z / dist)
    d = max(SNAP_DIRS, key=lambda c: _dot(c, v))
    cu = cam.upVector
    cu = (cu.x, cu.y, cu.z)
    up = None
    for a in AXES:
        k = _dot(a, d)
        p = (a[0] - k * d[0], a[1] - k * d[1], a[2] - k * d[2])  # a, perpendicular to d
        n = math.sqrt(_dot(p, p))
        if n < 1e-6:
            continue  # a is the viewing axis itself
        p = (p[0] / n, p[1] / n, p[2] / n)
        if up is None or _dot(p, cu) > _dot(up, cu):
            up = p
    eye = target.copy()
    eye.translateBy(adsk.core.Vector3D.create(-d[0] * dist, -d[1] * dist, -d[2] * dist))
    cam.eye = eye
    cam.upVector = adsk.core.Vector3D.create(*up)
    cam.isSmoothTransition = True
    vp.camera = cam


def spin_z(vp, deg, pivot):
    """Rotate the camera around the model's Z axis through pivot, like a
    turntable. Positive deg turns the camera counterclockwise seen from +Z,
    so the model looks like it turns clockwise."""
    cam = vp.camera
    rot = adsk.core.Matrix3D.create()
    rot.setToRotation(math.radians(deg), adsk.core.Vector3D.create(0, 0, 1), pivot)
    eye = cam.eye
    target = cam.target
    up = cam.upVector
    eye.transformBy(rot)
    target.transformBy(rot)
    up.transformBy(rot)
    cam.eye = eye
    cam.target = target
    cam.upVector = up
    cam.isSmoothTransition = False
    vp.camera = cam


def rotate_view(vp, yaw_deg, pitch_deg, pivot):
    """Rotate the camera around pivot: yaw around the screen's vertical axis
    (positive = like tilting the stick right), pitch around its horizontal
    axis. The target moves with it, so the pivot stays where it is on screen."""
    cam = vp.camera
    _, up, right = camera_axes(cam)
    yaw = adsk.core.Matrix3D.create()
    yaw.setToRotation(-math.radians(yaw_deg), up, pivot)
    pitch = adsk.core.Matrix3D.create()
    pitch.setToRotation(math.radians(pitch_deg), right, pivot)
    yaw.transformBy(pitch)
    eye = cam.eye
    target = cam.target
    eye.transformBy(yaw)
    target.transformBy(yaw)
    up.transformBy(yaw)
    cam.eye = eye
    cam.target = target
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
