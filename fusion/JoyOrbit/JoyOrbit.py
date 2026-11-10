# JoyOrbit: Fusion add-in that orbits/pans the active viewport from an
# XIAO ESP32C3 running this repo's firmware (x,y,sw CSV, 115200bps, USB serial).
# Without USB it connects over BLE instead (Nordic UART Service, same lines).
#
#   stick              -> move the view in the current mode
#                         (orbit pivots on the selection's center, if any)
#   sw short press     -> toggle mode: orbit <-> pan (on release)
#   sw long press      -> home view (fires once while still held)
#                         (sw = encoder push; the view doesn't move while it's held)
#   rotary encoder     -> zoom (clockwise = in), from the "E:<detents>" lines
#   left / right button -> fit / home view (on press; see BUTTON_ACTIONS),
#                         from the "B:<n>,<state>" lines
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
DEADZONE = 0             # raw counts from center (radius, 0-4095 scale) that are ignored
EXPO = 1.5                # response curve exponent: 1 = linear, larger = finer near center
ORBIT_DEG_PER_SEC = 90.0    # orbit speed at full deflection
ORBIT_AROUND_SELECTION = True  # orbit around the selection's center when something is selected
MAX_UPDATE_HZ = 30          # view updates per second at most (more makes the mouse sluggish)
PAN_PER_SEC = 0.3          # pan speed at full deflection, x eye-target distance
SIGN_X = 1                  # flip to -1 if left/right is reversed
SIGN_Y = -1                 # flip to -1 if up/down is reversed
DEBOUNCE_SEC = 0.3          # ignore sw pushes closer together than this
LONG_PRESS_SEC = 0.8        # hold sw this long to go to the home view
ZOOM_PER_DETENT = 1.1       # zoom factor per encoder click
ZOOM_SIGN = 1               # flip to -1 if clockwise zooms out
BUTTON_ACTIONS = {1: 'fit', 2: 'home'}  # 1 = left, 2 = right -> 'fit' or 'home'
# -----------------------------------------------------------------------------

EVENT_ID = 'JoyOrbitInput'
MODES = ('orbit', 'pan')
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
        self.mode = 0
        self.prev_sw = 1
        self.last_push = 0.0
        self.press_t = None       # when the current (debounced) press started
        self.long_fired = False   # home already sent for the current press
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
            if steps:
                app.fireCustomEvent(EVENT_ID, json.dumps({'action': 'zoom', 'steps': steps}))
            return
        if line.startswith('B:'):
            # "B:<n>,<state>" (state 0 = pressed; the device debounces)
            try:
                n, state = (int(v) for v in line[2:].split(','))
            except ValueError:
                return
            action = BUTTON_ACTIONS.get(n)
            if state == 0 and action:
                log(action)
                app.fireCustomEvent(EVENT_ID, json.dumps({'action': action}))
            return
        try:
            x, y, sw = (int(v) for v in line.split(','))
        except ValueError:
            return
        self.handle_sw(sw)
        if sw == 0:
            return  # don't move the view while sw (encoder push) is held
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
        self.latest_move = (nx, ny, MODES[self.mode])
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
        """Tell the device the current mode ("M:<mode>\\n")."""
        if mode is None:
            mode = MODES[self.mode]
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
                return
            if data['action'] == 'zoom':
                vp = app.activeViewport
                if vp:
                    zoom(vp, data['steps'])
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
                if self.pivot is None:
                    self.pivot = selection_center() or vp.camera.target
                orbit(vp, nx, ny, dt, self.pivot)
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
    """Rotate the camera around pivot (the target moves with it, so the pivot
    stays where it is on screen)."""
    cam = vp.camera
    _, up, right = camera_axes(cam)
    yaw = adsk.core.Matrix3D.create()
    yaw.setToRotation(-nx * ORBIT_DEG_PER_SEC * dt * 3.14159265 / 180, up, pivot)
    pitch = adsk.core.Matrix3D.create()
    pitch.setToRotation(ny * ORBIT_DEG_PER_SEC * dt * 3.14159265 / 180, right, pivot)
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


def zoom(vp, steps):
    """Zoom in by ZOOM_PER_DETENT per step (negative steps zoom out) toward
    the target. viewExtents covers orthographic; in perspective the eye is
    moved too, so the two stay consistent."""
    cam = vp.camera
    k = ZOOM_PER_DETENT ** (-steps * ZOOM_SIGN)
    cam.viewExtents = cam.viewExtents * k
    if cam.cameraType == adsk.core.CameraTypes.PerspectiveCameraType:
        back = cam.target.vectorTo(cam.eye)
        back.scaleBy(k)
        eye = cam.target.copy()
        eye.translateBy(back)
        cam.eye = eye
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
