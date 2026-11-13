"""X11 实际键位查询：补足 passive viewer 没有 key-release 回调的问题。

只在本进程 MuJoCo 窗口有焦点且鼠标位于三维区时启用驾驶。
W/S/A/D/Q/E 查询真实按住状态；J/X/R 查询按下边沿（100Hz）。
不安装监听库，不用按键重复事件猜测释放，也不读取/记录其它程序的文本。
"""
import ctypes as C
import ctypes.util
import threading
import os


class KeyboardInput:
    def __init__(self):
        library = ctypes.util.find_library('X11')
        if not library:
            raise RuntimeError('按住控制需要本机 X11 库；此后端尚未验证原生 Wayland/Windows。')
        self.x11 = C.CDLL(library)
        self.x11.XInitThreads()
        self.x11.XOpenDisplay.argtypes = [C.c_char_p]
        self.x11.XOpenDisplay.restype = C.c_void_p
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('无法连接桌面 X11；请在图形终端运行，数值检查用 --headless。')
        self.x11.XGetInputFocus.argtypes = [C.c_void_p, C.POINTER(C.c_ulong), C.POINTER(C.c_int)]
        self.x11.XQueryKeymap.argtypes = [C.c_void_p, C.c_void_p]
        self.x11.XStringToKeysym.argtypes = [C.c_char_p]
        self.x11.XStringToKeysym.restype = C.c_ulong
        self.x11.XKeysymToKeycode.argtypes = [C.c_void_p, C.c_ulong]
        self.x11.XKeysymToKeycode.restype = C.c_uint
        self.x11.XFetchName.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_char_p)]
        self.x11.XFree.argtypes = [C.c_void_p]
        self.x11.XCloseDisplay.argtypes = [C.c_void_p]
        self.keycodes = {key: self.x11.XKeysymToKeycode(self.display, self.x11.XStringToKeysym(key.encode()))
                         for key in 'wsadqejxr'}
        # 被动抓取仅作用于本进程窗口中的驾驶键，避免 W/J 等同时切换显示。
        signatures = {
            'XInternAtom': ([C.c_void_p, C.c_char_p, C.c_int], C.c_ulong),
            'XGetWindowProperty': ([C.c_void_p, C.c_ulong, C.c_ulong, C.c_long, C.c_long,
                                   C.c_int, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_int),
                                   C.POINTER(C.c_ulong), C.POINTER(C.c_ulong), C.POINTER(C.c_void_p)], C.c_int),
            'XGrabKey': ([C.c_void_p, C.c_int, C.c_uint, C.c_ulong, C.c_int, C.c_int, C.c_int], C.c_int),
            'XUngrabKey': ([C.c_void_p, C.c_int, C.c_uint, C.c_ulong], C.c_int),
            'XQueryPointer': ([C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_ulong),
                               C.POINTER(C.c_int), C.POINTER(C.c_int), C.POINTER(C.c_int),
                               C.POINTER(C.c_int), C.POINTER(C.c_uint)], C.c_int),
            'XGetGeometry': ([C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong), C.POINTER(C.c_int),
                              C.POINTER(C.c_int), C.POINTER(C.c_uint), C.POINTER(C.c_uint),
                              C.POINTER(C.c_uint), C.POINTER(C.c_uint)], C.c_int),
            'XPending': ([C.c_void_p], C.c_int),
            'XNextEvent': ([C.c_void_p, C.c_void_p], C.c_int),
            'XFlush': ([C.c_void_p], C.c_int),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(self.x11, name)
            function.argtypes, function.restype = arguments, result
        self.pid_atom = self.x11.XInternAtom(self.display, b'_NET_WM_PID', 0)
        self.window = None
        self.grabbed = False
        self.viewport = None
        self.previous = set()
        self.actions = []
        self.lock = threading.Lock()

    def _focus(self):
        window, revert = C.c_ulong(), C.c_int()
        self.x11.XGetInputFocus(self.display, C.byref(window), C.byref(revert))
        return window.value

    def set_viewport(self, viewport):
        """MuJoCo 公共 viewport；鼠标在侧面板内时释放驾驶键给原生 UI。"""
        self.viewport = (viewport.left, viewport.bottom, viewport.width, viewport.height)

    def _own_window(self, window):
        if window <= 1:
            return False
        kind, bits, count, remaining, value = C.c_ulong(), C.c_int(), C.c_ulong(), C.c_ulong(), C.c_void_p()
        self.x11.XGetWindowProperty(self.display, window, self.pid_atom, 0, 1, 0, 6,
                                   C.byref(kind), C.byref(bits), C.byref(count), C.byref(remaining), C.byref(value))
        own = bool(value.value and count.value == 1 and bits.value == 32
                   and C.cast(value, C.POINTER(C.c_ulong))[0] == os.getpid())
        if value.value:
            self.x11.XFree(value)
        return own

    def _in_viewport(self):
        if self.viewport is None:
            return False
        root, child = C.c_ulong(), C.c_ulong()
        rx, ry, x, y, mask = C.c_int(), C.c_int(), C.c_int(), C.c_int(), C.c_uint()
        if not self.x11.XQueryPointer(self.display, self.window, C.byref(root), C.byref(child),
                                      C.byref(rx), C.byref(ry), C.byref(x), C.byref(y), C.byref(mask)):
            return False
        gx, gy = C.c_int(), C.c_int()
        width, height, border, depth = C.c_uint(), C.c_uint(), C.c_uint(), C.c_uint()
        self.x11.XGetGeometry(self.display, self.window, C.byref(root), C.byref(gx), C.byref(gy),
                             C.byref(width), C.byref(height), C.byref(border), C.byref(depth))
        left, bottom, w, h = self.viewport
        return left <= x.value < left+w and bottom <= height.value-1-y.value < bottom+h

    def _grab(self, enabled):
        if enabled == self.grabbed:
            return
        for code in self.keycodes.values():
            for modifiers in (0, 2, 16, 18):  # 普通键、CapsLock、NumLock；保留 Ctrl/Alt 组合。
                if enabled:
                    self.x11.XGrabKey(self.display, code, modifiers, self.window, 0, 1, 1)
                else:
                    self.x11.XUngrabKey(self.display, code, modifiers, self.window)
        self.x11.XFlush(self.display)
        self.grabbed = enabled

    def on_key(self, key):
        """兼容入口；驾驶由 poll 处理，原生面板中的键不转换成驾驶动作。"""
        pass

    def poll(self):
        """返回 (当前按住的键集合, 单次动作列表)；失焦立即返回空集合。"""
        with self.lock:
            if not self.display:
                return set(), []
            focus = self._focus()
            if self.window is None and self._own_window(focus):
                self.window = focus
            enabled = self.window is not None and focus == self.window and self._in_viewport()
            self._grab(enabled)
            # 抓取的事件归本连接；连续轴和动作边沿仍以真实键位为准，不靠重复事件。
            event = (C.c_long * 24)()
            while self.x11.XPending(self.display):
                self.x11.XNextEvent(self.display, event)
            if not enabled:
                self.actions.clear()
                self.previous.clear()
                return set(), []
            bitmap = (C.c_ubyte * 32)()
            self.x11.XQueryKeymap(self.display, bitmap)
            pressed = {key for key, code in self.keycodes.items() if bitmap[code//8] & (1 << (code%8))}
            actions = sorted((pressed-self.previous) & set('jxr'))
            self.previous = pressed
            self.actions.clear()
            return pressed & set('wsadqe'), actions

    def close(self):
        with self.lock:
            if self.display:
                self._grab(False)
                self.x11.XCloseDisplay(self.display)
                self.display = None
