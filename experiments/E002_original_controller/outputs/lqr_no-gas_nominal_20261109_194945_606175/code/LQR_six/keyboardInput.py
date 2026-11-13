"""X11 实际键位查询：补足 passive viewer 没有 key-release 回调的问题。

只在自有 MuJoCo 窗口有焦点时读取 W/S/A/D/Q/E；J/X/R 从按下回调排队。
不安装监听库，不用按键重复事件猜测释放，也不读取/记录其它程序的文本。
"""
import ctypes as C
import ctypes.util
import threading


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
                         for key in 'wsadqe'}
        self.window = None
        self.actions = []
        self.lock = threading.Lock()

    def _focus(self):
        window, revert = C.c_ulong(), C.c_int()
        self.x11.XGetInputFocus(self.display, C.byref(window), C.byref(revert))
        return window.value

    def on_key(self, key):
        """仅由自有 viewer 调用；绑定焦点窗口，J/X/R 保存单次按下事件。"""
        with self.lock:
            if not self.display:
                return
            focus = self._focus()
            if focus <= 1:
                return
            name = C.c_char_p()
            if not self.x11.XFetchName(self.display, focus, C.byref(name)):
                return
            valid = bool(name.value and b'mujoco' in name.value.lower())
            self.x11.XFree(name)
            if not valid:
                return
            self.window = focus
            ch = chr(key).lower() if 32 <= key <= 126 else ''
            if ch and ch in 'jxr':
                self.actions.append(ch)

    def poll(self):
        """返回 (当前按住的键集合, 单次动作列表)；失焦立即返回空集合。"""
        with self.lock:
            if not self.display or self.window is None or self._focus() != self.window:
                self.actions.clear()
                return set(), []
            bitmap = (C.c_ubyte * 32)()
            self.x11.XQueryKeymap(self.display, bitmap)
            pressed = {key for key, code in self.keycodes.items() if bitmap[code//8] & (1 << (code%8))}
            actions, self.actions = self.actions, []
            return pressed, actions

    def close(self):
        with self.lock:
            if self.display:
                self.x11.XCloseDisplay(self.display)
                self.display = None
