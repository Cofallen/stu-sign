from pathlib import Path
import ctypes as C,ctypes.util,time,subprocess,os,json,datetime
from zoneinfo import ZoneInfo
from PIL import ImageGrab
root=Path('/home/cofallen/Code/毕业设计/experiments/E002_original_controller');out=root/'outputs'/('real_mouse_'+datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
X=C.CDLL(ctypes.util.find_library('X11'));T=C.CDLL(ctypes.util.find_library('Xtst'))
X.XOpenDisplay.argtypes=[C.c_char_p];X.XOpenDisplay.restype=C.c_void_p;display=X.XOpenDisplay(None)
if not display:raise RuntimeError("Cannot open X display; run GUI check at desktop execution boundary")
X.XDefaultRootWindow.argtypes=[C.c_void_p];X.XDefaultRootWindow.restype=C.c_ulong;winroot=X.XDefaultRootWindow(display)
X.XQueryTree.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_ulong),C.POINTER(C.POINTER(C.c_ulong)),C.POINTER(C.c_uint)]
X.XFetchName.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_char_p)]
X.XFree.argtypes=[C.c_void_p];X.XSetInputFocus.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_ulong];X.XRaiseWindow.argtypes=[C.c_void_p,C.c_ulong];X.XFlush.argtypes=[C.c_void_p]
X.XStringToKeysym.argtypes=[C.c_char_p];X.XStringToKeysym.restype=C.c_ulong;X.XKeysymToKeycode.argtypes=[C.c_void_p,C.c_ulong];X.XKeysymToKeycode.restype=C.c_uint
T.XTestFakeKeyEvent.argtypes=[C.c_void_p,C.c_uint,C.c_int,C.c_ulong]
def windows(w):
 parent=C.c_ulong();rr=C.c_ulong();children=C.POINTER(C.c_ulong)();n=C.c_uint()
 if not X.XQueryTree(display,w,C.byref(rr),C.byref(parent),C.byref(children),C.byref(n)):return []
 result=[]
 for i in range(n.value):
  child=children[i];name=C.c_char_p()
  if X.XFetchName(display,child,C.byref(name)) and name.value:
   title=name.value.decode(errors='replace');result.append((int(child),title));X.XFree(name)
  result+=windows(child)
 if children:X.XFree(children)
 return result
before=set(w for w,title in windows(winroot))
cmd=['/home/cofallen/miniconda3/envs/mjlab/bin/python','-B',str(root/'controller.py')]
log=(out/'console.log').open('w');child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
window=None
for _ in range(100):
 for w,title in windows(winroot):
  if 'mujoco' in title.lower() and w not in before:window=w;break
 if window:break
 if child.poll() is not None:raise RuntimeError('Viewer exited: '+(out/'console.log').read_text())
 time.sleep(.2)
if not window:raise RuntimeError('No viewer window found')
X.XRaiseWindow(display,window);X.XSetInputFocus(display,window,1,0);X.XFlush(display)
info=subprocess.check_output(['xwininfo','-id',str(window)],text=True)
import re
x=int(re.search(r'Absolute upper-left X:\s+(-?\d+)',info).group(1));y=int(re.search(r'Absolute upper-left Y:\s+(-?\d+)',info).group(1))
w=int(re.search(r'Width:\s+(\d+)',info).group(1));h=int(re.search(r'Height:\s+(\d+)',info).group(1))
def screenshot(path):ImageGrab.grab(bbox=(x,y,x+w,y+h)).save(path)
events=[]
def press(key):
 kc=X.XKeysymToKeycode(display,X.XStringToKeysym(key.encode()));T.XTestFakeKeyEvent(display,kc,1,0);T.XTestFakeKeyEvent(display,kc,0,0);X.XFlush(display);time.sleep(.10);events.append(dict(wall=time.monotonic(),key=key))
T.XTestFakeMotionEvent.argtypes=[C.c_void_p,C.c_int,C.c_int,C.c_int,C.c_ulong]
T.XTestFakeButtonEvent.argtypes=[C.c_void_p,C.c_uint,C.c_int,C.c_ulong]
time.sleep(2);screenshot(out/'mouse_before.png')
px,py=x+int(w*.60),y+int(h*.53)
T.XTestFakeMotionEvent(display,0,px,py,0);X.XFlush(display)
for _ in range(2):
 T.XTestFakeButtonEvent(display,1,1,0);T.XTestFakeButtonEvent(display,1,0,0);X.XFlush(display);time.sleep(.10)
time.sleep(.3)
control=X.XKeysymToKeycode(display,X.XStringToKeysym(b'Control_L'))
T.XTestFakeKeyEvent(display,control,1,0);T.XTestFakeButtonEvent(display,3,1,0);X.XFlush(display);time.sleep(.2)
T.XTestFakeMotionEvent(display,0,px+15,py-8,0);X.XFlush(display);time.sleep(1.)
screenshot(out/'mouse_drag.png')
T.XTestFakeButtonEvent(display,3,0,0);T.XTestFakeKeyEvent(display,control,0,0);X.XFlush(display)
time.sleep(3);screenshot(out/'mouse_after.png')
# Close our own focused default interactive window with the standard WM gesture.
ctrl=X.XKeysymToKeycode(display,X.XStringToKeysym(b'Alt_L'));f4=X.XKeysymToKeycode(display,X.XStringToKeysym(b'F4'))
T.XTestFakeKeyEvent(display,ctrl,1,0);T.XTestFakeKeyEvent(display,f4,1,0);T.XTestFakeKeyEvent(display,f4,0,0);T.XTestFakeKeyEvent(display,ctrl,0,0);X.XFlush(display)
try:code=child.wait(timeout=15)
except subprocess.TimeoutExpired:
 child.terminate();code=child.wait();raise RuntimeError('Viewer test did not complete in bounded time')
log.close();(out/'metadata.json').write_text(json.dumps(dict(command=cmd,window=window,events=events,exit_code=code,scope='Actual XTest key press/release to actual GLFW window; screenshots are entire desktop, not synthetic'),indent=2)+'\n');print(out);print('exit',code)
