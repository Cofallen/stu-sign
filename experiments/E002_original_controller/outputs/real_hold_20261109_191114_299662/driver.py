from pathlib import Path
import ctypes as C,ctypes.util,time,subprocess,os,json,datetime,re
from zoneinfo import ZoneInfo
from PIL import ImageGrab
p=Path('/home/cofallen/Code/毕业设计/experiments/E002_original_controller');out=p/'outputs'/('real_hold_'+datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'));out.mkdir();(out/'driver.py').write_text(Path(__file__).read_text())
X=C.CDLL(ctypes.util.find_library('X11'));T=C.CDLL(ctypes.util.find_library('Xtst'))
X.XOpenDisplay.argtypes=[C.c_char_p];X.XOpenDisplay.restype=C.c_void_p;d=X.XOpenDisplay(None)
if not d:raise RuntimeError('Desktop X11 unavailable')
X.XDefaultRootWindow.argtypes=[C.c_void_p];X.XDefaultRootWindow.restype=C.c_ulong;root=X.XDefaultRootWindow(d)
X.XQueryTree.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_ulong),C.POINTER(C.POINTER(C.c_ulong)),C.POINTER(C.c_uint)]
X.XFetchName.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_char_p)];X.XFree.argtypes=[C.c_void_p]
X.XSetInputFocus.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_ulong];X.XRaiseWindow.argtypes=[C.c_void_p,C.c_ulong];X.XFlush.argtypes=[C.c_void_p]
X.XStringToKeysym.argtypes=[C.c_char_p];X.XStringToKeysym.restype=C.c_ulong;X.XKeysymToKeycode.argtypes=[C.c_void_p,C.c_ulong];X.XKeysymToKeycode.restype=C.c_uint
T.XTestFakeKeyEvent.argtypes=[C.c_void_p,C.c_uint,C.c_int,C.c_ulong]
def windows(w):
 rr=C.c_ulong();parent=C.c_ulong();a=C.POINTER(C.c_ulong)();n=C.c_uint();found=[]
 if not X.XQueryTree(d,w,C.byref(rr),C.byref(parent),C.byref(a),C.byref(n)):return found
 children=[int(a[i]) for i in range(n.value)]
 if a:X.XFree(a)
 for child in children:
  name=C.c_char_p()
  if X.XFetchName(d,child,C.byref(name)) and name.value:
   found.append((child,name.value.decode(errors='replace')));X.XFree(name)
  found+=windows(child)
 return found
before={w for w,title in windows(root)};command=['/home/cofallen/miniconda3/envs/mjlab/bin/python','-B',str(p/'controller.py')]
log=(out/'console.log').open('w');proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy());window=None
for _ in range(120):
 for w,title in windows(root):
  if w not in before and 'mujoco' in title.lower():
   info=subprocess.run(['xwininfo','-id',str(w)],text=True,capture_output=True)
   if info.returncode==0 and 'IsViewable' in info.stdout:window=w;break
 if window:break
 if proc.poll() is not None:raise RuntimeError('Application exited before window opened: '+(out/'console.log').read_text())
 time.sleep(.2)
if not window:raise RuntimeError('No mapped MuJoCo window')
time.sleep(.7);X.XRaiseWindow(d,window);X.XSetInputFocus(d,window,1,0);X.XFlush(d)
info=subprocess.check_output(['xwininfo','-id',str(window)],text=True)
x=int(re.search(r'Absolute upper-left X:\s+(-?\d+)',info).group(1));y=int(re.search(r'Absolute upper-left Y:\s+(-?\d+)',info).group(1));width=int(re.search(r'Width:\s+(\d+)',info).group(1));height=int(re.search(r'Height:\s+(\d+)',info).group(1))
events=[]
def key(k,down):
 code=X.XKeysymToKeycode(d,X.XStringToKeysym(k.encode()));T.XTestFakeKeyEvent(d,code,int(down),0);X.XFlush(d);events.append(dict(wall=time.monotonic(),key=k,down=down))
def tap(k):key(k,True);time.sleep(.12);key(k,False)
def shot(name):ImageGrab.grab(bbox=(x,y,x+width,y+height)).save(out/(name+'.png'))
time.sleep(2);shot('initial');key('w',True);time.sleep(5);shot('held_w');key('w',False);time.sleep(4);shot('released_w')
key('s',True);time.sleep(3);key('s',False);time.sleep(4)
key('q',True);time.sleep(.8);key('q',False);time.sleep(2);shot('length_held_after_release')
key('e',True);time.sleep(.8);key('e',False);time.sleep(2);tap('r');time.sleep(3)
tap('j');time.sleep(1.5);shot('jump');time.sleep(4)
# Proper close of our own mapped application window.
key('Alt_L',True);key('F4',True);key('F4',False);key('Alt_L',False)
try:exit_code=proc.wait(timeout=20)
except subprocess.TimeoutExpired:raise RuntimeError('Application did not shut down')
log.close();(out/'metadata.json').write_text(json.dumps(dict(command=command,window=window,events=events,exit_code=exit_code,scope='Actual mapped GLFW window; XTest key down and key up, not callback-only'),indent=2)+'\n');print(out);print('exit_code',exit_code)
