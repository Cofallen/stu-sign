from pathlib import Path
import subprocess,sys,json,datetime,os,time
from zoneinfo import ZoneInfo
p=Path('/home/cofallen/Code/毕业设计/experiments/E002_original_controller');mode=sys.argv[1]
out=p/'outputs'/('realtime_gui_'+mode+'_'+datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'));out.mkdir();(out/'driver.py').write_text(Path(__file__).read_text())
command=['/home/cofallen/miniconda3/envs/mjlab/bin/python','-B',str(p/'controller.py'),'--viewer','--duration','10']
if mode=='hud':command+=['--hud']
if mode=='drive':command+=['--duration','14']
before=set((p/'outputs').iterdir());log=(out/'console.log').open('w');started=time.perf_counter();proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
if mode=='drive':
 # Existing XTest harness primitives; target only the new mapped viewer.
 import ctypes as C,ctypes.util,re
 X=C.CDLL(ctypes.util.find_library('X11'));T=C.CDLL(ctypes.util.find_library('Xtst'));X.XOpenDisplay.argtypes=[C.c_char_p];X.XOpenDisplay.restype=C.c_void_p;d=X.XOpenDisplay(None)
 if not d:raise RuntimeError('No X11 display')
 X.XDefaultRootWindow.argtypes=[C.c_void_p];X.XDefaultRootWindow.restype=C.c_ulong;root=X.XDefaultRootWindow(d)
 X.XQueryTree.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_ulong),C.POINTER(C.POINTER(C.c_ulong)),C.POINTER(C.c_uint)];X.XFetchName.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_char_p)];X.XFree.argtypes=[C.c_void_p]
 def windows(w):
  rr=C.c_ulong();pa=C.c_ulong();a=C.POINTER(C.c_ulong)();n=C.c_uint();found=[]
  if not X.XQueryTree(d,w,C.byref(rr),C.byref(pa),C.byref(a),C.byref(n)):return found
  children=[int(a[i]) for i in range(n.value)]
  if a:X.XFree(a)
  for child in children:
   name=C.c_char_p()
   if X.XFetchName(d,child,C.byref(name)) and name.value:
    title=name.value.decode(errors='replace');found.append((child,title));X.XFree(name)
   found+=windows(child)
  return found
 X.XSetInputFocus.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_ulong];X.XRaiseWindow.argtypes=[C.c_void_p,C.c_ulong];X.XFlush.argtypes=[C.c_void_p]
 X.XStringToKeysym.argtypes=[C.c_char_p];X.XStringToKeysym.restype=C.c_ulong;X.XKeysymToKeycode.argtypes=[C.c_void_p,C.c_ulong];X.XKeysymToKeycode.restype=C.c_uint;T.XTestFakeKeyEvent.argtypes=[C.c_void_p,C.c_uint,C.c_int,C.c_ulong]
 window=None
 for _ in range(80):
  if proc.poll() is not None:break
  for w,title in windows(root):
   if 'mujoco' in title.lower():
    info=subprocess.run(['xwininfo','-id',str(w)],text=True,capture_output=True)
    if info.returncode==0 and 'IsViewable' in info.stdout:window=w
  if window:break
  time.sleep(.1)
 if window is None:raise RuntimeError('No mapped window')
 time.sleep(.5);X.XRaiseWindow(d,window);X.XSetInputFocus(d,window,1,0);X.XFlush(d)
 def key(k,pressed):
  code=X.XKeysymToKeycode(d,X.XStringToKeysym(k.encode()));T.XTestFakeKeyEvent(d,code,int(pressed),0);X.XFlush(d)
 time.sleep(1.5);key('w',True);time.sleep(3);key('w',False);time.sleep(3);key('j',True);time.sleep(.12);key('j',False)
try:code=proc.wait(timeout=40)
except subprocess.TimeoutExpired:raise RuntimeError('Timed out, no success claimed')
log.close();runs=[x for x in (set((p/'outputs').iterdir())-before) if x.is_dir() and (x/'results.json').exists()]
if len(runs)!=1:raise RuntimeError('Unexpected result dirs '+str(runs))
r=json.loads((runs[0]/'results.json').read_text());(out/'results.json').write_text(json.dumps(dict(command=command,mode=mode,process_exit_code=code,run=runs[0].name,process_wall_including_startup_s=time.perf_counter()-started,result=r,near_realtime=.95<r['timing']['real_time_factor']<1.05),indent=2,ensure_ascii=False)+'\n');(out/'exit_status.json').write_text(json.dumps(dict(exit_code=code))+'\n');print(out);print(json.dumps(r['timing'],indent=2));print(r['viewer_options']);print('passed',r['passed']);sys.exit(code)
