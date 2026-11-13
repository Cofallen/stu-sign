"""Record actual passive MuJoCo states using existing Mesa software EGL."""
import argparse
import datetime
import json
import os
import subprocess

# Process-local rendering settings; leave host graphics configuration untouched.
os.environ['MUJOCO_GL'] = 'egl'
os.environ['EGL_PLATFORM'] = 'surfaceless'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = '/usr/share/glvnd/egl_vendor.d/50_mesa.json'

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from audit_model import baseline
from prepare_model import ROOT, OUTPUT, digest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=5.)
    args = parser.parse_args()
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    output = ROOT / 'runs/m01_model' / f'model_video_{stamp}'
    output.mkdir(parents=True)
    model = mujoco.MjModel.from_xml_path(str(OUTPUT / 'robot.xml'))
    data = baseline(model)
    data.qpos[2] = 0
    mujoco.mj_forward(model, data)
    contacts = [c for c in data.contact if any('wheel' in model.body(model.geom_bodyid[g]).name for g in c.geom)]
    data.qpos[2] = -min(c.dist for c in contacts) + .001
    mujoco.mj_forward(model, data)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, .2]
    cam.distance = 1.5
    cam.azimuth = 135
    cam.elevation = -15
    trajectory = []
    times = []
    options = mujoco.MjvOption()
    options.geomgroup[2] = 1  # Source visual triangles.
    options.geomgroup[3] = 0  # Collision envelopes are diagnostics, not appearance.
    encoder = subprocess.Popen(['/usr/bin/ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '640x480', '-r', '25', '-i', '-',
        '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', str(output / 'passive_contact.mp4')], stdin=subprocess.PIPE)
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        for frame_index in range(round(args.seconds * 25) + 1):
            while data.time + 1e-9 < frame_index / 25:
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            times.append(data.time); trajectory.append(data.qpos.copy())
            renderer.update_scene(data, camera=cam, scene_option=options)
            picture = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(picture)
            draw.rectangle([0, 0, 640, 43], fill='black')
            draw.text((10, 6), f'Fudan V4 simplified | passive physics | t={data.time:.2f} s', fill='white')
            draw.text((10, 24), 'Original visual mesh | upstream MJCF white/gray palette | no policy', fill='white')
            if frame_index == 0:
                picture.save(output / 'initial.png')
            if frame_index == round(args.seconds * 25):
                picture.save(output / 'final.png')
            encoder.stdin.write(np.array(picture).tobytes())
    encoder.stdin.close()
    if encoder.wait() != 0:
        raise RuntimeError('Existing ffmpeg failed to encode video')
    np.savez_compressed(output / 'states.npz', time=times, qpos=trajectory)
    (output / 'metadata.json').write_text(json.dumps(dict(
        mujoco=mujoco.__version__, xml_sha256=digest(OUTPUT/'robot.xml'), fps=25,
        renderer='Mesa software EGL; MuJoCo Renderer', policy=None,
        appearance='source visual mesh triangles; user-selected upstream MJCF palette; collision group hidden',
        task='passive contact diagnostics; not a trained controller'), indent=2))
    print(output)


if __name__ == '__main__':
    main()
