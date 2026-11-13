"""MuJoCo 可视化：相机预设、离屏 mp4 录像、passive viewer 相机跟随。

只读 model/data 用于显示，不写 qpos/qvel/ctrl，也不参与控制或验收。
--video 需要离屏 GL：启动文件在 import mujoco 前选择后端（有 DISPLAY 用 GLFW，
否则 EGL；可用 MUJOCO_GL 覆盖）；编码调用系统 ffmpeg；HUD 用 Pillow，缺失时自动关闭。
"""
import json
import os
import shutil
import subprocess
from pathlib import Path

import mujoco

# 名称、方位角/仰角（度）、距离（米）、注视点高度（米）。
# azimuth=0 从机身 +X 看，90 从 +Y 侧看；lookat 的 x/y 默认固定在世界原点。
CAMERA_PRESETS = {
    'iso': dict(azimuth=135., elevation=-12., distance=1.25, lookat_z=.32),
    'side': dict(azimuth=90., elevation=-5., distance=1.15, lookat_z=.30),
    'front': dict(azimuth=0., elevation=-5., distance=1.15, lookat_z=.30),
    'legs': dict(azimuth=100., elevation=-2., distance=.85, lookat_z=.21),
}
FONT_CANDIDATES = (
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
)


def make_camera(model, preset):
    """返回一个按预设配置的自由相机；不修改 model。"""
    if preset not in CAMERA_PRESETS:
        raise ValueError(f'未知相机预设 {preset!r}，可选 {sorted(CAMERA_PRESETS)}')
    p = CAMERA_PRESETS[preset]
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.azimuth = p['azimuth']
    camera.elevation = p['elevation']
    camera.distance = p['distance']
    camera.lookat[:] = [0., 0., p['lookat_z']]
    return camera


def setup_viewer(viewer, model, preset='iso'):
    """配置 passive viewer：隐藏辅助几何，相机跟随机身，方便看腿与轮。"""
    preset = preset if preset in CAMERA_PRESETS else 'iso'
    p = CAMERA_PRESETS[preset]
    viewer.opt.geomgroup[3] = 0  # 隐藏辅助/调试几何，和原启动文件一致
    viewer.opt.sitegroup[:] = 0
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = int(model.body('base_Link_del').id)
    viewer.cam.azimuth = p['azimuth']
    viewer.cam.elevation = p['elevation']
    viewer.cam.distance = p['distance']
    viewer.cam.lookat[:] = [0., 0., p['lookat_z']]


class StandVideo:
    """把仿真画面按固定帧率写成 mp4；渲染只在主循环里做，不改物理状态。"""

    def __init__(self, model, path, width=960, height=540, fps=30, camera='iso',
                 hud=True, follow=False):
        if width % 2 or height % 2:
            raise ValueError('视频宽高必须是偶数（libx264 yuv420p）')
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.width, self.height, self.fps = int(width), int(height), int(fps)
        self.follow = bool(follow)
        self.saved_stem = self.path.with_suffix('')
        ffmpeg = os.environ.get('FFMPEG') or shutil.which('ffmpeg')
        if ffmpeg is None:
            raise RuntimeError('找不到 ffmpeg；无法编码 mp4。请安装 ffmpeg 或设置 FFMPEG。')
        self.camera = make_camera(model, camera)
        self.camera_params = dict(preset=camera, azimuth=float(self.camera.azimuth),
                                  elevation=float(self.camera.elevation),
                                  distance=float(self.camera.distance),
                                  lookat_z=float(self.camera.lookat[2]))
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0
        self.option.sitegroup[:] = 0
        # 仅扩大内存里的离屏帧缓冲（XML 默认 640x480），不改物理或磁盘资产。
        model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), self.width)
        model.vis.global_.offheight = max(int(model.vis.global_.offheight), self.height)
        self.renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        self.encoder = subprocess.Popen(
            [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y',
             '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{self.width}x{self.height}',
             '-r', str(self.fps), '-i', '-', '-an', '-c:v', 'libx264',
             '-pix_fmt', 'yuv420p', '-crf', '18', '-movflags', '+faststart',
             str(self.path)], stdin=subprocess.PIPE)
        self.fonts = self._load_fonts() if hud else None
        self.frames = 0
        self.last_time = None
        self.first_time = None
        self.next_time = 0.
        self.last_image = None
        self.saved_first = False
        self.closed = False

    # ---------------- HUD ----------------
    def _load_fonts(self):
        try:
            from PIL import ImageFont
        except ImportError:
            print('提示：未安装 Pillow，录像不带 HUD。', flush=True)
            return None
        for candidate in FONT_CANDIDATES:
            if Path(candidate).is_file():
                size = max(12, int(self.height * .028))
                return (ImageFont.truetype(candidate, size),
                        ImageFont.truetype(candidate, int(size * 1.1)))
        return (ImageFont.load_default(size=max(12, int(self.height * .028))),
                ImageFont.load_default(size=max(13, int(self.height * .031))))

    def _draw(self, image, info):
        from PIL import ImageDraw
        draw = ImageDraw.Draw(image)
        bar = max(48, int(self.height * .115))
        font, title = self.fonts
        top = self.height - bar  # HUD 放底部，避免遮住机身上部/云台。
        draw.rectangle((0, top, self.width, self.height), fill=(18, 18, 18))
        t = float(info.get('t', 0.))
        duration = max(1e-9, float(info.get('duration', 1.)))
        status = str(info.get('status', 'standing')).upper()
        head = (f"STANDING DEMO | {str(info.get('control', '')).upper()} | "
                f"{info.get('variant', '')} | {info.get('case', '')} | "
                f"t = {t:5.2f} / {duration:.0f} s")
        draw.text((10, top + 5), head, font=title, fill=(255, 255, 255))
        line = max(15, int(self.height * .032))
        draw.text((10, top + 5 + line * 1.15),
                  f"roll {info.get('roll', 0.):+.3f} rad   pitch {info.get('pitch', 0.):+.3f} rad"
                  f"   tilt peak {info.get('tilt_peak', 0.):.3f} rad", font=font,
                  fill=(220, 220, 220))
        draw.text((10, top + 5 + line * 2.15),
                  f"leg L0  L {info.get('length_l', 0.):.4f} m   R {info.get('length_r', 0.):.4f} m"
                  f"   base z {info.get('base_z', 0.):.3f} m   {status}", font=font,
                  fill=(120, 230, 160) if status == 'STANDING' else (255, 180, 90))
        progress = min(1., max(0., t / duration))
        draw.rectangle((0, top, int(self.width * progress), top + 3),
                       fill=(80, 200, 120) if status == 'STANDING' else (230, 140, 60))

    # ---------------- 帧 ----------------
    def _capture(self, data, info):
        if self.follow:
            self.camera.lookat[0] = float(data.qpos[0])
            self.camera.lookat[1] = float(data.qpos[1])
        self.renderer.update_scene(data, camera=self.camera, scene_option=self.option)
        from PIL import Image
        image = Image.fromarray(self.renderer.render())
        if self.fonts is not None:
            self._draw(image, info)
        self.encoder.stdin.write(image.tobytes())
        self.frames += 1
        self.last_time = float(data.time)
        self.last_image = image
        if self.first_time is None:
            self.first_time = self.last_time
        if not self.saved_first:
            image.save(self.saved_stem.with_name(self.saved_stem.name + '_t000.png'))
            self.saved_first = True
        return image

    def maybe_capture(self, data, info):
        """到达下一帧时间就录一帧；帧率按仿真时间计算，与主循环步长无关。"""
        if data.time + 1e-9 < self.next_time:
            return False
        while data.time + 1e-9 >= self.next_time:
            self.next_time += 1. / self.fps
        self._capture(data, info)
        return True

    def capture_final(self, data, info):
        """结束时补最后一帧并保存终态 PNG（时间不前进则只保存最后画面）。"""
        if self.last_time is not None and data.time <= self.last_time + 1e-9:
            image = self.last_image
        else:
            image = self._capture(data, info)
        if image is not None:
            image.save(self.saved_stem.with_name(self.saved_stem.name + '_final.png'))
        return image

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.encoder.stdin is not None:
                self.encoder.stdin.close()
            code = self.encoder.wait()
        finally:
            self.renderer.close()
        if code != 0:
            raise RuntimeError(f'ffmpeg 编码失败，退出码 {code}')
        meta = dict(path=str(self.path), width=self.width, height=self.height, fps=self.fps,
                    frames=self.frames,
                    sim_duration_s=(self.last_time - self.first_time) if self.last_time is not None else 0.,
                    encoded_duration_s=(self.frames / self.fps if self.fps else 0.),
                    camera=dict(self.camera_params), follow=self.follow, hud=self.fonts is not None,
                    codec='libx264/yuv420p', note='Rendering-only output; not used by control or acceptance.')
        self.path.with_suffix('.json').write_text(json.dumps(meta, indent=2, ensure_ascii=False) + '\n')
        print(f'视频已保存：{self.path}（{self.frames} 帧，{self.fps} fps）', flush=True)
