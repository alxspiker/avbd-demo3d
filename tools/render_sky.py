"""Render avbd3d_sky_capture JSON to MP4. Needs pillow and ffmpeg in PATH."""
import json
import math
import subprocess
import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

if len(sys.argv) != 3:
    raise SystemExit('Usage: python tools/render_sky.py sky_trace.json sky_drop.mp4')
source, dest = map(Path, sys.argv[1:])
data = json.loads(source.read_text())
W, H = 800, 450
font = ImageFont.load_default()
faces = [[0, 2, 6, 4], [1, 5, 7, 3], [0, 4, 5, 1], [2, 3, 7, 6], [0, 1, 3, 2], [4, 6, 7, 5]]
colors = ['#43576b', '#59b5db', '#7199e9', '#ac88e0', '#efa85d', '#80cba6']
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def sub(a, b): return [x-y for x, y in zip(a, b)]
def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def norm(a):
    n = math.sqrt(dot(a, a)) or 1
    return [x/n for x in a]
def rotate(v, q):
    w, x, y, z = q
    u = [x, y, z]
    t = [2*a for a in cross(u, v)]
    v2 = cross(u, t)
    return [v[i]+w*t[i]+v2[i] for i in range(3)]
proc = subprocess.Popen([
    'ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
    '-s', f'{W}x{H}', '-r', '30', '-i', '-', '-c:v', 'libx264', '-preset', 'fast',
    '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(dest)
], stdin=subprocess.PIPE)
try:
    for idx, state in enumerate(data['frames']):
        im = Image.new('RGB', (W, H), '#101927')
        draw = ImageDraw.Draw(im)
        yaw, pitch, dist, center = .58, .34, 34, [0, 4, 0]
        cam = [math.sin(yaw)*math.cos(pitch)*dist, math.sin(pitch)*dist+center[1], math.cos(yaw)*math.cos(pitch)*dist]
        forward = norm(sub(center, cam))
        right = norm(cross(forward, [0, 1, 0]))
        up = cross(right, forward)
        def proj(v):
            a = sub(v, cam)
            z = dot(a, forward)
            return (W/2+dot(a, right)*700/z, H*.50-dot(a, up)*700/z, z)
        for i in range(-12, 13):
            for a, b in [([-12, 0, i], [12, 0, i]), ([i, 0, -12], [i, 0, 12])]:
                a, b = proj(a), proj(b)
                draw.line([(a[0], a[1]), (b[0], b[1])], fill='#32465a' if i else '#56768b', width=1)
        visible = []
        for bi, body in enumerate(state):
            if bi == 0: continue
            size = data['boxes'][bi]['size']
            pos, q = body[:3], body[3:]
            verts = []
            world_verts = []
            for x in [-1, 1]:
                for y in [-1, 1]:
                    for z in [-1, 1]:
                        v = rotate([x*size[0]/2, y*size[1]/2, z*size[2]/2], q)
                        wp = [pos[j]+v[j] for j in range(3)]
                        world_verts.append(wp)
                        verts.append(proj(wp))
            for face in faces:
                p = [verts[j] for j in face]
                wp = [world_verts[j] for j in face]
                outward = cross(sub(wp[1], wp[0]), sub(wp[2], wp[0]))
                if dot(outward, sub(cam, wp[0])) <= 0: continue
                visible.append((sum(v[2] for v in p)/4, bi, [(v[0], v[1]) for v in p]))
        visible.sort(reverse=True)
        for _, bi, p in visible:
            draw.polygon(p, fill=colors[1+(bi-1)%5])
            draw.line(p+[p[0]], fill='#192b42', width=2)
        draw.text((24, 22), 'AVBD 3D / RECORDED C++ PHYSICS', fill='#e6effa', font=font)
        draw.text((24, 50), f'{idx*data["dt"]:.2f}s / {idx+1} of {len(data["frames"])} frames', fill='#adc3d7', font=font)
        proc.stdin.write(im.tobytes())
finally:
    proc.stdin.close()
if proc.wait() != 0:
    raise SystemExit('ffmpeg failed')
print(dest)
