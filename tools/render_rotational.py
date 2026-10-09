"""Render real two-world rotating impact traces into an annotated side-by-side MP4.
The 6x pose replay is explicitly labeled; no body trajectory is interpolated or invented.
Requires Pillow + ffmpeg. Input: avbd3d_rotational_capture stdout JSON.
"""
import json, math, subprocess, sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
if len(sys.argv) != 3:
    raise SystemExit('Usage: python tools/render_rotational.py rotational_trace.json stage8-reviewed.mp4')
source, dest = map(Path, sys.argv[1:])
data = json.loads(source.read_text())
assert data['stage']==8
W,H=1200,600
FPS=data['display_fps']
fontfile='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
font=ImageFont.truetype(fontfile,16)
small=ImageFont.truetype(fontfile,14)
large=ImageFont.truetype(fontfile,25)
faces=[(0,4,6,2),(1,3,7,5),(0,1,5,4),(2,6,7,3),(0,2,3,1),(4,5,7,6)]
verts=[(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def sub(a,b):return [a[i]-b[i] for i in range(3)]
def add(a,b):return [a[i]+b[i] for i in range(3)]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def norm(a):return [v/max(1e-12,math.sqrt(dot(a,a))) for v in a]
def rot(p,q):
    w,x,y,z=q
    v=[x,y,z];t=[2*c for c in cross(v,p)];u=cross(v,t)
    return [p[i]+w*t[i]+u[i] for i in range(3)]
cam=[0,0,12];look=[0,0,0];forward=norm(sub(look,cam));right=norm(cross(forward,[0,1,0]));up=cross(right,forward)
def project(p,side):
    rel=sub(p,cam);z=max(.1,dot(rel,forward))
    return (side*600+300+dot(rel,right)*930/z,330-dot(rel,up)*930/z,z)
def draw_scene(draw,body,side,hit,initial,prev):
    x0=side*600
    # XY diagnostic grid. Zero gravity; objects occupy an XY plane in 3D.
    for k in range(-6,7):
        for a,b in [([k,-5,0],[k,5,0]),([-6,k,0],[6,k,0])]:
            pa,pb=project(a,side),project(b,side)
            draw.line([pa[:2],pb[:2]],fill='#243e54',width=1)
    # Actual 3D rotating solid cuboid, with outward-face visibility and lighting.
    p,q=body[0][:3],body[0][3:]
    size=data['boxes'][0]['size']
    world=[add(p,rot([v[j]*size[j]*.5 for j in range(3)],q)) for v in verts]
    shown=[]
    for face in faces:
        pts=[world[j] for j in face]
        inward=cross(sub(pts[1],pts[0]),sub(pts[2],pts[0]))
        outward=norm([-v for v in inward])
        if dot(outward,sub(cam,pts[0]))<=0:continue
        illumination=.60+.35*max(0,dot(outward,norm([-.35,.65,1])))
        color=tuple(min(255,int(c*illumination)) for c in (126,160,234))
        screen=[project(v,side)[:2] for v in pts]
        shown.append((sum(project(v,side)[2] for v in pts)/4,screen,color))
    shown.sort(reverse=True)
    for _,polygon,color in shown:
        draw.polygon(polygon,fill=color)
        draw.line(polygon+[polygon[0]],fill='#132137',width=2)
    # Dot-ring is the sphere's original start position, always grounded in capture.
    start=project([*initial,0],side)
    draw.ellipse((start[0]-11,start[1]-11,start[0]+11,start[1]+11),outline='#75899a',width=2)
    curr=project(body[1][:3],side)
    if math.dist(start[:2],curr[:2])>2:
        draw.line([start[:2],curr[:2]],fill='#ffdc91' if hit else '#f1a7a0',width=3)
    r=data['boxes'][1]['size'][0]*.5*930/curr[2]
    fill='#f9cf64' if hit else '#fa9a8c'
    draw.ellipse((curr[0]-r,curr[1]-r,curr[0]+r,curr[1]+r),fill=fill,outline='#192233',width=3)
    draw.ellipse((curr[0]-r*.55,curr[1]-r*.7,curr[0]+r*.25,curr[1]+r*.10),fill='#ffeeb6' if hit else '#fcd9bc')
    draw.text((x0+20,H-90),'Outlined circle = starting location',font=small,fill='#9fb1c6')
    draw.text((x0+20,H-68),'Line = measured sphere displacement',font=small,fill='#9fb1c6')

cmd=['ffmpeg','-y','-nostats','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(FPS),'-i','-','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(dest)]
proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
frames=data['frames'];total=len(frames)
try:
    for idx,f in enumerate(frames):
        im=Image.new('RGB',(W,H),'#111c2b');draw=ImageDraw.Draw(im)
        draw_scene(draw,f['off'],0,False,f['target'],None)
        draw_scene(draw,f['on'],1,True,f['target'],None)
        draw.line((600,0,600,H),fill='#40607c',width=3)
        draw.rounded_rectangle((14,14,579,126),radius=10,fill='#1d2c3e',outline='#ab6565',width=2)
        draw.rounded_rectangle((620,14,1186,126),radius=10,fill='#1d2c3e',outline='#328f80',width=2)
        draw.text((30,23),'DISCRETE COLLISIONS  /  CCD OFF',font=large,fill='#f1aaa3')
        draw.text((636,23),'ROTATIONAL CCD ON',font=large,fill='#9be6c4')
        draw.text((30,68),f'Sphere speed: {f["speed_off"]:.2f} units/s   |   Impact impulses: {f["off_impacts"]}',font=small,fill='#f0d4cf')
        draw.text((636,68),f'Sphere speed: {f["speed_on"]:.2f} units/s   |   Impact impulses: {f["ccd_impacts"]}',font=small,fill='#d7f8e9')
        draw.text((30,92),'Same initial conditions · bar, ball and timestep',font=small,fill='#9eafc2')
        draw.text((636,92),f'Time-of-impact events: {f["ccd"]}   |   Unresolved: {f["unresolved"]}',font=small,fill='#9ec8c9')
        draw.text((20,147),f'SHOT {f["shot"]+1} / 8   ·   bar spin {f["omega"]:+.0f} rad/s',font=font,fill='#e8f0fc')
        draw.text((20,175),f'Solver tick {f["tick"]} / 5   ·   each real tick held for {data["repeat_frames"]} display frames',font=small,fill='#9fb9d3')
        draw.rectangle((0,H-35,W,H),fill='#192d40')
        draw.text((22,H-27),'120 Hz C++ rigid-body simulation · 6× slow diagnostic replay · no scripted impact impulses',font=small,fill='#dbe8f3')
        draw.rectangle((22,H-9,W-22,H-5),fill='#38566b')
        draw.rectangle((22,H-9,22+(W-44)*(idx+1)/total,H-5),fill='#62c7cc')
        proc.stdin.write(im.tobytes())
finally:
    proc.stdin.close()
if proc.wait()!=0:raise SystemExit('ffmpeg render failed')
print(f'{dest}: {total} frames at {FPS}fps ({total/FPS}s)')
