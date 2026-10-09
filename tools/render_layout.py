"""Render captured Stage 11 actual rigid-body poses (4 independently solved towers).
Usage: python tools/render_scaling.py stage11.json stage11.mp4
"""
import json, math, subprocess, sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
if len(sys.argv)!=3:raise SystemExit('usage: render_scaling.py stage9-trace.json stage9.mp4')
trace,output=map(Path,sys.argv[1:]);data=json.loads(trace.read_text())
assert data['stage']==11 and len(data['frames'])==120
W,H=1120,640
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
f1=ImageFont.truetype(font,18);f2=ImageFont.truetype(font,27);f3=ImageFont.truetype(font,15)
colors=['#5bb8eb','#f8bf78','#82d7ae','#ae98f0']
verts=[(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)]
faces=[(0,4,6,2),(1,3,7,5),(0,1,5,4),(2,6,7,3),(0,2,3,1),(4,5,7,6)]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def sub(a,b):return [a[i]-b[i] for i in range(3)]
def add(a,b):return [a[i]+b[i] for i in range(3)]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def norm(a):return [x/max(1e-10,math.sqrt(dot(a,a))) for x in a]
def rotate(p,q):
 w,x,y,z=q;v=[x,y,z];t=[2*k for k in cross(v,p)];r=cross(v,t)
 return [p[i]+w*t[i]+r[i] for i in range(3)]
def rgb(h,k=1):return tuple(min(255,max(0,int(int(h[i:i+2],16)*k))) for i in (1,3,5))
cam=[32,25,38];target=[0,1.5,0]
forward=norm(sub(target,cam));right=norm(cross(forward,[0,1,0]));up=cross(right,forward)
def project(p):
 d=sub(p,cam);z=max(.2,dot(d,forward));f=1060
 return (W*.5+dot(d,right)*f/z,H*.52-dot(d,up)*f/z,z)
cmd=['ffmpeg','-y','-nostats','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r','24','-i','-',
'-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(output)]
p=subprocess.Popen(cmd,stdin=subprocess.PIPE)
try:
 for index,frame in enumerate(data['frames']):
  im=Image.new('RGB',(W,H),'#101a28');draw=ImageDraw.Draw(im)
  # Grid is diagnostic backdrop; the ground is a real static 50x50 slab.
  for i in range(-25,26,2):
   for a,b in [([i,0,-25],[i,0,25]),([-25,0,i],[25,0,i])]:
    s=project(a);t=project(b);draw.line([s[:2],t[:2]],fill='#263c52',width=1)
  rendered=[]
  for id,pose in enumerate(frame['b']):
   if id==0:continue
   position,quaternion=pose[:3],pose[3:]
   shape=data['shapes'][id];size=shape['size']
   zone=(id-1)//81
   base=colors[min(3,zone)]
   if shape['shape']==1:
    point=project(position);r=size[0]*.5*1060/point[2]
    rendered.append((point[2],('sphere',point,r,base)))
   else:
    positions=[add(position,rotate([v[j]*size[j]*.5 for j in range(3)],quaternion)) for v in verts]
    projected=[project(v) for v in positions]
    for face in faces:
     ps=[positions[j] for j in face]
     inward=cross(sub(ps[1],ps[0]),sub(ps[2],ps[0]))
     outward=norm([-q for q in inward])
     if dot(outward,sub(cam,ps[0]))<=0:continue
     lighting=.62+.36*max(0,dot(outward,norm([-.35,.8,.6])))
     points=[projected[j][:2] for j in face]
     rendered.append((sum(projected[j][2] for j in face)/4,('face',points,rgb(base,lighting))))
  rendered.sort(reverse=True,key=lambda e:e[0])
  for _,entity in rendered:
   if entity[0]=='face':
    _,pts,color=entity
    draw.polygon(pts,fill=color)
    draw.line(pts+[pts[0]],fill='#1b2b3c',width=1)
   else:
    _,pt,r,color=entity
    x,y=pt[:2]
    draw.ellipse((x-r,y-r,x+r,y+r),fill=rgb(color,.83),outline='#172a3f',width=2)
    draw.ellipse((x-r*.63,y-r*.72,x-r*.1,y-r*.2),fill=rgb(color,1.16))
  # HUD foreground, content comes from trace values (not inferred)
  draw.rounded_rectangle((18,15,1102,98),radius=12,fill='#192c3f',outline='#33566d',width=2)
  draw.text((34,25),'STAGE 11  /  SoA PREDICTOR + RIGID-BODY CONTACTS',font=f2,fill='#e4f0fc')
  draw.text((35,64),'324 dynamic rigid bodies · SoA integration · AVBD contacts',font=f3,fill='#a3bdd5')
  y=H-101
  draw.rounded_rectangle((18,y,1102,H-20),radius=10,fill='#192c3f',outline='#31516a',width=2)
  draw.text((36,y+12),f'CONTACTS  {frame["contacts"]:>4}',font=f1,fill='#e6f0fc')
  draw.text((320,y+12),f'ISLANDS  {frame["islands"]:>4}',font=f1,fill='#bce8d7')
  draw.text((580,y+12),f'NARROWPHASE PAIRS  {frame["pairs"]:>4}',font=f1,fill='#e7cef2')
  draw.text((36,y+49),f'Simulated time {index*2/120:.2f}s  ·  recorded 120 Hz · playback at 24 FPS (2.5× slow) · 4 CPU threads',font=f3,fill='#9eb5cd')
  p.stdin.write(im.tobytes())
finally:p.stdin.close()
if p.wait()!=0:raise SystemExit('ffmpeg failed')
print(f'Validated MP4: {output} ({len(data["frames"])} frames, {len(data["frames"])/24:.2f}s)')
