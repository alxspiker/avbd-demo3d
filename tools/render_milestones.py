"""Render C++ trace to MP4; no fabricated body positions. Pillow and ffmpeg required."""
import sys,json,math,subprocess
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
if len(sys.argv)!=3:raise SystemExit('usage: render_milestones.py trace.json output.mp4')
src,out=map(Path,sys.argv[1:]);data=json.loads(src.read_text())
stage=data['stage'];assert stage in (6,7)
W,H=(1200,480) if stage==6 else (960,540)
FPS=24;N=288
base=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
font=ImageFont.truetype(str(base),17);small=ImageFont.truetype(str(base),13);large=ImageFont.truetype(str(base),22)
verts=[(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)]
faces=[(0,4,6,2),(1,3,7,5),(0,1,5,4),(2,6,7,3),(0,2,3,1),(4,5,7,6)]
colors=['#66b5e5','#8db1ed','#9dc4e9','#d8ae81','#9dcfae','#b9a9dd','#d59492']
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def sub(a,b):return [a[i]-b[i] for i in range(3)]
def add(a,b):return [a[i]+b[i] for i in range(3)]
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def norm(a):return [x/max(1e-15,math.sqrt(dot(a,a))) for x in a]
def rot(p,q):
 w,x,y,z=q;v=[x,y,z];t=[u*2 for u in cross(v,p)];c=cross(v,t)
 return [p[i]+w*t[i]+c[i] for i in range(3)]
def shade(c,b):
 rgb=bytes.fromhex(c[1:]);return tuple(max(0,min(255,int(z*b))) for z in rgb)
def scene(draw,state,xoff,panelwidth,world,theme,hide_fixed=False):
 cam,look,F=([8,7,13],[0,2.5,0],650) if stage==6 else ([19,14,25],[0,3.5,0],930)
 forward=norm(sub(look,cam));right=norm(cross(forward,[0,1,0]));up=cross(right,forward)
 def project(p):
  d=sub(p,cam);z=dot(d,forward)
  if z<.1:z=.1
  return (xoff+panelwidth*.5+dot(d,right)*F/z,H*.52-dot(d,up)*F/z,z)
 span=9 if stage==6 else 14
 for j in range(-span,span+1):
  for a,b in [([-span,0,j],[span,0,j]),([j,0,-span],[j,0,span])]:
   pa,pb=project(a),project(b)
   if xoff-50 < pa[0] < xoff+panelwidth+50 or xoff-50 < pb[0] < xoff+panelwidth+50:
    draw.line([pa[:2],pb[:2]],fill='#2a4359',width=1)
 entities=[]
 for i,pose in enumerate(state):
  if hide_fixed and i==0:continue
  if pose[1]<-100:continue
  geo=data['boxes'][i];size=geo['size'];shape=geo['shape'];p=pose[:3];q=pose[3:]
  base=('#b07fdb' if i==0 else theme) if stage==6 else colors[(i//7)%len(colors)]
  if shape==1:
   pt=project(p);rad=size[0]*.5*F/pt[2]
   if not(.5<rad<1000):continue
   entities.append((pt[2],('sphere',pt,rad,base)))
  else:
   vv=[add(p,rot([v[j]*size[j]*.5 for j in range(3)],q)) for v in verts]
   pp=[project(v) for v in vv]
   for face in faces:
    poly=[vv[j] for j in face]
    inward=cross(sub(poly[1],poly[0]),sub(poly[2],poly[0]))
    n=norm([-v for v in inward])
    if dot(n,sub(cam,poly[0]))<=0:continue
    light=.54+.40*max(0,dot(n,norm([-.3,.9,.5])))
    points=[pp[j][:2] for j in face]
    entities.append((sum(pp[j][2] for j in face)/4,('face',points,shade(base,light))))
 entities.sort(reverse=True,key=lambda v:v[0])
 for _,e in entities:
  if e[0]=='sphere':
   _,pt,r,c=e;cx,cy=pt[:2]
   if cx+r <xoff or cx-r >xoff+panelwidth:continue
   draw.ellipse((cx-r,cy-r,cx+r,cy+r),fill=shade(c,.68),outline='#15273c',width=2)
   for scale,m in [(.86,.88),(.66,1.06),(.4,1.22)]:
    px=cx-r*(1-scale)*.43;py=cy-r*(1-scale)*.54
    draw.ellipse((px-r*scale,py-r*scale,px+r*scale,py+r*scale),fill=shade(c,m))
  else:
   _,pts,c=e
   if max(x for x,y in pts)<xoff or min(x for x,y in pts)>xoff+panelwidth:continue
   draw.polygon(pts,fill=c)
   draw.line(pts+[pts[0]],fill='#1a2b3e',width=2)
def draw_trails(draw,frames,idx,side,xoff,color):
 # Diagnostic trail from recorded sample positions, not interpolated motion.
 cam=[8,7,13];look=[0,2.5,0];F=650
 forward=norm(sub(look,cam));right=norm(cross(forward,[0,1,0]));up=cross(right,forward)
 def pt(p):
  d=sub(p,cam);z=max(.1,dot(d,forward))
  return (xoff+300+dot(d,right)*F/z,H*.52-dot(d,up)*F/z)
 start=max(0,idx-6)
 for body_id in range(1,len(data['boxes'])):
  last=None
  for frame in frames[start:idx+1]:
   p=frame[side][body_id][:3]
   if p[1]<-100: last=None;continue
   if last is not None and math.dist(p,last)<2.5:
    a,b=pt(last),pt(p)
    if xoff<=a[0]<=xoff+600 and xoff<=b[0]<=xoff+600:
     draw.line([a,b],fill=color,width=3)
  
   last=p

proc=subprocess.Popen(['ffmpeg','-y','-nostats','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(FPS),'-i','-','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE)
frames=data['frames'];n=len(frames)
try:
 for frameidx in range(N):
  idx=round((n-1)*frameidx/(N-1));f=frames[idx]
  im=Image.new('RGB',(W,H),'#101b2a');draw=ImageDraw.Draw(im)
  if stage==6:
   scene(draw,f['off'],0,600,0,'#ed8175')
   scene(draw,f['on'],600,600,1,'#65d5b9')
   draw_trails(draw,frames,idx,'off',0,'#ec8a77')
   draw_trails(draw,frames,idx,'on',600,'#64d4bb')
   draw.line([(600,0),(600,H)],fill='#4c6a7f',width=3)
   draw.rounded_rectangle((18,15,555,91),radius=8,fill='#1b2b3d',outline='#8c484a',width=2)
   draw.text((32,25),'DISCRETE  /  CCD OFF',font=large,fill='#f4aaa1')
   draw.text((32,61),'50 units/s  ·  1/30 s physics ticks  ·  thin wall',font=small,fill='#afc3d4')
   draw.rounded_rectangle((619,15,1178,91),radius=8,fill='#1b2b3d',outline='#408e81',width=2)
   draw.text((633,25),'SWEPT TOI  /  CCD ON',font=large,fill='#93e7ce')
   draw.text((633,61),f'{f["shots"]} launched  ·  {f["ccd_events"]} detected TOI events  ·  {f["unresolved"]} unresolved',font=small,fill='#afc3d4')
   draw.text((22,H-54),'Trails: measured C++ body positions. Launch positions are scripted.',font=small,fill='#c6d4e2')
  else:
   scene(draw,f['b'],0,W,0,'#66a5dd',hide_fixed=True)
   draw.rounded_rectangle((18,15,550,103),radius=10,fill='#192c41',outline='#496a83',width=2)
   draw.text((32,26),'07 / COLORED PARALLEL SOLVER',font=large,fill='#f0f7fe')
   draw.text((32,64),f'{len(data["boxes"])-1} moving bodies  ·  4 CPU threads  ·  {f["colors"]} scheduling colors',font=small,fill='#a7d4e8')
   draw.text((W-250,30),f'{f["contacts"]} active contact points',font=small,fill='#d7efff')
  draw.rounded_rectangle((25,H-29,W-25,H-22),radius=3,fill='#385068')
  draw.rounded_rectangle((25,H-29,25+(W-50)*(frameidx+1)/N,H-22),radius=3,fill='#63c5d1')
  draw.text((W-180,H-55),f'{idx*data["dt"]:5.2f} s simulated',font=small,fill='#bbd1e7')
  proc.stdin.write(im.tobytes())
finally:proc.stdin.close()
if proc.wait():raise SystemExit('ffmpeg failed')
print(f'{out} {N} frames / {N/FPS}s, {n} recorded frames')
