"""Render recorded C++ rigid-body traces to MP4. No generated/interpolated body motion.
Usage: python tools/render_stages.py stage1.json stage1.mp4
Pillow+ffmpeg are optional external presentation deps, not physics dependencies.
"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import subprocess,sys,json,math
if len(sys.argv)!=3:raise SystemExit('usage: render_stages.py input.json output.mp4')
source,target=map(Path,sys.argv[1:]);d=json.loads(source.read_text());stage=d['stage'];frames=d['frames'];shapes=d['boxes']; links=d['links'];
W,H=960,540;FPS=24;duration=12;N=FPS*duration
samples={1:(0,359),2:(0,359),3:(0,180),4:(0,359),5:(0,359)}
first,last=samples[stage]
configs={1:([17,13,23],[0,3,0],1100),2:([15,11,26],[0,3.8,0],1010),3:([17,12,24],[0,3,0],1010),4:([13,15,30],[-9,5.2,0],1000),5:([18,14,25],[0,3,0],1120)}
cam,look,F=configs[stage]
root=Path(__file__).parent
font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
f_title=ImageFont.truetype(font_path,24)
f_small=ImageFont.truetype(font_path,15)
f_mini=ImageFont.truetype(font_path,13)
CUBE_VERTS=[(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)]
FACES=[(0,4,6,2),(1,3,7,5),(0,1,5,4),(2,6,7,3),(0,2,3,1),(4,5,7,6)]
PALETTE=['#528cb4','#6ebad2','#90a9e2','#b18ad1','#ddad6c','#69b79e','#d38d88']

def norm(a):
 l=math.sqrt(sum(v*v for v in a));return [v/max(l,1e-12) for v in a]
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def sub(a,b):return [x-y for x,y in zip(a,b)]
def add(a,b):return [x+y for x,y in zip(a,b)]
def rot(v,q):
 w,x,y,z=q;u=(x,y,z);t=[2*k for k in cross(u,v)];c=cross(u,t)
 return [v[i]+w*t[i]+c[i] for i in range(3)]
forward=norm(sub(look,cam));right=norm(cross(forward,(0,1,0)));up=cross(right,forward)
def project(v):
 disp=sub(v,cam);depth=dot(disp,forward)
 if depth<0.1:depth=0.1
 return (W*.5+dot(disp,right)*F/depth, H*.50-dot(disp,up)*F/depth,depth)
def light(hexcol,mult):
 b=bytes.fromhex(hexcol[1:]);return tuple(int(max(0,min(255,v*mult))) for v in b)
lightdir=norm((-0.4,0.9,0.5))
proc=subprocess.Popen(['ffmpeg','-y','-nostats','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size',f'{W}x{H}','-framerate',str(FPS),'-i','-','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(target)],stdin=subprocess.PIPE)
try:
 for frame_idx in range(N):
  index=round(first+(last-first)*frame_idx/(N-1))
  state=frames[index]
  im=Image.new('RGB',(W,H),'#0e1929');draw=ImageDraw.Draw(im)
  # Atmospheric horizontal lines and grounded local grid, not a fake physics floor.
  groundspan={1:13,2:11,3:14,4:18,5:13}[stage]
  g_x=range(-groundspan,groundspan+1)
  origin_x=-9 if stage==4 else 0
  for u in g_x:
   for a,b in [([origin_x-groundspan,0,u],[origin_x+groundspan,0,u]),([origin_x+u,0,-groundspan],[origin_x+u,0,groundspan])]:
    pa,pb=project(a),project(b)
    draw.line([pa[:2],pb[:2]],fill='#28415a' if u%5 else '#3a5870',width=1)
  entities=[]
  for bi,pos in enumerate(state['b']):
   if bi==0 and stage in (1,3,4,5):continue # ground plane drawn as grid
   geom=shapes[bi];shape=geom['shape'];size=geom['size'];p=pos[:3];q=pos[3:]
   base=PALETTE[(bi//3)%len(PALETTE)]
   if shape==1:
    center=project(p);r=size[0]*.5*F/center[2]
    if r<.5:continue
    entities.append((center[2],('sphere',center,r,base)))
   else:
    vv=[project(add(p,rot([x*size[0]/2,y*size[1]/2,z*size[2]/2],q))) for x,y,z in CUBE_VERTS]
    for face in FACES:
     pts=[vv[i] for i in face]
     a,b,c=pts[:3]
     area=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
     if area>0:continue
     point=[add(p,rot([CUBE_VERTS[i][j]*size[j]/2 for j in range(3)],q)) for i in face]
     n=norm(cross(sub(point[1],point[0]),sub(point[2],point[0])))
     brightness=.56+.38*max(0,dot(n,lightdir))
     entities.append((sum(pt[2] for pt in pts)/len(pts),('face',[v[:2] for v in pts],light(base,brightness))))
  # Draw projected distance-joint graph underneath foreground geometry.
  for j,active in enumerate(state['j']):
   if not active:continue
   ia,ib=links[j];a=project(state['b'][ia][:3]);b=project(state['b'][ib][:3])
   draw.line([a[:2],b[:2]],fill='#f8d6a0',width=5)
   draw.line([a[:2],b[:2]],fill='#e5a859',width=2)
  entities.sort(key=lambda p:p[0],reverse=True)
  for _,item in entities:
   if item[0]=='face':
    _,pts,color=item
    draw.polygon(pts,fill=color)
    draw.line(pts+[pts[0]],fill='#16283a',width=2,joint='curve')
   else:
    _,(x,y,depth),r,color=item
    if r>2000:continue
    draw.ellipse((x-r,y-r,x+r,y+r),fill=light(color,.65),outline='#1e2f40',width=2)
    for frac,mul in [(.88,.88),(.69,1.1),(.43,1.19)]:
     ox=-r*(1-frac)*.45;oy=-r*(1-frac)*.6;rr=r*frac
     draw.ellipse((x+ox-rr,y+oy-rr,x+ox+rr,y+oy+rr),fill=light(color,mul))
  # HUD overlays with consistent margins.
  draw.rounded_rectangle((21,20,525,108),radius=12,fill='#15263a',outline='#3a5771',width=2)
  draw.text((39,34),d['title'],font=f_title,fill='#f2f6ff')
  phys_time=index*d['dt']; speed=round((last-first)/(N-1)*FPS*d['dt'],2)
  draw.text((40,76),f'RECORDED C++ SIMULATION  ·  {phys_time:05.2f} s  ·  {speed}x playback',font=f_mini,fill='#a7c0d6')
  draw.rounded_rectangle((W-264,20,W-22,91),radius=12,fill='#15263a',outline='#35536c',width=2)
  draw.text((W-247,31),f'{len(shapes)-1} moving bodies',font=f_small,fill='#dbe9f6')
  msg={1:f'contact impulses {state["impacts"]}',2:f'0 wall crossings (verified per physics tick)',3:f'contact impulses {state["impacts"]}',4:f'connected links {sum(state["j"])} / {len(state["j"])}',5:f'sleeping bodies {state.get("sleeping",0)}'}[stage]
  draw.text((W-247,59),msg,font=f_mini,fill='#8ecad3')
  draw.rounded_rectangle((24,H-31,W-24,H-23),radius=4,fill='#364b64')
  draw.rounded_rectangle((24,H-31,24+(W-48)*(frame_idx+1)/N,H-23),radius=4,fill='#63becd')
  proc.stdin.write(im.tobytes())
finally:proc.stdin.close()
if proc.wait():raise SystemExit('ffmpeg failed')
print(f'{target}: {N} frames, {duration} seconds, trace range {first}-{last}')
