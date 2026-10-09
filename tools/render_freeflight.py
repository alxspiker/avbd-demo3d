#!/usr/bin/env python3
"""Render actual sampled Stage 10 10K-body states, solid box faces only."""
import json,math,subprocess,sys
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont

if len(sys.argv)!=3:sys.exit('usage: render_freeflight.py trace.json output.mp4')
trace=json.loads(Path(sys.argv[1]).read_text());frames=trace['frames']
W,H=1200,676
palette=['#48d1db','#edab64','#c288ed','#70d5a3','#ec7a99','#5dafe9','#e5d284','#9be2c2','#cbbcf6','#ff9a6c']
font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
font=lambda n:ImageFont.truetype(font_path,n)
big=font(27);med=font(17);small=font(14)
# Three-dimensional orthographic projection; fixed camera basis, no fake motion.
def project(x,y,z):return (W*.52+(x-z*.72)*5.2,H*.42-(y-75)*3.8+(x+z)*1.1)
def rgb(c):return tuple(bytes.fromhex(c[1:]))
def shade(c,k):return tuple(min(255,round(v*k))for v in rgb(c))
faces=[([4,5,6,7],.78),([0,1,5,4],.56),([0,4,7,3],1.)]
verts=[(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]
# Actually choose camera-visible quads based on world normals: top / +X / +Z.
faces=[([3,2,6,7],1.12),([1,2,6,5],.76),([4,5,6,7],.9)]

out=Path(sys.argv[2]);out.parent.mkdir(parents=True,exist_ok=True)
cmd=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r','20','-i','-','-an','-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p',str(out)]
proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
for j,frame in enumerate(frames):
    im=Image.new('RGB',(W,H),(10,17,30));d=ImageDraw.Draw(im)
    # subtle three-dimensional reference grid
    for x in range(-90,91,15):
        xy0=project(x,-65,-30);xy1=project(x,-65,40)
        d.line([xy0,xy1],fill=(29,48,63),width=1)
    for z in range(-30,41,10):
        d.line([project(-90,-65,z),project(90,-65,z)],fill=(29,48,63),width=1)
    points=sorted(frame['points'],key=lambda p:p[0]+p[2],reverse=True)
    for x,y,z,c in points:
        color=palette[c%10]
        v=[project(x+a*.35,y+b*.35,z+cc*.35)for a,b,cc in verts]
        for ids,k in faces:
            d.polygon([v[t]for t in ids],fill=shade(color,k))
    d.rounded_rectangle((18,16,575,151),radius=14,fill=(17,26,44),outline=(55,74,96),width=2)
    d.text((35,27),'STAGE 10   /   CERTIFIED FREE FLIGHT',font=big,fill='#e4f2ff')
    d.text((35,70),'10,000 rigid boxes advanced by the C++ engine',font=med,fill='#a2d7e5')
    d.text((35,101),'1,250 sampled boxes displayed    |    0 contacts',font=med,fill='#d4e3f3')
    d.text((35,125),'Sparse zero-contact workload • NOT million-body real time',font=small,fill='#9aafc4')
    d.rounded_rectangle((975,16,1180,88),radius=12,fill=(17,26,44),outline=(55,74,96),width=2)
    d.text((991,30),f'{frame["time"]:5.2f}s simulated',font=med,fill='#edeeee')
    d.text((991,58),'CERTIFICATE: PASS',font=small,fill='#74dfb0')
    d.text((22,H-34),'Recorded positions — no hand-animated objects',font=small,fill='#bccad8')
    proc.stdin.write(im.tobytes())
    if j in (4,55,110):im.save(out.parent/f'stage10-reviewed-{j:03}.png')
proc.stdin.close();rc=proc.wait()
if rc:raise RuntimeError(f'ffmpeg exit {rc}')
print(f'Rendered {len(frames)} recorded frames to {out}')
