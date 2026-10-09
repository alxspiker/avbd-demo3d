#!/usr/bin/env python3
"""Render exact one-million-body C++ physics density captures into an MP4."""
import argparse
import json
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter

W, H = 1280, 720
COLORS = np.array([[58, 207, 230], [181, 132, 255], [253, 178, 105]], dtype=np.float32)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('capture', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--fps', type=int, default=12)
    args = parser.parse_args()
    if not 1 <= args.fps <= 60:
        parser.error('fps must be in 1..60')
    records = [json.loads(s) for s in (args.capture/'physics_stats.jsonl').read_text().splitlines() if s.strip()]
    if len(records) < 2:
        raise ValueError('capture contains too few frames')
    count = records[0]['bodies']
    for i, record in enumerate(records):
        if record['frame'] != i or record['bodies'] != count or record['projected'] != count:
            raise ValueError('capture body counts or frame indices inconsistent')
        if record['contacts'] or not record['certified']:
            raise ValueError('cannot label this a certified zero-contact simulation')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = subprocess.Popen([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
        '-r', str(args.fps), '-i', '-', '-an', '-c:v', 'libx264',
        '-preset', 'fast', '-crf', '20', '-pix_fmt', 'yuv420p', str(args.output)
    ], stdin=subprocess.PIPE)
    fonts = ['/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', '/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf']
    available = next((p for p in fonts if Path(p).is_file()), None)
    def font(size): return ImageFont.truetype(available, size) if available else ImageFont.load_default()
    large, medium, small = font(29), font(19), font(15)
    y, x = np.ogrid[:H,:W]
    gradient = (y/H).astype(np.float32)
    background = np.empty((H,W,3), dtype=np.uint8)
    background[:,:,0] = (8 + 7*gradient).astype(np.uint8)
    background[:,:,1] = (16 + 11*gradient).astype(np.uint8)
    background[:,:,2] = (29 + 18*gradient).astype(np.uint8)
    try:
        for index, record in enumerate(records):
            data = np.fromfile(args.capture/f'density_{index:04d}.bin', dtype='<u2')
            if data.size != W*H*3:
                raise ValueError(f'frame {index}: invalid raster length')
            density = data.reshape(H,W,3)
            if int(density.sum(dtype=np.uint64)) != count:
                raise ValueError(f'frame {index}: raster sum does not equal {count} physics bodies')
            # Color encodes depth bands; brightness encodes overlapping body count per pixel.
            intensities = np.log1p(density.astype(np.float32)) / np.log(18.)
            intensities = np.clip(intensities,0.,1.)
            light = np.clip(intensities @ COLORS, 0., 245.)
            composite = np.clip(background.astype(np.float32) + light*.88,0,255).astype(np.uint8)
            im = Image.fromarray(composite, 'RGB')
            # Soft glow changes only the visual display, never physical particle positions.
            glow = Image.fromarray(np.uint8(light*.42),'RGB').filter(ImageFilter.GaussianBlur(radius=3))
            im = Image.blend(im, Image.fromarray(np.maximum(np.asarray(im), np.asarray(glow)), 'RGB'), .19)
            draw = ImageDraw.Draw(im)
            draw.rounded_rectangle((22,20,688,160),radius=15,fill=(12,23,39),outline=(67,96,124),width=2)
            draw.text((42,34),'ONE MILLION REAL RIGID BODIES',font=large,fill=(226,246,252))
            draw.text((43,77),'Actual C++ World::step  •  Every body included in projection',font=medium,fill=(144,210,228))
            draw.text((43,110),'Sparse, certified ZERO CONTACTS  •  No ground or collision pile',font=medium,fill=(243,194,145))
            draw.text((43,136),'Colour = depth band  |  Brightness = overlapping projected bodies',font=small,fill=(157,172,190))
            draw.rounded_rectangle((955,24,1258,129),radius=13,fill=(12,23,39),outline=(67,96,124),width=2)
            draw.text((976,40),f'{count:,} / {count:,} bodies',font=medium,fill=(122,234,187))
            draw.text((976,73),f'Simulation time: {record["sim_seconds"]:.3f} s',font=medium,fill=(227,238,245))
            draw.text((976,104),f'Frame {index+1} / {len(records)}',font=small,fill=(174,193,211))
            draw.rounded_rectangle((18,H-69,W-18,H-17),radius=10,fill=(12,23,39),outline=(52,72,90),width=1)
            draw.text((37,H-57),'Fixed camera  •  Real engine state  •  All-body 3D projection',font=medium,fill=(205,220,231))
            draw.text((814,H-57),f'Playback: {args.fps} fps  |  NOT real-time physics',font=small,fill=(247,193,127))
            if index in (0,len(records)//2,len(records)-1):
                im.save(args.output.parent/f'stage11_million_preview_{index:03d}.png')
            ffmpeg.stdin.write(im.tobytes())
    except Exception:
        ffmpeg.stdin.close()
        ffmpeg.wait()
        raise
    ffmpeg.stdin.close()
    if ffmpeg.wait():
        raise RuntimeError('ffmpeg video encoding failed')
    print(f'PASS: rendered {len(records)} verified, full-population frames into {args.output}')
    print(f'Video playback {len(records)/args.fps:.2f}s; physics elapsed {records[-1]["sim_seconds"]:.3f}s; this is NOT real-time physics.')

if __name__=='__main__': main()
