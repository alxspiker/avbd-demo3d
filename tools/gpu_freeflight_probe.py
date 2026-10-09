#!/usr/bin/env python3
"""CUDA experiment: SoA ballistic rigid *translation*, not a GPU AVBD solver.

Requires an NVIDIA GPU plus matching CUDA-capable CuPy installation. CUDA work
runs on device; validation samples are transferred back to the host only after
synchronization and timing. This script does no contact handling or rotation.
"""
import argparse
import json
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bodies', type=int, default=1_000_000)
    parser.add_argument('--steps', type=int, default=100)
    args = parser.parse_args()
    if not (1 <= args.bodies <= 2_000_000 and 1 <= args.steps <= 10000):
        parser.error('bodies must be 1..2m and steps 1..10000')
    try:
        import cupy as cp
        import numpy as np
    except ImportError as e:
        sys.exit('CuPy and NumPy are required; install the CuPy wheel matching your CUDA version: ' + str(e))
    try:
        device_count = cp.cuda.runtime.getDeviceCount()
    except Exception as e:
        sys.exit('CUDA runtime unavailable (NOT a successful GPU test): ' + str(e))
    if not device_count:
        sys.exit('No CUDA GPU devices found; NOT a GPU test')
    dev = cp.cuda.Device()
    props = cp.cuda.runtime.getDeviceProperties(dev.id)
    name = props['name'].decode() if isinstance(props['name'], bytes) else str(props['name'])
    # Structure of arrays: contiguous GPU lanes for each position/velocity axis.
    n = args.bodies
    i = cp.arange(n, dtype=cp.float32)
    px = (i % 103.0).astype(cp.float32)
    py = cp.full(n, 100.0, dtype=cp.float32)
    pz = (i % 47.0).astype(cp.float32)
    vx = (i % 5.0) * cp.float32(0.1)
    vy = cp.full(n, -0.02, dtype=cp.float32)
    vz = (i % 7.0) * cp.float32(0.03)
    dt = cp.float32(1 / 120.0)
    gravity = cp.float32(-0.31)
    kernel = cp.RawKernel(r'''
    extern "C" __global__ void step_freeflight(
        float* px, float* py, float* pz,
        const float* vx, float* vy, const float* vz,
        int count, float dt, float g)
    {
        int j=blockIdx.x*blockDim.x+threadIdx.x;
        if (j >= count) return;
        px[j] += vx[j]*dt;
        py[j] += vy[j]*dt + g*dt*dt;
        pz[j] += vz[j]*dt;
        vy[j] += g*dt;
    }
    ''', 'step_freeflight')
    blocks = ((n+255)//256,)
    args_gpu = (px, py, pz, vx, vy, vz, np.int32(n), np.float32(dt), np.float32(gravity))
    # Compile and warm up WITHOUT modifying the measured arrays.
    tmp = cp.zeros(1, dtype=cp.float32)
    kernel((1,), (256,), (tmp, tmp, tmp, tmp, tmp, tmp, np.int32(0), np.float32(dt), np.float32(gravity)))
    cp.cuda.runtime.deviceSynchronize()
    start, stop = cp.cuda.Event(), cp.cuda.Event()
    start.record()
    for _ in range(args.steps):
        kernel(blocks, (256,), args_gpu)
    stop.record()
    stop.synchronize()
    ms_total = float(cp.cuda.get_elapsed_time(start, stop))
    # Independent double-precision analytic reference for the center sample.
    # BDF1 Euler: velocity gains gravity*dt once per step and position receives
    # the acceleration displacement at each step.
    ids = np.array(sorted({0, n//3, n//2, n-1}), dtype=int)
    read_x = cp.asnumpy(px[ids]);read_y = cp.asnumpy(py[ids]);read_z=cp.asnumpy(pz[ids])
    x0 = (ids % 103).astype(float)
    z0 = (ids % 47).astype(float)
    vx0 = (ids % 5)*0.1
    vz0 = (ids % 7)*0.03
    dt64 = float(dt); g64 = float(gravity); steps = args.steps
    expected_x = x0 + vx0*dt64*steps
    expected_z = z0 + vz0*dt64*steps
    expected_y = 100.0 - 0.02*dt64*steps + g64*dt64**2*steps*(steps+1)/2
    error = max(float(np.max(np.abs(read_x-expected_x))),float(np.max(np.abs(read_z-expected_z))),
                float(np.max(np.abs(read_y-expected_y))))
    tol = 0.002 + 0.00007*steps # generous FP32 repeated-increment drift; tighter than gross physics errors
    passed = bool(np.isfinite(error) and error <= tol)
    report = {
        'kind': 'CUDA_SoA_ballistic_translation_MICROBENCHMARK_NOT_AVBD',
        'gpu': name, 'cuda_devices': int(device_count), 'bodies': n, 'steps': steps,
        'kernel_ms_total': round(ms_total,4), 'kernel_ms_per_step': round(ms_total/steps,5),
        'max_sample_position_error': round(error,7), 'fp32_tolerance': round(tol,7),
        'reference_check_pass': passed, 'collisions': 'NOT IMPLEMENTED',
        'rotational_dynamics': 'NOT IMPLEMENTED', 'host_device_transfer_in_timing': False,
    }
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit('GPU validation FAILED; see error and tolerance above')


if __name__ == '__main__':
    main()
