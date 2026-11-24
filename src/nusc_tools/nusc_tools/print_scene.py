"""Print the GT boxes of every keyframe of a nuScenes scene (global frame).

Usage (inside the container):
    ros2 run nusc_tools print_scene --scene scene-0103 --max-frames 3
"""

import argparse
import os

from nuscenes.nuscenes import NuScenes

from nusc_tools.scene_loader import load_scene_keyframes


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--scene', default='scene-0103',
                        help='scene name (mini_val: scene-0103, scene-0916)')
    parser.add_argument('--version', default='v1.0-mini')
    parser.add_argument('--dataroot', default=os.environ.get('NUSC_ROOT'),
                        help='nuScenes root folder (default: $NUSC_ROOT)')
    parser.add_argument('--max-frames', type=int, default=None,
                        help='print only the first N keyframes')
    parser.add_argument('--keep-empty', action='store_true',
                        help='also print boxes with 0 lidar/radar points')
    args = parser.parse_args(argv)

    if not args.dataroot:
        parser.error('set NUSC_ROOT or pass --dataroot')

    nusc = NuScenes(version=args.version, dataroot=args.dataroot, verbose=False)
    keyframes = load_scene_keyframes(nusc, args.scene,
                                     drop_empty=not args.keep_empty)

    t0 = keyframes[0].timestamp_s
    prev_t = t0
    for i, frame in enumerate(keyframes[:args.max_frames]):
        dt = frame.timestamp_s - prev_t
        prev_t = frame.timestamp_s
        print(f'\n=== keyframe {i:2d}  t={frame.timestamp_s:.6f} s  '
              f'(t-t0={frame.timestamp_s - t0:6.3f} s, dt={dt:.3f} s)  '
              f'{len(frame.boxes)} boxes')
        print(f'{"class":<11} {"instance":<8} {"x":>9} {"y":>9} {"z":>6} '
              f'{"yaw":>6} {"w":>5} {"l":>5} {"h":>5} {"vx":>6} {"vy":>6} {"pts":>5}')
        for b in frame.boxes:
            x, y, z = b.translation
            w, l, h = b.size_wlh
            vx, vy = b.velocity_xy
            # Only the first 8 characters of the token: enough to tell instances apart.
            print(f'{b.tracking_name:<11} {b.instance_token[:8]:<8} '
                  f'{x:9.2f} {y:9.2f} {z:6.2f} {b.yaw:6.2f} '
                  f'{w:5.2f} {l:5.2f} {h:5.2f} {vx:6.2f} {vy:6.2f} {b.num_pts:5d}')

    print(f'\n{args.scene}: {len(keyframes)} keyframes, '
          f'{keyframes[-1].timestamp_s - t0:.2f} s')


if __name__ == '__main__':
    main()
