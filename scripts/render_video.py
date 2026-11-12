#!/usr/bin/env python3
"""Render the verified two-hand performance to MP4 with piano audio (offscreen EGL).

Reuses PlanRunner's model build and trajectory tracks and replicates its control /
step loop verbatim, so the motion is identical to the verified playback. Adds
per-frame offscreen rendering piped to ffmpeg, then synthesizes piano audio from
the observed key edges with fluidsynth and muxes it in.

Simulation-only visualization; not a real-robot recording.

Run with the package on PYTHONPATH, e.g.:
    export PYTHONPATH="$P:$P/scripts" MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1
    python3 render_video.py --package "$P" --output "$W"
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import mujoco as m
import numpy as np

from rm_mujoco.keyboard import KeyHighlight
from rm_mujoco.l20_joint_mapping import command_positions
from rm_mujoco.performance import load_plan
from run_performance import PlanRunner, observed_note_spans, write_observed_midi

FFMPEG = shutil.which("ffmpeg") or "ffmpeg"


def open_encoder(path, width, height, fps):
    return subprocess.Popen(
        [FFMPEG, "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
         "-c:v", "libx264", "-preset", "medium", "-crf", "18",
         "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--plan", type=Path, default=None)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--views", default="overview,keys")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--tail", type=float, default=0.8)
    parser.add_argument("--soundfont", default="/usr/share/sounds/sf2/FluidR3_GM.sf2")
    parser.add_argument("--no-audio", action="store_true")
    args = parser.parse_args(argv)

    package = args.package.resolve()
    plan_path = args.plan or package / "examples/two_tigers_flat_palms/verified_40/performance.yaml"
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    views = [v.strip() for v in args.views.split(",") if v.strip()]

    plan = load_plan(plan_path)
    tempo = float(plan["metadata"].get("tempo_scale", 1.0))
    tolerance = float(plan["metadata"].get("timing_tolerance_sec", 0.03))
    runner = PlanRunner(
        plan, package, package / "config/piano_scene.yaml",
        package / "models/rm_75_6fb_v_dual_l20.xml", tempo, tolerance,
        None, None, None, views[0])
    model, data = runner.model, runner.data
    highlight = KeyHighlight(model)

    last_event = max([e["off"] for e in runner.expected] or [0.0])
    last_state = max([s["time_sec"] for h in plan["hands"].values()
                      for s in h["hand_states"]] or [0.0]) * tempo
    last_arm = max([w["time_sec"] for h in plan["hands"].values()
                    for w in h["arm_waypoints"]] or [0.0]) * tempo
    end = max(last_event, last_state, last_arm) + args.tail

    frame_dt = 1.0 / args.fps
    encoders = {v: open_encoder(out / f"silent_{v}.mp4", args.width, args.height, args.fps)
                for v in views}
    renderers = {v: m.Renderer(model, height=args.height, width=args.width) for v in views}

    pressed = set()
    edges = []
    frames = 0
    next_frame = 0.0
    while data.time < end:
        for side, track in runner.arm_tracks.items():
            data.ctrl[runner.arm_act[side]] = track.at(data.time)
        for side, track in runner.hand_tracks.items():
            data.ctrl[runner.hand_act[side]] = command_positions(track.at(data.time).tolist())
        before = data.time
        m.mj_step(model, data)
        if data.time <= before or data.warning.number.any():
            raise RuntimeError("unstable MuJoCo trajectory")
        for t, edge, note in runner.key_events.update(data):
            edges.append((float(t), edge, int(note)))
            if edge == "on":
                pressed.add(int(note))
            else:
                pressed.discard(int(note))
        if data.time >= next_frame:
            highlight.set_pressed(pressed)
            for view in views:
                renderers[view].update_scene(data, camera=view)
                encoders[view].stdin.write(renderers[view].render().tobytes())
            frames += 1
            next_frame += frame_dt

    for view in views:
        renderers[view].close()
        encoders[view].stdin.close()
        encoders[view].wait()

    midi = out / "played_keys.mid"
    write_observed_midi(midi, observed_note_spans(edges, end_time=data.time))

    audio = None
    if not args.no_audio and shutil.which("fluidsynth") and Path(args.soundfont).is_file():
        audio = out / "piano.wav"
        subprocess.run(
            ["fluidsynth", "-ni", "-F", str(audio), "-r", "44100", "-g", "1.0",
             args.soundfont, str(midi)], check=True, capture_output=True,
            stdin=subprocess.DEVNULL)

    finals = []
    for view in views:
        silent = out / f"silent_{view}.mp4"
        final = out / f"two_tigers_{view}.mp4"
        if audio is not None:
            subprocess.run(
                [FFMPEG, "-y", "-i", str(silent), "-i", str(audio),
                 "-c:v", "copy", "-c:a", "aac", "-b:a", "192k",
                 "-shortest", "-movflags", "+faststart", str(final)],
                check=True, capture_output=True, stdin=subprocess.DEVNULL)
            silent.unlink()
        else:
            silent.rename(final)
        finals.append(final)

    print(f"RENDER frames={frames} sim_end={data.time:.3f}s fps={args.fps} "
          f"views={','.join(views)} audio={'on' if audio else 'off'}")
    for final in finals:
        print(f"VIDEO {final}")
    return 0


if __name__ == "__main__":
    sys.exit(main())