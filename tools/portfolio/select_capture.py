"""Make the publication crops from the original two-display capture artifacts."""
import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/"docs/media/capture"


def run(*args):
    subprocess.run(["ffmpeg", "-loglevel", "error", "-nostdin", "-y", *map(str, args)], check=True)


def prepare_first(directory):
    OUT.mkdir(parents=True, exist_ok=True)
    gz, rviz = directory/"gazebo.mkv", directory/"rviz.mkv"
    # Trim the GUI chrome for the film. The full RViz screenshot retains the Nav2 status panel.
    run("-ss", 31, "-i", gz, "-frames:v", 1, "-vf", "crop=852:480:4:165", OUT/"gazebo-hero.png")
    run("-ss", 29, "-i", rviz, "-frames:v", 1, OUT/"rviz-navigation.png")
    run("-ss", 28, "-i", gz, "-ss", 28, "-i", rviz, "-t", 8,
        "-filter_complex", "[0:v]crop=852:480:4:165,scale=880:496[a];"
                           "[1:v]crop=800:450:460:228,scale=880:496[b];[a][b]hstack[v]",
        "-map", "[v]", "-an", "-c:v", "libx264", "-crf", 18, "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", OUT/"nav.mp4")


def prepare_move(directory, start):
    if start is None:
        raise ValueError("Pass the verified clip start in recording seconds.")
    run("-ss", start, "-i", directory/"gazebo.mkv", "-t", 9, "-an",
        "-vf", "crop=852:480:4:165", "-c:v", "libx264", "-crf", 18, "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", OUT/"move.mp4")


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first", type=Path)
    parser.add_argument("--move", type=Path)
    parser.add_argument("--move-start", type=float)
    args=parser.parse_args()
    if args.first:
        prepare_first(args.first)
    if args.move:
        prepare_move(args.move, args.move_start)
