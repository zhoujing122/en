#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path


def read_pgm(path):
    data = Path(path).read_bytes()
    idx = 0

    def token():
        nonlocal idx
        while idx < len(data) and chr(data[idx]).isspace():
            idx += 1
        if idx < len(data) and data[idx] == ord("#"):
            while idx < len(data) and data[idx] != ord("\n"):
                idx += 1
            return token()
        start = idx
        while idx < len(data) and not chr(data[idx]).isspace():
            idx += 1
        return data[start:idx].decode("ascii")

    magic = token()
    width = int(token())
    height = int(token())
    maxval = int(token())
    while idx < len(data) and chr(data[idx]).isspace():
        idx += 1
    if magic == "P5":
        pixels = list(data[idx:idx + width * height])
    elif magic == "P2":
        pixels = [int(x) for x in data[idx:].split()]
    else:
        raise ValueError(f"unsupported PGM magic: {magic}")
    if maxval != 255:
        pixels = [int(p * 255 / maxval) for p in pixels]
    return width, height, pixels


def parse_map_yaml(path):
    resolution = 0.05
    origin = [0.0, 0.0, 0.0]
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = [x.strip() for x in line.split(":", 1)]
        if key == "resolution":
            resolution = float(value)
        elif key == "origin":
            origin = [float(x.strip()) for x in value.strip("[]").split(",")]
    return resolution, origin


def read_trajectory(path):
    with Path(path).open(newline="") as f:
        for row in csv.DictReader(f):
            yield float(row["x_m"]), float(row["y_m"]), float(row["yaw_rad"])


def set_px(img, width, height, x, y, color, radius=1):
    for yy in range(y - radius, y + radius + 1):
        for xx in range(x - radius, x + radius + 1):
            if 0 <= xx < width and 0 <= yy < height:
                img[(yy * width + xx) * 3:(yy * width + xx) * 3 + 3] = bytes(color)


def draw_line(img, width, height, x0, y0, x1, y1, color):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        set_px(img, width, height, x0, y0, color, radius=1)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def world_to_pixel(x, y, resolution, origin, height):
    px = int(round((x - origin[0]) / resolution))
    py_map = int(round((y - origin[1]) / resolution))
    return px, height - 1 - py_map


def scale_image(rgb, width, height, scale):
    if scale <= 1:
        return rgb, width, height
    out_w = width * scale
    out_h = height * scale
    out = bytearray(out_w * out_h * 3)
    for y in range(out_h):
        src_y = y // scale
        for x in range(out_w):
            src_x = x // scale
            out[(y * out_w + x) * 3:(y * out_w + x) * 3 + 3] = rgb[(src_y * width + src_x) * 3:(src_y * width + src_x) * 3 + 3]
    return out, out_w, out_h


def write_ppm(path, rgb, width, height):
    with Path(path).open("wb") as f:
        f.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        f.write(rgb)


def main():
    parser = argparse.ArgumentParser(description="Overlay trajectory.csv on map.pgm and write a portable PPM preview.")
    parser.add_argument("run_dir")
    parser.add_argument("--output", default=None)
    parser.add_argument("--scale", type=int, default=4)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    width, height, gray = read_pgm(run_dir / "map.pgm")
    resolution, origin = parse_map_yaml(run_dir / "map.yaml")
    rgb = bytearray()
    for p in gray:
        rgb.extend([p, p, p])

    points = [world_to_pixel(x, y, resolution, origin, height) + (yaw,) for x, y, yaw in read_trajectory(run_dir / "trajectory.csv")]
    for a, b in zip(points, points[1:]):
        draw_line(rgb, width, height, a[0], a[1], b[0], b[1], (220, 30, 30))
    if points:
        sx, sy, _ = points[0]
        ex, ey, yaw = points[-1]
        set_px(rgb, width, height, sx, sy, (30, 120, 240), radius=2)
        set_px(rgb, width, height, ex, ey, (30, 180, 60), radius=2)
        hx = ex + int(round(math.cos(yaw) * 6))
        hy = ey - int(round(math.sin(yaw) * 6))
        draw_line(rgb, width, height, ex, ey, hx, hy, (30, 180, 60))

    rgb, out_w, out_h = scale_image(rgb, width, height, max(1, args.scale))
    out = Path(args.output) if args.output else run_dir / "map_preview.ppm"
    write_ppm(out, rgb, out_w, out_h)
    print(out)


if __name__ == "__main__":
    main()
