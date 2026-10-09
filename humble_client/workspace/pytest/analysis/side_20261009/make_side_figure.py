#!/usr/bin/env python3
"""make_side_figure.py - picture of the pass-side rule and the threshold choice (9 Oct 2026).

Data: loc-corrected first offset samples from sidesweep_result.txt (same folder). Writes side_threshold.svg; convert to
PNG with ImageMagick:  convert -density 110 side_threshold.svg side_threshold.png
Plain SVG (no matplotlib: the host's matplotlib is broken by NumPy 2).
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
BLUE, ORANGE = '#2a78d6', '#eb6834'          # keep RIGHT / keep LEFT (validated pair, light surface)
SURF, INK, INK2, MUTED, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984', '#e4e3df'
W, H = 1400, 1000
svg = []
add = svg.append


def text(x, y, s, size=16, fill=INK, anchor='start', weight='normal', italic=False):
    st = ' font-style="italic"' if italic else ''
    add(f'<text x="{x}" y="{y}" font-size="{size}" fill="{fill}" text-anchor="{anchor}" '
        f'font-weight="{weight}"{st}>{s}</text>')


add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
    f'font-family="DejaVu Sans, Arial, sans-serif">')
add(f'<rect width="{W}" height="{H}" fill="{SURF}"/>')
text(60, 52, 'How the robot picks its pass side - and why the threshold should be 0.20 m', 26, weight='bold')
text(60, 82, 'Head-on hallway runs, TurtleBot 4, rec15 + noghost critic, 9 Oct 2026 (and the 5 Oct goal-side run)', 15, INK2)

# ---------------- Panel 1: top view of the hallway ----------------
text(60, 130, '1  The rule (top view)', 19, weight='bold')
x0, x1 = 120, 1290                 # hallway drawn from the robot (left) to the goal (right)
yc = 300                           # robot-goal line
ppm = 100                          # pixels per metre across the hallway
half = 1.25
yt, yb = yc - half * ppm, yc + half * ppm
yth = yc + 0.20 * ppm              # threshold 0.20 m to the robot's right (screen down)
# zones
add(f'<rect x="{x0}" y="{yt}" width="{x1 - x0}" height="{yth - yt}" fill="{BLUE}" fill-opacity="0.10"/>')
add(f'<rect x="{x0}" y="{yth}" width="{x1 - x0}" height="{yb - yth}" fill="{ORANGE}" fill-opacity="0.12"/>')
# walls
for y in (yt, yb):
    add(f'<line x1="{x0}" y1="{y}" x2="{x1}" y2="{y}" stroke="{INK2}" stroke-width="4"/>')
text(x1 - 6, yt - 10, 'wall', 13, MUTED, 'end')
text(x1 - 6, yb + 22, 'wall', 13, MUTED, 'end')
# robot-goal line and threshold line
add(f'<line x1="{x0 + 40}" y1="{yc}" x2="{x1 - 40}" y2="{yc}" stroke="{INK2}" stroke-width="2" stroke-dasharray="8 6"/>')
text(640, yc - 10, 'robot-goal line (offset 0)', 14, INK2, 'middle')
add(f'<line x1="{x0}" y1="{yth}" x2="{x1}" y2="{yth}" stroke="{ORANGE}" stroke-width="2"/>')
text(1040, yth + 18, 'threshold: 0.20 m to the robot\'s right', 14, ORANGE, 'middle', 'bold')
# robot (circle with heading tick) and goal
add(f'<circle cx="{x0 + 40}" cy="{yc}" r="22" fill="{INK}"/>')
add(f'<line x1="{x0 + 40}" y1="{yc}" x2="{x0 + 62}" y2="{yc}" stroke="{SURF}" stroke-width="4"/>')
text(x0 + 40, yc + 46, 'robot', 14, INK, 'middle', 'bold')
add(f'<path d="M {x1 - 40} {yc - 14} l 12 14 l -12 14 l -12 -14 z" fill="{INK2}"/>')
text(x1 - 40, yc + 36, 'goal', 14, INK2, 'middle')
# walker A: centred -> robot keeps RIGHT (screen down), passes on the walker's left
wa = 820
add(f'<circle cx="{wa}" cy="{yc - 12}" r="14" fill="{SURF}" stroke="{BLUE}" stroke-width="4"/>')
text(wa, yc - 40, 'walker centred', 14, INK, 'middle', 'bold')
add(f'<path d="M {x0 + 64} {yc} C 420 {yc}, 560 {yc + 70}, {wa - 90} {yc + 82} S {wa + 120} {yc + 82}, 1180 {yc + 20}" '
    f'fill="none" stroke="{BLUE}" stroke-width="3"/>')
text(560, yc + 104, 'robot keeps RIGHT  -  passes on your left (German keep-right)', 15, BLUE, 'start', 'bold')
# walker B: clearly to the robot's right -> robot keeps LEFT (screen up)
wb = 1000
add(f'<circle cx="{wb}" cy="{yc + 0.75 * ppm}" r="14" fill="{SURF}" stroke="{ORANGE}" stroke-width="4"/>')
text(wb + 24, yc + 0.75 * ppm + 6, 'walker clearly on the robot\'s right', 14, INK, 'start', 'bold')
add(f'<path d="M {x0 + 64} {yc} C 500 {yc}, 700 {yc - 60}, {wb - 60} {yc - 70} S {wb + 140} {yc - 60}, 1180 {yc - 16}" '
    f'fill="none" stroke="{ORANGE}" stroke-width="3" stroke-dasharray="10 6"/>')
text(700, yc - 84, 'robot keeps LEFT - does not cut across you', 15, ORANGE, 'start', 'bold')
text(60, yb + 58, 'The side is set by the FIRST position sample (~6.5 m away); the side hold keeps it unless you later read', 15, INK2)
text(60, yb + 80, 'more than 0.30 m on the other side. So the threshold decides on one early, noisy estimate.', 15, INK2)

# ---------------- Panel 2: ruler with today's first samples ----------------
top2 = 545
text(60, top2, '2  Where each run\'s first sample fell (corrected for the AMCL heading error)', 19, weight='bold')
lo, hi = -0.40, 0.20
rx0, rx1 = 150, 1290
def X(v):
    return rx0 + (v - lo) / (hi - lo) * (rx1 - rx0)
ybase = 800
# axis
add(f'<line x1="{rx0}" y1="{ybase}" x2="{rx1}" y2="{ybase}" stroke="{INK2}" stroke-width="2"/>')
v = lo
while v <= hi + 1e-9:
    add(f'<line x1="{X(v):.1f}" y1="{ybase}" x2="{X(v):.1f}" y2="{ybase + 8}" stroke="{INK2}" stroke-width="2"/>')
    text(X(v), ybase + 28, f'{v:+.2f}'.replace('+0.00', '0'), 14, INK2, 'middle')
    v = round(v + 0.05, 2)
text(rx0, ybase + 56, '<- robot\'s RIGHT', 15, INK2, 'start', 'bold')
text(rx1, ybase + 56, 'robot\'s LEFT ->', 15, INK2, 'end', 'bold')
text((rx0 + rx1) / 2, ybase + 56, 'walker offset from the robot-goal line at the first sample (m)', 15, INK2, 'middle')
# threshold lines
ylo = top2 + 40
for th, lab, col, dash, w in ((0.30, '0.30  (5 Oct): sends 5 Oct to RIGHT again', MUTED, '6 5', 2),
                              (0.20, '0.20  proposed: splits the two groups', INK, '', 3),
                              (0.15, '0.15  now: sends two centred walkers LEFT', MUTED, '6 5', 2)):
    xx = X(-th)
    d = f' stroke-dasharray="{dash}"' if dash else ''
    add(f'<line x1="{xx:.1f}" y1="{ylo + 12}" x2="{xx:.1f}" y2="{ybase}" stroke="{col}" stroke-width="{w}"{d}/>')
labs = {0.30: (ylo + 4, 'end'), 0.20: (ylo - 14, 'middle'), 0.15: (ylo + 4, 'start')}
for th, lab in ((0.30, '0.30 (5 Oct value): 5 Oct back to RIGHT  x'), (0.20, '0.20 proposed: splits the groups  ok'),
                (0.15, 'x  0.15 now: run 06, rec15 03 go LEFT')):
    y, a = labs[th]
    xx = X(-th) + (-8 if a == 'end' else (8 if a == 'start' else 0))
    text(xx, y, lab, 14, INK if th == 0.20 else INK2, a, 'bold' if th == 0.20 else 'normal')
# points (value, label, wanted side); stacked where they would overlap
pts = [(-0.33, 'rec15 01', 'L'), (-0.26, '5 Oct', 'L'), (-0.20, 'run 05', 'L'), (-0.16, 'run 06', 'R'),
       (-0.16, 'rec15 03', 'R'), (-0.13, 'run 02', 'R'), (-0.06, 'run 01', 'R'), (0.00, 'rec15 02', 'R'),
       (0.01, 'run 04', 'R'), (0.03, 'rec15 04', 'R'), (0.15, 'run 03', 'R')]
placed = []
for v, lab, side in sorted(pts, key=lambda p: p[0]):
    x = X(v); lvl = 0
    while any(abs(x - px) < 22 and l == lvl for px, l in placed):
        lvl += 1
    placed.append((x, lvl))
    y = ybase - 36 - lvl * 44
    col = ORANGE if side == 'L' else BLUE
    add(f'<circle cx="{x:.1f}" cy="{y}" r="9" fill="{col}" stroke="{SURF}" stroke-width="2"/>')
    text(x, y - 16, lab, 12, INK, 'middle')
# legend
lx, ly = 150, 940
add(f'<circle cx="{lx}" cy="{ly - 5}" r="8" fill="{ORANGE}"/>')
text(lx + 16, ly, 'should go LEFT (walker clearly on the robot\'s right; 5 Oct: Brian had to yield)', 14, INK)
add(f'<circle cx="{lx}" cy="{ly + 23}" r="8" fill="{BLUE}"/>')
text(lx + 16, ly + 28, 'should go RIGHT (walker centred; "run" = rec15 + noghost run of 9 Oct)', 14, INK)
text(W - 60, ly, 'Only holds if localization is right:', 14, INK2, 'end', 'bold')
text(W - 60, ly + 28, '2 deg heading error moves you 0.22 m at 6.4 m -> check_localization.py --fix', 14, INK2, 'end')
add('</svg>')
open(os.path.join(HERE, 'side_threshold.svg'), 'w').write('\n'.join(svg))
print('wrote side_threshold.svg')
