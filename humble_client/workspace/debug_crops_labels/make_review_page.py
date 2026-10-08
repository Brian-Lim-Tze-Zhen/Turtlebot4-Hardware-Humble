#!/usr/bin/env python3
"""make_review_page.py - one HTML page to check the facing labels of Table 6.1 (8 Oct 2026).

The labels in labels.csv were assigned by Claude from the contact sheets (2 Oct 2026, header
"to be checked"); this page lets the author check every one of them. It shows each crop large,
numbered in a shuffled order, WITHOUT its filename (the filename ends with the pose rule's own
answer) and without any classifier output. The current label is shown only as a hint; every crop
needs an explicit click (facing / not facing / no pair / can't tell). The unlabelled crops of
unsure.csv follow in a second, optional section. "Download" saves labels_checked_by_author.csv:
    filename,author_label,claude_label,section
Output: review_labels.html next to this script (self-contained, images embedded).
Usage:  python3 make_review_page.py
"""
import base64
import csv
import html
import json
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
CROPS = os.path.join(HERE, "..", "debug_crops")


def read(path, with_label):
    rows = []
    for r in csv.reader(open(path)):
        if not r or r[0].startswith("#"):
            continue
        rows.append((r[0].strip(), r[1].strip() if with_label else ""))
    return rows


labelled = read(os.path.join(HERE, "labels.csv"), True)
unsure = read(os.path.join(HERE, "unsure.csv"), False)
rng = random.Random(20261008)          # fixed shuffle: order does not follow the sheets
rng.shuffle(labelled)
rng.shuffle(unsure)

items = []
for section, rows in (("labelled", labelled), ("unsure", unsure)):
    for fn, lab in rows:
        with open(os.path.join(CROPS, fn), "rb") as f:
            b64 = base64.b64encode(f.read()).decode()
        items.append({"fn": fn, "claude": lab, "section": section, "img": "data:image/jpeg;base64," + b64})

page = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Facing Label Check</title>
<style>
:root{--bg:#f7f7f5;--card:#fff;--ink:#1d1d1b;--mute:#6b6b66;--line:#dcdcd6;--acc:#2f5fd0;--ok:#1f7a4a;--chg:#b5520f;--sel:#e8eefc}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#161615;--card:#1f1f1d;--ink:#ecece8;--mute:#a3a39c;--line:#3a3a36;--acc:#8fb0ff;--ok:#6fd19c;--chg:#f0a060;--sel:#24304d}}
:root[data-theme=dark]{--bg:#161615;--card:#1f1f1d;--ink:#ecece8;--mute:#a3a39c;--line:#3a3a36;--acc:#8fb0ff;--ok:#6fd19c;--chg:#f0a060;--sel:#24304d}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.45 system-ui,-apple-system,Segoe UI,sans-serif}
header{position:sticky;top:0;z-index:2;background:var(--bg);border-bottom:1px solid var(--line);padding:12px 16px;display:flex;flex-wrap:wrap;gap:10px 18px;align-items:center}
h1{font-size:17px;margin:0;font-weight:650}
.prog{color:var(--mute);font-variant-numeric:tabular-nums}
.prog b{color:var(--ink)}
button{font:inherit;cursor:pointer;border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:6px;padding:6px 10px}
button.primary{background:var(--acc);border-color:var(--acc);color:#fff}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]) button.primary{color:#111}}
main{padding:16px;max-width:1200px;margin:0 auto}
.note{color:var(--mute);margin:0 0 16px;max-width:75ch}
h2{font-size:15px;margin:28px 0 10px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:14px}
.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px;display:flex;flex-direction:column;gap:8px}
.card.done{border-color:var(--ok)}
.card.changed{border-color:var(--chg)}
.top{display:flex;justify-content:space-between;color:var(--mute);font-size:13px}
.num{font-weight:650;color:var(--ink)}
.img{display:flex;justify-content:center;align-items:center;background:#000;border-radius:4px;min-height:200px}
.img img{max-width:100%;height:auto;width:auto;max-height:320px;image-rendering:auto;min-height:200px;object-fit:contain}
.opts{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.opts button{padding:7px 6px;font-size:14px}
.opts button.on{background:var(--sel);border-color:var(--acc);font-weight:600}
.state{font-size:13px;min-height:1.2em}
.state.ok{color:var(--ok)} .state.chg{color:var(--chg)}
.hint{font-size:13px;color:var(--mute)}
</style></head><body>
<header>
  <h1>Facing label check</h1>
  <span class="prog">Labelled: <b id="pl">0</b> / <b id="nl">0</b> checked · changed <b id="pc">0</b> · optional: <b id="pu">0</b> / <b id="nu">0</b></span>
  <span style="flex:1"></span>
  <button id="dl" class="primary">Download CSV</button>
</header>
<main>
<p class="note">Decide for each crop whether the two people face each other. The grey hint is the earlier label; click your own answer even when you agree, so every label is checked. Crops are shuffled and unnamed on purpose. Progress is kept in this browser. When done, click <b>Download CSV</b>.</p>
<h2>Labelled crops (used in Table 6.1)</h2>
<div class="grid" id="g1"></div>
<h2>Optional: crops left unlabelled ("could not tell")</h2>
<div class="grid" id="g2"></div>
</main>
<script>
const ITEMS = __ITEMS__;
const OPTS = [["facing","Facing"],["not_facing","Not facing"],["no_pair","No pair"],["cant_tell","Can't tell"]];
const NAMES = Object.fromEntries(OPTS);
const KEY = "facing-label-check-v1";
let ans = {};
try { ans = JSON.parse(localStorage.getItem(KEY) || "{}"); } catch (e) { ans = {}; }
function save(){ try { localStorage.setItem(KEY, JSON.stringify(ans)); } catch (e) {} }
let n1 = 0, n2 = 0;
ITEMS.forEach(it => {
  const num = it.section === "labelled" ? ++n1 : "U" + (++n2);
  const c = document.createElement("div"); c.className = "card";
  c.innerHTML = `<div class="top"><span class="num">#${num}</span><span class="hint">${it.claude ? "earlier label: " + NAMES[it.claude] : "no earlier label"}</span></div>
    <div class="img"><img alt="crop ${num}" src="${it.img}"></div><div class="opts"></div><div class="state"></div>`;
  const opts = c.querySelector(".opts"), st = c.querySelector(".state");
  function paint(){
    const a = ans[it.fn];
    opts.querySelectorAll("button").forEach(b => b.classList.toggle("on", b.dataset.v === a));
    c.classList.toggle("done", !!a && (!it.claude || a === it.claude));
    c.classList.toggle("changed", !!a && !!it.claude && a !== it.claude);
    st.className = "state" + (a ? (it.claude && a !== it.claude ? " chg" : " ok") : "");
    st.textContent = !a ? "not checked yet" : (it.claude && a !== it.claude ? `changed: ${NAMES[it.claude]} → ${NAMES[a]}` : "checked");
  }
  OPTS.forEach(([v, t]) => {
    const b = document.createElement("button"); b.textContent = t; b.dataset.v = v;
    b.onclick = () => { ans[it.fn] = v; save(); paint(); count(); };
    opts.appendChild(b);
  });
  paint();
  document.getElementById(it.section === "labelled" ? "g1" : "g2").appendChild(c);
});
function count(){
  const L = ITEMS.filter(i => i.section === "labelled"), U = ITEMS.filter(i => i.section !== "labelled");
  document.getElementById("pl").textContent = L.filter(i => ans[i.fn]).length;
  document.getElementById("nl").textContent = L.length;
  document.getElementById("pc").textContent = L.filter(i => ans[i.fn] && ans[i.fn] !== i.claude).length;
  document.getElementById("pu").textContent = U.filter(i => ans[i.fn]).length;
  document.getElementById("nu").textContent = U.length;
}
count();
document.getElementById("dl").onclick = () => {
  const lines = ["filename,author_label,claude_label,section"];
  ITEMS.forEach(i => lines.push([i.fn, ans[i.fn] || "", i.claude, i.section].join(",")));
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([lines.join("\n") + "\n"], {type: "text/csv"}));
  a.download = "labels_checked_by_author.csv"; a.click();
};
</script></body></html>"""
out = os.path.join(HERE, "review_labels.html")
open(out, "w").write(page.replace("__ITEMS__", json.dumps(items)))
print(f"wrote {out}: {len(labelled)} labelled + {len(unsure)} unsure crops")
