"""Publish an atomic, self-contained view of completed closed-loop probes."""
import argparse
from datetime import datetime
from html import escape
import json
from pathlib import Path
import re
import time


TRIAL = re.compile(r"tr(\d{3})_a(\d+)$")

ACTION_LABELS = {
    "keep_v2": "上一版控制",
    "hold_line_brisk": "沿线快行",
    "minimum_left_brisk": "小幅左避",
    "minimum_right_brisk": "小幅右避",
    "buffer_left_moderate": "左避减速",
    "buffer_right_moderate": "右避减速",
    "hold_line_yield": "沿线收油",
}


def columns(log, count):
    book = log.parent / "action_bank.json"
    if not book.is_file():
        return "".join(f'<th>A{i}</th>' for i in range(count))
    menu = json.loads(book.read_bytes())
    if len(menu["names"]) != count or len(menu["actions"]) != count:
        raise ValueError("Progress grid and recorded action bank disagree")
    header = []
    for arm, (name, spec) in enumerate(zip(menu["names"], menu["actions"])):
        caption = escape(ACTION_LABELS.get(name,name))
        detail = "继续上一版策略" if arm == 0 else f'动作速度上限 {spec[2]:.2f} m/s'
        if spec[0]:
            detail += f' · 净距规划值 {spec[1]:.2f} m'
        header.append(f'<th title="{detail}">A{arm}<br>{caption}</th>')
    return "".join(header)


def snapshot(log):
    finished, started = {}, set()
    if log.is_file():
        for line in log.read_bytes().splitlines():
            try:
                event = json.loads(line)
            except (UnicodeDecodeError, json.JSONDecodeError):
                # An active writer may still be appending the last event.
                continue
            key = event.get("trial", "")
            if not TRIAL.fullmatch(key):
                continue
            if event["kind"] == "start":
                started.add(key)
            if event["kind"] == "finish":
                finished[key] = event
    return finished, sorted(started - finished.keys())


def table(rows, active, layouts, actions):
    cells = []
    for layout in range(1, layouts + 1):
        row = []
        for arm in range(actions):
            key = f"tr{layout:03}_a{arm}"
            event = rows.get(key)
            kind = "waiting"
            caption = "等待"
            detail = key
            if event:
                passed = event["status"] in ("segment", "lap")
                kind = "ok" if passed else "fail"
                caption = {"segment":"完段","lap":"完圈","obstacle":"碰桶","boundary":"出界",
                           "lost_lane":"丢线","stalled":"停滞"}.get(event["status"], event["status"])
                gap = event.get("minGap")
                if gap is not None and gap<90:
                    detail += f" · 最近桶距 {gap:.2f} m"
                detail += f" · {event['progress']:.1f} m · {event['seconds']:.2f} s"
            elif key in active:
                kind, caption = "active", "运行中"
            row.append(f'<td class="{kind}" title="{escape(detail)}">{escape(caption)}</td>')
        cells.append(f'<tr><th>{layout:02}</th>{"".join(row)}</tr>')
    return "".join(cells)


def publish(path, log, layouts, actions, watch, stop_at):
    previous = None
    planned = layouts * actions
    path.parent.mkdir(parents=True, exist_ok=True)
    while True:
        rows, active = snapshot(log)
        counting = len(rows) < planned and watch and time.monotonic() < stop_at
        signature = (len(rows), tuple(active), counting)
        if signature != previous:
            passed = sum(e["status"] in ("segment", "lap") for e in rows.values())
            timestamp = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")
            refresh = '<meta http-equiv="refresh" content="12">' if counting else ''
            arm_labels = columns(log, actions)
            percent = 100*len(rows)/planned
            page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">{refresh}<title>Unity 长训练记录</title>
<style>
*{{box-sizing:border-box}}body{{font:16px system-ui,"Microsoft YaHei",sans-serif;color:#203a3b;background:#f6f7f1;margin:0}}
main{{max-width:920px;margin:auto;padding:34px 20px}}h1{{font-size:clamp(30px,6vw,49px);margin:.1em 0 .2em}}p{{color:#48625f;line-height:1.6}}
header{{border-bottom:1px solid #b8c7bf;padding-bottom:8px}}.numbers{{display:flex;gap:24px;flex-wrap:wrap;margin:22px 0 14px}}
.numbers b{{display:block;font:600 32px ui-monospace,monospace}}progress{{width:100%;height:12px;accent-color:#26715e}}
table{{border-collapse:separate;border-spacing:3px;width:100%;text-align:center;font-size:14px}}thead th{{position:sticky;top:0;background:#f6f7f1;padding:8px 0}}
td{{border-radius:4px;padding:5px 0}}th{{color:#415552}}.ok{{background:#cee9dc;color:#20503e}}.fail{{background:#f5d7cb;color:#702d20}}
.waiting{{background:#e9ece5;color:#737e74}}.active{{background:#e6b94f;color:#412f00}}small{{color:#53655e}}
@media(max-width:520px){{table{{font-size:12px}}main{{padding:22px 8px}}.numbers{{gap:13px}}.numbers b{{font-size:25px}}}}
</style><main><header><small>AI FORMULA · UNITY / YOLOP A4</small><h1>长训练试跑记录</h1>
<p>每行是一种训练布局，每列是一种动作。格子只在 Unity 实际回传结果后记为完成，悬停可查看净距、距离和用时。</p></header>
<section class="numbers"><div><b>{len(rows)} / {planned}</b>已返回试跑</div><div><b>{passed}</b>完成路段</div>
<div><b>{len(rows)-passed}</b>提前终止</div></section><progress value="{percent:.4f}" max="100"></progress>
<p>七个动作里有意包含冒险的试探。这个表统计训练材料，冻结策略的通过率另做盲测。净距超过 0.5 m 时收益封顶；方向修正和直路 4 m/s 巡航分别测量。</p>
<p><small>记录更新于 {escape(timestamp)}</small></p><table><thead><tr><th>布局</th>{arm_labels}</tr></thead>
<tbody>{table(rows,active,layouts,actions)}</tbody></table></main></html>'''
            tmp = path.with_suffix(".partial.html")
            tmp.write_bytes(page.encode("utf8"))
            tmp.replace(path)
            previous = signature
        if not counting:
            break
        time.sleep(5)


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("log", type=Path)
    args.add_argument("output", type=Path)
    args.add_argument("--layouts", type=int, required=True)
    args.add_argument("--actions", type=int, required=True)
    args.add_argument("--watch", action="store_true")
    options = args.parse_args()
    publish(options.output, options.log, options.layouts, options.actions, options.watch,
            time.monotonic()+4*3600)
