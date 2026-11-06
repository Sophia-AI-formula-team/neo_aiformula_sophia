"""Encode the full measured curriculum, selecting the first originally declared complete blind seed for film."""
import argparse
import json
from render import OUT,RAW,compose
from camera_training import BLIND

def main():
    p=argparse.ArgumentParser();p.add_argument("--force",action="store_true");args=p.parse_args()
    def record(run,name,title,cut=False):
        if args.force or not all((OUT/(name+s)).exists() for s in (".json",".mp4",".jpg")):
            compose(run,name,title,cut)
    def read(run):return json.loads((RAW/(run+".json")).read_bytes())
    cases=[read(f"rgb_test_{s}_learned") for s in BLIND]
    complete=next((r for r in cases if r["status"]=="lap"),None)
    if complete:
        record(complete["id"],"rgb-highlights","A4 视觉训练后 · 三处路障实速回放",True)
        record(complete["id"],"rgb-full","A4 视觉训练后 · 新起点完整障碍圈")
    failing=[r for r in cases if r["status"]!="lap"]
    if failing:
        failed=max(failing,key=lambda r:r["progress"])
        record(failed["id"],"rgb-blind-failure","A4 新布局盲测 · 行驶最远的未完圈试次",True)
    record("rgb_clear_6329_learned","rgb-clean","A4 视觉训练后 · 新起点无新增路障")
    candidate=read("rgb_train_g1_4_s0");filmed=read("final_a4_practice")
    assert candidate["rows"]==filmed["rows"] and candidate["request"]["weights"]==filmed["request"]["weights"]
    record("final_a4_practice","rgb-pretraining","A4 预训练候选复跑 · 横移路障处中止",True)
    for run,name,title,cut in (
        ("final_a4_48133","traffic-highlights","A4 障碍考核 · 三段实速回放",True),
        ("final_a4_48133","traffic-full","A4 带路障 · 随机起点完整一圈",False),
        ("final_a4_clear","camera-final-clear","A4 联合策略 · 无新增路障完整一圈",False),
        ("film_pretrain_traffic","pretraining-start","初始策略练习 · 双桶处中止",True),
        ("film_pretrain_fast","pretraining-fast","空场预训练 · 第一个提速候选",False),
        ("film_practice_winner","pretraining-winner","联合预训练 · 冻结策略复跑",True),
        ("final_a4_25004","camera-failure","A4 留出失败 · 绕开双桶后越界",True),
        ("camera_a4_learned_traffic","first-failure","首轮障碍考核 · 连续双桶碰撞",True),
        ("camera_a4_learned_clear","camera-clear","A4 首轮提速策略 · 完整一圈",False)):
        record(run,name,title,cut)
if __name__=="__main__":main()
