import importlib.util
from pathlib import Path
from types import SimpleNamespace
from zipfile import ZipFile
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from run_vehicle import INPUT, node_args, profile, routes


def test_live_routes_preserve_vehicle_topics_and_shadow_is_disjoint():
    live, shadow = routes(True), routes(False)
    assert set(live) == set(shadow)
    assert set(live.values()) == {
        "/aiformula_perception/road_detector/mask_image",
        "/aiformula_visualization/road_detector/annotated_mask_image",
        "/cone_mask", "/cone_bbox_image",
        "/aiformula_perception/yolop_a3/detections",
        "/aiformula_perception/yolop_a3/traffic_light/state",
        "/aiformula_perception/yolop_a3/red_pixels_count",
    }
    assert len(set(shadow.values())) == 7
    assert all(v.startswith("/a3_shadow/") for v in shadow.values())
    assert not set(live.values()) & set(shadow.values())


def test_bundles_have_same_weight_and_only_lite_parameter_differs(tmp_path):
    subprocess.run([sys.executable,"-B",str(ROOT/"tools/package_vehicle.py"),
                    "--out",str(tmp_path)],check=True)
    argv=[]
    for name in ("A4", "A4_lite"):
        with ZipFile(tmp_path/f"{name}-neo-vehicle.zip") as arc:
            arc.extractall(tmp_path)
        directory=tmp_path/name
        subprocess.run([sys.executable,"-B",str(directory/"run_vehicle.py"),
                        "--profile",name,"--identity"],check=True)
        config=profile(directory/"profile.json",name)
        other="A4" if name=="A4_lite" else "A4_lite"
        with pytest.raises(ValueError,match="does not match"):
            profile(directory/"profile.json",other)
        args=SimpleNamespace(device="cuda",score_threshold=.5,input_image_topic=INPUT,
                             use_sim_time="true",live=False)
        argv.append(node_args(args,config,ROOT/"weights/shared_best.pth"))
    assert [(a,b) for a,b in zip(*argv) if a!=b] == [("lean_runtime:=false","lean_runtime:=true")]
    for a in argv:
        assert len([v for v in a if ":=/a3_shadow/" in v]) == 7
