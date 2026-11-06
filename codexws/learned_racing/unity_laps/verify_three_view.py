"""Compare decoded panel pixels to the real source JPEGs referenced by a film receipt."""
import argparse
import hashlib
import json
from pathlib import Path

import av
from PIL import Image, ImageChops, ImageDraw, ImageStat


def verify(receipt_path, frames, a4):
    receipt = json.loads(receipt_path.read_text(encoding="utf8"))
    samples = {sample["film_frame"]: sample for sample in receipt["samples"]}
    results = []
    with av.open(str(receipt_path.with_suffix(".mp4"))) as video:
        for number, frame in enumerate(video.decode(video=0)):
            if number not in samples:
                continue
            sample = samples[number]
            rendered = frame.to_image()
            row = {"frame":number, "video_s":float(frame.time), "simulation_s":sample["simulation_s"]}
            for channel, x in (("chase",42), ("a4",1046)):
                item = sample[channel]
                path = frames/f"{item['id']:04d}c.jpg" if channel=="chase" else a4/f"{item['id']:05d}.jpg"
                if item["sha256"] is None:
                    row[channel] = {"missing_source":True}
                    continue
                if hashlib.sha256(path.read_bytes()).hexdigest()!=item["sha256"]:
                    raise ValueError(f"Source JPEG changed: {path}")
                with Image.open(path) as image:
                    expected = image.convert("RGB").resize((960,540), Image.Resampling.BICUBIC)
                # Exclude only the bottom area covered by the terminal banner on a stopped chase frame.
                height = 460 if channel=="chase" and receipt.get("end_included") and sample["simulation_s"]==receipt["end_s"] else 540
                actual = rendered.crop((x,173,x+960,173+height))
                delta = ImageChops.difference(actual, expected.crop((0,0,960,height)))
                mask = Image.new("L",delta.size,255)
                excluded = None
                if channel=="chase" and receipt.get("referee_inset"):
                    excluded=receipt["referee_inset"]["overlay_chase_pixel_box"]
                    left,top,right,bottom=excluded
                    ImageDraw.Draw(mask).rectangle((left,top,right-1,bottom-1),fill=0)
                comparison=ImageStat.Stat(delta,mask)
                error = sum(comparison.mean)/3
                if error > 8:
                    raise ValueError(f"Unexpected pixel error {error:.2f} at frame {number}, {channel}")
                row[channel] = {"source_time":item["time_s"], "source_frame":item["id"],
                                "decoded_mean_abs_rgb_error_0_255":round(error,3),
                                "pixels_compared":comparison.count[0], "excluded_referee_pixel_box":excluded}
            results.append(row)
    if len(results)!=len(samples):
        raise ValueError("Unable to decode all sampled video frames")
    return {"run":receipt["run"], "start_s":receipt["start_s"], "end_s":receipt["end_s"],
            "encoded":receipt["encoded"], "source_counts":receipt["unique_sources"],
            "max_display_age_s":receipt["max_display_age_s"], "verified_samples":results}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("receipt","frames","a4","out"):
        p.add_argument("--"+name, type=Path, required=True)
    args = p.parse_args()
    result = verify(args.receipt, args.frames, args.a4)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf8")
    print(f"Matched real chase and A4 pixels for {len(result['verified_samples'])} decoded frames.")


if __name__=="__main__":
    main()
