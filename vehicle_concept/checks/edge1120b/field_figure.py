"""Plot a matched driving replay on the camera overlay stored in the November 19 bag."""
import argparse
import csv
import io
import json
import sqlite3
import struct
from datetime import datetime, timedelta, timezone
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from PIL import Image


def plot(bag, old, new, targets, font, output):
    font_manager.fontManager.addfont(str(font))
    plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['axes.spines.top'] = plt.rcParams['axes.spines.right'] = False
    colours = ('#a85719', '#00756c')
    versions = ('V6.27.6', 'V6.27.7 分步回正')
    rows = [json.loads(p.read_bytes()) for p in (old, new)]
    frame = 27
    capture = rows[0][frame]['capture_ns']
    assert [a['capture_ns'] for a in rows[0]] == [b['capture_ns'] for b in rows[1]]
    with sqlite3.connect(bag.resolve().as_uri()+'?mode=ro&immutable=1',uri=True) as conn:
        packets = conn.execute('SELECT data FROM messages JOIN topics ON topics.id=messages.topic_id '
                               'WHERE topics.name=?', ('/concept/perception/overlay/compressed',))
        for packet, in packets:
            sec, nsec = struct.unpack_from('<iI',packet,4)
            if sec*10**9+nsec == capture:
                photo = Image.open(io.BytesIO(packet[packet.index(b'\xff\xd8\xff'):])).convert('RGB')
                break
        else:
            raise ValueError('camera sample absent from bag')
    fig = plt.figure(figsize=(11,7.7),layout='constrained')
    axes = fig.subplot_mosaic([['a','b'],['c','c']],gridspec_kw={'height_ratios':[1.22, .83]})
    fig.suptitle('左侧外缘已经可见时，车还在往哪边打？',fontsize=19, fontweight='bold')
    for dataset, key, name, col in zip(rows, ('a','b'), versions, colours):
        d = dataset[frame]['replay']; ax = axes[key]
        ax.imshow(photo)
        # These coordinates describe the stored virtual camera overlay.
        # The physical camera height was 0.55 m. These pixels were already warped in the car.
        path = [(330.01302+254.391622*x/z,181.14964+254.391622*.655/z)
                for z, x in d['selected_path'] if z > .3]
        if path:
            x,y=zip(*path)
            ax.plot(x,y,color='white',linewidth=4,alpha=.88)
            ax.plot(x,y,color=col,linewidth=2.2)
        # Manually read left edge at 5 m in this exact frame (306 +/- 7 pixels).
        ref_y=181.14964+254.391622*.655/5
        ax.errorbar([306],[ref_y],xerr=7,fmt='o',ms=4,color='#ffec78',capsize=3)
        ax.set(xlim=(0,639),ylim=(359,0));ax.axis('off')
        cmd=d['command']; yaw=cmd['angular_z_radps']
        ax.set_title('%s　%s %.3f rad/s　·　%.2f m/s' %
                     (name, '向左' if yaw>0 else '向右', abs(yaw),cmd['linear_x_mps']),color=col,fontsize=12.5)
    ax=axes['c']
    with targets.open(encoding='utf-8-sig', newline='') as stream:
        window = [r for r in csv.DictReader(stream) if r['run']=='170817' and 17 <= int(r['frame']) <= 37]
    t = [(rows[0][int(r['frame'])]['capture_ns']-capture)/1e9 for r in window]
    for field, name, col, style in (
        ('previous_target_difference','原包上一条已放行指令','#697983',':'),
        ('v6276_diff',versions[0],colours[0],'-'),
        ('early_diff','试验一：立即反打','#aa579e','--'),
        ('bounded_diff',versions[1],colours[1],'-'),
    ):
        ax.plot(t,[float(r[field]) for r in window],style,marker='.',color=col,label=name,lw=1.8,ms=6)
    ax.axhline(0,color='#45616c',lw=.8)
    ax.axvline(0,color='#657680',ls=':',lw=1)
    ax.set(xlabel='距第 27 帧的拍摄时间（秒）',ylabel='右轮目标 − 左轮目标（RPM）\n正值向左，负值向右')
    ax.grid(axis='y',alpha=.23);ax.legend(loc='upper right',ncol=2,frameon=False,fontsize=8)
    stamp=datetime.fromtimestamp(capture/1e9,timezone(timedelta(hours=9))).strftime('%H:%M:%S.%f')[:-3]
    ax.set_title('11 月 19 日 %s 同帧回放。上图曲线是预测路线，黄点是前方 5 m 的手读外缘。' % stamp,
                 loc='left',fontsize=11)
    fig.savefig(output,dpi=140,bbox_inches='tight',facecolor='white')
    plt.close(fig)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('bag','old','new','font','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--targets',type=Path,default=Path(__file__).with_name('field-609.csv'))
    plot(**vars(parser.parse_args()))
