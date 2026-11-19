import json
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
root=Path(__file__).resolve().parent
source=json.loads((root/'distance.json').read_text())
items=[v for v in source['range_check'] if v['frame'] in (287,288)]
report=Image.new('RGB',(640,2*(360+54)),(245,247,251))
draw=ImageDraw.Draw(report)
font=ImageFont.truetype('/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf',16)
for idx,row in enumerate(items):
    y=idx*(360+54)
    report.paste(Image.open(root/('frame-%04d.jpg'%row['frame'])),(0,y))
    u,v=row['uv'];v+=y
    draw.ellipse((u-5,v-5,u+5,v+5),outline='#68ffde',width=2)
    draw.text((12,y+362),f"Frame {row['frame']}   |   Rectified ground contact at ({u:.1f}, {v-y:.1f}) px",font=font,fill='#102035')
    draw.text((12,y+385),f"Forward depth   {row['old_contact']:.2f} m (previous)   /   {row['field_contact']:.2f} m (Honda plane)",font=font,fill='#102035')
report.save(root/'field-contacts.jpg',quality=85)
