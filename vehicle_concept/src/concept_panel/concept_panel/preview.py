"""Render an explicitly offline sample with a local camera recording, without DDS."""
import argparse
import sys
from pathlib import Path

from .qt import QtGui,QtWidgets
from .window import Window


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    parser.add_argument('--output', required=True)
    args=parser.parse_args()
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    # Qt's Windows offscreen backend may start without any registered system fonts.
    if sys.platform=='win32' and not QtGui.QFontDatabase.families():
        for font in ('msyh.ttc','msyhbd.ttc'):
            QtGui.QFontDatabase.addApplicationFont(str(Path('C:/Windows/Fonts')/font))
    window=Window(offline=True)
    window.show()
    app.processEvents()
    img=QtGui.QImage(args.image)
    if img.isNull():
        raise ValueError('摄像头文件无法读取')
    window.image(img,100.,100.)
    window.gate.put(dict(mode='paused',reason='operator_pause',linear_mps=0.,max_auto_speed_mps=1.,
                         output_enabled=False,paused=True),100.,100.)
    window.decision.put(dict(status='shadow',reason='离线示例 · 录制图像',feasible_count=34,
                             candidates=[None]*75,speed_estimate_mps=.82,target_speed_mps=3.4),100.,100.)
    window.log_event('操作','离线预览已载入')
    for _ in range(3):
        window.log_event('决策','示例状态 · 影子模式')
    window.log_event('门控','示例状态 · 自动已暂停')
    window.pending_text='离线示例，控制服务未连接'
    window.refresh(100.04,100.04)
    app.processEvents()
    window.draw_image()
    app.processEvents()
    if not window.grab().save(args.output):
        raise RuntimeError('截图未保存')


if __name__=='__main__':
    main()
