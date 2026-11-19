"""Fixed overhead view of the driver's accumulated observations."""

import base64
import json
import math
import zlib

import numpy as np

from .qt import QtCore, QtGui, QtWidgets as W


ROAD = [(0, 0, 0, 0), (61, 132, 176, 255), (63, 156, 132, 255), (166, 198, 216, 255)]
PAINT = [(0, 0, 0, 0), (237, 132, 31, 255), (237, 132, 31, 255), (204, 125, 58, 180)]
CONE, MOVING, OPENING = '#ca4f2b', '#8648a0', '#7452af'


def unpack_snapshot(message):
    data = json.loads(message)
    if data.get('schema') != 'concept.lap-map.v1' or not 0 < data.get('cell_m', 0) <= 1.:
        raise ValueError('不支持的地图格式')
    for key, dtype in (('road', '<i4'), ('lines', '<i4'), ('track', '<f4')):
        packed = base64.b64decode(data[key], validate=True)
        try:
            data[key] = np.frombuffer(zlib.decompress(packed), dtype=dtype).reshape(-1, 3)
        except zlib.error as exc:
            raise ValueError('地图压缩数据无效') from exc
    if any(np.any((data[key][:, 2] < 1) | (data[key][:, 2] > 3)) for key in ('road', 'lines')):
        raise ValueError('地图图层无效')
    if len(data['pose']) != 3 or not np.isfinite(data['pose']).all() or not np.isfinite(data['track']).all():
        raise ValueError('地图坐标无效')
    return data


class MapCanvas(W.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(210, 160)
        self.data = self.raster = None

    def set_map(self, data):
        self.data = data
        cell = data['cell_m']
        coords = np.vstack((data['road'][:, :2], data['lines'][:, :2]))
        extras = np.asarray([[0., 0.], data['pose'][:2], *data['openings'],
                             *(item['position'] for item in data['static'] + data['moving'])])
        offsets = np.vstack((extras, data['track'][:, 1:3]))
        extents = np.vstack((coords, np.floor(offsets / cell).astype(np.int32)))
        low = extents.min(axis=0) - 5
        shape = extents.max(axis=0) + 6 - low
        stride = max(1, math.ceil(int(max(shape)) / 1200))
        height, width = [math.ceil(int(n) / stride) for n in shape]
        pixels = np.zeros((height, width, 4), np.uint8)
        for key, palette in (('road', ROAD), ('lines', PAINT)):
            cells = data[key]
            z, x = ((cells[:, :2] - low) // stride).T
            pixels[height - 1 - z, x] = np.asarray(palette, np.uint8)[cells[:, 2]]
        self.raster = QtGui.QImage(pixels.data, width, height, width * 4,
                                   QtGui.QImage.Format_RGBA8888).copy()
        self.bounds = (*map(float, low * cell), height * stride * cell, width * stride * cell)
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor('#fafbf8'))
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        if self.raster is None:
            painter.setPen(QtGui.QColor('#52685e'))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, '等待行驶中的地图数据')
            return
        low_z, low_x, height, width = self.bounds
        scale = min((self.width()-44)/width, (self.height()-44)/height)
        left = (self.width() - width*scale) / 2
        top = (self.height() - height*scale) / 2

        def point(z, x):
            return QtCore.QPointF(left + (x-low_x)*scale, top + (low_z+height-z)*scale)

        painter.drawImage(QtCore.QRectF(left, top, width*scale, height*scale), self.raster)
        data = self.data
        track = data['track']
        if len(track) > 1:
            step = max(1, len(track)//5000)
            painter.setPen(QtGui.QPen(QtGui.QColor('#25465b'), 1.3))
            painter.drawPolyline(QtGui.QPolygonF([point(z, x) for _, z, x in track[::step]] + [point(*track[-1, 1:])]))
        self.dot(painter, point(0., 0.), '#efcb60', 4.)
        for location in data['openings']:
            self.dot(painter, point(*location), OPENING, 4.)
        for item in data['static']:
            self.dot(painter, point(*item['position']), CONE, 4.7)
        for item in data['moving']:
            self.draw_moving(painter, item, point, data['track'][-1, 0] if len(track) else 0.)
        self.draw_car(painter, point(*data['pose'][:2]), data['pose'][2])

        metres = 10 if width > 45 else 5 if width > 18 else 1
        y = self.height() - 13
        painter.setPen(QtGui.QPen(QtGui.QColor('#243831'), 1.4))
        painter.drawLine(QtCore.QLineF(13., y, 13.+metres*scale, y))
        painter.drawText(QtCore.QPointF(13., y-4), f'{metres} m')

    @staticmethod
    def dot(painter, at, color, radius):
        painter.setPen(QtGui.QPen(QtGui.QColor('#ffffff'), 1.2))
        painter.setBrush(QtGui.QColor(color))
        painter.drawEllipse(at, radius, radius)

    @staticmethod
    def draw_moving(painter, item, point, last_t):
        color = QtGui.QColor(MOVING)
        if last_t-item['time'] > 2.:
            color.setAlpha(115)
        painter.setPen(QtGui.QPen(color, 1.4))
        painter.setBrush(QtCore.Qt.NoBrush)
        phi = np.linspace(0., 2.*np.pi, 21)
        unit = np.array([np.cos(phi), np.sin(phi)])
        for estimate in item.get('ranges', []):
            values, axes = np.linalg.eigh(estimate['covariance'])
            samples = (axes @ (np.sqrt(np.maximum(values, 0.))[:, None] * unit)).T + estimate['center']
            painter.drawPolygon(QtGui.QPolygonF([point(z, x) for z, x in samples]))
        MapCanvas.dot(painter, point(*item['position']), color, 3.2)

    @staticmethod
    def draw_car(painter, at, yaw):
        painter.save()
        painter.translate(at)
        painter.rotate(math.degrees(yaw))
        painter.setPen(QtGui.QPen(QtGui.QColor('#ffffff'), 1.2))
        painter.setBrush(QtGui.QColor('#122838'))
        painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(0, -10), QtCore.QPointF(6, 7),
                                            QtCore.QPointF(-6, 7)]))
        painter.restore()


class LapMapPanel(W.QFrame):
    def __init__(self, parent):
        super().__init__(parent)
        self.setObjectName('card')
        self.key = None
        shell = W.QVBoxLayout(self)
        shell.setContentsMargins(14, 9, 14, 9)
        self.paper = W.QWidget()
        self.paper.setStyleSheet('background: white')
        layout = W.QVBoxLayout(self.paper)
        layout.setContentsMargins(0, 0, 0, 0)
        heading = W.QLabel('累计赛道地图')
        heading.setObjectName('heading')
        layout.addWidget(heading)
        self.summary = W.QLabel('起点朝向为上')
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.canvas = MapCanvas()
        layout.addWidget(self.canvas, 1)
        legend = W.QLabel(
            '<span style="color:#3d84b0">■ 本车道</span>　<span style="color:#3f9c84">■ 邻道</span>　'
            '<span style="color:#ed841f">━ 标线</span>　'
            '<span style="color:#ca4f2b">● 静止桶</span>　<span style="color:#8648a0">○ 移动桶范围</span><br>'
            '黄点为起点 · 三角为车辆 · 紫点为场内开口')
        legend.setWordWrap(True)
        layout.addWidget(legend)
        shell.addWidget(self.paper, 1)
        footer = W.QHBoxLayout()
        self.connection = W.QLabel('等待地图服务')
        self.connection.setWordWrap(True)
        footer.addWidget(self.connection, 1)
        self.save_button = W.QPushButton('保存 PNG…')
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.save)
        footer.addWidget(self.save_button)
        shell.addLayout(footer)

    def clear_map(self):
        self.key = None
        self.canvas.data = self.canvas.raster = None
        self.canvas.update()
        self.summary.setText('起点朝向为上')
        self.connection.setText('等待第一帧有效地图')
        self.save_button.setEnabled(False)

    def show_map(self, message):
        data = unpack_snapshot(message)
        key = data['session'], data['stamp']
        if key != self.key:
            self.canvas.set_map(data)
            self.key = key
            self.summary.setText(f"累计 {data['distance_m']:.1f} m · {len(data['road']):,} 个路面格"
                                 f" · 静止桶 {len(data['static'])} · 移动桶 {len(data['moving'])}")
            self.save_button.setEnabled(True)
        self.connection.setText('实时累计 · 位置来自轮速和陀螺仪')

    def save(self):
        path, _ = W.QFileDialog.getSaveFileName(self, '保存累计赛道地图',
                                                'concept-track.png', 'PNG (*.png)')
        if path:
            if not path.lower().endswith('.png'):
                path += '.png'
            # Keep text and map details readable when the embedded panel is narrow.
            scale = max(1., 800 / self.paper.width())
            pixmap = QtGui.QPixmap(math.ceil(self.paper.width() * scale),
                                   math.ceil(self.paper.height() * scale))
            pixmap.setDevicePixelRatio(scale)
            self.paper.render(pixmap)
            if pixmap.save(path, 'PNG'):
                self.connection.setText('已保存 ' + path)
            else:
                W.QMessageBox.warning(self, '地图未保存', '无法写入这个 PNG 文件。')
