"""CPU-rendered rigid IMU pose; Hamilton wxyz maps body into world."""
import math
import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or not .5 <= norm <= 1.5:
        raise ValueError('四元数无效或模长异常')
    w,x,y,z = q / norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


class ImuCanvas(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(320,280)
        self.matrix = np.eye(3)
        self.has_pose = False
        self.fresh = False
        self.view = '立体'
        self.setAccessibleName('IMU长方体姿态，X前、Y左、Z上')

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(),QtGui.QColor('#111b2b'))
        bases = {'立体':[[.6,-.8,0],[.22,.165,-.95],[.76,.57,.275]],
                 '正视':[[0,-1,0],[0,0,-1],[1,0,0]],
                 '侧视':[[1,0,0],[0,0,-1],[0,-1,0]],
                 '俯视':[[0,-1,0],[-1,0,0],[0,0,1]]}
        basis = np.array(bases[self.view])
        scale = min(self.width()/3.7,self.height()/2.9)
        center = np.array([self.width()*.5,self.height()*.51])

        def project(v):
            p = basis @ np.asarray(v)
            xy = center+p[:2]*scale
            return QtCore.QPointF(*xy),p[2]

        def line(a,b,color,width=1):
            painter.setPen(QtGui.QPen(QtGui.QColor(color),width))
            painter.drawLine(project(a)[0],project(b)[0])

        for i in range(-5,6):
            line([i*.25,-1.25,-.7],[i*.25,1.25,-.7],'#253348')
            line([-1.25,i*.25,-.7],[1.25,i*.25,-.7],'#253348')
        # Fixed reference axes remain visible independently of the measured pose.
        anchor = np.array([-1.,.8,-.7])
        for name,v,color in [('X',[.35,0,0],'#ff8b94'),('Y',[0,.35,0],'#82d9a0'),('Z',[0,0,.35],'#45d5ff')]:
            end = anchor+v
            line(anchor,end,color)
            painter.drawText(project(end)[0]+QtCore.QPointF(4,-4),name+' 世界')
        if self.has_pose:
            vertices = np.array([[sx*.65,sy*.425,sz*.175] for sz in (-1,1) for sy in (-1,1) for sx in (-1,1)]) @ self.matrix.T
            definitions = [(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)]
            colors = ['#3d5871','#86a6bf','#597e9f','#496986','#415d79','#45d5ff']
            faces = []
            for ids,color in zip(definitions,colors):
                points = [project(vertices[i]) for i in ids]
                faces.append((sum(p[1] for p in points)/4,[p[0] for p in points],color))
            painter.setOpacity(1. if self.fresh else .4)
            for _,points,color in sorted(faces,key=lambda face:face[0]):
                painter.setPen(QtGui.QPen(QtGui.QColor('#182b40'),1))
                painter.setBrush(QtGui.QColor(color))
                painter.drawPolygon(QtGui.QPolygonF(points))
            painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            for name,v,color in [('X',[1.05,0,0],'#ff8b94'),('Y',[0,.85,0],'#82d9a0'),('Z',[0,0,.85],'#45d5ff')]:
                end = self.matrix @ v
                line([0,0,0],end,color,2)
                painter.drawText(project(end)[0]+QtCore.QPointF(5,-5),name)
            painter.setOpacity(1.)
        painter.setPen(QtGui.QColor('#aabdd5'))
        painter.drawText(16,25,'IMU 长方体 · 绕中心旋转' if self.has_pose else '等待有效 IMU 姿态数据')


class ImuPanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        self.state = QtWidgets.QLabel('等待有效 IMU 数据')
        layout.addWidget(self.state)
        row = QtWidgets.QHBoxLayout()
        layout.addLayout(row,1)
        self.canvas = ImuCanvas()
        row.addWidget(self.canvas,1)
        side = QtWidgets.QVBoxLayout()
        row.addLayout(side)
        self.angles = QtWidgets.QLabel()
        self.quaternion = QtWidgets.QLabel()
        self.gyro = QtWidgets.QLabel()
        for title,widget in [('硬件姿态角（ZYX，°）',self.angles),('四元数 wxyz（原始）',self.quaternion),('机身角速度（rad/s）',self.gyro)]:
            side.addWidget(QtWidgets.QLabel(title))
            widget.setMinimumWidth(210)
            widget.setStyleSheet('font-size: 16px; padding: 8px 0;')
            side.addWidget(widget)
        legend = QtWidgets.QLabel('X 前 · Y 左 · Z 上\n青色端面：X 正方向\n长方体比例仅为示意\n姿态角与网络观测倾角定义不同')
        legend.setWordWrap(True)
        side.addWidget(legend)
        side.addStretch()
        buttons = QtWidgets.QHBoxLayout()
        layout.addLayout(buttons)
        group = QtWidgets.QButtonGroup(self)
        self.view_buttons = {}
        for name in ('立体','正视','侧视','俯视'):
            button = QtWidgets.QPushButton(name)
            button.setCheckable(True)
            button.setChecked(name == '立体')
            button.clicked.connect(lambda _,v=name:self.set_view(v))
            group.addButton(button)
            self.view_buttons[name] = button
            buttons.addWidget(button)
        buttons.addStretch()
        self.reset()

    def set_view(self, view):
        self.canvas.view = view
        self.canvas.update()

    def reset(self):
        self.canvas.has_pose = self.canvas.fresh = False
        self.invalidate('等待有效 IMU 数据')

    def invalidate(self, reason):
        self.state.setText(reason+' · 模型冻结' if self.canvas.has_pose else reason)
        self.state.setStyleSheet('color: #ffcb6b;')
        self.angles.setText('Roll　—\nPitch　—\nYaw　—')
        self.quaternion.setText('w　—\nx　—\ny　—\nz　—')
        self.gyro.setText('X　—\nY　—\nZ　—')
        self.canvas.fresh = False
        self.canvas.update()

    def set_frame(self, frame, stale=False, paused=False):
        if stale:
            self.invalidate('数据过期或连接断开')
            return
        if not all(frame.available(f'body_quat_{i}') for i in range(4)):
            self.invalidate('IMU 姿态无效')
            return
        q = [frame.values[f'body_quat_{i}'] for i in range(4)]
        try:
            matrix = rotation_matrix(q)
        except ValueError:
            self.invalidate('四元数无效或模长异常')
            return
        if paused:
            self.state.setText('姿态显示已暂停 · 接收和记录继续')
            return
        roll = math.atan2(matrix[2,1],matrix[2,2])
        pitch = math.asin(float(np.clip(-matrix[2,0],-1,1)))
        yaw = math.atan2(matrix[1,0],matrix[0,0])
        self.angles.setText('\n'.join(f'{name}　{math.degrees(value):+.2f}°' for name,value in zip(('Roll','Pitch','Yaw'),(roll,pitch,yaw))))
        self.quaternion.setText('\n'.join(f'{name}　{value:+.5f}' for name,value in zip('wxyz',q)))
        self.gyro.setText('\n'.join(f'{name}　{frame.values[key]:+.4f}' if frame.available(key) else f'{name}　不可用' for name,key in zip('XYZ',[f'body_gyro_rad_s_{i}' for i in range(3)])))
        self.canvas.matrix = matrix
        self.canvas.has_pose = self.canvas.fresh = True
        self.state.setText('IMU 有效 · 绝对姿态 · 硬件机身坐标')
        self.state.setStyleSheet('color: #82d9a0;')
        self.canvas.update()
