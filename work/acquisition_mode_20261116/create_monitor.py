from pathlib import Path
s=Path('host/control_mode1_monitor/main.py').read_text(encoding='utf-8-sig')
a=s.index('from protocol import'); b=s.index('class Receiver',a)
s=s[:a]+'''from protocol import Parser, Timing, KEYS, LABELS, CATEGORIES, CSV_COLUMNS, csv_row, simulated_packet

def label(key): return LABELS[key]

'''+s[b:]
s=s.replace('deadline += .02','deadline += .01').replace('控制模式1 · 神经网络监测（只接收）','模式20 · 数据采集（预留串口1，只接收）').replace('实时信号监视器 · 控制模式1','实时数据采集 · 模式20').replace('self.history = deque(maxlen=6000)','self.history = deque(maxlen=12000)').replace('self.history_limit.setValue(.6)','self.history_limit.setValue(1.2)')
s=s.replace('仅模式1上传，普通模式无数据。所有左右均为穿戴者视角。','模式20且 data1.bit0=1 时100Hz上传。左右为穿戴者视角。')
s=s.replace("self.flags = QtWidgets.QLabel('有效位：—')","self.flags = QtWidgets.QLabel('帧无设备时间戳、序号、有效位和CRC；零值可能是失效占位，主机无法判定传感器健康。')\n        self.flags.setWordWrap(True)")
s=s.replace("        self.imu = ImuPanel()\n        self.tabs.addTab(self.imu,'IMU 姿态')\n",'')
s=s.replace("self.plot.setLabel('bottom','设备累计时间',units='s')","self.plot.setLabel('bottom','主机接收相对时间',units='s')")
a=s.index("        categories = {"); b=s.index('        self.readouts',a)
s=s[:a]+'        categories = CATEGORIES\n'+s[b:]
s=s.replace('enumerate(KEYS[1:])','enumerate(KEYS)')
a=s.index('            check.setChecked(key in'); b=s.index('            self.checks[key]',a)
s=s[:a]+"            check.setChecked(key in ('left_motor_angle','right_motor_angle','left_thigh_angle','right_thigh_angle','left_shank_angle','right_shank_angle'))\n"+s[b:]
s=s.replace('self.groups.setCurrentIndex(2)','self.groups.setCurrentIndex(0)').replace('value*50','value*100').replace("        self.imu.invalidate('连接已断开')\n",'').replace('        self.imu.reset()\n','').replace('mode1_record.csv','mode20_record.csv').replace('mode1_cache.csv','mode20_cache.csv')
a=s.index("            self.stats.setText(f'等待模式1"); b=s.index('        if self.paused:',a)
s=s[:a]+'''            self.stats.setText(f'等待模式20采集数据：请在遥控端选择20并打开bit0 · 收到帧 {rx.count} · 错帧候选 {rx.parser.bad} · 丢弃字节 {rx.parser.discarded}')
            return
        f = self.history[-1][2]
        elapsed = max(.001,self.history[-1][0])
        self.stats.setText(f'本机接收编号 {f.receive_index} · 接收 {rx.count} 帧 · 会话平均 {(rx.count-1)/elapsed:.1f} 帧/秒 · UI队列溢出 {rx.overflow} · 错帧候选 {rx.parser.bad} · 丢弃字节 {rx.parser.discarded} · 已记录 {rx.recorded}\\n'+('超过1秒无帧或连接断开' if stale else '正在接收（不代表各传感器有效）')+(' · 显示暂停，接收和记录继续' if self.paused else ''))
'''+s[b:]
s=s.replace("f'{f.values[key]:+.4f}' if not stale and f.available(key) else '不可用'","(str(f.values[key]) if key.endswith('pressure') else f'{f.values[key]:+.4f}') if not stale and f.available(key) else '不可用'")
s=s.replace("            window.tabs.setCurrentIndex(1)\n            app.processEvents()\n            window.grab().save(str(Path(__file__).with_name('imu_preview.png')))\n",'')
s=s.replace("模拟运行3秒、截图和CSV，自动退出","模拟运行3秒、截图和CSV，自动退出")
Path('host/acquisition_monitor/main.py').write_text(s,encoding='utf-8')
