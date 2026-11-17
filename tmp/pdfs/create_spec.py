from pathlib import Path
import sys, csv
import numpy as np
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Preformatted
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
sys.path.insert(0,str(Path('host/acquisition_monitor').resolve()))
from protocol import KEYS, LABELS, Parser
from log_data import load_log
root=Path('caiji_imu_data')
pdfmetrics.registerFont(TTFont('CN','C:/Windows/Fonts/msyh.ttc'))
S=getSampleStyleSheet()
for name in ['Normal','Title','Heading1','Heading2']:
 S[name].fontName='CN'
S['Normal'].fontSize=9; S['Normal'].leading=15
S['Title'].fontSize=23; S['Title'].leading=32
S['Heading1'].fontSize=16; S['Heading1'].leading=24
S['Heading2'].fontSize=11; S['Heading2'].leading=18
S.add(ParagraphStyle(name='SmallCN',fontName='CN',fontSize=8,leading=12))
story=[]
def p(t,style='Normal'): story.append(Paragraph(t,S[style])); story.append(Spacer(1,7))
def table(rows,widths):
 t=Table([[Paragraph(str(x),S['SmallCN']) for x in r] for r in rows],colWidths=widths,repeatRows=1)
 t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dceaf5')),('VALIGN',(0,0),(-1,-1),'TOP'),('GRID',(0,0),(-1,-1),.35,colors.HexColor('#c7d2dc')),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5)]))
 story.append(t); story.append(Spacer(1,10))
p('模式20 · 数据格式说明','Title')
p('原始采集日志 / CSV / NumPy NPZ','Heading2')
p('版本日期：2026-11-17　适用协议：预留串口1，80字节、带微秒时间戳版本')
p('1. 文件与采集概览','Heading1')
p('本说明依据当前主板固件及采集上位机源码编写。左右均指穿戴者视角。模式20用于数据采集，不下发调试或策略扭矩，不触发置零，电机继续原有静止查询。')
table([['文件类型','内容与用途'],['LOG*.TXT','二进制字节流，扩展名虽为TXT，内容不是文本表格；应以 rb 方式读取。'],['LOG00082.csv','上位机离线转换结果，UTF-8 BOM，20列，每行一帧。'],['LOG00082.npz','压缩NumPy数组，包含时间、17通道矩阵、独立通道及元数据。']],[105,390])
p('采集条件：遥控端 data2=20 且 data1.bit0=1；关闭bit0或退出模式20后停止新帧。串口1为JP4/UART4，921600波特率、8N1，目标上传频率100Hz，周期约10ms。连接电脑上位机本身不会开启采集。')
p('2. 文件夹内原始日志检查','Heading1')
rows=[['文件','字节数','80字节帧数','丢弃 / 剩余字节']]
for f in sorted(root.glob('*.TXT')):
 parser=Parser(); data=f.read_bytes(); frames=parser.feed(data)
 rows.append([f.name,len(data),len(frames),f'{parser.discarded} / {len(parser.buffer)}'])
table(rows,[125,105,110,155])
p('上述统计按当前解析器检查帧头、固定长度、帧尾及压力码范围。匹配到合法结构不代表所有传感器有效，也不能证明字节内容没有错误；无法解析部分不据此推断为旧协议。','SmallCN')
story.append(PageBreak())
p('3. 原始二进制帧布局','Heading1')
p('每帧80字节，小端序（Little-endian）。uint32为4字节无符号整数，float32为IEEE 754单精度浮点数。偏移从0起算，所有字段均占4字节，没有额外填充。')
rows=[['偏移','字段名','类型','含义 / 单位'],[0,'start_flag','uint32','帧头 0xAAAAAAAA'],[4,'timestamp_us','uint32','主板快照时刻 / 微秒']]
for i,k in enumerate(KEYS):
 rows.append([8+4*i,k,'uint32' if k.endswith('pressure') else 'float32',LABELS[k].replace('–','-')])
rows.append([76,'end_flag','uint32','帧尾 0xEEEEEEEE'])
table(rows,[35,170,65,225])
p('Python struct格式：&lt;2I11fI4fII；帧头字节 AA AA AA AA，帧尾字节 EE EE EE EE。两个压力字段位于偏移52、72，均为整数，不能按float32读取。','SmallCN')
story.append(PageBreak())
p('4. 通道含义与方向约定','Heading1')
p('大腿/小腿X角度使用对应级联板的Roll；X角速度使用对应陀螺仪X轴。主板已完成方向及零点修正，导出后不要再次执行同样修正。')
table([['通道','当前上传值'],['左大腿、左小腿角度','原始Roll - π/2（rad）'],['右大腿、右小腿角度','-原始Roll - π/2（rad）'],['腿板角速度','左侧保持原方向；右侧取反，不减零点'],['电机角度、角速度','左电机保持原方向；右电机取反，不应用腿板的π/2零点'],['机身IMU','安装校正后的机身坐标：gyro X/Y（rad/s）、accel Z（m/s²，含重力）'],['左右压力','采集板原始接触状态码0、1、2、3；不是牛顿、千克或原始电压']],[145,350])
p('示例：左腿板原始Roll为+90°、右腿板为-90°时，对应上传角度均为0。角度转度可乘180/π，角速度同理；本文件夹的串口1数据使用rad，与串口2反馈的角度单位“度”不同。')
p('压力码只可按类别使用。当前模式20代码直接转发status，未在本协议中定义0..3各自的受力位置含义；不能由单个码推导力值或重量。')
p('5. 时间戳与数据可信度','Heading1')
p('timestamp_us是主板读取快照的uint32微秒计数，不是Unix时间。约每4294.967296秒（71.58分钟）自然回绕一次。连续两帧的差值按 (当前值 - 上一值) &amp; 0xFFFFFFFF 计算，再除以1e6得到秒。')
p('离线time_s首帧为0，正常累加上述差值。差值达到或超过0x80000000（约35.79分钟）时，上位机视为时间倒退、重启或歧义长间隔，segment_id加1，并让time_s增加0.01秒；该增量是软件衔接值，不是实际间隔。')
p('协议没有CRC、设备帧序号或逐通道有效位，因此不能精确统计丢帧、去重，也不能区分有效零值与无效占位。固件对无效字段保留为0；有限数值只说明可绘图，不代表传感器健康。各传感器没有全身同步采样保证，帧时间戳是快照时间。')
p('解析建议：寻找4字节帧头，收齐80字节后检查帧尾及两路压力范围；失败则向后移动1字节继续搜索。文件可能包含噪声或末尾残帧，不要简单按文件长度除80后直接切帧。')
story.append(PageBreak())
p('6. CSV与NPZ导出格式','Heading1')
p('本目录LOG00082.csv为离线导出，共20列：time_s、timestamp_us、segment_id，随后按第3节顺序排列17个通道（从left_motor_angle到right_pressure）。不保存帧头与帧尾；压力写整数，其余测量值保留浮点数。')
table([['字段','类型 / 含义'],['time_s','float64，相对时间秒；跨异常段的连接规则见第5节'],['timestamp_us','uint32，原始设备微秒计数'],['segment_id','uint32，时间分段编号，从0开始'],['values','NPZ专用：float32矩阵，形状(N,17)，列顺序由channel_names给出'],['17个独立通道数组','NPZ：压力uint32，其他通道float32；各数组长度N'],['channel_names / units','17项字符串数组，与values各列对应；单位rad、rad/s、m/s2、code'],['source_file / frame_size','源日志路径字符串；帧长度标量uint32=80']],[145,350])
with np.load(root/'LOG00082.npz',allow_pickle=False) as z:
 p(f'现有LOG00082.npz核对：values形状为{z["values"].shape}，timestamp_us类型为{z["timestamp_us"].dtype}，time_s类型为{z["time_s"].dtype}，frame_size={int(z["frame_size"])}。')
p('注意：实时采集CSV是另一种表头，以host_unix_s、host_monotonic_s、host_receive_index、device_timestamp_us开头，随后17通道，共21列。host_receive_index只是电脑本地接收编号；不要与本目录的离线CSV混用列索引。')
p('7. Python读取示例','Heading1')
code='''import csv
import numpy as np

# Offline CSV: read by column name
with open("LOG00082.csv", encoding="utf-8-sig", newline="") as f:
    first = next(csv.DictReader(f))
    t = float(first["time_s"])
    angle = float(first["left_thigh_angle"])  # rad

# NPZ: no pickle required
with np.load("LOG00082.npz", allow_pickle=False) as z:
    t = z["time_s"]
    x = z["values"]  # N x 17
    names = z["channel_names"].tolist()
    left_thigh = x[:, names.index("left_thigh_angle")]
'''
story.append(Preformatted(code,ParagraphStyle(name='Code',fontName='Courier',fontSize=8,leading=12)))
p('依据：firmware/App/acquisition_mode.h、acquisition_mode.c；host/acquisition_monitor/protocol.py、log_data.py；firmware/RESERVED_UART2_PROTOCOL.md。文件统计为2026-11-17对当前目录的只读核对。','SmallCN')
def footer(c,d):
 c.setStrokeColor(colors.HexColor('#c7d2dc')); c.line(42,39,553,39)
 c.setFont('CN',8); c.setFillColor(colors.HexColor('#52677a')); c.drawString(42,26,'模式20数据格式说明 | 80-byte UART1 | 2026-11-17'); c.drawRightString(553,26,str(d.page))
out=root/'模式20_数据格式说明.pdf'
SimpleDocTemplate(str(out),pagesize=(595.28,841.89),rightMargin=50,leftMargin=50,topMargin=40,bottomMargin=53,title='模式20数据格式说明',author='项目数据采集说明').build(story,onFirstPage=footer,onLaterPages=footer)
print(out.resolve())

