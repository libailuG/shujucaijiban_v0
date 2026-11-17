from pathlib import Path
import json, hashlib
from xml.sax.saxutils import escape
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.pagesizes import A4
from pypdf import PdfReader

root=Path(__file__).resolve().parents[1]
folder=root/'caiji_kongzhi1_data'
source=folder/'LOG00128.TXT'
target=folder/'控制模式1_文件数据说明.pdf'
stats=json.loads((root/'work/control1_log_analysis_20261116/analysis.json').read_text(encoding='utf-8'))
replay=json.loads((root/'work/control1_log_analysis_20261116/replay.json').read_text(encoding='utf-8'))
pdfmetrics.registerFont(TTFont('CN','C:/Windows/Fonts/simsun.ttc',subfontIndex=0))
pdfmetrics.registerFont(TTFont('CNBold','C:/Windows/Fonts/simhei.ttf'))
styles={
 'title':ParagraphStyle('title',fontName='CNBold',fontSize=22,leading=29,textColor=colors.HexColor('#183650'),spaceAfter=10),
 'sub':ParagraphStyle('sub',fontName='CN',fontSize=10,leading=16,textColor=colors.HexColor('#617182'),spaceAfter=14),
 'h':ParagraphStyle('h',fontName='CNBold',fontSize=13,leading=19,textColor=colors.HexColor('#183650'),spaceBefore=13,spaceAfter=7),
 'body':ParagraphStyle('body',fontName='CN',fontSize=10,leading=17,spaceAfter=7),
 'small':ParagraphStyle('small',fontName='CN',fontSize=8.2,leading=12,spaceAfter=5),
 'cell':ParagraphStyle('cell',fontName='CN',fontSize=9,leading=13),
 'head':ParagraphStyle('head',fontName='CNBold',fontSize=9,leading=13,textColor=colors.white),
}
story=[]
def p(text,style='body'):story.append(Paragraph(text,styles[style]))
def table(rows,widths):
    data=[[Paragraph(str(v),styles['head' if i==0 else 'cell']) for v in row] for i,row in enumerate(rows)]
    t=Table(data,colWidths=widths,repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#244962')),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.HexColor('#f0f5f8'),colors.white]),
        ('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),8),
        ('RIGHTPADDING',(0,0),(-1,-1),8),('TOPPADDING',(0,0),(-1,-1),6),
        ('BOTTOMPADDING',(0,0),(-1,-1),6),('LINEBELOW',(0,0),(-1,0),.5,colors.HexColor('#244962'))]))
    story.append(t)

p('控制模式1 · 文件数据说明','title')
p('适用协议：52字节网络观测帧  |  编制日期：2026-11-16  |  单位：国际单位制','sub')
p('1. 文件与采集来源','h')
p('本目录的 LOG00128.TXT 是二进制日志，虽然扩展名为 TXT，内容并非文本，不能按行或以文本编码读取。日志记录主板预留串口1上传的控制模式1网络观测。')
p('同目录 LOG00128.csv 是对应的文本导出，含1行表头和4,026行数据，共14列：时间、时间依据、主机时间及11项观测。离线主机时间为空；数据可用Excel、Python或其他分析软件读取。NPZ是上位机支持的另一个导出格式，编制时目录内尚无NPZ文件。')
p('接口：JP4 / UART4，921600波特、8N1；第3脚PA0为TX，第1脚GND。通过预留串口2下发 data2=1，data1.bit1=1 开启观测上传。bit0独立控制策略扭矩许可：data1=2只上传；data1=3上传并允许扭矩，实际输出仍受暖机与保护条件门控。')
p('2. 帧结构与字段顺序','h')
p('每帧固定52字节：uint32帧头 + 11个float32 + uint32帧尾。小端字节序，浮点为IEEE 754单精度；Python struct格式为 &lt;I11fI。无校验和、序号、设备时间戳或有效位。')
rows=[['偏移/字节','类型','字段','单位/说明'],
 ['0','uint32','start_flag','0xAAAAAAAA'],
 ['4','float32','左关节角度','rad'],['8','float32','右关节角度','rad'],
 ['12','float32','左关节角速度','rad/s'],['16','float32','右关节角速度','rad/s'],
 ['20','float32','上一周期左主动扭矩命令','Nm，模型坐标'],['24','float32','上一周期右主动扭矩命令','Nm，模型坐标'],
 ['28','float32','前后倾斜角','rad，前倾为负'],['32','float32','左右倾斜角','rad，右倾为正'],
 ['36','float32','腰部X轴角速度','rad/s，模型坐标'],['40','float32','腰部Y轴角速度','rad/s，模型坐标'],
 ['44','float32','腰部Z轴角速度','rad/s，模型坐标'],['48','uint32','end_flag','0xEEEEEEEE']]
table(rows,[62,64,196,177])
p('帧头在线字节为 AA AA AA AA，帧尾为 EE EE EE EE。字段不含网络原始动作、实测扭矩、电流、扭矩许可位或故障原因。','small')
story.append(PageBreak())
p('字段语义与读取方法','title')
p('3. 坐标、归一化与扭矩历史','h')
p('左右均为穿戴者自身视角。上传数据采用网络模型坐标：腰部X向右、Y向前、Z向上。机身坐标X向前、Y向左、Z向上；模型角速度为 [-机身Y, 机身X, 机身Z]。左右髋关节以向后伸展为正，向前屈髋为负；当前右电机的角度与角速度已取反后上传。')
p('这11项是本次网络输入65维观测中的最新切片。完整输入包含10帧关节及扭矩历史（10×6维）和当前5维机身观测。串口只上传最新11项，不上传其余历史帧。')
p('上传值已去掉π、4π和40的尺度系数，且未做网络mean/std标准化：角度直接为rad，角速度直接为rad/s，扭矩直接为Nm。前后倾、左右倾由姿态四元数对应的重力投影计算，不应直接当作驱动层同名欧拉角。')
p('上一周期主动扭矩是策略计算后、经过主动限幅的模型坐标命令，再作为本周期观测输入。当前默认每侧限幅±20Nm；它不是网络原始动作、不是电机实测扭矩，也不是右电机方向转换后的驱动命令。即使bit0禁止扭矩下发，策略仍计算并更新非零扭矩历史。')
p('4. 时间轴与异常处理','h')
p('名义上传频率为50Hz。离线第i帧的时间定义为 t=i/50秒（i从0开始）；N帧的名义采样覆盖时长按N/50秒表述，首末样本的时间差为(N-1)/50秒。它们都不是设备记录的真实时间。')
p('只有有效策略更新（含暖机）才上传。IMU、电机反馈、置零、调度周期等条件异常时可能暂停，恢复后重新暖机。串口忙时丢弃当帧，不积压。因此，文件中相邻两帧并不保证真实间隔为20ms；没有时间戳和序号，不能确认漏帧数量或停发时长。')
p('解析时以帧头查找候选，检查52字节长度、固定位置帧尾及11项有限浮点值；遇到错帧逐字节重新同步。忽略并统计杂散或残留尾字节。帧头帧尾检查不能替代CRC，也不能保证检测出所有数据位错误。')
p('5. 上位机读取与导出','h')
p('运行 host/control_mode1_monitor/run.bat，断开串口，在“网络观测 · 国际单位”标签页点击“读取日志文件”，选择本目录的二进制TXT文件。加载后显示完整曲线，可缩放、拖动和勾选通道；离线数据保留全部帧。')
table([['导出格式','内容与解释'],
 ['CSV','time_s、time_basis、host_time及11项观测。离线time_basis=nominal_50hz，host_time留空；实时数据按电脑接收时间计时。'],
 ['NPZ','values：N×11 float32；keys：11项字段名；time_s：相对秒数；host_time：电脑接收时间，离线为NaN；time_basis、source与nominal_sample_rate_hz记录时间依据、来源和名义频率。']],[70,429])
p('NPZ可用 numpy.load(path, allow_pickle=False) 读取；values各列顺序与第1页11项浮点字段一致，单位由keys字段名的_rad、_rad_s、_nm后缀标明。','small')
story.append(PageBreak())
p('LOG00128.TXT · 文件核对','title')
p('以下数值针对当前文件，不代表所有控制模式1日志。时间均为按50Hz换算的名义时间。','sub')
p('6. 完整性与基本统计','h')
table([['项目','核对结果'],['文件大小','209,356字节'],['有效帧数','4,026帧，共209,352字节'],
 ['帧间情况','有效帧在文件中连续，未发现错位或非有限数值'],['尾部残留','4字节：FF FF FF FF；不属于观测帧'],
 ['CSV对应关系','4,026行数据，与TXT解码值一致（浮点文本精度内）'],
 ['时间换算','名义覆盖80.52秒；首末样本时间差80.50秒']],[120,379])
story.append(Spacer(1,10))
table([['项目','左侧','右侧'],['关节角度范围','-56.08至21.63°','-39.97至24.49°'],
 ['关节角速度范围','-6.83至4.33 rad/s','-5.19至3.48 rad/s'],
 ['主动扭矩命令范围','-6.00至19.01 Nm','-9.93至10.72 Nm'],
 ['主动扭矩命令均值','1.76 Nm','0.42 Nm'],['主动扭矩命令RMS','4.67 Nm','4.06 Nm']],[185,157,157])
p('表中角度为便于阅读换算成度；日志原始角度单位仍是rad。扭矩为策略历史命令，不是实际执行结果。','small')
p('7. 已发现现象及解释边界','h')
p('网络回放：以日志重建观测并使用当前固件FP32权重回放，共比较4,016帧，99.73%的帧左右命令误差均小于0.01Nm，支持字段顺序、尺度还原与扭矩历史基本匹配。回放依赖相邻记录与当前权重，不能替代硬件执行验证。')
p('名义16.40至16.60秒：左右历史扭矩连续11帧为零，与策略复位后重新暖机的特征一致；属于推断，日志没有状态码，无法确认具体触发原因或停发时长。')
p('名义52.00秒：左侧主动扭矩命令峰值约19.01Nm，附近关节与腰部运动变化明显。全文件没有达到±20Nm饱和。末尾约0.52秒关节基本静止，但左右平均策略命令仍约1.26、1.23Nm；不能据此认定电机实际输出。')
p('8. 版本依据与文件标识','h')
p('依据：当前 firmware/App/control_mode1.c、control_mode1_observation.h、control1_policy/s2_deploy.c，以及上位机 observation.py、main.py。日志说明与当前代码一致；后续若改变字段、单位或协议长度，应同步更新本说明。','small')
p('原始文件SHA-256：<br/>'+hashlib.sha256(source.read_bytes()).hexdigest(),'small')

def footer(c,doc):
    c.setStrokeColor(colors.HexColor('#d4dfe8'));c.line(48,42,A4[0]-48,42)
    c.setFont('CN',8);c.setFillColor(colors.HexColor('#617182'))
    c.drawString(48,28,'控制模式1 | 52字节网络观测日志 | 2026-11-16')
    c.drawRightString(A4[0]-48,28,f'{doc.page}')
doc=SimpleDocTemplate(str(target),pagesize=A4,rightMargin=48,leftMargin=48,topMargin=42,bottomMargin=55,
    title='控制模式1 文件数据说明',author='数据采集工程',pageCompression=1)
doc.build(story,onFirstPage=footer,onLaterPages=footer)
r=PdfReader(target)
print(target)
print('pages:',len(r.pages))
for i,page in enumerate(r.pages):print('page',i+1,'text chars:',len(page.extract_text()))
assert len(r.pages)==3
