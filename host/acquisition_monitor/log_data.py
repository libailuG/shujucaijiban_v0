"""Offline mode20 logs: preserve values, export typed arrays without pickle."""
import csv
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from protocol import Parser, Timing, KEYS

UNITS = ['code' if k.endswith('pressure') else 'rad/s' if k.endswith('speed') or 'gyro' in k
         else 'm/s2' if 'accel' in k else 'rad' for k in KEYS]

@dataclass
class LogData:
    source: str
    timestamp_us: np.ndarray
    time_s: np.ndarray
    values: np.ndarray
    segment_id: np.ndarray
    bad: int
    discarded: int
    trailing: int

    def check_destination(self,path):
        if Path(path).resolve()==Path(self.source).resolve():
            raise ValueError('导出文件不能覆盖原始采集日志，请另选文件名。')

    def export_csv(self,path):
        self.check_destination(path)
        with open(path,'w',newline='',encoding='utf-8-sig') as out:
            writer=csv.writer(out)
            writer.writerow(['time_s','timestamp_us','segment_id']+KEYS)
            for i in range(len(self.time_s)):
                values=[int(v) if k.endswith('pressure') else float(v) for k,v in zip(KEYS,self.values[i])]
                writer.writerow([float(self.time_s[i]),int(self.timestamp_us[i]),int(self.segment_id[i]),*values])

    def export_npz(self,path):
        self.check_destination(path)
        arrays={k:self.values[:,i].astype(np.uint32 if k.endswith('pressure') else np.float32)
                for i,k in enumerate(KEYS)}
        # Opening explicitly avoids numpy silently adding a second extension.
        with open(path,'wb') as out:
            np.savez_compressed(out,time_s=self.time_s,timestamp_us=self.timestamp_us,
                segment_id=self.segment_id,values=self.values,channel_names=np.asarray(KEYS),
                units=np.asarray(UNITS),source_file=np.asarray(self.source),
                frame_size=np.asarray(80,dtype=np.uint32),**arrays)


def load_log(path,progress=None,cancelled=None):
    path=Path(path)
    size=path.stat().st_size
    parser=Parser(); timing=Timing()
    timestamps=[]; times=[]; rows=[]; segments=[]
    consumed=0
    with path.open('rb') as source:
        while True:
            if cancelled and cancelled(): raise InterruptedError('已取消文件读取')
            chunk=source.read(256*1024)
            if not chunk: break
            consumed+=len(chunk)
            for frame in parser.feed(chunk):
                timestamps.append(frame.timestamp_us)
                times.append(timing.update(frame))
                segments.append(timing.resets)
                rows.append([frame.values[k] for k in KEYS])
            if progress: progress(min(100,int(consumed*100/max(1,size))))
    if not rows: raise ValueError('未找到80字节模式20采集帧，请确认日志格式与固件版本。')
    return LogData(str(path.resolve()),np.asarray(timestamps,dtype=np.uint32),
        np.asarray(times,dtype=np.float64),np.asarray(rows,dtype=np.float32),
        np.asarray(segments,dtype=np.uint32),parser.bad,parser.discarded,len(parser.buffer))

