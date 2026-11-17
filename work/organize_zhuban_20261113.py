from pathlib import Path
import zipfile, hashlib, json, shutil
from datetime import datetime

root=Path(r'C:\Users\libai\Desktop\X1').resolve()
target=root/'imu_caiji/zhuban_caiji'
stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
backup=root/'zhuban_caiji_before_merge_20261113_134506.zip'
items=[]
def add(src,dst):
    s=(root/src).resolve(); d=(target/dst).resolve()
    assert s.is_relative_to(root) and d.is_relative_to(target)
    assert s.exists() or d.exists(), s
    items.append((s,d))
for name in ['control_mode1_monitor','motor_simulator','work','电机通讯协议_提取说明.txt','电机接入准备_核对结果.txt']:
    add('dege_caiji/'+name,name)
for name in ['battery_adc','dap_motor_diagnosis','green_rx_status','imu_bdma_before','imu_tim7_async','monitor_uart1','motor_2m','motor_5ms','motor_dma_before','motor_rx_idle','rs485_link_test','uart_reserved2','uart_struct','uart1_flash','pdfs/zhuban']:
    add('dege_caiji/tmp/'+name,'tmp/'+name)
for s in (root/'dege_caiji/tmp').iterdir():
    if s.is_file(): add(str(s.relative_to(root)), 'tmp/'+s.name)
add('imu_caiji/zhuban_troubleshooting_archive','zhuban_troubleshooting_archive')
add('imu_caiji/data/sim2real_v0','data/sim2real_v0')
def files(p):
    return sorted(x for x in p.rglob('*') if x.is_file()) if p.is_dir() else [p]
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
with zipfile.ZipFile(backup) as z:
    manifest=json.loads(z.read('backup_manifest.json'))
    assert z.testzip() is None
    for x in manifest:
        assert hashlib.sha256(z.read(Path(x['path']).as_posix())).hexdigest()==x['sha256']
print('Backup verified:',backup,flush=True)
for s,d in items:
    if not d.exists():
        d.parent.mkdir(parents=True,exist_ok=True)
        shutil.move(str(s),str(d))
    for x in manifest:
        original=root/x['path']
        if original==s or original.is_relative_to(s):
            f=d if original==s else d/original.relative_to(s)
            assert f.is_file() and sha(f)==x['sha256'], f
    print('Moved:',s.relative_to(root),'->',d.relative_to(target),flush=True)
backups=target/'backups';backups.mkdir(exist_ok=True)
final_backup=backups/backup.name
shutil.move(str(backup),str(final_backup))
report={'backup':str(final_backup),'archive_sha256':sha(final_backup),'backed_up_files':len(manifest),'moves':[{'source':str(s),'destination':str(d)} for s,d in items],'verification':'ZIP CRC and SHA256 verified; every moved file SHA256 verified'}
(target/('整理记录_'+stamp+'.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False),flush=True)
