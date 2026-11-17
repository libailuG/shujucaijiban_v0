import sys
from pathlib import Path
sys.path.insert(0,str(Path('host/acquisition_monitor').resolve()))
from main import Window,create_application
from log_data import load_log
app=create_application();w=Window();w.show();w.tabs.setCurrentIndex(1)
w.offline.set_dataset(load_log('caiji_data/LOG00126.TXT'))
app.processEvents();w.grab().save('host/acquisition_monitor/offline_preview.png');w.close()
