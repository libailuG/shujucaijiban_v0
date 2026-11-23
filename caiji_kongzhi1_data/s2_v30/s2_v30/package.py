"""Create a portable source/weights/validation ZIP without host binaries/caches."""
from pathlib import Path
import hashlib,json,zipfile
ROOT=Path(__file__).resolve().parent

def main():
 files=sorted(p for p in ROOT.rglob('*') if p.is_file() and not any(x in ('build','__pycache__') for x in p.relative_to(ROOT).parts) and p.name!='package_manifest.json')
 manifest={str(p.relative_to(ROOT)):{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in files}
 mp=ROOT/'package_manifest.json';mp.write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
 target=ROOT.with_suffix('.zip')
 with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as z:
  for p in files+[mp]:z.write(p,Path(ROOT.name)/p.relative_to(ROOT))
 print(target)
if __name__=='__main__':main()
