from pathlib import Path
src=Path('tmp/pdfs/create_spec.py').read_text(encoding='utf-8-sig')
src=src.replace("SimpleDocTemplate(str(out)","breaks = [i for i, item in enumerate(story) if isinstance(item, PageBreak)]\nstory = story[breaks[0]+1:breaks[1]]\nSimpleDocTemplate(str(out)")
exec(compile(src,'create_spec_section3','exec'))
