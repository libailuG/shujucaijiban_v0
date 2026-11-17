from pathlib import Path
src=Path('tmp/pdfs/create_spec.py').read_text(encoding='utf-8-sig')
src=src.replace('fontSize=8,leading=12', 'fontSize=8,leading=11').replace('),5)', '),3)')
addition='''breaks = [i for i, item in enumerate(story) if isinstance(item, PageBreak)]
story = story[breaks[0]+1:breaks[1]]
p('压力标志含义','Heading2')
table([['状态码','含义'],['0','前脚掌与后脚掌均未触地（全部离地）'],['1','前脚掌与后脚掌同时触地'],['2','仅前脚掌触地'],['3','仅后脚掌触地']],[65,430])
p('左右压力字段使用相同定义。按映射后的ADC通道判断：ADC1或ADC2 > 2.5 V为前脚掌触地，ADC3 > 2.5 V为后脚掌触地；等于阈值按未触地处理。压力标志是枚举状态码，不是位掩码，也不是力值或重量。','SmallCN')
p('注意：主板在压力数据无效或超时时也填0；本帧没有有效位，单凭0不能区分“全部离地”和“无效占位”。依据：脚底板固件App/main.c、App/upload.h。','SmallCN')
'''
src=src.replace('SimpleDocTemplate(str(out)',addition+'SimpleDocTemplate(str(out)')
exec(compile(src,'create_spec_pressure','exec'))

