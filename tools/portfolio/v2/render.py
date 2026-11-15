"""Cut both portfolios from the original paired screen recordings and frozen mission."""
import argparse
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from functools import lru_cache
from hashlib import sha256
import json
from pathlib import Path
import re
import subprocess

from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT = Path(__file__).resolve().parents[3]
MEDIA = ROOT / 'docs/media'
RUN = MEDIA / 'capture/20261115_140724_e358'
EVIDENCE = ROOT / 'docs/evidence'
DEST = MEDIA / 'v2'
FPS = 8
INK = '#111b1c'
PANEL = '#1a292b'
PAPER = '#f4f4e9'
MUTED = '#b4c8c5'
MINT = '#a7f1d2'
GOLD = '#ffc680'
RED = '#ffad94'
LINE = '#405454'
RESAMPLE = Image.Resampling.LANCZOS
FONT_DIR = Path('/usr/share/fonts/truetype/open-sans')
MONO = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'
CAPTURE_HASHES = {
    'gazebo': '303210a6ce4f8ed145963b5a857ffb3b94b7f5b205502a4a84b742c19ca9b56d',
    'rviz': '121f0a085ca929799d3e4ebae9fad05cf49cc70575c88529f9469e39c046108c',
}
REC = json.loads((RUN / 'mission_record.json').read_text())
GAUGES = {g['id']: g for g in REC['gauges']}
EVAL = json.loads((RUN / 'evaluation.json').read_text())


@dataclass(frozen=True)
class Shot:
    key: str
    seconds: float
    start: float | None = None


MAIN = [
    Shot('opening', 6, 32), Shot('nav', 6, 38), Shot('p11', 5),
    Shot('arrive', 6.5, 96.5), Shot('retry', 4), Shot('new', 4),
    Shot('move', 21.5, 113.875), Shot('far', 4.5), Shot('near', 4.75, 144.75),
    Shot('ok', 6), Shot('crates', 3.5, 277.5), Shot('unresolved', 3.75, 294.875),
    Shot('alarm', 4.75), Shot('test', 7.5), Shot('results', 5.75, 447),
    Shot('credit', 3)
]
SHORT = [
    Shot('retry', 3), Shot('new', 2.75), Shot('move', 15.5, 117.75),
    Shot('far', 3.5), Shot('near', 4, 145.125), Shot('ok', 5.5)
]

# The timestamps are UTC policy log entries, not camera or capture clock values.
OBS = {
    'p11': ('PG-11', 1, '14:07:58.068', 'OpenCV reads PG-11 at 4.19 bar.', 'Four ArUco corners rectify the panel. The needle angle gives the pressure.'),
    'retry': ('PG-12', 1, '14:08:58.192', 'The conduit hides the gauge tag.', 'The robot takes a second burst from the same frontal pose.'),
    'new': ('PG-12', 2, '14:09:05.306', 'The tag is still hidden. Change the view.', 'Two frontal bursts, ten frames, zero usable readings.'),
    'far': ('PG-12', 3, '14:09:36.541', 'The needle is visible, but the dial is too far away.', 'Four frames yield a value. The distance flag sends the base closer.'),
    'ok': ('PG-12', 4, '14:09:44.768', 'Five valid frames. Reading accepted.', 'Median 6.7833 bar   /   Offline simulation reference 6.8000 bar'),
    'unresolved': ('PG-13', 6, '14:12:07.835', 'PG-13 closes as unresolved.', 'A crate obscured the gauge across six observations at three viewpoints.'),
    'alarm': ('PG-14', 1, '14:12:56.981', 'The accepted reading triggers a HIGH alarm.', 'The measured pressure exceeds the configured 8.0 bar upper limit.')
}
MOTION = {
    'opening': ('AUTONOMOUS INSPECTION', 'When can a robot trust a gauge reading?', 'Four pressure gauges. A mobile robot must decide when to move and look again.'),
    'nav': ('AUTONOMOUS NAVIGATION', 'Nav2 drives to the first gauge.', 'AMCL localizes the base on the warehouse map. RViz shows the plan and live feedback.'),
    'arrive': ('PG-12   /   FRONTAL APPROACH', 'The base reaches PG-12 head-on.', 'Nav2 reports arrival at V0 at 14:08:52.586 UTC. The gauge sits behind a conduit.'),
    'move': ('PG-12   /   FIRST REPOSITION', 'Nav2 drives to the side of the conduit.', 'V0 -> V3    /    Target angle −45°    /    Nominal standoff 1.50 m'),
    'near': ('PG-12   /   SECOND REPOSITION', 'Nav2 closes the gap at the same angle.', 'V3 -> V8    /    Nominal standoff 1.50 -> 1.20 m    /    Goal reached 14:09:39.026 UTC'),
    'crates': ('PG-13   /   OCCLUSION', 'Nav2 tries the last available viewpoint.', 'The base approaches its third inspection pose at PG-13.'),
}


@lru_cache(None)
def font(size, style):
    path = MONO if style == 'mono' else FONT_DIR / f'OpenSans-{style}.ttf'
    return ImageFont.truetype(str(path), size)


def text(im, xy, s, size=32, fill=PAPER, style='Regular', maxw=None):
    """Fit by measured glyphs, reserving a fixed cap-height origin."""
    while maxw and font(size, style).getlength(s) > maxw:
        size -= 1
    ImageDraw.Draw(im).text(xy, s, font=font(size, style), fill=fill, anchor='lt')


def rect(im, xy, color):
    ImageDraw.Draw(im).rectangle(xy, fill=color)


def rule(im, x, y, x2):
    rect(im, (x, y, x2, y+1), LINE)


def fit(im, source, xywh):
    x, y, w, h = xywh
    tile = ImageOps.contain(source, (w, h), RESAMPLE)
    im.paste(tile, (x+(w-tile.width)//2, y+(h-tile.height)//2))


def fill_image(im, source, xywh):
    x, y, w, h = xywh
    im.paste(ImageOps.fit(source, (w,h), RESAMPLE), (x,y))


def label(im, x, y, s, color=MUTED, width=None):
    text(im, (x,y), s, 22, color, 'mono', width)


def base(topic, vertical=False):
    w,h = (1080,1920) if vertical else (1920,1080)
    im = Image.new('RGB', (w,h), INK)
    rect(im, (48,43,54,85), MINT)
    text(im,(74,43),'InspectLoop',43,PAPER,'Bold')
    if vertical:
        label(im,48,126,'PG-12   /   ACTIVE REINSPECTION', MINT)
        rule(im,48,187,1032)
    else:
        label(im,365,57,topic,MINT,862)
        text(im,(1350,57),'GAZEBO HARMONIC  /  ROS 2 JAZZY',22,MUTED,'mono',510)
        rule(im,48,107,1872)
    return im


def footer(im, title, sub):
    rule(im,48,855,1872)
    text(im,(48,888),title,60,PAPER,'Semibold',1824)
    text(im,(50,983),sub,32,MUTED,'Regular',1820)


def record(gauge, index):
    return next(o for v in GAUGES[gauge]['viewpoints'] for o in v['observations'] if o['index']==index)


@lru_cache(None)
def camera(gauge, index, kind):
    path = record(gauge,index)[kind+'_image_path']
    return Image.open(RUN/'images'/Path(path).name).convert('RGB')


def stamp(t):
    return f'{int(t)//60:02d}:{int(t)%60:02d}.{int(round((t%1)*1000)):03d}'


def fields(key):
    gauge,index,utc,_,_=OBS[key]
    o=record(gauge,index)
    return [
        ('POLICY EVENT  /  UTC',utc),
        ('CAMERA BURST  /  SIM TIME',f"{o['first_frame_stamp_s']:.1f}–{o['last_frame_stamp_s']:.1f} s"),
        ('VALID FRAMES',f"{o['frames_valid']} of {o['frames_requested']}"),
        ('QUALITY FLAG',', '.join(o['quality_flags']) or 'Clear'),
    ]


def color_for(key):
    return RED if key=='alarm' else MINT if key in ('ok','p11') else GOLD


def evidence(im, key):
    gauge,index,_,title,sub=OBS[key]
    o=record(gauge,index)
    label(im,48,139,'ROBOT CAMERA   /   OPENCV ANNOTATION')
    label(im,969,139,'RECTIFIED PANEL' if o['rectified_image_path'] else 'OCCLUSION DETAIL')
    label(im,1440,139,'OBSERVATION RECORD')
    fit(im,camera(gauge,index,'annotated'),(48,182,882,652))
    if o['rectified_image_path']:
        fit(im,camera(gauge,index,'rectified'),(969,194,430,600))
    else:
        raw=camera(gauge,index,'raw')
        fit(im,raw.crop((335,100,617,395)),(969,237,430,521))
        label(im,971,747,'FRONTAL CAMERA CROP')
    for j,(k,v) in enumerate(fields(key)):
        y=220+j*117
        rule(im,1440,y-19,1870)
        label(im,1440,y,k)
        text(im,(1440,y+38),v,37,PAPER,'Semibold',420)
    decision=o['decision']
    text(im,(1440,704),decision,46,color_for(key),'Bold',425)
    if o['has_value']:
        text(im,(1441,774),f"{o['value']:.2f} bar",33,MUTED,'Semibold')
    else:
        label(im,1442,778,'NO PRESSURE VALUE')
    footer(im,title,sub)


def map_crop(key):
    if key in ('opening','nav'):
        return (460,290,870,582)
    if key=='crates':
        return (820,249,1180,506)
    return (690,458,1022,695)


def pair(im, key, gz, rv, sec):
    label(im,48,134,'GAZEBO   /   FOLLOW CAMERA')
    label(im,1322,134,'RVIZ   /   LOCALIZATION & PLAN')
    fill_image(im,gz.crop((4,168,856,646)),(48,177,1228,656))
    # The complete motion comes from two simultaneous X11 recordings. No map graphics are synthesized.
    fill_image(im,rv.crop(map_crop(key)),(1322,177,550,393))
    label(im,1322,601,'NAV2   /   RECORDED FEEDBACK')
    fit(im,rv.crop((10,628,268,710)),(1322,638,550,171))
    tc='REC  '+stamp(sec)
    rect(im,(61,780,334,821),INK)
    text(im,(73,790),tc,23,PAPER,'mono')
    _,title,sub=MOTION[key]
    if key=='move' and sec>=126:
        title='The robot turns to face PG-12.'
        sub='Same continuous shot   /   Nav2 reaches V3 at 14:09:24.957 UTC.'
    footer(im,title,sub)


def unresolved(im, gz, sec):
    label(im,48,134,'GAZEBO   /   FINAL PG-13 VIEWPOINT')
    fill_image(im,gz.crop((4,168,856,646)),(48,177,1228,656))
    rect(im,(61,780,334,821),INK)
    text(im,(73,790),'REC  '+stamp(sec),23,PAPER,'mono')
    label(im,1322,134,'ROBOT CAMERA   /   OBSERVATION 6')
    fit(im,camera('PG-13',6,'annotated'),(1322,177,550,413))
    text(im,(1320,626),'UNRESOLVED',52,GOLD,'Bold')
    text(im,(1322,709),'0 of 5 valid frames',34,PAPER,'Semibold')
    label(im,1322,784,'FINAL EVENT   14:12:07.835 UTC')
    footer(im,OBS['unresolved'][3],OBS['unresolved'][4])


def plot(im):
    rows=re.findall(r't\+([\d.]+)-[\d.]+s\s+max\|v\|=([\d.]+)',(EVIDENCE/'astra_v3_1_live/f3_tiny_inc/odom_speed.txt').read_text())
    label(im,65,166,'SEPARATE FAULT RUN   /   MEASURED ODOMETRY',MINT)
    text(im,(60,227),'A malformed scan stops the base.',62,PAPER,'Semibold',1195)
    text(im,(65,321),'Peak linear speed in each 0.5 s window',33,MUTED)
    x0,y0,x1,y1=154,443,1160,726
    for val in (0,.2,.4):
        y=y1-round((y1-y0)*val/.4)
        rule(im,x0,y,x1)
        text(im,(62,y-10),f'{val:.1f}',24,MUTED,'mono')
    bw=(x1-x0)/len(rows)
    for i,(_,val) in enumerate(rows):
        height=round((y1-y0)*float(val)/.4)
        x=round(x0+i*bw)+6
        rect(im,(x,y1-max(3,height),round(x+bw-13),y1),GOLD if height else MINT)
    label(im,63,399,'m/s')
    label(im,154,758,'0 s')
    label(im,1087,758,'5.5 s')
    rect(im,(1248,175,1871,809),PANEL)
    label(im,1280,214,'ROS 2 MOTION PATH',MINT)
    for i,line in enumerate(['Nav2 command','C++17 LiDAR gate','Gated base command']):
        text(im,(1280,272+i*67),line,39,PAPER,'Semibold')
        if i<2:
            text(im,(1284,313+i*67),'↓',22,MINT,'mono')
    rule(im,1281,498,1838)
    label(im,1280,544,'ARCHIVED TEST REPORT',MINT)
    for i,line in enumerate(['489 test cases','11 CTest wrapper entries','0 reported failures']):
        text(im,(1280,601+i*65),line,37,PAPER,'Semibold')
    footer(im,'The velocity gate rejects invalid LiDAR input.','Recorded odometry drops from 0.40 m/s to zero in the separate simulation fault test.')


def results(im,gz):
    label(im,48,140,'GAZEBO   /   RETURN LEG')
    fill_image(im,gz.crop((4,168,856,646)),(48,185,1070,635))
    label(im,1170,140,'MISSION REPORT   /   PRESSURE')
    for j,g in enumerate(REC['gauges']):
        y=229+146*j
        r=g['result']; rule(im,1172,y-20,1872)
        text(im,(1172,y+6),g['id'],36,MUTED,'Semibold')
        value=f"{r['value']:.2f} bar" if 'value' in r else 'Unresolved'
        color=RED if r.get('alarm')=='HIGH' else GOLD if 'value' not in r else PAPER
        text(im,(1393,y),value,47,color,'Semibold')
        text(im,(1395,y+72),'HIGH alarm' if r.get('alarm')=='HIGH' else 'No value recorded' if 'value' not in r else 'Accepted',25,MUTED)
    footer(im,'Four gauges. Three accepted readings.',
           f"{EVAL['observations_total']} observations   /   8 inspection viewpoints   /   Maximum absolute error among accepted readings: {EVAL['max_abs_error']:.4f} bar")


def credit():
    im=base('MISSION COMPLETE')
    label(im,90,335,'SOURCE CODE   /   RECORDINGS   /   MISSION REPORT',MINT)
    text(im,(80,430),'github.com/Tamer1020/InspectLoop',91,PAPER,'Semibold',1750)
    rule(im,88,600,1830)
    text(im,(91,651),'Recorded mission 20261115_140724_e358',33,MUTED,'mono')
    return im


def short_view(key, gz=None, rv=None, sec=0):
    im=base('PG-12',True)
    c=color_for(key)
    if key in ('move','near'):
        label(im,48,229,'GAZEBO    /    REC '+stamp(sec),MINT)
        # Enlarge the relevant viewport with a fixed crop per shot.
        src=gz.crop((116,172,820,639)) if key=='move' else gz.crop((200,167,860,633))
        fill_image(im,src,(48,284,984,662))
        label(im,48,983,'RVIZ    /    NAV2')
        fill_image(im,rv.crop((715,500,978,684)),(48,1031,552,362))
        fit(im,rv.crop((10,628,267,710)),(631,1040,395,195))
        text(im,(632,1261),'V0 -> V3' if key=='move' else 'V3 -> V8',46,MINT,'Semibold')
        label(im,632,1332,'RECORDED MOTION',MUTED,391)
        head='Drive to the side.' if key=='move' else 'Move closer.'
        lines=['Nav2 turns the base toward PG-12.','Target: −45° at 1.50 m.'] if key=='move' else ['Same angle. New standoff: 1.20 m.','Nav2 reaches the closer pose.']
    else:
        gauge,n,utc,_,_=OBS[key]
        o=record(gauge,n)
        label(im,48,226,'OBSERVATION '+str(n)+'    /    ROBOT CAMERA')
        fit(im,camera(gauge,n,'annotated'),(48,275,984,738))
        rule(im,48,1048,1032)
        thumb= camera(gauge,n,'rectified') if o['rectified_image_path'] else camera(gauge,n,'raw').crop((335,100,617,395))
        fit(im,thumb,(52,1091,232,293))
        label(im,320,1107,'MISSION POLICY   /   '+utc+' UTC',MUTED,699)
        text(im,(316,1179),o['decision'],64,c,'Bold',670)
        label(im,321,1291,f"{o['frames_valid']} OF 5 VALID   /   "+(', '.join(o['quality_flags']) or 'CLEAR'),MUTED,697)
        heads={'retry':'The tag is hidden.','new':'Change viewpoint.','far':'Still too far.','ok':'6.7833 bar'}
        copy={'retry':['The front view is obstructed.','Try the same pose once more.'],
              'new':['A second burst misses the tag.','The robot requests a new Nav2 goal.'],
              'far':['The first side view yields 6.81 bar.','TOO_FAR blocks acceptance.'],
              'ok':['All five frames give a usable reading.','Offline reference: 6.8000 bar.']}
        head,lines=heads[key],copy[key]
    rule(im,48,1442,1032)
    text(im,(45,1481),head,82,PAPER,'Bold',985)
    for i,line in enumerate(lines):
        text(im,(49,1607+60*i),line,40,MUTED,'Regular',977)
    rule(im,48,1807,1032)
    text(im,(49,1843),'github.com/Tamer1020/InspectLoop',30,MINT,'mono')
    return im


@contextmanager
def decode(cap, name, start, count):
    command=['ffmpeg','-v','error','-nostdin','-threads','2','-ss',str(start),'-i',str(cap/(name+'.mkv')),
             '-frames:v',str(count),'-pix_fmt','rgb24','-f','rawvideo','-threads','1','pipe:1']
    with subprocess.Popen(command,stdout=subprocess.PIPE) as proc:
        def read_frame():
            data=proc.stdout.read(1280*720*3)
            if len(data)!=1280*720*3:
                raise RuntimeError('Recording ended before requested frame')
            return Image.frombytes('RGB',(1280,720),data)
        yield read_frame
        proc.stdout.close()
        if proc.wait():
            raise RuntimeError(f'Decode failed for {name}')


def frames(cap, shot, vertical=False, count=None):
    k=shot.key
    total=count or round(shot.seconds*FPS)
    if shot.start is None:
        if vertical:
            im=short_view(k)
        elif k=='credit':
            im=credit()
        elif k=='test':
            im=base('ENGINEERING VERIFICATION')
            plot(im)
        else:
            im=base(OBS[k][0]+'   /   VISUAL INSPECTION')
            evidence(im,k)
        for _ in range(total):
            yield im
        return
    with ExitStack() as stack:
        gz=stack.enter_context(decode(cap,'gazebo',shot.start,total))
        rv=None if k in ('results','unresolved') else stack.enter_context(decode(cap,'rviz',shot.start,total))
        for i in range(total):
            a=gz(); b=rv() if rv else None; t=shot.start+i/FPS
            if vertical:
                yield short_view(k,a,b,t)
            elif k=='unresolved':
                im=base('PG-13   /   FINAL OBSERVATION')
                unresolved(im,a,t)
                yield im
            elif k=='results':
                im=base('MISSION EVALUATION')
                results(im,a)
                yield im
            else:
                im=base(MOTION[k][0])
                pair(im,k,a,b,t)
                yield im


def cover(cap):
    im=Image.new('RGB',(1600,900),INK)
    with decode(cap,'gazebo',122,1) as grab:
        gz=grab()
    fill_image(im,gz.crop((160,169,820,618)),(0,0,1084,593))
    fill_image(im,camera('PG-12',4,'rectified').crop((10,235,489,678)),(1094,0,506,593))
    label(im,52,633,'INSPECTLOOP   /   AUTONOMOUS GAUGE INSPECTION',MINT)
    text(im,(45,691),'When the robot changes its view',76,PAPER,'Bold',1510)
    text(im,(52,815),'PG-12    /    Two blocked bursts. Two moves. One accepted reading.',31,MUTED,'Semibold')
    return im


def render(cap, sequence, dest, vertical):
    w,h=(1080,1920) if vertical else (1920,1080)
    args=['ffmpeg','-nostdin','-v','error','-y','-f','rawvideo','-pixel_format','rgb24',
          '-video_size',f'{w}x{h}','-framerate',str(FPS),'-i','pipe:0','-an','-map_metadata','-1',
          '-vf','scale=out_color_matrix=bt709:out_range=tv,format=yuv420p','-color_primaries','bt709','-color_trc','bt709','-colorspace','bt709',
          '-c:v','libx264','-profile:v','high','-preset','medium','-crf','6','-maxrate','8M','-bufsize','12M',
          '-threads','4','-g','3','-movflags','+faststart',str(dest)]
    with subprocess.Popen(args,stdin=subprocess.PIPE) as proc:
        for shot in sequence:
            print(dest.name,shot.key,flush=True)
            for im in frames(cap,shot,vertical):
                proc.stdin.write(im.tobytes())
        proc.stdin.close()
        if proc.wait():
            raise RuntimeError('H.264 encode failed')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path,help='Directory containing the two original MKV files')
    parser.add_argument('--output',type=Path,default=DEST)
    parser.add_argument('--only',choices=['main','short','cover','all'],default='all')
    args=parser.parse_args()
    for stem,digest in CAPTURE_HASHES.items():
        if sha256((args.capture/(stem+'.mkv')).read_bytes()).hexdigest()!=digest:
            raise ValueError(f'Unexpected source recording: {stem}')
    args.output.mkdir(exist_ok=True,parents=True)
    if args.only in ('all','cover'):
        cover(args.capture).save(args.output/'InspectLoop-V2-Cover.jpg',quality=94,subsampling=0)
    for name,sequence,filename,vertical in [('main',MAIN,'InspectLoop-Demo-V2.mp4',False),
                                             ('short',SHORT,'InspectLoop-PG12-V2.mp4',True)]:
        if args.only in ('all',name):
            render(args.capture,sequence,args.output/filename,vertical)


if __name__=='__main__':
    main()
