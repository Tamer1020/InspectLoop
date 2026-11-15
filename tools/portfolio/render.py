"""Build InspectLoop's evidence film, vertical cut and publication stills from the repository."""
import argparse
import json
import math
import os
import re
import subprocess
import tempfile
from functools import lru_cache
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT = Path(__file__).resolve().parents[2]
MEDIA = ROOT / "docs/media"
RUN = ROOT / "docs/media/capture/20261115_140724_e358"
REC = json.loads((RUN / "mission_record.json").read_text())
GAUGES = {g["id"]: g for g in REC["gauges"]}
EVAL = json.loads((RUN / "evaluation.json").read_text())
W, H = 1920, 1080
BG, PANEL, LINE = "#101d29", "#1b3040", "#365061"
WHITE, MUTED, TEAL, GOLD = "#f5f5ed", "#b9cdd5", "#75e4cd", "#ffca75"
FONT = Path(os.environ.get("PORTFOLIO_FONTS", "/usr/share/fonts/truetype/open-sans"))
MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
ARCHIVE = "Recorded mission  /  20261115_140724_e358"
LIVE = MEDIA / "capture"
FPS = 24


@lru_cache(None)
def face(size, weight="Regular"):
    return ImageFont.truetype(MONO if weight == "Mono" else str(FONT / f"OpenSans-{weight}.ttf"), size)


def label(im, x, y, content, size=29, color=WHITE, weight="Regular"):
    ImageDraw.Draw(im).text((x, y), str(content), font=face(size, weight), fill=color, anchor="lt")


def block(im, x, y, content, width, size=35, color=MUTED, gap=14, weight="Regular"):
    """Wrap on measured glyph widths, preserving explicit paragraph breaks."""
    for paragraph in content.split("\n"):
        line = ""
        for word in paragraph.split():
            candidate = (line + " " + word).strip()
            if line and face(size, weight).getlength(candidate) > width:
                label(im, x, y, line, size, color, weight)
                y += size + gap
                line = word
            else:
                line = candidate
        label(im, x, y, line, size, color, weight)
        y += size + gap
    return y


def canvas(size=(W, H)):
    return Image.new("RGB", size, BG)


def rule(im, xy, color=LINE, width=2):
    ImageDraw.Draw(im).line(xy, fill=color, width=width)


def paste(im, source, box, fill=False, center=(0.5, 0.5)):
    if not isinstance(source, Image.Image):
        source = Image.open(source).convert("RGB")
    x, y, w, h = box
    if fill:
        tile = ImageOps.fit(source, (w, h), Image.Resampling.LANCZOS, centering=center)
    else:
        tile = ImageOps.contain(source, (w, h), Image.Resampling.LANCZOS)
    im.paste(tile, (x + (w - tile.width) // 2, y + (h - tile.height) // 2))


def obs(gauge):
    return [o for v in GAUGES[gauge]["viewpoints"] for o in v["observations"]]


def pic(gauge, index, kind):
    return RUN / "images" / Path(obs(gauge)[index][kind + "_image_path"]).name


def furniture(im, topic, source):
    label(im, 80, 50, "InspectLoop", 32, WHITE, "Bold")
    rule(im, (303, 48, 303, 82))
    label(im, 332, 55, topic.upper(), 23, TEAL, "Mono")
    rule(im, (80, 109, 1840, 109))
    rule(im, (80, 990, 1840, 990))
    label(im, 80, 1011, source, 23, MUTED, "Mono")
    label(im, 1595, 1011, "ROS 2  /  JAZZY", 23, TEAL, "Mono")


def specimen(topic, title, body, metric, detail, source, picture, pic_label, color=TEAL):
    im = canvas()
    furniture(im, topic, source)
    block(im, 80, 183, title, 713, 74, WHITE, 13, "Bold")
    block(im, 84, 474, body, 665, 39, MUTED, 19)
    rule(im, (84, 763, 723, 763))
    label(im, 83, 800, metric, 51, color, "Bold")
    label(im, 85, 886, detail, 23, MUTED, "Mono")
    paste(im, picture, (785, 153, 1055, 735))
    label(im, 801, 918, pic_label, 25, MUTED, "Mono")
    return im


def title_frame(closing=False):
    im = canvas()
    hero = LIVE / "gazebo-hero.png"
    photo = hero if hero.exists() else ROOT / "docs/images/gazebo_warehouse.png"
    paste(im, photo, (760, 0, 1160, H), True, (.63, .55))
    grad = Image.new("RGBA", (510, H))
    d = ImageDraw.Draw(grad)
    for x in range(510):
        d.line((x, 0, x, H), fill=(16, 29, 41, int(255 * (1-x/509)**1.15)))
    im.paste(grad, (760, 0), grad)
    label(im, 85, 106, "MOBILE ROBOTICS   /   ACTIVE PERCEPTION", 27, TEAL, "Mono")
    label(im, 78, 255, "InspectLoop", 114, WHITE, "Bold")
    title = "Autonomous robotics\nand active perception." if closing else "When the dial\nis hidden."
    block(im, 86, 463, title, 890, 66, WHITE, 12, "Semibold")
    body = "github.com/Tamer1020/InspectLoop" if closing else "A mobile robot changes its view when a pressure reading cannot be trusted."
    block(im, 90, 728, body, 765, 33, TEAL if closing else MUTED, 13)
    label(im, 87, 978, "GAZEBO HARMONIC   /   SIMULATION", 23, MUTED, "Mono")
    return im


def map_plate():
    base = Image.open(ROOT/"src/inspectloop_navigation/maps/warehouse.pgm").convert("L")
    tile = ImageOps.colorize(base, black="#537383", white="#152938").resize((1000,680), Image.Resampling.NEAREST)
    d = ImageDraw.Draw(tile)
    for gid,g in GAUGES.items():
        for view in g["viewpoints"]:
            p = view["navigation"]["final_pose"]
            x = round((p["x"] + 12.5)*40)
            y = 679 - round((p["y"] + 8.5)*40)
            color=GOLD if gid=="PG-12" else TEAL
            d.ellipse((x-9,y-9,x+9,y+9),fill=color,outline=BG,width=2)
            d.line((x,y,x+20*math.cos(p["yaw"]),y-20*math.sin(p["yaw"])),fill=color,width=4)
        pos=g["mount"]
        x=round((pos["x"]+12.5)*40);y=679-round((pos["y"]+8.5)*40)
        tx=max(10,min(x-37,890));ty=y-51 if pos["y"]>0 else y+23
        label(tile,tx,ty,gid,22,WHITE,"Bold")
    return tile


def pair_plate():
    im=canvas()
    furniture(im,"Active reinspection",ARCHIVE)
    label(im,80,154,"PG-12  /  Before and after reinspection",69,WHITE,"Bold")
    for x,idx,col,t,sub in [
        (80,1,GOLD,"01   Frontal view","Tag missing twice   /   MOVE_NEW"),
        (992,3,TEAL,"02   Final viewpoint","5 of 5 frames valid   /   ACCEPT")]:
        paste(im,pic("PG-12",idx,"raw"),(x,270,847,555),True)
        label(im,x,852,t,40,col,"Bold")
        label(im,x,924,sub,26,MUTED,"Mono")
    return im


def results():
    im=canvas();furniture(im,"Mission evaluation",ARCHIVE)
    label(im,80,161,"Four gauges. Three readings.",70,WHITE,"Bold")
    label(im,83,276,"One autonomous round in the simulated warehouse",36,MUTED)
    cols=[82,450,1130,1498]
    for x,head in zip(cols,["GAUGE","OBSERVATION","READING","ALARM"]):
        label(im,x,392,head,26,TEAL,"Mono")
    notes=["Clear frontal view","Recovered at −45°","6 attempts, 3 viewpoints","Above 8.0 bar limit"]
    for j,(g,note) in enumerate(zip(REC["gauges"],notes)):
        y=469+j*111;r=g["result"];rule(im,(80,y-19,1838,y-19))
        for x,val in zip(cols,[g["id"],note,f'{r["value"]:.2f} bar' if "value" in r else "Unresolved",r.get("alarm","—")]):
            label(im,x,y,val,39,GOLD if val in ("HIGH","Unresolved") else WHITE,"Semibold")
    label(im,82,917,f'Maximum absolute error among 3 accepted readings: {EVAL["max_abs_error"]:.4f} bar.',31,MUTED)
    return im


def architecture():
    im=canvas();furniture(im,"ROS 2 architecture","Simulation  /  node and topic relationships")
    label(im,80,150,"How an image changes the next goal",67,WHITE,"Bold")
    d=ImageDraw.Draw(im)
    def node(x,y,w,head,sub):
        d.rounded_rectangle((x,y,x+w,y+132),radius=10,fill=PANEL,outline=LINE,width=2)
        label(im,x+24,y+21,head,36,WHITE,"Bold");label(im,x+24,y+83,sub,23,MUTED)
    def arrow(points):
        d.line(points,fill=TEAL,width=5,joint="curve")
        a,b=points[-2:];an=math.atan2(b[1]-a[1],b[0]-a[0])
        d.polygon([b,(b[0]-17*math.cos(an-.5),b[1]-17*math.sin(an-.5)),(b[0]-17*math.cos(an+.5),b[1]-17*math.sin(an+.5))],fill=TEAL)
    node(80,320,440,"Gazebo camera","RGB frames via ros_gz")
    node(735,320,445,"Gauge reader","ArUco • homography • needle")
    node(1395,320,443,"Mission manager","Quality flags → decision")
    node(1395,680,443,"Nav2","AMCL • planner • MPPI")
    node(735,680,445,"Velocity gate","C++17 • scan • watchdog")
    node(80,680,440,"Simulated base","Gazebo DiffDrive")
    for p in [[(522,386),(727,386)],[(1182,386),(1387,386)],[(1615,455),(1615,673)],[(1393,746),(1188,746)],[(733,746),(528,746)],[(302,679),(302,460)]]:
        arrow(p)
    for x,y,t in [(545,345,"RGB"),(1195,345,"reading"),(1631,546,"Nav2 goal"),(1225,706,"cmd_vel"),(527,706,"cmd_vel_safe"),(325,548,"new view")]:
        label(im,x,y,t,22,TEAL,"Mono")
    label(im,80,906,"The mission selects the viewpoint. LiDAR coverage independently gates velocity.",31,MUTED)
    return im


def fault_plot():
    im=Image.new("RGB",(1000,680),PANEL)
    label(im,55,48,"Measured odometry",42,WHITE,"Bold")
    label(im,57,120,"Peak linear speed by 0.5 s window",27,MUTED)
    data=(ROOT/"docs/evidence/astra_v3_1_live/f3_tiny_inc/odom_speed.txt").read_text()
    rows=re.findall(r't\+([\d.]+)-[\d.]+s\s+max\|v\|=([\d.]+)',data)
    for i,(_,v) in enumerate(rows):
        x=85+i*72;h=round(650*float(v))
        ImageDraw.Draw(im).rectangle((x,480-h,x+45,483),fill=GOLD if h else TEAL)
    rule(im,(58,485,949,485),MUTED)
    label(im,57,510,"0 s",25,MUTED);label(im,868,510,"5.5 s",25,MUTED)
    label(im,57,600,"Source: fault-run odometry log",25,TEAL,"Mono")
    return im


def build_stills():
    value=lambda g:f'{GAUGES[g]["result"]["value"]:.2f} bar'
    im={}
    im["open"]=title_frame()
    im["nav"]=specimen("Navigation","Eight inspection\nviewpoints","AMCL localizes the base on this map. Each marker is a logged arrival pose.", "8 of 8 reached","INSPECTION GOALS  /  ONE RUN",ARCHIVE,map_plate(),"Map + measured poses  /  no trajectory interpolation")
    im["vision"]=specimen("Computer vision","Reading the\nfirst gauge","Four ArUco corners rectify the panel. Dark needle evidence across 16 rings determines its angle.",value("PG-11"),"PG-11  /  ACCEPT",ARCHIVE,pic("PG-11",0,"rectified"),"OpenCV output  /  rectified panel")
    im["front"]=specimen("PG-12   /   observation 1","The front view\nhides the dial","The conduit box blocks part of the panel. The reader cannot detect the expected tag in any of the five frames.","RETRY_SAME","TAG_NOT_VISIBLE  /  0 OF 5",ARCHIVE,pic("PG-12",0,"raw"),"Camera capture  /  frontal viewpoint",GOLD)
    im["retry"]=specimen("PG-12   /   observation 2","Same pose.\nSame problem.","A second burst still contains no usable tag. The mission spends its same-view retry and requests another pose.","MOVE_NEW","TAG_NOT_VISIBLE  /  0 OF 5",ARCHIVE,pic("PG-12",1,"annotated"),"Reader output  /  second frontal burst",GOLD)
    im["move"]=specimen("PG-12   /   navigation","Nav2 reaches\nthe new pose","The mission selects a −45° approach at a nominal 1.50 m standoff. Nav2 reports that the action succeeded.","10.20 s","SIMULATED NAVIGATION TIME",ARCHIVE,map_plate(),"Gold markers  /  three measured PG-12 arrival poses")
    im["accept"]=specimen("PG-12   /   observation 3","Still too far\nfrom the dial","The reader sees the needle, but the dial is still too far away. The quality flag sends the robot to a closer pose.","TOO_FAR","MOVE_NEW  /  4 OF 5 VALID",ARCHIVE,pic("PG-12",2,"annotated"),"Reader overlay  /  rejected at the first alternate pose",GOLD)
    im["accept-detail"]=specimen("PG-12   /   observation 4","Closer pose.\nUsable reading.","Nav2 reaches the closer target. The rectified dial now yields five valid frames and a median of 6.7833 bar.",value("PG-12"),"ACCEPT  /  5 OF 5 VALID",ARCHIVE,pic("PG-12",3,"rectified"),"Rectified PG-12 panel  /  actual OpenCV output")
    im["unresolved"]=specimen("PG-13   /   persistent occlusion","When every view\nis blocked","The crate stack fills the camera view. After six observations across three viewpoints, the mission closes this gauge without a value.","UNRESOLVED","6 OBSERVATIONS  /  3 VIEWPOINTS",ARCHIVE,pic("PG-13",-1,"annotated"),"Reader output  /  last attempted view",GOLD)
    im["alarm"]=specimen("PG-14   /   alarm","A valid reading\ntriggers HIGH","The accepted pressure exceeds this gauge’s 8.0 bar upper alarm limit. The report retains the measurement and alarm.",value("PG-14"),"ACCEPT  /  HIGH ALARM",ARCHIVE,pic("PG-14",0,"rectified"),"OpenCV output  /  rectified high-pressure gauge",GOLD)
    im["fault"]=specimen("Motion safety   /   separate fault run","Fault injection\nwhile driving","A malformed LaserScan reached the C++ gate. It rejected the scan and odometry subsequently registered zero speed.","0.40 to 0 m/s","RECORDED SIMULATION FAULT TEST","Separate archived test  /  f3_tiny_inc",fault_plot(),"Each bar is the maximum observed in one time window")
    im["results"]=results()
    im["close"]=title_frame(True)
    return im


def vertical(key):
    im=canvas((1080,1920));d=ImageDraw.Draw(im)
    label(im,60,65,"InspectLoop",45,WHITE,"Bold")
    label(im,61,157,"PG-12  /  ACTIVE REINSPECTION",28,TEAL,"Mono")
    rule(im,(60,123,1020,123))
    mapping={
      "front":("The tag is hidden.","The robot takes five camera frames from the front. The reader cannot find the gauge tag.","RETRY_SAME","First burst  /  frontal view",pic("PG-12",0,"raw")),
      "retry":("A retry confirms it.","A second burst from the same pose also fails. The mission asks Nav2 for another viewpoint.","MOVE_NEW","Second burst  /  frontal view",pic("PG-12",1,"annotated")),
      "move":("Change the view.","Nav2 drives to a −45° approach. The dial is still flagged TOO_FAR, so the robot requests a closer pose.","GOAL SUCCEEDED","Map with recorded arrival poses",map_plate()),
      "accept":("Read it again.","At the closer pose, five of five frames are valid. The robot records their median and continues its round.","6.78 bar","ACCEPT  /  closer viewpoint",pic("PG-12",3,"annotated")),
    }
    title,desc,metric,cap,src=mapping[key]
    block(im,55,244,title,976,84,WHITE,14,"Bold")
    paste(im,src,(54,455,972,729),key!="move")
    label(im,58,1233,cap,29,MUTED,"Mono")
    label(im,55,1353,metric,70,TEAL if key=="accept" else GOLD,"Bold")
    block(im,60,1483,desc,938,44,WHITE,16)
    if key == "move" and (LIVE / "move.mp4").exists():
        d.rectangle((53,452,1028,1190),fill=BG)
        d.rectangle((53,1212,1028,1310),fill=BG)
        label(im,58,1233,"Gazebo capture  /  PG-12 reinspection",29,MUTED,"Mono")
    rule(im,(60,1741,1020,1741))
    label(im,60,1774,"Recorded Gazebo mission  /  v3.2.0",30,MUTED)
    label(im,60,1836,"github.com/Tamer1020/InspectLoop",32,TEAL,"Semibold")
    return im


def thumbnail():
    im=canvas()
    p=Image.open(pic("PG-12",3,"rectified")).convert("RGB")
    paste(im,p,(1010,0,910,1080),True,(.5,.78))
    label(im,85,113,"InspectLoop",62,TEAL,"Bold")
    block(im,80,297,"A better\nviewpoint",900,131,WHITE,16,"Bold")
    block(im,86,723,"How a mobile robot recovers a hidden gauge reading",840,48,MUTED,16)
    label(im,86,968,"WATCH THE DEMO",31,GOLD,"Bold")
    ImageDraw.Draw(im).polygon([(453,968),(453,994),(474,981)], fill=GOLD)
    return im


MAIN=[("open",6),("nav",8),("vision",7),("front",7),("retry",6),("move",9),("accept",6),("accept-detail",6),
      ("unresolved",8),("alarm",6),("fault",7),("report",6),("results",6),("close",5)]
SHORT=[("front",7),("retry",6),("move",8),("accept",9)]


def report_capture():
    from weasyprint import HTML, CSS
    # Render the original self-contained HTML at a fixed page width for a legible film insert.
    with tempfile.TemporaryDirectory() as t:
        pdf = Path(t) / "report.pdf"
        HTML(filename=str(RUN / "mission_report.html")).write_pdf(
            pdf, stylesheets=[CSS(string="@page {size: 1320px 800px; margin:0} table {break-inside:avoid}")])
        subprocess.run(["pdftoppm", "-f", "1", "-l", "1", "-r", "96", "-singlefile",
                        "-png", str(pdf), str(MEDIA / "mission-report")], check=True)



def encode(stills, sequence, destination, temp, vertical_cut=False):
    clips=[]
    for index,(name,seconds) in enumerate(sequence):
        frame=stills[name]
        png=temp/f"{index}-{name}.png"; frame.save(png)
        path=temp/f"{index}-{name}.mp4";clips.append(path)
        cmd=["ffmpeg","-nostdin","-hide_banner","-loglevel","error","-y","-loop","1","-framerate",str(FPS),"-i",str(png)]
        insert=LIVE/(name+".mp4")
        # Supplemental GUI shots have their own attribution and are never synced to the archival clock.
        if insert.exists() and name in ("nav","move") and (not vertical_cut or name == "move"):
            if vertical_cut:
                w,h,x,y = 972,548,54,545
            elif name == "nav":
                w,h,x,y = 1760,496,80,354
            else:
                w,h,x,y = 1054,592,786,218
            cmd+=["-i",str(insert),"-filter_complex_threads","2","-filter_complex",
                  f"[1:v]fps=24,scale={w}:{h}:force_original_aspect_ratio=decrease,pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:color=0x101d29[panel];"
                  f"[0:v][panel]overlay=x={x}:y={y}:eof_action=repeat[out]","-map","[out]"]
        else:
            # A brief dip at each edit keeps frozen evidence and shot changes distinguishable.
            cmd+=["-vf",f"fade=t=in:st=0:d=0.125,fade=t=out:st={seconds-.125}:d=0.125"]
        cmd+=["-t",str(seconds),"-an","-c:v","libx264","-preset","veryfast","-threads","4",
              "-crf","19","-pix_fmt","yuv420p","-movflags","+faststart",str(path)]
        subprocess.run(cmd,check=True)
    playlist=temp/"sequence.txt"
    playlist.write_text("".join(f"file '{p.name}'\n" for p in clips))
    subprocess.run(["ffmpeg","-nostdin","-loglevel","error","-y","-f","concat","-safe","0","-i",str(playlist),
                    "-c","copy","-movflags","+faststart",str(destination)],check=True)


def main():
    args=argparse.ArgumentParser(description=__doc__)
    args.add_argument("--output",type=Path,default=ROOT/"dist/portfolio")
    args.add_argument("--stills-only",action="store_true")
    args.add_argument("--refresh-report",action="store_true")
    args.add_argument("--preview-dir",type=Path)
    args=args.parse_args()
    MEDIA.mkdir(parents=True,exist_ok=True);args.output.mkdir(parents=True,exist_ok=True)
    if args.refresh_report or not (MEDIA/"mission-report.png").exists():
        report_capture()
    s=build_stills()
    record=canvas();s["report"]=record
    furniture(record,"Reporting",ARCHIVE)
    label(record,80,165,"What the mission recorded",73,WHITE,"Bold")
    label(record,84,280,"The exported report preserves each decision, including the unresolved gauge.",35,MUTED)
    excerpt=Image.open(MEDIA/"mission-report.png").convert("RGB").crop((8,5,1312,357))
    paste(record,excerpt,(80,385,1760,495))
    label(record,84,919,"Actual HTML report  /  filmed mission",25,MUTED,"Mono")
    for name,caption in [("nav","RViz  /  approach to PG-11"),("move","Gazebo  /  PG-12 navigation to the first alternate pose")]:
        if (LIVE/(name+".mp4")).exists():
            im=s[name]
            if name == "nav":
                im=canvas();s[name]=im
                furniture(im,"Autonomous navigation","Synchronized display capture  /  Gazebo Harmonic + RViz2")
                label(im,80,151,"Driving to the first gauge",76,WHITE,"Bold")
                label(im,81,286,"GAZEBO   /   ROBOT IN THE WAREHOUSE",25,TEAL,"Mono")
                label(im,979,286,"RVIZ   /   ESTIMATED POSE AND PLAN",25,TEAL,"Mono")
                label(im,82,907,"Nav2 drives the base toward PG-11 as AMCL updates its pose on the known map.",31,MUTED)
            else:
                ImageDraw.Draw(im).rectangle((775,145,1850,910),fill=BG)
                ImageDraw.Draw(im).rectangle((775,910,1850,963),fill=BG)
                label(im,801,918,caption,25,MUTED,"Mono")
    pair_plate().save(MEDIA/"pg12-two-views.jpg",quality=93)
    results().save(MEDIA/"mission-results.png",optimize=True)
    architecture().save(MEDIA/"architecture.png",optimize=True)
    thumbnail().save(MEDIA/"inspectloop-cover.jpg",quality=92)
    if args.preview_dir:
        args.preview_dir.mkdir(parents=True,exist_ok=True)
        for name,frame in s.items():
            frame.resize((768,432),Image.Resampling.LANCZOS).save(args.preview_dir/(name+".jpg"))
    if not args.stills_only:
        with tempfile.TemporaryDirectory() as t:
            encode(s,MAIN,args.output/"InspectLoop-Demo.mp4",Path(t))
        with tempfile.TemporaryDirectory() as t:
            encode({k:vertical(k) for k,_ in SHORT},SHORT,args.output/"InspectLoop-PG12-Short.mp4",Path(t),True)


if __name__ == "__main__":
    main()
