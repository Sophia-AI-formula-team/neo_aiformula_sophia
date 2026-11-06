"""Fixed local referee view. Inputs here never feed the recorded driving policy."""
import bisect
import math

from PIL import Image, ImageDraw


# Insets are deliberately outside the vehicle at the center of the chase frame.
# Tuple is relative to the 960 x 540 chase pane and uses exclusive right/bottom coordinates.
CHASE_INSET = (11, 11, 351, 226)
SIZE = (CHASE_INSET[2]-CHASE_INSET[0], CHASE_INSET[3]-CHASE_INSET[1])
PLOT = (SIZE[0]-8, 145)
SCALE = 20.0
TRAIL_SECONDS = 1.5
STATIC = "#ffbf61"
MOVING = "#ff76a0"
EGO = "#75d5ed"


def position(prop, t):
    home_x, home_z, right_x, right_z, amplitude, phase, omega, _ = prop
    lateral = amplitude * math.sin(phase+t*omega)
    return home_x+right_x*lateral, home_z+right_z*lateral


def pose_at(rows, times, t):
    i = max(0, bisect.bisect_right(times, t+2e-5)-1)
    a, b = rows[i], rows[min(i+1,len(rows)-1)]
    if abs(t-a[0])<2e-5 or b[0]<=a[0]:
        return a[1:4]
    u = max(0., min(1., (t-a[0])/(b[0]-a[0])))
    turn = math.remainder(b[3]-a[3], 2*math.pi)
    return a[1]+(b[1]-a[1])*u, a[2]+(b[2]-a[2])*u, a[3]+turn*u


class RefereeInset:
    def __init__(self, run, start, end):
        self.rows = run["rows"]
        self.times = [r[0] for r in self.rows]
        i = max(0, bisect.bisect_right(self.times,(start+end)*.5)-1)
        station = self.rows[i][7]
        props = run.get("props",[])
        self.ids = []
        self.props = []
        if not props:
            return
        anchor = min(range(len(props)),key=lambda index: abs(props[index][7]-station))
        if abs(props[anchor][7]-station)>16:
            return  # no local encounter in this excerpt
        self.ids = [i for i,p in enumerate(props) if abs(p[7]-props[anchor][7])<=8]
        self.props = [props[i] for i in self.ids]
        self.center = tuple(sum(p[k] for p in self.props)/len(self.props) for k in (0,1))
        rx, rz = props[anchor][2:4]
        norm = math.hypot(rx,rz)
        self.right = rx/norm, rz/norm
        self.forward = -self.right[1], self.right[0]
        self.anchor = anchor
        # Fix the crop once per encounter. Include the car's bypass side and all
        # cone extrema, otherwise an outside racing line can disappear at the top.
        def local(world_x, world_z, axis):
            return (world_x-self.center[0])*axis[0]+(world_z-self.center[1])*axis[1]
        sides = [local(p[0]+sign*p[2]*p[4],p[1]+sign*p[3]*p[4],self.right)
                 for p in self.props for sign in (-1,1)]
        sides.extend(local(r[1],r[2],self.right) for r in self.rows
                     if start<=r[0]<=end and abs(local(r[1],r[2],self.forward))<=8.5)
        shift = (min(sides)+max(sides))*.5
        self.center = self.center[0]+shift*self.right[0], self.center[1]+shift*self.right[1]
        self.scale = min(SCALE, (PLOT[1]-20)/(max(sides)-min(sides)+1.2))

    def project(self, world_x, world_z):
        dx, dz = world_x-self.center[0], world_z-self.center[1]
        # Road forward is right on screen. Road right is down on screen.
        return (PLOT[0]/2+self.scale*(dx*self.forward[0]+dz*self.forward[1]),
                PLOT[1]/2+self.scale*(dx*self.right[0]+dz*self.right[1]))

    def receipt(self):
        return {"prop_ids_zero_based":self.ids, "anchor_id_zero_based":self.anchor,
                "center_world_xz":self.center,"road_forward_xz":self.forward,"road_right_xz":self.right,
                "overlay_chase_pixel_box":CHASE_INSET,"meters_per_pixel":1/self.scale,
                "trail_s":TRAIL_SECONDS,"display_time":"Recorded chase-frame timestamp",
                "world_source":"UnityRace.PropPosition home + road_right * amplitude * sin(phase + t * omega)",
                "ego_source":"Recorded rows x,z,yaw; linear position and shortest-arc angle interpolation only between logged poses.",
                "selection":"Closest obstacle station to mid-clip ego progress, within 16 m; show only neighboring stations within 8 m of it."}

    def render(self, t, label):
        inset = Image.new("RGB",SIZE,"#183641")
        d = ImageDraw.Draw(inset)
        label(d,(10,17),"场地俯视 · 裁判",20,"#ffffff",True)
        label(d,(SIZE[0]-9,17),"行进 →",16,"#c6d9dc",anchor="rm")
        plot = Image.new("RGB",PLOT,"#22414b")
        p = ImageDraw.Draw(plot)
        step=round(2*self.scale)
        for cx in range(round(PLOT[0]/2), PLOT[0]+step+1,step):
            for x in (cx, PLOT[0]-cx):
                p.line((x,0,x,PLOT[1]),fill="#36535a",width=1)
        for cy in range(round(PLOT[1]/2), PLOT[1]+step+1,step):
            for y in (cy,PLOT[1]-cy):
                p.line((0,y,PLOT[0],y),fill="#36535a",width=1)
        for prop in self.props:
            x,y=self.project(*position(prop,t))
            if prop[4]:
                # The transverse end-stops mark the whole logged range; the bright
                # line marks only positions actually occupied over the last 1.5 s.
                low=self.project(prop[0]-prop[2]*prop[4],prop[1]-prop[3]*prop[4])
                high=self.project(prop[0]+prop[2]*prop[4],prop[1]+prop[3]*prop[4])
                p.line((low,high),fill="#b391a6",width=2)
                for a,b in (low,high):
                    p.line((a-4,b,a+4,b),fill="#b391a6",width=2)
                trail=[self.project(*position(prop,max(0,t-TRAIL_SECONDS+j*.1))) for j in range(16)]
                for j in range(1,len(trail)):
                    color=(128+j*7,79+j*3,110+j*3)
                    p.line((trail[j-1],trail[j]),fill=color,width=4)
                p.ellipse((x-6,y-6,x+6,y+6), fill=MOVING, outline="#ffffff",width=1)
            else:
                p.polygon([(x,y-7),(x+7,y+6),(x-7,y+6)], fill=STATIC,outline="#ffffff",width=1)
        ex, ez, yaw=pose_at(self.rows,self.times,t)
        x,y=self.project(ex,ez)
        fx,fz=math.sin(yaw),math.cos(yaw)
        rx,rz=math.cos(yaw),-math.sin(yaw)
        if -5<x<PLOT[0]+5 and -5<y<PLOT[1]+5:
            # The marker uses the dimensions of the actual clearance proxy and a pointed nose.
            corners=[self.project(ex+fx*a+rx*b,ez+fz*a+rz*b)
                     for a,b in ((.28,0),(.10,.38),(-.88,.38),(-.88,-.38),(.10,-.38))]
            p.polygon(corners, fill=EGO, outline="#122a34",width=2)
            nose=self.project(ex+fx*.18,ez+fz*.18)
            p.line(((x,y),nose),fill="#12343b",width=3)
        else:
            side="左" if x<0 else "右" if x>=PLOT[0] else "上" if y<0 else "下"
            label(p,(6,17),f"车在{side}侧画外",15,"#c0e1ea")
        p.line((PLOT[0]-8-step,PLOT[1]-9,PLOT[0]-8,PLOT[1]-9), fill="#abc4c8",width=2)
        label(p,(PLOT[0]-27,PLOT[1]-20),"2 m",13,"#d9e6e6",mono=True,anchor="mm")
        inset.paste(plot,(4,36))
        y=SIZE[1]-17
        d.polygon([(11,y-5),(16,y+4),(6,y+4)],fill=STATIC)
        label(d,(20,y),"静桶",16,"#f1f1e8")
        d.ellipse((75,y-4,83,y+4),fill=MOVING)
        label(d,(88,y),"动桶 / 1.5 s 尾迹",16,"#f1f1e8")
        d.polygon([(250,y),(239,y-5),(239,y+5)],fill=EGO)
        label(d,(256,y),"车",16,"#f1f1e8")
        d.rectangle((0,0,SIZE[0]-1,SIZE[1]-1),outline="#c1d3d7",width=1)
        return inset
