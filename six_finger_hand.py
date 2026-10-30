#!/usr/bin/env python3
"""Six-finger experimental hand, generated from primitives with CadQuery 2.7.

Run: python six_finger_hand.py --out exports
Optional: --check-motion (additional geometric collision checks).
Dependencies: cadquery>=2.7,<2.8; Python 3.11/3.12 recommended.
Units: millimeters. No input CAD, FreeCAD, GUI or external assets required.

This is a bench mechanism, not a fitted/validated medical device. A tapered,
hollow wrist interface is provided, but no person's anatomy was measured.
The script generates 19 separately printable structural parts, six posed-digit
reference meshes, the assembled reference mesh, STEP, and a dimensions manifest.
Do NOT print posed-digit or assembled reference meshes as fused mechanisms.

FIT CONVENTION: nominal 3 mm pin + 0.3 mm RADIAL gap => 3.6 mm rotating bore.
A literal 3 mm bore cannot simultaneously give that clearance to a 3 mm pin.
Set PIN_RADIAL_GAP=0 for a literal 3 mm CAD bore, then finish/fit experimentally.
All parts print separately. STL does not retain assembly joints or units.
"""
from __future__ import annotations
import argparse, json, math, struct
from pathlib import Path
import cadquery as cq

# ---------------------------- Editable dimensions ---------------------------
PIN_D = 3.0
PIN_RADIAL_GAP = 0.30
PIN_HOLE_D = PIN_D + 2 * PIN_RADIAL_GAP
CLEVIS_SIDE_GAP = 0.30       # Each side of central lug, not a radial dimension.
LUG_W = 8.0
DIGIT_W = 21.0
EAR_W = (DIGIT_W - LUG_W - 2*CLEVIS_SIDE_GAP)/2
JOINT_R = 8.0
TENDON_D = 2.2
TENDON_Z = 5.0               # Upper side closes links toward +Z.
PALM_NOMINAL_WALL = 6.0      # Conservative stock before R3 external edge rounds.
PALM_MIN_WALL = 3.0
OUTER_FILLET = 3.0
DUCT_R = TENDON_D/2 + PALM_MIN_WALL
ROOT_Z = 40.0
# (x, y, yaw degrees, proximal pivot spacing, distal pivot-to-tip length)
LAYOUT = [(-44,88,0,40,35),(-22,94,0,44,38),(0,97,0,42,36),
          (22,94,0,36,32),(44,88,0,30,28),(-65,25,-45,30,30)]
PALM_OUTLINE = [(-28,-35),(28,-35),(48,-5),(64,30),(64,82),(58,92),
                (-58,92),(-68,64),(-72,40),(-86,15),(-80,-5),(-45,-15)]
# Tuned geometry constants below are expressed in the same mm coordinate system.
# Large outline edits require repeating wall, collision and physical fit checks.


def box(w,l,h,x=0,y=0,z=0):
    return cq.Workplane('XY').box(w,l,h).translate((x,y,z))

def cylinder(r,length,origin=(0,0,0),axis=(0,0,1)):
    return cq.Workplane(cq.Plane(origin=origin,normal=axis)).circle(r).extrude(length)

def combine(items):
    result=items[0]
    for item in items[1:]: result=result.union(item)
    return result.clean()

def rounded_box(w,l,h,x=0,y=0,z=0,r=OUTER_FILLET):
    # Fillets are applied before union with joint bosses to avoid fragile edge IDs.
    return cq.Workplane('XY').box(w,l,h).edges().fillet(r).translate((x,y,z))

def fork(y):
    """Two ears around an 8 mm lug, with explicit 0.3 mm side gaps."""
    ears=[]
    for x in [-DIGIT_W/2,LUG_W/2+CLEVIS_SIDE_GAP]:
        ears.append(cylinder(JOINT_R,EAR_W,(x,y,0),(1,0,0)).edges().fillet(3))
        ears.append(rounded_box(EAR_W,12,16,x+EAR_W/2,y-6,0,r=3))
    s=combine(ears)
    return s.cut(cylinder(PIN_HOLE_D/2,30,(-15,y,0),(1,0,0)))

def dorsal_slot(y):
    # Transverse enclosed slot for threading elastic; dorsal = -Z in this model.
    # Small tool-edge radius is intentional: R3 would erase the anchor bridge.
    return box(30,2.4,3.0,0,y,-4.5).edges('|X').fillet(.6)

def tendon_drill(length,y0=-12):
    return cylinder(TENDON_D/2,length,(0,y0,TENDON_Z),(0,1,0))

def make_base():
    # Removable mounting flange seats on a matching palm pedestal at ROOT_Z-12.
    s=combine([rounded_box(20,18,6.2,0,-19,-8.9,r=3),
               box(18,5,16,0,-11,0),fork(0)])
    s=s.cut(cylinder(2.2,25,(0,-19,-18))) # M4 mount, clearance bore.
    s=s.cut(tendon_drill(22,-15)).cut(dorsal_slot(-11))
    return s.clean()

def make_link(length,distal=False):
    """Narrow round root lug plus broad filleted body; proximal ends in a fork."""
    end=length if distal else length-9
    s=combine([cylinder(JOINT_R,LUG_W,(-LUG_W/2,0,0),(1,0,0)).edges().fillet(3),
               rounded_box(LUG_W,13,16,0,6.5,0,r=3),
               rounded_box(DIGIT_W,end-10,16,0,(end+10)/2,0,r=3)])
    if not distal:s=s.union(fork(length))
    s=s.cut(cylinder(PIN_HOLE_D/2,30,(-15,0,0),(1,0,0)))
    s=s.cut(tendon_drill(length+25,-12))
    # Elastic passes through these dorsal slots; knots/bands remain accessible.
    s=s.cut(dorsal_slot(14 if not distal else length-8))
    if distal:
        # Transverse tip tie hole intersects the flexor passage for a stopper knot.
        s=s.cut(cylinder(1.5,30,(-15,length-5,TENDON_Z),(1,0,0)))
    return s.clean()

def root_height(pose):
    # Elevated side thumb clears the raised front tendon conduits in extension.
    return ROOT_Z + (18.0 if pose[2] else 0.0)

def place(s,pose):
    x,y,yaw,_,_=pose
    return s.rotate((0,0,0),(0,0,1),yaw).translate((x,y,root_height(pose)))

def local_point(pose,local_y,z):
    x,y,a,_,_=pose;a=math.radians(a)
    return (x-math.sin(a)*local_y,y+math.cos(a)*local_y,z)

def swept_circle(points,radius,inner_radius=None):
    """A smooth 3D spline sweep, used for a duct and its coaxial cutting core."""
    edge=cq.Edge.makeSpline([cq.Vector(*p) for p in points])
    wire=cq.Wire.assembleEdges([edge])
    plane=cq.Plane(origin=points[0],normal=edge.tangentAt(0))
    profile=cq.Workplane(plane).circle(radius)
    if inner_radius is not None: profile=profile.circle(inner_radius)
    # Corrected Frenet transport avoids segmentation at spline inflections.
    return profile.sweep(cq.Workplane().newObject([wire]),isFrenet=False)

def palm_envelope():
    """Domed, tapered shell with a true curved underside, no flat rectangular slab.

    The concentric spheres are 6 mm apart. The inner plan is offset by 6 mm.
    R3 outer edge rounds use part of that stock; remaining material is designed
    for >=3 mm outside intentional holes/openings. Duct walls are 3 mm radial.
    The open underside is for assembly/access; it is not an anatomical socket.
    """
    outline=cq.Workplane('XY').polyline(PALM_OUTLINE).close().extrude(45).edges('|Z').fillet(4)
    outer=outline.intersect(cq.Workplane().sphere(230).translate((0,25,-205)))
    upper_edges=[e for e in outer.val().Edges() if e.BoundingBox().zmin>1]
    outer=outer.newObject(upper_edges).fillet(OUTER_FILLET)
    # Round the external lower rim before cutting the cavity.
    outer=outer.faces('<Z').edges().fillet(OUTER_FILLET)
    inner_plan=(cq.Workplane('XY').polyline(PALM_OUTLINE).close()
                .offset2D(-PALM_NOMINAL_WALL).extrude(45))
    cavity=inner_plan.intersect(cq.Workplane().sphere(230-PALM_NOMINAL_WALL).translate((0,25,-205)))
    return outer.cut(cavity),cavity

def route_points(i,pose):
    """Independent conduits fan out from six separate wrist manifold ports."""
    # Common manifold means a common housing, never shared cord lumens.
    # Thumb occupies the leftmost port to avoid crossing other flexor routes.
    sx=[-12,-6,0,6,12,-18][i]
    if i<5:
        x,y,_,_,_=pose
        return [(sx,-36,11),(sx,-22,12),(x*.65,10,17),
                (x,y-47,23),(x,y-34,38),(x,y-23,ROOT_Z+TENDON_Z),
                local_point(pose,-14,root_height(pose)+TENDON_Z)]
    return [(sx,-36,11),(-23,-22,12),(-52,-9,19),
            local_point(pose,-38,32),local_point(pose,-28,root_height(pose)+TENDON_Z),
            local_point(pose,-14,root_height(pose)+TENDON_Z)]

def make_palm():
    shell,cavity=palm_envelope()
    pedestal_solids=[]
    for pose in LAYOUT:
        # Feet intersect the shell, seat the removable bases, and avoid modifying
        # the hollow interior beyond their local reinforcement footprints.
        height=root_height(pose)-18
        foot=rounded_box(22,20,height,0,-19,-12-height/2,r=3)
        # Rounded block top is ROOT_Z-12 = base flange underside.
        pedestal_solids.append(place(foot,pose))
    shell=combine([shell]+pedestal_solids)
    # A central routing block at the wrist binds the duct entries to the chassis.
    shell=shell.union(rounded_box(44,18,12,-3.75,-27,10,r=3))
    routes=[route_points(i,p) for i,p in enumerate(LAYOUT)]
    # Cut the chassis lumens first, then add matching annular guide sweeps.
    # Some guided portions rise above the dome to meet removable finger bases.
    # Build annular sweeps directly to avoid coincident sweep-surface Boolean slivers.
    for pts in routes:shell=shell.cut(swept_circle(pts,TENDON_D/2))
    tubes=[swept_circle(p,DUCT_R,TENDON_D/2) for p in routes]
    shell=combine([shell]+tubes)
    for pose in LAYOUT:
        x,y,z=local_point(pose,-19,-5)
        shell=shell.cut(cylinder(2.2,root_height(pose)-7,(x,y,-5)))
    return trim_conduit_tips(shell),routes

def trim_conduit_tips(shell):
    """Leave a service gap before each removable base's upper tendon inlet.

    Trim only the elevated conduit; the pedestal below remains intact.
    The open cord span is intentional and must be checked during physical tests.
    """
    for pose in LAYOUT:
        cutter=place(box(14,8,14,0,-13,5),pose)
        shell=shell.cut(cutter)
    return shell.clean()

def digit_pose(parts,length,base_angle=0,tip_angle=0):
    b,p,d=parts
    pp=p.rotate((0,0,0),(1,0,0),base_angle)
    dd=(d.rotate((0,0,0),(1,0,0),tip_angle).translate((0,length,0))
          .rotate((0,0,0),(1,0,0),base_angle))
    return b,pp,dd

def compound(shapes):
    # Preserve separate solids; do not Boolean-fuse an articulated reference.
    return cq.Compound.makeCompound([s.val() for s in shapes])

def clean_stl(path):
    """Remove collapsed tessellation facets at 0.0001 mm vertex precision.

    OCC may emit zero-area seam triangles around rounded faces. This does not
    alter designed fit dimensions; it removes those collapsed mesh facets.
    """
    data=path.read_bytes()
    count=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+50*count:
        raise ValueError('Expected binary STL: '+str(path))
    kept=[]
    for offset in range(84,len(data),50):
        record=data[offset:offset+50]
        values=struct.unpack('<12fH',record)
        vertices=[tuple(round(v,4) for v in values[j:j+3]) for j in (3,6,9)]
        if len(set(vertices))==3: kept.append(record)
    path.write_bytes(data[:80]+struct.pack('<I',len(kept))+b''.join(kept))

def export_part(s,path):
    if not s.val().isValid() or len(s.solids().vals())!=1:
        raise ValueError('Invalid/disconnected printable component: '+path.stem)
    # Shift to bed, preserving size; slicer orientation still needs consideration.
    bed=s.translate((0,0,-s.val().BoundingBox().zmin))
    cq.exporters.export(bed,str(path),tolerance=.035,angularTolerance=.25)
    clean_stl(path)

def overlap_report(world):
    out=[]
    for i,(name,s) in enumerate(world):
        for other,t in world[i+1:]:
            a,b=s.val().BoundingBox(),t.val().BoundingBox()
            if a.xmax<b.xmin or b.xmax<a.xmin or a.ymax<b.ymin or b.ymax<a.ymin or a.zmax<b.zmin or b.zmax<a.zmin:continue
            volume=s.intersect(t).val().Volume()
            if volume>.02:out.append([name,other,round(volume,4)])
    return out

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=Path('exports'))
    parser.add_argument('--check-motion',action='store_true')
    args=parser.parse_args();out=args.out
    for folder in ['print_parts','reference','step']:(out/folder).mkdir(parents=True,exist_ok=True)
    print('Building contoured palm and six tendon conduits...',flush=True)
    palm,routes=make_palm();export_part(palm,out/'print_parts/palm.stl')
    assembly=cq.Assembly(name='SIX_DIGIT_V3')
    assembly.add(palm,name='palm',color=cq.Color(.3,.45,.55))
    world=[('palm',palm)];digit_parts=[]
    manifest={'units':'mm','nominal_pin_diameter':PIN_D,'radial_pin_clearance':PIN_RADIAL_GAP,
              'rotating_bore_diameter':PIN_HOLE_D,'clevis_side_clearance':CLEVIS_SIDE_GAP,
              'tendon_diameter':TENDON_D,'nominal_shell_stock':PALM_NOMINAL_WALL,
              'designed_minimum_shell_wall':PALM_MIN_WALL,'parts':[],'digit_count':6}
    for i,pose in enumerate(LAYOUT,1):
        parts=[make_base(),make_link(pose[3]),make_link(pose[4],True)];digit_parts.append(parts)
        group=cq.Assembly(name=f'digit_{i}');posed=digit_pose(parts,pose[3]);digit_world=[]
        for label,s,ps,col in zip(['base','proximal','distal'],parts,posed,[(.55,.65,.7),(.1,.65,.68),(.9,.65,.2)]):
            name=f'digit_{i}_{label}'
            export_part(s,out/'print_parts'/f'{name}.stl')
            cq.exporters.export(s,str(out/'step'/f'{name}.step'))
            w=place(ps,pose);world.append((name,w));digit_world.append(w)
            group.add(w,name=label,color=cq.Color(*col))
            manifest['parts'].append({'name':name,'quantity':1,'valid_solid':True})
        assembly.add(group)
        cq.exporters.export(compound(digit_world),str(out/'reference'/f'digit_{i}_POSED_DO_NOT_PRINT.stl'),tolerance=.1,angularTolerance=.3)
        print(f'Exported digit_{i}',flush=True)
    cq.exporters.export(palm,str(out/'step/palm.step'))
    assembly.save(str(out/'reference/FULL_ASSEMBLY.step'))
    cq.exporters.export(compound([s for _,s in world]),str(out/'reference/FULL_ASSEMBLY_DO_NOT_PRINT.stl'),tolerance=.1,angularTolerance=.3)
    manifest['parts'].insert(0,{'name':'palm','quantity':1,'valid_solid':True})
    manifest['routes_xyz']=routes
    manifest['open_pose_overlaps']=None
    manifest['collision_check_status']='Not run; use --check-motion for optional slow checks'
    (out/'dimensions_and_checks.json').write_text(json.dumps(manifest,indent=2))
    if args.check_motion:
        manifest['open_pose_overlaps']=overlap_report(world)
        manifest['collision_check_status']='Open-pose check complete; sampled motion in progress'
        (out/'dimensions_and_checks.json').write_text(json.dumps(manifest,indent=2))
        manifest['sampled_motion']=[]
        for angle in [30,60]:
            moving=[('palm',palm)]
            for i,(pose,parts) in enumerate(zip(LAYOUT,digit_parts),1):
                for label,s in zip(['base','proximal','distal'],digit_pose(parts,pose[3],angle,angle)):
                    moving.append((f'digit_{i}_{label}',place(s,pose)))
            manifest['sampled_motion'].append({'base_and_tip_degrees':angle,'overlaps':overlap_report(moving)})
        manifest['collision_check_status']='Completed sampled checks; inspect reported overlaps'
    (out/'dimensions_and_checks.json').write_text(json.dumps(manifest,indent=2))
    print('Finished. Assembly references are for viewing only. Print the 19 separate parts.',flush=True)
    print('Open-pose overlaps:',manifest['open_pose_overlaps'],flush=True)

if __name__=='__main__': main()
