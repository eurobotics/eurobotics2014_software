import math, sys, time, os, random, re
#from matplotlib.patches import Arrow, Circle, Wedge, Polygon, Rectangle
from visual import *

FLOAT = "([-+]?[0-9]*\.?[0-9]+)"
INT = "([-+]?[0-9][0-9]*)"

AREA_X = 3000.
AREA_Y = 2000.

ROBOT_HEIGHT = 350.0
WALL_HEIGHT = 70.0

ROBOT_WIDTH  = 330.0
ROBOT_LENGTH = 282.0
ROBOT_X_OFFSET = 21.75
ROBOT2_WIDTH  = 230.0
ROBOT2_LENGTH = 150.0

area = [ (0.0, 0.0, -0.2), (3000.0, 2000.0, 0.2) ]
areasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , area)
area_box = box(size=areasize, color=(0.0, 1.0, 0.0))

scene.autoscale = 1

# all positions of robot every 5ms
save_pos = []
save_pos2 = []

robot = box(color=(0.4, 0.4, 0.4))
robot2 = box(color=(0.4, 0.4, 0.4))
#lspickle = box(color=(0.4, 0.4, 0.4))
#rspickle = box(color=(0.4, 0.4, 0.4))

opp = box(color=(0.7, 0.2, 0.2))
opp2 = box(color=(0.2, 0.2, 0.7))

last_pos = (0.,0.,0.)
last_pos2 = (0.,0.,0.)

hcenter_line = curve()
hcenter_line.pos = [(-AREA_X/2, 0., 0.3), (AREA_X/2, 0., 0.3)]
vcenter_line = curve()
vcenter_line.pos = [(0., -AREA_Y/2, 0.3), (0., AREA_Y/2, 0.3)]

yellowarea = [ (0.0, 0.0, -0.5), (400.0, 700.0, 0.5) ]
yellowareasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , yellowarea)
yellowarea_box = box(pos=(-AREA_X/2+200,-AREA_Y/2+350,0), size=yellowareasize, color=(1.0, 1.0, 0.0))

redarea = [ (0.0, 0.0, -0.5), (400.0, 700.0, 0.5) ]
redareasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , redarea)
redarea_box = box(pos=(AREA_X/2-200,-AREA_Y/2+350,0), size=redareasize, color=(1.0, 0.0, 0.0))

wallx = [ (0.0, 0.0, -0.5), (AREA_X+44, 22, WALL_HEIGHT) ]
wallxsize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , wallx)
wallx1_box = box(pos=(0,-AREA_Y/2-11, WALL_HEIGHT/2), size=wallxsize, color=(0.5, 0.5, 0.5))
wallx2_box = box(pos=(0,AREA_Y/2+11, WALL_HEIGHT/2), size=wallxsize, color=(0.5, 0.5, 0.5))

wally = [ (0.0, 0.0, -0.5), (22, AREA_Y+44, WALL_HEIGHT) ]
wallysize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , wally)
wally1_box = box(pos=(-AREA_X/2-11, 0, WALL_HEIGHT/2), size=wallysize, color=(0.5, 0.5, 0.5))
wally2_box = box(pos=(AREA_X/2+11, 0, WALL_HEIGHT/2), size=wallysize, color=(0.5, 0.5, 0.5))


YELLOW = 0
RED    = 1
color = YELLOW

def square(sz):
    sq = curve()
    sq.pos = [(-sz, -sz, 0.3),
              (-sz, sz, 0.3),
              (sz, sz, 0.3),
              (sz, -sz, 0.3),
              (-sz, -sz, 0.3),]
    return sq

sq1 = square(250)
sq2 = square(500)

robot_x = 0.
robot_y = 0.
robot_a = 0.
robot2_x = 0.
robot2_y = 0.
robot2_a = 0.
#robot_lspickle_deployed = 0
#robot_rspickle_deployed = 0
#robot_lspickle_autoharvest = 0
#robot_rspickle_autoharvest = 0 
robot_trail = curve()
robot_trail_list = []
robot2_trail = curve()
robot2_trail_list = []
max_trail = 500

area_objects = []

BASKET_HEIGHT = 44.0
MAMUT_HEIGHT =  50.0
FRESCO_HEIGHT = 200.0
HEARTFIRE_HEIGHT = 30.0
TREETRUNK_HEIGHT = 320.0
TREETOP_HEIGHT = 10.0
FIRE_HEIGHT= 140.0

set_opp_nb = 1;

def toggle_obj_disp():
    global area_objects

    
    if area_objects == []:
        yellowbasket = [ (0.0, 0.0, -0.5), (700.0, 300.0, BASKET_HEIGHT) ]
        yellowbasketsize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , yellowbasket)
        c = box(pos=(AREA_X/2-400-350,-AREA_Y/2+150,BASKET_HEIGHT/2), size=yellowbasketsize, color=(1.0, 1.0, 0.0))
        area_objects.append(c)

        redbasket = [ (0.0, 0.0, -0.5), (700.0, 300.0, BASKET_HEIGHT) ]
        redbasketsize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , redbasket)
        c = box(pos=(-AREA_X/2+400+350,-AREA_Y/2+150,BASKET_HEIGHT/2), size=redbasketsize, color=(1.0, 0.0, 0.0))
        area_objects.append(c)

        mamut = [ (0.0, 0.0, -0.5), (700.0, 22.0, MAMUT_HEIGHT) ]
        mamutsize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , mamut)
        c = box(pos=(-AREA_X/2+400+350,-AREA_Y/2-11,MAMUT_HEIGHT/2+70), size=mamutsize, color=(0.6, 0.3, 0.0))
        area_objects.append(c)
        c = box(pos=(AREA_X/2-400-350,-AREA_Y/2-11,MAMUT_HEIGHT/2+70), size=mamutsize, color=(0.6, 0.3, 0.0))
        area_objects.append(c)
	      
        fire = [ (0.0, 0.0, -0.5), (22.0,140.0, FIRE_HEIGHT) ]
        firesize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , fire)
        c = box(pos=(-AREA_X/2,-AREA_Y/2+800,FIRE_HEIGHT/2), size=firesize, color=(1, 1, 0.0))
        area_objects.append(c)
        c = box(pos=(AREA_X/2,-AREA_Y/2+800,FIRE_HEIGHT/2), size=firesize, color=(1, 0, 0.0))
        area_objects.append(c)
        c = box(pos=(-AREA_X/2+900,-AREA_Y/2+600,FIRE_HEIGHT/2), size=firesize, color=(1, 0.0, 0.0))
        area_objects.append(c)
        c = box(pos=(AREA_X/2-900,-AREA_Y/2+600,FIRE_HEIGHT/2), size=firesize, color=(1, 1, 0.0))
        area_objects.append(c)
        c = box(pos=(-AREA_X/2+900,600,FIRE_HEIGHT/2), size=firesize, color=(1, 0.0, 0.0))
        area_objects.append(c)
        c = box(pos=(AREA_X/2-900,600,FIRE_HEIGHT/2), size=firesize, color=(1,1, 0.0))
        area_objects.append(c)

        fire2 = [ (0.0, 0.0, -0.5), (140.0,22.0, FIRE_HEIGHT) ]
        firesize2 = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , fire2)
        c = box(pos=(-AREA_X/2+400,100,FIRE_HEIGHT/2), size=firesize2, color=(1, 1, 0.0))
        area_objects.append(c)
        c = box(pos=(AREA_X/2-400,100,FIRE_HEIGHT/2), size=firesize2, color=(1, 0.0, 0.0))
        area_objects.append(c)
        c = box(pos=(-200,+AREA_Y/2,FIRE_HEIGHT/2), size=firesize2, color=(1,0.0, 0.0))
        area_objects.append(c)
        c = box(pos=(+200,+AREA_Y/2,FIRE_HEIGHT/2), size=firesize2, color=(1,1 , 0.0))
        area_objects.append(c)

        fresco = [ (0.0, 0.0, -0.5), (600.0, 22.0, FRESCO_HEIGHT) ]
        frescosize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , fresco)
        c = box(pos=(0,-AREA_Y/2-11, FRESCO_HEIGHT/2+70), size=frescosize, color=(0.5, 0.5, 0.5))
        area_objects.append(c)

        c = cylinder(axis=(0,0,1), length=HEARTFIRE_HEIGHT,
                     radius=150, color=(0.6, 0.3, 0.0),
                     pos=(0,50,HEARTFIRE_HEIGHT/2))
        area_objects.append(c)

        c = cylinder(axis=(0,0,1), length=HEARTFIRE_HEIGHT,
                     radius=250, color=(0.6, 0.3, 0.0),
                     pos=(-AREA_X/2,AREA_Y/2, 0))
        area_objects.append(c)

        c = cylinder(axis=(0,0,1), length=HEARTFIRE_HEIGHT,
                     radius=250, color=(0.6, 0.3, 0.0),
                     pos=(AREA_X/2,AREA_Y/2, 0))
        area_objects.append(c)

        # tree trunks
        c = cylinder(axis=(0,0,1), length=TREETRUNK_HEIGHT,
                     radius=25, color=(0.6, 0.3, 0.0),
                     pos=(-AREA_X/2,-AREA_Y/2+1300, 0))
        area_objects.append(c)

        c = cylinder(axis=(0,0,1), length=TREETRUNK_HEIGHT,
                     radius=25, color=(0.6, 0.3, 0.0),
                     pos=(AREA_X/2,-AREA_Y/2+1300, 0))
        area_objects.append(c)

        c = cylinder(axis=(0,0,1), length=TREETRUNK_HEIGHT,
                     radius=25, color=(0.6, 0.3, 0.0),
                     pos=(-AREA_X/2+700, AREA_Y/2, 0))
        area_objects.append(c)

        c = cylinder(axis=(0,0,1), length=TREETRUNK_HEIGHT,
                     radius=25, color=(0.6, 0.3, 0.0),
                     pos=(AREA_X/2-700, AREA_Y/2, 0))
        area_objects.append(c)

        # tree top
        c = cylinder(axis=(0,0,1), length=TREETOP_HEIGHT,
                     radius=150, color=(0.0, 1.0, 0.0),
                     pos=(-AREA_X/2,-AREA_Y/2+1300, TREETRUNK_HEIGHT))
        area_objects.append(c)

        c = cylinder(axis=(0,0,1), length=TREETOP_HEIGHT,
                     radius=150, color=(0.0, 1.0, 0.0),
                     pos=(AREA_X/2,-AREA_Y/2+1300, TREETRUNK_HEIGHT))
        area_objects.append(c)

        c = cylinder(axis=(0,0,1), length=TREETOP_HEIGHT,
                     radius=150, color=(0.0, 1.0, 0.0),
                     pos=(-AREA_X/2+700, AREA_Y/2, TREETRUNK_HEIGHT))
        area_objects.append(c)

        c = cylinder(axis=(0,0,1), length=TREETOP_HEIGHT,
                     radius=150, color=(0.0, 1.0, 0.0),
                     pos=(AREA_X/2-700, AREA_Y/2, TREETRUNK_HEIGHT))
        area_objects.append(c)


    else:
        for o in area_objects:
            if o.visible:
                o.visible = 0
            else:
                o.visible = 1

def toggle_color():
    global color
    global RED, YELLOW
    if color == YELLOW:
        color = RED
    else:
        color = YELLOW

def set_opp(x, y):
    opp.size = (300, 300, ROBOT_HEIGHT)
    opp.pos = (x, y, ROBOT_HEIGHT/2)

def set_opp2(x, y):
    opp2.size = (300, 300, ROBOT_HEIGHT)
    opp2.pos = (x, y, ROBOT_HEIGHT/2)

def set_robot():
    global robot, last_pos, robot_trail, robot_trail_list
    global save_pos, robot_x, robot_y, robot_a

    if color == YELLOW:
        tmp_x = robot_x - AREA_X/2
        tmp_y = robot_y - AREA_Y/2
        tmp_a = robot_a
    else:
        tmp_x = -robot_x + AREA_X/2
        tmp_y = -robot_y + AREA_Y/2
        tmp_a = robot_a

    robot.pos = (tmp_x+ROBOT_X_OFFSET, tmp_y, ROBOT_HEIGHT/2)
    axis = (math.cos(tmp_a*math.pi/180),
            math.sin(tmp_a*math.pi/180),
            0)

    robot.axis = axis
    robot.size = (ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT)
    """
    lspickle.pos = (tmp_x + (robot_lspickle_deployed*60) * math.cos((tmp_a+90)*math.pi/180),
                    tmp_y + (robot_lspickle_deployed*60) * math.sin((tmp_a+90)*math.pi/180),
                    ROBOT_HEIGHT/2)
    lspickle.axis = axis
    lspickle.size = (20, ROBOT_WIDTH, 5)
    if robot_lspickle_autoharvest:
        lspickle.color = (0.2, 0.2, 1)
    else:
        lspickle.color = (0.4, 0.4, 0.4)

    rspickle.pos = (tmp_x + (robot_rspickle_deployed*60) * math.cos((tmp_a-90)*math.pi/180),
                    tmp_y + (robot_rspickle_deployed*60) * math.sin((tmp_a-90)*math.pi/180),
                    ROBOT_HEIGHT/2)
    rspickle.axis = axis
    rspickle.size = (20, ROBOT_WIDTH, 5)
    if robot_rspickle_autoharvest:
        rspickle.color = (0.2, 0.2, 1)
    else:
        rspickle.color = (0.4, 0.4, 0.4)
    """
    # save position
    save_pos.append((robot.pos.x, robot.pos.y, tmp_a))

    pos = robot.pos.x, robot.pos.y, 0.3
    if pos != last_pos:
        robot_trail_list.append(pos)
        last_pos = pos
    robot_trail_l = len(robot_trail_list)
    if robot_trail_l > max_trail:
        robot_trail_list = robot_trail_list[robot_trail_l - max_trail:]
    robot_trail.pos = robot_trail_list

def set_robot2():
    global robot2, last_pos2, robot2_trail, robot2_trail_list
    global save_pos2, robot2_x, robot2_y, robot2_a

    if color == YELLOW:
        tmp_x = robot2_x - AREA_X/2
        tmp_y = robot2_y - AREA_Y/2
        tmp_a = robot2_a
    else:
        tmp_x = -robot2_x + AREA_X/2
        tmp_y = -robot2_y + AREA_Y/2
        tmp_a = robot2_a

    robot2.pos = (tmp_x, tmp_y, ROBOT_HEIGHT/2)
    axis = (math.cos(tmp_a*math.pi/180),
            math.sin(tmp_a*math.pi/180),
            0)

    robot2.axis = axis
    robot2.size = (ROBOT2_LENGTH, ROBOT2_WIDTH, ROBOT_HEIGHT)

    # save position
    save_pos2.append((robot2.pos.x, robot2.pos.y, tmp_a))

    pos2 = robot2.pos.x, robot2.pos.y, 0.3
    if pos2 != last_pos2:
        robot2_trail_list.append(pos2)
        last_pos2 = pos2
    robot2_trail_l = len(robot2_trail_list)
    if robot2_trail_l > max_trail:
        robot2_trail_list = robot2_trail_list[robot2_trail_l - max_trail:]
    robot2_trail.pos = robot2_trail_list

def graph():
    pass

def save():
    f = open("/tmp/robot_save", "w")
    for p in save_pos:
        f.write("%f %f %f\n"%(p[0], p[1], p[2]))
    f.close()

def silent_mkfifo(f):
    try:
        os.mkfifo(f)
    except:
        pass

toggle_obj_disp()

while True:
    silent_mkfifo("/tmp/.robot_sim2dis")
    silent_mkfifo("/tmp/.robot_dis2sim")
    silent_mkfifo("/tmp/.robot2_sim2dis")
    silent_mkfifo("/tmp/.robot2_dis2sim")

    while True:
        fr = open("/tmp/.robot_sim2dis", "r")
        fw = open("/tmp/.robot_dis2sim", "w", 0)
        fr2 = open("/tmp/.robot2_sim2dis", "r")
        fw2 = open("/tmp/.robot2_dis2sim", "w", 0)

        while True:

            # MAIN ROBOT MSGS
            m = None
            l = fr. readline()
            l2 = fr2. readline()

            # parse position
            if not m:
                m = re.match("pos=%s,%s,%s"%(INT,INT,INT), l)
                if m:
                    robot_x = int(m.groups()[0])
                    robot_y = int(m.groups()[1])
                    robot_a = int(m.groups()[2])
                    set_robot()
                    # XXX HACK, send pos robot mate
                    fw2.write("r2nd %d %d %d"%(int(robot_x), int(robot_y), int(robot_a)))
                    
                m = re.match("pos=%s,%s,%s"%(INT,INT,INT), l2)
                if m:
                    robot2_x = int(m.groups()[0])
                    robot2_y = int(m.groups()[1])
                    robot2_a = int(m.groups()[2])
                    set_robot2()
                    # XXX HACK, send pos robot mate
                    fw.write("r2nd %d %d %d"%(int(robot2_x), int(robot2_y), int(robot2_a)))

            """
            # TODO parse slavedspic
            if not m:
                m = re.match("ballboard=%s"%(INT), l)
                if m:
                    print "ballboard: %d"%(int(m.groups()[0]))

            # parse cobboard
            if not m:
                m = re.match("cobboard=%s,%s"%(INT,INT), l)
                if m:
                    print "cobboard: %x,%x"%(int(m.groups()[0]),int(m.groups()[1]))
                    side = int(m.groups()[0])
                    flags = int(m.groups()[1])
                    if (side == 0 and color == YELLOW) or (side == 1 and color == RED):
                        robot_lspickle_deployed = ((flags & 1) * 2)
                        robot_lspickle_autoharvest = ((flags & 2) != 0)
                    else:
                        robot_rspickle_deployed = ((flags & 1) * 2)
                        robot_rspickle_autoharvest = ((flags & 2) != 0)
            """
            # DISPLAY EVENTS
            if scene.mouse.events != 0:
                if set_opp_nb == 1:
                    oppx, oppy, oppz = scene.mouse.getevent().project(normal=(0,0,1))
                    set_opp(oppx, oppy)
                    try:
                        if color == YELLOW:
                          fw.write("opp_1 %d %d"%(int(oppx + 1500), int(oppy + 1050)))
                          fw2.write("opp_1 %d %d"%(int(oppx + 1500), int(oppy + 1050)))
                        else:
                          fw.write("opp_1 %d %d"%(int(1500 - oppx), int(1050 - oppy)))
                          fw2.write("opp_1 %d %d"%(int(1500 - oppx), int(1050 - oppy)))
                    except:
                        print "not connected"
                else:
                    opp2x, opp2y, opp2z = scene.mouse.getevent().project(normal=(0,0,1))
                    set_opp2(opp2x, opp2y)
                    try:
                        if color == YELLOW:
                            fw.write("opp_2 %d %d"%(int(opp2x + 1500), int(opp2y + 1050)))
                            fw2.write("opp_2 %d %d"%(int(opp2x + 1500), int(opp2y + 1050)))
                        else:
                            fw.write("opp_2 %d %d"%(int(1500 - opp2x), int(1050 - opp2y)))
                            fw2.write("opp_2 %d %d"%(int(1500 - opp2x), int(1050 - opp2y)))
                    except:
                        print "not connected"

            if scene.kb.keys == 0:
                continue

            k = scene.kb.getkey()
            x,y,z = scene.center
            if k == "left":
                scene.center = x-10,y,z
            elif k == "right":
                scene.center = x+10,y,z
            elif k == "up":
                scene.center = x,y+10,z
            elif k == "down":
                scene.center = x,y-10,z
            #elif k == "l":
            #    fw.write("l")
            #elif k == "r":
            #    fw.write("r")
            #elif k == "b":
            #    fw.write("b")
            elif k == "c":
                robot_trail_list = []
            elif k == "x":
                save_pos = []
            elif k == "g":
                graph()
            elif k == "s":
                save()
            elif k == "h":
                toggle_obj_disp()
            elif k == "i":
                toggle_color()
            elif k == "1":
                set_opp_nb = 1;
            elif k == "2":
                set_opp_nb = 2;

            else:
                print k

            # EOF
            if l == "":
                break

        fr.close()
        fw.close()


