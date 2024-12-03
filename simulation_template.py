from tkinter import Tk, Canvas
from graphics_template import *
import math, time

vp_width, vp_height = 1024, 768
w_xmin, w_ymin, w_xmax = -10, 0, 10
w_ymax = w_ymin + (w_xmax - w_xmin)/vp_width * vp_height

simulation_done = False
DELTA_TSIM = 0.0005
DELTA_TDRAW = 0.02  # 50 fps

CEILING = w_ymax - 0.5
FLOOR = w_ymin + 0.5

pos_x = 0.0
pos_y = []


def left_click (event):
    global simulation_done
    simulation_done = True


def do_simulation (dt):
    if (pos_y[0] > FLOOR):
        # each dt the object moves 2*dt meters down
        # i.e.: velocity is 2 m/s down
        pos_y[0] -= 2 * dt
    else:
        # if object reaches FLOOR, it is repositioned at CEILING
        pos_y[0] = CEILING


def draw_scene ():
    #draw_grid (canvas)
    RED = rgb_col (255,0,0)
    GREEN = rgb_col (0, 255, 0)
    YELLOW = rgb_col (255, 255, 0) 
    draw_line (canvas, w_xmin, FLOOR, w_xmax, FLOOR, RED)
    draw_line (canvas, w_xmin/2, CEILING, w_xmax/2, CEILING, GREEN)
    draw_dot (canvas, pos_x, pos_y[0], YELLOW)


def init_scene ():
    pos_y.append(CEILING) # init y-coord
    draw_scene()
    
    
window = Tk()
canvas = Canvas(window, width=vp_width, height=vp_height, bg=rgb_col(0,0,0))
canvas.pack()
canvas.bind("<Button-1>", left_click)

init_graphics (vp_width, vp_height, w_xmin, w_ymin, w_xmax)


# time.perf_counter() -> float. Return the value (in fractional seconds)
# of a performance counter, i.e. a clock with the highest available resolution
# to measure a short duration. It does include time elapsed during sleep and
# is system-wide. The reference point of the returned value is undefined,
# so that only the difference between the results of consecutive calls is valid.

init_time = time.perf_counter()
prev_draw_time = 0
prev_sim_time = 0

init_scene ()

while (not simulation_done):
    # simulating
    sim_dt = time.perf_counter() - init_time - prev_sim_time
    if (sim_dt > DELTA_TSIM):
        do_simulation(DELTA_TSIM)
        prev_sim_time += DELTA_TSIM
    # drawing
    draw_dt = time.perf_counter() - init_time - prev_draw_time
    if (draw_dt > DELTA_TDRAW): # 50 fps
        canvas.delete("all")
        draw_scene()
        canvas.update()
        prev_draw_time += DELTA_TDRAW 

