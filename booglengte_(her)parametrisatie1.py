from tkinter import Tk, Canvas
from graphics_template import *
import math, time

vp_width, vp_height = 1024, 768
w_xmin, w_ymin, w_xmax = -3, -3, 10
w_ymax = w_ymin + (w_xmax - w_xmin) / vp_width * vp_height

animation_done = False

B2 = [[0.0, 0.0], [2.0, 5.0], [1.0, 4.0]]
t_min = 0
t_max = 1
V_pos = []
V_vec = []


def eval_Bezier2(P, t):
    # P(t) = (1-t)².P[0] + 2t(1-t)P[1] + t².P[2]
    res = [0.0, 0.0]
    for xy in range(2):
        res[xy] = (
            ((1 - t) ** 2) * P[0][xy] + 2 * t * (1 - t) * P[1][xy] + (t**2) * P[2][xy]
        )
    return res


def eval_dBezier2(P, t):
    # P'(t) = 2.t.(P[0] - 2.P[1] + P[2]) + 2.P[1]
    res = [0.0, 0.0]
    for xy in range(2):
        res[xy] = 2 * t * (P[0][xy] - 2 * P[1][xy] + P[2][xy]) + 2 * P[1][xy]
    return res


def arc_length_exact(t):
    root = math.sqrt(4 * t**2 + 1)
    return 1 / 2 * root * t + 1 / 4 * math.log(root + 2 * t)


def d_arc_length_exact(t):
    return math.sqrt(4 * t**2 + 1)


def get_NR_curve_t(s):
    t = t_min + s / arc_length_exact(t_max) * (t_max - t_min)
    print("init t0", t)
    for it in range(5):
        t = t - (arc_length_exact(t) - s) / d_arc_length_exact(t)
        print("it t", t)
    return t


def do_animation(s):
    global animation_done
    speed = 1 / 2
    u = s * speed
    t = get_NR_curve_t(u)

    if t > t_max:
        animation_done = True
    else:
        pos = eval_Bezier2(B2, t)
        V_pos[0] = pos[0]
        V_pos[1] = pos[1]
        vec = eval_dBezier2(B2, t)
        V_vec[0] = speed * vec[0] / math.sqrt(vec[0] ** 2 + vec[1] ** 2)
        V_vec[1] = speed * vec[1] / math.sqrt(vec[0] ** 2 + vec[1] ** 2)


def draw_Bezier(P, nsteps):
    xi = P[0][0]
    yi = P[0][1]
    t_delta = 1 / nsteps
    t = t_delta
    for ti in range(nsteps):
        p = eval_Bezier2(P, t)
        draw_line(canvas, xi, yi, p[0], p[1], rgb_col(255, 0, 0))
        draw_small_square(canvas, xi, yi, rgb_col(255, 255, 0))
        xi = p[0]
        yi = p[1]
        t += t_delta
    for i in range(len(P)):
        draw_small_square(canvas, P[i][0], P[i][1], rgb_col(0, 255, 0))


def draw_scene():
    draw_grid(canvas)
    draw_axis(canvas)
    draw_Bezier(B2, 20)
    GREEN = rgb_col(0, 255, 0)
    draw_dot(canvas, V_pos[0], V_pos[1], GREEN)
    draw_line(
        canvas, V_pos[0], V_pos[1], V_pos[0] + V_vec[0], V_pos[1] + V_vec[1], GREEN
    )


def init_scene():
    V_pos.append(0.0)
    V_pos.append(0.0)
    V_vec.append(0.0)
    V_vec.append(0.0)
    do_animation(0.0)
    draw_scene()


window = Tk()
canvas = Canvas(window, width=vp_width, height=vp_height, bg=rgb_col(0, 0, 0))
canvas.pack()

init_graphics(vp_width, vp_height, w_xmin, w_ymin, w_xmax)

init_time = time.perf_counter()
prev_draw_time = 0
prev_sim_time = 0

init_scene()


while not animation_done:
    draw_dt = time.perf_counter() - init_time - prev_draw_time
    if draw_dt > 0.02:
        prev_draw_time += draw_dt
        do_animation(prev_draw_time)
        canvas.delete("all")
        draw_scene()
        canvas.update()

window.mainloop()
