import math, time


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


def eval_Bezier3(P, t):
    # P(t) = (1-t).B1(t) + t.B2(t)
    res = [0.0, 0.0]
    B1 = eval_Bezier2(P[:3], t)
    B2 = eval_Bezier2(P[1:], t)
    for xy in range(2):
        res[xy] = (1 - t) * B1[xy] + t * B2[xy]
    return res


def eval_dBezier3(P, t):
    # P'(t) = B1'(t).(1-t) + B2'(t).t - B1(t) + B2(t)
    res = [0.0, 0.0]
    B1 = eval_Bezier2(P[:3], t)
    B2 = eval_Bezier2(P[1:], t)
    dB1 = eval_dBezier2(P[:3], t)
    dB2 = eval_dBezier2(P[1:], t)
    for xy in range(2):
        res[xy] = dB1[xy] * (1 - t) + dB2[xy] * t - B1[xy] + B2[xy]
    return res


def arc_length_Bezier2_approx(P, nsteps):
    s = 0
    delta = 1 / nsteps
    t = delta
    B0 = [0, 0]
    for i in range(nsteps):
        B1 = eval_Bezier2(P, t)
        s += math.sqrt((B1[0] - B0[0]) ** 2 + (B1[1] - B0[1]) ** 2)
        B0 = B1
        t += delta
    return s


def f(t, graad):
    if graad == 2:
        return math.sqrt(eval_dBezier2(B2, t)[0] ** 2 + eval_dBezier2(B2, t)[1] ** 2)
    if graad == 3:
        return math.sqrt(eval_dBezier3(B3, t)[0] ** 2 + eval_dBezier3(B3, t)[1] ** 2)
    return 0


def Simpson_sum(x, a, b, graad):
    s = 0
    n = len(x) - 1
    delta_x_div_6 = (b - a) / (6 * n)
    for i in range(n):
        s += (
            f(x[i], graad) + 4 * f((x[i] + x[i + 1]) / 2, graad) + f(x[i + 1], graad)
        ) * delta_x_div_6
    return s


def generate_Riemann_reg_grid(x, n, a, b):
    delta_x = (b - a) / n
    for i in range(n + 1):
        x.append(a + i * delta_x)


def arc_length_Bezier2(P, t):
    ax = 2 * (P[1][0] - P[0][0])
    ay = 2 * (P[1][1] - P[0][1])
    bx = 2 * (P[2][0] - P[1][0])
    by = 2 * (P[2][1] - P[1][1])
    A = (bx - ax) ** 2 + (by - ay) ** 2
    B = ax * (bx - ax) + ay * (by - ay)
    C = ax**2 + ay**2
    b = B / A
    c = C / A
    k = -(b**2) + c
    sqrt_tbk = math.sqrt((t + b) ** 2 + k)
    sqrt_bk = math.sqrt(b**2 + k)
    return (
        math.sqrt(A)
        / 2
        * (
            (t + b) * sqrt_tbk
            - b * sqrt_bk
            + k * math.log((sqrt_tbk + t + b) / (sqrt_bk + b))
        )
    )


def arc_length_Bezier3_approx(P, t, nsteps):
    s = 0
    delta = 1 / nsteps
    t = delta
    B0 = [0, 0]
    for i in range(nsteps):
        B1 = eval_Bezier3(P, t)
        s += math.sqrt((B1[0] - B0[0]) ** 2 + (B1[1] - B0[1]) ** 2)
        B0 = B1
        t += delta
    return s


def printLenBezier2():
    print(f"Bezier curve van: {B2}")
    t_min = 0.0
    t_max = 1.0
    t_grid = []
    n_grid = 20

    generate_Riemann_reg_grid(t_grid, n_grid, t_min, t_max)

    print(f"lengte exact:\t\t{arc_length_Bezier2(B2,1)}")
    print(f"lengte Simpson:\t\t{Simpson_sum(t_grid, t_min, t_max, 2)}")
    print(f"lengte brute force:\t\t{arc_length_Bezier2_approx(B2, 1000)}")

    t0 = time.perf_counter()
    dum = arc_length_Bezier2(B2, 1)
    t1 = time.perf_counter()
    print(f"timing exact:\t\t{t1-t0}")

    t0 = time.perf_counter()
    dum = Simpson_sum(t_grid, t_min, t_max, 2)
    t1 = time.perf_counter()
    print(f"timing Simpson:\t\t{t1-t0}")

    t0 = time.perf_counter()
    dum = arc_length_Bezier2_approx(B2, 1000)
    t1 = time.perf_counter()
    print(f"timing brute force:\t\t{t1-t0}\n")


def printLenBezier3():
    print(f"Bezier curve van: {B3}")
    t_min = 0.0
    t_max = 1.0
    t_grid = []
    n_grid = 100

    generate_Riemann_reg_grid(t_grid, n_grid, t_min, t_max)
    print(f"lengte Simpson:\t\t{Simpson_sum(t_grid, t_min, t_max, 3)}")
    for n_steps in [100, 1000, 10000]:
        print(
            f"lengte brute force (n = {n_steps}):\t\t{arc_length_Bezier3_approx(B3, t_max, n_steps)}"
        )


B2 = [[0.0, 0.0], [7.0, 0.0], [1.0, 4.0]]
printLenBezier2()
B2 = [[0.0, 0.0], [2.0, 5.0], [1.0, 4.0]]
printLenBezier2()


B3 = [[0.0, 0.0], [0.5, -0.5], [5.5, -0.5], [7.0, 4.0]]
printLenBezier3()
