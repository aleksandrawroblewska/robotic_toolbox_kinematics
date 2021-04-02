# wczytanie potrzebnych bibliotek:
import roboticstoolbox as rtb
import numpy as np
import math
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *
import time



def zadanie_1():
    robot = rtb.DHRobot(
        [
            rtb.RevoluteDH(alpha=pi() / 2, d=symbol('l1')),
            rtb.RevoluteDH(alpha=pi() / 2, offset=pi() / 2),
            rtb.PrismaticDH(offset=symbol('l2')),
        ], name="My_Robot")
    for i in range(0, 3):
        print(robot.links[i])
    return robot


def zadanie_3(robot):
    zmienne = [symbol('l1'), symbol('l2'), symbol('q3')]
    jakobian = robot.jacob0(zmienne)
    jakobian = simplify(jakobian)
    print(jakobian)



def zadanie_5():
    robot1 = rtb.models.DH.Puma560()
    T = robot1.fkine(robot1.qn)
    start = time.perf_counter()
    ik_solution = robot1.ikine_a(T=T, config="rd")
    stop = time.perf_counter()
    print('Blad rozwiazania metoda ikine_a: ', abs(len(T) - len(ik_solution)))
    print(f"time = {(stop - start) * 1000:.3f} ms")
    start = time.perf_counter()
    ikLM_solution = robot1.ikine_LM(T=T)
    stop = time.perf_counter()
    print('Blad rozwiazania metoda ikine_LM: ', ikLM_solution.residual)
    print(f"time = {(stop - start) * 1000:.3f} ms")
    start = time.perf_counter()
    ikmin_solution = robot1.ikine_min(qlim=False, T=T)
    stop = time.perf_counter()
    print('Blad rozwiazania metoda ikine_min (False): ', ikmin_solution.residual)
    print(f"time = {(stop - start) * 1000:.3f} ms")
    start = time.perf_counter()
    ikminT_solution = robot1.ikine_min(qlim=True, T=T)
    stop = time.perf_counter()
    print('Blad rozwiazania metoda ikine_min (True): ', ikminT_solution.residual)
    print(f"time = {(stop - start) * 1000:.3f} ms")
    #Najmniejszym bledem cechuje się metoda ikine_min(qlim=True, T=T)
    #Najszybsza metoda to ikine_a


if __name__ == '__main__':
 robot = zadanie_1()
 zadanie_3(robot)
 zadanie_5()


