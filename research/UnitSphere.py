import matplotlib.pyplot as plt
from sympy import *

x1, x, v1, v = symbols("x1 x v1 v")
ta1 = (-2*v1 + (4*v1**2 + 4*(v**2 - v1**2)/2  + 4*(x-x1))**0.5)/2
T = (v1-v) + 2*ta1
expr = T - 1
expr = expr.subs([(x1,10),(v1,2)])
print expr
plot_implicit(expr)
