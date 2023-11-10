from casadi import *

import matplotlib.pyplot as plt
import matplotlib
from pylab import figure, colorbar, draw, show, plot, title
import numpy as np
import random
import time

delta = 0.1
minXY=-5.0
maxXY=5.0
nContour=50
alpha=0.01

def HimmelblauFunction(x,y):
    u"""
    Himmelblau's function
    see Himmelblau's function - Wikipedia, the free encyclopedia 
    http://en.wikipedia.org/wiki/Himmelblau%27s_function
    """
    return (x**2+y-11)**2+(x+y**2-7)**2

def CreateMeshData():
    x = np.arange(minXY, maxXY, delta)
    y = np.arange(minXY, maxXY, delta)
    X, Y = np.meshgrid(x, y)
    Z=[HimmelblauFunction(x,y) for (x,y) in zip(X,Y)]
    return(X,Y,Z)


class MyCallback(Callback):
    def __init__(self, name, nx, ng, np, opts={}):
        Callback.__init__(self)

        self.nx = nx
        self.ng = ng
        self.np = np

        figure(1)

        (X,Y,Z)=CreateMeshData()
        plt.contour(X, Y, Z, nContour)
        colorbar()
        title('Iterations of Rosenbrock')
        draw()

        self.x_sols = []
        self.y_sols = []

        # Initialize internal objects
        self.construct(name, opts)

    def get_n_in(self): return nlpsol_n_out()
    def get_n_out(self): return 1
    def get_name_in(self, i): return nlpsol_out(i)
    def get_name_out(self, i): return "ret"

    def get_sparsity_in(self, i):
        n = nlpsol_out(i)
        if n=='f':
            return Sparsity. scalar()
        elif n in ('x', 'lam_x'):
            return Sparsity.dense(self.nx)
        elif n in ('g', 'lam_g'):
            return Sparsity.dense(self.ng)
        else:
            return Sparsity(0,0)
    def eval(self, arg):
        # Create dictionary
        darg = {}
        for (i,s) in enumerate(nlpsol_out()): darg[s] = arg[i]

        sol = darg['x']
        self.x_sols.append(float(sol[0]))
        self.y_sols.append(float(sol[1]))

        if hasattr(self,'lines'):
            if "template" not in matplotlib.get_backend(): # Broken for template: https://github.com/matplotlib/matplotlib/issues/8516/
                self.lines[0].set_data(self.x_sols,self.y_sols)

        else:
            self.lines = plot(self.x_sols,self.y_sols,'or-')

        draw()
        time.sleep(0.2)

        return [0]


def main(args=None):
    mycallback = MyCallback('mycallback', 2, 0, 0)
    x = SX.sym("x")
    y = SX.sym("y")

    # Himmelblau's function
    f = HimmelblauFunction(x, y)
    nlp = {'x':vertcat(x,y), 'f':f }
    
    opts = {}
    opts["verbose"] = True
    opts['iteration_callback'] = mycallback
  
    # Create an NLP solver
    solver = nlpsol("solver", "ipopt", nlp, opts)
    start=np.array([random.uniform(minXY,maxXY),random.uniform(minXY,maxXY)])

    # Solve the Himmelblau's problem
    res = solver(x0 = start)
    
    # Print solution
    print()
    print("%50s " % "Optimal cost:", res["f"])
    print("%50s " % "Primal solution:", res["x"])
    print("%50s " % "Dual solution (simple bounds):", res["lam_x"])
    print("%50s " % "Dual solution (nonlinear bounds):", res["lam_g"])

    matplotlib.interactive(False)
    show()


if __name__ == '__main__':
    main()
