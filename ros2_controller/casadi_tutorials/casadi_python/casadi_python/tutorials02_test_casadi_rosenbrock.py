from casadi import *

"""
Solve the Rosenbrock problem, formulated as the NLP:

minimize     x^2 + 100*z^2
subject to   z+(1-x)^2-y == 0

Joel Andersson, 2015
"""




def main(args=None):
    # rclpy.init(args=args)
    # tutorials_node = TutorialsCasADIEnvNode()
    # rclpy.spin(tutorials_node)
    # tutorials_node.destroy_node()
    # rclpy.shutdown()
    # Declare variables
    x = SX.sym("x")
    y = SX.sym("y")
    z = SX.sym("z")

    # Formulate the NLP
    f = x**2 + 100*z**2
    g = z + (1-x)**2 - y
    nlp = {'x':vertcat(x,y,z), 'f':f, 'g':g}

    # Create an NLP solver
    solver = nlpsol("solver", "ipopt", nlp)

    # Solve the Rosenbrock problem
    res = solver(x0  = [2.5,3.0,0.75],
                ubg = 0,
                lbg = 0)

    # Print solution
    print()
    print("%50s " % "Optimal cost:", res["f"])
    print("%50s " % "Primal solution:", res["x"])
    print("%50s " % "Dual solution (simple bounds):", res["lam_x"])
    print("%50s " % "Dual solution (nonlinear bounds):", res["lam_g"])

if __name__ == '__main__':
    main()
