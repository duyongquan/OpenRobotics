import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d


# https://xavierbourretsicotte.github.io/Intro_optimization.html

def Rosenbrock(x,y):
    return (1 + x)**2 + 100*(y - x**2)**2

def GradRosenbrock(x,y):
    g1 = -400*x*y + 400*x**3 + 2*x -2
    g2 = 200*y -200*x**2
    return np.array([g1,g2])

def HessianRosenbrock(x,y):
    h11 = -400*y + 1200*x**2 + 2
    h12 = -400 * x
    h21 = -400 * x
    h22 = 200
    return np.array([[h11,h12],[h21,h22]])
    
def GradientDescent(Grad,x,y, gamma = 0.00125, epsilon=0.0001, nMax = 10000 ):
    #Initialization
    i = 0
    iter_x, iter_y, iter_count = np.empty(0),np.empty(0), np.empty(0)
    error = 10
    X = np.array([x,y])
    
    #Looping as long as error is greater than epsilon
    while np.linalg.norm(error) > epsilon and i < nMax:
        i +=1
        iter_x = np.append(iter_x,x)
        iter_y = np.append(iter_y,y)
        iter_count = np.append(iter_count ,i)    
        
        X_prev = X
        X = X - gamma * Grad(x,y)
        error = X - X_prev
        x,y = X[0], X[1]
          
    print(X)
    return X, iter_x,iter_y, iter_count


def NewtonMethod():
    ## 1 Newton's Method
    root,iter_x,iter_y, iter_count = GradientDescent(GradRosenbrock,-2,2)
    x = np.linspace(-2,2,250)
    y = np.linspace(-1,3,250)
    X, Y = np.meshgrid(x, y)
    Z = Rosenbrock(X, Y)

    #Angles needed for quiver plot
    anglesx = iter_x[1:] - iter_x[:-1]
    anglesy = iter_y[1:] - iter_y[:-1]
    

    ## 2 Surface plot
    fig = plt.figure(figsize = (16,8))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.plot_surface(X,Y,Z,rstride = 5, cstride = 5, cmap = 'jet', alpha = .4, edgecolor = 'none' )
    ax.plot(iter_x,iter_y, Rosenbrock(iter_x,iter_y),color = 'r', marker = '*', alpha = .4)

    ax.view_init(45, 280)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    ax.set_title(r"$f(x) = (1 - x)^2 + 100(y - x^2)^2$",
                c='g', horizontalalignment='center', fontsize=10)

    #Contour plot
    ax = fig.add_subplot(1, 2, 2)
    ax.contour(X,Y,Z, 50, cmap = 'jet')
    #Plotting the iterations and intermediate values
    ax.scatter(iter_x,iter_y,color = 'r', marker = '*')
    ax.quiver(iter_x[:-1], iter_y[:-1], anglesx, anglesy, scale_units = 'xy', angles = 'xy', scale = 1, color = 'r', alpha = .3)
    ax.set_title('Gradient Descent with {} iterations'.format(len(iter_count)))

    plt.show()


def f(x,y):
    return .01*x**2 + .1*y**2

def Grad_f(x,y):
    g1 = 2*.01*x
    g2 = 2*.1*y
    return np.array([g1,g2])

def NewtonMethodQuadratic():
    root,iter_x,iter_y, iter_count = GradientDescent(Grad_f,-2,-2,1)

    x = np.linspace(-3,3,250)
    y = np.linspace(-3,3,250)
    X, Y = np.meshgrid(x, y)
    Z = f(X, Y)

    #Angles needed for quiver plot
    anglesx = iter_x[1:] - iter_x[:-1]
    anglesy = iter_y[1:] - iter_y[:-1]

    fig = plt.figure(figsize = (16,8))

    #Surface plot
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.plot_surface(X,Y,Z,rstride = 5, cstride = 5, cmap = 'jet', alpha = .4, edgecolor = 'none' )
    ax.plot(iter_x,iter_y, f(iter_x,iter_y),color = 'r', marker = '*', alpha = .4)
    ax.set_title('f(x,y) = .01x^2 + .1y^2')

    ax.view_init(65, 340)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    #Contour plot
    ax = fig.add_subplot(1, 2, 2)
    ax.contour(X,Y,Z, 50, cmap = 'jet')
    #Plotting the iterations and intermediate values
    ax.scatter(iter_x,iter_y,color = 'r', marker = '*')
    ax.quiver(iter_x[:-1], iter_y[:-1], anglesx, anglesy, scale_units = 'xy', angles = 'xy', scale = 1, color = 'r', alpha = .3)
    ax.set_title('Gradient Descent with {} iterations'.format(len(iter_count)))

    plt.show()

def Newton_Raphson_Optimize(Grad, Hess, x,y, epsilon=0.000001, nMax = 200):
    #Initialization
    i = 0
    iter_x, iter_y, iter_count = np.empty(0),np.empty(0), np.empty(0)
    error = 10
    X = np.array([x,y])
    
    #Looping as long as error is greater than epsilon
    while np.linalg.norm(error) > epsilon and i < nMax:
        i +=1
        iter_x = np.append(iter_x,x)
        iter_y = np.append(iter_y,y)
        iter_count = np.append(iter_count ,i)   
        print(X) 
        
        X_prev = X
        X = X - np.linalg.inv(Hess(x,y)) @ Grad(x,y)
        error = X - X_prev
        x,y = X[0], X[1]
          
    return X, iter_x,iter_y, iter_count


def NewtonMethodRaphsonOptimize():
    root,iter_x,iter_y, iter_count = Newton_Raphson_Optimize(GradRosenbrock,HessianRosenbrock,-2,2)

    x = np.linspace(-3,3,250)
    y = np.linspace(-9,8,350)
    X, Y = np.meshgrid(x, y)
    Z = Rosenbrock(X, Y)

    #Angles needed for quiver plot
    anglesx = iter_x[1:] - iter_x[:-1]
    anglesy = iter_y[1:] - iter_y[:-1]

    fig = plt.figure(figsize = (16,8))

    #Surface plot
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.plot_surface(X,Y,Z,rstride = 5, cstride = 5, cmap = 'jet', alpha = .4, edgecolor = 'none' )
    ax.plot(iter_x,iter_y, Rosenbrock(iter_x,iter_y),color = 'r', marker = '*', alpha = .4)

    #Rotate the initialization to help viewing the graph
    ax.view_init(45, 280)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    #Contour plot
    ax = fig.add_subplot(1, 2, 2)
    ax.contour(X,Y,Z, 60, cmap = 'jet')
    #Plotting the iterations and intermediate values
    ax.scatter(iter_x,iter_y,color = 'r', marker = '*')
    ax.quiver(iter_x[:-1], iter_y[:-1], anglesx, anglesy, scale_units = 'xy', angles = 'xy', scale = 1, color = 'r', alpha = .3)
    ax.set_title('Newton method with {} iterations'.format(len(iter_count)))

    plt.show()

#### Comparing Newton and Gradient Descent in presence of a single saddle point
def f_2(x,y):
    return .01*x**2 - .1*y**2

def Grad_f_2(x,y):
    g1 = 2*.01*x
    g2 = - 2*.1*y
    return np.array([g1,g2])

def Hessian_f_2(x,y):
    return np.array([[.02,0],[0,-.2]]) 

def NewtonAandGradientDescentOneSaddlePoint():
    root_gd,iter_x_gd,iter_y_gd, iter_count_gd = GradientDescent(Grad_f_2,-2.5,-0.010,1, nMax = 25)
    root_nr,iter_x_nr,iter_y_nr, iter_count_nr = Newton_Raphson_Optimize(Grad_f_2,Hessian_f_2,-2,-.01, nMax = 25)

    x = np.linspace(-3,3,100)
    y = np.linspace(-1,1,100)
    X, Y = np.meshgrid(x, y)
    Z = f_2(X, Y)

    #Angles needed for quiver plot
    anglesx = iter_x_gd[1:] - iter_x_gd[:-1]
    anglesy = iter_y_gd[1:] - iter_y_gd[:-1]
    anglesx_nr = iter_x_nr[1:] - iter_x_nr[:-1]
    anglesy_nr = iter_y_nr[1:] - iter_y_nr[:-1]

    fig = plt.figure(figsize = (16,8))

    #Surface plot
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.plot_surface(X,Y,Z,rstride = 5, cstride = 5, cmap = 'jet', alpha = .4, edgecolor = 'none' )
    ax.plot(iter_x_gd,iter_y_gd, f_2(iter_x_gd,iter_y_gd),color = 'r', marker = '*', alpha = .4, label = 'Gradient descent')
    ax.plot(iter_x_nr,iter_y_nr, f_2(iter_x_nr,iter_y_nr),color = 'darkblue', marker = 'o', alpha = .4, label = 'Newton')
    ax.legend()

    #Rotate the initialization to help viewing the graph
    #ax.view_init(45, 280)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    #Contour plot
    ax = fig.add_subplot(1, 2, 2)
    ax.contour(X,Y,Z, 60, cmap = 'jet')
    #Plotting the iterations and intermediate values
    ax.scatter(iter_x_gd,iter_y_gd,color = 'r', marker = '*', label = 'Gradient descent')
    ax.quiver(iter_x_gd[:-1], iter_y_gd[:-1], anglesx, anglesy, scale_units = 'xy', angles = 'xy', scale = 1, color = 'r', alpha = .3)
    ax.scatter(iter_x_nr,iter_y_nr,color = 'darkblue', marker = 'o',  label = 'Newton')
    ax.quiver(iter_x_nr[:-1], iter_y_nr[:-1], anglesx_nr, anglesy_nr, scale_units = 'xy', angles = 'xy', scale = 1, color = 'darkblue', alpha = .3)
    ax.legend()

    ax.set_title('Comparing Newton and Gradient descent')

    plt.show()


### Comparing Newton and Gradient Descent in presence of a multiple saddle points
# https://en.wikipedia.org/wiki/Himmelblau%27s_function

def Himmer(x,y):
    return (x**2 + y - 11)**2 + ( x + y**2 - 7 )**2

def Grad_Himmer(x,y):
    return np.array([2 * (-7 + x + y**2 + 2 * x * (-11 + x**2 + y)), 2 * (-11 + x**2 + y + 2 * y * (-7 + x + y**2))])

def Hessian_Himmer(x,y):
    h11 = 4 * (x**2 + y - 11) + 8 * x**2 + 2
    h12 = 4 * x + 4 * y
    h21 = 4 * x + 4 * y 
    h22 = 4 * (x + y**2 - 7) + 8 * y**2 + 2
    
    return np.array([[h11,h12],[h21,h22]]) 

def NewtonAandGradientDescentMultipleSaddlePoint():
    root_gd,iter_x_gd,iter_y_gd, iter_count_gd = GradientDescent(Grad_Himmer,0.5,-2,gamma = 0.001, epsilon=0.01, nMax = 1000)
    root_nr,iter_x_nr,iter_y_nr, iter_count_nr = Newton_Raphson_Optimize(Grad_Himmer,Hessian_Himmer,0.5,-2, nMax = 50)

    x = np.linspace(-5,5,100)
    y = np.linspace(-5,5,100)
    X, Y = np.meshgrid(x, y)
    Z = Himmer(X, Y)

    #Angles needed for quiver plot
    anglesx = iter_x_gd[1:] - iter_x_gd[:-1]
    anglesy = iter_y_gd[1:] - iter_y_gd[:-1]
    anglesx_nr = iter_x_nr[1:] - iter_x_nr[:-1]
    anglesy_nr = iter_y_nr[1:] - iter_y_nr[:-1]

    fig = plt.figure(figsize = (16,8))

    #Surface plot
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.plot_surface(X,Y,Z,rstride = 5, cstride = 5, cmap = 'jet', alpha = .4, edgecolor = 'none' )
    ax.plot(iter_x_gd,iter_y_gd, f_2(iter_x_gd,iter_y_gd),color = 'orange', marker = '*', alpha = .4, label = 'Gradient descent')
    ax.plot(iter_x_nr,iter_y_nr, f_2(iter_x_nr,iter_y_nr),color = 'darkblue', marker = 'o', alpha = .4, label = 'Newton')
    ax.legend()

    #Rotate the initialization to help viewing the graph
    ax.view_init(45, 60)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    #Contour plot
    ax = fig.add_subplot(1, 2, 2)
    ax.contour(X,Y,Z, 60, cmap = 'jet')

    #Plotting the iterations and intermediate values
    ax.scatter(iter_x_gd,iter_y_gd,color = 'orange', marker = '*', label = 'Gradient descent')
    ax.quiver(iter_x_gd[:-1], iter_y_gd[:-1], anglesx, anglesy, scale_units = 'xy', angles = 'xy', scale = 1, color = 'orange', alpha = .3)
    ax.scatter(iter_x_nr,iter_y_nr,color = 'darkblue', marker = 'o',  label = 'Newton')
    ax.quiver(iter_x_nr[:-1], iter_y_nr[:-1], anglesx_nr, anglesy_nr, scale_units = 'xy', angles = 'xy', scale = 1, color = 'darkblue', alpha = .3)
    ax.legend()

    ax.set_title('Comparing Newton and Gradient descent')

    plt.show()


def main():
    # NewtonMethod()
    # NewtonMethodQuadratic()
    # NewtonMethodRaphsonOptimize()
    NewtonAandGradientDescentOneSaddlePoint()
    # NewtonAandGradientDescentMultipleSaddlePoint()


if __name__ == '__main__':
    main()