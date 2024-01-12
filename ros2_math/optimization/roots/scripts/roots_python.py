import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits import mplot3d

# https://xavierbourretsicotte.github.io/Intro_optimization.html

#Defining the functions and its first derivative
def f(x):
    return x**3 - 2*x**2 - 11*x + 12

def dfdx(x):
    return 3*x**2 - 4*x - 11
    
#Initializing data
x = np.linspace(-4,6,100)
y = f(x)
y_dash = dfdx(x)

#Plotting the function and its derivative
fig = plt.figure()
plt.xlabel('x')
plt.axhline(0)
plt.plot(x,y, label = 'f(x)')
plt.plot(x,y_dash, '--r', label = 'df(x)\dx')
plt.legend()
plt.show()


def Bisection_Search(func,a,b,epsilon,nMax = 1000):
    #Initializating variables, iter_x, iter_x are used to plot results
    i = 0
    iter_x, iter_y, iter_count = np.empty(0),np.empty(0),np.empty(0)
    
    #Looping condition to ensure that loop is not infinite
    while i < nMax:
        i +=1
        c = .5 *(a + b)
        
        iter_x = np.append(iter_x,c)
        iter_y = np.append(iter_y,func(c))
        iter_count = np.append(iter_count ,i)
        
        if np.abs(func(c)) < epsilon:
            return c, iter_x, iter_y, iter_count
        elif np.sign(func(c)) == np.sign(func(a)): 
            a = c
        elif np.sign(func(c)) == np.sign(func(b)): 
            b = c


def Newton_Raphson(func, deriv, x, epsilon, nMax = 100):
    #Initializating variables, iter_x, iter_x are used to plot results
    i = 0
    iter_x, iter_y, iter_count = np.empty(0),np.empty(0),np.empty(0)
    
    error = x - (x - func(x)/ deriv(x))
    
    #Looping as long as error is greater than epsilon
    while np.abs(error) > epsilon and i < nMax:
        i +=1
        iter_x = np.append(iter_x,x)
        iter_y = np.append(iter_y,func(x))
        iter_count = np.append(iter_count ,i)

        error = x - (x - func(x)/ deriv(x))
        x = x - func(x)/ deriv(x)
    
    
    return x, iter_x, iter_y, iter_count

def Newton_Raphson(func, deriv, x, epsilon, nMax = 100):
    #Initializating variables, iter_x, iter_x are used to plot results
    i = 0
    iter_x, iter_y, iter_count = np.empty(0),np.empty(0),np.empty(0)
    
    error = x - (x - func(x)/ deriv(x))
    
    #Looping as long as error is greater than epsilon
    while np.abs(error) > epsilon and i < nMax:
        i +=1
        iter_x = np.append(iter_x,x)
        iter_y = np.append(iter_y,func(x))
        iter_count = np.append(iter_count ,i)

        error = x - (x - func(x)/ deriv(x))
        x = x - func(x)/ deriv(x)
    
    
    return x, iter_x, iter_y, iter_count

def Secant_method(func, x0,x1, epsilon, nMax = 100):
    #Initialization
    i = 0
    x2 = 0
    iter_x, iter_y, iter_count = np.empty(0),np.empty(0),np.empty(0)
    error = 100
    
    x2 = x1 - func(x1) * ((x1 - x0 ) / (func(x1) - func(x0))) 
    
    #Looping as long as error is greater than epsilon
    while np.abs(error) > epsilon and i < nMax:
        
        #Variables for plotting
        i +=1  
        iter_x = np.append(iter_x,x2)
        iter_y = np.append(iter_y,func(x2))
        iter_count = np.append(iter_count ,i)
        
        # Calculate new value
        x2 = x1 - func(x1) * ((x1 - x0 ) / (func(x1) - func(x0))) 
        error = x2 - x1
        x0 = x1
        x1 = x2
   
    return x2, iter_x, iter_y, iter_count

def RootsFindBisectionSSearch():
    root,iter_x,iter_y, iter_count = Bisection_Search(f,-5,-2,0.01)

    #Plotting the iterations and intermediate values
    fig, ax = plt.subplots() 
    ax.set_xlabel('x')
    ax.axhline(0)
    ax.plot(x,y)
    ax.scatter(x = iter_x,y = iter_y)
    ax.set_title('Bisection Method: {} iterations'.format(len(iter_count)))

    #Loop to add text annotations to the iteration points
    for i, txt in enumerate(iter_count):
        ax.annotate(txt, (iter_x[i], iter_y[i]))
        
    plt.show()


def RootsFindNewtonRaphson():
    root,iter_x,iter_y, iter_count = Newton_Raphson(f,dfdx,-2,0.01)

    #Plotting the iterations and intermediate values
    fig, ax = plt.subplots() 
    ax.set_xlabel('x')
    ax.axhline(0)
    ax.plot(x,y)
    ax.scatter(x = iter_x,y = iter_y)
    ax.set_title('Newton Method: {} iterations'.format(len(iter_count)))

    for i, txt in enumerate(iter_count):
        ax.annotate(txt, (iter_x[i], iter_y[i]))
        
    plt.show()


def RootsFindSecant():
    root,iter_x,iter_y, iter_count = Secant_method(f,-5,-2,0.01)

    #Plotting the iterations and intermediate values
    fig, ax = plt.subplots() 
    ax.set_xlabel('x')
    ax.axhline(0)
    ax.plot(x,y)
    ax.scatter(x = iter_x,y = iter_y)
    ax.set_title('Secant Method: {} iterations'.format(len(iter_count)))

    for i, txt in enumerate(iter_count):
        ax.annotate(txt, (iter_x[i], iter_y[i]))
        
    plt.show()


if __name__ == '__main__':
    # RootsFindBisectionSSearch()
    # RootsFindNewtonRaphson()
    RootsFindSecant()