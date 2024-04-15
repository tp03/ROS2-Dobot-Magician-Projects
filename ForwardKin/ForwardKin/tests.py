import numpy as np
from numpy import cos, sin
from sympy import symbols, solve
import sympy as sp

theta1 = symbols('theta1')
theta2 = symbols('theta2')
theta3 = symbols('theta3')
theta4 = symbols('theta4')
theta5 = symbols('theta5')


m1 = sp.Matrix([[sp.cos(theta1), -sp.sin(theta1), 0, 0],
                                   [sp.sin(theta1), sp.cos(theta1), 0, 0],
                                   [0, 0, 1, 0.05],
                                   [0, 0, 0, 1]                                   
                                   ])
        
m2 = sp.Matrix([
                    [sp.cos(theta2), 0, sp.sin(theta2), 0],
                    [0, 1, 0, 0],
                    [-sp.sin(theta2), 0, sp.cos(theta2), 0.088],
                    [0, 0, 0, 1]
        ])

m3 = sp.Matrix([
                    [sp.cos(theta3), 0, sp.sin(theta3), 0],
                    [0, 1, 0, 0],
                    [-sp.sin(theta3), 0, sp.cos(theta3), 0.135],
                    [0, 0, 0, 1]
        ])

m4 = sp.Matrix([
                    [sp.cos(theta4), 0, sp.sin(theta4), 0.147],
                    [0, 1, 0, 0],
                    [-sp.sin(theta4), 0, sp.cos(theta4), 0],
                    [0, 0, 0, 1]
        ])

m5 = sp.Matrix([[sp.cos(theta5), -sp.sin(theta5), 0, 0.03],
                                   [sp.sin(theta5), sp.cos(theta5), 0, 0],
                                   [0, 0, 1, -0.024-0.04],
                                   [0, 0, 0, 1]                                   
                                   ])

m6 = sp.Matrix([
                    [1, 0, 0, 0],
                    [0, cos(np.pi), -sin(np.pi), 0],
                    [0, sin(np.pi), cos(np.pi), 0],
                    [0, 0, 0, 1]
        ])

result = m1*m2*m3*m4*m5*m6
print(result)