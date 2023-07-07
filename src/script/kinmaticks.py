#! /usr/bin/env python3

import math
import numpy

x = 0.15-0.019
y = 0.25+0.019
z = 0.33

a1 = 142 / 1000
a2 = 219.5 / 1000
a3 = 155 / 1000

r1 = math.sqrt((x**2) + (y**2))
r2 = z - a1
r3 = math.sqrt((r2**2 ) + (r1**2))

if (x > -1 ):

    theta1 = math.atan(y/x) #in rad
else:
    theta1 = math.atan(y/-x)
    theta1 = math.pi - theta1


ome1 = math.atan(r2/r1)
ome2 = math.acos(((a2**2)+(r3**2)-(a3**2))/(2*a2*r3))


theta2 = ome1 + ome2
print(((a2**2)+(a3**2)-(r3**2))/(2*a2*a3))
ome3 = math.acos(((a2**2)+(a3**2)-(r3**2))/(2*a2*a3))

theta3 = math.pi - ome3

print("theta1 = ",theta1)
print("theta2 = ",theta2)
print("theta3 = ",theta3)

