# https://stackoverflow.com/a/47379030
import math

def distance(a, b):
    if (a == b):
        return 0
    elif (a < 0) and (b < 0) or (a > 0) and (b > 0):
        if (a < b):
            return (abs(abs(a) - abs(b)))
        else:
            return -(abs(abs(a) - abs(b)))
    else:
        return math.copysign((abs(a) + abs(b)),b)
    
'''
print(distance(3,-5))  # -8
print(distance(-3,5))  #  8
print(distance(-3,-5)) #  2
print(distance(5,3))   # -2
print(distance(5,5))   #  0
print(distance(-5,3))  #  8
print(distance(5,-3))  # -8
'''