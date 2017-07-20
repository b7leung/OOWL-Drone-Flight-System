from numpy import *


a = array([179,176,180,170,171,0, 2 ,3 ,4, 5, 9,11])

print(a)
x= (a > 170)

y= (a < 10)
print(x|y)
b = y*a
c=sum(b)
x1=sum(x)
y1=sum(y)
if( sum(x) > sum(y)):
    print(x)
    print("large angle",x1)
else: 
    print(y)
    print("small angle",y1)
    print(b)
    print(c)
    
