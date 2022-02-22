from math import *

class Line:
    def __init__(self, A=0, B=0, C=0):
        # Ay + Bx + C = 0
        self.A = A
        self.B = B
        self.C = C
    def drawLineWithTwoPoint(self, p1, p2):
        x1 = p1['x']
        y1 = p1['y']
        x2 = p2['x']
        y2 = p2['y']
        self.A = x1 - x2
        self.B = y2 - y1
        self.C = y1*x2 - y2*x1
    def drawLineWithPointAndAngle(self, p, a):
        pass
    def getIntersectionWithCircle(self, circle):
        pass
    def getIntersectionWithLine(self, line):
        if(self.A*line.B == line.A*self.B ):
            # print('montabegh')
            return
        elif(self.A == 0):
            y = (line.B*self.C - self.B*line.C)/(self.B*line.A - line.B*self.A)
            x = self.getX(y)
            return {'x': x, 'y': y}
        else:
            x = (line.A*self.C - self.A*line.C)/(self.A*line.B - line.A*self.B)
            y = self.getY(x)
            return {'x': x, 'y': y}
    def getY(self, x):
        return(-(self.B*x + self.C)/self.A)
    def getX(self, y):
        return(-(self.A*y + self.C)/self.B)
    def print(self):
        print(str(self.A) + 'y + ' + str(self.B) + 'x + ' + str(self.C) + ' = 0')
    
def getDisgtance(p1, p2):
    return sqrt((p1['x']-p2['x'])**2 + (p1['y']-p2['y'])**2)
def dotProduct(l1, l2):
    temp = {'x': 0, 'y': 0}
    a = atan2(l1['y'] - l2['y'], l1['x'] - l2['x'])
    return getDisgtance(l1, temp)*getDisgtance(l2, temp)*cos(a)
def getAngle(l1, l2={'x': 0, 'y': 0}):
    return atan2(l1['y'] - l2['y'], l1['x'] - l2['x'])
class Circle:
    def __init__(self, center, r):
        self.center = center
        self.r = r
    def getIntersectionWithLine(self, line):
        y1 = self.center['y']
        x1 = self.center['x']
        r = self.r
        a = line.A
        b = line.B
        c = line.C
        Px1, Px2, Py1, Py2 = 0,0,0,0
        delta = a**2 * r**2 - a**2 * y1**2 - 2 * a * b * x1 * y1 - 2 * a * c * y1 + b**2 * r**2 - b**2 * x1**2 - 2 * b * c * x1 - c**2
        if(delta < 0):
            return []
        if(a!=0):
            Px1 = (-sqrt(delta)/a - (b * c)/a**2 - (b * y1)/a + x1)/(b**2/a**2 + 1)
            Px2 = ( sqrt(delta)/a - (b * c)/a**2 - (b * y1)/a + x1)/(b**2/a**2 + 1)
            Py1 = line.getY(Px1)
            Py2 = line.getY(Px2)
        else:
            Py1 = (-sqrt(delta)/b - (a * c)/b**2 - (a * x1)/b + y1)/(a**2/b**2 + 1)
            Py2 = ( sqrt(delta)/b - (a * c)/b**2 - (a * x1)/b + y1)/(a**2/b**2 + 1)
            Px1 = line.getX(Py1)
            Px2 = line.getX(Py2)
        if(Px1 == Px2 and Py1 == Py2):
            return [
                {
                'x': Px1,
                'y': Py1
                }
            ]
        else: 
            return [
                {
                'x': Px1,
                'y': Py1
                },
                {
                'x': Px2,
                'y': Py2
                },
            ]
    
# circle intersection with line:
# https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm