import RobotFuncs
x1 = 0
x2 = 0
x3 = 0
x4 = 0
y1 = -0.15
y2 = -0.15
y3 = -0.15
y4 = -0.15
#x3,x4,y3,y4 = -0.113, -0.113, -0.113, -0.113
leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2  = RobotFuncs.actualReference(x1,y1,x2,y2,x3,y3,x4,y4)
print  leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2
