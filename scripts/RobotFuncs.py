import math
def trajectory(ptd,H,L,Td,Ta,t) :
	p1 =[ptd[0] + L/2, ptd[1]]
	p2 =[ptd[0] - L/2, ptd[1]]
	p3 =[ptd[0], ptd[1] + H]
	x1 = p1[0]
	x2 = p2[0]
	x3 = p3[0]
	y1 = p1[1]
	y2 = p2[1]
	y3 = p3[1]	
	a = -(x1*(y2 - y3) - x2*(y1 - y3) + x3*(y1 - y2))/((x1 - x2)*(x1 - x3)*(x2 - x3))
	b = (x1*x1*y2 - x2*x2*y1 - x1*x1*y3 + x3*x3*y1 + x2*x2*y3 - x3*x3*y2)/((x1 - x2)*(x1 - x3)*(x2 - x3))
	c = (x1*y2 - x2*y1)/(x1 - x2) + (x1*x2*(y1 - y3))/((x1 - x2)*(x1 - x3)) - (x1*x2*(y2 - y3))/((x1 - x2)*(x2 - x3))
	T = Td + Ta
	tt = t%T
	if tt <= Td :
		x = p1[0]-tt*(p1[0]-p2[0])/Td;
		y = p1[1];
	else:
		x = p2[0] + (tt-Td)*(p1[0]-p2[0])/Ta;
		y = a*x**2+b*x+c;

	return x,y

def reverse_trajectory(ptd,H,L,Td,Ta,t) :
	p1 =[ptd[0] + L/2, ptd[1]]
	p2 =[ptd[0] - L/2, ptd[1]]
	p3 =[ptd[0], ptd[1] + H]
	x1 = p1[0]
	x2 = p2[0]
	x3 = p3[0]
	y1 = p1[1]
	y2 = p2[1]
	y3 = p3[1]	
	a = -(x1*(y2 - y3) - x2*(y1 - y3) + x3*(y1 - y2))/((x1 - x2)*(x1 - x3)*(x2 - x3))
	b = (x1*x1*y2 - x2*x2*y1 - x1*x1*y3 + x3*x3*y1 + x2*x2*y3 - x3*x3*y2)/((x1 - x2)*(x1 - x3)*(x2 - x3))
	c = (x1*y2 - x2*y1)/(x1 - x2) + (x1*x2*(y1 - y3))/((x1 - x2)*(x1 - x3)) - (x1*x2*(y2 - y3))/((x1 - x2)*(x2 - x3))
	T = Td + Ta
	tt = t%T
	if tt <= Td :
		x = p2[0] + tt*(p1[0]-p2[0])/Td;
		y = p1[1];
	else:
		x = p1[0] - (tt-Td)*(p1[0]-p2[0])/Ta;
		y = a*x**2+b*x+c;

	return x,y

def KneeRotationAng(th2) :
	l1 = 0.093
	l2	= 0.093
	l0 = 0.16
	r = 0.028
	lp = math.sqrt(l1**2+l2**2-2*l1*l2*math.cos(math.pi - math.fabs(th2)))
	return (l0-lp)/r

def  inverseCoordFront(x,y) :
	l1 = 0.093
	l2 = 0.093
	r_2= x**2 + y**2
	phi = math.atan2(y,x)
	temp = math.acos((l1**2+l2**2-r_2)/(2*l1*l2));
	th2 = math.pi - temp;
	temp = (math.pi - temp)/2;
	th1 = phi - temp;
	return th1,KneeRotationAng(th2)
	
def  inverseCoordRear(x,y) :
	l1 = 0.093
	l2 = 0.093
	r_2= x**2 + y**2
	phi = math.atan2(y,x)
	temp = math.acos((l1**2+l2**2-r_2)/(2*l1*l2));
	th2 = -math.pi + temp;
	temp = (math.pi - temp)/2;
	th1 = phi + temp;
	return th1,KneeRotationAng(th2)



def rad2ang(rad) :
	return rad*180.0/math.pi

def actualReference(x1,y1,x2,y2,x3,y3,x4,y4):
	leg1th1,leg1th2 = inverseCoordFront(x1,y1)
	leg2th1,leg2th2 = inverseCoordFront(x2,y2)
	leg3th1,leg3th2 = inverseCoordRear(x3,y3)
	leg4th1,leg4th2 = inverseCoordRear(x4,y4)
	leg1th1 = - rad2ang(leg1th1)
	leg1th2 = rad2ang(leg1th2)
	leg2th1 = rad2ang(leg2th1)
	leg2th2 = - rad2ang(leg2th2)
	leg3th1 = - rad2ang(leg3th1)
	leg3th2 = - rad2ang(leg3th2)
	leg4th1 = rad2ang(leg4th1)
	leg4th2 = rad2ang(leg4th2)
	return leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2
def generateCommand(t):
	H = 0.05
	L =0.06
	Td = 0.8
	Ta =  0.5
	ptd = [-0.02,-0.15]
	ptd1 = [0.0,-0.15]
	x1,y1 = trajectory(ptd,H+0.01,L,Td,Ta,t + Td)
	x2,y2 = trajectory(ptd,H+0.01,L,Td,Ta,t + (Td-Ta)/2.0)
	x3,y3 = trajectory(ptd1,H,L,Td,Ta,t + (Td-Ta)/2.0)
	x4,y4 = trajectory(ptd1,H,L,Td,Ta,t + Td)
	#x1,y1 = trajectory([0,-0.16],H,L,Td,Ta,t+(Td+Ta)*2/4)
	#x2,y2 = trajectory([0,-0.16],H,L,Td,Ta,t )
	#x3,y3 = trajectory(ptd,H,L,Td,Ta,t )
	#x4,y4 = trajectory(ptd,H,L,Td,Ta,t + (Td+Ta)*2/4)
	leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = actualReference(x1,y1,x2,y2,x3,y3,x4,y4)
	return leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2

def generateRemoteCommand(ptd,ptd1,LeftH,RightH,LeftL,RightL,Ta,Td,t):
	x1,y1 = trajectory(ptd,LeftH,LeftL,Td,Ta,t + Td)
	x2,y2 = trajectory(ptd,RightH,RightL,Td,Ta,t + (Td-Ta)/2.0)
	x3,y3 = trajectory(ptd1,LeftH,LeftL,Td,Ta,t + (Td-Ta)/2.0)
	x4,y4 = trajectory(ptd1,RightH,RightL,Td,Ta,t + Td)
	leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = actualReference(x1,y1,x2,y2,x3,y3,x4,y4)
	return leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2

def generateRemoteTurnCommand(ptd,ptd1,LeftH,RightH,LeftL,RightL,Ta,Td,t):
	x1,y1 = trajectory(ptd,LeftH,LeftL,Td,Ta,t + Td)
	x2,y2 = trajectory(ptd,RightH,RightL,Td,Ta,t + (Td-Ta)/2.0)
	x3,y3 = trajectory(ptd,LeftH,LeftL,Td,Ta,t + (Td-Ta)/2.0)
	x4,y4 = trajectory(ptd,RightH,RightL,Td,Ta,t + Td)
	leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = actualReference(x1,y1,x2,y2,x3,y3,x4,y4)
	return leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2

def upDownTrajectory(ptd,H,T,t):
	x = ptd[0]
	tt = t%T
	if tt <= T/2.0:
		y = ptd[1] + (H*2.0/T)*tt
	else:
		y = ptd[1] +H - (H*2.0/T)*(tt-T/2.0)
	return x,y
def generateUpDownCommand(t):
	H = 0.06
	T = 1
	ptd = [0,-0.16]
	x1,y1 = upDownTrajectory(ptd,H,T,t+T/2.0)
	x2,y2 = upDownTrajectory(ptd,H,T,t)
	x3,y3 = upDownTrajectory(ptd,H,T,t)
	x4,y4 = upDownTrajectory(ptd,H,T,t+T/2.0)
	leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2 = actualReference(x1,y1,x2,y2,x3,y3,x4,y4)
	return leg1th1,leg1th2,leg2th1,leg2th2 ,leg3th1,leg3th2,leg4th1,leg4th2	
	
	
