import numpy
import math
from priodict import priorityDictionary
import copy
#x,y  grid 10x10
sX =  numpy.zeros((4,4),dtype=numpy.int)
#x,y action space
Ux =  numpy.array([[0,1],[1,0],[0,-1],[-1,0],[1,1],[-1,1],[1,-1],[-1,-1]])
#can move up down left right
INF=1000000.0 
def state_trans_fuction(x,action_space,sX):
    xy=numpy.array([
                    [x[0],
                     x[1] ]
                    ])
    asp= xy+action_space
    mr=[]
    for aC in asp:
        valid=True
        for i in range(0,2):
            if not (aC[i]>=0 and aC[i]<sX.shape[i]):
                valid=False
        if valid==True:
            mr.append((aC[0],aC[1]))
    return mr
        


class state:
    def __init__(self,tuple_point,start,goal):
        self.point = tuple_point
        self.start= start
        self.goal = goal
        self.gofs= INF 
        self.rhs_value=None
        self.min_succ=None
        self.succesors = None 
    def __hash__(self):
        return self.point.__hash__()
    def min_of_successors(self):
        min = None
        for aS in self.succesors:
            if min==None :
                min =self.csprime_gsprime(aS)
            elif min> self.csprime_gsprime(aS):
                min = self.csprime_gsprime(aS)
        self.min_succ= min
        return min
    def rhs(self):# one-step lookahead cost rhs(s)
            return self.rhs_value 
    def set_rhs(self,rhs):
        self.rhs_value=rhs
    def csprime_gsprime(self,sprime):
        return self.cost(sprime.point)+sprime.g()
    def cost(self,sprime):
        return 1
    def g(self):#estimated cost of moving from state s(including previous
                #states) to the goal
        return self.gofs     
        #abs((self.goal[0]-self.point[0]))+abs((self.goal[1]-self.point[1]))
    def set_g(self,g):
        self.gofs= g
    def h(self):
        return abs((self.start[0]-self.point[0]))+abs((self.start[1]-self.point[1]))
    def isGoal(self):
        return self.point == self.goal
class Key:
    def __init__(self,an_array=[0,0]):
        self.k1= an_array[0]
        self.k2= an_array[1]
    def __lt__(self,o):
        if self.k1<o.k1:
            return True
        if self.k1==o.k1 and  self.k2<o.k2:
            return True
        return False
    def __eq__(self,o):
        return self.k1==o.k1 and self.k2==o.k2
    def __le__(self,o):
        if self < o or self == o:
            return True
        return False
    def __gt__(self,o):
        return not self<o
    def __ge__(self,o):
        if self>o or self==o:
            return True
        return False

class AnytimeDstar:
    def __init__(self,start,goal,forbidden):
        self.OPEN = priorityDictionary()
        self.INCONS= set() 
        self.KEYS = priorityDictionary()
        self.CLOSED = set()
        self.start=start
        self.goal=goal
        self.s_start = state(start,start,goal)
        self.s_start.set_rhs(INF)
        self.s_start.set_g(INF)
        self.s_goal = state(goal,start,goal)
        self.s_goal.set_g(INF)
        self.s_goal.set_rhs(0)
        self.eps = 2.5 # TODO
        self.PREC = {}
        self.G = {} 
        self.forbidden = forbidden#TODO
        self.OPEN[self.s_goal] = self.keys(self.s_goal)

        
    def keys(self,s):  #state 
        if s.g()>s.rhs():
            return Key([ s.rhs() + self.eps*s.h(),s.rhs()])
        else:
            return Key([ s.g() + s.h(),s.g()])

    def UpdateState(self,s):
        if not s.isGoal():
            s.set_rhs(s.min_of_successors())
        if s in self.OPEN :
            self.OPEN.remove(s)
        if s.g()!=s.rhs():
            if not s in self.CLOSED:
                self.OPEN[s] = self.keys(s)
            else:
                self.INCONS.add(s) 
    def __build_state__(self,s):
        if not self.G.has_key(s):
            self.G[s.point]= s
        if s.succesors!=None:
            return
        values= state_trans_fuction(s.point,Ux,sX)
        hold_v = copy.deepcopy(values)
        for x in values:
            if x in forbidden:
                hold_v.remove(x)
        s.succesors = set()
        for x in values:
            newstate =None
            if self.G.has_key(x):
                newstate = self.G[x]
            else:
                newstate = state(x,self.start,self.goal)
                self.G[x] = newstate
            s.succesors.add(newstate)
          #if x in hold_v and x in forbidden:
           #   hold_v.remove(x)
              
 #           for w in G[v]:
 #             if w in forbidden:
 #                 pass
 #             vwLength = D[v] + 1 + abs((end[0]-w[0]))+abs((end[1]-w[1]))
 #             if w not in D or vwLength < D[w]:
 #                 Q[w] = vwLength
 #                 P[w] = v

    def ComputeorImprovePath(self):
        while len(self.OPEN)>0 and ( self.keys(self.OPEN.smallest())< self.keys(self.s_start) \
                or self.s_start.g()!= self.s_start.rhs()):
            hold_state = self.OPEN.smallest()
            self.OPEN.remove(hold_state)
            if hold_state.g() > hold_state.rhs():
                hold_state.set_g(hold_state.rhs())
                self.CLOSED.add(hold_state)
                self.__build_state__(hold_state)
                self.UpdateState(hold_state)
                for aState in hold_state.succesors:
                    self.__build_state__(aState)
                    self.UpdateState(aState)
            else:
                hold_state.set_g(INF)
                hold_state.__build_state__(hold_state)
                for aState in hold_state.succesors:
                    self.__build_state__(aState)
                    self.UpdateState(aState)

start=(0,0)
goal=(3,3)
forbidden= set()

aDstart =  AnytimeDstar(start,goal,forbidden)
aDstart.ComputeorImprovePath() 
print aDstart.CLOSED



