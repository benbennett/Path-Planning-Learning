"""
Simple implementation of
"Anytime Dynamic A*: An Anytime, Replanning Algorithm" by
Maxim Likhachev , Dave Ferguson , Geoff Gordon , Anthony Stentz , and Sebastian Thrun

Warning this may be incorrect, mainly put together to learn from there paper.
See paper at: 
http://www.ri.cmu.edu/pub_files/pub4/likhachev_maxim_2005_1/likhachev_maxim_2005_1.pdf


"""

import numpy
import math
from priodict import priorityDictionary
import copy
#local modules
from state import State 
import constants

        
class StateTranSpace:
    def __init__(self,state_space,action_space):
        self.sX = state_space 
        self.Ux= action_space 
    def state_trans_fuction(self,x):
        xy=numpy.array([
                        [x[0],
                         x[1] ]
                        ])
        asp= xy+self.Ux
        mr=[]
        for aC in asp:
            valid=True
            for i in range(0,2):
                if not (aC[i]>=0 and aC[i]<self.sX.shape[i]):
                    valid=False
            if valid==True:
                mr.append((aC[0],aC[1]))
        return mr
class Key:
    """
    Used for the priority queue .
    OVERVIEW of class 
   key(s) = [k1 (s), k2 (s)]
       = [min(g(s), rhs(s)) + h(sstart , s),
          min(g(s), rhs(s))].
 
    key(s) < key(s ),
    iff k1 (s) < k1 (s ) OR
    both k1 (s) = k1 (s )  and k2 (s) < k2 (s ). 
    
    The algorithm will go states with increasing priority,
    updating there g-values and rhs-values of their PREDECESSORS,
    'Until there is no state in the queue with a key value less than 
    that of the start state.Thus,
    during its generation of an initial solution path, it performs
    in exactly the same manner as a backwards A* search'
    
    So the rhs values change for predecessors 
    """
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
    def __repr__(self):
        return " ".join([str(self.k1)," ,",str(self.k2)])

class AnytimeDstar:
    """
    Main portion of algorithm. 
    """
    def __init__(self,start,goal,state_trans,forbidden=set()):
        self.OPEN = priorityDictionary()
        self.INCONS= set() 
        self.CLOSED = set()
        self.s_start = State(start,start,goal)
        self.s_start.set_rhs(constants.INF)
        self.s_start.set_g(constants.INF)
        self.s_goal = State(goal,start,goal)
        self.s_goal.set_g(constants.INF)
        self.s_goal.set_rhs(0)
        self.eps = 2.5 # TODO make it change during iterations. 
        self.PREC = {}
        self.G = {} 
        self.path_= {}
        self.G[self.s_start] = self.s_start
        self.forbidden = forbidden#TODO add "obstacles" 
        self.OPEN[self.s_goal] = self.keys(self.s_goal)
        self.state_trans= state_trans
    def __build_state__(self,s):
        """ Utility method to build up a unvisited state"""
        if not self.G.has_key(s):
            self.G[s]= s
        if s.successors!=None:
            return
        values= self.state_trans.state_trans_fuction(s.point)
        hold_v = copy.deepcopy(values)
        for x in values:
            if x in self.forbidden:
                hold_v.remove(x)
        s.successors = set()
        for x in hold_v:
            newstate = State(x,self.s_start.point,self.s_goal.point)
            if self.G.has_key(newstate):
                newstate = self.G[newstate]
            self.G[newstate]= newstate
            s.successors.add(newstate)
    def get_start(self):
        return self.s_start
    def keys(self,s):  #state 
        if s.g()>s.rhs():
            return Key([ s.rhs() + self.eps*s.h(),s.rhs()])
        else:
            return Key([ s.g() + s.h(),s.g()])

    def UpdateAllPriorities(self):
        states= [s for s in self.OPEN]
        for s in states:
            self.OPEN[s] = self.keys(s)
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

    def ComputeorImprovePath(self):
        states = 0
        while len(self.OPEN)>0 and ( self.keys(self.OPEN.smallest())< self.keys(self.s_start) \
             or self.s_start.g()!= self.s_start.rhs()):

            #print self.keys(self.OPEN.smallest()) , "----",self.keys(self.s_start)
            hold_state = self.OPEN.smallest()
            #TODO debug methods 
            states+=1 
            self.OPEN.remove(hold_state)
            self.__build_state__(hold_state)
            if hold_state.g() > hold_state.rhs():
                hold_state.set_g(hold_state.rhs())
                self.CLOSED.add(hold_state)
                for aState in hold_state.successors:
                    self.__build_state__(aState)
                    self.UpdateState(aState)
            else:
                hold_state.set_g(constants.INF)
                self.UpdateState(hold_state)
                for aState in hold_state.successors:
                    self.__build_state__(aState)
                    self.UpdateState(aState)
        print states 
    def getPath(self):
        curP = aDstart.get_start()
        path = []
        self.path_=set()
        self.path_.add(curP)
        while(curP!=None):
            path.append(curP)
            curP = curP.get_min_succesor()
            if curP in self.path_:
                err = str(curP)
                print path
                raise Exception('Cycle detected'+err)
            
            self.path_.add(curP)
        return path
    def addForbidden(self,point):
        if point in self.forbidden:
            return 
        self.forbidden.add(point)
        state = State(point)
        if self.G.has_key(state):
            state = self.G[state]
        else:
            return
        toUpdate = []
        for aS in state.successors:
            if aS.successors==None:
                continue
            aS.remove_successor(state)
            toUpdate.append(aS)
        for aS in toUpdate:
            self.UpdateState(aS)
            aS.min_of_successors()
    def moveAllFromIncsToOpen(self):
        for aS in self.INCONS:
            self.OPEN[aS] = self.keys(aS)
        self.INCONS = set()
if __name__== "__main__": 
    start=(5,0)
    goal=(0,6)
    forbidden= set()

    #x,y  grid 10x10
    sX =  numpy.zeros((140,140),dtype=numpy.int)
    #x,y action space
    Ux =  numpy.array([[0,1],[1,0],[0,-1],[-1,0],[1,1],[-1,1],[1,-1],[-1,-1]])
    state_trans = StateTranSpace(state_space=sX,action_space=Ux)
    #can move up down left right
    aDstart =  AnytimeDstar(start,goal,state_trans)

    #change to have example from paper.

    '''When changing the edge cost or graph one must
       Move all states to the open state.
        update all the priorities
        set the closed state to the empty set. 
        then call ComputeorImprovePath '''

    forbids = [
            (0,0),(1,0),(2,0),(3,0),(4,0),(4,1),(4,2),(2,2),(1,2),(1,3),(1,4),(1,5),
            (2,5),(3,5),(4,5) ] 
    for x in forbids:
        aDstart.addForbidden(x)
    aDstart.ComputeorImprovePath() 
    
    print aDstart.getPath() 
    start=(1,1)
    goal=(100,100)
    forbidden= set()

    #x,y  grid 10x10
    sX =  numpy.zeros((140,140),dtype=numpy.int)
    #x,y action space
    Ux =  numpy.array([[0,1],[1,0],[0,-1],[-1,0],[1,1],[-1,1],[1,-1],[-1,-1]])
    state_trans = StateTranSpace(state_space=sX,action_space=Ux)
    #can move up down left right
    aDstart =  AnytimeDstar(start,goal,state_trans)
    aDstart.ComputeorImprovePath() 
    print aDstart.getPath()
    aDstart.addForbidden((2,2))

    aDstart.moveAllFromIncsToOpen()
    aDstart.UpdateAllPriorities()
    aDstart.CLOSED = set()
    aDstart.ComputeorImprovePath() 
    print aDstart.getPath()
    '''When changing the edge cost or graph one must
       Move all states to the open state.
        update all the priorities
        set the closed state to the empty set. 
        then call ComputeorImprovePath '''
    aDstart.addForbidden((3,3))
    aDstart.addForbidden((5,3))
    aDstart.addForbidden((5,5))
    aDstart.addForbidden((5,4))
    aDstart.addForbidden((5,5))
    aDstart.addForbidden((5,5))
    aDstart.addForbidden((5,6))
    aDstart.addForbidden((5,7))
    aDstart.addForbidden((5,4))
    aDstart.addForbidden((5,5))
    start=(1,1)
    goal=(250,250)
    forbidden= set()

    #x,y  grid 10x10
    sX =  numpy.zeros((500,500),dtype=numpy.int)
    #x,y action space
    Ux =  numpy.array([[0,1],[1,0],[0,-1],[-1,0],[1,1],[-1,1],[1,-1],[-1,-1]])
    state_trans = StateTranSpace(state_space=sX,action_space=Ux)
    #can move up down left right
    aDstart =  AnytimeDstar(start,goal,state_trans)
    aDstart.ComputeorImprovePath() 


    for i in range(5,200):
        aDstart.addForbidden((i,i))
        aDstart.moveAllFromIncsToOpen()
        aDstart.UpdateAllPriorities()
        aDstart.CLOSED = set()
        aDstart.ComputeorImprovePath() 
        print aDstart.getPath()
