import constants

class state:
    def __init__(self,tuple_point,start,goal):
        self.point = tuple_point
        self.start= start
        self.goal = goal
        self.gofs= constants.INF 
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
