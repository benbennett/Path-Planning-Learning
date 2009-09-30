import constants

class State:
    """
    Holds a discrete state along with utility methods.
    'A state is called consistent iff its g-value equals
    its rhs-value, otherwise it is either overconsistent (if
    g(s) > rhs(s)) or underconsistent (if g(s) < rhs(s))'
 
    """
    def __init__(self,tuple_point=(0,0),start=(0,0),goal=(0,0)):
        self.point = tuple_point
        self.start= start    
        self.goal = goal
        self.gofs= constants.INF      #Estimate cost to the goal 
        self.rhs_value=None #One-step lookahead cost rhs(s)
        self.min_succ=None  #Min successor of the state
        self.successors = None #Set of all successors of the state 
    def __hash__(self):
        return self.point.__hash__()
    def get_min_succesor(self):
        if(self.point == self.goal):
            return None 
        self.min_of_successors()
        return self.min_succ
    def min_of_successors(self):
        """
        Used to compute the rhs(s) (see pg 2 of pdf)
        rhs(s) = { 0                  if s=s goal 
                 { min s' exists Succ(s) ( c(s,s') + g(s'))

        """
        min = None
        for aS in self.successors:
            if min==None:
                min =self.csprime_gsprime(aS)
                self.min_succ = aS
            elif min> self.csprime_gsprime(aS):
                min = self.csprime_gsprime(aS)
                self.min_succ= aS 
        return min
    def remove_successor(self,s):
        if s in self.successors:
            self.successors.remove(s)
        self.min_of_successors()
    def rhs(self):
        """One-step lookahead cost rhs(s)"""
        return self.rhs_value 

    def set_rhs(self,rhs):
        self.rhs_value=rhs

    def csprime_gsprime(self,sprime):
        """Computes  c(s,s') + g(s') """
        return self.cost(sprime.point)+sprime.g()

    def cost(self,sprime):
        """
        Cost of moving in the grid.
        Future  have setable method 
        """
        return 1

    def g(self):
        """
        estimated cost of moving from state s(including previous
        states) to the goal
        """
        return self.gofs     
    
    def set_g(self,g):
        self.gofs= g

    def h(self):
        """ Euclidean distance  heuristic
            ****MUST*** satisfy the following
            h(s,GOAL) <= c(s,GOAL)
            where c is the least cost of moving from s to the goal. 
            Cannot 'overestimate' the cost 
        """ 
        x= abs((self.goal[0]-self.point[0]))
        y= abs((self.goal[1]-self.point[1]))
        x1= abs((self.start[0]-self.point[0]))
        y1= abs((self.start[1]-self.point[1]))
        if x>y:
            y=x
        if x1>y1:
            y1=x1
        return y1+y 

    def isGoal(self):
        return self.point == self.goal

    def __repr__(self):
        return str(self.point)

    def __eq__(self,o):
        return self.point == o.point 
