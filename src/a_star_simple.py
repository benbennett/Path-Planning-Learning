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
        

unvisited=0
dead=-1 #need to set dead state

def alive(v):
    if v>=0:
        return True
    else:
        return False


def astar (start,end,forbidden):
  D = {}    # dictionary of final distances
  P = {}    # dictionary of predecessors
  Q = priorityDictionary()   # est.dist. of non-final vert.
  Q[start] = 0
  G={}
  for v in Q:
      D[v] = Q[v]
      if v == end: 
          break
      if not G.has_key(v):
          values= state_trans_fuction(v,Ux,sX)
          hold_v = copy.deepcopy(values)
          for x in values:
              if G.has_key(x) :  
                  #hold_v.remove(x)
                  continue
              if x in forbidden:
                   hold_v.remove(x)
                   print x,"forbidden"
              #if x in hold_v and x in forbidden:
               #   hold_v.remove(x)
          print hold_v
          G[v]=hold_v
          

          for w in G[v]:
              if w in forbidden:
                  pass
              vwLength = D[v] + 1 + abs((end[0]-w[0]))+abs((end[1]-w[1]))
              if w not in D or vwLength < D[w]:
                  Q[w] = vwLength
                  P[w] = v
  
  return (D,P)




#for i in range(1,10,2):
#    for j in range(0,8):
#        forbidden.add((i,j))
#print forbidde

#forbidden = set()
##for i in range(8,2,-2):
##    for j in range(0,8):
##        forbidden.add((i,j))
#if (1,1) in forbidden:
#    print "forbidden"
#D,P = astar(start,end,forbidden)
#Path = []
#while 1:
#    print end
#    Path.append(end)
#    if end == start: 
#        break
#    end = P[end]
#Path.reverse()
#print Path
#
