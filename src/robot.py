"""
Simple robot implementations. 
Distances in meters.
"""
class SimpleRobot:
    """"""

    def __init__(self,pos=[0,0]):
        self.pos =pos 
        self.velocity = 0.0
        self.delay = 1.0
        pass
    def set_max_velocity(self,v_in): 
        self.velocity = v_in
    def go(self):
        pass    
    def stop(self):
        pass
    def get_position(self):
        pass
    def set_next_pos(self):
        pass
    def run():
        pass
    def get_status(self):
        pass


