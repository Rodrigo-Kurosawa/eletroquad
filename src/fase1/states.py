from FSM import State
import colorful as cf
'''
Phase 1 FSM 

                                       ((  Land  ))
                                            ^
                                            |
                                         no_bars
                                            |
((  Take Off  )) --TakeOff_complete--> ((  Yaw  )) --centralized--> (( Foward ))
                                            ^                             |
                                            |                     distance_achieved
                                            |                             |
                                            |                             V
                                            L  --rotate_complete--  ((  Rotate  ))

'''

#teste comentario
class TakeOff(State):
    def __init__(self,name="") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('takeOff_complete'):
            return Yaw

class Yaw(State):
    def __init__(self,name="") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('centralized'):
            return Foward
        elif self.tail('no_bars'):
            return Land
            
        
class Foward(State):
    def __init__(self,name="") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('distance_achieved'):
            return Rotate
        
class Rotate(State):
    def __init__(self,name="") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('rotate_complete'):
            return Yaw
        
class Land(State):
    def __init__(self,name="") -> None:
        super().__init__(name)
        
    def event(self):
        if self.tail('finished'):
            return Finish
        
class Finish(State):
    def __init__(self,name="") -> None:
        super().__init__(name)
        
    def event(self):
        None