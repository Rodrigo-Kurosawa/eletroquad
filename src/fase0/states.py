from FSM import State
import colorful as cf
'''



'''
#teste comentario
class TakeOff(State):
    def __init__(self,name="") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('TakeOff_complete'):
            return Foward


class Foward(State):
    def __init__(self,name="") -> None:
        super().__init__(name)

    def event(self):
        if self.tail('Foward_complete'):
            return Land
        
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