import weakref
import sys
import math
#carla_path = '/home/itayb/simulator/carla_0.98/PythonAPI/carla/dist/carla-0.9.8-py2.7-linux-x86_64.egg' # Use for running via Python2, but youll have bugs in rescaling images with cv2.
carla_path = '/home/itayb/simulator/carla_0.98/PythonAPI/carla/dist/carla-0.9.8-py3.6-linux-x86_64.egg'
sys.path.append(carla_path)
import carla

# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))
        self.history.append(event.frame)
        if len(self.history) > 4000:
            self.history.pop(0)
        self.history = [frame for frame in self.history if event.frame - frame < 100]
