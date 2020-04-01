import weakref
import sys
import math
#carla_path = '/home/itayb/simulator/carla_0.98/PythonAPI/carla/dist/carla-0.9.8-py2.7-linux-x86_64.egg' # Use for running via Python2, but youll have bugs in rescaling images with cv2.
carla_path = '/home/itayb/simulator/carla_0.98/PythonAPI/carla/dist/carla-0.9.8-py3.6-linux-x86_64.egg'
sys.path.append(carla_path)
import carla

# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude