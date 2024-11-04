import rospy

class ROSBackend(BaseBackend):
    def __init__(self) -> None:
        super().__init__()
    
    def initiate_rgb_sensor(self) -> BaseSensorBackend: # TODO: what about other sensors?
        class ROSSensorBackend(BaseSensorBackend):
            def _my_image_callback(self, msg):  # TODO: is it under self? under what context does it live?
                self.buffer.append(msg) # TODO: flush the buffer as well
            def __init__(self) -> None:
                super().__init__()
                self.buffer = []
                sub = rospy.Subscriber("/robot1/D455_1/color/image_raw", Image, self._my_image_callback)                
            def get(self):
                return self.buffer[-1] # TODO: also convert to agreed format (numpy?)
        sensor_backend = ROSSensorBackend()
        return sensor_backend
