import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from .dmx_light_interface.astora_if import ASTORA_Lights
from capture_types.srv import LightParams
from capture_types.action import LightsFade

class ASTORA_node(Node):
    '''
    ROS wrapper for the ASTORA_lights class. Provides services to get and set the light parameters, and an action server to fade the lights.
    '''

    def __init__(self):
        super().__init__('lights')
        # Init parameters
        # Each one of our ASTORA Soft Panels has 2 knobs, so we need 2 channels per panel
        # 2 lights --> 4 channels
        # Make sure to select the correct channel on each light, one should be set to 1, the next to 3 
        self.declare_parameter('num_knobs', 2) # how many knowbs (controllable params) per light
        self.declare_parameter('num_lights', 2) # how many lights are we controlling
        try:
            self.channels = self.get_parameter('num_knobs').value * self.get_parameter('num_lights').value
        except rclpy.exceptions.ParameterUninitializedException: 
            warnings.warn("Could not start lights node, the number of channels (or number of lights) has not been specified")
            exit()
        
        # set up Lights interface object
        self.lights = ASTORA_Lights(num_knobs=self.get_parameter('num_knobs').value, num_lights=self.get_parameter('num_lights').value)
        
        # set up services
        self.get_params_srv = self.create_service(LightParams, '~/get_light_parameters', self.get_params_callback)
        self.set_params_srv = self.create_service(LightParams, '~/set_light_parameters', self.set_params_callback)

        # set up action server
        self._fade_svr= ActionServer(self,LightsFade, '~/fade_lights', self.fade_lights)

    def get_params_callback(self, request, response):
        response.intensity, response.colour_temp = self.lights.get_light_parameters()
        response.out_msg = 'Light parameters retrieved. Accepted range: 0-255'
        return response

    def set_params_callback(self, request, response):
        # Accept values between 0 and 255

        # check the input validity, what if no duration is given? Should always be passed as int or float
        response.intensity, response.colour_temp, response.out_msg = self.lights.set_light_parameters(request.intensity, request.colour_temp)
        return response
    
    def fade_lights(self, goal_handle):

        # check if duration is valid
        # should only ever come in as float

        response = LightsFade.Result()
        if goal_handle.request.duration > 0:
            intensity, colour_temp, response.out_msg = self.lights.set_light_parameters(goal_handle.request.intensity, goal_handle.request.colour_temp, goal_handle.request.duration)
            if intensity is not None and colour_temp is not None:
                response.intensity = intensity
                response.colour_temp = colour_temp
                goal_handle.succeed()
                return response
            else:
                goal_handle.abort()
                return response
        else:
            goal_handle.abort()
            response.out_msg = 'Please specify a fade duration > 0 seconds.'
            return response
        
def main():
    rclpy.init()
    lights_node = ASTORA_node()

    rclpy.spin(lights_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()