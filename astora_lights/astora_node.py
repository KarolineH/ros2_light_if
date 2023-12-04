import rclpy
from rclpy.node import Node
import websocket
import subprocess as sp
import numpy as np
import time
from capture_types.srv import LightParams

class ASTORA_node(Node):
    def __init__(self):
        super().__init__('lights')
        # Each one of our ASTORA Soft Panels has 2 knobs, so we need 2 channels per panel
        # 2 lights --> 4 channels
        # Make sure to select the correct channel on each light, one should be set to 1, the next to 3 
        self.declare_parameter('num_knobs', 2) # how many knowbs (controllable params) per light
        self.declare_parameter('num_lights', 2) # how many lights are we controlling
        # when initializing the lights node, please make sure to specify the number of channels

        try:
            self.channels = self.get_parameter('num_knobs').value * self.get_parameter('num_lights').value
        except rclpy.exceptions.ParameterUninitializedException: 
            warnings.warn("Could not start lights node, the number of channels (or number of lights) has not been specified")
            exit()
        
        self.server = sp.Popen(['qlcplus', '--web', '--nowm', '--nogui'], stdout=sp.DEVNULL)
        self.port = 9999        # Param: I don't know if qlc+ can use a different port
        self.ws = websocket.WebSocket()
        time.sleep(5)
        self.ws.connect(f'ws://127.0.0.1:{self.port}/qlcplusWS')

        # set up services
        self.get_params_srv = self.create_service(LightParams, '~/get_light_parameters', self.get_params)
        self.set_params_srv = self.create_service(LightParams, '~/set_light_parameters', self.set_params)

    def get_params(self, request, response):
        # Command syntax: 'api trigger|command|universe|request starting index|request range'
        self.ws.send(f'QLC+API|getChannelsValues|1|1|{self.channels}')
        out = self.ws.recv()
        channel_values = np.asarray(out.split('|')[2:]).reshape(-1,3)[:,:2].astype(int) # reformat into the individual channels
        knob_values = channel_values[:,1].reshape(-1,self.get_parameter('num_lights').value) # reshape into the individual knobs
        response.intensity = knob_values[:, 0].tolist()
        response.colour_temp = knob_values[:, 1].tolist()
        response.out_msg = 'Light parameters retrieved. Accepted range: 0-255'
        return response
    
    def set_params(self, request, response):
        # Accept values between 0 and 255
        response.out_msg = ''
        combined_array = np.column_stack((request.intensity, request.colour_temp)).reshape(-1)
        if not all([0 <= value <= 255 for value in combined_array]):
            response.out_msg += f'Some Parameters were limited to the accepted interval 0-255. '
        for channel, value in enumerate(combined_array.clip(0,255)):
            self.ws.send(f'CH|{channel + 1}|{value}')
        response.intensity = combined_array.clip(0,255).reshape(-1,self.get_parameter('num_lights').value)[:,0].tolist()
        response.colour_temp = combined_array.clip(0,255).reshape(-1,self.get_parameter('num_lights').value)[:,1].tolist()
        response.out_msg += f'Light parameters set.'
        return response

def main():
    rclpy.init()
    lights_node = ASTORA_node()

    rclpy.spin(lights_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()