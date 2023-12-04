import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import websocket
import subprocess as sp
import numpy as np
import time
from capture_types.srv import LightParams
from capture_types.action import LightsFade

class ASTORA_node(Node):
    def __init__(self):
        super().__init__('lights')
        # Each one of our ASTORA Soft Panels has 2 knobs, so we need 2 channels per panel
        # 2 lights --> 4 channels
        # Make sure to select the correct channel on each light, one should be set to 1, the next to 3 
        self.declare_parameter('num_knobs', 2) # how many knowbs (controllable params) per light
        self.declare_parameter('num_lights', 2) # how many lights are we controlling
        # when initializing the lights node, please make sure to specify the number of channels
        self.declare_parameter('fade_frequency', 1) # Hz, use this as an upper bound only, the actual possible highest frequency is determined by the websocket connection

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
        self.get_params_srv = self.create_service(LightParams, '~/get_light_parameters', self.get_params_callback)
        self.set_params_srv = self.create_service(LightParams, '~/set_light_parameters', self.set_params_callback)

        # set up action server
        self._fade_svr= ActionServer(self,LightsFade, '~/fade_lights', self.fade_lights)

    def get_params_callback(self, request, response):
        # Command syntax: 'api trigger|command|universe|request starting index|request range'
        response.intensity, response.colour_temp = self.get_params()
        response.out_msg = 'Light parameters retrieved. Accepted range: 0-255'
        return response
    
    def get_params(self):
        self.ws.send(f'QLC+API|getChannelsValues|1|1|{self.channels}')
        out = self.ws.recv()
        channel_values = np.asarray(out.split('|')[2:]).reshape(-1,3)[:,:2].astype(int) # reformat into the individual channels
        knob_values = channel_values[:,1].reshape(-1,self.get_parameter('num_lights').value) # reshape into the individual knobs
        intensity = knob_values[:,0].tolist()
        colour_temp = knob_values[:,1].tolist()
        return intensity, colour_temp

    def set_params_callback(self, request, response):
        # Accept values between 0 and 255
        response.out_msg = ''
        if len(request.intensity) < 1 or len(request.colour_temp) < 1 or len(request.intensity) != len(request.colour_temp):
            response.out_msg += f'Invalid number of parameters. Specify one intensity and one colour temperature per light.'
            return response
        
        combined_array = np.column_stack((request.intensity, request.colour_temp)).reshape(-1)
        if not all([0 <= value <= 255 for value in combined_array]):
            combined_array = combined_array.clip(0,255)
            response.out_msg += f'Some Parameters were limited to the accepted interval 0-255. '
        self.set_params(combined_array)
        response.intensity = combined_array.clip(0,255).reshape(-1,self.get_parameter('num_lights').value)[:,0].tolist()
        response.colour_temp = combined_array.clip(0,255).reshape(-1,self.get_parameter('num_lights').value)[:,1].tolist()
        response.out_msg += f'{len(combined_array)} Light parameters set.'
        return response
    
    def set_params(self, params):
        for channel, value in enumerate(params):
            self.ws.send(f'CH|{channel + 1}|{value}')
        return
    
    def fade_lights(self, goal_handle):
        # The smoothness of the fade is limited by the frequency of the websocket connection
        # The exact duration also sometimes varies by a small margin, due to comms time
        # TODO: Determine the intermediate step parameters by time passed in each loop instead of pre-calculating the needed steps and sleeping for a fixed time

        result = LightsFade.Result()

        if len(goal_handle.request.intensity) < 1 or len(goal_handle.request.colour_temp) < 1 or len(goal_handle.request.intensity) != len(goal_handle.request.colour_temp):
            result.out_msg = 'Invalid number of parameters. Specify one intensity and one colour temperature per light.'
            goal_handle.abort()
            return result
        if goal_handle.request.duration == 0:
            result.out_msg = 'Please specify a fade duration > 0 seconds.'
            goal_handle.abort()
            return result
        
        start_vals = np.asarray(self.get_params())
        target_vals = np.asarray([goal_handle.request.intensity, goal_handle.request.colour_temp])

        if start_vals.shape != target_vals.shape:
            result.out_msg = f'Invalid number of parameters. Expected ({start_vals.shape}) parameters.'
            goal_handle.abort()
            return result
        
        num_steps = int(goal_handle.request.duration * self.get_parameter('fade_frequency').value)
        time_per_step = 1 / self.get_parameter('fade_frequency').value
        intermediate_vals = np.zeros((num_steps,) + start_vals.shape, dtype=int)
        
        for step in range(num_steps):
            t = (step+1) * time_per_step / goal_handle.request.duration
            intermediate_vals[step] = np.rint(start_vals + (target_vals - start_vals) * t).astype(int)

        ss = time.time()
        for step in range(num_steps):
            self.set_params(np.concatenate(intermediate_vals[step].T))
            time.sleep(time_per_step)
        es = time.time()

        goal_handle.succeed()
        result = LightsFade.Result()
        result.out_msg = f'{goal_handle.request.duration} second light fade completed.'
        return result

def main():
    rclpy.init()
    lights_node = ASTORA_node()

    rclpy.spin(lights_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()