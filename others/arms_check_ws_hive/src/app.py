import argparse
from raya.application_base import RayaApplicationBase
from raya.controllers import ArmsController
import json

from raya.enumerations import ANG_UNIT
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import numpy as np
from .constants import *

class RayaApplication(RayaApplicationBase):
    async def setup(self):
        self.arms: ArmsController = await self.enable_controller("arms")
        self.arm_joints = self.arms.get_joints_list(arm=ARM)
        await self.add_constraints()


    async def loop(self):
        samples_height = int((MAX_Z-MIN_Z)/STEP_Z)+1
        samples_x = int((MAX_X-MIN_X)/STEP_X)+1
        samples_y = int((MAX_Y-MIN_Y)/STEP_Y)+1
        results = {}
        for height in np.linspace(MIN_Z, MAX_Z, samples_height):
            results[height]=[]
            for x in np.linspace(MIN_X, MAX_X, samples_x):
                for y in np.linspace(MIN_Y, MAX_Y, samples_y):
                    print(f'{x,y,height}')
                    _y = y if ARM != 'right_arm' else -y
                    pick = {"x": x, "y": _y, "z": height, "roll": ROLL,
                            "pitch": PITCH, "yaw": YAW}
                    post_pick = {"x": x, "y": _y, "z": height + POST_PICK_OFFSET, 
                                 "roll": ROLL,"pitch": PITCH, "yaw": YAW}
                    if await self.check_pick(
                        pick, post_pick, execute_pick=EXECUTE):
                        results[height].append([y, x])
                        print(x,y,height)
            if not len(results[height]):
                results.pop(height)
        
        if len(results):
            self.plot_results(results)
        else:
            self.log.error("any solution has been found in the choosen ranges")
        self.finish_app()


    async def finish(self):
        await self.arms.remove_constraints(arm=ARM)
        print(f"Application has finished")
    
    async def check_pick(self, pick:dict, post_pick: dict, execute_pick: False):
        try:
            res= await self.arms.is_pose_valid(arm= ARM, **pick,
                                          name_trajectory='pick_hive',
                                          save_trajectory=True,
                                          start_joints=PRE_PICK_JOINTS,
                                          name_start_joints=self.arm_joints,
                                          use_start_joints= True,
                                          wait= True
                                          )
            joints = self.arms.convert_angle_joints_to_degrees(
                arm=ARM,
                name_joints=self.arm_joints,
                angle_joints=list(res[2]))
            res = await self.arms.is_pose_valid(arm= ARM, **post_pick, 
                                          cartesian_path= True,
                                          name_trajectory='post_pick_hive',
                                          save_trajectory=True,
                                          start_joints=joints,
                                          name_start_joints=self.arm_joints,
                                          use_start_joints= True,
                                          wait= True)
            if res[1]<0.9:
                return False
            
        except Exception as e:
            print(e)
            return False
        
        if execute_pick:
            await self.arms.execute_predefined_trajectory('pick_hive', 
                                                    go_to_start_position=True,
                                                    wait=True)
            await self.arms.execute_predefined_trajectory('post_pick_hive', 
                                                    go_to_start_position=False,
                                                    wait=True)

        return True


    async def add_constraints(self):
        pass


    def plot_results(self,results):
        num_subplt = len(results)
        adding_row = 1 if num_subplt %3 != 0 else 0
        fig, ax = plt.subplots(ncols=3, 
                          nrows=(num_subplt//3) + adding_row, 
                          sharex= 'col', 
                          sharey= 'row') 
        ax = ax.flatten()


        # Plot the data
        for c, height in enumerate(results):
            res = np.array(results[height])
            ax[c].plot(res[:,0],res[:,1], 'x')
            ax[c].set_title(f'{height}')
            ax[c].grid(True)

        # Hide any empty subplots
        # for i in range(len(results), len(ax)):
        #     ax[i].axis('off')

        # Adjust layout
        plt.tight_layout()

        # Show the plot
        plt.show()