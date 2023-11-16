from raya.application_base import RayaApplicationBase
from raya.controllers import NavigationController
from raya.skills import RayaSkillHandler

from skills.hive_selection import SkillHiveSelection

# ------------------------------- Application ------------------------------- #
class RayaApplication(RayaApplicationBase):

    async def setup(self):
        self.hive_selection = self.register_skill(SkillHiveSelection)
        await self.hive_selection.execute_setup(
            setup_args = {
                'working_camera_1' : self.camera_1,
                'working_camera_2' : self.camera_2,
                'map_name' : self.map_name,
                'item_name' : self.item_name,
                'tag_size' : self.tag_size
            }
        )

        self.item_dict = {'bottle' : 4,
                          'towel' : 2}


    async def main(self):
        execute_results = await self.hive_selection.execute_main(
            execute_args = {
                'angle_to_goal' : self.angle_to_goal,
                'identifier' : [self.item_dict[self.item_name]]
            },
            callback_feedback = self.cb_feedback
        )
        self.log.debug(execute_results)


    async def finish(self):
        await self.hive_selection.execute_finish()


# -------------------------------- Helpers -------------------------------- #
    async def cb_feedback(self, feedback):
        self.log.debug(feedback)

    def get_arguments(self):
        self.camera_1 = self.get_argument('-c1', '--camera1', 
                type = str, 
                required = True,
                help = 'name of camera to use on approach'
            )

        self.camera_2 = self.get_argument('-c2', '--camera2', 
                type = str, 
                required = True,
                help = 'name of camera to use on detection'
            )      
        
        self.angle_to_goal = self.get_argument('-a', '--angle', 
                type = float, 
                required = True,
                help = 'Angle to approach'
            )  
        
        self.map_name = self.get_argument('-m', '--map_name',
                type = str,
                required = True,
                help = 'map name to use'
            )   
        
        self.item_name = self.get_argument('-i', '--item_name',
                type = str,
                required = True,
                help = 'item to pick')
    

        self.tag_size = self.get_argument(
            '-ts',  '--tag_size', 
            type = float, 
            default = 0.04,
            required = True,
            help = 'tag size in meters'
        )

