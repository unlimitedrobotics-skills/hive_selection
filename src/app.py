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
                'working_camera' : self.camera,
                'map_name' : self.map_name,
                'item_name' : self.item_name,
                'tag_size' : self.tag_size
            }
        )


    async def main(self):
        execute_results = await self.hive_selection.execute_main(
            execute_args = {
                'angle_to_goal' : self.angle_to_goal
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
        self.camera = self.get_argument('-c', '--camera', 
                type = str, 
                required = True,
                help = 'name of camera to use'
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

