import py_trees

class AvatarHandler(py_trees.behaviour.Behaviour):
    """Handle normal requests of the avatar"""

    def __init__(self, name):
        super(AvatarHandler, self).__init__(name)

    def setup(self, **kwargs):
        #kwargs['node'] is our node (ignored here)
        self.logger.debug(f"  {self.name} [NormalHandler:setup()]")

    def initialise(self):
        self.logger.debug(f"  {self.name} [NormalHandler:initialise()]")

    def update(self):
        self.logger.debug(f"  {self.name} [NormalHandler:update()]")

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"  {self.name} [NormalHandler:terminate()][{self.status}]->{new_status}]")
        
        
