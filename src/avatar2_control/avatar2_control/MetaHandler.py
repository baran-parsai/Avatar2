import py_trees

class MetaHandler(py_trees.behaviour.Behaviour):
    """Handle meta requests to the avatar"""

    def __init__(self, name):
        super(MetaHandler,self).__init__(name)

    def setup(self, node='eh'):
        self.logger.debug(f"  {self.name} [MetaHandler:setup()]")
        self.logger.debug(node)

    def initialise(self):
        self.logger.debug(f"  {self.name} [MetaHandler:initialise()]")
        self._blackboard = py_trees.blackboard.Client(name="Meta Handler")
        self._blackboard.register_key(key="/in_message", access=py_trees.common.Access.READ)
        print(self._blackboard)

    def update(self):
        self.logger.debug(f"  {self.name} [MetaHandler:update()]")
        print(self._blackboard)


        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"  {self.name} [MetaHandler:terminate()][{self.status}->{new_status}]")
        
        
