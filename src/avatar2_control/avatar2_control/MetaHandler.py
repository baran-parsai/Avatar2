import py_trees

class MetaHandler(py_trees.behaviour.Behaviour):
    """Handle meta requests to the avatar"""

    def __init__(self, name):
        super(MetaHandler,self).__init__(name)
        self._last_audio_sequence = -1

    def setup(self, **kwargs):
        #kwargs['node'] is our ros node
        self._node = kwargs['node']
        self._node.get_logger().warning('meta handler is alive')
        self.logger.debug(f"  {self.name} [MetaHandler:setup()]")

    def initialise(self):
        self.logger.debug(f"  {self.name} [MetaHandler:initialise()]")
        self._blackboard = py_trees.blackboard.Client(name="Meta Handler")
        self._blackboard.register_key(key="/in_message", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_message.text", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_message.text.data", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_message.audio_sequence_number", access=py_trees.common.Access.READ)

    def update(self):
        self.logger.debug(f"  {self.name} [MetaHandler:update()]")
        self._node.get_logger().warning(f'meta handler update text is {self._blackboard.in_message.text.data}')
        self._node.get_logger().warning(f'meta handler update number is {self._blackboard.in_message.audio_sequence_number}')

        sequence = self._blackboard.in_message.audio_sequence_number
        in_text = self._blackboard.in_message.text.data

        if sequence == self._last_audio_sequence:
            return py_trees.common.Status.FAILURE

        self._node.get_logger().warning(f'meta handler processing a new bit of text')
        self._last_audio_sequence = sequence

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"  {self.name} [MetaHandler:terminate()][{self.status}->{new_status}]")
        
        
