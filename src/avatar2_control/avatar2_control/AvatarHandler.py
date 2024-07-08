import py_trees
from avatar2_interfaces.msg import TaggedString


class AvatarHandler(py_trees.behaviour.Behaviour):
    """Handle normal requests of the avatar"""

    def __init__(self, name):
        super(AvatarHandler, self).__init__(name)

    def setup(self, **kwargs):
        #kwargs['node'] is our node (ignored here)
        self._node = kwargs['node']
        self._node.get_logger().warning('avatar handler is alive')
        self.logger.debug(f"  {self.name} [AvatarHandler:setup()]")

    def initialise(self):
        self.logger.debug(f"  {self.name} [NormalHandler:initialise()]")
        self._blackboard = py_trees.blackboard.Client(name="Avatar Handler")
        self._blackboard.register_key(key="/in_message", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_message.text", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_message.text.data", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_message.audio_sequence_number", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/avatar_name", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/out_message", access=py_trees.common.Access.WRITE)


    def update(self):
        self.logger.debug(f"  {self.name} [AvatarHandler:update()]")
        self._node.get_logger().warning(f'avatar handler update text is {self._blackboard.in_message.text.data}')
        self._node.get_logger().warning(f'avatar handler update number is {self._blackboard.in_message.audio_sequence_number}')

        sequence = self._blackboard.in_message.audio_sequence_number
        in_text = self._blackboard.in_message.text.data.lower()

        avatar_name = self._blackboard.avatar_name.lower()
        self._node.get_logger().warning(f'Avatars name is {avatar_name}')

        response =  "avatar responding to " + in_text
        tagged_string = TaggedString()
        tagged_string.header.stamp = self._node.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = sequence
        tagged_string.text.data = str(response)
        self._blackboard.out_message = tagged_string



        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"  {self.name} [NormalHandler:terminate()][{self.status}]->{new_status}]")
        
        
