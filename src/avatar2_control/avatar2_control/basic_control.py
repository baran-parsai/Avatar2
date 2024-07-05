import py_trees
import py_trees_ros
import py_trees_ros.trees
import py_trees.console as console
import rclpy

from . import MetaHandler
from . import AvatarHandler
from avatar2_interfaces.msg import TaggedString



def controller_create_root() -> py_trees.behaviour.Behaviour:
    """Create the entire tree"""

    # High to low priority and restart on each tick
    root = py_trees.composites.Sequence(name = "Simple controller", memory = False)

    # Map interesting ros topics to the blackboard
    topics2bb = py_trees.composites.Sequence(name = "Topics2BB", memory = True)
    in_message = py_trees_ros.subscribers.ToBlackboard(name="in_message",
         qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
         topic_name="/avatar2/in_message", topic_type=TaggedString, blackboard_variables = {'in_message' : None})

    # Deal with meta commands to the avatar
    meta = MetaHandler.MetaHandler("meta handler")

    # High to low priority and restart on each tick
    bits = py_trees.composites.Selector(name = "Behaviour types", memory = False)

    # Deal with regular avatar commands
    avatar = AvatarHandler.AvatarHandler("avatar handler")

    # an always running behaviour
    idle = py_trees.behaviours.Running(name="Idle")

    # build the tree
    bits.add_child(meta)
    bits.add_child(avatar)
    topics2bb.add_child(in_message)
    root.add_child(topics2bb)
    root.add_child(bits)
    root.add_child(idle)

    return root

        

def main(args=None):
    rclpy.init(args=args)
    root = controller_create_root()
    tree = py_trees_ros.trees.BehaviourTree(root = root, unicode_tree_debug = True)
    try:
        tree.setup(node_name = "basic_control", timeout = 15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + f"failed tosetup the tree, aborting [{e}]" + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror(console.red + "tree setup interrupted" + console.reset)
        tree.shutdown()

    tree.tick_tock(period_ms = 1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
