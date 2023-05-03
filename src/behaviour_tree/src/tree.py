#!/usr/bin/env python3
import functools
import pdb

import behaviours
import py_trees as pt
import py_trees_ros as ptr
import rospy
import tf2_geometry_msgs
from behaviours import *
from geometry_msgs.msg import PoseStamped
from reactive_sequence import RSequence
from tf2_ros import Buffer, TransformListener, TransformStamped


class BehaviourTree(ptr.trees.BehaviourTree):
    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")

        # self.buffer = Buffer(rospy.Duration(100.0))
        # self.listener = TransformListener(self.buffer)

        # behaviour1 = behaviours.B1
        # behaviour2 = behaviours.B2
        # behaviour3 = behaviours.B3
        # behaviour4 = behaviours.B4
        # behaviour5 = behaviours.B5

        # #AND node example
        # AND = RSequence(name="Main sequence", children=[behaviour1,behaviour2])

        # #OR node example
        # OR = pt.composites.Selector(
        #     name="Sucessful placement on table 2 or reset tree ",
        #     children=[behaviour1,behaviour2, behaviour3]
        # )

        # #Initialise tree like this
        # tree = RSequence(name="Main sequence", children=[AND,OR])

        # #Create another node
        # AND2 = RSequence(name="Main sequence", children=[behaviour4,behaviour5])

        # #Continue building like this
        # tree = RSequence(name="Main sequence", children=[tree,AND2])
        # #
        # #...
        # #... add more nodes
        # #...

        ## EXPLORATION SUBTREE 
        root_explore = pt.composites.Sequence(
            name="->",
        )

        P = 0.9 #percentage to check for complete exploration

        frontier_exploration_behaviour = behaviours.FrontierExploration()
        give_path_behaviour = behaviours.give_path()
        percentage_of_known_behaviour_1 = behaviours.ReturnKnownMapPercent(P)
        stop_robot_behaviour = behaviours.StopRobot(2)
        lebron_tune_behaviour = behaviours.playTuneBehaviour("lebronjames")

        gothim_tune_behaviour = behaviours.playTuneBehaviour("gothim")

        percentage_of_known_behaviour_2 = behaviours.ReturnKnownMapPercent(P)
        

        
        root_explore.add_child(stop_robot_behaviour)
        root_explore.add_child(lebron_tune_behaviour)
        root_explore.add_child(frontier_exploration_behaviour)
        root_explore.add_child(give_path_behaviour)
        root_explore.add_child(percentage_of_known_behaviour_1)

        OR_explore = pt.composites.Selector(
            name="?"
        )

        OR_explore.add_child(percentage_of_known_behaviour_2)
        OR_explore.add_child(root_explore)

        explore_subtree = pt.composites.Sequence(
            name="->",
        )
        explore_subtree.add_child(OR_explore)
        explore_subtree.add_child(gothim_tune_behaviour)

        

        ## MAIN MISSION SUBTREE

        # rospy.wait_for_message("pickup_goal", PoseStamped)
        # behavs = [
        #     PickupToTarget(),
        #     Wait(4),
        #     MoveArmToUnfold(),
        #     Wait(4),
        #     MoveArmToDrop(),
        #     Wait(4),
        #     OpenGripper(),
        # ]

        # pickup_behavs = pt.Sequence(
        #     "PICKUP", [PickupToTarget(), Wait(4), MoveArmToUnfold(), Wait(4)]
        # )

        # drop_behavs = pt.Sequence(
        #     "DROP", [MoveArmToDrop(), Wait(4), OpenGripper(), Wait(4), MoveArmToHome()]
        # )

        # test = pt.Sequence("pickanddrop", [pickup_behavs, drop_behavs])
        # # root.add_child(test)

        # root_pickup = pt.composites.Sequence(name="Pick up object")
        # root_drop = pt.composites.Sequence(name="Drop object")

        

        # pick_up_object_behaviour = behaviours.PickUpObject()
        # drop_object_behaviour = behaviours.DropObject()

        # root_pickup.add_child(pick_up_object_behaviour)
        # root_pickup.add_child(give_path_behaviour)

        # root_drop.add_child(drop_object_behaviour)
        # root_drop.add_child(give_path_behaviour)

        ## MAIN ROOT
        ROOT_node = pt.composites.Sequence(
            name="ROOT_seq",
        )
        ROOT_node.add_child(explore_subtree)
        pt.display.render_dot_tree(ROOT_node)

        super(BehaviourTree, self).__init__(ROOT_node)

        # Post tick handlers
        self.visitors.append(pt.visitors.DebugVisitor())
        snapshot_visitor = pt.visitors.SnapshotVisitor()
        self.visitors.append(snapshot_visitor)

        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown():
            self.tick()
            # print("TICK IN TREE")
            ascii_tree = pt.display.ascii_tree(
                self.root, snapshot_information=snapshot_visitor
            )
            print(ascii_tree)


if __name__ == "__main__":

    rospy.init_node("behaviour_tree")
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
