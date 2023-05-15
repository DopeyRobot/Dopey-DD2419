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
        # #...
        # #... add more nodes
        # #...

        ## EXPLORATION SUBTREE 


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


        frontier_exploration_behaviour = FrontierExploration()
        give_path_behaviour = give_path()
        #appraoch_goal_behaviour = approach_goal()
        percentage_of_known_behaviour_1 = ReturnKnownMapPercent(P)
        stop_robot_behaviour = StopRobot(2)
        lebron_tune_behaviour = playTuneBehaviour("lebronjames")

        gothim_tune_behaviour = playTuneBehaviour("gothim")

        percentage_of_known_behaviour_2 = ReturnKnownMapPercent(P)
        

        
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
        main_mission_subtree = pt.composites.Sequence(
            name="->",
        )


        stop_2_sec = StopRobot(2)
        get_closest_obj = GetClosestObjectPose()
        go_to_pose= give_path(exploring=False)
        # approach_pose = approach_goal()
        # look_at_focus = LookatCurrentFocus()
        stop_2_sec_again = StopRobot(2)
        send_goal_to_arm = SendGoalToArm()
        pickup_behavs = [PickupToTarget(), Wait(4), MoveArmToUnfold(), Wait(4)]
        pickup_seq = pt.Sequence(
            "PICKUP", [*pickup_behavs, Reset(pickup_behavs)]
        )
        get_box_pose = GetBoxPose()
        go_to_box = give_path(exploring=False)
        # approach_pose_again = approach_goal()
        # look_at_focus_again = LookatCurrentFocus()
        drop_behavs= [MoveArmToDrop(), Wait(4), OpenGripper(), Wait(4), MoveArmToHome()]
        drop_seq = pt.Sequence(
            "DROP", [*drop_behavs, Reset(drop_behavs)]
        )
        aaaah_tune_behavior = playTuneBehaviour("agh")
        # root.add_child(test)
        bombastic_tune_behavior = playTuneBehaviour("bombastic")
        bombastic_tune_behavior_again = playTuneBehaviour("bombastic")

        # approach_behavs = [Wait(8), approach_goal(), Wait(8)]
        # approach_seq = pt.Sequence("Approach", [*approach_behavs, Reset(approach_behavs)])
        # approach_behavs_again = [Wait(8), approach_goal(), Wait(8)]
        # approach_seq_again = pt.Sequence("Approach", [*approach_behavs_again, Reset(approach_behavs_again)])

        approach_behavs = approach_goal()
        approach_behavs_again = approach_goal()
        

        

        main_mission_subtree.add_children(
            [stop_2_sec, 
             get_closest_obj, 
             go_to_pose, 
             bombastic_tune_behavior,
             approach_behavs,
            #  approach_behavs_test,
            #  approach_seq,
            #  look_at_focus, 
             stop_2_sec_again, 
             send_goal_to_arm, 
             pickup_seq, 
             get_box_pose, 
             go_to_box, 
             bombastic_tune_behavior_again, 
             approach_behavs_again,
            #  approach_behavs_test_2,
            #  approach_seq_again,
            #  look_at_focus_again, 
             drop_seq,
             aaaah_tune_behavior]   
        )

        ## TEST 
        give_path_behaviour_test = give_path(exploring= False)
        stop_2_sec_first = StopRobot(2)

        test_root = pt.composites.Sequence(
            name="->",
        )
        stop_behav = StopRobot(4)
        clear_path_behav = ClearPathStuff()

        testing_services_behavs = [stop_2_sec_first,get_closest_obj, give_path_behaviour_test, bombastic_tune_behavior, approach_behavs, stop_2_sec, send_goal_to_arm, pickup_seq,
                                    get_box_pose, 
                                    go_to_box, 
                                    bombastic_tune_behavior_again, 
                                    approach_behavs_again,
                                    #  approach_behavs_test_2,
                                    #  approach_seq_again,
                                    #  look_at_focus_again, 
                                    drop_seq,
                                    aaaah_tune_behavior]
        # testing_services_behavs = [get_closest_obj, give_path_behaviour_test, bombastic_tune_behavior,stop_behav, approach_seq]


        test_root.add_children(testing_services_behavs)

        ## MAIN ROOT
        ROOT_node = pt.composites.Sequence(
            name="ROOT_seq",
        )
        ROOT_node.add_child(explore_subtree)
        ROOT_node.add_child(main_mission_subtree)
        # ROOT_node.add_child(test_root)
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
            # print(ascii_tree)


if __name__ == "__main__":

    rospy.init_node("behaviour_tree")
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
