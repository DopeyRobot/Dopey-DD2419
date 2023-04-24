#!/usr/bin/env python3
import py_trees as pt, py_trees_ros as ptr, rospy
from reactive_sequence import RSequence
from behaviours import *
import behaviours

from tf2_ros import Buffer, TransformListener, TransformStamped
import tf2_geometry_msgs

import functools
import pdb

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

        root = pt.composites.Parallel(
        name="root",
            )
        
        give_path_behaviour = behaviours.give_path()
	
        test = behaviours.give_path()
        test1 = behaviours.give_path()
        test2 = behaviours.give_path()

        AND = RSequence(name="->", children=[test1,test2])

        OR = pt.composites.Selector(
            name="?", 
            children=[give_path_behaviour,test,AND]
	    
	    )

        root.add_child(OR)
	

        # for job in ["Action 1", "Action 2", "Action 3"]:
        #     success_after_two = pt.behaviours.Success(
        #         name=job,
        #         queue=[pt.common.Status.RUNNING],
        #         eventually = pt.common.Status.SUCCESS
        #     )
        #     give_path_behaviour.add_child(success_after_two)
        pt.display.render_dot_tree(root)
        # tree = give_path_behaviour
        # tree = root 
        
        

        # behaviour_tree = root
        # behaviour_tree = pt.trees.BehaviourTree(root)
        
        
        super(BehaviourTree, self).__init__(root)

        # ##ATTEMPT TO PRINT CURRENT TREEE
        # snapshot_visitor = pt.visitors.SnapshotVisitor()
        
        # self.add_post_tick_handler(
        #     functools.partial(self.post_tick_handler,
        #                     snapshot_visitor))
        # self.visitors.append(snapshot_visitor)
        

        # TEST FOR CHECKING TRANSFORMS OF BASELINK TO MAP
        # base_link_origin = PoseStamped()
        # base_link_origin.header.stamp = rospy.Time.now()
        # transform_to_map = self.buffer.lookup_transform("map", "base_link", base_link_origin.header.stamp , rospy.Duration(1))  
        # transform_to_map_from_odom = self.buffer.lookup_transform("map", "odom", base_link_origin.header.stamp , rospy.Duration(1))  
        # baseInMapPose = tf2_geometry_msgs.do_transform_pose(base_link_origin, transform_to_map)
        # print(rospy.Time.now())
        # print(transform_to_map,transform_to_map_from_odom)

        
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): self.tick_tock(1)

        # #MAJA
        # print("\n" + "-"*80)
        # print("Behavior Tree")
        # print("-"*80)
        # print(pt.display.ascii_tree(root))
        # rospy.sleep(0.5)
	
    # def post_tick_handler(self,snapshot_visitor, behaviour_tree):
    #     pdb.set_trace()
    #     print(
    #         pt.display.ascii_tree(
    #             behaviour_tree.root,
    #             visited=snapshot_visitor.visited,
    #             previously_visited=snapshot_visitor.visited
    #         )
    #     )
    # def print_snapshot_continously(self,behaviour_tree):
    #     snapshot_visitor = pt.visitors.SnapshotVisitor()
    #     behaviour_tree.add_post_tick_handler(
    #         functools.partial(self.post_tick_handler,
    #                         snapshot_visitor))
    #     behaviour_tree.visitors.append(snapshot_visitor)
            
           



if __name__ == "__main__":


	rospy.init_node('behaviour_tree')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
