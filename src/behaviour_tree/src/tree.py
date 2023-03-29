import py_trees as pt, py_trees_ros as ptr, rospy
from reactive_sequence import RSequence
from behaviours import *


class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")

        behaviour1 = behaviours.B1
        behaviour2 = behaviours.B2
        behaviour3 = behaviours.B3
        behaviour4 = behaviours.B4
        behaviour5 = behaviours.B5


        #AND node example
        AND = RSequence(name="Main sequence", children=[behaviour1,behaviour2])

        #OR node example
        OR = pt.composites.Selector(
            name="Sucessful placement on table 2 or reset tree ", 
            children=[behaviour1,behaviour2, behaviour3]
	    )

        #Initialise tree like this
        tree = RSequence(name="Main sequence", children=[AND,OR])

        #Create another node
        AND2 = RSequence(name="Main sequence", children=[behaviour4,behaviour5])

        #Continue building like this
        tree = RSequence(name="Main sequence", children=[tree,AND2])
        # 
        #...
        #... add more nodes
        #...
        
        super(BehaviourTree, self).__init__(tree)

        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): self.tick_tock(1)



if __name__ == "__main__":


	rospy.init_node('behaviour_tree')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
