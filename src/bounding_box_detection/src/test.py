from collections import Counter
import numpy as np

class DetectionBuffer:
    """The detection buffer is a filter to reject random false detections.
    It keeps track of how many times an instance has been detected and how many times a class has been detected."""

    def __init__(self, distance_threshold = 0.05, time_threshold = 10) -> None:
        self.instances_detected_counter = Counter() # keeps track of how many times the same instance has been detected
        self.class_counter = Counter() # keeps track of how many times a new element of every class has been detected
        self.last_time_seen = {}
        self.average_positions= {}
        self.distance_threshold = distance_threshold
        self.time_threshold = time_threshold

    def _get_instance_name(self, class_name):
        return class_name + "_" +  str(self.class_counter[class_name])
    
    def _get_class_name(self, instance_name):
        return instance_name.split("_")[0]

    def _check_position(self, position):
        keys = []
        for key, value in self.average_positions.items():
            if np.linalg.norm(position - value) < self.distance_threshold:
                keys.append(key)
        keys = sorted(keys, key=lambda x: np.linalg.norm(position - self.average_positions[x]))
        return keys
    
    def _check_time(self, timestamp):
        """Deletes all instances that have not been seen for more than time_threshold"""
        keys = []
        for key, value in self.last_time_seen.items():
            if timestamp - value > self.time_threshold:
                keys.append(key)
        
        for key in keys:
            del self.instances_detected_counter[key]
            del self.last_time_seen[key]
            del self.average_positions[key]
            # do we want to decrease the class counter as well? Problem: maybe we just deleted the last instance of a class but we want to add to a higher number.
            # What if everytime we called this we passed it the external counter?

    
    def _add(self, class_name, instance_name, position, timestamp):
        """Increments the counter for the instance (whther it exists or not) and updates the average position"""

        if instance_name in self.instances_detected_counter:
            self.average_positions[instance_name] = (self.instances_detected_counter[instance_name]/(self.instances_detected_counter[instance_name]+1))*(self.average_positions[instance_name] + position/(self.instances_detected_counter[instance_name]))
            self.instances_detected_counter[instance_name] += 1 # we saw this instance again
            self.last_time_seen[instance_name] = timestamp

        else:
            self.instances_detected_counter[instance_name] = 1
            self.average_positions[instance_name] = position
            self.class_counter[class_name] += 1 #we saw a new instance belonging to this class
            self.last_time_seen[instance_name] = timestamp
    
    def add(self, class_name, position, timestamp):
        self._check_time(timestamp) # delete old instances (possible fix: only delete them if is was only seen for e.g. 3 frames)
        keys = self._check_position(position)


        for key in keys:
            if len(keys) != 0 and self._get_class_name(key) == class_name:
                #print(f"adding {class_name} at {position} to {key}")
                self._add(class_name, key, position, timestamp)
                return
        
        #print(f"adding {class_name} at {position} to new instance")
        instance_name = self._get_instance_name(class_name)
        self._add(class_name, instance_name, position, timestamp)



# The LongTermMememory database will add an object to its list IF AND ONLY IF the detection buffer has identified an intance more than N times,
# This LongTermMemory database will check if it exists in it already by comparing class name and position (not instance name)-->
# -->There will be different numeration inside the different buffers:
#  so for example kiki_1 in the long term memory can be kiki 14 in the short detection buffer!

# If this istance does exist in the LongTermMemory, it will update it's position and timestamp
# If it has been detected more than N times in the DetectionBuffer but it doens't exist in the LongTermMemory, it will add it to the LongTermMemory

# The longTermMemory database will keep track of the detected objects' names, where they are 
# (M = Map, T=Tray, G=Grip, B = box (or maybe they can be forgoten when they are in a box?)), and their timestamps
# The longTermMemory database will only delete objects from its list when they are outdated:
# e.g., position does not correspond with TF tree, the object was picked up, etc.

class LongTermMemory:
    """Stores the objects that have been detected more than N times in the DetectionBuffer"""

    def __init__(self, frames_needed_for_reconition, distance_threshold) -> None:
        self.instances_in_memory=[]
        self.locations={} # is the instance in the Map, Tray, Grip, Box?
        self.last_time_seen = {}
        self.positions= {}
        self.distance_threshold = distance_threshold
        self.min_frames_needed = frames_needed_for_reconition # how many times the object has to be detected in the DetectionBuffer to be added to the LongTermMemory

    def _get_instance_name(self, class_name):
        return class_name + "_" +  str(self.class_counter[class_name])
    
    def _get_class_name(self, instance_name):
        return instance_name.split("_")[0]

    def _check_position(self, position):
        keys = []
        for key, value in self.positions.items():
            if np.linalg.norm(position - value) < self.distance_threshold:
                keys.append(key)
        keys = sorted(keys, key=lambda x: np.linalg.norm(position - self.average_positions[x]))
        return keys
            # do we want to decrease the class counter as well? Problem: maybe we just deleted the last instance of a class but we want to add to a higher number.
            # What if everytime we called this we passed it the external counter?

    
    def _add(self, class_name, instance_name, position, timestamp):
        """Increments the elements in the Long term memory"""
        
        if class_name in self._get_class_name(self, self.instances_in_memory): # PROBABLY AND  INCORRECT ITERATION?
            #TODO set different locations (M,T,G,B) depending on where the obkect is
            self.locations[instance_name] = "M" # we assume that the object is in the map (FOR NOW ONLY)
            self.instances_in_memory.append(instance_name)
            self.positions[instance_name] = position
            #self.last_time_seen[instance_name] = timestamp
            return

        instance_name = self._get_instance_name(class_name) # create a new instance name
        self.instances_in_memory.append(instance_name)
        self.positions[instance_name] = position
        #self.last_time_seen[instance_name] = timestamp
    
    def add(self, class_name, position, timestamp, db: DetectionBuffer):
        for key, value in db.instances_detected_counter.items():
            if value > self.min_frames_needed:
                keys = self._check_position(db.average_positions[key])
                if len(keys) != 0 and self._get_class_name(key) == class_name:
                    self._add(class_name, key, position, timestamp)
                    return






        

if __name__ == "__main__":
    db = DetectionBuffer()
    print(db._get_class_name("person_0"))
    print(db._get_instance_name("person"))
    db.add("person", np.array([0,0,0]))
    db.add("person", np.array([0.1,0.1,0.1]))
    print(db.instances_detected_counter)
    db.add("person", np.array([0.1,0.1,0.1]))
    print(db.instances_detected_counter)
    db.add("person", -np.array([0.1,0.1,0.1]))
    print(db.instances_detected_counter)
    db.add("kiki", np.array([0.1,0.1,0.1]))
    print(db.instances_detected_counter)
    db.add("kiki", np.array([0.1,0.1,0.1]))
    print(db.instances_detected_counter)
    db.add("kiki", -np.array([0.1,0.1,0.1]))
    print(db.instances_detected_counter)