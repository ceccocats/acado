import rospy
import tf

rospy.init_node('record_path')
listener = tf.TransformListener()
rate = rospy.Rate(2)

path = []
path.append((0,0))

while not rospy.is_shutdown():
    try:
        x, y, z = listener.lookupTransform("map", "footprint", rospy.Time(0))[0]
        if abs(path[-1][0] - x) > 0.1 or abs(path[-1][1] - y) > 0.1:  
            path.append((x,y))
            print (x,y)

            f = open("path.txt", "w")
            f.write(str(len(path)) + "\n")
            for p in path:
                f.write(str(p[0]) + " " + str(p[1]) + "\n")
            f.close()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
   
    rate.sleep()