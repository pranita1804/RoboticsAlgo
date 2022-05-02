#!/usr/bin/env python
import rospy
import heapq
import math
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Twist
path = []
r = True
end = False
index = 0
map_pos = []	
publish_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=100)
def h(p1,p2):
	return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
def astar(grid,start,goal):
	global path
	close_list = set()
	open_list = []
	parent = {}
	f = {}
	g = {}
	heapq.heapify(open_list)
	f[start] = h(start,goal)
	g[start] = 0
	adjecent_nodes = [(0,1),(0,-1),(1,0),(-1,0)]
	heapq.heappush(open_list , (f[start],start))
	while open_list > 0:
		current_node = heapq.heappop(open_list)[1]
		if current_node == goal:
			while current_node !=start:
				path.append(current_node)
				current_node = parent[current_node]
			path.reverse()
			return path
		close_list.add(current_node)
		for x in adjecent_nodes:
			adjecent_node = current_node[0]+x[0],current_node[1]+x[1]
			if adjecent_node[0] <= 0 or adjecent_node[0] > (len(grid[len(grid)-1])-1) or adjecent_node[1] <= 0 or adjecent_node[1] > (len(grid)-1):
				continue
			if grid[adjecent_node[1]][adjecent_node[0]] == 1:
				continue
			g_score = g[current_node] + h(current_node,adjecent_node)
			if adjecent_node in close_list and g_score >= g.get(adjecent_node, 0):
				continue
			if g_score < g.get(adjecent_node,0) or adjecent_node not in [i[1] for i in open_list]:
				g[adjecent_node] = g_score
				f[adjecent_node] = g[adjecent_node] + h(adjecent_node, goal)
				parent[adjecent_node] = current_node
				heapq.heappush(open_list, (f[adjecent_node],adjecent_node))
	return false			


def callBack(data):
	global end,index,publish_vel,path,r
	if not end:
		current_pos_x = data.pose.pose.position.x
		current_pos_y = data.pose.pose.position.y
		orient = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]
		map_pos_x = path[index][0]-9+0.7
		map_pos_y = 10-path[index][1]-0.7
		map_pos = (map_pos_x,map_pos_y)
     		orient_goal=math.atan2((map_pos[1]-current_pos_y),(map_pos[0]-current_pos_x))
		error = orient_goal - orient
		if r == True:
			if math.fabs(error) > math.radians(6):
                    		if error < 0:
                        		error += math.pi * 2
                    		elif error > math.pi * 2:
                        		error -= math.pi * 2
                    		if error > math.pi:
                        		msg=Twist()
					msg.linear.x=0
					msg.linear.y=0
					msg.linear.z=0
					msg.angular.z=-0.75
					publish_vel.publish(msg)

                    		else:
                        		msg=Twist()
					msg.linear.x=0
					msg.linear.y=0
					msg.linear.z=0
					msg.angular.z=+0.75
					publish_vel.publish(msg)
                	else:
                		r = False
            	else:
                	error=math.sqrt((map_pos[0]-current_pos_x)**2+(map_pos[1]-current_pos_y)**2)
                	if error> 0.5:
                    		msg1=Twist()
				msg1.linear.x=0.75
				msg1.linear.y=0
				msg1.linear.z=0
				publish_vel.publish(msg1)
                	else:
				r=True
                    		if index+1<len(path):
					rospy.loginfo(map_pos[0])
					rospy.loginfo(map_pos[1])
                        		index+=1
                    		else:
					
                        		end=True


	
	
if __name__ == '__main__':
	try:
        	rospy.init_node('lab4', anonymous=True)
		global end
		grid = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       			[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       			[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       			[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       			[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       			[0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       			[0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       			[0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
       			[0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       			[0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       			[0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       			[0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       			[0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       			[0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
       			[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       			[0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
       			[0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       			[0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       			[0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       			[0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]
			]
		goalx = rospy.get_param('goalx')
		goaly = rospy.get_param('goaly')
		rospy.loginfo(goalx)
		rospy.loginfo(goaly)
		startx = -8.0
		starty = -2.0
		goalxmap = int(9+goalx)
		goalymap = int(10-goaly)
		startxmap = int(9+startx)
		startymap = int(10-starty)
		start = (startxmap,startymap)
		goal = (goalxmap,goalymap)
		path = astar(grid,start,goal)
		rospy.loginfo(path)
		for i in path:
			m = (i[0]-9,10-i[1])
			rospy.loginfo(m)
		robot_pos_pub = rospy.Subscriber("/base_pose_ground_truth", Odometry,callBack )
		rospy.spin()
	except rospy.ROSInterruptException:
        	pass
  
