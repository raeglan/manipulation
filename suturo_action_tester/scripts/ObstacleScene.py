#!/usr/bin/env python
import rospy

import tf

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from suturo_action_tester.msg import SuturoCreateObject
from suturo_action_tester.msg import AttachObject
from std_msgs.msg import *
from suturo_perception_msgs.msg import ObjectDetection
from geometry_msgs.msg import *

from InteractionManager import *
import rospkg

from yaml import load, dump

from threading import Lock

def createIdentityTf(ref):
	out = {}
	out['frame_id'] = ref 
	out['translation'] = (0,0,0)
	out['orientation'] = (0,0,0,1)

	return out

def resolvePath(path):
	fragments = path.split('/')
	rospack = rospkg.RosPack()
	pp = rospack.get_path(fragments[0])

	for x in range(1,len(fragments)):
		pp += '/' + fragments[x]

	return pp 


class ObstacleScene(object):

	def __init__(self, ns, refFrame):
		self.visPub = rospy.Publisher('suturo/obstacle_vis', MarkerArray, queue_size=2)
		self.percPub = rospy.Publisher('/percepteros/object_detection', ObjectDetection, queue_size=2)
		self.tfBr = tf.TransformBroadcaster()
		self.tfLt = tf.TransformListener()
		self.objects = {}
		self.ns = ns
		self.nid = 0;
		self.refFrame = refFrame
		rospy.Subscriber('suturo/create_object', SuturoCreateObject, self.createObject)
		rospy.Subscriber('suturo/attach_object', AttachObject, self.attachTo)

		rospy.Subscriber('suturo/save_scene', String, self.saveScene)
		rospy.Subscriber('suturo/load_scene', String, self.loadScene)		

		self.intManager = InteractionManager(self)
		self.mutex = Lock()

	def createObject(self, sco):
		self.addObject(sco.name, createIdentityTf(sco.refFrame), sco.vis)


	def removeObject(self, name):
		if name in self.objects:
			self.intManager.removeObject(name)

			del self.objects[name]
			m = Marker()
			m.ns = self.ns
			m.header.stamp = rospy.get_rostime()
			m.action = Marker.DELETE
			ma = MarkerArray()
			ma.markers = [m]

			self.visPub.publish(ma)

	def setObjectPose(self, name, pose):
		self.objects[name]['pose'] = pose
		

	def addObject(self, name, pose, vis):

		if not name in self.objects:
			self.objects[name] = {}
			self.objects[name]['id'] = self.nid
			self.nid += 1

		self.setObjectPose(name, pose)

		self.intManager.addObject(name, pose, vis)

		vis.ns = self.ns
		vis.id = self.objects[name]['id']
		self.objects[name]['vis'] = vis

		#ma = MarkerArray()
		#ma.markers = [vis]
		#self.visPub.publish(ma)


	def attachTo(self, ao):
		name = ao.object
		to = ao.frame
		if not name in self.objects:
			pass
		
		try:
			(trans, rot) = self.tfLt.lookupTransform(to, name, rospy.Time(0))
			self.objects[name]['pose']['translation'] = trans
			self.objects[name]['pose']['orientation'] = rot
			self.objects[name]['pose']['frame_id'] = to
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass 

	def updateTfs(self):
		for n, o in self.objects.iteritems():
			#self.tfBr.sendTransform(o['pose']['translation'], o['pose']['orientation'], rospy.Time.now(), n, o['pose']['frame_id'])
			self.percPub.publish(ObjectDetection(
	            name = n,
	            type = 6,
	            width = 0.015,
	            height = 0.068,
	            depth = 0.29,
	            pose = PoseStamped(
	                header = Header(
	                    stamp = rospy.Time.now(),
	                    frame_id = 'odom_combined'
	                ),
	                pose = makeMsgPose(o['pose'])
	            )
	        )
)


	def saveScene(self, msg):
		resolved = resolvePath(msg.data)
		stream = file(resolved, 'w')
		dump(self.objects, stream)


	def loadScene(self, msg):
		resolved = resolvePath(msg.data)
		stream = file(resolved, 'r')
		temp = load(stream)

		ma = MarkerArray()
		for x, y in self.objects.iteritems():
			m = Marker()
			m.ns = self.ns
			m.id = y['id']
			m.action = Marker.DELETE
			ma.markers.append(m)

		self.visPub.publish(ma)
		self.objects.clear()

		for o, n in temp.iteritems():
			self.addObject(o, n['pose'], n['vis'])

		#print(temp)		


if __name__ == '__main__':
	rospy.init_node('suturo_obstacle_scene')
	scene = ObstacleScene('suturo_obstacle_scene', 'base_link')

	rate = rospy.Rate(25)
	while not rospy.is_shutdown():
		scene.updateTfs()

		rate.sleep()