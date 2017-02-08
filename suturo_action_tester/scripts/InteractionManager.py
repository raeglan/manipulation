#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *

def makeMsgPose(poseDict):
	posT = poseDict['translation']
	rotT = poseDict['orientation']
	p = Pose()
	p.position.x = posT[0]
	p.position.y = posT[1]
	p.position.z = posT[2]

	p.orientation.x = rotT[0]
	p.orientation.y = rotT[1]
	p.orientation.z = rotT[2]
	p.orientation.w = rotT[3]

	return p

def makeDictPose(pose, frame):
	p = pose.position
	o = pose.orientation
	return {'frame_id': frame, 'translation': (p.x, p.y, p.z), 'orientation': (o.x, o.y, o.z, o.w)}

def make6DOFGimbal(intMarker):
	pitch = InteractiveMarkerControl()
	pitch.orientation.x = 1
	pitch.orientation.y = 0
	pitch.orientation.z = 0
	pitch.orientation.w = 1
	pitch.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
	intMarker.controls.append(pitch)

	yaw = InteractiveMarkerControl()
	yaw.orientation.x = 0
	yaw.orientation.y = 1
	yaw.orientation.z = 0
	yaw.orientation.w = 1
	yaw.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
	intMarker.controls.append(yaw)

	roll = InteractiveMarkerControl()
	roll.orientation.x = 0
	roll.orientation.y = 0
	roll.orientation.z = 1
	roll.orientation.w = 1
	roll.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
	intMarker.controls.append(roll)

	transX = InteractiveMarkerControl()
	transX.orientation.x = 1
	transX.orientation.y = 0
	transX.orientation.z = 0
	transX.orientation.w = 1
	transX.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
	intMarker.controls.append(transX)

	transY = InteractiveMarkerControl()
	transY.orientation.x = 0
	transY.orientation.y = 1
	transY.orientation.z = 0
	transY.orientation.w = 1
	transY.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
	intMarker.controls.append(transY)

	transZ = InteractiveMarkerControl()
	transZ.orientation.x = 0
	transZ.orientation.y = 0
	transZ.orientation.z = 1
	transZ.orientation.w = 1
	transZ.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
	intMarker.controls.append(transZ)

def activateMarker(intMarker, active):
	if active:
		intMarker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_3D
		intMarker.controls[1].interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
		intMarker.controls[2].interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
		intMarker.controls[3].interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
		intMarker.controls[4].interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		intMarker.controls[5].interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		intMarker.controls[6].interaction_mode = InteractiveMarkerControl.MOVE_AXIS
	else:
		intMarker.controls[0].interaction_mode = InteractiveMarkerControl.BUTTON
		intMarker.controls[1].interaction_mode = InteractiveMarkerControl.NONE
		intMarker.controls[2].interaction_mode = InteractiveMarkerControl.NONE
		intMarker.controls[3].interaction_mode = InteractiveMarkerControl.NONE
		intMarker.controls[4].interaction_mode = InteractiveMarkerControl.NONE
		intMarker.controls[5].interaction_mode = InteractiveMarkerControl.NONE
		intMarker.controls[6].interaction_mode = InteractiveMarkerControl.NONE


class InteractionManager(object):
	def __init__(self, scene):
		self.selected = None
		self.controllers = {}
		self.server = InteractiveMarkerServer('object_manipulation')
		self.scene = scene
		self.menuHandler = MenuHandler()
		self.menuHandler.insert('Delete', callback=self.processFeedback)


	def addObject(self, name, pose, vis):
		if self.selected == name:
			self.selectObject(None, None, None, None, False)

		intMarker = InteractiveMarker()
		intMarker.name = name
		intMarker.header.frame_id = pose['frame_id']
		intMarker.header.stamp = rospy.Time.now()
		intMarker.pose = makeMsgPose(pose)
		intMarker.scale = 1.0
		control = InteractiveMarkerControl()
		control.interaction_mode = InteractiveMarkerControl.MOVE_3D
		control.always_visible = True
		control.orientation.w = 1.0
		control.markers.append(vis)
		control.description = name
		intMarker.controls.append(control)
		make6DOFGimbal(intMarker)
		activateMarker(intMarker, False)
		menuCtrl = InteractiveMarkerControl()
		menuCtrl.interaction_mode = InteractiveMarkerControl.MENU
		menuCtrl.description = 'Options'
		menuCtrl.name = 'menu_ctrl'
		intMarker.controls.append(menuCtrl)

		self.server.insert(intMarker, self.processFeedback)
		self.menuHandler.apply(self.server, intMarker.name)

		self.server.applyChanges()

		self.controllers[name] = intMarker

		print('Added new marker: ' + name)


	def selectObject(self, name, pose, fId, _apply=True):
		if self.selected != None and name != self.selected:
			activateMarker(self.controllers[self.selected], False)
			self.server.insert(self.controllers[self.selected], self.processFeedback)

		if name in self.controllers:
			self.selected = name
			activateMarker(self.controllers[self.selected], True)
			self.server.insert(self.controllers[self.selected], self.processFeedback)
		else:
			self.selected = None

		if _apply:
			self.server.applyChanges()

	def removeObject(self, name):
		if self.selected == name:
			self.selected = None

		del self.controllers[name]
		if not self.server.erase(name):
			print('NO MARKER WITH NAME: ' + str(name) + ' TO DELETE!')
		self.server.applyChanges()


	def processFeedback(self, feedback):

		if feedback.marker_name != self.selected:
			self.selectObject(feedback.marker_name, feedback.pose, feedback.header.frame_id)
		else:
			if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
				self.scene.setObjectPose(feedback.marker_name, makeDictPose(feedback.pose, feedback.header.frame_id))
				self.controllers[self.selected].pose = feedback.pose
				self.controllers[self.selected].header.frame_id = feedback.header.frame_id
		
		if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
			print("Received menu feedback from: " + feedback.marker_name + ' id: ' + str(feedback.menu_entry_id))
			if feedback.menu_entry_id == 1:
				self.scene.removeObject(feedback.marker_name)
