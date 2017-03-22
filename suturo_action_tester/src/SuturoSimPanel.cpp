#include <QLineEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QColor>
#include <QStyleOptionViewItem>
#include <QPushButton>

#include <std_msgs/String.h>

#include <rviz/properties/property_tree_widget.h>
#include <rviz/visualization_manager.h>
#include <OgreVector3.h>

#include "suturo_action_tester/SuturoSimPanel.h"
#include <suturo_action_tester/SuturoCreateObject.h>


namespace SuturoSim {

SuturoSimPanel::SuturoSimPanel(QWidget* pParent) 
: rviz::Panel(pParent)
, rootProperty("PanelRootProperty")
, topicsProperty("Topics", QVariant(), QString(), &rootProperty)
, simTopicProperty("Simulation Topic", "/suturo/create_object", "suturo_action_tester/SuturoCreateObject", "Topic on which to connect to the simulation", &topicsProperty, SLOT(updateTopic()), this)
, saveTopicProperty("Simulation Save Topic", "/suturo/save_scene", "std_msgs/String", "Topic on which to connect to the simulation", &topicsProperty, SLOT(updateTopic()), this)
, loadTopicProperty("Simulation Load Topic", "/suturo/load_scene", "std_msgs/String", "Topic on which to connect to the simulation", &topicsProperty, SLOT(updateTopic()), this)

, objRootProperty("Object", QVariant(), QString(), &rootProperty)
, objNameProperty("Object Name", "object1", "Name of the object to create", &objRootProperty, SLOT(updateVisMarker()), this)
, objColorProperty("Color", QColor(204,51,204), "Color of the marker", &objRootProperty, SLOT(updateVisMarker()), this)
, objAlphaProperty("Alpha", 1.0f, "Alpha of the marker", &objRootProperty, SLOT(updateVisMarker()), this)
, objShapeProperty("Shape", "Cube", "Shape of the new object", &objRootProperty, SLOT(updateVisMarker()), this)
, objScaleProperty("Scale", Ogre::Vector3(1,1,1), "Scale of the object", &objRootProperty, SLOT(updateVisMarker()), this)
, objMeshProperty("Mesh", "", "Mesh for the object. First part of path is ros-package", &objRootProperty, SLOT(updateVisMarker()), this)
, objFrameProperty(0)
, scenePathProperty("Scene Path", "", "Path to save the scene to and load it from", &rootProperty)
, bSimCmdConnected(false)
, bSimSaveConnected(false) 
, bSimLoadConnected(false)
{
	QVBoxLayout* masterLayout = new QVBoxLayout;

	objAlphaProperty.setMin(0.f);
	objAlphaProperty.setMax(1.f);

	objShapeProperty.addOption("Cube", visualization_msgs::Marker::CUBE);
	objShapeProperty.addOption("Sphere", visualization_msgs::Marker::SPHERE);
	objShapeProperty.addOption("Cylinder", visualization_msgs::Marker::CYLINDER);
	objShapeProperty.addOption("Mesh", visualization_msgs::Marker::MESH_RESOURCE);

	objPTree = new rviz::PropertyTreeModel(&rootProperty, this);

	QVBoxLayout* markerLayout = new QVBoxLayout;
 	rviz::PropertyTreeWidget* ptWidget = new rviz::PropertyTreeWidget;
 	ptWidget->setModel(rootProperty.getModel());
 	markerLayout->addWidget(ptWidget);

 	QPushButton* btnCreateObject = new QPushButton("Create Object", this);
 	markerLayout->addWidget(btnCreateObject);

 	QHBoxLayout* sceneLayout = new QHBoxLayout;
 	QPushButton* btnLoadObject = new QPushButton("Load Scene", this);
 	sceneLayout->addWidget(btnLoadObject);

 	QPushButton* btnSaveObject = new QPushButton("Save Scene", this);
 	sceneLayout->addWidget(btnSaveObject);

 	markerLayout->addLayout(sceneLayout);
	masterLayout->addLayout(markerLayout);
	setLayout(masterLayout);

	connect(btnCreateObject, SIGNAL(clicked()), this, SLOT(createObject()));
	connect(btnLoadObject, SIGNAL(clicked()), this, SLOT(loadScene()));
	connect(btnSaveObject, SIGNAL(clicked()), this, SLOT(saveScene()));
}

void SuturoSimPanel::onInitialize() {
	rviz::Panel::onInitialize();

	objFrameProperty = new rviz::TfFrameProperty("Frame", "odom_combined", "Reference frame for the object", &objRootProperty, vis_manager_->getFrameManager(), false, SLOT(updateVisMarker()), this);	
}

void SuturoSimPanel::updateTopic() {
	setCmdTopic(simTopicProperty.getString());
	setSaveTopic(saveTopicProperty.getString());
	setLoadTopic(loadTopicProperty.getString());
}

void SuturoSimPanel::updateVisMarker() {
	visMarker.header.frame_id = objNameProperty.getStdString();

	if (objShapeProperty.getOptionInt() == visualization_msgs::Marker::MESH_RESOURCE) {
		if (objMeshProperty.getHidden()) {
			objMeshProperty.show();
		}
		visMarker.mesh_resource = "package://" + objMeshProperty.getStdString();
	} else if (!objMeshProperty.getHidden()) {
		objMeshProperty.hide();
	}

	QColor color = objColorProperty.getColor();

	visMarker.action = visualization_msgs::Marker::ADD;
	visMarker.type = objShapeProperty.getOptionInt();

	visMarker.pose.orientation.x = 0;
	visMarker.pose.orientation.y = 0;
	visMarker.pose.orientation.z = 0;
	visMarker.pose.orientation.w = 1;

	Ogre::Vector3 scale = objScaleProperty.getVector();
	visMarker.scale.x = scale.x;
	visMarker.scale.y = scale.y;
	visMarker.scale.z = scale.z;

	visMarker.color.a = objAlphaProperty.getFloat();
	visMarker.color.r = color.redF();
	visMarker.color.g = color.greenF();
	visMarker.color.b = color.blueF();
	visMarker.mesh_use_embedded_materials = true;

	Q_EMIT configChanged();
}

void SuturoSimPanel::setCmdTopic(const QString& topic) {
	if (topic != simTopic) {
		simTopic = topic;
		if (simTopic == "") {
			simCmdPublisher.shutdown();
			bSimCmdConnected = false;
		} else {
			simCmdPublisher = nh.advertise<suturo_action_tester::SuturoCreateObject>(simTopic.toStdString(), 1);
			bSimCmdConnected = true;
		}
		Q_EMIT configChanged();		
	}
}

void SuturoSimPanel::setSaveTopic(const QString& topic) {
	if (topic != saveTopic) {
		saveTopic = topic;
		if (saveTopic == "") {
			simSavePublisher.shutdown();
			bSimSaveConnected = false;
		} else {
			simSavePublisher = nh.advertise<std_msgs::String>(saveTopic.toStdString(), 1);
			bSimSaveConnected = true;
		}
		Q_EMIT configChanged();
	}
}

void SuturoSimPanel::setLoadTopic(const QString& topic) {
	if (topic != loadTopic) {
		loadTopic = topic;
		if (loadTopic == "") {
			simLoadPublisher.shutdown();
			bSimLoadConnected = false;
		} else {
			simLoadPublisher = nh.advertise<std_msgs::String>(loadTopic.toStdString(), 1);
			bSimLoadConnected = true;
		}
		Q_EMIT configChanged();
	}


}

void SuturoSimPanel::createObject() {
	suturo_action_tester::SuturoCreateObject msg;
	msg.name = objNameProperty.getStdString();
	msg.refFrame = objFrameProperty->getStdString();
	msg.vis = visMarker;
	if (bSimCmdConnected)
		simCmdPublisher.publish(msg);	
}

void SuturoSimPanel::save(rviz::Config config) const {
	rviz::Panel::save(config);
	rootProperty.save(config);
}

void SuturoSimPanel::load(const rviz::Config& config) {
	rviz::Panel::load(config);
	rootProperty.load(config);
	updateVisMarker();
	updateTopic();
}

void SuturoSimPanel::loadScene() {
	if (bSimLoadConnected && scenePathProperty.getString() != "") {
		std_msgs::String msg;
		msg.data = scenePathProperty.getStdString();
		simLoadPublisher.publish(msg);
	}
}

void SuturoSimPanel::saveScene() {
	if (bSimSaveConnected && scenePathProperty.getString() != "") {
		std_msgs::String msg;
		msg.data = scenePathProperty.getStdString();
		simSavePublisher.publish(msg);
	}
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SuturoSim::SuturoSimPanel, rviz::Panel)