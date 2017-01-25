#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include <visualization_msgs/Marker.h>

#include <rviz/properties/string_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/property_tree_model.h>

class QLineEdit;
class QVBoxLayout;

namespace rviz {
	class ColorProperty;
	class FloatProperty;
	class RosTopicProperty;
	class TfFrameProperty;
}

namespace SuturoSim {
	
class SuturoSimPanel : public rviz::Panel {
	Q_OBJECT
public:
	SuturoSimPanel(QWidget* parent = 0);

	virtual void load(const rviz::Config& config);
	virtual void save(rviz::Config config) const;

	virtual void onInitialize();

public Q_SLOTS:
	
	void setCmdTopic(const QString& topic);
	void setSaveTopic(const QString& topic);
	void setLoadTopic(const QString& topic);
	void createObject();
	void loadScene();
	void saveScene();
	void updateVisMarker();

protected Q_SLOTS:

	void updateTopic();

protected:
	QLineEdit* simTopicWidget;
	QString simTopic, saveTopic, loadTopic;

	QVBoxLayout* markerLayout;

	visualization_msgs::Marker visMarker;

	rviz::PropertyTreeModel* objPTree;

	rviz::Property rootProperty;
	rviz::StringProperty scenePathProperty;
	rviz::Property objRootProperty;

	rviz::Property topicsProperty;
	rviz::RosTopicProperty simTopicProperty;
	rviz::RosTopicProperty saveTopicProperty;
	rviz::RosTopicProperty loadTopicProperty;

	rviz::StringProperty objNameProperty;
	rviz::TfFrameProperty* objFrameProperty;
	rviz::ColorProperty objColorProperty;
	rviz::FloatProperty objAlphaProperty;
	rviz::EnumProperty objShapeProperty;
	rviz::VectorProperty objScaleProperty;
	rviz::StringProperty objMeshProperty;

	ros::Publisher simCmdPublisher;
	ros::Publisher simSavePublisher;
	ros::Publisher simLoadPublisher;

	ros::NodeHandle nh;
private:
	bool bSimCmdConnected, bSimSaveConnected, bSimLoadConnected;
};

}