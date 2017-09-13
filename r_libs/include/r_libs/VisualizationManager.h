#pragma once
#include <ros/ros.h>
#include <map>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>
#include "eigen3/Eigen/Eigen"

using namespace Eigen;
using namespace std;


/**
 * @brief      Class allowing a visualization supporting a frame like update style.
 * 
 * The visualization manager manages namespaces and marker Ids automatically. It can be used to draw in frames, e.g. start a frame, draw markers, end frame. The markers of the previous frame are automatically deleted.
 */
class VisualizationManager
{
public:

    /**
     * @brief      Add a namespace.
     *
     * @param[in]  ns    The Id of the namespace
     * @param[in]  name  The readable name of the namespace
     */
    void addNamespace(int ns, string name);


    /**
     * @brief      Clear all markers within a namespace.
     *
     * @param[in]  ns         Id of the namespace
     * @param      publisher  Publisher used to publish the delete command
     */
    void clearMarkerNS(int ns, ros::Publisher* publisher);

    
    /**
     * @brief      Delete all markers managed by this manager.
     *
     * @param      publisher  Publisher used to publish the delete command
     */
    void clearAllMarkers(ros::Publisher* publisher);

    /**
     * @brief      Begin a new draw cycle.
     */
    void beginNewDrawCycle();
    

    /**
     * @brief      End the current draw cycle and append delete commands to marker array.
     *
     * @param      vis   Marker array to append delete commands to
     */
    void endDrawCycle(vector<visualization_msgs::Marker>& vis);

    /**
     * @brief      Creates a trail marker from an array of points.
     *
     * @param[in]  ns        Namespace to add the marker to
     * @param[in]  points    Points used in the trail
     * @param[in]  width     Width of the trail
     * @param[in]  r         Trail color: Red
     * @param[in]  g         Trail color: Green
     * @param[in]  b         Trail color: Blue
     * @param[in]  a         Trail color: Alpha
     * @param[in]  frame_id  Reference frame for the marker
     *
     * @return     A trail marker
     */
    visualization_msgs::Marker trailMarker(int ns, 
                                            const vector<Vector3d>& points, 
                                            double width = 0.1,
                                            float r = 0.f, 
                                            float g = 1.f, 
                                            float b = 0.f, 
                                            float a = 1.f, 
                                            string frame_id = "odom_combined");

    /**
     * @brief      Creates a marker with an arbitrary shape.
     *
     * @param[in]  ns        Namespace to add the marker to
     * @param[in]  pose      Pose of the marker
     * @param[in]  shape     Shape as specified by visualization_msgs::Marker
     * @param[in]  size      Scaling vector of the marker
     * @param[in]  r         Marker color: Red
     * @param[in]  g         Marker color: Green
     * @param[in]  b         Marker color: Blue
     * @param[in]  a         Marker color: Alpha
     * @param[in]  frame_id  Reference frame for the marker
     *
     * @return     Shape marker
     */
    visualization_msgs::Marker shapeMarker(int ns, 
                                            Affine3d pose, 
                                            int shape, 
                                            Vector3d size,
                                            float r = 0.f, 
                                            float g = 1.f, 
                                            float b = 0.f, 
                                            float a = 1.f, 
                                            string frame_id = "odom_combined");

    /**
     * @brief      Creates a spherical marker
     *
     * @param[in]  ns        Namespace to add the marker to
     * @param[in]  center    Center point of the marker
     * @param[in]  radius    Radius of the sphere
     * @param[in]  r         Marker color: Red
     * @param[in]  g         Marker color: Green
     * @param[in]  b         Marker color: Blue
     * @param[in]  a         Marker color: Alpha
     * @param[in]  frame_id  Reference frame for the marker
     *
     * @return     Sphere marker
     */
    visualization_msgs::Marker sphereMarker(int ns, 
                                            Vector3d center, 
                                            double radius, 
                                            float r = 1.f, 
                                            float g = 1.f, 
                                            float b = 1.f, 
                                            float a = 1.f, 
                                            string frame_id = "odom_combined");

    /**
     * @brief      Creates an arrow marker visualizing a vector as direction.
     *
     * @param[in]  ns         Namespace to add the marker to
     * @param[in]  pos        Position of the base of the arrow
     * @param[in]  vector     Vector to visualize
     * @param[in]  r          Marker color: Red
     * @param[in]  g          Marker color: Green
     * @param[in]  b          Marker color: Blue
     * @param[in]  a          Marker color: Alpha
     * @param[in]  width      Width of the arrow
     * @param[in]  headWidth  Width of the arrowhead
     * @param[in]  frame_id   Reference frame for the marker
     *
     * @return     Arrow marker 
     */
    visualization_msgs::Marker vectorMarker(int ns, 
                                            Vector3d pos, 
                                            Vector3d vector, 
                                            float r = 1.f, 
                                            float g = 1.f, 
                                            float b = 1.f, 
                                            float a = 1.f, 
                                            double width = 0.01, 
                                            double headWidth = 0.02, 
                                            string frame_id = "odom_combined");

    /**
     * @brief      Creates an arrow marker connecting two points.
     *
     * @param[in]  ns         Namespace to add the marker to
     * @param[in]  start      Starting point
     * @param[in]  end        End point
     * @param[in]  r          Marker color: Red
     * @param[in]  g          Marker color: Green
     * @param[in]  b          Marker color: Blue
     * @param[in]  a          Marker color: Alpha
     * @param[in]  width      Width of the arrow
     * @param[in]  headWidth  Width of the arrowhead
     * @param[in]  frame_id   Reference frame for the marker
     *
     * @return     Arrow marker connecting start and end point
     */
    visualization_msgs::Marker arrowMarker(int ns, 
                                            Vector3d start, 
                                            Vector3d end, 
                                            float r = 1.f, 
                                            float g = 1.f, 
                                            float b = 1.f, 
                                            float a = 1.f, 
                                            double width = 0.01, 
                                            double headWidth = 0.02, 
                                            string frame_id = "odom_combined");

    /**
     * @brief      Creates a text marker.
     *
     * @param[in]  ns        Namespace to add the marker to
     * @param[in]  pos       Position of the text.
     * @param[in]  text      The text to display
     * @param[in]  r         Marker color: Red
     * @param[in]  g         Marker color: Green
     * @param[in]  b         Marker color: Blue
     * @param[in]  a         Marker color: Alpha
     * @param[in]  height    Height of the text
     * @param[in]  frame_id   Reference frame for the marker
     *
     * @return     Text marker
     */
    visualization_msgs::Marker textMarker(int ns, 
                                            Vector3d pos, 
                                            string text, 
                                            float r = 1.f, 
                                            float g = 1.f, 
                                            float b = 1.f, 
                                            float a = 1.f, 
                                            double height = 0.015, 
                                            string frame_id = "odom_combined");

    /**
     * @brief      Creates a vector with a label
     *
     * @param      array      Marker array to add the markers to
     * @param[in]  ns         Namespace to add the marker to
     * @param[in]  pos        Position of the base of the arrow
     * @param[in]  vector     Vector to visualize
     * @param[in]  text       The text
     * @param[in]  r          Marker color: Red
     * @param[in]  g          Marker color: Green
     * @param[in]  b          Marker color: Blue
     * @param[in]  a          Marker color: Alpha
     * @param[in]  frame_id   Reference frame for the marker
     */
    void annotatedVector(vector<visualization_msgs::Marker> &array,
                                            int ns, 
                                            Vector3d pos,
                                            Vector3d vector, 
                                            string text, 
                                            float r = 1.f, 
                                            float g = 1.f, 
                                            float b = 1.f, 
                                            float a = 1.f, 
                                            string frame_id = "odom_combined");
    /**
     * @brief      Creates a point with a label
     *
     * @param      array      Marker array to add the markers to
     * @param[in]  ns         Namespace to add the marker to
     * @param[in]  pos        Position of the point
     * @param[in]  text       Text of the label
     * @param[in]  r          Marker color: Red
     * @param[in]  g          Marker color: Green
     * @param[in]  b          Marker color: Blue
     * @param[in]  a          Marker color: Alpha
     * @param[in]  frame_id   Reference frame for the marker
     */
    void annotatedPoint(vector<visualization_msgs::Marker> &array,
                                            int ns, 
                                            Vector3d pos,
                                            string text, 
                                            float r = 1.f, 
                                            float g = 1.f, 
                                            float b = 1.f, 
                                            float a = 1.f, 
                                            string frame_id = "odom_combined");

    /**
     * @brief      Creates a marker visualizing a pose as coordinate axes.
     *
     * @param      array        Marker array to add the markers to
     * @param[in]  ns           Namespace to add the marker to
     * @param[in]  pose         Pose to visualize
     * @param[in]  arrowLength  The length of the axes arrows
     * @param[in]  a            Maker Color: Alpha
     * @param[in]  frame_id     Reference frame for the marker
     */
    void poseMarker(vector<visualization_msgs::Marker> &array, 
                    int ns, 
                    Affine3d pose, 
                    double arrowLength = 0.1, 
                    float a = 1.f, 
                    string frame_id = "odom_combined");

    /**
     * @brief      Creates a mesh marker using a URI.
     *
     * @param[in]  ns        Namespace to add the marker to
     * @param[in]  pose      Pose for mesh marker
     * @param[in]  scale     Scale vector for the marker
     * @param[in]  resource  Resource URI
     * @param[in]  frame_id  Reference frame for the marker
     * @param[in]  color     Marker color as vector
     *
     * @return     Returns a mesh marker
     */
    visualization_msgs::Marker meshMarker(int ns, 
                                            Affine3d pose,
                                            Vector3d scale, 
                                            string resource, 
                                            string frame_id = "odom_combined", 
                                            Vector4f color = Vector4f::Zero()) {
        return meshMarker(ns, pose, scale, resource, frame_id, color[0], color[1], color[2], color[3]);
    }

    /**
     * @brief      { function_description }
     *
     * @param[in]  ns        Namespace to add the marker to
     * @param[in]  pose      Pose for mesh marker
     * @param[in]  scale     Scale vector for the marker
     * @param[in]  resource  Resource URI
     * @param[in]  frame_id  Reference frame for the marker
     * @param[in]  r         Marker color: Red
     * @param[in]  g         Marker color: Green
     * @param[in]  b         Marker color: Blue
     * @param[in]  a         Marker color: Alpha
     *
     * @return     Returns a mesh marker
     */
    visualization_msgs::Marker meshMarker(int ns, 
                                            Affine3d pose,
                                            Vector3d scale, 
                                            string resource, 
                                            string frame_id = "odom_combined", 
                                            float r = 0, 
                                            float g = 0, 
                                            float b = 0, 
                                            float a = 0);

    /**
     * @brief      Manually get the next available Id for a namespace.
     *
     * @param[in]  ns    Id of the Namespace to take the Id from
     *
     * @return     Returns next available Id for a namespace
     */
    int consumeId(int ns);


    /**
     * @brief      Gets the readable name of a namespace
     *
     * @param[in]  ns    Id of the Namespace
     *
     * @return     Name of the namespace
     */
    string getNamespace(int ns);

private:

    /**
     * @brief      Internal function to convert vectors to point messages.
     *
     * @param[in]  vec   Vector to convert
     *
     * @return     Point message
     */
    geometry_msgs::Point vec2Point(Vector3d vec);
    
    /**
     * @brief      Internal function to convert Quaternions to Quaternion messages.
     *
     * @param[in]  quat  Quaternion to convert
     *
     * @return     Quaternion message
     */
    geometry_msgs::Quaternion quat2QuatMsg(Quaterniond quat);
    
    /** Mapping of Ids to namespace names */
    map<int, string> namespaces;

    /** Mapping of namespace Ids to next available Id */
    map<int,int> ids;

    /** Highest Ids of the last draw cycle */
    map<int,int> lastIds;
};