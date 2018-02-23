#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <static_object_position_tracking/ObjectsPositionsMap.h>
#include <static_object_position_tracking/ObjectPositionID.h>

class tf_publisher{
    public:
        void init(){
                _nh.getParam("/", _parameters);
                _camera_type = static_cast<std::string>(_parameters["camera_type"]);
                _object_update_sub = _nh.subscribe<static_object_position_tracking::ObjectsPositionsMap>("/visual/cam_frame_obj_pos_vector", 50, &tf_publisher::object_callback, this);
                _number_of_objects = 0;

                ros::AsyncSpinner my_spinner(4);
                my_spinner.start();
            }

        void object_callback(const static_object_position_tracking::ObjectsPositionsMapConstPtr& object_msgs){
                _number_of_objects = object_msgs->objects_positions_id.size();
                for(size_t i = 0; i < _number_of_objects; i++){

                        //ROS_WARN_STREAM("The choosen point in camera frame is: X = " << point.point.x << ", Y = " << point.point.y << ", and Z = " << point.point.z);

                        publish_point_frame(object_msgs->objects_positions_id[i].object_position,
                                            "/visual/object_base_frame_" + std::to_string(object_msgs->objects_positions_id[i].object_id));
                    }
            }

        void publish_point_frame(geometry_msgs::PointStamped point, std::string child_frame_id){
                // publish transform
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(point.point.x,
                                                 point.point.y,
                                                 point.point.z));

                tf::Quaternion q(0.,
                                 0.,
                                 0.,
                                 1.0);

                // Handle different coordinate systems (Arena vs. rviz)
                transform.setRotation(q);
                ros::Time timestamp(ros::Time::now());
                if(_camera_type == "kinect_2")
                    tf_pub.sendTransform(tf::StampedTransform(transform, timestamp, "kinect2_link", child_frame_id));
                else
                    tf_pub.sendTransform(tf::StampedTransform(transform, timestamp, "camera_depth_optical_frame", child_frame_id));
            }

        int get_number_of_objects(){
                return _number_of_objects;
            }

    private:
        ros::NodeHandle _nh;
        XmlRpc::XmlRpcValue _parameters;
        ros::Subscriber _object_update_sub;
        tf::TransformBroadcaster tf_pub;
        tf::StampedTransform stamped_transform;
        geometry_msgs::TransformStamped msg;
        std::string _camera_type;
        int _number_of_objects;
    };



int main(int argc, char **argv)
    {
        usleep(15e6);

        ros::init(argc, argv, "test_octomap_node");
        ros::NodeHandle node;

        ros::Publisher _objects_positions_pub = node.advertise<static_object_position_tracking::ObjectsPositionsMap>("/visual/obj_pos_vector", 1000);
        tf::StampedTransform transform;
        tf::TransformListener listener;
        geometry_msgs::TransformStamped msg;
        std::string parent_frame = "/base";
        std::string child_frame_id;
        static_object_position_tracking::ObjectsPositionsMap obj_pos_msg_;

        tf_publisher my_tf_publisher;
        my_tf_publisher.init();

        ros::Rate my_rate(20);

        while (ros::ok()) {
                ros::spinOnce();

                geometry_msgs::PointStamped point;
                static_object_position_tracking::ObjectPositionID current_object;
                for(int i = 0; i < my_tf_publisher.get_number_of_objects(); i++){
                        try{
                            child_frame_id = "/visual/object_base_frame_" + std::to_string(i);
                            listener.lookupTransform(parent_frame, child_frame_id, ros::Time(0), transform);
                            tf::transformStampedTFToMsg(transform, msg);

                            point.header.stamp = ros::Time::now();
                            point.header.frame_id = parent_frame;
                            point.point.x = msg.transform.translation.x;
                            point.point.y = msg.transform.translation.y;
                            point.point.z = msg.transform.translation.z;
                            point.header.seq = i;
                            current_object.object_id = i;
                            current_object.object_position = point;
                            obj_pos_msg_.objects_positions_id.push_back(current_object);
                            //                ROS_ERROR_STREAM("Object " << i << " : " <<
                            //                                                point.point.x << " " <<
                            //                                                point.point.y << " " <<
                            //                                                point.point.z);
                        }
                        catch (tf::TransformException &ex) {
                                ROS_ERROR("%s",ex.what());
                                ros::Duration(1.0).sleep();
                            }
                    }
                _objects_positions_pub.publish(obj_pos_msg_);
                obj_pos_msg_.objects_positions_id.clear();
                my_rate.sleep();
            }
        ros::waitForShutdown();
        return 0;
    }
