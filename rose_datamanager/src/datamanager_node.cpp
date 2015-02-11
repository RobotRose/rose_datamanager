/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/29
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "rose_datamanager/datamanager_node.hpp"

int main( int argc, char **argv )
{

// Set up ROS.
  ros::init(argc, argv, "datamanager");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("topic", topic, string("datamanager"));

  // Create a new ScriptInteractionNode object.
  Datamanager* datamanager = new Datamanager("datamanager", n);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  bool stop = false;

  while (n.ok() && !stop)
  {
    ros::spinOnce();
    r.sleep();

    if(rose_conversions::kbhit())
    {
      uint c = getchar();
      ROS_DEBUG_NAMED("wheel_controller", "Key pressed: %c", (char)c);
      switch(c)
      {
    //     case 'x':
    //       stop = true;
    //       break;
        // case 'd':
        //   datamanager->addToDatabase();
        //   break;
        // case 'f':
        //   datamanager->retrieveFromDatabase();
        //   break;
      }
    }
  }

  delete datamanager;

  return 0;
}