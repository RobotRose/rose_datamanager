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
#include "datamanager.hpp"

Datamanager::Datamanager( std::string name, ros::NodeHandle n )
	: name_ ( name )
	, n_ ( n )
	, database_ ("localhost", "5432", "rose", "roseindezorg", "rose_object_database")
{
	ROS_INFO("Datamanager::Datamanager::begin");
	
	// Get one item
	get_item_service_			= n.advertiseService("/datamanager/get_" + Item::IDENTIFIER, 		&Datamanager::get<Item>, this);
	get_person_service_			= n.advertiseService("/datamanager/get_" + Person::IDENTIFIER, 		&Datamanager::get<Person>, this);
	get_script_service_			= n.advertiseService("/datamanager/get_" + Script::IDENTIFIER, 		&Datamanager::get<Script>, this);
	get_waypoint_service_		= n.advertiseService("/datamanager/get_" + Waypoint::IDENTIFIER, 	&Datamanager::get<Waypoint>, this);
	get_resource_service_		= n.advertiseService("/datamanager/get_" + Resource::IDENTIFIER, 	&Datamanager::get<Resource>, this);

	// Get all items
	get_all_items_service_ 		= n.advertiseService("/datamanager/get_all_" + Item::IDENTIFIER + "s",     &Datamanager::getAll<Item>, this);
	get_all_persons_service_ 	= n.advertiseService("/datamanager/get_all_" + Person::IDENTIFIER + "s",   &Datamanager::getAll<Person>, this);
	get_all_scripts_service_ 	= n.advertiseService("/datamanager/get_all_" + Script::IDENTIFIER + "s",   &Datamanager::getAll<Script>, this);
	get_all_waypoints_service_ 	= n.advertiseService("/datamanager/get_all_" + Waypoint::IDENTIFIER + "s", &Datamanager::getAll<Waypoint>, this);
	get_all_resources_service_ 	= n.advertiseService("/datamanager/get_all_" + Resource::IDENTIFIER + "s", &Datamanager::getAll<Resource>, this);

	// Store item
	store_item_service_			= n.advertiseService("/datamanager/store_" + Item::IDENTIFIER, 		&Datamanager::store<Item>, this);
	store_person_service_		= n.advertiseService("/datamanager/store_" + Person::IDENTIFIER, 	&Datamanager::store<Person>, this);
	store_script_service_		= n.advertiseService("/datamanager/store_" + Script::IDENTIFIER, 	&Datamanager::store<Script>, this);
	store_waypoint_service_		= n.advertiseService("/datamanager/store_" + Waypoint::IDENTIFIER, 	&Datamanager::store<Waypoint>, this);

	// Delete item
	del_item_service_			= n.advertiseService("/datamanager/del_" + Item::IDENTIFIER, 		&Datamanager::del<Item>, this);
	del_person_service_			= n.advertiseService("/datamanager/del_" + Person::IDENTIFIER, 		&Datamanager::del<Person>, this);
	del_script_service_			= n.advertiseService("/datamanager/del_" + Script::IDENTIFIER, 		&Datamanager::del<Script>, this);
	del_waypoint_service_		= n.advertiseService("/datamanager/del_" + Waypoint::IDENTIFIER, 	&Datamanager::del<Waypoint>, this);	

	// Connect to the database
	if (!database_.isConnected())
		ROS_ERROR("Database failed to connect");
	else
		ROS_INFO("Database connected successfully");

	ROS_INFO("Datamanager::Datamanager::end");
}

Datamanager::~Datamanager()
{

}
