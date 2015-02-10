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
#ifndef DATAMANAGER_HPP
#define DATAMANAGER_HPP

#include <actionlib/server/simple_action_server.h>
#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "action_specification.hpp"
#include "database_table_base.hpp"

#include "item.hpp"
#include "person.hpp"
#include "script.hpp"
#include "waypoint.hpp"
#include "resource.hpp"

#include "rose_common/common.hpp"

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"

class Datamanager
{
  public:
	Datamanager( std::string name, ros::NodeHandle n );
	~Datamanager();

  private:
  	template <typename T>
	bool get( T& item_with_id, T& item )
	{
		std::vector< boost::shared_ptr<DatabaseTableBase<T> > > db_items;
		std::string where_clause(T::IDENTIFIER + "_id=" + item_with_id.getIdWithoutIdentifier());
		database_.getList(db_items, where_clause);

		if ( db_items.size() != 1 )
		{
			ROS_ERROR(" Datamanager::get : Error retrieving items");
			if ( db_items.size() > 1 )
			 	ROS_ERROR(" Datamanager::get : Retrieved more than 1 items");
			if ( db_items.size() == 0 )
			 	ROS_ERROR(" Datamanager::get : Retrieved no items");		 
			return false;
		}

		item = db_items.at(0)->serialized_.data();
		item.set_id(db_items.at(0)->getStringId());

		ROS_INFO("Datamanager::get::end");
		return true;
	}

	template <typename T>
	bool getAll( T& item, std::vector<T>& items )
	{
		std::vector< boost::shared_ptr<DatabaseTableBase<T> > > db_items;
		if (!database_.getList(db_items))
		{
			ROS_ERROR("Failed to get list of items");
			return false;
		}
		else
			ROS_INFO("Retrieved %d item(s) for %s", (int)db_items.size(), T::IDENTIFIER.c_str());

		// Set the right IDs
		for ( auto it = db_items.begin() ; it != db_items.end() ; it++ )
		{
			T new_item( (*it)->serialized_.data() );
			new_item.set_id( (*it)->getStringId() );
			items.push_back( new_item );
		}

		return true;
	}

	template <typename T> 
	bool store( T& item, T& stored_item )
	{
		DatabaseTableBase<T> db_item;
		stored_item = item;
		ROS_INFO("stored item id = %s", stored_item.get_id().c_str());

		db_item.serialized_.data() = stored_item;

		// String item-id, if this is set
		if ( item.hasId() )
		{
			ROS_INFO("Item id set");
			db_item.id_.data() = rose_conversions::stringToInt(stored_item.getIdWithoutIdentifier());

			if (!database_.saveToDatabase(&db_item.serialized_)) 
			{
				ROS_ERROR("Item insertion failed");
				return false;
			}
				else 
				{
				ROS_INFO("Item inserted into database");
				return true;
			}
		}	
		// If no ID is set, one will be created by the database
		else 
		{
			ROS_INFO("Item id not set");
			if (!database_.insertIntoDatabase(&db_item)) 
			{
				ROS_ERROR("Item not stored");
				return false;
			}
			else 
			{
				ROS_INFO("Item stored into database");
				stored_item.set_id(db_item.getStringId());
				return true;
			}
		}

	}

	template <typename T>
	bool del ( T& item, T&deleted_item )
	{
		std::vector< boost::shared_ptr<DatabaseTableBase<T> > > db_items;
		std::string where_clause(T::IDENTIFIER + "_id=" + item.getIdWithoutIdentifier());
		database_.getList(db_items, where_clause);

		if (!database_.deleteFromDatabase(db_items.at(0).get())) 
	    {
	    	ROS_ERROR("Item deletion failed");
	    	return false;
	    }
	  	else 
	  	{
	    	ROS_INFO("Item deleted from database");
	    	return true;
	    }
	}

	ros::NodeHandle 	n_;
	std::string			name_;

	ros::ServiceServer 	get_item_service_;
	ros::ServiceServer 	get_person_service_;
	ros::ServiceServer 	get_script_service_;
	ros::ServiceServer 	get_waypoint_service_;
	ros::ServiceServer 	get_resource_service_;

	ros::ServiceServer 	get_all_items_service_;
	ros::ServiceServer 	get_all_persons_service_;
	ros::ServiceServer 	get_all_scripts_service_;
	ros::ServiceServer 	get_all_waypoints_service_;
	ros::ServiceServer 	get_all_resources_service_;

	ros::ServiceServer  store_item_service_;
	ros::ServiceServer  store_person_service_;
	ros::ServiceServer  store_script_service_;
	ros::ServiceServer  store_waypoint_service_;

	ros::ServiceServer 	del_item_service_;
	ros::ServiceServer 	del_person_service_;
	ros::ServiceServer 	del_script_service_;
	ros::ServiceServer 	del_waypoint_service_;

	database_interface::PostgresqlDatabase database_;
}; 

#endif // DATAMANAGER_HPP