/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/24
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef ROSE_DATAMANAGER_API_HPP
#define ROSE_DATAMANAGER_API_HPP

#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <vector>

#include "item.hpp"
#include "waypoint.hpp"
#include "person.hpp"
#include "script.hpp"

class DatamanagerAPI
{
  public:
	DatamanagerAPI();
	~DatamanagerAPI();

	template <typename T>
	T get( std::string id )
	{
		// Set up client
		ros::ServiceClient client;
		client = n_.serviceClient<T>("/datamanager/get_" + T::IDENTIFIER);

		// Request/Response variables
		T item_with_id(id, "item-with-id");
		T item;

		if ( client.call(item_with_id, item) )
			ROS_INFO("Received item");
		else
			ROS_INFO("Could not receive item");	

		return item;
	}

	template <typename T>
	std::vector<T> getAll()
	{
		// Set up client
		ros::ServiceClient client;
		client = n_.serviceClient<T>("/datamanager/get_all_" + T::IDENTIFIER + "s");

		// Request/Response variables
		vector<T> items;
		T item;

		if ( client.call(item, items) )
			ROS_INFO("Received %d items", (int)items.size());
		else
			ROS_INFO("Could not receive items");	

		return items;
	}

	template <typename T>
	T store( T item )
	{
		// Set up client
		ros::ServiceClient client;
		client = n_.serviceClient<T>("/datamanager/store_" + T::IDENTIFIER );

		// Response variable
		T stored_item;

		if ( client.call(item, stored_item) )
			ROS_INFO("Stored item with id %s", stored_item.get_id().c_str());
		else
			ROS_INFO("Could not store with id %s", stored_item.get_id().c_str());	

		return stored_item;
	}

	template <typename T>
	T deleteObject( T item )
	{
		// Set up client
		ros::ServiceClient client;
		client = n_.serviceClient<T>("/datamanager/del_" + T::IDENTIFIER );

		// Response variable
		T deleted_item; // stays empty, needed for servivce call

		if ( client.call(item, deleted_item) )
			ROS_INFO("Deleted item from database with id %s", item.get_id().c_str());
		else
			ROS_INFO("Could not delete with id %s", item.get_id().c_str());	

		return deleted_item;
	}
	
  private:
  	ros::NodeHandle 	n_;
};

#endif // ROSE_DATAMANAGER_API_HPP