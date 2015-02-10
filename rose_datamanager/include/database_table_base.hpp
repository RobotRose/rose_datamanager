/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/02/19
* 		- File created.
*
* Description:
*	description
*
***********************************************************************************/
#ifndef DATABASE_TABLE_BASE_HPP
#define DATABASE_TABLE_BASE_HPP

#include <database_interface/db_class.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "item.hpp"
#include "rose_common/common.hpp"

using namespace database_interface;

template <class Serialized> class DatabaseTableBase : public DBClass
{
  public:
	DatabaseTableBase()
	: id_ ( DBFieldBase::TEXT, 
		this, 
		Serialized::IDENTIFIER + "_id", 
		Serialized::IDENTIFIER + "s", 
		true )
	, serialized_ ( DBFieldBase::TEXT,
		this,
		Serialized::IDENTIFIER + "_serialized",
		Serialized::IDENTIFIER + "s",
		true
		)
	{
		primary_key_field_ = &id_;

	    fields_.push_back( &serialized_ );
		
	    // Set read/write permissions
	    setAllFieldsReadFromDatabase( true) ;
	    setAllFieldsWriteToDatabase( true );

	    id_.setSequenceName( Serialized::IDENTIFIER + "_id_seq" );
	    id_.setWriteToDatabase( false );
	}

	~DatabaseTableBase(){}

	std::string getStringId() 	{ return Serialized::IDENTIFIER + rose_conversions::intToString( id_.data() ); }
	int 		getIntId() 		{ return id_.data(); }

  	DBField<Serialized> 	serialized_;
	DBField<int>			id_;

  private:
};

#endif //DATABASE_TABLE_BASE_HPP