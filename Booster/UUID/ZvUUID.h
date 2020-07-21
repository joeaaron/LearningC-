#pragma once

#include <map>
#include <Windows.h>
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_generators.hpp"

class ZcUUID : public boost::uuids::uuid
{
public:
	ZcUUID() : boost::uuids::uuid()
	{
		//transform to base class pointer
		auto pUUID = dynamic_cast<boost::uuids::uuid*>(this);
		if (pUUID)
		{
			// check valid generator
			static std::map<DWORD, boost::uuids::random_generator> map_IdGen;
			DWORD dwCurID = GetCurrentThreadId();
			auto iterGen = map_IdGen.find(dwCurID);
			if (iterGen != map_IdGen.end())
			{
				// acquire the uuid
				*pUUID = iterGen->second();            
			}
			else
			{
				auto _iterGen = map_IdGen.insert(std::make_pair(dwCurID, boost::uuids::random_generator()));
				if (_iterGen.second) *pUUID = _iterGen.first->second();
			}
		}
	}

	ZcUUID(int) : boost::uuids::uuid(){}

	explicit ZcUUID(boost::uuids::uuid const& u) : boost::uuids::uuid(u){}

	operator boost::uuids::uuid()
	{
		return static_cast<boost::uuids::uuid&>(*this);
	}

	operator boost::uuids::uuid() const
	{
		return static_cast<boost::uuids::uuid const&>(*this);
	}

	void make_nil()
	{
		memset(data, 0, sizeof(data));
	}
};