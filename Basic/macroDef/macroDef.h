#pragma once
#include <unordered_map>
#include <string>

#define BEGIN_REG_COMMAND_FUN static const std::unordered_map<std::string, int> r_Func = {
#define END_REG_COMMAND_FUN };

#define REG_COMMAND_FUN(id, func_name){#func_name, id},

#define GET_COMMAND_ID(func_name) [func_name]{			\
	auto iterFunc = r_Func.find(func_name);				\
if (iterFunc != r_Func.end()) return iterFunc->second;	\
else return -1;}()										\

typedef void* HParamCam;