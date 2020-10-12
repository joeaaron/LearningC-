cmake_minimum_required(VERSION 3.9)
project(ExpCalc)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp Parser.h Parser.cpp Node.h Node.cpp Calc.h Calc.cpp Scanner.h Scanner.cpp 
	Serial.h Serial.cpp FunctionTable.h FunctionTable.cpp Storage.h Storage.cpp SymbolTable.h SymbolTable.cpp 
	Exception.h Exception.cpp Tracer.h Tracer.cpp ptr_vector.h DebugNew.h CommandParser.h CommandParser.cpp)
add_executable(ExpCalc ${SOURCE_FILES})
