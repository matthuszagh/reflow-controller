sources = pc.cpp
executable = pc
compiler = clang++
err_flags = -Wall -Wextra -Wshadow -Weffc++ -Wstrict-aliasing -ansi -pedantic -Werror
flags = -O3 -g -pthread $(err_flags) -std=c++11 -lusb-1.0
debug_flags = -g -pthread $(err_flags) -std=c++11 -lusb-1.0

all:
	bear --append $(compiler) $(sources) $(flags) -o $(executable)

debug:
	$(compiler) $(sources) $(debug_flags) -o $(executable)
