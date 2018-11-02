sources = pc.cpp
executable = pc
compiler = clang++
err_flags = -Wall -Wextra -Wshadow -Weffc++ -Wstrict-aliasing -ansi -pedantic -Werro
flags = -O3 -g -pthread $(err_flags) -std=c++11 -lusb-1.0 -I/usr/include/python2.7 -lpython2.7
debug_flags = -g -pthread $(err_flags) -std=c++11 -lusb-1.0 -I/usr/include/python2.7 -lpython2.7

all:
	$(compiler) $(sources) $(flags) -o $(executable)

debug:
	$(compiler) $(sources) $(debug_flags) -o $(executable)
