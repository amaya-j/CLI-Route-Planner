all: CLI mytests

CLI: CLI.o graph.o priorityqueue.o
	g++ -std=c++14 -o CLI CLI.o graph.o priorityqueue.o
	./CLI  # Run CLI route planner and wait for user input

mytests: mytests.o graph.o priorityqueue.o
	g++ -std=c++14 -o mytests mytests.o graph.o priorityqueue.o
	./mytests  # Run the unit tests

CLI.o: CLI.cpp Graph.hpp PriorityQueue.hpp customexceptions.hpp
	g++ -std=c++14 -o CLI.o -c CLI.cpp

mytests.o: mytests.cpp Graph.hpp PriorityQueue.hpp customexceptions.hpp
	g++ -std=c++14 -o mytests.o -c mytests.cpp

graph.o: Graph.cpp Graph.hpp customexceptions.hpp
	g++ -std=c++14 -o graph.o -c Graph.cpp

priorityqueue.o: PriorityQueue.cpp PriorityQueue.hpp
	g++ -std=c++14 -o priorityqueue.o -c PriorityQueue.cpp

clean:
	rm -f *.o CLI mytests
	clear
