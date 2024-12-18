#include <iostream>
#include <exception>
using namespace std;

#ifndef CUSTOMEXCEPTIONS_HPP
#define CUSTOMEXCEPTIONS_HPP

class edge_exception : public exception {
public:
    const char* what() const throw() {
        return "Error: Edge Exception.";
    }
};

class vertex_exception : public exception {
public:
    const char* what() const throw() {
        return "Error: Vertex Exception.";
    }
};
#endif

