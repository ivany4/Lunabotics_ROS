#ifndef _CARTOGRAPHER_H_
#define _CARTOGRAPHER_H_

#include "NodeAssistant.h"

namespace lunabotics {

class Cartographer : NodeAssistant {
private:

public:
	Cartographer(int argc, char **argv);
	~Cartographer();
	
	void exec();
};




}



#endif
