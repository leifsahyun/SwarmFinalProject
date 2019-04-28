#ifndef NOVEL_CMPGA_H
#define NOVEL_CMPGA_H
#include <argos3-examples/mpga.h>

using namespace argos;

class NovelCMPGA : public CMPGA {
public:
	void LaunchARGoS(UInt32 un_slave_id);
	void Evaluate();
}

#endif
