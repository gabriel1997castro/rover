#include "rover/SSC.h"
#include <stdio.h>
#include <string>

int main()
{
	std::string command;
	command = "#8 P1500 #9 P1500";
	sendCommand(command.c_str());
	return 0;
}
