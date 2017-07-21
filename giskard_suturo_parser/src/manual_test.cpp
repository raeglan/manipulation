#include "giskard_suturo_parser/parser.h"
#include <resource_retriever/retriever.h>
#include <giskard_core/giskard_core.hpp>

using namespace giskard_suturo;
using namespace giskard_core;
using namespace std;



int main(int argc, char const *argv[])
{
	GiskardPPParser parser;

    QPControllerSpec qpSpec = parser.parseFromFile("test_controller.gpp");
    QPController controller = generate(qpSpec);

	return 0;
}
