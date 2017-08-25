#include "giskard_suturo_parser/parser.h"
#include <resource_retriever/retriever.h>
#include <giskard_core/giskard_core.hpp>
#include <giskard_suturo_parser/utils.h>

using namespace giskard_suturo;
using namespace giskard_core;
using namespace std;



int main(int argc, char const *argv[])
{
	GiskardPPParser parser;

    QPControllerSpec qpSpec = parser.parseFromFile("test_controller.gpp");

    cout << "SCOPE: --------------------------------------------" << endl;
    for (size_t i = 0; i < qpSpec.scope_.size(); i++) {
        std::string specStr = qpSpec.scope_[i].name + " = ";
        specToString(specStr, qpSpec.scope_[i].spec);
        cout << '\t' << specStr << endl;
    }
    cout << "---------------------------------------------------" << endl;

/*     cout << "CONTROLLABLES: --------------------------------------------" << endl;
    for (size_t i = 0; i < qpSpec.controllable_constraints_.size(); i++){
        std::string specStr;
        specToString(specStr, qpSpec.controllable_constraints_[i]);
        cout << '\t' << specStr << endl;
    }

    cout << "SOFTS: --------------------------------------------" << endl;
    for (size_t i = 0; i < qpSpec.soft_constraints_.size(); i++){
        std::string specStr;
        specToString(specStr, qpSpec.soft_constraints_[i]);
        cout << '\t' << specStr << endl;
    }

    cout << "HARDS: --------------------------------------------" << endl;
    for (size_t i = 0; i < qpSpec.hard_constraints_.size(); i++){
        std::string specStr;
        specToString(specStr, qpSpec.hard_constraints_[i]);
        cout << '\t' << specStr << endl;
    } */

    QPController controller = generate(qpSpec);

	return 0;
}
