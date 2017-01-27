#include <iostream>
#include <giskard/giskard.hpp>
#include <giskard/GiskardLangParser.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <fstream>
#include <streambuf>

int main(int argc, char const *argv[])
{
	if (argc < 2) {
		cerr << "GIMME PATH!" << endl;
		return 0;
	}

	std::ifstream t(argv[1]);
	std::string f((std::istreambuf_iterator<char>(t)),
                 std::istreambuf_iterator<char>());

	giskard::GiskardLangParser parser;
	giskard::QPControllerSpec spec = parser.parseQPController(f);
	YAML::Node yamlSpec = YAML::Load("");
	yamlSpec["spec"] = spec;

	std::ofstream fout("out.yaml");
	fout << yamlSpec;
	fout.close();

	giskard::QPController controller = giskard::generate(spec);


	std::cout << "ALL IS GOOD NOW" << std::endl;

	return 0;
}
