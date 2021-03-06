/*
* Copyright (C) 2016 Georg Bartels <georg.bartels@cs.uni-bremen.de>
*
*
* This file is part of giskard_examples.
*
* giskard_examples is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2 
* of the License, or (at your option) any later version.  
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <giskard_core/giskard_core.hpp>
#include <giskard_suturo_parser/giskard_parser.hpp>
#include <giskard_suturo_parser/parser.h>
#include <dirent.h>

#include <string>

using namespace std;

int main(int argc, char **argv)
{

  if (argc < 2) {
    cerr << "Need at least one argument!" << endl;
  }

  int total = 0;
  int worked = 0;

  std::ofstream fout("controller_interfaces");

  
  for (int i = 1; i < argc; i++) {
    DIR* dir;
    struct dirent* ent;
    string dirPath = argv[i];
    if (dirPath[dirPath.size() - 1] != '/')
      dirPath += '/';

    if((dir = opendir(dirPath.c_str())) != 0) {
      while((ent = readdir(dir)) != 0) {
        string filename = ent->d_name;
        string filePath = dirPath + filename;
        
        giskard_core::QPControllerSpec spec;
        bool isController = false;
        bool failed = false;
        if (filename.find(".yaml") != string::npos) {
          total ++;
          try {
            YAML::Node node = YAML::LoadFile(filePath);
            spec = node.as<giskard_core::QPControllerSpec>();
            isController = true;
          } catch (const YAML::Exception& e) {
            cerr << "Error while parsing '" << (filePath) << "':" << endl << e.what() << endl; 
            failed = true;
          }
        } else if (filename.find(".giskard") != string::npos) {
          total ++;
          try {
            std::ifstream t(filePath);
            std::string fileStr;

            t.seekg(0, std::ios::end);   
            fileStr.reserve(t.tellg());
            t.seekg(0, std::ios::beg);

            fileStr.assign((std::istreambuf_iterator<char>(t)),
              std::istreambuf_iterator<char>());

            giskard_suturo::GiskardLangParser glParser;
            spec = glParser.parseQPController(fileStr);
            isController = true;
          } catch (giskard_suturo::GiskardLangParser::EOSException e) {
            cerr << "Error while parsing '" << (filePath) << "':" << std::endl << e.what() << std::endl; 
            failed = true;
          } catch (giskard_suturo::GiskardLangParser::ParseException e) {
            cerr << "Error while parsing '" << (filePath) << "':" << std::endl << e.what() << std::endl; 
            failed = true;
          }
        } else if (filename.find(".gpp") != string::npos) {
          total ++;
          try {
            giskard_suturo::GiskardPPParser glParser;
            spec = glParser.parseFromFile(filePath);
            isController = true;
          } catch (const std::exception& e) {
            cerr << "Error while parsing '" << (filePath) << "':" << std::endl << e.what() << std::endl; 
            failed = true;
          }
        }            
        
        if (isController) {
          try {
            giskard_core::QPController controller = giskard_core::generate(spec);
            fout << filename << std::endl;

            std::map<std::string, const giskard_core::Scope::InputPtr&> inputs = controller.get_input_map();
            for(auto it = inputs.begin(); it != inputs.end(); it++) {
              if (it->second->get_type() != giskard_core::tJoint) {
                fout << "   ";
                switch (it->second->get_type()) {
                  case giskard_core::tScalar:
                    fout << "double  " << it->first << std::endl;
                  break;
                  case giskard_core::tVector3:
                    fout << "vector3  " << it->first << std::endl;
                  break;
                  case giskard_core::tRotation:
                    fout << "rotation  " << it->first << std::endl;
                  break;
                  case giskard_core::tFrame:
                    fout << "frame  " << it->first << std::endl;
                  break;
                  default:
                  break;
                }
              }
            }
            fout << "------------------------------------------------------" << std::endl;
            worked++;
          } catch (const std::exception& e) {
            cerr << "Error while generating controller from file '" << (filePath) << "':" << endl << e.what() << endl; 
            failed = true;
          }
        }
        if (failed)
          cout << "--------------------------------------------------" << endl;
      }
      

    } else {
      cerr << "Couldn't open path '" << argv[i] << "'" << endl;
    }
  }

  cout << "------- " << worked << "/" << total << " controllers were successfully parsed." << endl; 
  fout.close();

  return 0;
}