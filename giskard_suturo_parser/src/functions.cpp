#include "giskard_suturo_parser/functions.h"
#include "giskard_suturo_parser/utils.h"

namespace giskard_suturo {

void FunctionDefinition::addScope(boost::shared_ptr<AdvancedScope> superScope) { throw std::domain_error("[FunctionDefinition] Can not add additional super scopes."); }
void FunctionDefinition::addScope(std::string alias, boost::shared_ptr<AdvancedScope> superScope) { throw std::domain_error("[FunctionDefinition] Can not add additional super scopes."); }

void FunctionDefinition::addArgument(std::string name, SpecPtr spec) {
	AdvancedScope::addSpec(name, spec);
	constSpecMap[name] = false;
	arguments.push_back(name);
}

void FunctionDefinition::addSpec(std::string name, SpecPtr spec) {
	AdvancedScope::addSpec(name, spec);
	constSpecMap[name] = isConstSpec(spec);
}

bool FunctionDefinition::isConstSpec(const SpecPtr& specPtr) const {
	std::vector<SpecPtr> references;
	getReferenceSpecs(references, specPtr);
	for (size_t i = 0; i < references.size(); i++) {
		std::string refName = getReferenceName(references[i]);
		auto it = constSpecMap.find(refName);
		if (it != constSpecMap.end() && !it->second)
			return false;
	}
	return true;
}

std::vector<SpecPtr> FunctionDefinition::getSignature() const {
	std::vector<SpecPtr> out;
	for (size_t i = 0; i < arguments.size(); i++)
		out.push_back(getLocalSpec(arguments[i]));
	return out;
}

bool FunctionDefinition::checkTypeSignatureAgreement(const std::vector<SpecPtr> args) const {
	if (args.size() != arguments.size())
		return false;

	for (size_t i = 0; i < arguments.size(); i++) {
		SpecPtr argument = getLocalSpec(arguments[i]);
		if(!typesAreEqual(args[i], argument))
			return false;
	}
	return true;
}

SpecPtr FunctionDefinition::createInstance(const std::vector<SpecPtr>& args, AdvancedScopePtr& scope) const {
	if (!checkTypeSignatureAgreement(args))
		throw std::invalid_argument("Can't create instance for function " + name + " with arguments " + toTypeList(args));


	std::string fnName = name + toTypeList(args);

	std::string instName = name + '(';
	specToString(instName, args[0]);
	for (size_t i = 1; i < args.size(); i++) {
		instName += ',';
		specToString(instName, args[i]);
	}
	instName += ')';

	for (size_t i = 0; i < arguments.size(); i++) {
		scope->addSpec(instName + "::" + arguments[i], args[i]);	
	}

    for (size_t i = args.size(); i < specInsertHistory.size(); i++) {
		auto it = constSpecMap.find(specInsertHistory[i]);
		if (it->second) {
			std::string resolvedName = fnName + "::" + specInsertHistory[i];
			if (!scope->getSpec(resolvedName));
				scope->addSpec(resolvedName, getSpec(specInsertHistory[i]));
		} else {
			SpecPtr specCopy = deepCopySpec(getLocalSpec(specInsertHistory[i]));
			std::vector<SpecPtr> references;
    		getReferenceSpecs(references, specCopy);
    		for (size_t i = 0; i < references.size(); i++) {
		      std::string refName = getReferenceName(references[i]);
		      
		      auto it = constSpecMap.find(refName);
		      if (it != constSpecMap.end()) {
		      	if (it->second) {
		      		std::string resolvedName = fnName + "::" + refName;
		      		setReferenceName(references[i], resolvedName);
		      	} else {
		      		std::string resolvedName = instName + "::" + refName;
		      		setReferenceName(references[i], resolvedName);
		      	}
		      }
		    }
		    scope->addSpec(instName + "::" + specInsertHistory[i], specCopy);
		}
	}

	SpecPtr returnCopy = deepCopySpec(returnExpression);
	std::vector<SpecPtr> references;
	getReferenceSpecs(references, returnCopy);
	for (size_t i = 0; i < references.size(); i++) {
      std::string refName = getReferenceName(references[i]);
      
      auto it = constSpecMap.find(refName);
      if (it != constSpecMap.end()) {
      	if (it->second) {
      		std::string resolvedName = fnName + "::" + refName;
      		setReferenceName(references[i], resolvedName);
      	} else {
      		std::string resolvedName = instName + "::" + refName;
      		setReferenceName(references[i], resolvedName);
      	}
      }
    }
    scope->addSpec(instName, returnCopy);

    return createReferenceSpec(instName, returnCopy, scope);
}

}
