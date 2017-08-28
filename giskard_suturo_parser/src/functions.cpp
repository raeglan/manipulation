#include "giskard_suturo_parser/functions.h"
#include "giskard_suturo_parser/utils.h"

namespace giskard_suturo {

void FunctionDefinition::addScope(boost::shared_ptr<AdvancedScope> superScope) { throw std::domain_error("[FunctionDefinition] Can not add additional super scopes."); }
void FunctionDefinition::addScope(std::string alias, boost::shared_ptr<AdvancedScope> superScope) { throw std::domain_error("[FunctionDefinition] Can not add additional super scopes."); }

void FunctionDefinition::addArgument(std::string name, SpecPtr spec) {
	AdvancedScope::addSpec(name, spec);
	constSpecMap[name] = false;
	argumentSpecs.push_back(spec);
	arguments.push_back(name);
	prefix = getSuperScopes()[0]->getPrefix() + this->name + toTypeList(argumentSpecs) + "::";
}

void FunctionDefinition::addSpec(std::string name, SpecPtr spec) {
	AdvancedScope::addSpec(name, spec);
	if (dynamic_pointer_cast<SFunctionCallCache>(spec)) {
		SFunctionCallCachePtr fnCache = dynamic_pointer_cast<SFunctionCallCache>(spec);
		constSpecMap[name] = true;
		for (size_t i = 0; i < fnCache->arguments.size(); i++) {
			if (!isConstSpec(fnCache->arguments[i])) {
				constSpecMap[name] = false;
				return;
			}
		}
	} else {
		constSpecMap[name] = isConstSpec(spec);
	}
}

bool FunctionDefinition::isConstSpec(const SpecPtr& specPtr) const {
	std::vector<SpecPtr> references;
	getReferenceSpecs(references, specPtr);
	for (size_t i = 0; i < references.size(); i++) {
		std::string refName = getReferenceName(references[i]);
		if (refName.find(prefix) == std::string::npos)
			continue;

		std::string localName = refName.substr(prefix.size());

		auto it = constSpecMap.find(localName);
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

	std::unordered_map<std::string, std::string> referenceRenaming;
	auto superScope = getSuperScopes()[0];
    for (size_t i = args.size(); i < specInsertHistory.size(); i++) {
		auto it = constSpecMap.find(specInsertHistory[i]);
		
		SpecPtr specToAdd = getSpec(specInsertHistory[i]);
		if (dynamic_pointer_cast<SFunctionCallCache>(specToAdd)) {
			if (it->second) {
				if (!superScope->getSpec(specInsertHistory[i])) {
					SpecPtr result = dynamic_pointer_cast<SFunctionCallCache>(specToAdd)->createInstance(scope);
					std::string resolvedName = getReferenceName(result);
					referenceRenaming[prefix + specInsertHistory[i]] = resolvedName;
				}
			} else {
				SFunctionCallCachePtr fnCall = dynamic_pointer_cast<SFunctionCallCache>(specToAdd);
				std::vector<SpecPtr> resolvedArgs;
				for (size_t m = 0; m < fnCall->arguments.size(); m++) {
					SpecPtr specCopy = deepCopySpec(fnCall->arguments[m]);
					std::vector<SpecPtr> references;
					getReferenceSpecs(references, specCopy);
					for (size_t n = 0; n < references.size(); n++) {
						std::string refName = getReferenceName(references[n]);
						if (refName.find(prefix) == std::string::npos)
							continue;
	
						std::string localName = refName.substr(prefix.size());
	
						auto it = constSpecMap.find(localName);
						if (it != constSpecMap.end()) {
							if (!it->second) {
								std::string resolvedName = scope->getPrefix() + instName + "::" + localName;
								setReferenceName(references[n], resolvedName);
							}
						}
					}
					resolvedArgs.push_back(specCopy);
				}
				SpecPtr result = fnCall->functionDefinition->createInstance(resolvedArgs, scope);
				std::string resolvedName = getReferenceName(result);
				referenceRenaming[prefix + specInsertHistory[i]] = resolvedName;
			}
		} else {	
			if (it->second) {
				std::string resolvedName = fnName + "::" + specInsertHistory[i];
				if (!superScope->getSpec(resolvedName))
					superScope->addSpec(resolvedName, specToAdd);
			} else {
				SpecPtr specCopy = deepCopySpec(getLocalSpec(specInsertHistory[i]));
				std::vector<SpecPtr> references;
				getReferenceSpecs(references, specCopy);
				for (size_t i = 0; i < references.size(); i++) {
					std::string refName = getReferenceName(references[i]);
					if (refName.find(prefix) == std::string::npos)
						continue;

					auto rit = referenceRenaming.find(refName);
					if (rit != referenceRenaming.end()) {
						setReferenceName(references[i], rit->second);
					} else {
						std::string localName = refName.substr(prefix.size());

						auto it = constSpecMap.find(localName);
						if (it != constSpecMap.end()) {
							if (!it->second) {
								std::string resolvedName = scope->getPrefix() + instName + "::" + localName;
								setReferenceName(references[i], resolvedName);
							}
						}
					}
				}
				scope->addSpec(instName + "::" + specInsertHistory[i], specCopy);
			}
		}
	}

	SpecPtr returnCopy = deepCopySpec(returnExpression);
	std::vector<SpecPtr> references;
	getReferenceSpecs(references, returnCopy);
	for (size_t i = 0; i < references.size(); i++) {
      std::string refName = getReferenceName(references[i]);
	  if (refName.find(prefix) == std::string::npos)
		  continue;

	  std::string localName = refName.substr(prefix.size());

      auto it = constSpecMap.find(localName);
      if (it != constSpecMap.end()) {
      	if (!it->second) {
      		std::string resolvedName = scope->getPrefix() + instName + "::" + localName;
      		setReferenceName(references[i], resolvedName);
      	}
      }
    }
    scope->addSpec(instName, returnCopy);

    return createReferenceSpec(instName, returnCopy, scope);
}

SpecPtr createFunctionCallCache(std::vector<SpecPtr> arguments, FnDefPtr fnDef) {
	if (dynamic_pointer_cast<DoubleSpec>(fnDef->getReturnSpec())) {
		return instance<DoubleFunctionCallCache>(arguments, fnDef);

	} else if (dynamic_pointer_cast<VectorSpec>(fnDef->getReturnSpec())) {
		return instance<VectorFunctionCallCache>(arguments, fnDef);

	} else if (dynamic_pointer_cast<RotationSpec>(fnDef->getReturnSpec())) {
		return instance<RotationFunctionCallCache>(arguments, fnDef);

	} else if (dynamic_pointer_cast<FrameSpec>(fnDef->getReturnSpec())) {
		return instance<FrameFunctionCallCache>(arguments, fnDef);

	} else if (dynamic_pointer_cast<StringSpec>(fnDef->getReturnSpec())) {
		return instance<StringFunctionCallCache>(arguments, fnDef);

	} else if (dynamic_pointer_cast<ListSpec>(fnDef->getReturnSpec())) {
		return instance<ListFunctionCallCache>(arguments, fnDef);
	}

	throw std::domain_error("Could not create function call cache for funtion with return type " + toTypeString(fnDef->getReturnSpec()));
}

}
