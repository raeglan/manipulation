#pragma once

#include "giskard_suturo_parser/advanced_scope.h"

namespace giskard_suturo {

class FunctionDefinition : public AdvancedScope {
public:
    FunctionDefinition(const std::string& _name, AdvancedScopePtr superScope) 
    : AdvancedScope()
    , name(_name) { 
    	AdvancedScope::addScope(superScope);
    }

    void addArgument(std::string name, SpecPtr spec);
    void addSpec(std::string name, SpecPtr spec);
	void addScope(boost::shared_ptr<AdvancedScope> superScope);
    void addScope(std::string alias, boost::shared_ptr<AdvancedScope> superScope);
    SpecPtr createInstance(const std::vector<SpecPtr>& args, AdvancedScopePtr& scope) const;
    bool checkTypeSignatureAgreement(const std::vector<SpecPtr> args) const;
    void setReturnSpec(const SpecPtr& retSpec) { returnExpression = retSpec; }
    const SpecPtr getReturnSpec() const { return returnExpression; }
    std::vector<SpecPtr> getSignature() const;

	const std::string name;
protected:
	bool isConstSpec(const SpecPtr& specPtr) const;
	
private:
	SpecPtr returnExpression;
	std::vector<std::string> arguments;
	std::unordered_map<std::string, bool> constSpecMap;
};

}