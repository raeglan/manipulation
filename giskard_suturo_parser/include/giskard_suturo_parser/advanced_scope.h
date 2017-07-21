#pragma once

#include <giskard_suturo_parser/specifications.h>

#include <string>
#include <unordered_map>

using namespace giskard_core;

namespace giskard_suturo {

class AdvancedScope;
class FunctionDefinition;
// struct IFunctionDefinition {
//     virtual void addArgument(std::string name, SpecPtr spec) = 0;
//     virtual SpecPtr createInstance(const std::vector<SpecPtr>& args, boost::shared_ptr<AdvancedScope>& scope) const = 0;
//     virtual bool checkTypeSignatureAgreement(const std::vector<SpecPtr> args) const = 0;
//     virtual void setReturnSpec(const SpecPtr& retSpec) = 0;
//     virtual const SpecPtr getReturnSpec() const = 0;
// };

typedef typename boost::shared_ptr<FunctionDefinition> FnDefPtr;

struct FunctionCall {
    FnDefPtr function;
    std::vector<SpecPtr> arguments;
    SpecPtr returnReference; 
};

typedef typename boost::shared_ptr<AdvancedScope> AdvancedScopePtr;

class AdvancedScope : public Spec {
    struct ScopeInsert {
        std::string alias;
        AdvancedScopePtr scope;
    };
public:
    AdvancedScope(std::string _filePath = "",
     std::string _searchPath = "",
     std::vector<AdvancedScopePtr> _superScopes = std::vector<AdvancedScopePtr>())
    : filePath(_filePath)
    , searchPath(_searchPath)
    , superScopes(_superScopes) {}

    virtual void addSpec(ScopeEntry entry);
    virtual void addSpec(std::string name, SpecPtr spec);
    virtual void addScope(AdvancedScopePtr superScope);
    virtual void addScope(std::string alias, AdvancedScopePtr superScope);
    virtual void addFunction(FnDefPtr function);
    SpecPtr callFunction(const std::string& name, std::vector<SpecPtr>& arguments, AdvancedScopePtr& caller);
    SpecPtr callLocalFunction(const std::string& name, std::vector<SpecPtr>& arguments, AdvancedScopePtr& caller);
    bool hasFunctionCall(const std::string& name, std::vector<SpecPtr>& arguments, FunctionCall& call) const;
    bool hasFunction(const std::string& name, const std::vector<SpecPtr>& signature) const;

    SpecPtr getLocalSpec(const std::string& name) const;
    SpecPtr getSpec(const std::string& name) const;
    AdvancedScopePtr getScope(const std::string& alias) const;
    AdvancedScopePtr getScopeByFile(const std::string& filePath) const;
    bool isSuperScope(const std::string& filePath) const;
    bool isNameTaken(const std::string& name) const;

    bool equals(const Spec& other) const;
    void get_input_specs(std::vector<const InputSpec*>& inputs) const {};

    void convert(ScopeSpec& scope, std::string prefix) const;

    inline std::vector<AdvancedScopePtr> getSuperScopes() const { return superScopes; };

    const std::string filePath;
    const std::string searchPath;
    const static std::string fAbs, 
    fSin, fCos, fTan, fASin, fACos, 
    fATan, fFMod, fMax, fMin, fIf, 
    fNorm, fSlerp, fCross, fInvert, 
    fInScalar, fInJoint, fInVec3, 
    fInRotation, fInFrame,
    fCVec3, fCRotation, fCFrame,
    fCHardC, fCSoftC, fCContC;

protected:
    std::vector<std::string> specInsertHistory;
    std::vector<ScopeInsert> scopeInsertHistory;
private:
    std::vector<AdvancedScopePtr> superScopes;
    std::unordered_map<std::string, AdvancedScopePtr> aliasedScopes;
    std::unordered_map<std::string, std::vector<FnDefPtr>> functionDefinitions;
    std::unordered_map<std::string, std::vector<FunctionCall>> functionCalls;

    std::unordered_map<std::string, SpecPtr> specs;
};
}
