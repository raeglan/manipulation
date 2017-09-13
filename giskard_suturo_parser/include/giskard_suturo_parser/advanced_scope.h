#pragma once

#include <giskard_suturo_parser/specifications.h>

#include <string>
#include <unordered_map>
#include <unordered_set>

using namespace giskard_core;

namespace giskard_suturo {

class AdvancedScope;
class FunctionDefinition;

typedef typename boost::shared_ptr<FunctionDefinition> FnDefPtr;


/**
 * @brief      Structure containing data needed for a function call.
 */
struct FunctionCall {
    FnDefPtr function;
    std::vector<SpecPtr> arguments;
    SpecPtr returnReference; 
};

typedef typename boost::shared_ptr<AdvancedScope> AdvancedScopePtr;

/**
 * @brief      The advanced scope class supports layered scopes and named search scopes.
 * 
 * This class was specifically designed for the GLang++ parser procedure. It supports concepts like super scopes that it searches when queried for an expression. It also supports aliased imported scopes that can specifically queried for expressions.
 *  
 */
class AdvancedScope : public Spec {
    
    /**
     * @brief      Structure recording under which name a scope was inserted into this scope.
     */
    struct ScopeInsert {
        std::string alias;
        AdvancedScopePtr scope;
    };
public:

    /**
     * @brief      Constructor for the advanced scope class.
     *
     * @param[in]  _filePath     File that this scope is being parsed from
     * @param[in]  _searchPath   Base path for the relative search for imported files.
     * @param[in]  _prefix       Prefix of this scope.
     * @param[in]  _superScopes  List of super scopes
     */
    AdvancedScope(std::string _filePath = "",
     std::string _searchPath = "",
     std::string _prefix = "",
     std::vector<AdvancedScopePtr> _superScopes = std::vector<AdvancedScopePtr>())
    : filePath(_filePath)
    , searchPath(_searchPath)
    , superScopes(_superScopes)
    , prefix(_prefix) {}

    /**
     * @brief      Inserts a scope entry into this scope.
     *
     * @param[in]  entry  The entry to insert
     */
    virtual void addSpec(ScopeEntry entry);

    /**
     * @brief      Inserts an entry into this scope.
     *
     * @param[in]  name  Name of the entry
     * @param[in]  spec  Specification of the expression
     */
    virtual void addSpec(std::string name, SpecPtr spec);


    /**
     * @brief      Adds a super scope to this scope.
     *
     * @param[in]  superScope  The super scope
     */
    virtual void addScope(AdvancedScopePtr superScope);

    /**
     * @brief      Adds a super scope to this scope, that can be referenced by an alias.
     *
     * @param[in]  alias       Alias to use for this scope
     * @param[in]  superScope  Scope to add
     */
    virtual void addScope(std::string alias, AdvancedScopePtr superScope);

    /**
     * @brief      Adds a function definition to this scope.
     *
     * @param[in]  function  The function definition to add
     */
    virtual void addFunction(FnDefPtr function);

    /**
     * @brief      Generates a function call to the named function with the given arguments. The function can be defined either in this scope or in the super scopes.
     *
     * @param[in]  name       Name of the function
     * @param      arguments  Arguments for the call
     * @param      caller     The scope that is calling the function
     *
     * @return     The generated specification
     */
    SpecPtr callFunction(const std::string& name, std::vector<SpecPtr>& arguments, AdvancedScopePtr& caller);

    /**
     * @brief      Generates a function call to the named function with the given arguments. The function must be defined in this scope.
     *
     * @param[in]  name       Name of the function
     * @param      arguments  Arguments for the call
     * @param      caller     The scope making the call
     *
     * @return     { description_of_the_return_value }
     */
    SpecPtr callLocalFunction(const std::string& name, std::vector<SpecPtr>& arguments, AdvancedScopePtr& caller);

    /**
     * @brief      Determines whether a function has been called before with the given arguments.
     *
     * @param[in]  name       Name of the function
     * @param      arguments  Arguments for the call
     * @param      call       The call structure storing this call
     *
     * @return     True if this function has been called with these arguments before, False otherwise.
     */
    bool hasFunctionCall(const std::string& name, std::vector<SpecPtr>& arguments, FunctionCall& call) const;


    /**
     * @brief      Determines whether this scope or one of it's super scopes defines a function with matching name and argument signature.
     *
     * @param[in]  name       Name of the function
     * @param[in]  signature  Argument signature
     *
     * @return     True if a matching function is defined
     */
    bool hasFunction(const std::string& name, const std::vector<SpecPtr>& signature) const;

    /**
     * @brief      Search for a specification only in this scope.
     *
     * @param[in]  name  Name of the specification
     *
     * @return     Found specification
     */
    SpecPtr getLocalSpec(const std::string& name) const;

    /**
     * @brief      Find a specification in this scope or it's super scopes.
     *
     * @param[in]  name  Name of the specification
     *
     * @return     Found specification
     */
    SpecPtr getSpec(const std::string& name) const;

    /**
     * @brief      Find a specification and attach the give prefix to the reference.
     *
     * @param[in]  name          Name of the specification
     * @param      globalPrefix  Prefix for the reference
     *
     * @return     Reference specification
     */
    SpecPtr getSpec(const std::string& name, std::string& globalPrefix) const;
    
    /**
     * @brief      Get a scope by it's alias.
     *
     * @param[in]  alias  Alias of the scope
     *
     * @return     Pointer to the scope
     */
    AdvancedScopePtr getScope(const std::string& alias) const;
    
    /**
     * @brief      Get a scope by the file it was generated from.
     *
     * @param[in]  filePath  File path
     *
     * @return     Pointer to the scope
     */
    AdvancedScopePtr getScopeByFile(const std::string& filePath) const;
    
    /**
     * @brief      Determines whether one of this scope's super scopes is generated from the given file.
     *
     * @param[in]  filePath  File path
     *
     * @return     True if a super scope is generated by this file.
     */
    bool isSuperScope(const std::string& filePath) const;

    /**
     * @brief      Determines whether a name is already taken.
     *
     * @param[in]  name  Name to check
     *
     * @return     True if the name is taken.
     */
    bool isNameTaken(const std::string& name) const;

    /**
     * @brief      Equality check required by giskard_core::Spec
     *
     * @param[in]  other  The other specification
     *
     * @return     True if scope equals other specification
     */
    bool equals(const Spec& other) const;

    /**
     * @brief      Gets the input specs. Required by giskard_core::Spec
     *
     * @param      inputs  List of found inputs
     */
    void get_input_specs(std::vector<const InputSpec*>& inputs) const {};

    /**
     * @brief      Generation method generating a giskard_core::ScopeSpec from this scope.
     *
     * @param      scope      Scope being generated
     * @param      generated  Scopes that were already generated
     */
    void convert(ScopeSpec& scope, std::unordered_set<const AdvancedScope*>& generated) const;

    /**
     * @brief      Get a list of all of this scope's super scopes.
     *
     * @return     The super scopes.
     */
    inline std::vector<AdvancedScopePtr> getSuperScopes() const { return superScopes; };

    /**
     * @brief      Get this scope's global prefix.
     *
     * @return     Prefix.
     */
    const std::string& getPrefix() const { return prefix; }
    
    /** File path this scope was generated from */
    const std::string filePath;

    /** Path to begin local searches from */
    const std::string searchPath;

    /** Names of predefined functions */
    const static std::string fAbs, 
    fSin, fCos, fTan, fASin, fACos, 
    fATan, fFMod, fMax, fMin, fIf, 
    fNorm, fSlerp, fCross, fInvert, 
    fInScalar, fInJoint, fInVec3, 
    fInRotation, fInFrame,
    fCVec3, fCRotation, fCFrame,
    fCHardC, fCSoftC, fCContC;

protected:
    /** This scope's global prefix */
    std::string prefix;

    /** Insert order of specifications */ 
    std::vector<std::string> specInsertHistory;
    
    /** Insert history of scopes */
    std::vector<ScopeInsert> scopeInsertHistory;
private:
    /** List of super scopes */
    std::vector<AdvancedScopePtr> superScopes;
    
    /** Aliased scopes */
    std::unordered_map<std::string, AdvancedScopePtr> aliasedScopes;

    /** Functions defined by this scope */
    std::unordered_map<std::string, std::vector<FnDefPtr>> functionDefinitions;

    /** Cache of function calls */ 
    std::unordered_map<std::string, std::vector<FunctionCall>> functionCalls;

    /** Scope entries */
    std::unordered_map<std::string, SpecPtr> specs;
};
}
