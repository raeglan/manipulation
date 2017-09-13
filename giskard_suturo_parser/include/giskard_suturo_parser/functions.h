#pragma once

#include "giskard_suturo_parser/advanced_scope.h"

namespace giskard_suturo {
    
    /**
     * @brief      This class models the definition of a function.
     */
    class FunctionDefinition : public AdvancedScope {
    public:
        FunctionDefinition(const std::string& _name, AdvancedScopePtr superScope) 
        : AdvancedScope()
        , name(_name) 
        { 
            AdvancedScope::addScope(superScope);
        }
        
        /**
         * @brief      Adds an argument to the function's signature.
         *
         * @param[in]  name  Name of the argument
         * @param[in]  spec  Specification of the argument
         */
        void addArgument(std::string name, SpecPtr spec);

        /**
         * @brief      Adds a scope entry.
         *
         * @param[in]  name  Name of the entry
         * @param[in]  spec  Specification
         */
        void addSpec(std::string name, SpecPtr spec);

        /**
         * @brief      Adds a super scope. Function definitions can only have one super scope. Trying to add more will cause an exception to be thrown.
         *
         * @param[in]  superScope  Super scope to add
         */
        void addScope(boost::shared_ptr<AdvancedScope> superScope);

        /**
         * @brief      Adds a scope.
         *
         * @param[in]  alias       The alias
         * @param[in]  superScope  The super scope
         */
        void addScope(std::string alias, boost::shared_ptr<AdvancedScope> superScope);


        /**
         * @brief      Creates an instance of this function with the given parameters in the given scope.
         *
         * @param[in]  args   Arguments for the function call
         * @param      scope  The scope creating the instance
         *
         * @return     A reference specification linking to the name of the return value of the instance
         */
        SpecPtr createInstance(const std::vector<SpecPtr>& args, AdvancedScopePtr& scope) const;
        
        /**
         * @brief      Checks whether a list of specifications matches the function's signature.
         *
         * @param[in]  args  The specifications to check against
         *
         * @return     True if the check passes, false otherwise.
         */
        bool checkTypeSignatureAgreement(const std::vector<SpecPtr> args) const;
        
        /**
         * @brief      Sets the specification calculating the return value of the function.
         *
         * @param[in]  retSpec  The return specification
         */
        void setReturnSpec(const SpecPtr& retSpec) { returnExpression = retSpec; }
        
        /**
         * @brief      Gets the return specification.
         *
         * @return     The return specification.
         */
        const SpecPtr getReturnSpec() const { return returnExpression; }

        /**
         * @brief      Gets the type trace of the function's signature.
         *
         * @return     List of types matching the function's signature
         */
        std::vector<SpecPtr> getSignature() const;
        
        /** Name of the defined function */
        const std::string name;
    protected:

        /**
         * @brief      Internal method for checking whether a specification is constant within this function definition.
         *
         * @param[in]  specPtr  Specification to check.
         *
         * @return     True if specification is constant, False otherwise.
         */
        bool isConstSpec(const SpecPtr& specPtr) const;
        
    private:
        /** Specification of this function's return value */
        SpecPtr returnExpression;

        /** Placeholder specifications for arguments */
        std::vector<SpecPtr> argumentSpecs;

        /** Names of the function's arguments */
        std::vector<std::string> arguments;

        /** Map caching whether specifications are constant */
        std::unordered_map<std::string, bool> constSpecMap;
    };
    
    /**
     * @brief      Base structure for caching function instances within function definitions.
     */
    struct SFunctionCallCache {
        SFunctionCallCache(std::vector<SpecPtr> _arguments, FnDefPtr _fnDef)
        : arguments(_arguments)
        , functionDefinition(_fnDef)
        { }
        
        /** Arguments of the instance */
        std::vector<SpecPtr> arguments;

        /** Function definition used for instancing */
        FnDefPtr functionDefinition;
        
        /** Instantiates function call */
        SpecPtr createInstance(AdvancedScopePtr& scope) const {
            return functionDefinition->createInstance(arguments, scope);
        }
    };
    
    typedef typename boost::shared_ptr<SFunctionCallCache> SFunctionCallCachePtr;
    
    /**
     * @brief      Function call cache for scalar functions.
     */
    class DoubleFunctionCallCache : public DoubleSpec, public SFunctionCallCache {
    public:
        DoubleFunctionCallCache(std::vector<SpecPtr> _arguments, FnDefPtr _fnDef)
        : SFunctionCallCache(_arguments, _fnDef) {}
        
        virtual bool equals(const Spec& other) const {
            if (!dynamic_cast<const DoubleFunctionCallCache*>(&other))
                return false;
            
            const DoubleFunctionCallCache* pOther = dynamic_cast<const DoubleFunctionCallCache*>(&other);
            if (pOther->arguments.size() != arguments.size())
                return false;
            
            if (pOther->functionDefinition->name != functionDefinition->name)
                return false;
            
            if (!functionDefinition->checkTypeSignatureAgreement(pOther->functionDefinition->getSignature()))
                return false;
            
            for (size_t i = 0; i < arguments.size(); i++)
                if (!arguments[i]->equals(*(pOther->arguments[i])))
                    return false;
            
            return true;
        }
        
        void get_input_specs(std::vector<const InputSpec*>& inputs) const {};
        
        KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope) {
            return KDL::Expression<double>::Ptr();
        }   
    };
    
    /**
     * @brief      Function call cache for vector functions. 
     */
    class VectorFunctionCallCache : public VectorSpec, public SFunctionCallCache {
    public:
        VectorFunctionCallCache(std::vector<SpecPtr> _arguments, FnDefPtr _fnDef)
        : SFunctionCallCache(_arguments, _fnDef) {}
        
        virtual bool equals(const Spec& other) const {
            if (!dynamic_cast<const VectorFunctionCallCache*>(&other))
                return false;
            
            const VectorFunctionCallCache* pOther = dynamic_cast<const VectorFunctionCallCache*>(&other);
            if (pOther->arguments.size() != arguments.size())
                return false;
            
            if (pOther->functionDefinition->name != functionDefinition->name)
                return false;
            
            if (!functionDefinition->checkTypeSignatureAgreement(pOther->functionDefinition->getSignature()))
                return false;
            
            for (size_t i = 0; i < arguments.size(); i++)
                if (!arguments[i]->equals(*(pOther->arguments[i])))
                    return false;
            
            return true;
        }
        
        void get_input_specs(std::vector<const InputSpec*>& inputs) const {};
        
        KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope) {
            return KDL::Expression<KDL::Vector>::Ptr();
        }   
    };
    
    /**
     * @brief      Function call cache for rotation functions.
     */
    class RotationFunctionCallCache : public RotationSpec, public SFunctionCallCache {
    public:
        RotationFunctionCallCache(std::vector<SpecPtr> _arguments, FnDefPtr _fnDef)
        : SFunctionCallCache(_arguments, _fnDef) {}
        
        virtual bool equals(const Spec& other) const {
            if (!dynamic_cast<const RotationFunctionCallCache*>(&other))
                return false;
            
            const RotationFunctionCallCache* pOther = dynamic_cast<const RotationFunctionCallCache*>(&other);
            if (pOther->arguments.size() != arguments.size())
                return false;
            
            if (pOther->functionDefinition->name != functionDefinition->name)
                return false;
            
            if (!functionDefinition->checkTypeSignatureAgreement(pOther->functionDefinition->getSignature()))
                return false;
            
            for (size_t i = 0; i < arguments.size(); i++)
                if (!arguments[i]->equals(*(pOther->arguments[i])))
                    return false;
            
            return true;
        }
        
        void get_input_specs(std::vector<const InputSpec*>& inputs) const {};
        
        KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard_core::Scope& scope) {
            return KDL::Expression<KDL::Rotation>::Ptr();
        }   
    };
    
    /**
     * @brief      Function call cache for frame functions.
     */
    class FrameFunctionCallCache : public FrameSpec, public SFunctionCallCache {
    public:
        FrameFunctionCallCache(std::vector<SpecPtr> _arguments, FnDefPtr _fnDef)
        : SFunctionCallCache(_arguments, _fnDef) {}
        
        virtual bool equals(const Spec& other) const {
            if (!dynamic_cast<const FrameFunctionCallCache*>(&other))
                return false;
            
            const FrameFunctionCallCache* pOther = dynamic_cast<const FrameFunctionCallCache*>(&other);
            if (pOther->arguments.size() != arguments.size())
                return false;
            
            if (pOther->functionDefinition->name != functionDefinition->name)
                return false;
            
            if (!functionDefinition->checkTypeSignatureAgreement(pOther->functionDefinition->getSignature()))
                return false;
            
            for (size_t i = 0; i < arguments.size(); i++)
                if (!arguments[i]->equals(*(pOther->arguments[i])))
                    return false;
            
            return true;
        }
        
        void get_input_specs(std::vector<const InputSpec*>& inputs) const {};
        
        KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard_core::Scope& scope) {
            return KDL::Expression<KDL::Frame>::Ptr();
        }   
    };
    
    /**
     * @brief      Function call cache for string functions.
     */
    class StringFunctionCallCache : public StringSpec, public SFunctionCallCache {
    public:
        StringFunctionCallCache(std::vector<SpecPtr> _arguments, FnDefPtr _fnDef)
        : SFunctionCallCache(_arguments, _fnDef) {}
        
        virtual bool equals(const Spec& other) const {
            if (!dynamic_cast<const StringFunctionCallCache*>(&other))
                return false;
            
            const StringFunctionCallCache* pOther = dynamic_cast<const StringFunctionCallCache*>(&other);
            if (pOther->arguments.size() != arguments.size())
                return false;
            
            if (pOther->functionDefinition->name != functionDefinition->name)
                return false;
            
            if (!functionDefinition->checkTypeSignatureAgreement(pOther->functionDefinition->getSignature()))
                return false;
            
            for (size_t i = 0; i < arguments.size(); i++)
                if (!arguments[i]->equals(*(pOther->arguments[i])))
                    return false;
            
            return true;
        }
        
        void get_input_specs(std::vector<const InputSpec*>& inputs) const {};
        
        std::string get_value() const {
            return "";
        }   
    };
    
    /**
     * @brief      Function call cache for list functions.
     */
    class ListFunctionCallCache : public ListSpec, public SFunctionCallCache {
    public:
        ListFunctionCallCache(std::vector<SpecPtr> _arguments, FnDefPtr _fnDef)
        : SFunctionCallCache(_arguments, _fnDef) {}
        
        virtual bool equals(const Spec& other) const {
            if (!dynamic_cast<const ListFunctionCallCache*>(&other))
                return false;
            
            const ListFunctionCallCache* pOther = dynamic_cast<const ListFunctionCallCache*>(&other);
            if (pOther->arguments.size() != arguments.size())
                return false;
            
            if (pOther->functionDefinition->name != functionDefinition->name)
                return false;
            
            if (!functionDefinition->checkTypeSignatureAgreement(pOther->functionDefinition->getSignature()))
                return false;
            
            for (size_t i = 0; i < arguments.size(); i++)
                if (!arguments[i]->equals(*(pOther->arguments[i])))
                    return false;
            
            return true;
        }
        
        void get_input_specs(std::vector<const InputSpec*>& inputs) const {};
        
        std::vector<SpecPtr> get_value() const {
            return std::vector<SpecPtr>();
        }
        
        const SpecPtr innerType() const {
            return boost::dynamic_pointer_cast<ListSpec>(functionDefinition->getReturnSpec())->innerType();
        }
    };
    
    /**
     * @brief      Creates a function call cache.
     *
     * @param[in]  arguments  Arguments for the call
     * @param[in]  fnDef      Function definition
     *
     * @return     A function call cache
     */
    SpecPtr createFunctionCallCache(std::vector<SpecPtr> arguments, FnDefPtr fnDef);
    
}