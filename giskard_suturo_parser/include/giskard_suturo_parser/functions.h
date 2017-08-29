#pragma once

#include "giskard_suturo_parser/advanced_scope.h"

namespace giskard_suturo {
    
    class FunctionDefinition : public AdvancedScope {
    public:
        FunctionDefinition(const std::string& _name, AdvancedScopePtr superScope) 
        : AdvancedScope()
        , name(_name) 
        { 
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
        std::vector<SpecPtr> argumentSpecs;
        std::vector<std::string> arguments;
        std::unordered_map<std::string, bool> constSpecMap;
    };
    
    struct SFunctionCallCache {
        SFunctionCallCache(std::vector<SpecPtr> _arguments, FnDefPtr _fnDef)
        : arguments(_arguments)
        , functionDefinition(_fnDef)
        { }
        
        std::vector<SpecPtr> arguments;
        FnDefPtr functionDefinition;
        
        SpecPtr createInstance(AdvancedScopePtr& scope) const {
            return functionDefinition->createInstance(arguments, scope);
        }
    };
    
    typedef typename boost::shared_ptr<SFunctionCallCache> SFunctionCallCachePtr;
    
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
    
    SpecPtr createFunctionCallCache(std::vector<SpecPtr> arguments, FnDefPtr fnDef);
    
}