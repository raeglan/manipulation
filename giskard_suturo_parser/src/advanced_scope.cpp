#include "giskard_suturo_parser/advanced_scope.h"
#include "giskard_suturo_parser/parser.h"
#include "giskard_suturo_parser/functions.h"

#include "giskard_suturo_parser/utils.h"

namespace giskard_suturo {
const std::string AdvancedScope::fAbs  = "abs";
const std::string AdvancedScope::fSin  = "sin";
const std::string AdvancedScope::fCos  = "cos";
const std::string AdvancedScope::fTan  = "tan";
const std::string AdvancedScope::fASin = "asin";
const std::string AdvancedScope::fACos = "acos";
const std::string AdvancedScope::fATan = "atan";
const std::string AdvancedScope::fFMod = "fmod";
const std::string AdvancedScope::fMax  = "max";
const std::string AdvancedScope::fMin  = "min";
const std::string AdvancedScope::fIf   = "if";
const std::string AdvancedScope::fNorm = "norm";
const std::string AdvancedScope::fSlerp = "slerp";
const std::string AdvancedScope::fCross = "cross";
const std::string AdvancedScope::fInvert = "invert";
const std::string AdvancedScope::fInScalar = "inputScalar";
const std::string AdvancedScope::fInJoint = "inputJoint";
const std::string AdvancedScope::fInVec3 = "inputVec3";
const std::string AdvancedScope::fInRotation = "inputRotation";
const std::string AdvancedScope::fInFrame = "inputFrame";
const std::string AdvancedScope::fCVec3 = GiskardPPParser::sVec3;
const std::string AdvancedScope::fCRotation = GiskardPPParser::sRotation;
const std::string AdvancedScope::fCFrame = GiskardPPParser::sFrame;
const std::string AdvancedScope::fCHardC = GiskardPPParser::sHard;
const std::string AdvancedScope::fCSoftC = GiskardPPParser::sSoft;
const std::string AdvancedScope::fCContC = GiskardPPParser::sControllable;

bool AdvancedScope::equals(const Spec& other) const {
	return dynamic_cast<const AdvancedScope*>(&other) && dynamic_cast<const AdvancedScope*>(&other)->filePath == filePath;
}

void AdvancedScope::addSpec(ScopeEntry entry) {
	addSpec(entry.name, entry.spec);
}

void AdvancedScope::addSpec(std::string name, SpecPtr spec) {
	if (specs.find(name) != specs.end() || getScope(name))
		throw std::invalid_argument("Name '" + name + "' is already taken in scope '" + filePath + "'");

	specs[name] = spec;
	specInsertHistory.push_back(name);
}

void AdvancedScope::addFunction(FnDefPtr function) {
	auto signature = function->getSignature();
	if (hasFunction(function->name, signature))
		throw std::domain_error("Function '" + function->name + toTypeList(signature) + "' is already defined within scope: " + filePath);

	functionDefinitions[function->name].push_back(function);
}

void AdvancedScope::addScope(AdvancedScopePtr superScope) {
	AdvancedScopePtr existingScope = getScopeByFile(superScope->filePath);
	if (!existingScope) {
		superScopes.push_back(superScope);
		scopeInsertHistory.push_back({"", superScope});
	} else if (std::find(superScopes.begin(), superScopes.end(), existingScope) == superScopes.end()){
		superScopes.push_back(existingScope);
	}
}

void AdvancedScope::addScope(std::string alias, AdvancedScopePtr superScope) {
	if (getScope(alias))
		throw std::domain_error("Scope with alias '" + alias + "' already exists!");

	aliasedScopes[alias] = superScope;
	scopeInsertHistory.push_back({alias, superScope});
}


SpecPtr AdvancedScope::getLocalSpec(const std::string& name) const {
	auto it = specs.find(name);
	if (it != specs.end())
		return it->second;

	return SpecPtr();	
}

SpecPtr AdvancedScope::getSpec(const std::string& name) const {
	SpecPtr local = getLocalSpec(name);
	if (local)
		return local;

	for (size_t i = 0; i < superScopes.size(); i++) {
		SpecPtr out = superScopes[i]->getSpec(name);
		if (out)
			return out;
	}
	return local;
}

AdvancedScopePtr AdvancedScope::getScope(const std::string& alias) const {
	auto it = aliasedScopes.find(alias);
	if (it != aliasedScopes.end())
		return it->second;

	for (size_t i = 0; i < superScopes.size(); i++) {
		AdvancedScopePtr out = superScopes[i]->getScope(alias);
		if (out)
			return out;
	}

    return AdvancedScopePtr();
}

AdvancedScopePtr AdvancedScope::getScopeByFile(const std::string& filePath) const {
	for(auto it = aliasedScopes.begin(); it  != aliasedScopes.end(); it++) {
		if (it->second->filePath == filePath)
			return it->second;
		else {
			AdvancedScopePtr out = it->second->getScopeByFile(filePath);
			if (out)
				return out;
		}		
	}

	for (size_t i = 0; i < superScopes.size(); i++) {
		if (filePath == superScopes[i]->filePath)
			return superScopes[i];
		else {
			AdvancedScopePtr out = superScopes[i]->getScopeByFile(filePath);
			if (out)
				return out;	
		}
	}
	return AdvancedScopePtr();
}

bool AdvancedScope::isSuperScope(const std::string& filePath) const {
	for (size_t i = 0; i < superScopes.size(); i++) {
		if (filePath == superScopes[i]->filePath || superScopes[i]->isSuperScope(filePath))
			return true;
	}

	return false;
}

bool AdvancedScope::isNameTaken(const std::string& name) const {
	return !!getSpec(name);
}

SpecPtr AdvancedScope::callFunction(const std::string& name, std::vector<SpecPtr>& arguments, boost::shared_ptr<AdvancedScope>& caller) {
	if (name == fAbs) {
		DoubleSpecPtr dbl;
		if (arguments.size() == 1 && matches(arguments[0], dbl))
			return instance<AbsSpec>(dbl);

	} else if (name == fSin) {
		DoubleSpecPtr val;
		if (arguments.size() == 1 && matches(arguments[0], val))
			return instance<SinSpec>(val);

	} else if (name == fCos) {
		DoubleSpecPtr val;
		if (arguments.size() == 1 && matches(arguments[0], val))
			return instance<CosSpec>(val);

	} else if (name == fTan) {
		DoubleSpecPtr val;
		if (arguments.size() == 1 && matches(arguments[0], val))
			return instance<TanSpec>(val);

	} else if (name == fASin) {
		DoubleSpecPtr val;
		if (arguments.size() == 1 && matches(arguments[0], val))
			return instance<ASinSpec>(val);

	} else if (name == fACos) {
		DoubleSpecPtr val;
		if (arguments.size() == 1 && matches(arguments[0], val))
			return instance<ACosSpec>(val);

	} else if (name == fATan) {
		DoubleSpecPtr val;
		if (arguments.size() == 1 && matches(arguments[0], val))
			return instance<ATanSpec>(val);

	} else if (name == fFMod) {
		DoubleSpecPtr lhs, rhs;
		if (arguments.size() == 2 && matches(arguments[0], arguments[1], lhs, rhs))
			return instance<FmodSpec>(lhs, rhs);

	} else if (name == fMax) {
		DoubleSpecPtr lhs, rhs;
		if (arguments.size() == 2 && matches(arguments[0], arguments[1], lhs, rhs))
			return instance<MaxSpec>(lhs, rhs);

	} else if (name == fMin) {
		DoubleSpecPtr lhs, rhs;
		if (arguments.size() == 2 && matches(arguments[0], arguments[1], lhs, rhs))
			return instance<MinSpec>(lhs, rhs);

	} else if (name == fIf) {
		DoubleSpecPtr lhs, mhs, rhs;
		if (arguments.size() == 3 && matches(arguments[0], arguments[1], arguments[2], lhs, mhs, rhs))
			return instance<DoubleIfSpec>(lhs, mhs, rhs);

	} else if (name == fNorm) {
		VectorSpecPtr vec;
		if (arguments.size() == 1 && matches(arguments[0], vec))
			return instance<DoubleNormOfSpec>(vec);

	} else if (name == fSlerp) {
		DoubleSpecPtr dbl;
		RotationSpecPtr from, to;
		if (arguments.size() == 3 && matches(arguments[0], arguments[1], arguments[2], from, to, dbl))
			return instance<SlerpSpec>(from, to, dbl);

	} else if (name == fCross) {
		VectorSpecPtr lhs, rhs;
		if (arguments.size() == 2 && matches(arguments[0], arguments[1], lhs, rhs))
			return instance<VectorCrossSpec>(lhs, rhs);

	} else if (name == fInvert) {
		RotationSpecPtr rot;
		FrameSpecPtr frame;
		if (arguments.size() == 1 && matches(arguments[0], rot))
			return instance<InverseRotationSpec>(rot);
		else if (arguments.size() == 1 && matches(arguments[0], frame))
			return instance<InverseFrameSpec>(frame);

	} else if (name == fCVec3) {
		DoubleSpecPtr x, y, z;
		RotationSpecPtr rot;
		if (arguments.size() == 1 && matches(arguments[0], rot))
			return instance<VectorRotationVectorSpec>(rot);
		else if (arguments.size() == 3 && matches(arguments[0], arguments[1], arguments[2], x, y, z))
			return instance<VectorConstructorSpec>(x,y,z);

	} else if (name == fCRotation) {
		DoubleConstSpecPtr x, y, z, w;
		DoubleSpecPtr angle;
		VectorSpecPtr axis;
		if (arguments.size() == 2 && matches(arguments[0], arguments[1], axis, angle))
			return instance<AxisAngleSpec>(axis, angle);
		else if (arguments.size() == 4 && matches(arguments[0], arguments[1], arguments[2], arguments[3], x, y, z, w))
			return instance<RotationQuaternionConstructorSpec>(x->get_value(),y->get_value(),z->get_value(),w->get_value());

	} else if (name == fCFrame) {
		RotationSpecPtr rot;
		VectorSpecPtr pos;
		if (arguments.size() == 2 && matches(arguments[0], arguments[1], rot, pos))
			return instance<FrameConstructorSpec>(pos, rot);

	} else if (name == fCHardC) {
		DoubleSpecPtr lower, upper, expr;
		if (arguments.size() == 3 && matches(arguments[0], arguments[1], arguments[2], lower, upper, expr))
			return instance<HardConstraintSpec>(lower, upper, expr);

	} else if (name == fCContC) {
		DoubleSpecPtr lower, upper, weight;
		StringSpecPtr input;
		if (arguments.size() == 4 && matches(arguments[0], arguments[1], arguments[2], arguments[3], lower, upper, weight, input))
			return instance<ControllableConstraintSpec>(lower, upper, weight, input);

	} else if (name == fCSoftC) {
		DoubleSpecPtr lower, upper, weight, expr;
		StringSpecPtr name;
		if (arguments.size() == 5 && matches(arguments[0], arguments[1], arguments[2], arguments[3], arguments[4], lower, upper, weight, expr, name))
			return instance<SoftConstraintSpec>(expr, lower, upper, weight, name);
	
	} else if (name == fInScalar) {
		StringSpecPtr inputName;
		if (arguments.size() == 1 && matches(arguments[0], inputName))
			return DoubleInputSpecPtr(new DoubleInputSpec(inputName));
	
	} else if (name == fInJoint) {
		StringSpecPtr inputName;
		if (arguments.size() == 1 && matches(arguments[0], inputName))
			return JointInputSpecPtr(new JointInputSpec(inputName));
	
	} else if (name == fInVec3) {
		StringSpecPtr inputName;
		if (arguments.size() == 1 && matches(arguments[0], inputName))
			return VectorInputSpecPtr(new VectorInputSpec(inputName));
	
	} else if (name == fInRotation) {
		StringSpecPtr inputName;
		if (arguments.size() == 1 && matches(arguments[0], inputName))
			return RotationInputSpecPtr(new RotationInputSpec(inputName));
	
	} else if (name == fInFrame) {
		StringSpecPtr inputName;
		if (arguments.size() == 1 && matches(arguments[0], inputName))
			return FrameInputSpecPtr(new FrameInputSpec(inputName));
	}

	
	SpecPtr out = callLocalFunction(name, arguments, caller);
	if (out)
		return out;

	for (size_t i = 0; i < superScopes.size(); i++) {
		out = superScopes[i]->callFunction(name, arguments, caller);
		if (out)
			return out;
	}

	return SpecPtr();
    }

    bool AdvancedScope::hasFunctionCall(const std::string& name, std::vector<SpecPtr>& arguments, FunctionCall& call) const {
    	auto it = functionCalls.find(name);
    	if (it == functionCalls.end())
    		return false;

    	for (size_t i = 0; i < it->second.size(); i++) {
    		if (it->second[i].arguments.size() == arguments.size()) {
    			bool match = true;
    			for (size_t n = 0; n < arguments.size() && match; n++)
    				match = match && it->second[i].arguments[n]->equals(*arguments[n]);

    			if (match) {
    				call = it->second[i];
    				return true;
    			}
    		}
    	}
    	return false;
    }

    SpecPtr AdvancedScope::callLocalFunction(const std::string& name, std::vector<SpecPtr>& arguments, boost::shared_ptr<AdvancedScope>& caller) {
      auto it = functionDefinitions.find(name);
      if (it != functionDefinitions.end()) {
     	for (size_t i = 0; it->second.size(); i++) {
     		if (it->second[i]->checkTypeSignatureAgreement(arguments)) {
     			FunctionCall call;
     			if (!caller->hasFunctionCall(name, arguments, call)) {
     				SpecPtr returnExpression = it->second[i]->createInstance(arguments, caller);
     				caller->functionCalls[name].push_back({it->second[i], arguments, returnExpression});
     				return returnExpression;
     			} else {
     				return call.returnReference;
     			}
     		}
     	}
      }
      return SpecPtr();
	}

	bool AdvancedScope::hasFunction(const std::string& name, const std::vector<SpecPtr>& signature) const {
		auto it = functionDefinitions.find(name);
		if (it == functionDefinitions.end()) {
			for (size_t i = 0; i < superScopes.size(); i++)
				if (superScopes[i]->hasFunction(name, signature))
					return true;
		} else {
			for (size_t i = 0; i < it->second.size(); i++) {
				if (it->second[i]->checkTypeSignatureAgreement(signature))
					return true;
			}
		}
		return false;
	}

	void AdvancedScope::convert(ScopeSpec& scope, std::string prefix) const {
		for (size_t i = 0; i < scopeInsertHistory.size(); i++) {
			if (scopeInsertHistory[i].alias.empty())
				scopeInsertHistory[i].scope->convert(scope, prefix);
			else
				scopeInsertHistory[i].scope->convert(scope, prefix + scopeInsertHistory[i].alias + "::");
		}

		for (size_t i = 0; i < specInsertHistory.size(); i++) {
			SpecPtr ptr = deepCopySpec(specs.find(specInsertHistory[i])->second);
			scope.push_back({ prefix + specInsertHistory[i], ptr});
			if (dynamic_pointer_cast<DoubleSpec>(ptr) || dynamic_pointer_cast<VectorSpec>(ptr) 
				|| dynamic_pointer_cast<RotationSpec>(ptr) || dynamic_pointer_cast<FrameSpec>(ptr)) {
				std::vector<SpecPtr> references;
				getReferenceSpecs(references, ptr);
				for (size_t n = 0; n < references.size(); n++) {
					if (dynamic_pointer_cast<DoubleReferenceSpec>(references[n])) {
						DoubleReferenceSpecPtr ref = dynamic_pointer_cast<DoubleReferenceSpec>(references[n]);
						ref->set_reference_name(prefix + ref->get_reference_name());
					} else if (dynamic_pointer_cast<VectorReferenceSpec>(references[n])) {
						VectorReferenceSpecPtr ref = dynamic_pointer_cast<VectorReferenceSpec>(references[n]);
						ref->set_reference_name(prefix + ref->get_reference_name());
					} else if (dynamic_pointer_cast<RotationReferenceSpec>(references[n])) {
						RotationReferenceSpecPtr ref = dynamic_pointer_cast<RotationReferenceSpec>(references[n]);
						ref->set_reference_name(prefix + ref->get_reference_name());
					} else if (dynamic_pointer_cast<FrameReferenceSpec>(references[n])) {
						FrameReferenceSpecPtr ref = dynamic_pointer_cast<FrameReferenceSpec>(references[n]);
						ref->set_reference_name(prefix + ref->get_reference_name());
					}
				}
			} 
			// else if (dynamic_pointer_cast<StringSpec>(ptr)) {
			// 	return StringReferenceSpecPtr(new StringReferenceSpec(name, searchScope));
			// } else if (dynamic_pointer_cast<ListSpec>(ptr)) {
			// 	return ListReferenceSpecPtr(new ListReferenceSpec(name, searchScope, dynamic_pointer_cast<ListSpec>(ptr)->innerType()));
			// }
		}
	}
}