#include "giskard_suturo_parser/advanced_scope.h"
#include "giskard_suturo_parser/utils.h"

namespace giskard_suturo {

SpecPtr createTypeObject(const SpecPtr& ptr) {
  if (dynamic_pointer_cast<DoubleSpec>(ptr))
    return instance<DoubleConstSpec>();
  
  else if (dynamic_pointer_cast<VectorSpec>(ptr))
    return instance<VectorConstructorSpec>();
  
  else if (dynamic_pointer_cast<RotationSpec>(ptr))
    return instance<RotationQuaternionConstructorSpec>();
  
  else if (dynamic_pointer_cast<FrameSpec>(ptr))
    return instance<FrameConstructorSpec>();
  
  else if (dynamic_pointer_cast<ControllableConstraintSpec>(ptr))
    return instance<ControllableConstraintSpec>();
  
  else if (dynamic_pointer_cast<SoftConstraintSpec>(ptr))
    return instance<SoftConstraintSpec>();
  
  else if (dynamic_pointer_cast<HardConstraintSpec>(ptr))
    return instance<HardConstraintSpec>();
  
  else if (dynamic_pointer_cast<AdvancedScope>(ptr))
    return instance<AdvancedScope>();

  else if (dynamic_pointer_cast<ListSpec>(ptr)) {
    std::vector<SpecPtr> temp;
    temp.push_back(dynamic_pointer_cast<ListSpec>(ptr)->innerType());
    return instance<ConstListSpec>(temp);
  } else if (dynamic_pointer_cast<StringSpec>(ptr)) {
      return instance<ConstStringSpec>("");
  }

  throw std::domain_error("Failed to create type tracer.");
  // else if (dynamic_pointer_cast<VectorSpec>(ptr))
  //  return sVec3;
}

const std::string& StringReferenceSpec::get_reference_name() const {
  return reference_name;
}

void StringReferenceSpec::set_reference_name(const std::string& refName) {
  reference_name = refName;
}

bool StringReferenceSpec::equals(const Spec& other) const {
  if(!dynamic_cast<const StringReferenceSpec*>(&other))
    return false;

  return (dynamic_cast<const StringReferenceSpec*>(&other)->get_reference_name().compare(this->get_reference_name()) == 0);
}

std::string StringReferenceSpec::get_value() const {
	 SpecPtr spec = scope->getSpec(reference_name);
	 StringSpecPtr strSpec;
	 if (matches(spec, strSpec))
	 	return strSpec->get_value();
	 throw std::invalid_argument("There is no string value named '" + reference_name + "' in this scope: " + scope->filePath);
}

bool ConcatStringSpec::equals(const Spec& other) const {
    if (!dynamic_cast<const ConcatStringSpec*>(&other))
        return false;

    const ConcatStringSpec* pOther = dynamic_cast<const ConcatStringSpec*>(&other);

    return pOther->lhs->equals(*lhs) && pOther->rhs->equals(*rhs);
}

void ConcatStringSpec::set_lhs(const StringSpecPtr& _lhs) {
    lhs = _lhs;
}

void ConcatStringSpec::set_rhs(const StringSpecPtr& _rhs) {
    rhs = _rhs;
}

std::string ConcatStringSpec::get_value() const {
    return lhs->get_value() + rhs->get_value();
}

ConstListSpec::ConstListSpec(const std::vector<SpecPtr>& val)
: value(val) {
  for (size_t i = 1; i < val.size(); i++)
    if (!typesAreEqual(val[0], val[1]))
      throw std::domain_error("Lists need to be type homogenous! List types: " + toTypeList(val));
}

const std::string& ListReferenceSpec::get_reference_name() const {
  return reference_name;
}

void ListReferenceSpec::set_reference_name(const std::string& refName) {
  reference_name = refName;
}

bool ListReferenceSpec::equals(const Spec& other) const {
  if(!dynamic_cast<const ListReferenceSpec*>(&other))
    return false;

  return (dynamic_cast<const ListReferenceSpec*>(&other)->get_reference_name().compare(this->get_reference_name()) == 0);
}

std::vector<SpecPtr> ListReferenceSpec::get_value() const {
   SpecPtr spec = scope->getSpec(reference_name);
   ListSpecPtr listSpec;
   if (matches(spec, listSpec))
    return listSpec->get_value();
   throw std::invalid_argument("There is no string value named '" + reference_name + "' in this scope: " + scope->filePath);
}

bool ConstListSpec::equals(const Spec& other) const {
  if (!dynamic_cast<const ConstListSpec*>(&other))
    return false;

  const ConstListSpec* pOther = dynamic_cast<const ConstListSpec*>(&other);
  if (pOther->value.size() != value.size())
    return false;

  for (size_t i = 0; i < value.size(); i++)
    if (!value[i]->equals(*(pOther->value[i])))
      return false;

  return true;
}

void ConstListSpec::set_value(const std::vector<SpecPtr>& val) {
  value = val;
}

std::vector<SpecPtr> ConstListSpec::get_value() const {
  return value;
}

void ConstListSpec::get_input_specs(std::vector<const InputSpec*>& inputs) const {
  for (size_t i = 0; i < value.size(); i++)
    value[i]->get_input_specs(inputs);
}

ConcatListSpec::ConcatListSpec(const ListSpecPtr& _lhs, const ListSpecPtr& _rhs)
: lhs(_lhs)
, rhs(_rhs) {
    if (!typesAreEqual(lhs, rhs))
        throw std::invalid_argument("Lists need to be homogenous!");
}

bool ConcatListSpec::equals(const Spec& other) const {
  if (!dynamic_cast<const ConcatListSpec*>(&other))
    return false;

  const ConcatListSpec* pOther = dynamic_cast<const ConcatListSpec*>(&other);
  return lhs->equals(*(pOther->lhs)) && rhs->equals(*(pOther->rhs));
}

void ConcatListSpec::set_lhs(const ListSpecPtr& _lhs) { 
  lhs = _lhs; 
}

void ConcatListSpec::set_rhs(const ListSpecPtr& _rhs) { 
  rhs = _rhs; 
}

std::vector<SpecPtr> ConcatListSpec::get_value() const {
  std::vector<SpecPtr> l = lhs->get_value();
  std::vector<SpecPtr> r = rhs->get_value();
  std::vector<SpecPtr> out(l.begin(), l.end());
  for (size_t i = 0; i < r.size(); i++)
    out.push_back(r[i]);
  return out;
}

void ConcatListSpec::get_input_specs(std::vector<const InputSpec*>& inputs) const { 
  lhs->get_input_specs(inputs);
  rhs->get_input_specs(inputs);
}

}
