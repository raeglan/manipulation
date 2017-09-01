#include "giskard_suturo_parser/utils.h"
#include "giskard_suturo_parser/parser.h"

namespace giskard_suturo {

bool typesAreEqual(const SpecPtr& a, const SpecPtr& b) {
  if (dynamic_pointer_cast<DoubleSpec>(a) && dynamic_pointer_cast<DoubleSpec>(b))
    return true;
  else if (dynamic_pointer_cast<VectorSpec>(a) && dynamic_pointer_cast<VectorSpec>(b))
    return true;
  else if (dynamic_pointer_cast<RotationSpec>(a) && dynamic_pointer_cast<RotationSpec>(b))
    return true;
  else if (dynamic_pointer_cast<FrameSpec>(a) && dynamic_pointer_cast<FrameSpec>(b))
    return true;
  else if (dynamic_pointer_cast<StringSpec>(a) && dynamic_pointer_cast<StringSpec>(b))
    return true;
  else if (dynamic_pointer_cast<SoftConstraintSpec>(a) && dynamic_pointer_cast<SoftConstraintSpec>(b))
    return true;
  else if (dynamic_pointer_cast<HardConstraintSpec>(a) && dynamic_pointer_cast<HardConstraintSpec>(b))
    return true;
  else if (dynamic_pointer_cast<ControllableConstraintSpec>(a) && dynamic_pointer_cast<ControllableConstraintSpec>(b))
    return true;
  else if (dynamic_pointer_cast<ListSpec>(a) && dynamic_pointer_cast<ListSpec>(b))
    return typesAreEqual(dynamic_pointer_cast<ListSpec>(a)->innerType(), dynamic_pointer_cast<ListSpec>(b)->innerType());
  return false;
}

std::string toTypeString(const SpecPtr& ptr) {
  if (dynamic_pointer_cast<DoubleSpec>(ptr))
    return GiskardPPParser::sScalar;
  
  else if (dynamic_pointer_cast<VectorSpec>(ptr))
    return GiskardPPParser::sVec3;
  
  else if (dynamic_pointer_cast<RotationSpec>(ptr))
    return GiskardPPParser::sRotation;
  
  else if (dynamic_pointer_cast<FrameSpec>(ptr))
    return GiskardPPParser::sFrame;
  
  else if (dynamic_pointer_cast<ControllableConstraintSpec>(ptr))
    return GiskardPPParser::sControllable;
  
  else if (dynamic_pointer_cast<SoftConstraintSpec>(ptr))
    return GiskardPPParser::sSoft;
  
  else if (dynamic_pointer_cast<HardConstraintSpec>(ptr))
    return GiskardPPParser::sHard;
  
  else if (dynamic_pointer_cast<AdvancedScope>(ptr))
    return GiskardPPParser::sScope;

  else if (dynamic_pointer_cast<StringSpec>(ptr))
    return GiskardPPParser::sString;

  else if (dynamic_pointer_cast<ListSpec>(ptr)) {
    SpecPtr inner = dynamic_pointer_cast<ListSpec>(ptr)->innerType();
    return GiskardPPParser::sList + '<' + toTypeString(inner) + '>';
  }

  return "NULL";
  // else if (dynamic_pointer_cast<VectorSpec>(ptr))
  //  return sVec3;
}

std::string toTypeList(const std::vector<SpecPtr>& v) {
  if (v.size() == 0)
    return "()";

  std::stringstream ss;
  ss << '(' << toTypeString(v[0]);

  for (size_t i = 1; i < v.size(); i++) {
    ss << ", " << toTypeString(v[i]);
  }

  ss << ')';
  return ss.str();
}

SpecPtr createReferenceSpec(const std::string& name, SpecPtr ptr, const AdvancedScopePtr& searchScope) {
  if (dynamic_pointer_cast<DoubleSpec>(ptr)) {
    DoubleReferenceSpecPtr out = DoubleReferenceSpecPtr(new DoubleReferenceSpec());
    out->set_reference_name(name);
    return out;
  } else if (dynamic_pointer_cast<VectorSpec>(ptr)) {
    VectorReferenceSpecPtr out = VectorReferenceSpecPtr(new VectorReferenceSpec());
    out->set_reference_name(name);
    return out;
  } else if (dynamic_pointer_cast<RotationSpec>(ptr)) {
    RotationReferenceSpecPtr out = RotationReferenceSpecPtr(new RotationReferenceSpec());
    out->set_reference_name(name);
    return out;
  } else if (dynamic_pointer_cast<FrameSpec>(ptr)) {
    FrameReferenceSpecPtr out = FrameReferenceSpecPtr(new FrameReferenceSpec());
    out->set_reference_name(name);
    return out;
  } else if (dynamic_pointer_cast<StringSpec>(ptr)) {
    return StringReferenceSpecPtr(new StringReferenceSpec(name, searchScope));
  } else if (dynamic_pointer_cast<ListSpec>(ptr)) {
    return ListReferenceSpecPtr(new ListReferenceSpec(name, searchScope, dynamic_pointer_cast<ListSpec>(ptr)->innerType()));
  }
  // else if (dynamic_pointer_cast<ControllableConstraintSpec>(ptr))
  //  return GiskardPPParser::sControllable;
  // else if (dynamic_pointer_cast<SoftConstraintSpec>(ptr))
  //  return GiskardPPParser::sSoft;
  // else if (dynamic_pointer_cast<HardConstraintSpec>(ptr))
  //  return GiskardPPParser::sHard;
  // else if (dynamic_pointer_cast<AdvancedScope>(ptr))
  //  return GiskardPPParser::sScope;

  return SpecPtr();
}

std::string getReferenceName(const SpecPtr& ptr) {
  if (dynamic_pointer_cast<DoubleReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<DoubleReferenceSpec>(ptr)->get_reference_name();
  
  } else if (dynamic_pointer_cast<VectorReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<VectorReferenceSpec>(ptr)->get_reference_name();
  
  } else if (dynamic_pointer_cast<RotationReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<RotationReferenceSpec>(ptr)->get_reference_name();
  
  } else if (dynamic_pointer_cast<FrameReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<FrameReferenceSpec>(ptr)->get_reference_name();
  
  } else if (dynamic_pointer_cast<StringReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<StringReferenceSpec>(ptr)->get_reference_name();
  
  } else if (dynamic_pointer_cast<ListReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<ListReferenceSpec>(ptr)->get_reference_name();
  
  } 
  throw std::domain_error("[getReferenceName()] Unsupported reference type");
}
void setReferenceName(const SpecPtr& ptr, std::string name) {
  if (dynamic_pointer_cast<DoubleReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<DoubleReferenceSpec>(ptr)->set_reference_name(name);
  
  } else if (dynamic_pointer_cast<VectorReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<VectorReferenceSpec>(ptr)->set_reference_name(name);
  
  } else if (dynamic_pointer_cast<RotationReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<RotationReferenceSpec>(ptr)->set_reference_name(name);
  
  } else if (dynamic_pointer_cast<FrameReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<FrameReferenceSpec>(ptr)->set_reference_name(name);
  
  } else if (dynamic_pointer_cast<StringReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<StringReferenceSpec>(ptr)->set_reference_name(name);
  
  } else if (dynamic_pointer_cast<ListReferenceSpec>(ptr)) {
    return dynamic_pointer_cast<ListReferenceSpec>(ptr)->set_reference_name(name);
  
  } 
  throw std::domain_error("[setReferenceName()] Unsupported reference type"); 
}

void getReferenceSpecs(std::vector<SpecPtr>& references, const SpecPtr& specPtr) {
  if (dynamic_pointer_cast<DoubleInputSpec>(specPtr)) {
    DoubleInputSpecPtr castPtr = dynamic_pointer_cast<DoubleInputSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_name());

  } else if (dynamic_pointer_cast<JointInputSpec>(specPtr)) {
    JointInputSpecPtr castPtr = dynamic_pointer_cast<JointInputSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_name());
  
  } else if (dynamic_pointer_cast<DoubleReferenceSpec>(specPtr)) {
    DoubleReferenceSpecPtr castPtr = dynamic_pointer_cast<DoubleReferenceSpec>(specPtr);
    references.push_back(castPtr);
  } else if (dynamic_pointer_cast<DoubleAdditionSpec>(specPtr)) {
    DoubleAdditionSpecPtr castPtr = dynamic_pointer_cast<DoubleAdditionSpec>(specPtr);
    const std::vector<DoubleSpecPtr>& temp = castPtr->get_inputs();
    for (size_t i = 0; i < temp.size(); i++)
      getReferenceSpecs(references, temp[i]);

  } else if (dynamic_pointer_cast<DoubleSubtractionSpec>(specPtr)) {
    DoubleSubtractionSpecPtr castPtr = dynamic_pointer_cast<DoubleSubtractionSpec>(specPtr);
    const std::vector<DoubleSpecPtr>& temp = castPtr->get_inputs();
    for (size_t i = 0; i < temp.size(); i++)
      getReferenceSpecs(references, temp[i]);

  } else if (dynamic_pointer_cast<DoubleNormOfSpec>(specPtr)) {
    DoubleNormOfSpecPtr castPtr = dynamic_pointer_cast<DoubleNormOfSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_vector());

  } else if (dynamic_pointer_cast<DoubleMultiplicationSpec>(specPtr)) {
    DoubleMultiplicationSpecPtr castPtr = dynamic_pointer_cast<DoubleMultiplicationSpec>(specPtr);
    const std::vector<DoubleSpecPtr>& temp = castPtr->get_inputs();
    for (size_t i = 0; i < temp.size(); i++)
      getReferenceSpecs(references, temp[i]);

  } else if (dynamic_pointer_cast<DoubleDivisionSpec>(specPtr)) {
    DoubleDivisionSpecPtr castPtr = dynamic_pointer_cast<DoubleDivisionSpec>(specPtr);
    const std::vector<DoubleSpecPtr>& temp = castPtr->get_inputs();
    for (size_t i = 0; i < temp.size(); i++)
      getReferenceSpecs(references, temp[i]);

  } else if (dynamic_pointer_cast<DoubleXCoordOfSpec>(specPtr)) {
    DoubleXCoordOfSpecPtr castPtr = dynamic_pointer_cast<DoubleXCoordOfSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_vector());

  } else if (dynamic_pointer_cast<DoubleYCoordOfSpec>(specPtr)) {
    DoubleYCoordOfSpecPtr castPtr = dynamic_pointer_cast<DoubleYCoordOfSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_vector());
  
  } else if (dynamic_pointer_cast<DoubleZCoordOfSpec>(specPtr)) {
    DoubleZCoordOfSpecPtr castPtr = dynamic_pointer_cast<DoubleZCoordOfSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_vector());

  } else if (dynamic_pointer_cast<VectorDotSpec>(specPtr)) {
    VectorDotSpecPtr castPtr = dynamic_pointer_cast<VectorDotSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_lhs());
    getReferenceSpecs(references, castPtr->get_rhs());

  } else if (dynamic_pointer_cast<MinSpec>(specPtr)) {
    MinSpecPtr castPtr = dynamic_pointer_cast<MinSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_lhs());
    getReferenceSpecs(references, castPtr->get_rhs());

  } else if (dynamic_pointer_cast<MaxSpec>(specPtr)) {
    MaxSpecPtr castPtr = dynamic_pointer_cast<MaxSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_lhs());
    getReferenceSpecs(references, castPtr->get_rhs());

  } else if (dynamic_pointer_cast<AbsSpec>(specPtr)) {
    AbsSpecPtr castPtr = dynamic_pointer_cast<AbsSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_value());

  } else if (dynamic_pointer_cast<DoubleIfSpec>(specPtr)) {
    DoubleIfSpecPtr castPtr = dynamic_pointer_cast<DoubleIfSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_condition());
    getReferenceSpecs(references, castPtr->get_if());
    getReferenceSpecs(references, castPtr->get_else());

  } else if (dynamic_pointer_cast<FmodSpec>(specPtr)) {
    FmodSpecPtr castPtr = dynamic_pointer_cast<FmodSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_nominator());
    getReferenceSpecs(references, castPtr->get_denominator());

  } else if (dynamic_pointer_cast<SinSpec>(specPtr)) {
    SinSpecPtr castPtr = dynamic_pointer_cast<SinSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_value());

  } else if (dynamic_pointer_cast<CosSpec>(specPtr)) {
    CosSpecPtr castPtr = dynamic_pointer_cast<CosSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_value());

  } else if (dynamic_pointer_cast<TanSpec>(specPtr)) {
    TanSpecPtr castPtr = dynamic_pointer_cast<TanSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_value());

  } else if (dynamic_pointer_cast<ASinSpec>(specPtr)) {
    ASinSpecPtr castPtr = dynamic_pointer_cast<ASinSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_value());

  } else if (dynamic_pointer_cast<ACosSpec>(specPtr)) {
    ACosSpecPtr castPtr = dynamic_pointer_cast<ACosSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_value());

  } else if (dynamic_pointer_cast<ATanSpec>(specPtr)) {
    ATanSpecPtr castPtr = dynamic_pointer_cast<ATanSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_value());

  } else if (dynamic_pointer_cast<VectorInputSpec>(specPtr)) {
    VectorInputSpecPtr castPtr = dynamic_pointer_cast<VectorInputSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_name());

  } else if (dynamic_pointer_cast<VectorCachedSpec>(specPtr)) {
    VectorCachedSpecPtr castPtr = dynamic_pointer_cast<VectorCachedSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_vector());

  } else if (dynamic_pointer_cast<VectorConstructorSpec>(specPtr)) {
    VectorConstructorSpecPtr castPtr = dynamic_pointer_cast<VectorConstructorSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_x());
    getReferenceSpecs(references, castPtr->get_y());
    getReferenceSpecs(references, castPtr->get_z());

  } else if (dynamic_pointer_cast<VectorAdditionSpec>(specPtr)) {
    VectorAdditionSpecPtr castPtr = dynamic_pointer_cast<VectorAdditionSpec>(specPtr);
    const std::vector<VectorSpecPtr>& temp = castPtr->get_inputs();
    for (size_t i = 0; i < temp.size(); i++)
      getReferenceSpecs(references, temp[i]);

  } else if (dynamic_pointer_cast<VectorSubtractionSpec>(specPtr)) {
    VectorSubtractionSpecPtr castPtr = dynamic_pointer_cast<VectorSubtractionSpec>(specPtr);
    const std::vector<VectorSpecPtr>& temp = castPtr->get_inputs();
    for (size_t i = 0; i < temp.size(); i++)
      getReferenceSpecs(references, temp[i]);

  } else if (dynamic_pointer_cast<VectorReferenceSpec>(specPtr)) {
    VectorReferenceSpecPtr castPtr = dynamic_pointer_cast<VectorReferenceSpec>(specPtr);
    references.push_back(castPtr);

  } else if (dynamic_pointer_cast<VectorOriginOfSpec>(specPtr)) {
    VectorOriginOfSpecPtr castPtr = dynamic_pointer_cast<VectorOriginOfSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_frame());

  } else if (dynamic_pointer_cast<VectorFrameMultiplicationSpec>(specPtr)) {
    VectorFrameMultiplicationSpecPtr castPtr = dynamic_pointer_cast<VectorFrameMultiplicationSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_frame());
    getReferenceSpecs(references, castPtr->get_vector());

  } else if (dynamic_pointer_cast<VectorRotationMultiplicationSpec>(specPtr)) {
    VectorRotationMultiplicationSpecPtr castPtr = dynamic_pointer_cast<VectorRotationMultiplicationSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_rotation());
    getReferenceSpecs(references, castPtr->get_vector());

  } else if (dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(specPtr)) {
    VectorDoubleMultiplicationSpecPtr castPtr = dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_vector());
    getReferenceSpecs(references, castPtr->get_double());

  } else if (dynamic_pointer_cast<VectorRotationVectorSpec>(specPtr)) {
    VectorRotationVectorSpecPtr castPtr = dynamic_pointer_cast<VectorRotationVectorSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_rotation());

  } else if (dynamic_pointer_cast<VectorCrossSpec>(specPtr)) {
    VectorCrossSpecPtr castPtr = dynamic_pointer_cast<VectorCrossSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_lhs());
    getReferenceSpecs(references, castPtr->get_rhs());

  } else if (dynamic_pointer_cast<RotationInputSpec>(specPtr)) {
    RotationInputSpecPtr castPtr = dynamic_pointer_cast<RotationInputSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_name());

  } else if (dynamic_pointer_cast<AxisAngleSpec>(specPtr)) {
    AxisAngleSpecPtr castPtr = dynamic_pointer_cast<AxisAngleSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_axis());
    getReferenceSpecs(references, castPtr->get_angle());

  } else if (dynamic_pointer_cast<SlerpSpec>(specPtr)) {
    SlerpSpecPtr castPtr = dynamic_pointer_cast<SlerpSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_from());
    getReferenceSpecs(references, castPtr->get_to());
    getReferenceSpecs(references, castPtr->get_param());

  } else if (dynamic_pointer_cast<RotationReferenceSpec>(specPtr)) {
    RotationReferenceSpecPtr castPtr = dynamic_pointer_cast<RotationReferenceSpec>(specPtr);
    references.push_back(castPtr);
  
  } else if (dynamic_pointer_cast<InverseRotationSpec>(specPtr)) {
    InverseRotationSpecPtr castPtr = dynamic_pointer_cast<InverseRotationSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_rotation());

  } else if (dynamic_pointer_cast<RotationMultiplicationSpec>(specPtr)) {
    RotationMultiplicationSpecPtr castPtr = dynamic_pointer_cast<RotationMultiplicationSpec>(specPtr);
    const std::vector<RotationSpecPtr>& temp = castPtr->get_inputs();
    for (size_t i = 0; i < temp.size(); i++)
      getReferenceSpecs(references, temp[i]);

  } else if (dynamic_pointer_cast<FrameInputSpec>(specPtr)) {
    FrameInputSpecPtr castPtr = dynamic_pointer_cast<FrameInputSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_name());

  } else if (dynamic_pointer_cast<FrameCachedSpec>(specPtr)) {
    FrameCachedSpecPtr castPtr = dynamic_pointer_cast<FrameCachedSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_frame());

  } else if (dynamic_pointer_cast<FrameConstructorSpec>(specPtr)) {
    FrameConstructorSpecPtr castPtr = dynamic_pointer_cast<FrameConstructorSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_translation());
    getReferenceSpecs(references, castPtr->get_rotation());

  } else if (dynamic_pointer_cast<OrientationOfSpec>(specPtr)) {
    OrientationOfSpecPtr castPtr = dynamic_pointer_cast<OrientationOfSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_frame());

  } else if (dynamic_pointer_cast<FrameMultiplicationSpec>(specPtr)) {
    FrameMultiplicationSpecPtr castPtr = dynamic_pointer_cast<FrameMultiplicationSpec>(specPtr);
    const std::vector<FrameSpecPtr>& temp = castPtr->get_inputs();
    for (size_t i = 0; i < temp.size(); i++)
      getReferenceSpecs(references, temp[i]);

  } else if (dynamic_pointer_cast<FrameReferenceSpec>(specPtr)) {
    FrameReferenceSpecPtr castPtr = dynamic_pointer_cast<FrameReferenceSpec>(specPtr);
    references.push_back(castPtr);

  } else if (dynamic_pointer_cast<InverseFrameSpec>(specPtr)) {
    InverseFrameSpecPtr castPtr = dynamic_pointer_cast<InverseFrameSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_frame());

  } else if (dynamic_pointer_cast<ControllableConstraintSpec>(specPtr)) {
    ControllableConstraintSpecPtr castPtr = dynamic_pointer_cast<ControllableConstraintSpec>(specPtr);
    getReferenceSpecs(references, castPtr->lower_);
    getReferenceSpecs(references, castPtr->upper_);
    getReferenceSpecs(references, castPtr->weight_);
    getReferenceSpecs(references, castPtr->input_);

  } else if (dynamic_pointer_cast<SoftConstraintSpec>(specPtr)) {
    SoftConstraintSpecPtr castPtr = dynamic_pointer_cast<SoftConstraintSpec>(specPtr);
    getReferenceSpecs(references, castPtr->lower_);
    getReferenceSpecs(references, castPtr->upper_);
    getReferenceSpecs(references, castPtr->weight_);
    getReferenceSpecs(references, castPtr->expression_);
    getReferenceSpecs(references, castPtr->name_);

  } else if (dynamic_pointer_cast<HardConstraintSpec>(specPtr)) {
    HardConstraintSpecPtr castPtr = dynamic_pointer_cast<HardConstraintSpec>(specPtr);
    getReferenceSpecs(references, castPtr->lower_);
    getReferenceSpecs(references, castPtr->upper_);
    getReferenceSpecs(references, castPtr->expression_);

  } else if (dynamic_pointer_cast<ConcatStringSpec>(specPtr)) {
    ConcatStringSpecPtr castPtr = dynamic_pointer_cast<ConcatStringSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_lhs());
    getReferenceSpecs(references, castPtr->get_rhs());

  } else if (dynamic_pointer_cast<StringReferenceSpec>(specPtr)) {
    StringReferenceSpecPtr castPtr = dynamic_pointer_cast<StringReferenceSpec>(specPtr);
    references.push_back(castPtr);

  } else if (dynamic_pointer_cast<ListReferenceSpec>(specPtr)) {
    ListReferenceSpecPtr castPtr = dynamic_pointer_cast<ListReferenceSpec>(specPtr);
    references.push_back(castPtr);
  
  } else if (dynamic_pointer_cast<ConcatListSpec>(specPtr)) {
    ConcatListSpecPtr castPtr = dynamic_pointer_cast<ConcatListSpec>(specPtr);
    getReferenceSpecs(references, castPtr->get_lhs());
    getReferenceSpecs(references, castPtr->get_rhs());
  
  } else if (dynamic_pointer_cast<ConstListSpec>(specPtr)) {
    ConstListSpecPtr castPtr = dynamic_pointer_cast<ConstListSpec>(specPtr);
    auto content = castPtr->get_value();
    for (size_t i = 0; i < content.size(); i++)
      getReferenceSpecs(references, content[i]);
  
  }
  
  //else if (dynamic_pointer_cast<ConduitSpec>(specPtr)) {
    //references.push_back(dynamic_pointer_cast<ConduitSpec>(specPtr)->getSpec());
  //}
}

template<>
SpecPtr deepCopySpec(const SpecPtr specPtr) {
  if (dynamic_pointer_cast<DoubleSpec>(specPtr))
    return deepCopySpec(dynamic_pointer_cast<DoubleSpec>(specPtr));
  else if (dynamic_pointer_cast<VectorSpec>(specPtr))
    return deepCopySpec(dynamic_pointer_cast<VectorSpec>(specPtr));
  else if (dynamic_pointer_cast<RotationSpec>(specPtr))
    return deepCopySpec(dynamic_pointer_cast<RotationSpec>(specPtr));
  else if (dynamic_pointer_cast<FrameSpec>(specPtr))
    return deepCopySpec(dynamic_pointer_cast<FrameSpec>(specPtr));
  else if (dynamic_pointer_cast<StringSpec>(specPtr))
    return deepCopySpec(dynamic_pointer_cast<StringSpec>(specPtr));
  else if (dynamic_pointer_cast<ListSpec>(specPtr))
    return deepCopySpec(dynamic_pointer_cast<ListSpec>(specPtr));
  else if (dynamic_pointer_cast<SoftConstraintSpec>(specPtr))
    return deepCopySpec(dynamic_pointer_cast<SoftConstraintSpec>(specPtr));
  else if (dynamic_pointer_cast<HardConstraintSpec>(specPtr))
    return deepCopySpec(dynamic_pointer_cast<HardConstraintSpec>(specPtr));
  else if (dynamic_pointer_cast<ControllableConstraintSpec>(specPtr))
    return deepCopySpec(dynamic_pointer_cast<ControllableConstraintSpec>(specPtr));

  throw std::domain_error("Failed to copy spec!");
}

template<>
DoubleSpecPtr deepCopySpec(const DoubleSpecPtr specPtr) {
  if (dynamic_pointer_cast<DoubleInputSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleInputSpec>(specPtr));

  } else if (dynamic_pointer_cast<JointInputSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<JointInputSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleConstSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleConstSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleReferenceSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleReferenceSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleAdditionSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleAdditionSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleSubtractionSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleSubtractionSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleNormOfSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleNormOfSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleMultiplicationSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleMultiplicationSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleDivisionSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleDivisionSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleXCoordOfSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleXCoordOfSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleYCoordOfSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleYCoordOfSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleZCoordOfSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleZCoordOfSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorDotSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorDotSpec>(specPtr));

  } else if (dynamic_pointer_cast<MinSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<MinSpec>(specPtr));

  } else if (dynamic_pointer_cast<MaxSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<MaxSpec>(specPtr));

  } else if (dynamic_pointer_cast<AbsSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<AbsSpec>(specPtr));

  } else if (dynamic_pointer_cast<DoubleIfSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<DoubleIfSpec>(specPtr));

  } else if (dynamic_pointer_cast<FmodSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<FmodSpec>(specPtr));

  } else if (dynamic_pointer_cast<SinSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<SinSpec>(specPtr));

  } else if (dynamic_pointer_cast<CosSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<CosSpec>(specPtr));

  } else if (dynamic_pointer_cast<TanSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<TanSpec>(specPtr));

  } else if (dynamic_pointer_cast<ASinSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<ASinSpec>(specPtr));

  } else if (dynamic_pointer_cast<ACosSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<ACosSpec>(specPtr));

  } else if (dynamic_pointer_cast<ATanSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<ATanSpec>(specPtr));
  
  } else if (dynamic_pointer_cast<DoubleFunctionCallCache>(specPtr)) {
    auto ptr = dynamic_pointer_cast<DoubleFunctionCallCache>(specPtr);
    std::vector<SpecPtr> args;
    for (size_t i = 0; i < ptr->arguments.size(); i++)
      args.push_back(deepCopySpec(ptr->arguments[i]));

    return instance<DoubleFunctionCallCache>(args, ptr->functionDefinition);
  }

  throw std::domain_error("Failed to copy double spec!");
}

template<>
VectorSpecPtr deepCopySpec(const VectorSpecPtr specPtr) {
  if (dynamic_pointer_cast<VectorInputSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorInputSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorCachedSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorCachedSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorConstructorSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorConstructorSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorAdditionSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorAdditionSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorSubtractionSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorSubtractionSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorReferenceSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorReferenceSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorOriginOfSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorOriginOfSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorFrameMultiplicationSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorFrameMultiplicationSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorRotationMultiplicationSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorRotationMultiplicationSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorRotationVectorSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorRotationVectorSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorCrossSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<VectorCrossSpec>(specPtr));

  } else if (dynamic_pointer_cast<VectorFunctionCallCache>(specPtr)) {
    auto ptr = dynamic_pointer_cast<VectorFunctionCallCache>(specPtr);
    std::vector<SpecPtr> args;
    for (size_t i = 0; i < ptr->arguments.size(); i++)
      args.push_back(deepCopySpec(ptr->arguments[i]));

    return instance<VectorFunctionCallCache>(args, ptr->functionDefinition);
  }

  throw std::domain_error("Failed to copy vector spec!");
}

template<>
RotationSpecPtr deepCopySpec(const RotationSpecPtr specPtr) {
  if (dynamic_pointer_cast<RotationInputSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<RotationInputSpec>(specPtr));

  } else if (dynamic_pointer_cast<RotationQuaternionConstructorSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<RotationQuaternionConstructorSpec>(specPtr));

  } else if (dynamic_pointer_cast<AxisAngleSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<AxisAngleSpec>(specPtr));

  } else if (dynamic_pointer_cast<SlerpSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<SlerpSpec>(specPtr));

  } else if (dynamic_pointer_cast<RotationReferenceSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<RotationReferenceSpec>(specPtr));

  } else if (dynamic_pointer_cast<InverseRotationSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<InverseRotationSpec>(specPtr));

  } else if (dynamic_pointer_cast<RotationMultiplicationSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<RotationMultiplicationSpec>(specPtr));
  
  } else if (dynamic_pointer_cast<OrientationOfSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<OrientationOfSpec>(specPtr));
  
  } else if (dynamic_pointer_cast<RotationFunctionCallCache>(specPtr)) {
    auto ptr = dynamic_pointer_cast<RotationFunctionCallCache>(specPtr);
    std::vector<SpecPtr> args;
    for (size_t i = 0; i < ptr->arguments.size(); i++)
      args.push_back(deepCopySpec(ptr->arguments[i]));

    return instance<RotationFunctionCallCache>(args, ptr->functionDefinition);
  }

  throw std::domain_error("Failed to copy rotation spec!");
}

template<>
FrameSpecPtr deepCopySpec(const FrameSpecPtr specPtr) {
  if (dynamic_pointer_cast<FrameInputSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<FrameInputSpec>(specPtr));

  } else if (dynamic_pointer_cast<FrameCachedSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<FrameCachedSpec>(specPtr));

  } else if (dynamic_pointer_cast<FrameConstructorSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<FrameConstructorSpec>(specPtr));

  } else if (dynamic_pointer_cast<FrameMultiplicationSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<FrameMultiplicationSpec>(specPtr));

  } else if (dynamic_pointer_cast<FrameReferenceSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<FrameReferenceSpec>(specPtr));

  } else if (dynamic_pointer_cast<InverseFrameSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<InverseFrameSpec>(specPtr));
  } else if (dynamic_pointer_cast<FrameFunctionCallCache>(specPtr)) {
    auto ptr = dynamic_pointer_cast<FrameFunctionCallCache>(specPtr);
    std::vector<SpecPtr> args;
    for (size_t i = 0; i < ptr->arguments.size(); i++)
      args.push_back(deepCopySpec(ptr->arguments[i]));

    return instance<FrameFunctionCallCache>(args, ptr->functionDefinition);
  }

  throw std::domain_error("Failed to copy frame spec!");
}

template<>
StringSpecPtr deepCopySpec(const StringSpecPtr specPtr) {
  if (dynamic_pointer_cast<ConstStringSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<ConstStringSpec>(specPtr));

  } else if (dynamic_pointer_cast<ConcatStringSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<ConcatStringSpec>(specPtr));

  } else if (dynamic_pointer_cast<StringReferenceSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<StringReferenceSpec>(specPtr));
  } else if (dynamic_pointer_cast<StringFunctionCallCache>(specPtr)) {
    auto ptr = dynamic_pointer_cast<StringFunctionCallCache>(specPtr);
    std::vector<SpecPtr> args;
    for (size_t i = 0; i < ptr->arguments.size(); i++)
      args.push_back(deepCopySpec(ptr->arguments[i]));

    return instance<StringFunctionCallCache>(args, ptr->functionDefinition);
  }

  throw std::domain_error("Failed to copy string spec!"); 
}

template<>
ListSpecPtr deepCopySpec(const ListSpecPtr specPtr) {
  if (dynamic_pointer_cast<ConstListSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<ConstListSpec>(specPtr));

  } else if (dynamic_pointer_cast<ConcatListSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<ConcatListSpec>(specPtr));

  } else if (dynamic_pointer_cast<ListReferenceSpec>(specPtr)) {
    return deepCopySpec(dynamic_pointer_cast<ListReferenceSpec>(specPtr));
  
  } else if (dynamic_pointer_cast<ListFunctionCallCache>(specPtr)) {
    auto ptr = dynamic_pointer_cast<ListFunctionCallCache>(specPtr);
    std::vector<SpecPtr> args;
    for (size_t i = 0; i < ptr->arguments.size(); i++)
      args.push_back(deepCopySpec(ptr->arguments[i]));

    return instance<ListFunctionCallCache>(args, ptr->functionDefinition);
  }

  throw std::domain_error("Failed to copy list spec!"); 
}

template<>
DoubleInputSpecPtr deepCopySpec(const DoubleInputSpecPtr specPtr) {
  DoubleInputSpecPtr castPtr = dynamic_pointer_cast<DoubleInputSpec>(specPtr);
  return instance<DoubleInputSpec>(deepCopySpec(castPtr->get_name()));
}

template<>
JointInputSpecPtr deepCopySpec(const JointInputSpecPtr specPtr) {
  JointInputSpecPtr castPtr = dynamic_pointer_cast<JointInputSpec>(specPtr);
  return instance<JointInputSpec>(deepCopySpec(castPtr->get_name()));
}

template<>
DoubleConstSpecPtr deepCopySpec(const DoubleConstSpecPtr specPtr) {
  DoubleConstSpecPtr castPtr = dynamic_pointer_cast<DoubleConstSpec>(specPtr);
  return instance<DoubleConstSpec>(castPtr->get_value());
}

template<>
DoubleReferenceSpecPtr deepCopySpec(const DoubleReferenceSpecPtr specPtr) {
  DoubleReferenceSpecPtr castPtr = dynamic_pointer_cast<DoubleReferenceSpec>(specPtr);
  return instance<DoubleReferenceSpec>(castPtr->get_reference_name());
}

template<>
DoubleAdditionSpecPtr deepCopySpec(const DoubleAdditionSpecPtr specPtr) {
  DoubleAdditionSpecPtr castPtr = dynamic_pointer_cast<DoubleAdditionSpec>(specPtr);
  const std::vector<DoubleSpecPtr>& inputs = castPtr->get_inputs();
  std::vector<DoubleSpecPtr> duplInputs;
  for (size_t i = 0; i < inputs.size(); i++)
    duplInputs.push_back(deepCopySpec(inputs[i]));
  DoubleAdditionSpecPtr out(new DoubleAdditionSpec());
  out->set_inputs(duplInputs);
  return out;
}

template<>
DoubleSubtractionSpecPtr deepCopySpec(const DoubleSubtractionSpecPtr specPtr) {
  DoubleSubtractionSpecPtr castPtr = dynamic_pointer_cast<DoubleSubtractionSpec>(specPtr);
  const std::vector<DoubleSpecPtr>& inputs = castPtr->get_inputs();
  std::vector<DoubleSpecPtr> duplInputs;
  for (size_t i = 0; i < inputs.size(); i++)
    duplInputs.push_back(deepCopySpec(inputs[i]));
  DoubleSubtractionSpecPtr out(new DoubleSubtractionSpec());
  out->set_inputs(duplInputs);
  return out;
}

template<>
DoubleNormOfSpecPtr deepCopySpec(const DoubleNormOfSpecPtr specPtr) {
  DoubleNormOfSpecPtr castPtr = dynamic_pointer_cast<DoubleNormOfSpec>(specPtr);
  return instance<DoubleNormOfSpec>(deepCopySpec(castPtr->get_vector()));
}

template<>
DoubleMultiplicationSpecPtr deepCopySpec(const DoubleMultiplicationSpecPtr specPtr) {
  DoubleMultiplicationSpecPtr castPtr = dynamic_pointer_cast<DoubleMultiplicationSpec>(specPtr);
  const std::vector<DoubleSpecPtr>& inputs = castPtr->get_inputs();
  std::vector<DoubleSpecPtr> duplInputs;
  for (size_t i = 0; i < inputs.size(); i++)
    duplInputs.push_back(deepCopySpec(inputs[i]));
  DoubleMultiplicationSpecPtr out(new DoubleMultiplicationSpec());
  out->set_inputs(duplInputs);
  return out;
}

template<>
DoubleDivisionSpecPtr deepCopySpec(const DoubleDivisionSpecPtr specPtr) {
  DoubleDivisionSpecPtr castPtr = dynamic_pointer_cast<DoubleDivisionSpec>(specPtr);
  const std::vector<DoubleSpecPtr>& inputs = castPtr->get_inputs();
  std::vector<DoubleSpecPtr> duplInputs;
  for (size_t i = 0; i < inputs.size(); i++)
    duplInputs.push_back(deepCopySpec(inputs[i]));
  DoubleDivisionSpecPtr out(new DoubleDivisionSpec());
  out->set_inputs(duplInputs);
  return out;
}

template<>
DoubleXCoordOfSpecPtr deepCopySpec(const DoubleXCoordOfSpecPtr specPtr) {
  DoubleXCoordOfSpecPtr castPtr = dynamic_pointer_cast<DoubleXCoordOfSpec>(specPtr);
  return instance<DoubleXCoordOfSpec>(deepCopySpec(castPtr->get_vector()));
}

template<>
DoubleYCoordOfSpecPtr deepCopySpec(const DoubleYCoordOfSpecPtr specPtr) {
  DoubleYCoordOfSpecPtr castPtr = dynamic_pointer_cast<DoubleYCoordOfSpec>(specPtr);
  return instance<DoubleYCoordOfSpec>(deepCopySpec(castPtr->get_vector()));
}

template<>
DoubleZCoordOfSpecPtr deepCopySpec(const DoubleZCoordOfSpecPtr specPtr) {
  DoubleZCoordOfSpecPtr castPtr = dynamic_pointer_cast<DoubleZCoordOfSpec>(specPtr);
  return instance<DoubleZCoordOfSpec>(deepCopySpec(castPtr->get_vector()));
}

template<>
VectorDotSpecPtr deepCopySpec(const VectorDotSpecPtr specPtr) {
  VectorDotSpecPtr castPtr = dynamic_pointer_cast<VectorDotSpec>(specPtr);
  return instance<VectorDotSpec>(deepCopySpec(castPtr->get_lhs()), deepCopySpec(castPtr->get_rhs()));
}

template<>
MinSpecPtr deepCopySpec(const MinSpecPtr specPtr) {
  MinSpecPtr castPtr = dynamic_pointer_cast<MinSpec>(specPtr);
  return instance<MinSpec>(deepCopySpec(castPtr->get_lhs()), deepCopySpec(castPtr->get_rhs()));
}

template<>
MaxSpecPtr deepCopySpec(const MaxSpecPtr specPtr) {
  MaxSpecPtr castPtr = dynamic_pointer_cast<MaxSpec>(specPtr);
  return instance<MaxSpec>(deepCopySpec(castPtr->get_lhs()), deepCopySpec(castPtr->get_rhs()));
}

template<>
AbsSpecPtr deepCopySpec(const AbsSpecPtr specPtr) {
  AbsSpecPtr castPtr = dynamic_pointer_cast<AbsSpec>(specPtr);
  return instance<AbsSpec>(deepCopySpec(castPtr->get_value()));
}

template<>
DoubleIfSpecPtr deepCopySpec(const DoubleIfSpecPtr specPtr) {
  DoubleIfSpecPtr castPtr = dynamic_pointer_cast<DoubleIfSpec>(specPtr);
  return instance<DoubleIfSpec>(deepCopySpec(castPtr->get_condition()), deepCopySpec(castPtr->get_if()), deepCopySpec(castPtr->get_else()));
}

template<>
FmodSpecPtr deepCopySpec(const FmodSpecPtr specPtr) {
  FmodSpecPtr castPtr = dynamic_pointer_cast<FmodSpec>(specPtr);
  return instance<FmodSpec>(deepCopySpec(castPtr->get_nominator()), deepCopySpec(castPtr->get_denominator()));    
}

template<>
SinSpecPtr deepCopySpec(const SinSpecPtr specPtr) {
  SinSpecPtr castPtr = dynamic_pointer_cast<SinSpec>(specPtr);
  return instance<SinSpec>(deepCopySpec(castPtr->get_value()));
}

template<>
CosSpecPtr deepCopySpec(const CosSpecPtr specPtr) {
  CosSpecPtr castPtr = dynamic_pointer_cast<CosSpec>(specPtr);
  return instance<CosSpec>(deepCopySpec(castPtr->get_value()));
}

template<>
TanSpecPtr deepCopySpec(const TanSpecPtr specPtr) {
  TanSpecPtr castPtr = dynamic_pointer_cast<TanSpec>(specPtr);
  return instance<TanSpec>(deepCopySpec(castPtr->get_value()));
}

template<>
ASinSpecPtr deepCopySpec(const ASinSpecPtr specPtr) {
  ASinSpecPtr castPtr = dynamic_pointer_cast<ASinSpec>(specPtr);
  return instance<ASinSpec>(deepCopySpec(castPtr->get_value()));
}

template<>
ACosSpecPtr deepCopySpec(const ACosSpecPtr specPtr) {
  ACosSpecPtr castPtr = dynamic_pointer_cast<ACosSpec>(specPtr);
  return instance<ACosSpec>(deepCopySpec(castPtr->get_value()));
}

template<>
ATanSpecPtr deepCopySpec(const ATanSpecPtr specPtr) {
  ATanSpecPtr castPtr = dynamic_pointer_cast<ATanSpec>(specPtr);
  return instance<ATanSpec>(deepCopySpec(castPtr->get_value()));
}

template<>
VectorInputSpecPtr deepCopySpec(const VectorInputSpecPtr specPtr) {
  VectorInputSpecPtr castPtr = dynamic_pointer_cast<VectorInputSpec>(specPtr);
  return instance<VectorInputSpec>(deepCopySpec(castPtr->get_name()));
}

template<>
VectorCachedSpecPtr deepCopySpec(const VectorCachedSpecPtr specPtr) {
  VectorCachedSpecPtr castPtr = dynamic_pointer_cast<VectorCachedSpec>(specPtr);
  return instance<VectorCachedSpec>(deepCopySpec(castPtr->get_vector()));    
}

template<>
VectorConstructorSpecPtr deepCopySpec(const VectorConstructorSpecPtr specPtr) {
  VectorConstructorSpecPtr castPtr = dynamic_pointer_cast<VectorConstructorSpec>(specPtr);
  return instance<VectorConstructorSpec>(deepCopySpec(castPtr->get_x()), deepCopySpec(castPtr->get_y()), deepCopySpec(castPtr->get_z()));
}

template<>
VectorAdditionSpecPtr deepCopySpec(const VectorAdditionSpecPtr specPtr) {
  VectorAdditionSpecPtr castPtr = dynamic_pointer_cast<VectorAdditionSpec>(specPtr);
  const std::vector<VectorSpecPtr>& inputs = castPtr->get_inputs();
  std::vector<VectorSpecPtr> duplInputs;
  for (size_t i = 0; i < inputs.size(); i++)
    duplInputs.push_back(deepCopySpec(inputs[i]));
  VectorAdditionSpecPtr out(new VectorAdditionSpec());
  out->set_inputs(duplInputs);
  return out;
}

template<>
VectorSubtractionSpecPtr deepCopySpec(const VectorSubtractionSpecPtr specPtr) {
  VectorSubtractionSpecPtr castPtr = dynamic_pointer_cast<VectorSubtractionSpec>(specPtr);
  const std::vector<VectorSpecPtr>& inputs = castPtr->get_inputs();
  std::vector<VectorSpecPtr> duplInputs;
  for (size_t i = 0; i < inputs.size(); i++)
    duplInputs.push_back(deepCopySpec(inputs[i]));
  VectorSubtractionSpecPtr out(new VectorSubtractionSpec());
  out->set_inputs(duplInputs);
  return out;
}

template<>
VectorReferenceSpecPtr deepCopySpec(const VectorReferenceSpecPtr specPtr) {
  VectorReferenceSpecPtr castPtr = dynamic_pointer_cast<VectorReferenceSpec>(specPtr);
  return instance<VectorReferenceSpec>(castPtr->get_reference_name());
}

template<>
VectorOriginOfSpecPtr deepCopySpec(const VectorOriginOfSpecPtr specPtr) {
  VectorOriginOfSpecPtr castPtr = dynamic_pointer_cast<VectorOriginOfSpec>(specPtr);
  return instance<VectorOriginOfSpec>(deepCopySpec(castPtr->get_frame()));
}

template<>
VectorFrameMultiplicationSpecPtr deepCopySpec(const VectorFrameMultiplicationSpecPtr specPtr) {
  VectorFrameMultiplicationSpecPtr castPtr = dynamic_pointer_cast<VectorFrameMultiplicationSpec>(specPtr);
  return instance<VectorFrameMultiplicationSpec>(deepCopySpec(castPtr->get_frame()), deepCopySpec(castPtr->get_vector()));
}

template<>
VectorRotationMultiplicationSpecPtr deepCopySpec(const VectorRotationMultiplicationSpecPtr specPtr) {
  VectorRotationMultiplicationSpecPtr castPtr = dynamic_pointer_cast<VectorRotationMultiplicationSpec>(specPtr);
  return instance<VectorRotationMultiplicationSpec>(deepCopySpec(castPtr->get_rotation()), deepCopySpec(castPtr->get_vector()));
}

template<>
VectorDoubleMultiplicationSpecPtr deepCopySpec(const VectorDoubleMultiplicationSpecPtr specPtr) {
  VectorDoubleMultiplicationSpecPtr castPtr = dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(specPtr);
  return instance<VectorDoubleMultiplicationSpec>(deepCopySpec(castPtr->get_vector()), deepCopySpec(castPtr->get_double()));
}

template<>
VectorRotationVectorSpecPtr deepCopySpec(const VectorRotationVectorSpecPtr specPtr) {
  VectorRotationVectorSpecPtr castPtr = dynamic_pointer_cast<VectorRotationVectorSpec>(specPtr);
  return instance<VectorRotationVectorSpec>(deepCopySpec(castPtr->get_rotation()));
}

template<>
VectorCrossSpecPtr deepCopySpec(const VectorCrossSpecPtr specPtr) {
  VectorCrossSpecPtr castPtr = dynamic_pointer_cast<VectorCrossSpec>(specPtr);
  return instance<VectorCrossSpec>(deepCopySpec(castPtr->get_lhs()), deepCopySpec(castPtr->get_rhs()));
}

template<>
RotationInputSpecPtr deepCopySpec(const RotationInputSpecPtr specPtr) {
  RotationInputSpecPtr castPtr = dynamic_pointer_cast<RotationInputSpec>(specPtr);
  return instance<RotationInputSpec>(deepCopySpec(castPtr->get_name()));
}

template<>
RotationQuaternionConstructorSpecPtr deepCopySpec(const RotationQuaternionConstructorSpecPtr specPtr) {
  RotationQuaternionConstructorSpecPtr castPtr = dynamic_pointer_cast<RotationQuaternionConstructorSpec>(specPtr);
  return instance<RotationQuaternionConstructorSpec>(castPtr->get_x(), castPtr->get_y(), castPtr->get_z(),castPtr->get_w());
}

template<>
AxisAngleSpecPtr deepCopySpec(const AxisAngleSpecPtr specPtr) {
  AxisAngleSpecPtr castPtr = dynamic_pointer_cast<AxisAngleSpec>(specPtr);
  return instance<AxisAngleSpec>(deepCopySpec(castPtr->get_axis()), deepCopySpec(castPtr->get_angle()));
}

template<>
SlerpSpecPtr deepCopySpec(const SlerpSpecPtr specPtr) {
  SlerpSpecPtr castPtr = dynamic_pointer_cast<SlerpSpec>(specPtr);
  return instance<SlerpSpec>(deepCopySpec(castPtr->get_from()), deepCopySpec(castPtr->get_to()), deepCopySpec(castPtr->get_param()));
}

template<>
RotationReferenceSpecPtr deepCopySpec(const RotationReferenceSpecPtr specPtr) {
  RotationReferenceSpecPtr castPtr = dynamic_pointer_cast<RotationReferenceSpec>(specPtr);
  return instance<RotationReferenceSpec>(castPtr->get_reference_name());
}

template<>
InverseRotationSpecPtr deepCopySpec(const InverseRotationSpecPtr specPtr) {
  InverseRotationSpecPtr castPtr = dynamic_pointer_cast<InverseRotationSpec>(specPtr);
  return instance<InverseRotationSpec>(deepCopySpec(castPtr->get_rotation()));
}

template<>
RotationMultiplicationSpecPtr deepCopySpec(const RotationMultiplicationSpecPtr specPtr) {
  RotationMultiplicationSpecPtr castPtr = dynamic_pointer_cast<RotationMultiplicationSpec>(specPtr);
  const std::vector<RotationSpecPtr>& inputs = castPtr->get_inputs();
  std::vector<RotationSpecPtr> duplInputs;
  for (size_t i = 0; i < inputs.size(); i++)
    duplInputs.push_back(deepCopySpec(inputs[i]));
  RotationMultiplicationSpecPtr out(new RotationMultiplicationSpec());
  out->set_inputs(duplInputs);
  return out;
}

template<>
FrameInputSpecPtr deepCopySpec(const FrameInputSpecPtr specPtr) {
  FrameInputSpecPtr castPtr = dynamic_pointer_cast<FrameInputSpec>(specPtr);
  return instance<FrameInputSpec>(deepCopySpec(castPtr->get_name()));
}

template<>
FrameCachedSpecPtr deepCopySpec(const FrameCachedSpecPtr specPtr) {
  FrameCachedSpecPtr castPtr = dynamic_pointer_cast<FrameCachedSpec>(specPtr);
  return instance<FrameCachedSpec>(deepCopySpec(castPtr->get_frame()));
}

template<>
FrameConstructorSpecPtr deepCopySpec(const FrameConstructorSpecPtr specPtr) {
  FrameConstructorSpecPtr castPtr = dynamic_pointer_cast<FrameConstructorSpec>(specPtr);
  return instance<FrameConstructorSpec>(deepCopySpec(castPtr->get_translation()), deepCopySpec(castPtr->get_rotation()));
}

template<>
OrientationOfSpecPtr deepCopySpec(const OrientationOfSpecPtr specPtr) {
  OrientationOfSpecPtr castPtr = dynamic_pointer_cast<OrientationOfSpec>(specPtr);
  return instance<OrientationOfSpec>(deepCopySpec(castPtr->get_frame()));
}

template<>
FrameMultiplicationSpecPtr deepCopySpec(const FrameMultiplicationSpecPtr specPtr) {
  FrameMultiplicationSpecPtr castPtr = dynamic_pointer_cast<FrameMultiplicationSpec>(specPtr);
  const std::vector<FrameSpecPtr>& inputs = castPtr->get_inputs();
  std::vector<FrameSpecPtr> duplInputs;
  for (size_t i = 0; i < inputs.size(); i++)
    duplInputs.push_back(deepCopySpec(inputs[i]));
  FrameMultiplicationSpecPtr out(new FrameMultiplicationSpec());
  out->set_inputs(duplInputs);
  return out;
}

template<>
FrameReferenceSpecPtr deepCopySpec(const FrameReferenceSpecPtr specPtr) {
  FrameReferenceSpecPtr castPtr = dynamic_pointer_cast<FrameReferenceSpec>(specPtr);
  return instance<FrameReferenceSpec>(castPtr->get_reference_name());
}

template<>
InverseFrameSpecPtr deepCopySpec(const InverseFrameSpecPtr specPtr) {
  InverseFrameSpecPtr castPtr = dynamic_pointer_cast<InverseFrameSpec>(specPtr);
  return instance<InverseFrameSpec>(deepCopySpec(castPtr->get_frame()));
}

template<>
ControllableConstraintSpecPtr deepCopySpec(const ControllableConstraintSpecPtr specPtr) {
  ControllableConstraintSpecPtr castPtr = dynamic_pointer_cast<ControllableConstraintSpec>(specPtr);
  return instance<ControllableConstraintSpec>(deepCopySpec(castPtr->lower_), deepCopySpec(castPtr->upper_), deepCopySpec(castPtr->weight_), deepCopySpec(castPtr->input_));
}

template<>
SoftConstraintSpecPtr deepCopySpec(const SoftConstraintSpecPtr specPtr) {
  SoftConstraintSpecPtr castPtr = dynamic_pointer_cast<SoftConstraintSpec>(specPtr);
  return instance<SoftConstraintSpec>(deepCopySpec(castPtr->lower_), deepCopySpec(castPtr->upper_), deepCopySpec(castPtr->weight_), deepCopySpec(castPtr->expression_), deepCopySpec(castPtr->name_));
}

template<>
HardConstraintSpecPtr deepCopySpec(const HardConstraintSpecPtr specPtr) {
  HardConstraintSpecPtr castPtr = dynamic_pointer_cast<HardConstraintSpec>(specPtr);
  return instance<HardConstraintSpec>(deepCopySpec(castPtr->lower_), deepCopySpec(castPtr->upper_), deepCopySpec(castPtr->expression_));
}

template<>
ConstStringSpecPtr deepCopySpec(const ConstStringSpecPtr specPtr) {
  ConstStringSpecPtr castPtr = dynamic_pointer_cast<ConstStringSpec>(specPtr);
  return instance<ConstStringSpec>(castPtr->get_value());
}

template<>
ConcatStringSpecPtr deepCopySpec(const ConcatStringSpecPtr specPtr) {
  ConcatStringSpecPtr castPtr = dynamic_pointer_cast<ConcatStringSpec>(specPtr);
  return instance<ConcatStringSpec>(deepCopySpec(castPtr->get_lhs()), deepCopySpec(castPtr->get_rhs()));
}

template<>
StringReferenceSpecPtr deepCopySpec(const StringReferenceSpecPtr specPtr) {
  StringReferenceSpecPtr castPtr = dynamic_pointer_cast<StringReferenceSpec>(specPtr);
  return instance<StringReferenceSpec>(castPtr->get_reference_name(), castPtr->getScope());
}

template<>
ConstListSpecPtr deepCopySpec(const ConstListSpecPtr specPtr) {
  ConstListSpecPtr castPtr = dynamic_pointer_cast<ConstListSpec>(specPtr);
  std::vector<SpecPtr> values = castPtr->get_value();
  std::vector<SpecPtr> duplValues;
  for (size_t i = 0; i < values.size(); i++)
    duplValues.push_back(deepCopySpec(values[i]));

  return instance<ConstListSpec>(duplValues);
}

template<>
ConcatListSpecPtr deepCopySpec(const ConcatListSpecPtr specPtr) {
  ConcatListSpecPtr castPtr = dynamic_pointer_cast<ConcatListSpec>(specPtr);

  return instance<ConcatListSpec>(deepCopySpec(castPtr->get_lhs()), deepCopySpec(castPtr->get_rhs()));
}

template<>
ListReferenceSpecPtr deepCopySpec(const ListReferenceSpecPtr specPtr) {
  ListReferenceSpecPtr castPtr = dynamic_pointer_cast<ListReferenceSpec>(specPtr);
  return instance<ListReferenceSpec>(castPtr->get_reference_name(), castPtr->getScope(), castPtr->innerType());
}

void specToString(std::string& out, const SpecPtr& specPtr) {
  if (specPtr)
    specToString(out, *specPtr);
}

void specToString(std::string& out, const Spec& specPtr) {
  if (dynamic_cast<const DoubleInputSpec*>(&specPtr)) {
    const DoubleInputSpec* castPtr = dynamic_cast<const DoubleInputSpec*>(&specPtr);
    out += AdvancedScope::fInScalar + "(\"" + castPtr->get_name()->get_value() + "\")";

  } else if (dynamic_cast<const JointInputSpec*>(&specPtr)) {
    const JointInputSpec* castPtr = dynamic_cast<const JointInputSpec*>(&specPtr);
    out += AdvancedScope::fInJoint + "(\"" + castPtr->get_name()->get_value() + "\")";
    
  } else if (dynamic_cast<const DoubleConstSpec*>(&specPtr)) {
    const DoubleConstSpec* castPtr = dynamic_cast<const DoubleConstSpec*>(&specPtr);
    out += std::to_string(castPtr->get_value());

  } else if (dynamic_cast<const DoubleReferenceSpec*>(&specPtr)) {
    const DoubleReferenceSpec* castPtr = dynamic_cast<const DoubleReferenceSpec*>(&specPtr);
    out += castPtr->get_reference_name();

  } else if (dynamic_cast<const DoubleAdditionSpec*>(&specPtr)) {
    const DoubleAdditionSpec* castPtr = dynamic_cast<const DoubleAdditionSpec*>(&specPtr);
    const std::vector<DoubleSpecPtr>& temp = castPtr->get_inputs();
    specToString(out, temp[0]);
    for (size_t i = 1; i < temp.size(); i++) {
      out += '+';
      specToString(out, temp[i]);
    }

  } else if (dynamic_cast<const DoubleSubtractionSpec*>(&specPtr)) {
    const DoubleSubtractionSpec* castPtr = dynamic_cast<const DoubleSubtractionSpec*>(&specPtr);
    const std::vector<DoubleSpecPtr>& temp = castPtr->get_inputs();
    specToString(out, temp[0]);
    for (size_t i = 1; i < temp.size(); i++) {
      out += '-';
      bool useBrackets = (dynamic_pointer_cast<DoubleAdditionSpec>(temp[i]) || dynamic_pointer_cast<DoubleSubtractionSpec>(temp[i]));
      if (useBrackets)
        out += '(';
      specToString(out, temp[i]);
      if (useBrackets)
        out += ')';
    }

  } else if (dynamic_cast<const DoubleNormOfSpec*>(&specPtr)) {
    const DoubleNormOfSpec* castPtr = dynamic_cast<const DoubleNormOfSpec*>(&specPtr);
    out += AdvancedScope::fNorm + '(';
    specToString(out, castPtr->get_vector());
    out += ')';

  } else if (dynamic_cast<const DoubleMultiplicationSpec*>(&specPtr)) {
    const DoubleMultiplicationSpec* castPtr = dynamic_cast<const DoubleMultiplicationSpec*>(&specPtr);
    const std::vector<DoubleSpecPtr>& temp = castPtr->get_inputs();
    bool useBrackets = (dynamic_pointer_cast<DoubleAdditionSpec>(temp[0]) || dynamic_pointer_cast<DoubleSubtractionSpec>(temp[0]));
    if (useBrackets)
      out += '(';
    specToString(out, temp[0]);
    if (useBrackets)
      out += ')';

    for (size_t i = 1; i < temp.size(); i++) {
      out += '*';
      useBrackets = (dynamic_pointer_cast<DoubleAdditionSpec>(temp[i]) 
                          || dynamic_pointer_cast<DoubleDivisionSpec>(temp[i]) 
                          || dynamic_pointer_cast<VectorDotSpec>(temp[i]) 
                          || dynamic_pointer_cast<DoubleSubtractionSpec>(temp[i]));
    if (useBrackets)
      out += '(';
    specToString(out, temp[i]);
    if (useBrackets)
      out += ')';

    }

  } else if (dynamic_cast<const DoubleDivisionSpec*>(&specPtr)) {
    const DoubleDivisionSpec* castPtr = dynamic_cast<const DoubleDivisionSpec*>(&specPtr);
    const std::vector<DoubleSpecPtr>& temp = castPtr->get_inputs();
    bool useBrackets = (dynamic_pointer_cast<DoubleAdditionSpec>(temp[0]) || dynamic_pointer_cast<DoubleSubtractionSpec>(temp[0]));
    if (useBrackets)
      out += '(';
    specToString(out, temp[0]);
    if (useBrackets)
      out += ')';

    for (size_t i = 1; i < temp.size(); i++) {
      out += '/';
      useBrackets = (dynamic_pointer_cast<DoubleAdditionSpec>(temp[i]) 
                          || dynamic_pointer_cast<DoubleDivisionSpec>(temp[i]) 
                          || dynamic_pointer_cast<DoubleMultiplicationSpec>(temp[i])
                          || dynamic_pointer_cast<VectorDotSpec>(temp[i]) 
                          || dynamic_pointer_cast<DoubleSubtractionSpec>(temp[i]));
    if (useBrackets)
      out += '(';
    specToString(out, temp[i]);
    if (useBrackets)
      out += ')';
    
    }

  } else if (dynamic_cast<const DoubleXCoordOfSpec*>(&specPtr)) {
    const DoubleXCoordOfSpec* castPtr = dynamic_cast<const DoubleXCoordOfSpec*>(&specPtr);
    bool useBrackets = dynamic_pointer_cast<VectorAdditionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorSubtractionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorFrameMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorRotationMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(castPtr->get_vector());
    if (useBrackets)
      out += '(';
    specToString(out, castPtr->get_vector());
    if (useBrackets)
      out += ')';

    out += ".x";

  } else if (dynamic_cast<const DoubleYCoordOfSpec*>(&specPtr)) {
    const DoubleYCoordOfSpec* castPtr = dynamic_cast<const DoubleYCoordOfSpec*>(&specPtr);
    bool useBrackets = dynamic_pointer_cast<VectorAdditionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorSubtractionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorFrameMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorRotationMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(castPtr->get_vector());
    if (useBrackets)
      out += '(';
    specToString(out, castPtr->get_vector());
    if (useBrackets)
      out += ')';
    out += ".y";

  } else if (dynamic_cast<const DoubleZCoordOfSpec*>(&specPtr)) {
    const DoubleZCoordOfSpec* castPtr = dynamic_cast<const DoubleZCoordOfSpec*>(&specPtr);
    bool useBrackets = dynamic_pointer_cast<VectorAdditionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorSubtractionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorFrameMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorRotationMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(castPtr->get_vector());
    if (useBrackets)
      out += '(';
    specToString(out, castPtr->get_vector());
    if (useBrackets)
      out += ')';
    out += ".z";

  } else if (dynamic_cast<const VectorDotSpec*>(&specPtr)) {
    const VectorDotSpec* castPtr = dynamic_cast<const VectorDotSpec*>(&specPtr);
    bool useBracketsLhs = !!dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(castPtr->get_lhs());
    bool useBracketsRhs = !!dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(castPtr->get_rhs());
    if (useBracketsLhs)
      out += '(';
    specToString(out, castPtr->get_lhs());
    if (useBracketsLhs)
      out += ')';

    out += '*';

    if (useBracketsRhs)
      out += '(';
    specToString(out, castPtr->get_rhs());
    if (useBracketsRhs)
      out += ')';

  } else if (dynamic_cast<const MinSpec*>(&specPtr)) {
    const MinSpec* castPtr = dynamic_cast<const MinSpec*>(&specPtr);
    out += AdvancedScope::fMin + '(';
    specToString(out, castPtr->get_lhs());
    out+=',';
    specToString(out, castPtr->get_rhs());
    out+=')';

  } else if (dynamic_cast<const MaxSpec*>(&specPtr)) {
    const MaxSpec* castPtr = dynamic_cast<const MaxSpec*>(&specPtr);
    out += AdvancedScope::fMax + '(';
    specToString(out, castPtr->get_lhs());
    out+=',';
    specToString(out, castPtr->get_rhs());
    out+=')';

  } else if (dynamic_cast<const AbsSpec*>(&specPtr)) {
    const AbsSpec* castPtr = dynamic_cast<const AbsSpec*>(&specPtr);
    out += AdvancedScope::fAbs + '(';
    specToString(out, castPtr->get_value());
    out+=')';

  } else if (dynamic_cast<const DoubleIfSpec*>(&specPtr)) {
    const DoubleIfSpec* castPtr = dynamic_cast<const DoubleIfSpec*>(&specPtr);
    out += AdvancedScope::fIf + '(';
    specToString(out, castPtr->get_condition());
    out+=',';
    specToString(out, castPtr->get_if());
    out+=',';
    specToString(out, castPtr->get_else());
    out+=')';

  } else if (dynamic_cast<const FmodSpec*>(&specPtr)) {
    const FmodSpec* castPtr = dynamic_cast<const FmodSpec*>(&specPtr);
    out += AdvancedScope::fFMod + '(';
    specToString(out, castPtr->get_nominator());
    out+=',';
    specToString(out, castPtr->get_denominator());
    out+=')';

  } else if (dynamic_cast<const SinSpec*>(&specPtr)) {
    const SinSpec* castPtr = dynamic_cast<const SinSpec*>(&specPtr);
    out += AdvancedScope::fSin + '(';
    specToString(out, castPtr->get_value());
    out+=')';

  } else if (dynamic_cast<const CosSpec*>(&specPtr)) {
    const CosSpec* castPtr = dynamic_cast<const CosSpec*>(&specPtr);
    out += AdvancedScope::fCos + '(';
    specToString(out, castPtr->get_value());
    out+=')';

  } else if (dynamic_cast<const TanSpec*>(&specPtr)) {
    const TanSpec* castPtr = dynamic_cast<const TanSpec*>(&specPtr);
    out += AdvancedScope::fTan + '(';
    specToString(out, castPtr->get_value());
    out+=')';

  } else if (dynamic_cast<const ASinSpec*>(&specPtr)) {
    const ASinSpec* castPtr = dynamic_cast<const ASinSpec*>(&specPtr);
    out += AdvancedScope::fASin + '(';
    specToString(out, castPtr->get_value());
    out+=')';

  } else if (dynamic_cast<const ACosSpec*>(&specPtr)) {
    const ACosSpec* castPtr = dynamic_cast<const ACosSpec*>(&specPtr);
    out += AdvancedScope::fACos + '(';
    specToString(out, castPtr->get_value());
    out+=')';

  } else if (dynamic_cast<const ATanSpec*>(&specPtr)) {
    const ATanSpec* castPtr = dynamic_cast<const ATanSpec*>(&specPtr);
    out += AdvancedScope::fATan + '(';
    specToString(out, castPtr->get_value());
    out+=')';

  } else if (dynamic_cast<const VectorInputSpec*>(&specPtr)) {
    const VectorInputSpec* castPtr = dynamic_cast<const VectorInputSpec*>(&specPtr);
    out += AdvancedScope::fInVec3 + "(\"" + castPtr->get_name()->get_value() + "\")";

  } else if (dynamic_cast<const VectorCachedSpec*>(&specPtr)) {
    const VectorCachedSpec* castPtr = dynamic_cast<const VectorCachedSpec*>(&specPtr);
    specToString(out, castPtr->get_vector());

  } else if (dynamic_cast<const VectorConstructorSpec*>(&specPtr)) {
    const VectorConstructorSpec* castPtr = dynamic_cast<const VectorConstructorSpec*>(&specPtr);
    out += AdvancedScope::fCVec3 + '(';
    specToString(out, castPtr->get_x());
    out += ',';
    specToString(out, castPtr->get_y());
    out += ',';
    specToString(out, castPtr->get_z());
    out += ')';

  } else if (dynamic_cast<const VectorAdditionSpec*>(&specPtr)) {
    const VectorAdditionSpec* castPtr = dynamic_cast<const VectorAdditionSpec*>(&specPtr);
    const std::vector<VectorSpecPtr>& temp = castPtr->get_inputs();
    specToString(out, temp[0]);
    for (size_t i = 1; i < temp.size(); i++) {
      out += '+';
      specToString(out, temp[i]);
    }

  } else if (dynamic_cast<const VectorSubtractionSpec*>(&specPtr)) {
    const VectorSubtractionSpec* castPtr = dynamic_cast<const VectorSubtractionSpec*>(&specPtr);
    const std::vector<VectorSpecPtr>& temp = castPtr->get_inputs();
    specToString(out, temp[0]);
    for (size_t i = 1; i < temp.size(); i++) {
      out += '-';
      bool useBrackets = (dynamic_pointer_cast<VectorAdditionSpec>(temp[i]) || dynamic_pointer_cast<VectorSubtractionSpec>(temp[i]));
      if (useBrackets)
        out += '(';
      specToString(out, temp[i]);
      if (useBrackets)
        out += ')';
    }

  } else if (dynamic_cast<const VectorReferenceSpec*>(&specPtr)) {
    const VectorReferenceSpec* castPtr = dynamic_cast<const VectorReferenceSpec*>(&specPtr);
    out += castPtr->get_reference_name();

  } else if (dynamic_cast<const VectorOriginOfSpec*>(&specPtr)) {
    const VectorOriginOfSpec* castPtr = dynamic_cast<const VectorOriginOfSpec*>(&specPtr);
    bool useBrackets = !!dynamic_pointer_cast<FrameMultiplicationSpec>(castPtr->get_frame());

    if (useBrackets)
      out += '(';
    specToString(out, castPtr->get_frame());
    if (useBrackets)
      out += ')';
    out += ".pos";

  } else if (dynamic_cast<const VectorFrameMultiplicationSpec*>(&specPtr)) {
    const VectorFrameMultiplicationSpec* castPtr = dynamic_cast<const VectorFrameMultiplicationSpec*>(&specPtr);
    bool useBrackets = dynamic_pointer_cast<VectorAdditionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorSubtractionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorFrameMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorRotationMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(castPtr->get_vector());
    specToString(out, castPtr->get_frame());
    out += '*';
    if (useBrackets)
      out += '(';
    specToString(out, castPtr->get_vector());
    if (useBrackets)
      out += ')';

  } else if (dynamic_cast<const VectorRotationMultiplicationSpec*>(&specPtr)) {
    const VectorRotationMultiplicationSpec* castPtr = dynamic_cast<const VectorRotationMultiplicationSpec*>(&specPtr);
    bool useBrackets = dynamic_pointer_cast<VectorAdditionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorSubtractionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorFrameMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorRotationMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(castPtr->get_vector());
    specToString(out, castPtr->get_rotation());
    out += '*';
    if (useBrackets)
      out += '(';
    specToString(out, castPtr->get_vector());
    if (useBrackets)
      out += ')';

  } else if (dynamic_cast<const VectorDoubleMultiplicationSpec*>(&specPtr)) {
    const VectorDoubleMultiplicationSpec* castPtr = dynamic_cast<const VectorDoubleMultiplicationSpec*>(&specPtr);
    bool useBracketsVec = dynamic_pointer_cast<VectorAdditionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorSubtractionSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorFrameMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorRotationMultiplicationSpec>(castPtr->get_vector())
                    || dynamic_pointer_cast<VectorDoubleMultiplicationSpec>(castPtr->get_vector());
    bool useBracketsScalar = (dynamic_pointer_cast<DoubleAdditionSpec>(castPtr->get_double())  
                          || dynamic_pointer_cast<DoubleSubtractionSpec>(castPtr->get_double()));
    if (useBracketsScalar)
      out += '(';
    specToString(out, castPtr->get_double());
    if (useBracketsScalar)
      out += ')';

    if (useBracketsVec)
      out += '(';
    specToString(out, castPtr->get_vector());
    if (useBracketsVec)
      out += ')';

  } else if (dynamic_cast<const VectorRotationVectorSpec*>(&specPtr)) {
    const VectorRotationVectorSpec* castPtr = dynamic_cast<const VectorRotationVectorSpec*>(&specPtr);
    out+= AdvancedScope::fCVec3 + '(';
    specToString(out, castPtr->get_rotation());
    out+= ')';

  } else if (dynamic_cast<const VectorCrossSpec*>(&specPtr)) {
    const VectorCrossSpec* castPtr = dynamic_cast<const VectorCrossSpec*>(&specPtr);
    out+= AdvancedScope::fCross + '(';
    specToString(out, castPtr->get_lhs());
    out+=',';
    specToString(out, castPtr->get_rhs());
    out+= ')';

  } else if (dynamic_cast<const RotationInputSpec*>(&specPtr)) {
    const RotationInputSpec* castPtr = dynamic_cast<const RotationInputSpec*>(&specPtr);
    out += AdvancedScope::fInRotation + "(\"" + castPtr->get_name()->get_value() + "\")";

  } else if (dynamic_cast<const RotationQuaternionConstructorSpec*>(&specPtr)) {
    const RotationQuaternionConstructorSpec* castPtr = dynamic_cast<const RotationQuaternionConstructorSpec*>(&specPtr);
    out += AdvancedScope::fCRotation + '(' + std::to_string(castPtr->get_x()) + ',' + std::to_string(castPtr->get_y()) + ',' + std::to_string(castPtr->get_z()) + ',' + std::to_string(castPtr->get_w()) + ')';

  } else if (dynamic_cast<const AxisAngleSpec*>(&specPtr)) {
    const AxisAngleSpec* castPtr = dynamic_cast<const AxisAngleSpec*>(&specPtr);
    out += AdvancedScope::fCRotation + '(';
    specToString(out, castPtr->get_axis());
    out += ',';
    specToString(out, castPtr->get_angle());
    out += ')';

  } else if (dynamic_cast<const SlerpSpec*>(&specPtr)) {
    const SlerpSpec* castPtr = dynamic_cast<const SlerpSpec*>(&specPtr);
    out += AdvancedScope::fSlerp + '(';
    specToString(out, castPtr->get_from());
    out += ',';
    specToString(out, castPtr->get_to());
    out += ',';
    specToString(out, castPtr->get_param());
    out += ')';

  } else if (dynamic_cast<const RotationReferenceSpec*>(&specPtr)) {
    const RotationReferenceSpec* castPtr = dynamic_cast<const RotationReferenceSpec*>(&specPtr);
    out += castPtr->get_reference_name();

  } else if (dynamic_cast<const InverseRotationSpec*>(&specPtr)) {
    const InverseRotationSpec* castPtr = dynamic_cast<const InverseRotationSpec*>(&specPtr);
    out += AdvancedScope::fInvert + '(';
    specToString(out, castPtr->get_rotation());
    out += ')';

  } else if (dynamic_cast<const RotationMultiplicationSpec*>(&specPtr)) {
    const RotationMultiplicationSpec* castPtr = dynamic_cast<const RotationMultiplicationSpec*>(&specPtr);
    const std::vector<RotationSpecPtr>& temp = castPtr->get_inputs();
    specToString(out, temp[0]);

    for (size_t i = 1; i < temp.size(); i++) {
      out += '*';
      specToString(out, temp[i]);
    }

  } else if (dynamic_cast<const FrameInputSpec*>(&specPtr)) {
    const FrameInputSpec* castPtr = dynamic_cast<const FrameInputSpec*>(&specPtr);
    out += AdvancedScope::fInFrame + "(\"" + castPtr->get_name()->get_value() + "\")";

  } else if (dynamic_cast<const FrameCachedSpec*>(&specPtr)) {
    const FrameCachedSpec* castPtr = dynamic_cast<const FrameCachedSpec*>(&specPtr);
    specToString(out, castPtr->get_frame());

  } else if (dynamic_cast<const FrameConstructorSpec*>(&specPtr)) {
    const FrameConstructorSpec* castPtr = dynamic_cast<const FrameConstructorSpec*>(&specPtr);
    out += AdvancedScope::fCFrame + '(';
    specToString(out, castPtr->get_rotation());
    out += ',';
    specToString(out, castPtr->get_translation());
    out += ')';

  } else if (dynamic_cast<const OrientationOfSpec*>(&specPtr)) {
    const OrientationOfSpec* castPtr = dynamic_cast<const OrientationOfSpec*>(&specPtr);
    bool useBrackets = !!dynamic_pointer_cast<FrameMultiplicationSpec>(castPtr->get_frame());

    if (useBrackets)
      out += '(';
    specToString(out, castPtr->get_frame());
    if (useBrackets)
      out += ')';
    out += ".rot";

  } else if (dynamic_cast<const FrameMultiplicationSpec*>(&specPtr)) {
    const FrameMultiplicationSpec* castPtr = dynamic_cast<const FrameMultiplicationSpec*>(&specPtr);
    const std::vector<FrameSpecPtr>& temp = castPtr->get_inputs();
    specToString(out, temp[0]);

    for (size_t i = 1; i < temp.size(); i++) {
      out += '*';
      specToString(out, temp[i]);
    }

  } else if (dynamic_cast<const FrameReferenceSpec*>(&specPtr)) {
    const FrameReferenceSpec* castPtr = dynamic_cast<const FrameReferenceSpec*>(&specPtr);
    out += castPtr->get_reference_name();

  } else if (dynamic_cast<const InverseFrameSpec*>(&specPtr)) {
    const InverseFrameSpec* castPtr = dynamic_cast<const InverseFrameSpec*>(&specPtr);
    out += AdvancedScope::fInvert + '(';
    specToString(out, castPtr->get_frame());
    out += ')';

  } else if (dynamic_cast<const ControllableConstraintSpec*>(&specPtr)) {
    const ControllableConstraintSpec* castPtr = dynamic_cast<const ControllableConstraintSpec*>(&specPtr);
    out += AdvancedScope::fCContC + '(';
    specToString(out, castPtr->lower_);
    out += ',';
    specToString(out, castPtr->upper_);
    out += ',';
    specToString(out, castPtr->weight_);
    out += ",\"" + castPtr->input_->get_value() + "\")";

  } else if (dynamic_cast<const SoftConstraintSpec*>(&specPtr)) {
    const SoftConstraintSpec* castPtr = dynamic_cast<const SoftConstraintSpec*>(&specPtr);
    out += AdvancedScope::fCSoftC + '(';
    specToString(out, castPtr->lower_);
    out += ',';
    specToString(out, castPtr->upper_);
    out += ',';
    specToString(out, castPtr->weight_);
    out += ',';
    specToString(out, castPtr->expression_);
    out += ",\"" + castPtr->name_->get_value() + "\")";

  } else if (dynamic_cast<const HardConstraintSpec*>(&specPtr)) {
    const HardConstraintSpec* castPtr = dynamic_cast<const HardConstraintSpec*>(&specPtr);
    out += AdvancedScope::fCHardC + '(';
    specToString(out, castPtr->lower_);
    out += ',';
    specToString(out, castPtr->upper_);
    out += ',';
    specToString(out, castPtr->expression_);
    out += ')';

  } else if (dynamic_cast<const StringSpec*>(&specPtr)) {
    const StringSpec* castPtr = dynamic_cast<const StringSpec*>(&specPtr);
    out += '"' + castPtr->get_value() + '"';

  } else if (dynamic_cast<const ListSpec*>(&specPtr)) {
    const ListSpec* castPtr = dynamic_cast<const ListSpec*>(&specPtr);
    std::vector<SpecPtr> contents = castPtr->get_value();
    out += '[';
    specToString(out, contents[0]);
    for (size_t i = 1; i < contents.size(); i++) {
      specToString(out, contents[i]);
      out += ',';
    }
    out += ']';
  } 
}

///////////////////////////////////////////////////////////////////////////////////////////////

template<>
DoubleReferenceSpecPtr instance(const std::string& a) {
  DoubleReferenceSpecPtr out = DoubleReferenceSpecPtr(new DoubleReferenceSpec());
  out->set_reference_name(a);
  return out;
}

template<>
VectorReferenceSpecPtr instance(const std::string& a) {
  VectorReferenceSpecPtr out = VectorReferenceSpecPtr(new VectorReferenceSpec());
  out->set_reference_name(a);
  return out;
}

template<>
RotationReferenceSpecPtr instance(const std::string& a) {
  RotationReferenceSpecPtr out = RotationReferenceSpecPtr(new RotationReferenceSpec());
  out->set_reference_name(a);
  return out;
}

template<>
FrameReferenceSpecPtr instance(const std::string& a) {
  FrameReferenceSpecPtr out = FrameReferenceSpecPtr(new FrameReferenceSpec());
  out->set_reference_name(a);
  return out;
}

template<>
DoubleReferenceSpecPtr instance(std::string a) {
  DoubleReferenceSpecPtr out = DoubleReferenceSpecPtr(new DoubleReferenceSpec());
  out->set_reference_name(a);
  return out;
}

template<>
VectorReferenceSpecPtr instance(std::string a) {
  VectorReferenceSpecPtr out = VectorReferenceSpecPtr(new VectorReferenceSpec());
  out->set_reference_name(a);
  return out;
}

template<>
RotationReferenceSpecPtr instance(std::string a) {
  RotationReferenceSpecPtr out = RotationReferenceSpecPtr(new RotationReferenceSpec());
  out->set_reference_name(a);
  return out;
}

template<>
FrameReferenceSpecPtr instance(std::string a) {
  FrameReferenceSpecPtr out = FrameReferenceSpecPtr(new FrameReferenceSpec());
  out->set_reference_name(a);
  return out;
}

template<>
AbsSpecPtr instance(DoubleSpecPtr a) {
  AbsSpecPtr out = AbsSpecPtr(new AbsSpec());
  out->set_value(a);
  return out;
}

template<>
SinSpecPtr instance(DoubleSpecPtr a) {
  SinSpecPtr out = SinSpecPtr(new SinSpec());
  out->set_value(a);
  return out;
}

template<>
CosSpecPtr instance(DoubleSpecPtr a) {
  CosSpecPtr out = CosSpecPtr(new CosSpec());
  out->set_value(a);
  return out;
}

template<>
ASinSpecPtr instance(DoubleSpecPtr a) {
  ASinSpecPtr out = ASinSpecPtr(new ASinSpec());
  out->set_value(a);
  return out;
}

template<>
ACosSpecPtr instance(DoubleSpecPtr a) {
  ACosSpecPtr out = ACosSpecPtr(new ACosSpec());
  out->set_value(a);
  return out;
}

template<>
TanSpecPtr instance(DoubleSpecPtr a) {
  TanSpecPtr out = TanSpecPtr(new TanSpec());
  out->set_value(a);
  return out;
}

template<>
ATanSpecPtr instance(DoubleSpecPtr a) {
  ATanSpecPtr out = ATanSpecPtr(new ATanSpec());
  out->set_value(a);
  return out;
}

template<>
DoubleNormOfSpecPtr instance(VectorSpecPtr a) {
  DoubleNormOfSpecPtr out = DoubleNormOfSpecPtr(new DoubleNormOfSpec());
  out->set_vector(a);
  return out;
}

template<>
DoubleXCoordOfSpecPtr instance(VectorSpecPtr a) {
  DoubleXCoordOfSpecPtr out = DoubleXCoordOfSpecPtr(new DoubleXCoordOfSpec());
  out->set_vector(a);
  return out;
}

template<>
DoubleYCoordOfSpecPtr instance(VectorSpecPtr a) {
  DoubleYCoordOfSpecPtr out = DoubleYCoordOfSpecPtr(new DoubleYCoordOfSpec());
  out->set_vector(a);
  return out;
}

template<>
DoubleZCoordOfSpecPtr instance(VectorSpecPtr a) {
  DoubleZCoordOfSpecPtr out = DoubleZCoordOfSpecPtr(new DoubleZCoordOfSpec());
  out->set_vector(a);
  return out;
}

template<>
VectorRotationVectorSpecPtr instance(RotationSpecPtr a) {
  VectorRotationVectorSpecPtr out = VectorRotationVectorSpecPtr(new VectorRotationVectorSpec());
  out->set_rotation(a);
  return out;
}

template<>
VectorOriginOfSpecPtr instance(FrameSpecPtr a) {
  VectorOriginOfSpecPtr out = VectorOriginOfSpecPtr(new VectorOriginOfSpec());
  out->set_frame(a);
  return out;
}

template<>
DoubleAdditionSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b) {
  DoubleAdditionSpecPtr out = DoubleAdditionSpecPtr(new DoubleAdditionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
DoubleSubtractionSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b) {
  DoubleSubtractionSpecPtr out = DoubleSubtractionSpecPtr(new DoubleSubtractionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
DoubleMultiplicationSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b) {
  DoubleMultiplicationSpecPtr out = DoubleMultiplicationSpecPtr(new DoubleMultiplicationSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
DoubleDivisionSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b) {
  DoubleDivisionSpecPtr out = DoubleDivisionSpecPtr(new DoubleDivisionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
MinSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b) {
  MinSpecPtr out = MinSpecPtr(new MinSpec());
  out->set_lhs(a);
  out->set_rhs(b);
  return out;
}

template<>
MaxSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b) {
  MaxSpecPtr out = MaxSpecPtr(new MaxSpec());
  out->set_lhs(a);
  out->set_rhs(b);
  return out;
}

template<>
FmodSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b) {
  FmodSpecPtr out = FmodSpecPtr(new FmodSpec());
  out->set_nominator(a);
  out->set_denominator(b);
  return out;
}

template<>
VectorAdditionSpecPtr instance(VectorSpecPtr a, VectorSpecPtr b) {
  VectorAdditionSpecPtr out = VectorAdditionSpecPtr(new VectorAdditionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
VectorSubtractionSpecPtr instance(VectorSpecPtr a, VectorSpecPtr b) {
  VectorSubtractionSpecPtr out = VectorSubtractionSpecPtr(new VectorSubtractionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
FrameMultiplicationSpecPtr instance(FrameSpecPtr a, FrameSpecPtr b) {
  FrameMultiplicationSpecPtr out = FrameMultiplicationSpecPtr(new FrameMultiplicationSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
VectorDotSpecPtr instance(VectorSpecPtr a, VectorSpecPtr b) {
  VectorDotSpecPtr out = VectorDotSpecPtr(new VectorDotSpec());
  out->set_lhs(a);
  out->set_rhs(b);
  return out;
}

template<>
VectorCrossSpecPtr instance(VectorSpecPtr a, VectorSpecPtr b) {
  VectorCrossSpecPtr out = VectorCrossSpecPtr(new VectorCrossSpec());
  out->set_lhs(a);
  out->set_rhs(b);
  return out;
}

template<>
RotationMultiplicationSpecPtr instance(RotationSpecPtr a, RotationSpecPtr b) {
  RotationMultiplicationSpecPtr out = RotationMultiplicationSpecPtr(new RotationMultiplicationSpec({a,b}));
  return out;
}

template<>
VectorDoubleMultiplicationSpecPtr instance(VectorSpecPtr a, DoubleSpecPtr b) {
  VectorDoubleMultiplicationSpecPtr out = VectorDoubleMultiplicationSpecPtr(new VectorDoubleMultiplicationSpec());
  out->set_vector(a);
  out->set_double(b);
  return out;
}

template<>
VectorDoubleMultiplicationSpecPtr instance(DoubleSpecPtr a, VectorSpecPtr b) {
  VectorDoubleMultiplicationSpecPtr out = VectorDoubleMultiplicationSpecPtr(new VectorDoubleMultiplicationSpec());
  out->set_vector(b);
  out->set_double(a);
  return out;
}

template<>
AxisAngleSpecPtr instance(VectorSpecPtr a, DoubleSpecPtr b) {
  AxisAngleSpecPtr out = AxisAngleSpecPtr(new AxisAngleSpec());
  out->set_axis(a);
  out->set_angle(b);
  return out;
}

template<>
VectorFrameMultiplicationSpecPtr instance(FrameSpecPtr a, VectorSpecPtr b) {
  VectorFrameMultiplicationSpecPtr out = VectorFrameMultiplicationSpecPtr(new VectorFrameMultiplicationSpec());
  out->set_frame(a);
  out->set_vector(b);
  return out;
}

template<>
VectorRotationMultiplicationSpecPtr instance(RotationSpecPtr a, VectorSpecPtr b) {
  VectorRotationMultiplicationSpecPtr out = VectorRotationMultiplicationSpecPtr(new VectorRotationMultiplicationSpec());
  out->set_rotation(a);
  out->set_vector(b);
  return out;
}

template<>
DoubleIfSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b, DoubleSpecPtr c) {
  DoubleIfSpecPtr out = DoubleIfSpecPtr(new DoubleIfSpec());
  out->set_condition(a);
  out->set_if(b);
  out->set_else(c);
  return out;
}

template<>
HardConstraintSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b, DoubleSpecPtr c) {
  HardConstraintSpecPtr out = HardConstraintSpecPtr(new HardConstraintSpec());
  out->lower_ = a;
  out->upper_ = b;
  out->expression_ = c;
  return out;
}

template<>
SlerpSpecPtr instance(RotationSpecPtr a, RotationSpecPtr b, DoubleSpecPtr c) {
  SlerpSpecPtr out = SlerpSpecPtr(new SlerpSpec());
  out->set_from(a);
  out->set_to(b);
  out->set_param(c);
  return out;
}

template<>
ControllableConstraintSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b, DoubleSpecPtr c, StringSpecPtr d) {
  ControllableConstraintSpecPtr out = ControllableConstraintSpecPtr(new ControllableConstraintSpec());
  out->lower_ = a;
  out->upper_ = b;
  out->weight_ = c;
  out->input_ = d;
  return out;
}


template<>
SoftConstraintSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b, DoubleSpecPtr c, DoubleSpecPtr d, StringSpecPtr e) {
  SoftConstraintSpecPtr out = SoftConstraintSpecPtr(new SoftConstraintSpec());
  out->lower_ = a;
  out->upper_ = b;
  out->weight_ = c;
  out->expression_ = d;
  out->name_ = e;
  return out;
}

template<>
AbsSpecPtr instance(DoubleSpecPtr& a) {
  AbsSpecPtr out = AbsSpecPtr(new AbsSpec());
  out->set_value(a);
  return out;
}

template<>
SinSpecPtr instance(DoubleSpecPtr& a) {
  SinSpecPtr out = SinSpecPtr(new SinSpec());
  out->set_value(a);
  return out;
}

template<>
CosSpecPtr instance(DoubleSpecPtr& a) {
  CosSpecPtr out = CosSpecPtr(new CosSpec());
  out->set_value(a);
  return out;
}

template<>
ASinSpecPtr instance(DoubleSpecPtr& a) {
  ASinSpecPtr out = ASinSpecPtr(new ASinSpec());
  out->set_value(a);
  return out;
}

template<>
ACosSpecPtr instance(DoubleSpecPtr& a) {
  ACosSpecPtr out = ACosSpecPtr(new ACosSpec());
  out->set_value(a);
  return out;
}

template<>
TanSpecPtr instance(DoubleSpecPtr& a) {
  TanSpecPtr out = TanSpecPtr(new TanSpec());
  out->set_value(a);
  return out;
}

template<>
ATanSpecPtr instance(DoubleSpecPtr& a) {
  ATanSpecPtr out = ATanSpecPtr(new ATanSpec());
  out->set_value(a);
  return out;
}

template<>
DoubleNormOfSpecPtr instance(VectorSpecPtr& a) {
  DoubleNormOfSpecPtr out = DoubleNormOfSpecPtr(new DoubleNormOfSpec());
  out->set_vector(a);
  return out;
}

template<>
DoubleXCoordOfSpecPtr instance(VectorSpecPtr& a) {
  DoubleXCoordOfSpecPtr out = DoubleXCoordOfSpecPtr(new DoubleXCoordOfSpec());
  out->set_vector(a);
  return out;
}

template<>
DoubleYCoordOfSpecPtr instance(VectorSpecPtr& a) {
  DoubleYCoordOfSpecPtr out = DoubleYCoordOfSpecPtr(new DoubleYCoordOfSpec());
  out->set_vector(a);
  return out;
}

template<>
DoubleZCoordOfSpecPtr instance(VectorSpecPtr& a) {
  DoubleZCoordOfSpecPtr out = DoubleZCoordOfSpecPtr(new DoubleZCoordOfSpec());
  out->set_vector(a);
  return out;
}

template<>
VectorRotationVectorSpecPtr instance(RotationSpecPtr& a) {
  VectorRotationVectorSpecPtr out = VectorRotationVectorSpecPtr(new VectorRotationVectorSpec());
  out->set_rotation(a);
  return out;
}

template<>
VectorOriginOfSpecPtr instance(FrameSpecPtr& a) {
  VectorOriginOfSpecPtr out = VectorOriginOfSpecPtr(new VectorOriginOfSpec());
  out->set_frame(a);
  return out;
}

template<>
DoubleAdditionSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b) {
  DoubleAdditionSpecPtr out = DoubleAdditionSpecPtr(new DoubleAdditionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
DoubleSubtractionSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b) {
  DoubleSubtractionSpecPtr out = DoubleSubtractionSpecPtr(new DoubleSubtractionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
DoubleMultiplicationSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b) {
  DoubleMultiplicationSpecPtr out = DoubleMultiplicationSpecPtr(new DoubleMultiplicationSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
DoubleDivisionSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b) {
  DoubleDivisionSpecPtr out = DoubleDivisionSpecPtr(new DoubleDivisionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
MinSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b) {
  MinSpecPtr out = MinSpecPtr(new MinSpec());
  out->set_lhs(a);
  out->set_rhs(b);
  return out;
}

template<>
MaxSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b) {
  MaxSpecPtr out = MaxSpecPtr(new MaxSpec());
  out->set_lhs(a);
  out->set_rhs(b);
  return out;
}

template<>
FmodSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b) {
  FmodSpecPtr out = FmodSpecPtr(new FmodSpec());
  out->set_nominator(a);
  out->set_denominator(b);
  return out;
}

template<>
VectorAdditionSpecPtr instance(VectorSpecPtr& a, VectorSpecPtr& b) {
  VectorAdditionSpecPtr out = VectorAdditionSpecPtr(new VectorAdditionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
VectorSubtractionSpecPtr instance(VectorSpecPtr& a, VectorSpecPtr& b) {
  VectorSubtractionSpecPtr out = VectorSubtractionSpecPtr(new VectorSubtractionSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
FrameMultiplicationSpecPtr instance(FrameSpecPtr& a, FrameSpecPtr& b) {
  FrameMultiplicationSpecPtr out = FrameMultiplicationSpecPtr(new FrameMultiplicationSpec());
  out->set_inputs({a,b});
  return out;
}

template<>
VectorDotSpecPtr instance(VectorSpecPtr& a, VectorSpecPtr& b) {
  VectorDotSpecPtr out = VectorDotSpecPtr(new VectorDotSpec());
  out->set_lhs(a);
  out->set_rhs(b);
  return out;
}

template<>
VectorCrossSpecPtr instance(VectorSpecPtr& a, VectorSpecPtr& b) {
  VectorCrossSpecPtr out = VectorCrossSpecPtr(new VectorCrossSpec());
  out->set_lhs(a);
  out->set_rhs(b);
  return out;
}

template<>
RotationMultiplicationSpecPtr instance(RotationSpecPtr& a, RotationSpecPtr& b) {
  RotationMultiplicationSpecPtr out = RotationMultiplicationSpecPtr(new RotationMultiplicationSpec({a,b}));
  return out;
}

template<>
VectorDoubleMultiplicationSpecPtr instance(VectorSpecPtr& a, DoubleSpecPtr& b) {
  VectorDoubleMultiplicationSpecPtr out = VectorDoubleMultiplicationSpecPtr(new VectorDoubleMultiplicationSpec());
  out->set_vector(a);
  out->set_double(b);
  return out;
}

template<>
VectorDoubleMultiplicationSpecPtr instance(DoubleSpecPtr& a, VectorSpecPtr& b) {
  VectorDoubleMultiplicationSpecPtr out = VectorDoubleMultiplicationSpecPtr(new VectorDoubleMultiplicationSpec());
  out->set_vector(b);
  out->set_double(a);
  return out;
}

template<>
AxisAngleSpecPtr instance(VectorSpecPtr& a, DoubleSpecPtr& b) {
  AxisAngleSpecPtr out = AxisAngleSpecPtr(new AxisAngleSpec());
  out->set_axis(a);
  out->set_angle(b);
  return out;
}

template<>
VectorFrameMultiplicationSpecPtr instance(FrameSpecPtr& a, VectorSpecPtr& b) {
  VectorFrameMultiplicationSpecPtr out = VectorFrameMultiplicationSpecPtr(new VectorFrameMultiplicationSpec());
  out->set_frame(a);
  out->set_vector(b);
  return out;
}

template<>
VectorRotationMultiplicationSpecPtr instance(RotationSpecPtr& a, VectorSpecPtr& b) {
  VectorRotationMultiplicationSpecPtr out = VectorRotationMultiplicationSpecPtr(new VectorRotationMultiplicationSpec());
  out->set_rotation(a);
  out->set_vector(b);
  return out;
}

template<>
DoubleIfSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b, DoubleSpecPtr& c) {
  DoubleIfSpecPtr out = DoubleIfSpecPtr(new DoubleIfSpec());
  out->set_condition(a);
  out->set_if(b);
  out->set_else(c);
  return out;
}

template<>
HardConstraintSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b, DoubleSpecPtr& c) {
  HardConstraintSpecPtr out = HardConstraintSpecPtr(new HardConstraintSpec());
  out->lower_ = a;
  out->upper_ = b;
  out->expression_ = c;
  return out;
}

template<>
SlerpSpecPtr instance(RotationSpecPtr& a, RotationSpecPtr& b, DoubleSpecPtr& c) {
  SlerpSpecPtr out = SlerpSpecPtr(new SlerpSpec());
  out->set_from(a);
  out->set_to(b);
  out->set_param(c);
  return out;
}

template<>
ControllableConstraintSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b, DoubleSpecPtr& c, StringSpecPtr& d) {
  ControllableConstraintSpecPtr out = ControllableConstraintSpecPtr(new ControllableConstraintSpec());
  out->lower_ = a;
  out->upper_ = b;
  out->weight_ = c;
  out->input_ = d;
  return out;
}


template<>
SoftConstraintSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b, DoubleSpecPtr& c, DoubleSpecPtr& d, StringSpecPtr& e) {
  SoftConstraintSpecPtr out = SoftConstraintSpecPtr(new SoftConstraintSpec());
  out->lower_ = a;
  out->upper_ = b;
  out->weight_ = c;
  out->expression_ = d;
  out->name_ = e;
  return out;
}

template<>
VectorCachedSpecPtr instance(VectorSpecPtr a) {
  VectorCachedSpecPtr out = VectorCachedSpecPtr(new VectorCachedSpec());
  out->set_vector(a);
  return out;
}

template<>
VectorCachedSpecPtr instance(VectorSpecPtr& a) {
  VectorCachedSpecPtr out = VectorCachedSpecPtr(new VectorCachedSpec());
  out->set_vector(a);
  return out;
}

template<>
FrameCachedSpecPtr instance(FrameSpecPtr a) {
  FrameCachedSpecPtr out = FrameCachedSpecPtr(new FrameCachedSpec());
  out->set_frame(a);
  return out;
}

template<>
FrameCachedSpecPtr instance(FrameSpecPtr& a) {
  FrameCachedSpecPtr out = FrameCachedSpecPtr(new FrameCachedSpec());
  out->set_frame(a);
  return out;
}

}