#pragma once
#include <giskard_suturo_parser/advanced_scope.h>

using namespace giskard_core;

namespace giskard_suturo {
using boost::dynamic_pointer_cast;

  struct NotImplementedException : public std::exception {
    const char* what() noexcept { return "TODO"; }
  };

  /**
   * @brief      Function for easy instantiation of classes needing no construction arguments.
   *
   * @tparam     T     Class to instantiate
   *
   * @return     Shared pointer to the new object
   */
  template<class T>
  boost::shared_ptr<T> instance() {
    return boost::shared_ptr<T>(new T());
  }

  /**
   * @brief      Function for easy instantiation of classes needing one construction argument.
   *
   * @param[in]  a     First argument
   *
   * @tparam     T     Class to instantiate
   * @tparam     A     Type of first argument
   *
   * @return     Shared pointer to the new object
   */
  template<class T, typename A>
  boost::shared_ptr<T> instance(A a) {
    return boost::shared_ptr<T>(new T(a));
  }

  /**
   * @brief      Function for easy instantiation of classes needing two construction arguments.
   *
   * @param[in]  a     First argument
   * @param[in]  b     Second argument
   *
   * @tparam     T     Class to instantiate
   * @tparam     A     Type of first argument
   * @tparam     B     Type of second argument
   *
   * @return     Shared pointer to the new object
   */
  template<class T, typename A, typename B>
  boost::shared_ptr<T> instance(A a, B b) {
    return boost::shared_ptr<T>(new T(a, b));
  }

  /**
   * @brief      Function for easy instantiation of classes needing three construction arguments.
   *
   * @param[in]  a     First argument
   * @param[in]  b     Second argument
   * @param[in]  c     Third argument
   *
   * @tparam     T     Class to instantiate
   * @tparam     A     Type of first argument
   * @tparam     B     Type of second argument
   * @tparam     C     Type of third argument
   *
   * @return     Shared pointer to the new object
   */
  template<class T, typename A, typename B, typename C>
  boost::shared_ptr<T> instance(A a, B b, C c) {
    return boost::shared_ptr<T>(new T(a, b, c));
  }

  /**
   * @brief      Function for easy instantiation of classes needing four construction arguments.
   *
   * @param[in]  a     First argument
   * @param[in]  b     Second argument
   * @param[in]  c     Third argument
   * @param[in]  d     Fourth argument
   *
   * @tparam     T     Class to instantiate
   * @tparam     A     Type of first argument
   * @tparam     B     Type of second argument
   * @tparam     C     Type of third argument
   * @tparam     D     Type of fourth argument
   *
   * @return     Shared pointer to the new object
   */
  template<class T, typename A, typename B, typename C, typename D>
  boost::shared_ptr<T> instance(A a, B b, C c, D d) {
    return boost::shared_ptr<T>(new T(a, b, c, d));
  }

  /**
   * @brief      Function for easy instantiation of classes needing five construction arguments.
   *
   * @param[in]  a     First argument
   * @param[in]  b     Second argument
   * @param[in]  c     Third argument
   * @param[in]  d     Fourth argument
   * @param[in]  e     Fifth argument
   *
   * @tparam     T     Class to instantiate
   * @tparam     A     Type of first argument
   * @tparam     B     Type of second argument
   * @tparam     C     Type of third argument
   * @tparam     D     Type of fourth argument
   * @tparam     E     Type of fifth argument
   *
   * @return     Shared pointer to the new object
   */
  template<class T, typename A, typename B, typename C, typename D, typename E>
  boost::shared_ptr<T> instance(A a, B b, C c, D d, E e) {
    return boost::shared_ptr<T>(new T(a, b, c, d, e));
  }

  /**
   * @brief      Operator for comfortably comparing strings and c-strings.
   *
   * @param[in]  str   A string
   * @param[in]  cstr  A c-string
   *
   * @return     True if the string's contents are equal, false otherwise.
   */
  inline bool operator == (const std::string& str, const char* cstr) {
    return str.compare(cstr) == 0;
  }

  /**
   * @brief      Function trying to do a pointer cast.
   *
   * @param      p     Pointer to cast
   * @param      out   Successfully cast pointer
   *
   * @tparam     T     Desired pointer type
   *
   * @return     True if cast was successful
   */
  template<typename T>  
  bool matches(SpecPtr& p, boost::shared_ptr<T>& out) {
    out = dynamic_pointer_cast<T>(p);
    return !!out;
  }

  template<typename A, typename B>
  bool matches(SpecPtr& a, SpecPtr& b, boost::shared_ptr<A>& aout, boost::shared_ptr<B>& bout) {
    return matches(a, aout) && matches(b, bout);
  }

  template<typename A, typename B, typename C>
  bool matches(SpecPtr& a, SpecPtr& b, SpecPtr& c, boost::shared_ptr<A>& aout, boost::shared_ptr<B>& bout, boost::shared_ptr<C>& cout) {
    return matches(a, aout) && matches(b, bout) && matches(c, cout);
  }

  template<typename A, typename B, typename C, typename D>
  bool matches(SpecPtr& a, SpecPtr& b, SpecPtr& c, SpecPtr& d, boost::shared_ptr<A>& aout, boost::shared_ptr<B>& bout, boost::shared_ptr<C>& cout, boost::shared_ptr<D>& dout) {
    return matches(a, aout) && matches(b, bout) && matches(c, cout) && matches(d, dout);
  }

  template<typename A, typename B, typename C, typename D, typename E>
  bool matches(SpecPtr& a, SpecPtr& b, SpecPtr& c, SpecPtr& d, SpecPtr& e, boost::shared_ptr<A>& aout, boost::shared_ptr<B>& bout, boost::shared_ptr<C>& cout, boost::shared_ptr<D>& dout, boost::shared_ptr<E>& eout) {
    return matches(a, aout) && matches(b, bout) && matches(c, cout) && matches(d, dout) && matches(e, eout);
  }

  /**
   * @brief      Function generating a readable name for a pointer's type.
   *
   * @param[in]  ptr   Pointer to generate type name for
   *
   * @return     Readable type name
   */
  std::string toTypeString(const SpecPtr& ptr);

  /**
   * @brief      Function generating a readable list of name for a vector containing pointers.
   *
   * @param[in]  v     Vector to generate names for.
   *
   * @return     List of names
   */
  std::string toTypeList(const std::vector<SpecPtr>& v);

  /**
   * @brief      Creates a reference specification for the given pointer and scope.
   *
   * @param[in]  name         Name of the reference
   * @param[in]  ptr          Pointer to reference
   * @param[in]  searchScope  Scope the pointer is defined in
   *
   * @return     A global reference to the pointer with the given name
   */
  SpecPtr createReferenceSpec(const std::string& name, SpecPtr ptr, const AdvancedScopePtr& searchScope);
  
  /**
   * @brief      Checks whether the two pointers match in their base types.
   *
   * @param[in]  a     First pointer for comparison
   * @param[in]  b     Second pointer for comparison
   *
   * @return     True if pointer types match.
   */
  bool typesAreEqual(const SpecPtr& a, const SpecPtr& b);

  /**
   * @brief      Recursively find all references within a specification.
   *
   * @param      references  Found references
   * @param[in]  specPtr     Specification to search
   */
  void getReferenceSpecs(std::vector<SpecPtr>& references, const SpecPtr& specPtr);
  
  /**
   * @brief      Generate an algebraic string for a specification.
   *
   * @param      out      Resulting string
   * @param[in]  specPtr  Specification to generate string for
   */
  void specToString(std::string& out, const SpecPtr& specPtr);
  
  /**
   * @brief      Generate an algebraic string for a specification.
   *
   * @param      out      Resulting string
   * @param[in]  specPtr  Specification to generate string for
   */
  void specToString(std::string& out, const Spec& specPtr);

  /**
   * @brief      Get the name of the referenced specification from a reference.
   *
   * @param[in]  ptr   Reference to get name from
   *
   * @return     Name of the referenced specification
   */
  std::string getReferenceName(const SpecPtr& ptr);

  /**
   * @brief      Sets the name of the specification referenced by a reference.
   *
   * @param[in]  ptr   Reference specification
   * @param[in]  name  New name of referenced specification
   */
  void setReferenceName(const SpecPtr& ptr, std::string name);

  //////// Specializations to deal with the lack of constructors for the giskard specifications

template<>
DoubleReferenceSpecPtr instance(const std::string& a);

template<>
VectorReferenceSpecPtr instance(const std::string& a);

template<>
RotationReferenceSpecPtr instance(const std::string& a);

template<>
FrameReferenceSpecPtr instance(const std::string& a);

template<>
DoubleReferenceSpecPtr instance(std::string a);

template<>
VectorReferenceSpecPtr instance(std::string a);

template<>
RotationReferenceSpecPtr instance(std::string a);

template<>
FrameReferenceSpecPtr instance(std::string a);

template<>
AbsSpecPtr instance(DoubleSpecPtr a);

template<>
SinSpecPtr instance(DoubleSpecPtr a);

template<>
CosSpecPtr instance(DoubleSpecPtr a);

template<>
ASinSpecPtr instance(DoubleSpecPtr a);

template<>
ACosSpecPtr instance(DoubleSpecPtr a);

template<>
TanSpecPtr instance(DoubleSpecPtr a);

template<>
ATanSpecPtr instance(DoubleSpecPtr a);

template<>
DoubleNormOfSpecPtr instance(VectorSpecPtr a);

template<>
DoubleXCoordOfSpecPtr instance(VectorSpecPtr a);

template<>
DoubleYCoordOfSpecPtr instance(VectorSpecPtr a);

template<>
DoubleZCoordOfSpecPtr instance(VectorSpecPtr a);

template<>
VectorRotationVectorSpecPtr instance(RotationSpecPtr a);

template<>
VectorOriginOfSpecPtr instance(FrameSpecPtr a);

template<>
DoubleAdditionSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b);

template<>
DoubleSubtractionSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b);

template<>
DoubleMultiplicationSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b);

template<>
DoubleDivisionSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b);

template<>
MinSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b);

template<>
MaxSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b);

template<>
FmodSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b);

template<>
VectorAdditionSpecPtr instance(VectorSpecPtr a, VectorSpecPtr b);

template<>
VectorSubtractionSpecPtr instance(VectorSpecPtr a, VectorSpecPtr b);

template<>
FrameMultiplicationSpecPtr instance(FrameSpecPtr a, FrameSpecPtr b);

template<>
VectorDotSpecPtr instance(VectorSpecPtr a, VectorSpecPtr b);

template<>
VectorCachedSpecPtr instance(VectorSpecPtr a);

template<>
VectorCrossSpecPtr instance(VectorSpecPtr a, VectorSpecPtr b);

template<>
RotationMultiplicationSpecPtr instance(RotationSpecPtr a, RotationSpecPtr b);

template<>
VectorDoubleMultiplicationSpecPtr instance(VectorSpecPtr a, DoubleSpecPtr b);

template<>
VectorDoubleMultiplicationSpecPtr instance(DoubleSpecPtr a, VectorSpecPtr b);

template<>
AxisAngleSpecPtr instance(VectorSpecPtr a, DoubleSpecPtr b);

template<>
VectorFrameMultiplicationSpecPtr instance(FrameSpecPtr a, VectorSpecPtr b);

template<>
VectorRotationMultiplicationSpecPtr instance(RotationSpecPtr a, VectorSpecPtr b);

template<>
DoubleIfSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b, DoubleSpecPtr c);

template<>
HardConstraintSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b, DoubleSpecPtr c);

template<>
SlerpSpecPtr instance(RotationSpecPtr a, RotationSpecPtr b, DoubleSpecPtr c);

template<>
ControllableConstraintSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b, DoubleSpecPtr c, StringSpecPtr d);

template<>
SoftConstraintSpecPtr instance(DoubleSpecPtr a, DoubleSpecPtr b, DoubleSpecPtr c, DoubleSpecPtr d, StringSpecPtr e);

template<>
FrameCachedSpecPtr instance(FrameSpecPtr a);

template<>
AbsSpecPtr instance(DoubleSpecPtr& a);

template<>
SinSpecPtr instance(DoubleSpecPtr& a);

template<>
CosSpecPtr instance(DoubleSpecPtr& a);

template<>
ASinSpecPtr instance(DoubleSpecPtr& a);

template<>
ACosSpecPtr instance(DoubleSpecPtr& a);

template<>
TanSpecPtr instance(DoubleSpecPtr& a);

template<>
ATanSpecPtr instance(DoubleSpecPtr& a);

template<>
DoubleNormOfSpecPtr instance(VectorSpecPtr& a);

template<>
DoubleXCoordOfSpecPtr instance(VectorSpecPtr& a);

template<>
DoubleYCoordOfSpecPtr instance(VectorSpecPtr& a);

template<>
DoubleZCoordOfSpecPtr instance(VectorSpecPtr& a);

template<>
VectorRotationVectorSpecPtr instance(RotationSpecPtr& a);

template<>
VectorOriginOfSpecPtr instance(FrameSpecPtr& a);

template<>
DoubleAdditionSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b);

template<>
DoubleSubtractionSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b);

template<>
DoubleMultiplicationSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b);

template<>
DoubleDivisionSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b);

template<>
MinSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b);

template<>
MaxSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b);

template<>
FmodSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b);

template<>
VectorAdditionSpecPtr instance(VectorSpecPtr& a, VectorSpecPtr& b);

template<>
VectorSubtractionSpecPtr instance(VectorSpecPtr& a, VectorSpecPtr& b);

template<>
FrameMultiplicationSpecPtr instance(FrameSpecPtr& a, FrameSpecPtr& b);

template<>
VectorDotSpecPtr instance(VectorSpecPtr& a, VectorSpecPtr& b);

template<>
VectorCachedSpecPtr instance(VectorSpecPtr& a);

template<>
VectorCrossSpecPtr instance(VectorSpecPtr& a, VectorSpecPtr& b);

template<>
RotationMultiplicationSpecPtr instance(RotationSpecPtr& a, RotationSpecPtr& b);

template<>
VectorDoubleMultiplicationSpecPtr instance(VectorSpecPtr& a, DoubleSpecPtr& b);

template<>
VectorDoubleMultiplicationSpecPtr instance(DoubleSpecPtr& a, VectorSpecPtr& b);

template<>
AxisAngleSpecPtr instance(VectorSpecPtr& a, DoubleSpecPtr& b);

template<>
VectorFrameMultiplicationSpecPtr instance(FrameSpecPtr& a, VectorSpecPtr& b);

template<>
VectorRotationMultiplicationSpecPtr instance(RotationSpecPtr& a, VectorSpecPtr& b);

template<>
DoubleIfSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b, DoubleSpecPtr& c);

template<>
HardConstraintSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b, DoubleSpecPtr& c);

template<>
SlerpSpecPtr instance(RotationSpecPtr& a, RotationSpecPtr& b, DoubleSpecPtr& c);

template<>
ControllableConstraintSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b, DoubleSpecPtr& c, StringSpecPtr& d);


template<>
SoftConstraintSpecPtr instance(DoubleSpecPtr& a, DoubleSpecPtr& b, DoubleSpecPtr& c, DoubleSpecPtr& d, StringSpecPtr& e);

template<>
FrameCachedSpecPtr instance(FrameSpecPtr& a);


template<class T>
boost::shared_ptr<T> deepCopySpec(const boost::shared_ptr<T> specPtr);

template<>
DoubleSpecPtr deepCopySpec(const DoubleSpecPtr specPtr);
template<>
VectorSpecPtr deepCopySpec(const VectorSpecPtr specPtr);
template<>
RotationSpecPtr deepCopySpec(const RotationSpecPtr specPtr);
template<>
FrameSpecPtr deepCopySpec(const FrameSpecPtr specPtr);
template<>
StringSpecPtr deepCopySpec(const StringSpecPtr specPtr);
template<>
ListSpecPtr deepCopySpec(const ListSpecPtr specPtr);

template<>
DoubleInputSpecPtr deepCopySpec(const DoubleInputSpecPtr specPtr);
template<>
JointInputSpecPtr deepCopySpec(const JointInputSpecPtr specPtr);
template<>
DoubleConstSpecPtr deepCopySpec(const DoubleConstSpecPtr specPtr);
template<>
DoubleReferenceSpecPtr deepCopySpec(const DoubleReferenceSpecPtr specPtr);
template<>
DoubleAdditionSpecPtr deepCopySpec(const DoubleAdditionSpecPtr specPtr);
template<>
DoubleSubtractionSpecPtr deepCopySpec(const DoubleSubtractionSpecPtr specPtr);
template<>
DoubleNormOfSpecPtr deepCopySpec(const DoubleNormOfSpecPtr specPtr);
template<>
DoubleMultiplicationSpecPtr deepCopySpec(const DoubleMultiplicationSpecPtr specPtr);
template<>
DoubleDivisionSpecPtr deepCopySpec(const DoubleDivisionSpecPtr specPtr);
template<>
DoubleXCoordOfSpecPtr deepCopySpec(const DoubleXCoordOfSpecPtr specPtr);
template<>
DoubleYCoordOfSpecPtr deepCopySpec(const DoubleYCoordOfSpecPtr specPtr);
template<>
DoubleZCoordOfSpecPtr deepCopySpec(const DoubleZCoordOfSpecPtr specPtr);
template<>
VectorDotSpecPtr deepCopySpec(const VectorDotSpecPtr specPtr);
template<>
MinSpecPtr deepCopySpec(const MinSpecPtr specPtr);
template<>
MaxSpecPtr deepCopySpec(const MaxSpecPtr specPtr);
template<>
AbsSpecPtr deepCopySpec(const AbsSpecPtr specPtr);
template<>
DoubleIfSpecPtr deepCopySpec(const DoubleIfSpecPtr specPtr);
template<>
FmodSpecPtr deepCopySpec(const FmodSpecPtr specPtr);
template<>
SinSpecPtr deepCopySpec(const SinSpecPtr specPtr);
template<>
CosSpecPtr deepCopySpec(const CosSpecPtr specPtr);
template<>
TanSpecPtr deepCopySpec(const TanSpecPtr specPtr);
template<>
ASinSpecPtr deepCopySpec(const ASinSpecPtr specPtr);
template<>
ACosSpecPtr deepCopySpec(const ACosSpecPtr specPtr);
template<>
ATanSpecPtr deepCopySpec(const ATanSpecPtr specPtr);
template<>
VectorInputSpecPtr deepCopySpec(const VectorInputSpecPtr specPtr);
template<>
VectorCachedSpecPtr deepCopySpec(const VectorCachedSpecPtr specPtr);
template<>
VectorConstructorSpecPtr deepCopySpec(const VectorConstructorSpecPtr specPtr);
template<>
VectorAdditionSpecPtr deepCopySpec(const VectorAdditionSpecPtr specPtr);
template<>
VectorSubtractionSpecPtr deepCopySpec(const VectorSubtractionSpecPtr specPtr);
template<>
VectorReferenceSpecPtr deepCopySpec(const VectorReferenceSpecPtr specPtr);
template<>
VectorOriginOfSpecPtr deepCopySpec(const VectorOriginOfSpecPtr specPtr);
template<>
VectorFrameMultiplicationSpecPtr deepCopySpec(const VectorFrameMultiplicationSpecPtr specPtr);
template<>
VectorRotationMultiplicationSpecPtr deepCopySpec(const VectorRotationMultiplicationSpecPtr specPtr);
template<>
VectorDoubleMultiplicationSpecPtr deepCopySpec(const VectorDoubleMultiplicationSpecPtr specPtr);
template<>
VectorRotationVectorSpecPtr deepCopySpec(const VectorRotationVectorSpecPtr specPtr);
template<>
VectorCrossSpecPtr deepCopySpec(const VectorCrossSpecPtr specPtr);
template<>
RotationInputSpecPtr deepCopySpec(const RotationInputSpecPtr specPtr);
template<>
RotationQuaternionConstructorSpecPtr deepCopySpec(const RotationQuaternionConstructorSpecPtr specPtr);
template<>
AxisAngleSpecPtr deepCopySpec(const AxisAngleSpecPtr specPtr);
template<>
SlerpSpecPtr deepCopySpec(const SlerpSpecPtr specPtr);
template<>
RotationReferenceSpecPtr deepCopySpec(const RotationReferenceSpecPtr specPtr);
template<>
InverseRotationSpecPtr deepCopySpec(const InverseRotationSpecPtr specPtr);
template<>
RotationMultiplicationSpecPtr deepCopySpec(const RotationMultiplicationSpecPtr specPtr);
template<>
FrameInputSpecPtr deepCopySpec(const FrameInputSpecPtr specPtr);
template<>
FrameCachedSpecPtr deepCopySpec(const FrameCachedSpecPtr specPtr);
template<>
FrameConstructorSpecPtr deepCopySpec(const FrameConstructorSpecPtr specPtr);
template<>
OrientationOfSpecPtr deepCopySpec(const OrientationOfSpecPtr specPtr);
template<>
FrameMultiplicationSpecPtr deepCopySpec(const FrameMultiplicationSpecPtr specPtr);
template<>
FrameReferenceSpecPtr deepCopySpec(const FrameReferenceSpecPtr specPtr);
template<>
InverseFrameSpecPtr deepCopySpec(const InverseFrameSpecPtr specPtr);
template<>
ControllableConstraintSpecPtr deepCopySpec(const ControllableConstraintSpecPtr specPtr);
template<>
SoftConstraintSpecPtr deepCopySpec(const SoftConstraintSpecPtr specPtr);
template<>
HardConstraintSpecPtr deepCopySpec(const HardConstraintSpecPtr specPtr);
template<>
ConstStringSpecPtr deepCopySpec(const ConstStringSpecPtr specPtr);
template<>
ConcatStringSpecPtr deepCopySpec(const ConcatStringSpecPtr specPtr);
template<>
StringReferenceSpecPtr deepCopySpec(const StringReferenceSpecPtr specPtr);
template<>
ConstListSpecPtr deepCopySpec(const ConstListSpecPtr specPtr);
template<>
ConcatListSpecPtr deepCopySpec(const ConcatListSpecPtr specPtr);
template<>
ListReferenceSpecPtr deepCopySpec(const ListReferenceSpecPtr specPtr);

}