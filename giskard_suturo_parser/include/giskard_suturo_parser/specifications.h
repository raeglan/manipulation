#pragma once
#include <giskard_core/specifications.hpp>

using namespace giskard_core;

namespace giskard_suturo {
SpecPtr createTypeObject(const SpecPtr& ptr);

class AdvancedScope;

class StringReferenceSpec : public StringSpec
{
public:
  StringReferenceSpec(const std::string& refName, const boost::shared_ptr<AdvancedScope>& _scope)
  : reference_name(refName)
  , scope(_scope) {}

  const std::string& get_reference_name() const;
  void set_reference_name(const std::string& refName);
  void get_input_specs(std::vector<const InputSpec*>& inputs) const { }
  bool equals(const Spec& other) const;
  const boost::shared_ptr<AdvancedScope>& getScope() const { return scope; }
  std::string get_value() const;
private:
  boost::shared_ptr<AdvancedScope> scope;
  std::string reference_name;
};

typedef typename boost::shared_ptr<StringReferenceSpec> StringReferenceSpecPtr;

class ConcatStringSpec : public StringSpec {
public:
    ConcatStringSpec(const StringSpecPtr& _lhs, const StringSpecPtr& _rhs)
    : lhs(_lhs) 
    , rhs(_rhs) {}

    bool equals(const Spec& other) const;
    void set_lhs(const StringSpecPtr& _lhs);
    void set_rhs(const StringSpecPtr& _rhs);
    inline const StringSpecPtr& get_lhs() const { return lhs; }
    inline const StringSpecPtr& get_rhs() const { return rhs; }
    std::string get_value() const;
    void get_input_specs(std::vector<const InputSpec*>& inputs) const { }
private:
    StringSpecPtr lhs, rhs;
};

typedef boost::shared_ptr<ConcatStringSpec> ConcatStringSpecPtr;


class ListSpec : public Spec {
public:
  virtual std::vector<SpecPtr> get_value() const = 0;
  virtual const SpecPtr innerType() const = 0;
};

typedef typename boost::shared_ptr<ListSpec> ListSpecPtr;

class ListReferenceSpec : public ListSpec
{
public:
  ListReferenceSpec(const std::string& refName, const boost::shared_ptr<AdvancedScope>& _scope, const SpecPtr& _innerType)
  : reference_name(refName)
  , scope(_scope)
  , innerT(_innerType) {}

  const SpecPtr innerType() const { return innerT; }
  const std::string& get_reference_name() const;
  void set_reference_name(const std::string& refName);
  void get_input_specs(std::vector<const InputSpec*>& inputs) const { }
  bool equals(const Spec& other) const;
  std::vector<SpecPtr> get_value() const;
  const boost::shared_ptr<AdvancedScope>& getScope() const { return scope; }
private:
  SpecPtr innerT;
  boost::shared_ptr<AdvancedScope> scope;
  std::string reference_name;
};

typedef typename boost::shared_ptr<ListReferenceSpec> ListReferenceSpecPtr;

class ConstListSpec : public ListSpec {
public:
	ConstListSpec(const std::vector<SpecPtr>& val);

  const SpecPtr innerType() const { 
    if (value.size() == 0) throw std::domain_error("Can not determine inner type of empty list"); 
    SpecPtr first = value[0]; 
    return createTypeObject(first); 
  }
	bool equals(const Spec& other) const;
	void set_value(const std::vector<SpecPtr>& val);
	std::vector<SpecPtr> get_value() const;
	void get_input_specs(std::vector<const InputSpec*>& inputs) const;
private:
	std::vector<SpecPtr> value;
};

typedef boost::shared_ptr<ConstListSpec> ConstListSpecPtr;

class ConcatListSpec : public ListSpec {
public:
    ConcatListSpec(const ListSpecPtr& _lhs, const ListSpecPtr& _rhs);

    const SpecPtr innerType() const { return lhs->innerType(); }
    bool equals(const Spec& other) const;
    void set_lhs(const ListSpecPtr& _lhs);
    void set_rhs(const ListSpecPtr& _rhs);
    inline const ListSpecPtr& get_lhs() const { return lhs; }
    inline const ListSpecPtr& get_rhs() const { return rhs; }
    std::vector<SpecPtr> get_value() const;
    void get_input_specs(std::vector<const InputSpec*>& inputs) const;
private:
    ListSpecPtr lhs, rhs;
};

typedef boost::shared_ptr<ConcatListSpec> ConcatListSpecPtr;
}
