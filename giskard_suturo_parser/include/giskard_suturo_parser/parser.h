#pragma once
#include <giskard_suturo_parser/functions.h>
#include <resource_retriever/retriever.h>

#include <stack>

using namespace giskard_core;

  /// 
  /// Some utilities
  /// 

namespace giskard_suturo {

  struct ParseException : public std::exception {
    ParseException(std::string _msg)
    : msg(_msg)
    { }

    const char* what() const noexcept {
      return msg.c_str();
    }

    const std::string msg;
  };

  struct Import {
    Import(std::string _path, std::string _alias) : path(_path), alias(_alias) {}
    std::string path;
    std::string alias;
  };
  typedef typename boost::shared_ptr<Import> ImportPtr;

  struct Declaration {
    Declaration(std::string _name, SpecPtr _type) : name(_name), type(_type) {}
    std::string name;
    SpecPtr type;
  };
  typedef typename boost::shared_ptr<Declaration> DeclPtr;


  class GiskardPPParser {
		typedef std::string::const_iterator SIt;
    
    struct Context {
      SIt begin;
      size_t line;
      std::string name;
    };
  public:
    GiskardPPParser();

    std::string language_name() const { return "Giskard++"; }
    std::string file_suffix() const { return "gpp"; }
    QPControllerSpec parseFromFile(const std::string& filePath);
    QPControllerSpec parseFromString(const std::string& file);

    boost::shared_ptr<AdvancedScope> getTopScope() const { return scopeStack.top(); }
    const boost::shared_ptr<AdvancedScope> parseScope(const std::string& input);
    ImportPtr parseImportStatement(const std::string& input);
    ScopeEntry parseNamedExpression(const std::string& input);
    SpecPtr parseExpression(const std::string& input);
    SpecPtr parseTerm(const std::string& input);
    SpecPtr parseFactor(const std::string& input);
    SpecPtr parseMemberAccess(const std::string& input);
    SpecPtr parseLiteral(const std::string& input);
    DeclPtr parseDeclaration(const std::string& input);
    SpecPtr parseType(const std::string& input);
    //SpecPtr functionCall(const std::string& input);
    FnDefPtr parseFunctionDefinition(const std::string& input);

    virtual void reset();

  private:
    void setInput(const std::string& input);

    QPControllerSpec generateQP();
    boost::shared_ptr<AdvancedScope> parseScope();
    ImportPtr parseImportStatement();
    ScopeEntry parseNamedExpression();
    SpecPtr parseExpression();
    SpecPtr parseTerm();
    SpecPtr parseFactor();
    SpecPtr parseMemberAccess();
    SpecPtr parseLiteral();
    SpecPtr parseReference(std::string name);
    DeclPtr parseDeclaration();
    SpecPtr parseType();
    SpecPtr parseFunctionCall(const std::string& name);
    FnDefPtr parseFunctionDefinition();
    char moveahead();
    bool skipChar(char c);
    std::string lstr();
    std::string qstr();
    double scalar();
    char skip();

    std::string typeString(SpecPtr& ptr);
    std::string typeList(std::vector<SpecPtr>& v);

    resource_retriever::Retriever resourceRetriever;

    boost::shared_ptr<AdvancedScope> searchScope;
    std::stack<boost::shared_ptr<AdvancedScope>> scopeStack;
    std::stack<Context> traceStack;
    std::string accessPrefix;
    //boost::shared_ptr<std::unordered_map<std::string, IScopePtr>> cachedfiles;


    inline bool isBlacklisted(std::string name) {
      return std::find(blacklist.begin(), blacklist.end(), name) != blacklist.end(); 
    }

    void throwError(std::string msg);

    std::string lastKeyword;
    std::string dir;

    SIt it;
    SIt end;
    size_t lineNumber;
    SIt l_begin;
    SIt litIt;
    const std::vector<std::string> blacklist;
public:
    static const std::string sScalar, sVec3, sRotation, sFrame, sControllable, sSoft, sHard, sString, sMap, sList, sScope;
  };
}
