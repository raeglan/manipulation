#pragma once
#include <giskard_suturo_parser/functions.h>
#include <resource_retriever/retriever.h>

#include <stack>

using namespace giskard_core;

  /// 
  /// Some utilities
  /// 

namespace giskard_suturo {

  /**
   * @brief      Custom wrapper for plain exceptions caused by parsing
   */
  struct ParseException : public std::exception {
    ParseException(std::string _msg)
    : msg(_msg)
    { }

    const char* what() const noexcept {
      return msg.c_str();
    }

    const std::string msg;
  };

  /**
   * @brief      Structure containing data for an import statement
   */
  struct Import {
    Import(std::string _path, std::string _alias) : path(_path), alias(_alias) {}
    std::string path;
    std::string alias;
  };
  typedef typename boost::shared_ptr<Import> ImportPtr;

  /**
   * @brief      Structure containing a declaration
   */
  struct Declaration {
    Declaration(std::string _name, SpecPtr _type) : name(_name), type(_type) {}
    std::string name;
    SpecPtr type;
  };
  typedef typename boost::shared_ptr<Declaration> DeclPtr;


  /**
   * @brief      Parser for the second suturo Giskard language. (GLang++)
   */
  class GiskardPPParser {
		typedef std::string::const_iterator SIt;
    
    /**
     * @brief      Internal structure used for generating stack traces
     */
    struct Context {
      SIt begin;
      size_t line;
      std::string name;
    };
  public:
    GiskardPPParser();

    /**
     * @return     The name of the parsed language
     */
    std::string language_name() const { return "Giskard++"; }
    
    /**
     * @return     File suffix associated with this language 
     */
    std::string file_suffix() const { return "gpp"; }

    /**
     * @brief      Parses a controller specification from a file
     *
     * @param[in]  filePath  File path
     *
     * @return     Controller specification
     */
    QPControllerSpec parseFromFile(const std::string& filePath);
    
    /**
     * @brief      Parses a controller specification form an input string.
     *
     * @param[in]  input  
     *
     * @return     Controller specification
     */
    QPControllerSpec parseFromString(const std::string& input);

    /**
     * @brief      Returns the scope that is currently active.
     *
     * @return     Active scope
     */
    boost::shared_ptr<AdvancedScope> getTopScope() const { return scopeStack.top(); }
    
    /**
     * @brief      Parse a scope from a string.
     *
     * @param[in]  input  Input string
     *
     * @return     Parsed scope
     */
    const boost::shared_ptr<AdvancedScope> parseScope(const std::string& input);
    
    /**
     * @brief      Parses an import from a string.
     *
     * @param[in]  input  Input string
     *
     * @return     Parsed input statement
     */
    ImportPtr parseImportStatement(const std::string& input);
    
    /**
     * @brief      Parses a scope entry from a string.
     *
     * @param[in]  input  Input String
     *
     * @return     Parsed scope entry
     */
    ScopeEntry parseNamedExpression(const std::string& input);
    
    /**
     * @brief      Parses an expression from a string.
     *
     * @param[in]  input  Input string
     *
     * @return     Parsed expression specification
     */
    SpecPtr parseExpression(const std::string& input);

    /**
     * @brief      Parses a term from a string.
     *
     * @param[in]  input  Input string
     *
     * @return     Parsed term specification
     */
    SpecPtr parseTerm(const std::string& input);
    
    /**
     * @brief      Parses a factor from a string.
     *
     * @param[in]  input  Input string
     *
     * @return     Parsed factor specification
     */
    SpecPtr parseFactor(const std::string& input);

    /**
     * @brief      Parses a member access from a string.
     *
     * @param[in]  input  Input string
     *
     * @return     Parsed member access specification
     */
    SpecPtr parseMemberAccess(const std::string& input);

    /**
     * @brief      Parses a literal from a string
     *
     * @param[in]  input  Input string
     *
     * @return     Parsed literal specification
     */
    SpecPtr parseLiteral(const std::string& input);

    /**
     * @brief      Parses a declaration from a string.
     *
     * @param[in]  input  Input string
     *
     * @return     Parsed declaration structure
     */
    DeclPtr parseDeclaration(const std::string& input);
    
    /**
     * @brief      Parses a type from a string.
     *
     * @param[in]  input  Input string
     *
     * @return     Pointer of parsed type
     */
    SpecPtr parseType(const std::string& input);
    
    /**
     * @brief      Parses a function definition from a string.
     *
     * @param[in]  input  Input string
     *
     * @return     Parsed function definition
     */
    FnDefPtr parseFunctionDefinition(const std::string& input);

    /**
     * @brief      Resets the internal state of the parser.
     */
    virtual void reset();

  private:

    /**
     * @brief      Sets the current parser input.
     *
     * @param[in]  input  Input string
     */
    void setInput(const std::string& input);

    /**
     * @brief      Parser rule, parsing and generating the "QPController" constructor.
     *
     * @return     Generated QPController specification
     */
    QPControllerSpec generateQP();

    /**
     * @brief      Parser rule parsing a scope.
     *
     * @return     Parsed scope
     */
    boost::shared_ptr<AdvancedScope> parseScope();

    /**
     * @brief      Parser rule parsing an import statement.
     *
     * @return     Parsed import statement
     */
    ImportPtr parseImportStatement();

    /**
     * @brief      Parser rule parsing a scope entry.
     *
     * @return     Parsed scope entry
     */
    ScopeEntry parseNamedExpression();

    /**
     * @brief      Parser rule parsing an expression.
     *
     * @return     Parsed expression
     */
    SpecPtr parseExpression();

    /**
     * @brief      Parser rule parsing a term.
     *
     * @return     Parsed term
     */
    SpecPtr parseTerm();

    /**
     * @brief      Parser rule parsing a factor.
     *
     * @return     Parsed factor
     */
    SpecPtr parseFactor();

    /**
     * @brief      Parser rule parsing member access.
     *
     * @return     Parsed member access
     */
    SpecPtr parseMemberAccess();

    /**
     * @brief      Parser rule parsing member.
     *
     * @return     Parsed literal
     */
    SpecPtr parseLiteral();

    /**
     * @brief      Function creating a reference based on a name and the current state of the parser.
     *
     * @param[in]  name  Name of the reference
     *
     * @return     Created reference
     */
    SpecPtr parseReference(std::string name);

    /**
     * @brief      Parser rule parsing an import statement.
     *
     * @return     Parsed import statement
     */
    DeclPtr parseDeclaration();
    
    /**
     * @brief      Parser rule parsing a type keyword.
     *
     * @return     Pointer of parsed type
     */
    SpecPtr parseType();

    /**
     * @brief      Parser rule parsing a function call.
     *
     * @param[in]  name  Name of the function
     *
     * @return     Parsed function call
     */
    SpecPtr parseFunctionCall(const std::string& name);

    /**
     * @brief      Parser rule parsing a function definition.
     *
     * @return     Parsed function definition
     */
    FnDefPtr parseFunctionDefinition();

    /**
     * @brief      Skips at least one character and all the following white spaces, line breaks and comments  
     *
     * @return     The character the skipper stopped on
     */
    char moveahead();

    /**
     * @brief      Skipper that returns true if it skipped the given character.
     *
     * @param[in]  c     Character that should be skipped
     *
     * @return     True if c was skipped
     */
    bool skipChar(char c);

    /**
     * @brief      Parser rule parses a string consisting of alphanumeric characters and underscores. The string must not begin with a digit.
     *
     * @return     Parsed string
     */
    std::string lstr();

    /**
     * @brief      Parser rule parsing a quoted string.
     *
     * @return     Parsed string
     */
    std::string qstr();

    /**
     * @brief      Parser rule parsing a scalar value.
     *
     * @return     { description_of_the_return_value }
     */
    double scalar();
    
    /**
     * @brief      Skipper rule, skipping white spaces, line breaks and comments.
     *
     * @return     Character the skipper stopped on
     */
    char skip();

    /**
     * @brief      Helper function generating a readable name for the type of a specification.
     *
     * @param      ptr   Specification pointer
     *
     * @return     Type string
     */
    std::string typeString(SpecPtr& ptr);

    /**
     * @brief      Helper function generating a readable list of type names for a list of specifications.
     *
     * @param      v     List of specifications
     *
     * @return     List type string
     */
    std::string typeList(std::vector<SpecPtr>& v);

    /** Resource retriever used for retrieving external files */
    resource_retriever::Retriever resourceRetriever;

    /** Current scope to be searched for symbols */
    boost::shared_ptr<AdvancedScope> searchScope;

    /** Stack of scopes being parsed */
    std::stack<boost::shared_ptr<AdvancedScope>> scopeStack;

    /** Current rule call trace */
    std::stack<Context> traceStack;

    /** Prefix used for generating resolved reference names */
    std::string accessPrefix;

    /**
     * @brief      Determines whether a name is already taken by a keyword.
     *
     * @param[in]  name  Name to be checked
     *
     * @return     True if blacklisted, False otherwise.
     */
    inline bool isBlacklisted(std::string name) {
      return std::find(blacklist.begin(), blacklist.end(), name) != blacklist.end(); 
    }

    /**
     * @brief      Function for throwing parser errors containing a call trace.
     *
     * @param[in]  msg   Error message
     */
    void throwError(std::string msg);

    /** Last parsed keyword */
    std::string lastKeyword;

    /** Cursor iterator */
    SIt it;

    /** End of input iterator */
    SIt end;

    /** Current line number */
    size_t lineNumber;

    /** Begin of line iterator */
    SIt l_begin;

    /** Begin of literal iterator */
    SIt litIt;

    /** List of keywords reserved by the language */
    const std::vector<std::string> blacklist;
public:
    /** Keywords reserved by the language */
    static const std::string sScalar, sVec3, sRotation, sFrame, sControllable, sSoft, sHard, sString, sMap, sList, sScope;
  };
}
