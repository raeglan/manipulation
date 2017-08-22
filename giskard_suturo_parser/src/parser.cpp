#include "giskard_suturo_parser/parser.h"
#include "giskard_suturo_parser/utils.h"

#include <resource_retriever/retriever.h>

namespace giskard_suturo {

#define CALL_RULE(f, r) \
	traceStack.push({it, lineNumber, #f}); \
	r = f; \
	traceStack.pop()

GiskardPPParser::GiskardPPParser() 
    : blacklist({"def", "return", "import",
        sScalar, sVec3, sRotation, sFrame,
        sString, sControllable, sSoft, sHard,
        sMap, sList, sScope})
{
//    blacklist.push_back("def");
//    blacklist.push_back("return");
//    blacklist.push_back("import");
//    blacklist.push_back(sScalar);
//    blacklist.push_back(sVec3);
//    blacklist.push_back(sRotation);
//    blacklist.push_back(sFrame);
//    blacklist.push_back(sString);
//    blacklist.push_back(sControllable);
//    blacklist.push_back(sSoft);
//    blacklist.push_back(sHard);
//    blacklist.push_back(sMap);
//    blacklist.push_back(sList);
//    blacklist.push_back(sScope);
}

std::string GiskardPPParser::typeString(SpecPtr& ptr) {
	std::string out = toTypeString(ptr);

	if (out.empty())
		throwError("Failed to stringify type!");
	
	return out;
}

std::string GiskardPPParser::typeList(std::vector<SpecPtr>& v) {
	if (v.size() == 0)
		return "()";

	std::stringstream ss;
	ss << '(' << typeString(v[0]);

	for (size_t i = 1; i < v.size(); i++) {
		ss << ", " << typeString(v[i]);
	}

	ss << ')';
	return ss.str();
}


using boost::dynamic_pointer_cast;
using boost::static_pointer_cast;

const DoubleConstSpecPtr constZero = instance<DoubleConstSpec>(0);
const DoubleConstSpecPtr constNegOne = instance<DoubleConstSpec>(-1.0);
const VectorSpecPtr constZeroVec = instance<VectorConstructorSpec>();

void GiskardPPParser::setInput(const std::string& input) {
	it = input.begin();
	end = input.end();
	litIt = it;
	lineNumber = 1;
	lastKeyword = "";
	if (scopeStack.empty()) {
		scopeStack.push(AdvancedScopePtr(new AdvancedScope()));
		searchScope = scopeStack.top();
	}
}

void GiskardPPParser::reset() {
	it = end;
	lineNumber = 0;
	lastKeyword = "";
	while(!scopeStack.empty())
		scopeStack.pop();
}

QPControllerSpec GiskardPPParser::parseFromFile(const std::string& filePath) {
	std::ifstream file(filePath);

	if (!file.is_open())
		throwError("Can't open file: " + filePath);

	std::stringstream buffer;
	buffer << file.rdbuf();
    std::string input = buffer.str();
    setInput(input);

	scopeStack.push(AdvancedScopePtr(new AdvancedScope(filePath)));
	searchScope = scopeStack.top();
	const AdvancedScopePtr scope = parseScope();

	CALL_RULE(generateQP(), QPControllerSpec out);
	file.close();
	return out;
}

QPControllerSpec GiskardPPParser::parseFromString(const std::string& string) {
	setInput(string);
	scopeStack.push(AdvancedScopePtr(new AdvancedScope()));
	searchScope = scopeStack.top();
	const AdvancedScopePtr scope = parseScope();
	CALL_RULE(generateQP(), QPControllerSpec out);

	return out;
}


QPControllerSpec GiskardPPParser::generateQP() {
	if (lastKeyword == "QPController") {
		std::vector<SpecPtr> arguments;

		searchScope = scopeStack.top();
		do {
			CALL_RULE(parseExpression(), SpecPtr e);
			if (!e)
				throwError("Empty expression in argument list.");

			arguments.push_back(e);
		} while(skipChar(','));

		ListSpecPtr conts, softs, hards;
		ControllableConstraintSpecPtr cc;
		SoftConstraintSpecPtr sc;
		HardConstraintSpecPtr hc;
		if (arguments.size() == 3 && matches(arguments[0], conts) && matches(arguments[1], softs) && matches(arguments[2], hards)
			&& typesAreEqual(conts->innerType(), cc) && typesAreEqual(softs->innerType(), sc) && typesAreEqual(hards->innerType(), hc)) {
			std::vector<SpecPtr> temp = conts->get_value();
			QPControllerSpec out;
			for (size_t i = 0; i < temp.size(); i++) {
				ControllableConstraintSpecPtr c;
				if (!matches(temp[i], c))
					throwError("List is not type homogenous");
				out.controllable_constraints_.push_back(*c);
			}

			temp = softs->get_value();
			for (size_t i = 0; i < temp.size(); i++) {
				SoftConstraintSpecPtr c;
				if (!matches(temp[i], c))
					throwError("List is not type homogenous");
				out.soft_constraints_.push_back(*c);
			}

			temp = hards->get_value();
			for (size_t i = 0; i < temp.size(); i++) {
				HardConstraintSpecPtr c;
				if (!matches(temp[i], c))
					throwError("List is not type homogenous");
				out.hard_constraints_.push_back(*c);
			}

			scopeStack.top()->convert(out.scope_, "");
			return out;
		} else 
			throwError("QPController can not be instanciated with types " + toTypeList(arguments));
	}
	throwError("Expected 'QPController'");
}

const AdvancedScopePtr GiskardPPParser::parseScope(const std::string& input) {
	setInput(input);
	return parseScope();
}

ImportPtr GiskardPPParser::parseImportStatement(const std::string& input) {
	setInput(input);
	return parseImportStatement();
}

ScopeEntry GiskardPPParser::parseNamedExpression(const std::string& input) {
	setInput(input);
    CALL_RULE(parseNamedExpression(), ScopeEntry out);
    scopeStack.top()->addSpec(out);
    return out;
}

SpecPtr GiskardPPParser::parseExpression(const std::string& input) {
	setInput(input);
	return parseExpression();
}

SpecPtr GiskardPPParser::parseTerm(const std::string& input) {
	setInput(input);
	return parseTerm();
}

SpecPtr GiskardPPParser::parseFactor(const std::string& input) {
	setInput(input);
	return parseFactor();
}

SpecPtr GiskardPPParser::parseMemberAccess(const std::string& input) {
	setInput(input);
	return parseMemberAccess();
}

SpecPtr GiskardPPParser::parseLiteral(const std::string& input) {
	setInput(input);
	return parseLiteral();
}

DeclPtr GiskardPPParser::parseDeclaration(const std::string& input) {
	setInput(input);
	return parseDeclaration();
}

SpecPtr GiskardPPParser::parseType(const std::string& input) {
	setInput(input);
	return parseType();
}

// SpecPtr functionCall(const std::string& input) {
// 	setInput(input);
// 	return functionCall();
// }

FnDefPtr GiskardPPParser::parseFunctionDefinition(const std::string& input) {
	setInput(input);
    CALL_RULE(parseFunctionDefinition(), FnDefPtr out);
	scopeStack.top()->addFunction(out);
	return out;
}

AdvancedScopePtr GiskardPPParser::parseScope() {
	CALL_RULE(lstr(), lastKeyword);

	while (lastKeyword == "import") {
		CALL_RULE(parseImportStatement(), ImportPtr import);
		if(!skipChar(';'))
			throwError("Expected ';'.");

		std::string folder = "";
		size_t folderIdx = import->path.rfind('/');
		if (folderIdx != std::string::npos)
			folder = import->path.substr(0, folderIdx + 1);

		AdvancedScopePtr existingScope = searchScope->getScopeByFile(import->path);
		if (existingScope) {
			if (searchScope->isSuperScope(import->path)) {
				if (!import->alias.empty()) {
					searchScope->addScope(import->alias, existingScope);
				}
			} else {
				if (import->alias.empty()) {
					searchScope->addScope(existingScope);
				}
			}
		} else {
			std::string otherFile;
            std::string filePath = import->path;
            if (import->path[0] != '/' && import->path.find("://") == std::string::npos) {
                filePath = searchScope->searchPath + import->path;
                folder = searchScope->searchPath + folder;
            }

            if (filePath.find("://") != std::string::npos) {
				try {
                    resource_retriever::MemoryResource resource = resourceRetriever.get(filePath);
					otherFile = std::string((const char*)resource.data.get(), resource.size);
				} catch (resource_retriever::Exception& e) {
					throwError("Error while retrieving resource for import: " + std::string(e.what()));
				} 
			} else {
				std::ifstream file(filePath);

				if (!file.is_open())
					throwError("Can't open file: " + filePath);

				std::stringstream buffer;
				buffer << file.rdbuf();
				otherFile = buffer.str();
			}

			SIt stashedCursor = it;
			SIt stashedEnd = end;
			SIt stashedLitIt = litIt;
			size_t stashedLineNumber = lineNumber;
			scopeStack.push(AdvancedScopePtr(new AdvancedScope(import->path, folder, searchScope->getSuperScopes())));
			searchScope = scopeStack.top();

			std::string stashedPrefix = accessPrefix;
			if (!import->alias.empty())
				accessPrefix += import->alias + "::";

			setInput(otherFile);
			AdvancedScopePtr importedScope = parseScope();
			scopeStack.pop();

			it = stashedCursor;
			end = stashedEnd;
			litIt = stashedLitIt;
			lineNumber = stashedLineNumber;
			accessPrefix = stashedPrefix;

			searchScope = scopeStack.top();
			if (import->alias.empty())
				searchScope->addScope(importedScope);
			else
				searchScope->addScope(import->alias, importedScope);
		}
		CALL_RULE(lstr(), lastKeyword);
	}

	while (lastKeyword != "") {
		if (lastKeyword == "def") {
            CALL_RULE(parseFunctionDefinition(), FnDefPtr fnPtr);
			if (!skipChar(';'))
				throwError("Expected ';'.");
            scopeStack.top()->addFunction(fnPtr);
		} else if (lastKeyword == "QPController") {
			return scopeStack.top();		
		} else {
			CALL_RULE(parseNamedExpression(), ScopeEntry e);
			if (!skipChar(';'))
				throwError("Expected ';'.");
			scopeStack.top()->addSpec(e);
		}
		CALL_RULE(lstr(), lastKeyword);
	}
	return scopeStack.top();
}

ImportPtr GiskardPPParser::parseImportStatement() {
	skip();
	litIt = it;

    if (lastKeyword == "") {
		CALL_RULE(lstr(), lastKeyword);
    }

	if (lastKeyword == "import") {
		CALL_RULE(qstr(), std::string path);

		if (path.empty())
			throwError("Import path can not be empty.");

		CALL_RULE(lstr(), lastKeyword);
		std::string alias = "";
		if (lastKeyword == "as") {
			CALL_RULE(lstr(), alias);
			if (alias.empty())
				throwError("Namespace alias can not be empty.");

			if (isBlacklisted(alias)) 
				throwError("Illegal alias '" + alias + "'. Name is a keyword.");

			if (scopeStack.top()->isNameTaken(alias))
				throwError("Illegal alias '" + alias + "'. Name is already taken.");
		}

		return ImportPtr(new Import(path, alias));
	}

	throwError("Expected keyword 'import'. Got: '" + lastKeyword + "'");
}

ScopeEntry GiskardPPParser::parseNamedExpression() {
	std::string name = lastKeyword;
    if (name.empty()) {
		CALL_RULE(lstr(), name);
    }

	if (isBlacklisted(name))
		throwError("Can not declare expression with name '" + name + "'. The name refers to a keyword.");

	if (searchScope->isNameTaken(name))
		throwError("Can not declare expression with name '" + name + "'. The name is already taken.");


	// TODO: Check collisions in scope.

	if (!skipChar('='))
		throwError("Expected '='.");

	CALL_RULE(parseExpression(), SpecPtr expr);
	return {name, expr};
}

SpecPtr GiskardPPParser::parseExpression() { 

	CALL_RULE(parseTerm(), SpecPtr lhs);

	char c = skip();

	if (c == '+' || c == '-') {
		moveahead();
        CALL_RULE(parseExpression(), SpecPtr rhs);
		
		if (c == '+') {
			// + overloads
			DoubleSpecPtr dlhs, drhs;
			VectorSpecPtr vlhs, vrhs;
			ListSpecPtr  llhs, lrhs;
			StringSpecPtr slhs, srhs;
			// AMapSpecPtr  mlhs, mrhs;
			if (matches(lhs, rhs, dlhs, drhs)) {
				return instance<DoubleAdditionSpec>(dlhs, drhs);
			} else if (matches(lhs, rhs, vlhs, vrhs)) {
				return instance<VectorAdditionSpec>(vlhs, vrhs);
			} else if (matches(lhs, rhs, llhs, lrhs)) {
				return instance<ConcatListSpec>(llhs, lrhs);
			} else if (matches(lhs, rhs, slhs, srhs)) {
				return instance<ConcatStringSpec>(slhs, srhs);
			} //else if (matches(lhs, rhs, mlhs, mrhs)) {
			// 	throw NotImplementedException();
			// }
			throwError("No overload known for '+' matching '" + typeString(lhs) + " + " + typeString(rhs) + "'");
		} else {
			// - overloads
			DoubleSpecPtr dlhs, drhs;
			VectorSpecPtr vlhs, vrhs;
			if (matches(lhs, rhs, dlhs, drhs)) {
				return instance<DoubleSubtractionSpec>(dlhs, drhs);
			} else if (matches(lhs, rhs, vlhs, vrhs)) {
				return instance<VectorSubtractionSpec>(vlhs, vrhs);
			} 

			throwError("No overload known for '-' matching '" + typeString(lhs) + " - " + typeString(rhs) + "'");
		}	
	}

	return lhs;
}

SpecPtr GiskardPPParser::parseTerm() {

	CALL_RULE(parseFactor(), SpecPtr lhs);

	char c = skip();

	if (c == '*' || c == '/') {
		moveahead();
        CALL_RULE(parseTerm(), SpecPtr rhs);
		if (c == '*') {
			// * overloads
			DoubleSpecPtr dlhs, drhs;
			VectorSpecPtr vlhs, vrhs;
			RotationSpecPtr rlhs, rrhs;
			FrameSpecPtr flhs, frhs;
			if (matches(lhs, rhs, dlhs, drhs)) {

				return instance<DoubleMultiplicationSpec>(dlhs, drhs);
			} else if (matches(lhs, rhs, dlhs, vrhs)) {

				return instance<VectorDoubleMultiplicationSpec>(dlhs, vrhs);
			} else if (matches(lhs, rhs, vlhs, drhs)) {

				return instance<VectorDoubleMultiplicationSpec>(drhs, vlhs);
			} else if (matches(lhs, rhs, vlhs, vrhs)) {

				return instance<VectorDotSpec>(vlhs, vrhs);
			} else if (matches(lhs, rhs, rlhs, rrhs)) {
				
				return instance<RotationMultiplicationSpec>(rlhs, rrhs);
			} else if (matches(lhs, rhs, flhs, frhs)) {
				
				return instance<FrameMultiplicationSpec>(flhs, frhs);
			} else if (matches(lhs, rhs, rlhs, vrhs)) {
				
				return instance<VectorRotationMultiplicationSpec>(rlhs, vrhs);
			} else if (matches(lhs, rhs, flhs, vrhs)) {
				
				return instance<VectorFrameMultiplicationSpec>(flhs, vrhs);
			}

			throwError("No overload known for '*' matching '" + typeString(lhs) + " * " + typeString(rhs) + "'");
		} else {
			// / overloads
			DoubleSpecPtr dlhs, drhs;
			if (matches(lhs, rhs, dlhs, drhs)) {

				return instance<DoubleDivisionSpec>(dlhs, drhs);
			}

			throwError("No overload known for '/' matching '" + typeString(lhs) + " / " + typeString(rhs) + "'");
		}

	}

	return lhs;	
}

SpecPtr GiskardPPParser::parseFactor() {

	char c = skip();
	if (c == '-') {
		moveahead();
		CALL_RULE(parseFactor(), SpecPtr fac);
		// Negation

		DoubleSpecPtr dblSpec;
		VectorSpecPtr vecSpec;
		if (matches(fac, dblSpec)) {
			DoubleConstSpecPtr cd = dynamic_pointer_cast<DoubleConstSpec>(dblSpec);
			if (cd) {
				return instance<DoubleConstSpec>(-cd->get_value());
			} 

			return instance<DoubleSubtractionSpec>(dynamic_pointer_cast<DoubleSpec>(constZero), dblSpec);
		} else if (matches(fac, vecSpec)) {
			return instance<VectorDoubleMultiplicationSpec>(dynamic_pointer_cast<DoubleSpec>(constNegOne), vecSpec);
		}

		throwError("No operation known to negate '" + typeString(fac) + "'");
	} else {
		CALL_RULE(parseMemberAccess(), SpecPtr out);
		return out;
	}
}

SpecPtr GiskardPPParser::parseMemberAccess() {
	CALL_RULE(parseLiteral(), SpecPtr lit);

	if (it != end && (*it) == '.') {
		moveahead();

		VectorSpecPtr vec;
		FrameSpecPtr frame;
		AdvancedScopePtr subScope;
		if (matches(lit, vec)) {
			CALL_RULE(lstr(), std::string att);

			if (att == "x") {
				return instance<DoubleXCoordOfSpec>(vec);
			} else if (att == "y") {
				return instance<DoubleYCoordOfSpec>(vec);
			} else if (att == "z") {
				return instance<DoubleZCoordOfSpec>(vec);
			}

			throwError(sVec3 + " has no attribute '" + att + "'.");
		} else if(matches(lit, frame)) {
			std::string att = lstr();

			if (att == "pos") {
				return instance<VectorOriginOfSpec>(frame);
			} else if (att == "rot") {
				return instance<OrientationOfSpec>(frame);
			} 

			throwError(sFrame + " has no attribute '" + att + "'.");
		} else if(matches(lit, subScope)) {
			searchScope = subScope;
			std::string currentPrefix = accessPrefix;
			accessPrefix += lastKeyword + "::";
			CALL_RULE(parseLiteral(), SpecPtr member);
			searchScope = scopeStack.top();
			accessPrefix = currentPrefix;
			return member;
		}

	}

	return lit;
}

SpecPtr GiskardPPParser::parseLiteral() {
	litIt = it;
	char c = skip();

	if (c == '(') {
		moveahead();
		CALL_RULE(parseExpression(), SpecPtr out);
		if (!out)
			throwError("Empty brackets.");

		skipChar(')');
		return out;
	} else if (c == '[') {
		moveahead();

		std::vector<SpecPtr> elems;

		do {
			CALL_RULE(parseExpression(), SpecPtr e);
			if (!e)
				throwError("Empty expression in list definition.");
			
			if (elems.size() > 0 && !matches(elems[0], e))
				throwError("Mismatched inner type!\n    Inner type is: " + typeString(elems[0]) + "\n    Inserted type is: " + typeString(e));

			std::cout << elems.size() << ". element is of type " << typeString(e) << std::endl;

			elems.push_back(e);
		} while(skipChar(','));

        if (!skipChar(']'))
			throwError("Expected ']' at end of list");

		return instance<ConstListSpec>(elems);
	} else if (c == '"') {
		CALL_RULE(qstr(), std::string s);
		return instance<ConstStringSpec>(s);
	}

	CALL_RULE(lstr(), std::string word);
	if (word =="") {
		CALL_RULE(scalar(), double d);
		return instance<DoubleConstSpec>(d);
	}

	if (skipChar('(')) {
		CALL_RULE(parseFunctionCall(word), SpecPtr out);
		return out;
	}

	CALL_RULE(parseReference(word), SpecPtr out);
	return out;
}

SpecPtr GiskardPPParser::parseFunctionCall(const std::string& name) {
	std::vector<SpecPtr> arguments;

	AdvancedScopePtr fnSearch = searchScope;
	searchScope = scopeStack.top();
	do {
		CALL_RULE(parseExpression(), SpecPtr e);
		if (!e)
			throwError("Empty expression in argument list.");

		arguments.push_back(e);
	} while(skipChar(','));

    if (!skipChar(')'))
        throwError("Expected closing ')' in function call.");

    searchScope = fnSearch;

    SpecPtr out;
	if (scopeStack.top() == searchScope) {
        out = searchScope->callFunction(name, arguments, scopeStack.top());
	} else {
        out = searchScope->callLocalFunction(name, arguments, scopeStack.top());
	}
    if (!out)
        throwError("Function '" + name + toTypeList(arguments) + "' doesn't exist!");

    return out;
}

SpecPtr GiskardPPParser::parseReference(std::string name) {
	// Resolve reference
	if (scopeStack.top() == searchScope) {
		SpecPtr spec = searchScope->getSpec(name);
		if (spec)
            return createReferenceSpec(name, spec, searchScope);

		SpecPtr out = searchScope->getScope(name);
		if (out) {
			lastKeyword = name;
			return out;
		}
	} else {
		SpecPtr spec = searchScope->getLocalSpec(name);
		if (spec) {
			if (dynamic_pointer_cast<StringSpec>(spec) || dynamic_pointer_cast<ListSpec>(spec))
            	return createReferenceSpec(name, spec, searchScope);
            else 
            	return createReferenceSpec(accessPrefix + name, spec, searchScope);
		}
	}

	throwError("Unknown name '" + name +"' in scope '" + searchScope->filePath + "'");
}

FnDefPtr GiskardPPParser::parseFunctionDefinition() {

	// Check name 
	if (lastKeyword == "")
		lastKeyword = lstr();

	if (!(lastKeyword == "def"))
		throwError("Expected keyword 'def'.");

	CALL_RULE(parseDeclaration(), DeclPtr fnDecl);
	SpecPtr typeTrace = fnDecl->type;
	std::string fnName = fnDecl->name;

	if (isBlacklisted(fnName))
		throwError("Illegal function name '"+ fnName +"'! The name refers to a keyword.");

	if (searchScope->isNameTaken(fnName))
		throwError("Can't declare new function. Name '"+ fnName +"' is already taken.");

	if (!skipChar('('))
		throwError("Expected '(' to begin function argument list.");

    FnDefPtr functionDefinition = FnDefPtr(new FunctionDefinition(fnName, scopeStack.top()));
	std::vector<SpecPtr> signature;

	do {
		CALL_RULE(parseDeclaration(), DeclPtr argDecl);

		signature.push_back(argDecl->type);
		functionDefinition->addArgument(argDecl->name, argDecl->type);
	} while(skipChar(','));

	if (!skipChar(')'))
		throwError("Expected ')'.");

	if (scopeStack.top()->hasFunction(fnName, signature))
		throwError("Function '" + fnName + toTypeList(signature) + "' is already defined.");

	if (!skipChar('{'))
		throwError("Expected '{'.");

	scopeStack.push(functionDefinition);
	searchScope = functionDefinition;


	do {
        CALL_RULE(lstr(), lastKeyword);
		if (lastKeyword == "return") {
			CALL_RULE(parseExpression(), SpecPtr expr);
			if (!typesAreEqual(typeTrace, expr)) {
				scopeStack.pop();
				searchScope = scopeStack.top();
				throwError("Mismatched return type! Expected: '" + typeString(typeTrace) + "' Got: '" + typeString(expr) + "'.");
			}
			if (!skipChar(';')) {
				scopeStack.pop();
				searchScope = scopeStack.top();
				throwError("Expected ';'.");
			}

			if (!skipChar('}')) {
				scopeStack.pop();
				searchScope = scopeStack.top();
				throwError("Expected '}'.");
			}

            functionDefinition->setReturnSpec(expr);
			scopeStack.pop();
			searchScope = scopeStack.top();
			return functionDefinition;
		} else {
            CALL_RULE(parseNamedExpression(), ScopeEntry se);
			functionDefinition->addSpec(se.name, se.spec);
		}
	} while(skipChar(';'));

	scopeStack.pop();
	searchScope = scopeStack.top();
	throwError("Unexpected end of function '" + fnName + "'.");
}

DeclPtr GiskardPPParser::parseDeclaration() {
	CALL_RULE(parseType(), SpecPtr type);
	CALL_RULE(lstr(), std::string name);
	if (name.empty())
		throwError("Empty declaration name.");
	return DeclPtr(new Declaration(name, type));
}

SpecPtr GiskardPPParser::parseType() {
	std::string word = lstr();

	if (word == sScalar) { 
		return instance<DoubleConstSpec>(0);
	
	} else if (word == sVec3) { 
		return instance<VectorConstructorSpec>();
	
	} else if (word == sRotation) { 
		return instance<RotationQuaternionConstructorSpec>();
	
	} else if (word == sFrame) { 
		return instance<FrameConstructorSpec>();
	
	} else if (word == sControllable) { 
		return instance<ControllableConstraintSpec>();
	
	} else if (word == sSoft) { 
		return instance<SoftConstraintSpec>();
	
	} else if (word == sHard) { 
		return instance<HardConstraintSpec>();
	
	} else if (word == sString) { 
		return instance<ConstStringSpec>("");
	
	} else if (word == sScope) { 
		return instance<AdvancedScope>();
	
	// } 
	// else if (word == sMap) { 
	// 	if (!skipChar('<'))
	// 		throwError("Inner type for " + word + " needed. Declare using '<', '>'");

	// 	SpecPtr subTrace = type();
		
	// 	if (!skipChar('>'))
	// 		throwError("Expected '>' at the end of inner type declaration.");

	// 	return instance<AMapSpec>(subTrace);			
	} else if (word == sList) { 
		if (!skipChar('<'))
			throwError("Inner type for " + word + " needed. Declare using '<', '>'");

		SpecPtr subTrace = parseType();
		
		if (!skipChar('>'))
			throwError("Expected '>' at the end of inner type declaration.");

		std::vector<SpecPtr> temp {subTrace};
		return instance<ConstListSpec>(temp);			
	}

	throwError("Unknown type '" + word + "'");
}


char GiskardPPParser::moveahead() {
	auto conB = it;
	it++;
	if (it == end)
		throwError("Unexpected end of input!");
	return skip();
}

bool GiskardPPParser::skipChar(char c) {
	if (skip() == c) {
		it++;
		skip();
		return true;
	}
	return false;
}

std::string GiskardPPParser::lstr() {
	skip();

	auto ab = it;

	while (it != end) {
		char c = *it;

		if (ab == it && std::isdigit(c))
			break;

		if (!std::isalnum(c) && c != '_')
			break;

		it++;
	}

	return std::string(ab, it);
}

std::string GiskardPPParser::qstr() {
	if (skip() != '"')
		throwError("Expected '\"'.");

	it++;

	auto ab = it;

	while (it != end) {
		char c = *it;

		if (c == '"')
			break;

		it++;
	}

	if (it == end)
		throwError("Expected closing '\"'.");

	it++;

	return std::string(ab, it - 1);
}

double GiskardPPParser::scalar() {
	if(!skip())
		throwError("Unexpected end of file");
	auto sb = it;
	char c = *it;
	while(it != end && (isdigit(c) || c == '.')) {
		it++;
		c = *it;
	}


	double out = stod(std::string(sb, it));
	return out;
}

char GiskardPPParser::skip() {
	bool comment = false;
	while (it != end) {
		char c = *it;
		if (!comment) {
			switch (c) {
				case ' ':
				case '\t':
					break;
				case '\n':
					lineNumber++;
					l_begin = it+1;
					break;
				case '#':
					comment = true;
					break;
				default:
					return c;
			}
		} else {
			comment = c != '\n';
		}
		it++;
	}
	return 0;
}

void GiskardPPParser::throwError(std::string msg) {
	std::string temp = "";
	while (!traceStack.empty()) {
		Context c = traceStack.top();
		temp += "In ";
		temp += c.name + " from line ";
		temp += std::to_string(c.line);
		temp += ":\n" + std::string(c.begin, it);
		temp += "\n";
		traceStack.pop();
	}
	throw ParseException(temp + msg);
}

const std::string GiskardPPParser::sScalar = "scalar";
  const std::string GiskardPPParser::sVec3 = "vec3";
  const std::string GiskardPPParser::sRotation = "rotation";
  const std::string GiskardPPParser::sFrame = "frame";
  const std::string GiskardPPParser::sControllable = "controllableC";
  const std::string GiskardPPParser::sSoft = "softC";
  const std::string GiskardPPParser::sHard = "hardC";
  const std::string GiskardPPParser::sString = "string";
  const std::string GiskardPPParser::sMap = "map";
  const std::string GiskardPPParser::sList = "list";
  const std::string GiskardPPParser::sScope = "scope";

}
