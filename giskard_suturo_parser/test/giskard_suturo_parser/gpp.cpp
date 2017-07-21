/*
 * Copyright (C) 2015 Georg Bartels <georg.bartels@cs.uni-bremen.de>
 * 
 * This file is part of giskard.
 * 
 * giskard is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <gtest/gtest.h>
#include <giskard_suturo_parser/parser.h>
#include <giskard_suturo_parser/utils.h>
#include <giskard_core/giskard_core.hpp>

using namespace giskard_suturo;
using namespace giskard_core;

class GiskardPPTest : public ::testing::Test
{
   protected:
    virtual void SetUp() {
    	parser.reset();
    }

    virtual void TearDown(){}

    GiskardPPParser parser;
};

TEST_F(GiskardPPTest, matchTest) {
	SpecPtr scalar = instance<DoubleConstSpec>(1.0);
	SpecPtr vector = instance<VectorConstructorSpec>();
	SpecPtr rotation = instance<RotationQuaternionConstructorSpec>(0.0,0.0,0.0,1.0);
	SpecPtr frame = instance<FrameConstructorSpec>(instance<VectorConstructorSpec>(), instance<RotationQuaternionConstructorSpec>(0.0,0.0,0.0,1.0));
	DoubleSpecPtr d;
	VectorSpecPtr v;
	RotationSpecPtr r;
	FrameSpecPtr f;
	EXPECT_TRUE(matches(scalar, d));
	EXPECT_FALSE(matches(scalar, v));
	EXPECT_FALSE(matches(scalar, r));
	EXPECT_FALSE(matches(scalar, f));

	EXPECT_FALSE(matches(vector, d));
	EXPECT_TRUE(matches(vector, v));
	EXPECT_FALSE(matches(vector, r));
	EXPECT_FALSE(matches(vector, f));

	EXPECT_FALSE(matches(rotation, d));
	EXPECT_FALSE(matches(rotation, v));
	EXPECT_TRUE(matches(rotation, r));
	EXPECT_FALSE(matches(rotation, f));

	EXPECT_FALSE(matches(frame, d));
	EXPECT_FALSE(matches(frame, v));
	EXPECT_FALSE(matches(frame, r));
	EXPECT_TRUE(matches(frame, f));
}

TEST_F(GiskardPPTest, parseType) {
	DoubleSpecPtr d;
	VectorSpecPtr v;
	RotationSpecPtr r;
	FrameSpecPtr f;
	SpecPtr tempD = parser.parseType(GiskardPPParser::sScalar);
	SpecPtr tempV = parser.parseType(GiskardPPParser::sVec3);
	SpecPtr tempR = parser.parseType(GiskardPPParser::sRotation);
	SpecPtr tempF = parser.parseType(GiskardPPParser::sFrame);
	EXPECT_TRUE(matches(tempD, d));
	EXPECT_TRUE(matches(tempV, v));
	EXPECT_TRUE(matches(tempR, r));
	EXPECT_TRUE(matches(tempF, f));
}

TEST_F(GiskardPPTest, parseDeclaration) {
	EXPECT_ANY_THROW(parser.parseDeclaration("bla "));
	EXPECT_ANY_THROW(parser.parseDeclaration("frame"));
	EXPECT_ANY_THROW(parser.parseDeclaration("bla foo"));

	DeclPtr decl;
	ASSERT_NO_THROW(decl = parser.parseDeclaration("scalar foo"));
	DoubleSpecPtr d;
	EXPECT_TRUE(matches(decl->type, d));
	EXPECT_EQ("foo", decl->name);
}

TEST_F(GiskardPPTest, parseLiteral) {
	SpecPtr expr;
	ASSERT_NO_THROW( expr = parser.parseLiteral("10"));
	DoubleConstSpecPtr d;
	ASSERT_TRUE(matches(expr, d));
	EXPECT_EQ(10.0, d->get_value());

	ASSERT_NO_THROW( expr = parser.parseLiteral("vec3(1,2,3)"));
	VectorConstructorSpecPtr v;
	ASSERT_TRUE(matches(expr, v));
	VectorSpecPtr vec = instance<VectorConstructorSpec>(double_const_spec(1.0), double_const_spec(2.0), double_const_spec(3.0));
	EXPECT_TRUE(vec->equals(*v));
}

TEST_F(GiskardPPTest, parseFactor) {
	try {
        parser.parseNamedExpression("someScalar = 5 * 4");
        parser.parseNamedExpression("someVector = vec3(1,0,0)");
        parser.parseNamedExpression("someRotation = rotation(vec3(0,0,1), 1.57)");
        parser.parseNamedExpression("someFrame = frame(rotation(0,0,0,1), vec3(0,0,1))");
	} catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		FAIL();
	}

	SpecPtr expr; 
	ASSERT_NO_THROW(expr = parser.parseFactor("-someScalar"));
	DoubleSubtractionSpecPtr d;
	EXPECT_TRUE(matches(expr, d));

	ASSERT_NO_THROW(expr = parser.parseFactor("someScalar"));
	DoubleReferenceSpecPtr dref;
	EXPECT_TRUE(matches(expr, dref));

	ASSERT_NO_THROW(expr = parser.parseFactor("-someVector"));
	VectorDoubleMultiplicationSpecPtr v;
	EXPECT_TRUE(matches(expr, v));

    ASSERT_NO_THROW(expr = parser.parseFactor("someVector"));
	VectorReferenceSpecPtr vref;
	EXPECT_TRUE(matches(expr, vref));
}

TEST_F(GiskardPPTest, parseTerm) {
	try {
        parser.parseNamedExpression("scal1 = 3");
        parser.parseNamedExpression("scal2 = 5");
        parser.parseNamedExpression("vec1 = vec3(1,0,0)");
        parser.parseNamedExpression("vec2 = vec3(0,0,1)");
        parser.parseNamedExpression("rot1 = rotation(vec1, 0.5)");
        parser.parseNamedExpression("rot2 = rotation(vec2, 0.5)");
        parser.parseNamedExpression("frame1 = frame(rot1, vec2)");
        parser.parseNamedExpression("frame2 = frame(rot2, vec2)");
	} catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		FAIL();
	}

	EXPECT_ANY_THROW(parser.parseTerm("scal1 * "));
	EXPECT_ANY_THROW(parser.parseTerm("* scal1"));
	EXPECT_ANY_THROW(parser.parseTerm("scal1 * rot1"));
	EXPECT_ANY_THROW(parser.parseTerm("rot1 * scal1"));
	EXPECT_ANY_THROW(parser.parseTerm("scal1 * frame1"));
	EXPECT_ANY_THROW(parser.parseTerm("frame1 * scal1"));
	EXPECT_ANY_THROW(parser.parseTerm("vec1 * rot1"));
	EXPECT_ANY_THROW(parser.parseTerm("vec1 * frame1"));

	EXPECT_ANY_THROW(parser.parseTerm("scal1 / "));
	EXPECT_ANY_THROW(parser.parseTerm("/ scal1"));
	EXPECT_ANY_THROW(parser.parseTerm("scal1 / rot1"));
	EXPECT_ANY_THROW(parser.parseTerm("rot1 / scal1"));
	EXPECT_ANY_THROW(parser.parseTerm("rot1 / rot2"));
	EXPECT_ANY_THROW(parser.parseTerm("scal1 / frame1"));
	EXPECT_ANY_THROW(parser.parseTerm("frame1 / scal1"));
	EXPECT_ANY_THROW(parser.parseTerm("frame1 / frame2"));
	EXPECT_ANY_THROW(parser.parseTerm("vec1 / rot1"));
	EXPECT_ANY_THROW(parser.parseTerm("vec1 / frame1"));

	SpecPtr scalMul;
	ASSERT_NO_THROW(scalMul = parser.parseTerm("scal1 * scal2"));
	DoubleMultiplicationSpecPtr smul;
	EXPECT_TRUE(matches(scalMul, smul));

	SpecPtr scalDiv;
	ASSERT_NO_THROW(scalDiv = parser.parseTerm("scal1 / scal2"));
	DoubleDivisionSpecPtr sdiv;
	EXPECT_TRUE(matches(scalDiv, sdiv));

	SpecPtr scalVecMul1;
	ASSERT_NO_THROW(scalVecMul1 = parser.parseTerm("scal1 * vec1"));
	VectorDoubleMultiplicationSpecPtr vsmul;
	ASSERT_TRUE(matches(scalVecMul1, vsmul));

	SpecPtr scalVecMul2;
	ASSERT_NO_THROW(scalVecMul2 = parser.parseTerm("vec1 * scal1"));
	ASSERT_TRUE(matches(scalVecMul2, vsmul));
	EXPECT_TRUE(scalVecMul1->equals(*scalVecMul2));
	
	SpecPtr vecDot;
	ASSERT_NO_THROW(vecDot = parser.parseTerm("vec1 * vec2"));
	VectorDotSpecPtr vdot;
	EXPECT_TRUE(matches(vecDot, vdot));

	SpecPtr vecRot;
	ASSERT_NO_THROW(vecRot = parser.parseTerm("rot1 * vec1"));
	VectorRotationMultiplicationSpecPtr vrot;
	EXPECT_TRUE(matches(vecRot, vrot));	

	SpecPtr vecFrame;
	ASSERT_NO_THROW(vecFrame = parser.parseTerm("frame1 * vec1"));
	VectorFrameMultiplicationSpecPtr vframe;
	EXPECT_TRUE(matches(vecFrame, vframe));		

	SpecPtr rotMul;
	ASSERT_NO_THROW(rotMul = parser.parseTerm("rot1 * rot2"));
	RotationMultiplicationSpecPtr rmul;
	EXPECT_TRUE(matches(rotMul, rmul));	

	SpecPtr frameMul;
	ASSERT_NO_THROW(frameMul = parser.parseTerm("frame1 * frame2"));
	FrameMultiplicationSpecPtr fmul;
	EXPECT_TRUE(matches(frameMul, fmul));	
}

TEST_F(GiskardPPTest, parseExpression) {
	try {
        parser.parseNamedExpression("scal1 = 3");
        parser.parseNamedExpression("scal2 = 5");
        parser.parseNamedExpression("vec1 = vec3(1,0,0)");
        parser.parseNamedExpression("vec2 = vec3(0,0,1)");
        parser.parseNamedExpression("rot1 = rotation(vec1, 0.5)");
        parser.parseNamedExpression("rot2 = rotation(vec2, 0.5)");
        parser.parseNamedExpression("frame1 = frame(rot1, vec2)");
        parser.parseNamedExpression("frame2 = frame(rot2, vec2)");
	} catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		FAIL();
	}

	EXPECT_ANY_THROW(parser.parseExpression("scal1 + "));
	EXPECT_ANY_THROW(parser.parseExpression("+ scal1"));
	EXPECT_ANY_THROW(parser.parseExpression("scal1 + vec1"));
	EXPECT_ANY_THROW(parser.parseExpression("vec1 + scal1"));
	EXPECT_ANY_THROW(parser.parseExpression("scal1 + rot1"));
	EXPECT_ANY_THROW(parser.parseExpression("rot1 + scal1"));
	EXPECT_ANY_THROW(parser.parseExpression("scal1 + frame1"));
	EXPECT_ANY_THROW(parser.parseExpression("frame1 + scal1"));
	EXPECT_ANY_THROW(parser.parseExpression("vec1 + rot1"));
	EXPECT_ANY_THROW(parser.parseExpression("vec1 + frame1"));

	EXPECT_ANY_THROW(parser.parseExpression("scal1 - "));
	EXPECT_ANY_THROW(parser.parseExpression("scal1 - vec1"));
	EXPECT_ANY_THROW(parser.parseExpression("vec1 - scal1"));
	EXPECT_ANY_THROW(parser.parseExpression("scal1 - rot1"));
	EXPECT_ANY_THROW(parser.parseExpression("rot1 - scal1"));
	EXPECT_ANY_THROW(parser.parseExpression("scal1 - frame1"));
	EXPECT_ANY_THROW(parser.parseExpression("frame1 - scal1"));
	EXPECT_ANY_THROW(parser.parseExpression("vec1 - rot1"));
	EXPECT_ANY_THROW(parser.parseExpression("vec1 - frame1"));

	SpecPtr scalAdd;
	ASSERT_NO_THROW(scalAdd = parser.parseExpression("scal1 + scal2"));
	DoubleAdditionSpecPtr sadd;
	EXPECT_TRUE(matches(scalAdd, sadd));

	SpecPtr scalSub; 
	ASSERT_NO_THROW(scalSub = parser.parseExpression("scal1 - scal2"));
	DoubleSubtractionSpecPtr ssub;
	EXPECT_TRUE(matches(scalSub, ssub));

	SpecPtr vecAdd; 
	ASSERT_NO_THROW(vecAdd = parser.parseExpression("vec1 + vec2"));
	VectorAdditionSpecPtr vadd;
	EXPECT_TRUE(matches(vecAdd, vadd));

	SpecPtr vecSub; 
	ASSERT_NO_THROW(vecSub = parser.parseExpression("vec1 - vec2"));
	VectorSubtractionSpecPtr vsub;
	EXPECT_TRUE(matches(vecSub, vsub));

	giskard_core::Scope scope;
	SpecPtr execOrder1, execOrder2, execOrder3, execOrder4;
	ASSERT_NO_THROW(execOrder1 = parser.parseExpression("4 + 2 * 3"));
	ASSERT_NO_THROW(execOrder2 = parser.parseExpression("4 * 2 + 3"));
	ASSERT_NO_THROW(execOrder3 = parser.parseExpression("(4 + 2) * 3"));
	ASSERT_NO_THROW(execOrder4 = parser.parseExpression("4 * (2 + 3)"));

	DoubleSpecPtr eo1, eo2, eo3, eo4;
	ASSERT_TRUE(matches(execOrder1, eo1));
	ASSERT_TRUE(matches(execOrder2, eo2));
	ASSERT_TRUE(matches(execOrder3, eo3));
	ASSERT_TRUE(matches(execOrder4, eo4));

	KDL::Expression<double>::Ptr expr1 = eo1->get_expression(scope);
	KDL::Expression<double>::Ptr expr2 = eo2->get_expression(scope);
	KDL::Expression<double>::Ptr expr3 = eo3->get_expression(scope);
	KDL::Expression<double>::Ptr expr4 = eo4->get_expression(scope);

	EXPECT_EQ(10.0, expr1->value());
	EXPECT_EQ(11.0, expr2->value());
	EXPECT_EQ(18.0, expr3->value());
	EXPECT_EQ(20.0, expr4->value());
}

TEST_F(GiskardPPTest, parseNamedExpression) {
	EXPECT_ANY_THROW(parser.parseNamedExpression("stuff"));
	EXPECT_ANY_THROW(parser.parseNamedExpression("stuff = "));

	SpecPtr expr; 
	ASSERT_NO_THROW(expr = parser.parseExpression("4 * 6 + 9"));
	ScopeEntry s;
	ASSERT_NO_THROW(s = parser.parseNamedExpression("stuff = 4 * 6 + 9"));
	EXPECT_EQ("stuff", s.name);
	EXPECT_TRUE(s.spec->equals(*expr));	
}

TEST_F(GiskardPPTest, parseImportStatement) {
	EXPECT_ANY_THROW(parser.parseImportStatement("bla"));
	EXPECT_ANY_THROW(parser.parseImportStatement("import "));
	EXPECT_ANY_THROW(parser.parseImportStatement("import \"stuff.gpp\" as"));
	EXPECT_ANY_THROW(parser.parseImportStatement("import \"stuff.gpp\" as \"lol\""));

	ImportPtr import1;
	ASSERT_NO_THROW(import1 = parser.parseImportStatement("import \"stuff.gpp\""));
	EXPECT_EQ("stuff.gpp", import1->path);
	EXPECT_TRUE(import1->alias.empty());

	ImportPtr import2;
	ASSERT_NO_THROW(import2 = parser.parseImportStatement("import \"stuff.gpp\" as lol"));
	EXPECT_EQ("stuff.gpp", import2->path);
	EXPECT_EQ("lol", import2->alias);
}

TEST_F(GiskardPPTest, parseMemberAccess) {
	try {
        parser.parseNamedExpression("vec1 = vec3(1,0,0)");
        parser.parseNamedExpression("rot1 = rotation(vec1, 0.5)");
        parser.parseNamedExpression("frame1 = frame(rot1, vec1)");
	} catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		FAIL();
	}

	SpecPtr xOf, yOf, zOf, posOf, rotOf;
	ASSERT_NO_THROW(xOf = parser.parseMemberAccess("vec1.x"));
	ASSERT_NO_THROW(yOf = parser.parseMemberAccess("vec1.y"));
	ASSERT_NO_THROW(zOf = parser.parseMemberAccess("vec1.z"));
	ASSERT_NO_THROW(posOf = parser.parseMemberAccess("frame1.pos"));
	ASSERT_NO_THROW(rotOf = parser.parseMemberAccess("frame1.rot"));

	DoubleXCoordOfSpecPtr xptr;
	DoubleYCoordOfSpecPtr yptr;
	DoubleZCoordOfSpecPtr zptr;
	VectorOriginOfSpecPtr pptr;
	OrientationOfSpecPtr rptr;

	EXPECT_TRUE(matches(xOf, xptr));
	EXPECT_TRUE(matches(yOf, yptr));
	EXPECT_TRUE(matches(zOf, zptr));

	EXPECT_TRUE(matches(posOf, pptr));
	EXPECT_TRUE(matches(rotOf, rptr));	
}

TEST_F(GiskardPPTest, parseFunctionDefinition) {
	EXPECT_ANY_THROW(parser.parseFunctionDefinition("def"));
	EXPECT_ANY_THROW(parser.parseFunctionDefinition("def f(scalar x) { return x; }"));
	FnDefPtr f1, f2;
	EXPECT_NO_THROW(f1 = parser.parseFunctionDefinition("def scalar f(scalar x) { return x; }"));
	EXPECT_NO_THROW(f2 = parser.parseFunctionDefinition("def scalar f(scalar x, vec3 v, frame f) { return x; }"));

	EXPECT_TRUE(!!f1->getLocalSpec("x"));
	EXPECT_TRUE(!!f2->getLocalSpec("x"));
	EXPECT_TRUE(!!f2->getLocalSpec("v"));
	EXPECT_TRUE(!!f2->getLocalSpec("f"));
}

TEST_F(GiskardPPTest, functionCall) {
	FnDefPtr function;
	try {
        parser.parseNamedExpression("superA = 5");
		function = parser.parseFunctionDefinition("def scalar f(scalar x) { sqA = superA * superA; rVal = sqA * x; return rVal; }");
	} catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		FAIL();
	}

	SpecPtr fnCall;
	EXPECT_NO_THROW(fnCall = parser.parseExpression("f(2)"));
	DoubleReferenceSpecPtr retRef;
    ASSERT_TRUE(matches(fnCall, retRef));
	EXPECT_EQ("f(2.000000)", retRef->get_reference_name());
	AdvancedScopePtr scope = parser.getTopScope();
	EXPECT_TRUE(!!scope->getLocalSpec("f(scalar)::sqA"));
	EXPECT_TRUE(!!scope->getLocalSpec("f(2.000000)::x"));
	EXPECT_TRUE(!!scope->getLocalSpec("f(2.000000)::rVal"));
	EXPECT_TRUE(!!scope->getLocalSpec("f(2.000000)"));
}

TEST_F(GiskardPPTest, StringSpecTest) {
	EXPECT_NO_THROW(parser.parseNamedExpression("str1 = \"abc\""));
	SpecPtr expr;
	StringSpecPtr str;
	EXPECT_NO_THROW(expr = parser.parseExpression("str1 + \"123\""));
	ASSERT_TRUE(matches(expr, str));
	EXPECT_EQ("abc123", str->get_value());
}

TEST_F(GiskardPPTest, LoadFile) {
	try {
		QPControllerSpec qpSpec = parser.parseFromFile("test_controller.gpp");
		QPController controller = generate(qpSpec); 
	} catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		FAIL();
	}
}