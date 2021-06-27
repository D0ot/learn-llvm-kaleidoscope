#include <cctype>
#include <cstdio>
#include <iostream>
#include <string>
#include <map>
#include "ast.h"

enum Token {
  tok_eof = -1,

  // commands
  tok_def = -2,
  tok_extern = -3,

  // primary
  tok_identifier = -4,
  tok_number = -5,
};

static std::string IdentifierStr;
static double NumVal;

static int gettok() {
  static int LastChar = ' ';

  // Skip any whitespace.
  while(isspace(LastChar)) {
    LastChar = getchar();
  }

  if(std::isalpha(LastChar)) {
    IdentifierStr = LastChar;
    while(std::isalnum((LastChar = getchar()))) {
      IdentifierStr += LastChar;
    }

    if(IdentifierStr == "def") {
      return tok_def;
    }

    if(IdentifierStr == "extern") {
      return tok_extern;
    }
    return tok_identifier;
  }

  if(std::isdigit(LastChar) || LastChar == '.') {
    std::string NumStr;
    do {
      NumStr += LastChar;
      LastChar = getchar();
    } while(isdigit(LastChar) || LastChar == '.');
    NumVal = strtod(NumStr.c_str(), 0);
    return tok_number;
  }

  if(LastChar == '#') {
    do {
      LastChar = getchar();
    }while(LastChar != EOF && LastChar != '\n' && LastChar != '\r');

    if(LastChar != EOF) {
      return gettok();
    }
  }

  if(LastChar == EOF) {
    return tok_eof;
  }

  int ThisChar = LastChar;
  LastChar = getchar();
  return ThisChar;

}


static int CurTok;
static int getNextToken() {
  return CurTok = gettok();
}

static std::map<char, int> BinopPrecedence;

static int GetTokPrecedence() {
  if(!isascii(CurTok)) {
    return -1;
  }

  int TokPrec = BinopPrecedence[CurTok];
  if(TokPrec <= 0) {
    return -1;
  }
  return TokPrec;
}

std::unique_ptr<ExprAST> LogError(const char *Str) {
  fprintf(stderr, "Error: %s\n", Str);
  return nullptr;
}

std::unique_ptr<PrototypeAST> LogErrorP(const char *Str) {
  LogError(Str);
  return nullptr;
}

static std::unique_ptr<ExprAST> ParseExpression();

/// numberexpr ::= number
static std::unique_ptr<ExprAST> ParseNumberExpr() {
  auto Result = std::make_unique<NumberExprAST>(NumVal);
  getNextToken();
  return std::move(Result);
}

/// parenexpr ::= '(' expression ')'
static std::unique_ptr<ExprAST> ParseParenExpr() {
  getNextToken();
  auto V = ParseExpression();
  if(!V) {
    return nullptr;
  }

  if(CurTok != ')') {
    return LogError("expected ')' ");
  }

  getNextToken(); // eat ')'
  return V;
}

/// identifierexpr
///    ::= identifier
///    ::= identifier '(' expression* ')'

static std::unique_ptr<ExprAST> ParseIdentifierExpr() {
  std::string IdName = IdentifierStr;
  getNextToken();

  if(CurTok != ')') {
    return std::make_unique<VariableExprAST>(IdName);
  }

  // Call
  getNextToken(); // eat (

  std::vector<std::unique_ptr<ExprAST>> Args;
  if(CurTok != ')') {
    while(true) {
      if(auto Arg = ParseExpression()) {
        Args.push_back(std::move(Arg));
      } else {
        return nullptr;
      }

      if(CurTok == ')') {
        break;
      }

      if(CurTok != ',') {
        return LogError("Expected ')' or ',' in argument list");
      }

      getNextToken();
    }
  }

  // Eat the ')'
  getNextToken();
  return std::make_unique<CallExprAST>(IdName, std::move(Args));
}

/// primary
///   ::= identifierexpr
///   ::= numberexpr
///   ::= parenexpr
static std::unique_ptr<ExprAST> ParsePrimary() {
  switch(CurTok) {
    default:
      return LogError("unknown token when expecting an expession");
    case tok_identifier:
      return ParseIdentifierExpr();
    case tok_number:
      return ParseNumberExpr();
    case '(':
      return ParseParenExpr();
  }
}


/// binoprhs
///   ::= ('+' primary)*
static std::unique_ptr<ExprAST> ParseBinOpRHS(int ExprPrec, std::unique_ptr<ExprAST> LHS) {
  while(true) {
    int TokPrec = GetTokPrecedence();
    if(TokPrec < ExprPrec) {
      return LHS;
    }

    int BinOp = CurTok;
    getNextToken();

    auto RHS = ParsePrimary();
    if(!RHS) {
      return nullptr;
    }

    int NextPrec = GetTokPrecedence();
    if(TokPrec < NextPrec) {
      RHS = ParseBinOpRHS(TokPrec + 1, std::move(RHS));
      if(!RHS) {
        return nullptr;
      }
    }
    LHS = std::make_unique<BinaryExprAST>(BinOp, std::move(LHS), std::move(RHS));
  }
}

/// expression
///   ::= primary binoprhs

static std::unique_ptr<ExprAST> ParseExpression() {
  auto LHS = ParsePrimary();
  if(!LHS) {
    return nullptr;
  }
  return ParseBinOpRHS(0, std::move(LHS));
}

/// prototype
///   ::= id '(' id* ')'
static std::unique_ptr<PrototypeAST> ParsePrototype() {
  if(CurTok != tok_identifier) {
    return LogErrorP("Expected function name in prototype");
  }

  std::string FnName = IdentifierStr;
  getNextToken();

}

int main(int argc, char **argv) {
  std::cout << "Hello World" << std::endl;
}
