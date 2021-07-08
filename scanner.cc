#include <cctype>
#include <cstdio>
#include <iostream>
#include "scanner.h"
#include <map>


std::string IdentifierStr;
double NumVal;

int gettok() {
  static int LastChar = ' ';

  // Skip any whitespace.
  while(isspace(LastChar)) {
    LastChar = getchar();
  }

  if(isalpha(LastChar)) {
    IdentifierStr = LastChar;
    while(isalnum((LastChar = getchar()))) {
      IdentifierStr += LastChar;
    }

    if(IdentifierStr == "def") {
      return tok_def;
    }

    if(IdentifierStr == "extern") {
      return tok_extern;
    }

    if(IdentifierStr == "if") {
      return tok_if;
    }

    if(IdentifierStr == "then") {
      return tok_then;
    }

    if(IdentifierStr == "else") {
      return tok_else;
    }

    return tok_identifier;
  }

  if(isdigit(LastChar) || LastChar == '.') {
    std::string NumStr;
    do {
      NumStr += LastChar;
      LastChar = getchar();
    } while(isdigit(LastChar) || LastChar == '.');
    NumVal = strtod(NumStr.c_str(), nullptr);
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


int CurTok;
int getNextToken() {
  return CurTok = gettok();
}

std::map<char, int> BinopPrecedence;

int GetTokPrecedence() {
  if(!isascii(CurTok)) {
    return -1;
  }

  int TokPrec = BinopPrecedence[CurTok];
  if(TokPrec <= 0) {
    return -1;
  }
  return TokPrec;
}

