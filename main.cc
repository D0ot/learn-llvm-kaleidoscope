#include <cctype>
#include <cstdio>
#include <iostream>
#include <string>

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


int main(int argc, char **argv) {
  std::cout << "Hello World" << std::endl;
}
