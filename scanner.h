#ifndef __SCANNER_H_
#define __SCANNER_H_

#include <map>
#include <string>

enum Token {
  tok_eof = -1,

  // commands
  tok_def = -2,
  tok_extern = -3,

  // primary
  tok_identifier = -4,
  tok_number = -5,

  // control
  tok_if = -6,
  tok_then = -7,
  tok_else = -8,

  tok_for = -9,
  tok_in = -10,
};


extern std::string IdentifierStr;
extern double NumVal;
extern int CurTok;
extern std::map<char, int> BinopPrecedence;


int getNextToken();
int GetTokPrecedence();

#endif
