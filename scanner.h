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
};


extern std::string IdentifierStr;
extern double NumVal;
extern int CurTok;
extern std::map<char, int> BinopPrecedence;


int getNextToken();
int GetTokPrecedence();

#endif
