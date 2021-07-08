#include <cctype>
#include <cstdio>
#include <iostream>
#include <string>
#include <memory>
#include <map>

#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h>

#include "parser.h"
#include "scanner.h"



static void HandleDefinition() {
  if(auto FnAST = ParseDefinition()) {
    if(auto *FnIR = FnAST->codegen()) {
      fprintf(stderr, "Read function definition:\n");
      FnIR->print(errs());
      fprintf(stderr, "\n");
    }
  } else {
    getNextToken();
  }
}

static void HandleExtern() {
  if(auto ProtoAST = ParseExtern()) {
    if(auto *ProtoIR = ProtoAST ->codegen()) {
      fprintf(stderr, "Read Extern:\n");
      ProtoIR ->print(errs());
      fprintf(stderr, "\n");
    }
  } else {
    getNextToken();
  }
}

static void HandleTopLevelExpression() {
  if(auto FnAST = ParseTopLevelExpr()) {
    if(auto *FnIR= FnAST ->codegen()) {
      fprintf(stderr, "Read top-level expression:\n");
      FnIR ->print(errs());
      fprintf(stderr, "\n");
    }
  } else {
    getNextToken();
  }
}


static void MainLoop() {
  while(1) {
    fprintf(stderr, "ready> ");
    switch(CurTok) {
      case tok_eof:
        return;
      case ';':
        getNextToken();
        break;
      case tok_def:
        HandleDefinition();
        break;
      case tok_extern:
        HandleExtern();
        break;
      default:
        HandleTopLevelExpression();
        break;
    }
  }
}

int main(int argc, char **argv) {
  BinopPrecedence['<'] = 10;
  BinopPrecedence['+'] = 20;
  BinopPrecedence['-'] = 20;
  BinopPrecedence['*'] = 40;

  fprintf(stderr, "ready> ");
  getNextToken();

  InitializeModuleAndPassManager();
  MainLoop();

  return 0;
}
