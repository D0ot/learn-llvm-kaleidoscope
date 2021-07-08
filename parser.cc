#include <cstdio>
#include <algorithm>
#include <memory>

#include "parser.h"
#include "scanner.h"

#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/GVN.h"

using namespace llvm;
using namespace llvm::orc;

std::unique_ptr<ExprAST> LogError(const char *Str) {
  fprintf(stderr, "Error: %s\n", Str);
  return nullptr;
}

std::unique_ptr<PrototypeAST> LogErrorP(const char *Str) {
  LogError(Str);
  return nullptr;
}

std::unique_ptr<ExprAST> ParseExpression();

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

  if(CurTok != '(') {
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

/// ifexpr ::= 'if' expression 'then' expression 'else' expression
static std::unique_ptr<ExprAST> ParseIfExpr() {
  getNextToken();  // eat the if.

  // condition.
  auto Cond = ParseExpression();
  if (!Cond)
    return nullptr;

  if (CurTok != tok_then)
    return LogError("expected then");
  getNextToken();  // eat the then

  auto Then = ParseExpression();
  if (!Then)
    return nullptr;

  if (CurTok != tok_else)
    return LogError("expected else");

  getNextToken();

  auto Else = ParseExpression();
  if (!Else)
    return nullptr;

  return std::make_unique<IfExprAST>(std::move(Cond), std::move(Then),
                                      std::move(Else));
}

/// forexpr ::= 'for' identifier '=' expr ',' expr (',' expr)? 'in' expression
/// example: 
//      for i = 1, i < n, 1.0 in expr
static std::unique_ptr<ExprAST> ParseForExpr() {
  getNextToken();  // eat the for.

  if (CurTok != tok_identifier)
    return LogError("expected identifier after for");

  std::string IdName = IdentifierStr;
  getNextToken();  // eat identifier.

  if (CurTok != '=')
    return LogError("expected '=' after for");
  getNextToken();  // eat '='.


  auto Start = ParseExpression();
  if (!Start)
    return nullptr;
  if (CurTok != ',')
    return LogError("expected ',' after for start value");
  getNextToken();

  auto End = ParseExpression();
  if (!End)
    return nullptr;

  // The step value is optional.
  std::unique_ptr<ExprAST> Step;
  if (CurTok == ',') {
    getNextToken();
    Step = ParseExpression();
    if (!Step)
      return nullptr;
  }

  if (CurTok != tok_in)
    return LogError("expected 'in' after for");
  getNextToken();  // eat 'in'.

  auto Body = ParseExpression();
  if (!Body)
    return nullptr;

  return std::make_unique<ForExprAST>(IdName, std::move(Start),
                                       std::move(End), std::move(Step),
                                       std::move(Body));
}

/// primary
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
    case tok_if:
      return ParseIfExpr();
    case tok_for:
      return ParseForExpr();
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

std::unique_ptr<ExprAST> ParseExpression() {
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

  if(CurTok != '(') {
    return LogErrorP("Expected '(' in prototype");
  }

  std::vector<std::string> ArgNames;
  while(getNextToken() == tok_identifier) {
    ArgNames.push_back(IdentifierStr);
  }

  if(CurTok != ')') {
    return LogErrorP("Expected ')' in prototype");
  }

  getNextToken(); //eat ')'

  return std::make_unique<PrototypeAST>(FnName, std::move(ArgNames));
}

std::unique_ptr<FunctionAST> ParseDefinition() {
  getNextToken();

  auto Proto = ParsePrototype();

  if(!Proto) return nullptr;

  if(auto E = ParseExpression()) {
    return std::make_unique<FunctionAST>(std::move(Proto), std::move(E));
  }

  return nullptr;
}

std::unique_ptr<PrototypeAST> ParseExtern() {
  getNextToken();
  return ParsePrototype();
}

std::unique_ptr<FunctionAST> ParseTopLevelExpr() {
  if(auto E = ParseExpression()) {
    auto Proto = std::make_unique<PrototypeAST>("", std::vector<std::string>());
    return std::make_unique<FunctionAST>(std::move(Proto), std::move(E));
  }
  return nullptr;
}

/// Code Gen
//
std::unique_ptr<LLVMContext> TheContext;
std::unique_ptr<Module> TheModule;
std::unique_ptr<IRBuilder<>> Builder;
std::map<std::string, Value *> NamedValues;
std::unique_ptr<legacy::FunctionPassManager> TheFPM;

Value *LogErrorV(const char *Str) {
  LogError(Str);
  return nullptr;
}

Value *NumberExprAST::codegen() {
  return ConstantFP::get(*TheContext, APFloat(Val));
}

Value *VariableExprAST::codegen() {
  Value *V = NamedValues[Name];
  if(!V) {
    return LogErrorV("Unknow variable name");
  }
  return V;
}

Value *BinaryExprAST::codegen() {
  Value *L = LHS->codegen();
  Value *R = RHS->codegen();

  if(!L || !R) {
    return nullptr;
  }

  switch(Op) {
    case '+':
      return Builder->CreateFAdd(L, R, "addtmp");
    case '-':
      return Builder->CreateFSub(L, R, "subtmp");
    case '*':
      return Builder->CreateFMul(L, R, "multmp");
    case '<':
      L = Builder->CreateFCmpULT(L, R, "cmptmp");
      return Builder->CreateUIToFP(L, Type::getDoubleTy(*TheContext), "booltmp");
    default:
      return LogErrorV("invalid binary operator");
  }
}

Value *CallExprAST::codegen() {
  Function *CalleeF = TheModule->getFunction(Callee);
  if(!CalleeF) {
    return LogErrorV("Unknow function referenced");
  }

  if(CalleeF -> arg_size() != Args.size()) {
    return LogErrorV("Incorrect # arguments passed");
  }

  std::vector<Value *> ArgsV;
  for(unsigned i = 0, e = Args.size(); i != e; ++i) {
    ArgsV.push_back(Args[i] -> codegen());
    if(!ArgsV.back()) {
      return nullptr;
    }
  }
  return Builder->CreateCall(CalleeF, ArgsV, "calltmp");
}

Function *PrototypeAST::codegen() {
  std::vector<Type *> Doubles(Args.size(), Type::getDoubleTy(*TheContext));

  FunctionType *FT = FunctionType::get(Type::getDoubleTy(*TheContext), Doubles, false);
  
  Function *F = Function::Create(FT, Function::ExternalLinkage, Name, TheModule.get());

  // Set names for all arguments

  unsigned Idx = 0;
  for(auto &Arg : F->args()) {
    Arg.setName(Args[Idx++]);
  }

  return F;
}

Function *FunctionAST::codegen() {
  Function *TheFunction = TheModule->getFunction(Proto->getName());

  if(!TheFunction) {
    TheFunction = Proto->codegen();
  }

  if(!TheFunction) {
    return nullptr;
  }

  // create a new basic block to insert into
  BasicBlock *BB = BasicBlock::Create(*TheContext, "entry", TheFunction);
  Builder->SetInsertPoint(BB);

  NamedValues.clear();
  
  for(auto &Arg : TheFunction->args()) {
    NamedValues[std::string(Arg.getName())] = &Arg;
  }

  if(Value *RetVal = Body->codegen()) {
    Builder->CreateRet(RetVal);
    verifyFunction(*TheFunction);
    TheFPM->run(*TheFunction);
    return TheFunction;
  }
  
  TheFunction->eraseFromParent();
  return nullptr;
}

Value *IfExprAST::codegen() {
  Value *CondV = Cond->codegen();
  if (!CondV)
    return nullptr;

  // Convert condition to a bool by comparing non-equal to 0.0.
  CondV = Builder->CreateFCmpONE(
      CondV, ConstantFP::get(*TheContext, APFloat(0.0)), "ifcond");

  Function *TheFunction = Builder->GetInsertBlock()->getParent();

  // Create blocks for the then and else cases.  Insert the 'then' block at the
  // end of the function.
  BasicBlock *ThenBB = BasicBlock::Create(*TheContext, "then", TheFunction);
  BasicBlock *ElseBB = BasicBlock::Create(*TheContext, "else");
  BasicBlock *MergeBB = BasicBlock::Create(*TheContext, "ifcont");

  Builder->CreateCondBr(CondV, ThenBB, ElseBB);

  // Emit then value.
  Builder->SetInsertPoint(ThenBB);

  Value *ThenV = Then->codegen();
  if (!ThenV)
    return nullptr;

  Builder->CreateBr(MergeBB);
  // Codegen of 'Then' can change the current block, update ThenBB for the PHI.
  ThenBB = Builder->GetInsertBlock();

  // Emit else block.
  TheFunction->getBasicBlockList().push_back(ElseBB);
  Builder->SetInsertPoint(ElseBB);

  Value *ElseV = Else->codegen();
  if (!ElseV)
    return nullptr;

  Builder->CreateBr(MergeBB);
  // Codegen of 'Else' can change the current block, update ElseBB for the PHI.
  ElseBB = Builder->GetInsertBlock();

  // Emit merge block.
  TheFunction->getBasicBlockList().push_back(MergeBB);
  Builder->SetInsertPoint(MergeBB);
  PHINode *PN = Builder->CreatePHI(Type::getDoubleTy(*TheContext), 2, "iftmp");

  PN->addIncoming(ThenV, ThenBB);
  PN->addIncoming(ElseV, ElseBB);
  return PN;
}

Value *ForExprAST::codegen() {
  // Emit the start code first, without 'variable' in scope.
  Value *StartVal = Start->codegen();
  if (!StartVal)
    return nullptr;

  // Make the new basic block for the loop header, inserting after current
  // block.
  Function *TheFunction = Builder->GetInsertBlock()->getParent();
  BasicBlock *PreheaderBB = Builder->GetInsertBlock();
  BasicBlock *LoopBB = BasicBlock::Create(*TheContext, "loop", TheFunction);

  // Insert an explicit fall through from the current block to the LoopBB.
  Builder->CreateBr(LoopBB);

  // Start insertion in LoopBB.
  Builder->SetInsertPoint(LoopBB);

  // Start the PHI node with an entry for Start.
  PHINode *Variable =
      Builder->CreatePHI(Type::getDoubleTy(*TheContext), 2, VarName);
  Variable->addIncoming(StartVal, PreheaderBB);

  // Within the loop, the variable is defined equal to the PHI node.  If it
  // shadows an existing variable, we have to restore it, so save it now.
  Value *OldVal = NamedValues[VarName];
  NamedValues[VarName] = Variable;

  // Emit the body of the loop.  This, like any other expr, can change the
  // current BB.  Note that we ignore the value computed by the body, but don't
  // allow an error.
  if (!Body->codegen())
    return nullptr;

  // Emit the step value.
  Value *StepVal = nullptr;
  if (Step) {
    StepVal = Step->codegen();
    if (!StepVal)
      return nullptr;
  } else {
    // If not specified, use 1.0.
    StepVal = ConstantFP::get(*TheContext, APFloat(1.0));
  }

  Value *NextVar = Builder->CreateFAdd(Variable, StepVal, "nextvar");

  // Compute the end condition.
  Value *EndCond = End->codegen();
  if (!EndCond)
    return nullptr;

  // Convert condition to a bool by comparing non-equal to 0.0.
  EndCond = Builder->CreateFCmpONE(
      EndCond, ConstantFP::get(*TheContext, APFloat(0.0)), "loopcond");

  // Create the "after loop" block and insert it.
  BasicBlock *LoopEndBB = Builder->GetInsertBlock();
  BasicBlock *AfterBB =
      BasicBlock::Create(*TheContext, "afterloop", TheFunction);

  // Insert the conditional branch into the end of LoopEndBB.
  Builder->CreateCondBr(EndCond, LoopBB, AfterBB);

  // Any new code will be inserted in AfterBB.
  Builder->SetInsertPoint(AfterBB);

  // Add a new entry to the PHI node for the backedge.
  Variable->addIncoming(NextVar, LoopEndBB);

  // Restore the unshadowed variable.
  if (OldVal)
    NamedValues[VarName] = OldVal;
  else
    NamedValues.erase(VarName);

  // for expr always returns 0.0.
  return Constant::getNullValue(Type::getDoubleTy(*TheContext));
}

void InitializeModuleAndPassManager() {
  TheContext = std::make_unique<LLVMContext>();
  TheModule = std::make_unique<Module>("my jit", *TheContext);
  Builder = std::make_unique<IRBuilder<>>(*TheContext);

  TheFPM = std::make_unique<legacy::FunctionPassManager>(TheModule.get());

  TheFPM->add(createInstructionCombiningPass());
  TheFPM->add(createReassociatePass());
  TheFPM->add(createGVNPass());
  TheFPM->add(createCFGSimplificationPass());
  TheFPM->doInitialization();
  
}
