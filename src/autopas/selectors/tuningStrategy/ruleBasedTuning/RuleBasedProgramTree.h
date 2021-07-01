#pragma once

#include "RuleVM.h"

/**
 * define_list VerletListsContainer = "VerletLists", "VerletClusterLists", "VerletListsCells";
  define LC_VL_Threshold = 2^20;

  if numParticles > LC_VL_Threshold and numCells < numParticles:
       {container="LinkedCells", dataLayout="SoA"} >= {container=VerletListsContainer};
  endif
 */

namespace autopas {

struct ConfigurationPattern {
  std::set<ContainerOption> _containers;
  std::set<TraversalOption> _traversals;
  std::set<LoadEstimatorOption> _loadEstimators;
  std::set<DataLayoutOption> _dataLayouts;
  std::set<Newton3Option> _newton3Options;
  std::set<double> _cellSizeFactors;

  [[nodiscard]] bool matches(const Configuration &configuration) const {
    return (_containers.empty() or contains(_containers, configuration.container)) and
           (_traversals.empty() or contains(_traversals, configuration.traversal)) and
           (_loadEstimators.empty() or contains(_loadEstimators, configuration.loadEstimator)) and
           (_dataLayouts.empty() or contains(_dataLayouts, configuration.dataLayout)) and
           (_newton3Options.empty() or contains(_newton3Options, configuration.newton3)) and
           (_cellSizeFactors.empty() or contains(_cellSizeFactors, configuration.cellSizeFactor));
  }

 private:
  template <typename T>
  [[nodiscard]] bool contains(const std::set<T> &set, const T &option) const {
    return set.find(option) != set.end();
  }
};

namespace rule_syntax {

class CodeGenerationContext;

struct Statement {
  virtual void generateCode(CodeGenerationContext &context, RuleVM::Program &program) const = 0;
};

enum class Type { BOOL, DOUBLE, SIZE_T, CONTAINER, TRAVERSAL, LOAD_ESTIMATOR, DATA_LAYOUT, NEWTON3, CELL_SIZE_FACTOR };

inline Type typeOf(const RuleVM::MemoryCell &memoryCell) { return static_cast<Type>(memoryCell.index()); }

struct Expression {
  [[nodiscard]] virtual Type getType() const = 0;
  virtual void generateCode(CodeGenerationContext &context, RuleVM::Program &program) const = 0;
};

struct DefineList;

class CodeGenerationContext {
 public:
  explicit CodeGenerationContext(std::map<std::string, size_t> initialAddressEnvironment)
      : currentNeededStack(0), maxNeededStack(0), addressEnvironment(std::move(initialAddressEnvironment)) {}

  void allocateStack(size_t num) {
    currentNeededStack += num;
    maxNeededStack = std::max(maxNeededStack, currentNeededStack);
  }

  void freeStack(size_t num) { currentNeededStack -= num; }

  void addVariable(const std::string &name) { addressEnvironment.insert({name, addressEnvironment.size()}); }

  [[nodiscard]] size_t addressOf(const std::string &name) const { return addressEnvironment.at(name); }

  [[nodiscard]] auto getMaxStackSize() const { return maxNeededStack; }

  void addList(const std::string &name, const DefineList *list) { lists.insert({name, list}); }

  auto getList(const std::string &name) { return lists.at(name); }

  auto addConfigurationPattern(const ConfigurationPattern &configurationPattern) {
    configurationPatterns.push_back(configurationPattern);
    return configurationPatterns.size() - 1;
  }

  [[nodiscard]] auto getConfigurationPatterns() const { return configurationPatterns; }

 private:
  size_t currentNeededStack;
  size_t maxNeededStack;

  std::map<std::string, size_t> addressEnvironment;
  std::map<std::string, const DefineList *> lists;
  std::vector<ConfigurationPattern> configurationPatterns;
};

struct Literal : public Expression {
  RuleVM::MemoryCell value;
  Type type;

  Literal() = default;

  explicit Literal(RuleVM::MemoryCell value) : value(value), type(typeOf(value)) {}

  [[nodiscard]] Type getType() const override { return type; }

  void generateCode(CodeGenerationContext &context, RuleVM::Program &program) const override {
    program.instructions.push_back({RuleVM::LOADC, value});
    context.allocateStack(1);
  }
};

struct DefineList : public Statement {
  std::string listName;
  std::vector<std::shared_ptr<Literal>> values;

  DefineList(std::string listName, std::vector<std::shared_ptr<Literal>> values)
      : listName(std::move(listName)), values(std::move(values)) {}

  void generateCode(CodeGenerationContext &context, RuleVM::Program &program) const override {
    context.addList(listName, this);
  }
};

struct ConfigurationOrder : public Statement {
  ConfigurationPattern greater;
  ConfigurationPattern smaller;

  ConfigurationOrder(ConfigurationPattern greater, ConfigurationPattern smaller)
      : greater(std::move(greater)), smaller(std::move(smaller)) {}

  void generateCode(CodeGenerationContext &context, RuleVM::Program &program) const override {
    auto idx = context.addConfigurationPattern(smaller);
    program.instructions.push_back({RuleVM::OUTPUTC, idx});
  }
};

struct Define : public Statement {
  std::string variable;
  std::shared_ptr<Expression> value;

  Define(std::string variable, std::shared_ptr<Expression> value)
      : variable(std::move(variable)), value(std::move(value)) {}

  void generateCode(CodeGenerationContext &context, RuleVM::Program &program) const override {
    value->generateCode(context, program);

    context.addVariable(variable);
    program.instructions.push_back({RuleVM::STOREA, context.addressOf(variable)});
    context.freeStack(1);
  }
};

struct Variable : public Expression {
  Define *definition;

  explicit Variable(Define *definition) : definition(definition) {}

  [[nodiscard]] Type getType() const override { return definition->value->getType(); }

  void generateCode(CodeGenerationContext &context, RuleVM::Program &program) const override {
    program.instructions.push_back({RuleVM::LOADA, context.addressOf(definition->variable)});
    context.allocateStack(1);
  }
};

struct BinaryOperator : public Expression {
  enum Operator { LESS, GREATER, AND };

  Operator op;
  std::shared_ptr<Expression> left;
  std::shared_ptr<Expression> right;

  BinaryOperator(Operator op, std::shared_ptr<Expression> left, std::shared_ptr<Expression> right)
      : op(op), left(std::move(left)), right(std::move(right)) {}

  [[nodiscard]] Type getType() const override { return Type::BOOL; }

  void generateCode(CodeGenerationContext &context, RuleVM::Program &program) const override {
    left->generateCode(context, program);
    right->generateCode(context, program);

    auto opCode = op == LESS ? RuleVM::LESS : (op == GREATER ? RuleVM::GREATER : RuleVM::AND);
    program.instructions.push_back({opCode});

    context.freeStack(1);
  }
};

struct If : public Statement {
  std::shared_ptr<Expression> condition;
  std::vector<std::shared_ptr<Statement>> consequences;

  If(std::shared_ptr<Expression> condition, std::vector<std::shared_ptr<Statement>> consequences)
      : condition(std::move(condition)), consequences(std::move(consequences)) {}

  void generateCode(CodeGenerationContext &context, RuleVM::Program &program) const override {
    condition->generateCode(context, program);
    program.instructions.push_back({RuleVM::JUMPZERO});
    context.freeStack(1);
    auto &jumpInstr = program.instructions.back();

    for (const auto &statement : consequences) {
      statement->generateCode(context, program);
    }

    jumpInstr.payload = program.instructions.size();
  }
};

struct RuleBasedProgramTree {
  std::vector<std::shared_ptr<Statement>> statements;

  [[nodiscard]] RuleVM::Program generateCode(CodeGenerationContext context) const {
    RuleVM::Program program;
    for (const auto &statement : statements) {
      statement->generateCode(context, program);
    }
    program.neededStackSize = context.getMaxStackSize();
    program.instructions.push_back({RuleVM::HALT});
    return program;
  }
};

} // namespace syntax
} // namespace autopas
