#include "monkey_brain_core/parse_expression.hpp"

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <boost/spirit/include/qi.hpp>
#include <boost/phoenix/core.hpp>
#include <boost/phoenix/operator.hpp>
#include <boost/phoenix/fusion.hpp>
#include <boost/phoenix/stl.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/phoenix/function/adapt_function.hpp>

#include <boost/lexical_cast.hpp>

#include <set>
#include <string>
#include <vector>

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;

BOOST_FUSION_ADAPT_STRUCT(
  monkey_brain_core::ExpressionDescription,
  (std::string, type)(std::string, argument)(
    std::vector<monkey_brain_core::ExpressionDescription>,
    sub_expressions)
)

namespace monkey_brain_core
{

namespace
{

using Symbols = qi::symbols<char, std::string>;

ExpressionDescription make_reference(std::string value)
{
  return {"REFERENCE", {}, {value}};
}
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_reference, make_reference, 1)

template<typename T>
ExpressionDescription make_value(T value)
{
  return {"VALUE", {}, {value}};
}
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_string_value, make_value<std::string>, 1)
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_character_value, make_value<char>, 1)
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_signed_int_value, make_value<int64_t>, 1)
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_unsigned_int_value, make_value<uint64_t>, 1)
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_double_value, make_value<double>, 1)
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_bool_value, make_value<bool>, 1)

ExpressionDescription make_unary_expression(std::string op, ExpressionDescription c)
{
  return {std::move(op), {std::move(c)}, {}};
}
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_unary_expression, make_unary_expression, 2)

ExpressionDescription make_binary_expression(
  std::string op, ExpressionDescription c1,
  ExpressionDescription c2)
{
  return {std::move(op), {std::move(c1), std::move(c2)}, {}};
}
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_binary_expression, make_binary_expression, 3)

ExpressionDescription make_nary_expression(
  ExpressionDescription c0,
  std::vector<boost::fusion::vector<std::string, ExpressionDescription>> cs)
{
  if (cs.empty()) {
    return c0;
  }
  std::vector<ExpressionDescription> sub_expressions;
  sub_expressions.push_back(std::move(c0));
  using boost::fusion::at_c;
  std::string op = at_c<0>(cs.front());
  const auto end = std::find_if(
    cs.begin(), cs.end(),
    [&](const auto & c) {return op != at_c<0>(c);});
  sub_expressions.reserve(1 + std::distance(cs.begin(), end));
  std::transform(
    cs.begin(), cs.end(), std::back_inserter(sub_expressions),
    [](const auto & c) {return at_c<1>(c);});
  ExpressionDescription c{std::move(op), std::move(sub_expressions), {}};
  if (end == cs.end()) {
    return c;
  }
  cs.erase(cs.begin(), end);
  return make_nary_expression(std::move(c), std::move(cs));
}

BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_nary_expression, make_nary_expression, 2)

std::vector<ExpressionDescription> make_argument_list(
  ExpressionDescription c0,
  std::vector<boost::fusion::vector<std::string, ExpressionDescription>> cs)
{
  std::vector<ExpressionDescription> sub_expressions;
  sub_expressions.reserve(1 + cs.size());
  sub_expressions.push_back(std::move(c0));
  std::transform(
    cs.begin(), cs.end(), std::back_inserter(sub_expressions),
    [](const auto & c) {return boost::fusion::at_c<1>(c);});
  return sub_expressions;
}

BOOST_PHOENIX_ADAPT_FUNCTION(std::vector<ExpressionDescription>, to_argument_list,
      make_argument_list, 2)

ExpressionDescription make_function_expression(
  std::string name,
  std::vector<ExpressionDescription> arguments)
{
  return {std::move(name), {std::move(arguments)}, {}};
}
BOOST_PHOENIX_ADAPT_FUNCTION(ExpressionDescription, to_function_expression,
      make_function_expression, 2)

template<typename Iterator>
struct ExpressionDescriptionGrammar
  : qi::grammar<Iterator, ExpressionDescription(), ascii::space_type>
{
  ExpressionDescriptionGrammar()
  : ExpressionDescriptionGrammar::base_type(logical_expression)
  {
    using namespace qi::labels;
    using namespace boost::phoenix;
    using boost::phoenix::at_c;
    using qi::lexeme;
    using qi::char_;
    using qi::long_;
    using qi::ulong_;
    using qi::bool_;
    using qi::lit;
    using qi::hold;
    using qi::omit;
    using strict_double_ = qi::real_parser<double, qi::strict_real_policies<double>>;

    negation_op.add("NOT", "NOT")("!", "NOT");
    conjunction_op.add("AND", "AND")("&&", "AND");
    disjunction_op.add("OR", "OR")("||", "OR");

    comparison_op.add("==", "==")("!=", "!=")("<", "<")(">", ">")(">=", ">=")("<=", "<=");

    additive_op.add("+", "+")("-", "-");
    multiplicative_op.add("*", "*")("/", "/")("%", "%");
    comma.add(",", ",");

    logical_expression %= disjunctive /*| expression*/;
    disjunctive =
      (conjunctive >> *(disjunction_op >> conjunctive))[_val = to_nary_expression(_1, _2)];
    conjunctive =
      (expression >> *(conjunction_op >> expression))[_val = to_nary_expression(_1, _2)];
    negated =
      (negation_op >>
      (('(' > logical_expression > ')') | value))[_val = to_unary_expression(_1, _2)];
    expression %= negated | comparison | ('(' > logical_expression > ')') | value;
    comparison =
      (additive >> comparison_op >>
      additive)[_val = to_binary_expression(_2, _1, _3)] | additive[_val = _1];
    additive = (multiplicative >> *(additive_op >> multiplicative))[_val = to_nary_expression(
          _1,
          _2)];
    multiplicative = (factor >> *(multiplicative_op > factor))[_val = to_nary_expression(_1, _2)];
    factor = primary.alias();         // maybe add power in the future
    //primary %= ('(' > additive > ')' ) | function_or_value;
    // this is basically a HACK for making conditions like "(a + 1) < 2" work.
    primary %= ('(' > logical_expression > ')' ) | function_or_value;

    function_or_value %= hold[function] | value;
    function = (identifier >> ('(' >> arguments >> ')'))[_val = to_function_expression(_1, _2)];
    identifier %= +char_("0-9a-zA-Z/._");
    arguments = (logical_expression >> *(comma >> logical_expression))[_val = to_argument_list(_1,
          _2)];
    value = !(negation_op | conjunction_op | disjunction_op) >>
      (quoted_string[_val = to_string_value(_1)] |
      lexeme['\'' >> (char_ - '\'') >> '\''][_val = to_character_value(_1)] |
      hold[strict_double_()][_val = to_double_value(_1)] |
      hold[ulong_ >> (lit('U') | lit('u'))][_val = to_unsigned_int_value(_1)] |
      hold[long_][_val = to_signed_int_value(_1)] |
      hold[bool_[_val = to_bool_value(_1)]] |
      identifier[_val = to_reference(_1)]);
    quoted_string %= lexeme['"' >> *(char_ - '"') >> '"'];
  }

  Symbols negation_op;
  Symbols conjunction_op;
  Symbols disjunction_op;

  Symbols comparison_op;

  Symbols additive_op;
  Symbols multiplicative_op;
  Symbols comma;

  using SubExpressionRule = qi::rule<Iterator, ExpressionDescription(), ascii::space_type>;
  SubExpressionRule logical_expression;
  SubExpressionRule disjunctive;
  SubExpressionRule conjunctive;
  SubExpressionRule negated;
  SubExpressionRule expression;
  SubExpressionRule comparison;
  SubExpressionRule additive;
  SubExpressionRule multiplicative;
  SubExpressionRule factor;
  SubExpressionRule primary;
  SubExpressionRule function_or_value;
  SubExpressionRule function;
  SubExpressionRule value;

  using ArgumentListExpressionRule = qi::rule<Iterator, std::vector<ExpressionDescription>(),
      ascii::space_type>;
  ArgumentListExpressionRule arguments;

  using ValueCategoryRule = qi::rule<Iterator, std::string()>;
  ValueCategoryRule identifier;
  ValueCategoryRule quoted_string;
};

} // namespace

std::optional<ExpressionDescription> parse_expression(const std::string & expression)
{
  if (expression.empty()) {
    return std::nullopt;
  }
  ExpressionDescriptionGrammar<std::string::const_iterator> grammar;
  ExpressionDescription result;
  using boost::spirit::ascii::space;
  auto begin = expression.begin();
  auto end = expression.end();
  bool r = phrase_parse(begin, end, grammar, space, result);

  if (r && begin == end) {
    return {result};
  } else {
    throw std::invalid_argument("parser error: invalid Expression: '" + expression + "'");
  }
}

} // namespace monkey_brain_core
