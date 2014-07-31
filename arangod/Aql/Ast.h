////////////////////////////////////////////////////////////////////////////////
/// @brief Aql, query AST
///
/// @file
///
/// DISCLAIMER
///
/// Copyright 2014 ArangoDB GmbH, Cologne, Germany
/// Copyright 2004-2014 triAGENS GmbH, Cologne, Germany
///
/// Licensed under the Apache License, Version 2.0 (the "License");
/// you may not use this file except in compliance with the License.
/// You may obtain a copy of the License at
///
///     http://www.apache.org/licenses/LICENSE-2.0
///
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.
///
/// Copyright holder is ArangoDB GmbH, Cologne, Germany
///
/// @author Jan Steemann
/// @author Copyright 2014, ArangoDB GmbH, Cologne, Germany
/// @author Copyright 2012-2013, triAGENS GmbH, Cologne, Germany
////////////////////////////////////////////////////////////////////////////////

#ifndef ARANGODB_AQL_AST_H
#define ARANGODB_AQL_AST_H 1

#include "Basics/Common.h"
#include "Aql/AstNode.h"
#include "Aql/BindParameters.h"
#include "Aql/Scopes.h"
#include "Aql/Variable.h"
#include "BasicsC/json.h"

#include <functional>

struct TRI_json_s;

namespace triagens {
  namespace aql {

    class Parser;
    class Query;

// -----------------------------------------------------------------------------
// --SECTION--                                                         class Ast
// -----------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/// @brief the AST
////////////////////////////////////////////////////////////////////////////////

    class Ast {

      enum FilterType {
        FILTER_UNKNOWN,
        FILTER_TRUE,
        FILTER_FALSE
      };

// -----------------------------------------------------------------------------
// --SECTION--                                        constructors / destructors
// -----------------------------------------------------------------------------

      public:

////////////////////////////////////////////////////////////////////////////////
/// @brief create the AST
////////////////////////////////////////////////////////////////////////////////

        Ast (Query*,
             Parser*);

////////////////////////////////////////////////////////////////////////////////
/// @brief destroy the AST
////////////////////////////////////////////////////////////////////////////////

        ~Ast ();

// -----------------------------------------------------------------------------
// --SECTION--                                                    public methods
// -----------------------------------------------------------------------------

      public:

////////////////////////////////////////////////////////////////////////////////
/// @brief return the root of the AST
////////////////////////////////////////////////////////////////////////////////

        inline AstNode const* root () const {
          return _root;
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief begin a subquery
////////////////////////////////////////////////////////////////////////////////

        void startSubQuery () {
          // insert a new root node
          AstNodeType type;

          if (_queries.empty()) {
            // root node of query
            type = NODE_TYPE_ROOT;
          }
          else {
            // sub query node
            type = NODE_TYPE_SUBQUERY;
          }

          auto root = createNode(type);

          // save the root node
          _queries.push_back(root);

          // set the current root node if everything went well
          _root = root;
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief end a subquery
////////////////////////////////////////////////////////////////////////////////

        AstNode* endSubQuery () {
          // get the current root node
          AstNode* root = _queries.back();
          // remove it from the stack
          _queries.pop_back();

          // set root node to previous root node
          _root = _queries.back();

          // return the root node we just popped from the stack
          return root;
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief whether or not we currently are in a subquery
////////////////////////////////////////////////////////////////////////////////

        bool isInSubQuery () const {
          return (_queries.size() > 1);
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief return a copy of our own bind parameters
////////////////////////////////////////////////////////////////////////////////

        std::unordered_set<std::string> bindParameters () const {
          return std::unordered_set<std::string>(_bindParameters);
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief return a copy of the collection names used in the query
////////////////////////////////////////////////////////////////////////////////

        std::unordered_set<std::string> collectionNames () const {
          return std::unordered_set<std::string>(_collectionNames);
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief get the query scopes
////////////////////////////////////////////////////////////////////////////////

        inline Scopes* scopes () {
          return &_scopes;
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief track the write collection
////////////////////////////////////////////////////////////////////////////////

        inline void setWriteCollection (AstNode const* node) {
          TRI_ASSERT(node->type == NODE_TYPE_COLLECTION ||
                     node->type == NODE_TYPE_PARAMETER);

          _writeCollection = node;
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief track the write query options
////////////////////////////////////////////////////////////////////////////////

        inline void setWriteOptions (AstNode const* node) {
          _writeOptions = node;
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief returns the next variable id
////////////////////////////////////////////////////////////////////////////////

        inline VariableId nextVariableId () {
          return ++_variableId;
        }

////////////////////////////////////////////////////////////////////////////////
/// @brief generate a new unique variable name
////////////////////////////////////////////////////////////////////////////////
  
        char const* generateName ();

////////////////////////////////////////////////////////////////////////////////
/// @brief convert the AST into JSON
/// the caller is responsible for freeing the JSON later
////////////////////////////////////////////////////////////////////////////////

        struct TRI_json_s* toJson (TRI_memory_zone_t*);

////////////////////////////////////////////////////////////////////////////////
/// @brief add an operation to the root node
////////////////////////////////////////////////////////////////////////////////

        void addOperation (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST for node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeFor (char const*,
                                AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST let node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeLet (char const*,
                                AstNode const*,
                                bool);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST filter node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeFilter (AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST return node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeReturn (AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST remove node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeRemove (AstNode const*,
                                   AstNode const*,
                                   AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST insert node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeInsert (AstNode const*,
                                   AstNode const*,
                                   AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST update node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeUpdate (AstNode const*,
                                   AstNode const*,
                                   AstNode const*,
                                   AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST replace node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeReplace (AstNode const*,
                                    AstNode const*,
                                    AstNode const*,
                                    AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST collect node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeCollect (AstNode const*,
                                    char const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST sort node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeSort (AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST sort element node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeSortElement (AstNode const*,
                                        bool);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST limit node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeLimit (AstNode const*,
                                  AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST assign node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeAssign (char const*,
                                   AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST variable node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeVariable (char const*,
                                     bool);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST collection node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeCollection (char const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST reference node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeReference (char const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST parameter node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeParameter (char const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST unary operator
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeUnaryOperator (AstNodeType type,
                                          AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST binary operator
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeBinaryOperator (AstNodeType type,
                                           AstNode const*,
                                           AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST ternary operator
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeTernaryOperator (AstNode const*,
                                            AstNode const*,
                                            AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST subquery node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeSubquery (char const*,
                                     AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST attribute access node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeAttributeAccess (AstNode const*,
                                            char const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST attribute access node w/ bind parameter
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeBoundAttributeAccess (AstNode const*,
                                                 AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST index access node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeIndexedAccess (AstNode const*,
                                          AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST expand node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeExpand (AstNode const*,
                                   AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST iterator node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeIterator (char const*,
                                     AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST null value node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeValueNull ();

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST bool value node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeValueBool (bool);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST int value node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeValueInt (int64_t);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST double value node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeValueDouble (double);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST string value node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeValueString (char const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST list node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeList ();

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST array node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeArray ();

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST array element node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeArrayElement (char const*,
                                         AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST function call node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeFunctionCall (char const*,
                                         AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST range node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeRange (AstNode const*,
                                  AstNode const*); 

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST nop node
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNodeNop ();

////////////////////////////////////////////////////////////////////////////////
/// @brief injects bind parameters into the AST
////////////////////////////////////////////////////////////////////////////////

        void injectBindParameters (BindParameters&);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the AST
////////////////////////////////////////////////////////////////////////////////

        void optimize ();

// -----------------------------------------------------------------------------
// --SECTION--                                                   private methods
// -----------------------------------------------------------------------------

      private:

////////////////////////////////////////////////////////////////////////////////
/// @brief executes a comparison function using two constant values
////////////////////////////////////////////////////////////////////////////////

        AstNode* executeConstComparison (std::string const&,
                                         AstNode const*,
                                         AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes a FILTER node
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeFilter (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the unary operators + and -
/// the unary plus will be converted into a simple value node if the operand of
/// the operation is a constant number
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeUnaryOperatorArithmetic (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the unary operator NOT
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeUnaryOperatorLogical (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the binary logical operators && and ||
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeBinaryOperatorLogical (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the binary relational operators <, <=, >, >=, ==, != and IN
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeBinaryOperatorRelational (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the binary arithmetic operators +, -, *, / and %
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeBinaryOperatorArithmetic (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the ternary operator
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeTernaryOperator (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes a reference to a variable
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeReference (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the range operator
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeRange (AstNode*);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the LET statement
////////////////////////////////////////////////////////////////////////////////

        AstNode* optimizeLet (AstNode*,
                              int);

////////////////////////////////////////////////////////////////////////////////
/// @brief optimizes the top-level statements
////////////////////////////////////////////////////////////////////////////////

        void optimizeRoot ();

////////////////////////////////////////////////////////////////////////////////
/// @brief create an AST node from JSON
////////////////////////////////////////////////////////////////////////////////

        AstNode* nodeFromJson (TRI_json_t const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief traverse the AST
////////////////////////////////////////////////////////////////////////////////

        AstNode* traverse (AstNode*,
                           std::function<AstNode*(AstNode*, void*)>,
                           void*);

////////////////////////////////////////////////////////////////////////////////
/// @brief determines the variables referenced in an expression
////////////////////////////////////////////////////////////////////////////////

        std::unordered_set<VariableId> getReferencedVariables (AstNode const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief normalize a function name
////////////////////////////////////////////////////////////////////////////////

        std::string normalizeFunctionName (char const*);

////////////////////////////////////////////////////////////////////////////////
/// @brief create a node of the specified type
////////////////////////////////////////////////////////////////////////////////

        AstNode* createNode (AstNodeType);

// -----------------------------------------------------------------------------
// --SECTION--                                                 private variables
// -----------------------------------------------------------------------------

      private:

////////////////////////////////////////////////////////////////////////////////
/// @brief the query
////////////////////////////////////////////////////////////////////////////////

        Query*                           _query;

////////////////////////////////////////////////////////////////////////////////
/// @brief the query parser
////////////////////////////////////////////////////////////////////////////////

        Parser*                          _parser;

////////////////////////////////////////////////////////////////////////////////
/// @brief the next assigned variable id
////////////////////////////////////////////////////////////////////////////////

        VariableId                       _variableId;

////////////////////////////////////////////////////////////////////////////////
/// @brief all nodes created in the AST - will be used for freeing them later
////////////////////////////////////////////////////////////////////////////////

        std::vector<AstNode*>            _nodes;

////////////////////////////////////////////////////////////////////////////////
/// @brief all scopes used in the query
////////////////////////////////////////////////////////////////////////////////
        
        Scopes                           _scopes;

////////////////////////////////////////////////////////////////////////////////
/// @brief the bind parameters we found in the query
////////////////////////////////////////////////////////////////////////////////

        std::unordered_set<std::string>  _bindParameters;

////////////////////////////////////////////////////////////////////////////////
/// @brief the collection names used found in the query
////////////////////////////////////////////////////////////////////////////////

        std::unordered_set<std::string>  _collectionNames;

////////////////////////////////////////////////////////////////////////////////
/// @brief root node of the AST
////////////////////////////////////////////////////////////////////////////////

        AstNode*                         _root;

////////////////////////////////////////////////////////////////////////////////
/// @brief root nodes of queries and subqueries
////////////////////////////////////////////////////////////////////////////////

        std::vector<AstNode*>            _queries;

////////////////////////////////////////////////////////////////////////////////
/// @brief which collection is going to be modified in the query 
////////////////////////////////////////////////////////////////////////////////

        AstNode const*                   _writeCollection;

////////////////////////////////////////////////////////////////////////////////
/// @brief which options are used for write queries
////////////////////////////////////////////////////////////////////////////////

        AstNode const*                   _writeOptions;

////////////////////////////////////////////////////////////////////////////////
/// @brief AQL function names
////////////////////////////////////////////////////////////////////////////////

        static std::unordered_map<int, std::string> const FunctionNames;

    };

  }
}

#endif

// -----------------------------------------------------------------------------
// --SECTION--                                                       END-OF-FILE
// -----------------------------------------------------------------------------

// Local Variables:
// mode: outline-minor
// outline-regexp: "/// @brief\\|/// {@inheritDoc}\\|/// @page\\|// --SECTION--\\|/// @\\}"
// End:
