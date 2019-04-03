////////////////////////////////////////////////////////////////////////////////
/// DISCLAIMER
///
/// Copyright 2014-2016 ArangoDB GmbH, Cologne, Germany
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
/// @author Markus Pfeiffer
////////////////////////////////////////////////////////////////////////////////

#include "ConstantWeightKShortestPathsFinder.h"

#include "Aql/AqlValue.h"
#include "Cluster/ServerState.h"
#include "Graph/EdgeCursor.h"
#include "Graph/EdgeDocumentToken.h"
#include "Graph/ShortestPathOptions.h"
#include "Graph/ShortestPathResult.h"
#include "Graph/TraverserCache.h"
#include "Transaction/Helpers.h"
#include "Utils/OperationCursor.h"
#include "VocBase/LogicalCollection.h"

#include <velocypack/Iterator.h>
#include <velocypack/Slice.h>
#include <velocypack/StringRef.h>
#include <velocypack/velocypack-aliases.h>

using namespace arangodb;
using namespace arangodb::graph;

ConstantWeightKShortestPathsFinder::PathSnippet::PathSnippet(VertexRef const& pred,
                                                           EdgeDocumentToken&& path)
    : _pred(pred), _path(std::move(path)) {}



//
ConstantWeightKShortestPathsFinder::ConstantWeightKShortestPathsFinder(ShortestPathOptions& options)
  : ShortestPathFinder(options) {
}

ConstantWeightKShortestPathsFinder::~ConstantWeightKShortestPathsFinder() {
}

// Sets up k-shortest-paths traversal from start to end
// Returns number of currently known paths
size_t ConstantWeightKShortestPathsFinder::startKShortestPathsTraversal(
  arangodb::velocypack::Slice const& start, arangodb::velocypack::Slice const& end) {
  if (start == end) {
    return 1;
  }

  _firstPath = true;
  _left.setup(arangodb::velocypack::StringRef(start), FORWARD);
  _right.setup(arangodb::velocypack::StringRef(end), BACKWARD);

  // TODO: This probably should be moved
  TRI_IF_FAILURE("TraversalOOMInitialize") {
    THROW_ARANGO_EXCEPTION(TRI_ERROR_DEBUG);
  }

  return 0;
}

// Finds the shortest path
void ConstantWeightKShortestPathsFinder::meetClosures() {
  std::vector<VertexRef> intersection;

  while (!_left._frontier.empty() && !_right._frontier.empty() && intersection.empty()) {
//    callback();

    // Choose the smaller frontier to expand.
    // TODO: could we use neighbourhood size? It requires
    //       fetching the neighbours, which might be expensive...
    if (_left._frontier.size() < _right._frontier.size()) {
      //
      advanceFrontier(_left, _right, intersection);
    } else {
      advanceFrontier(_right, _left, intersection);
    }
  }
}

void ConstantWeightKShortestPathsFinder::computeNeighbourhoodOfVertex(
    VertexRef vertex, Direction direction, std::vector<VertexRef>& neighbours,
    std::vector<graph::EdgeDocumentToken> edges) {

  std::unique_ptr<EdgeCursor> edgeCursor;

  switch (direction) {
    case BACKWARD:
      edgeCursor.reset(_options.nextReverseCursor(vertex));
      break;
    case FORWARD:
      edgeCursor.reset(_options.nextCursor(vertex));
      break;
    default:
      TRI_ASSERT(true);
  }

  auto callback = [&](EdgeDocumentToken&& eid, VPackSlice edge, size_t cursorIdx) -> void {
    if (edge.isString()) {
      if (edge.compareString(vertex.data(), vertex.length()) != 0) {
        VertexRef id = _options.cache()->persistString(VertexRef(edge));
        edges.emplace_back(std::move(eid));
        neighbours.emplace_back(id);
      }
    } else {
      VertexRef other(transaction::helpers::extractFromFromDocument(edge));
      if (other == vertex) {
        other = VertexRef(transaction::helpers::extractToFromDocument(edge));
      }
      if (other != vertex) {
        VertexRef id = _options.cache()->persistString(other);
        edges.emplace_back(std::move(eid));
        neighbours.emplace_back(id);
      }
    }
  };
  edgeCursor->readAll(callback);
}

size_t ConstantWeightKShortestPathsFinder::advanceFrontier(Ball& source,
                                                           const Ball& target,
                                                           std::vector<VertexRef>& intersection) {
  std::vector<VertexRef> neighbours;
  std::vector<graph::EdgeDocumentToken> edges;
  size_t depth = 0;
  size_t pathsToV = 0;
  Frontier newFrontier;

  for (auto& v : source._frontier) {
    neighbours.clear();
    edges.clear();

    computeNeighbourhoodOfVertex(v, source._direction, neighbours, edges);
    // TODO: This means "neighbours" can contain the same neighbour
    //       more than once
    TRI_ASSERT(edges.size() == neighbours.size());
    size_t const neighboursSize = neighbours.size();
    for (size_t i = 0; i < neighboursSize; ++i) {
      auto const& n = neighbours[i];

      // NOTE: _edges[i] stays intact after move
      // and is reset to a nullptr. So if we crash
      // here no mem-leaks. or undefined behavior
      // Just make sure _edges is not used after
      auto snippet = PathSnippet(v, std::move(edges[i]));
      auto inserted = source._vertices.emplace(n, FoundVertex(false, depth + 1, pathsToV));
      auto& w = inserted.first->second;
      w._snippets.emplace_back(snippet);

      // If we know this vertex and it is at the frontier, we found more paths
      if (!inserted.second && w._depth == depth + 1) {
        w._npaths += pathsToV;
      }

      auto found = target._vertices.find(n);
      if (found != target._vertices.end()) {
        // This is a path joining node, but we do not know
        // yet whether we added all paths to it, so we have
        // to finish computing the closure.
        intersection.emplace_back(n);
      }

      // vertex was new
      if (inserted.second) {
        newFrontier.emplace_back(n);
      }
    }
  }
  source._frontier = newFrontier;
  return 0;
}

bool ConstantWeightKShortestPathsFinder::getNextPath(arangodb::graph::ShortestPathResult& path) {
  return false;
}

bool ConstantWeightKShortestPathsFinder::getNextPathAql(arangodb::velocypack::Builder& result) {
  ShortestPathResult path;

  if(getNextPath(path)) {
    result.clear();
    result.openObject();

    result.add(VPackValue("edges"));
    result.openArray();
    for (auto const& it : path._edges) {
      // TRI_ASSERT(it != nullptr);
      _options.cache()->insertEdgeIntoResult(it, result);
    }
    result.close(); // Array

    result.add(VPackValue("vertices"));
    result.openArray();
    for (auto const& it : path._vertices) {
      _options.cache()->insertVertexIntoResult(it, result);
    }
    result.close(); // Array

    result.close(); // Object
    TRI_ASSERT(result.isClosed());
    auto retresult = arangodb::aql::AqlValue(result.slice());
    return true;
  } else {
    return false;
  }
}

void ConstantWeightKShortestPathsFinder::preparePathIteration(void) {
}

void ConstantWeightKShortestPathsFinder::advancePathIterator(void) {
}

