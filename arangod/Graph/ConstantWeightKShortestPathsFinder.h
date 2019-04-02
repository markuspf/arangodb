////////////////////////////////////////////////////////////////////////////////
/// DISCLAIMER
///
/// Copyright 2014-2019 ArangoDB GmbH, Cologne, Germany
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

// TODO: callback

#ifndef ARANGODB_GRAPH_CONSTANT_WEIGHT_K_SHORTEST_PATHS_FINDER_H
#define ARANGODB_GRAPH_CONSTANT_WEIGHT_K_SHORTEST_PATHS_FINDER_H 1

#include "Aql/AqlValue.h"
#include "Basics/VelocyPackHelper.h"
#include "Graph/EdgeDocumentToken.h"
#include "Graph/ShortestPathFinder.h"

#include <velocypack/StringRef.h>

namespace arangodb {

namespace velocypack {
class Slice;
}

namespace graph {

struct ShortestPathOptions;

// Class to find k shortest paths (with constant weight).
// Given two vertices <start> and <end>, forms to balls around
// <start> and <end> and expands them until they intersect in a
// vertex
//
// Inherit from ShortestPathfinder to get destroyEngines and not copy it
// again.
// TODO: Traverser.h has destroyEngines as well (the code for the two functions
//       is identical), refactor?
class ConstantWeightKShortestPathsFinder : public ShortestPathFinder {
 private:
  // Mainly for readability
  typedef arangodb::velocypack::StringRef VertexRef;
  enum Direction { FORWARD, BACKWARD };

  // A path snippet contains an edge and a vertex
  // and is used to reconstruct the path
  struct PathSnippet {
    VertexRef const _pred;
    graph::EdgeDocumentToken _path;

    PathSnippet(VertexRef& pred, graph::EdgeDocumentToken&& path);
  };

  struct FoundVertex {
    // Number of paths to this vertex
    bool _startOrEnd;
    size_t _depth;
    size_t _npaths;

    // Predecessor edges
    std::vector<PathSnippet> _snippets;

    // Used to assemble paths
    std::vector<PathSnippet>::iterator _tracer;

    FoundVertex(void)
        : _startOrEnd(false), _depth(0), _npaths(0), _snippets({}){};
    FoundVertex(bool startOrEnd)  // _npaths is 1 for start/end vertices
        : _startOrEnd(startOrEnd), _depth(0), _npaths(1), _snippets({}){};
    FoundVertex(bool startOrEnd, size_t depth, size_t npaths)
        : _startOrEnd(startOrEnd), _depth(depth), _npaths(npaths), _snippets({}){};
  };
  typedef std::deque<VertexRef> Frontier;
  typedef std::deque<VertexRef> Trace;

  // Contains the vertices that were found while searching
  // for a shortest path between start and end together with
  // the number of paths leading to that vertex and information
  // how to trace paths from the vertex from start/to end.
  typedef std::unordered_map<VertexRef, FoundVertex> FoundVertices;

  struct Ball {
    VertexRef _centre;
    Direction _direction;
    FoundVertices _vertices;
    Frontier _frontier;
    Trace _trace;

    Ball(void) {};
    void clear() {
      _vertices.clear();
      _frontier.clear();
      _trace.clear();
    };
    void setCentre(const VertexRef& centre) {
      _centre = centre;
      _frontier.emplace_back(centre);
      _vertices.emplace(centre, FoundVertex(true));
    }
    void setDirection(Direction direction) {
      _direction = direction;
    }
    void setup(const VertexRef& centre, Direction direction){
      clear();
      setCentre(centre);
      setDirection(direction);
    }
  };

 public:
  explicit ConstantWeightKShortestPathsFinder(ShortestPathOptions& options);
  ~ConstantWeightKShortestPathsFinder();

  //
  void destroyEngines();

  // This is here because we inherit from ShortestPathFinder (to get the destroyEngines function)
  // TODO: Remove
  bool shortestPath(arangodb::velocypack::Slice const& start,
                    arangodb::velocypack::Slice const& target,
                    arangodb::graph::ShortestPathResult& result,
                    std::function<void()> const& callback) { TRI_ASSERT(true); };

  //
  size_t startKShortestPathsTraversal(arangodb::velocypack::Slice const& start,
                                      arangodb::velocypack::Slice const& end);

  // get the next available path as AQL value.
  bool getNextPathAql(arangodb::velocypack::Builder& builder);
  // get the next available path as a ShortestPathResult
  bool getNextPath(arangodb::graph::ShortestPathResult& path);
  bool isPathAvailable( void ) { return false; };

private:
  void resetSearch();
  void meetClosures();
  void computeNeighbourhoodOfVertex(VertexRef vertex, Direction direction, std::vector<VertexRef>& neighbours,
                                    std::vector<graph::EdgeDocumentToken> edges);

  // returns the number of paths found
  size_t advanceFrontier(Ball& source, Direction direction);

  // Set all iterators in _leftFound and _rightFound to the beginning
  void preparePathIteration(void);

  // Move to the next path
  void advancePathIterator(void);

 private:
  Ball _left;
  Ball _right;

  // A bit ugly: I want the time to produce the next path
  // to be spend when actually making that path, not after
  // making the previous one.
  // This makes the first path special, since all
  // that needs to be done is to set all iterators to begin()
  // If you have a prettier way of doing this, I'd like to hear it.
  bool _firstPath;
};

}  // namespace graph
}  // namespace arangodb
#endif
