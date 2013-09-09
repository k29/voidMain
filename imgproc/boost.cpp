//=======================================================================
// Copyright 2001 Jeremy G. Siek, Andrew Lumsdaine, Lie-Quan Lee, 
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
#include <boost/config.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <boost/graph/adjacency_list.hpp>
#include <boost/tuple/tuple.hpp>
enum family
{ Jeanie, Debbie, Rick, John, Amanda, Margaret, Benjamin, N };

struct Point
{
  double x;
  double y;
};

struct Edge
{
  double d;
};

int
main()
{
  using namespace boost;
  const char *name[] = { "Jeanie", "Debbie", "Rick", "John", "Amanda",
    "Margaret", "Benjamin"
  };

  adjacency_list <> g(N);
  typedef adjacency_list <vecS, vecS, directedS, Point, Edge> MyGraph;
  MyGraph graph(2);

  add_edge(0, 1, graph);



  MyGraph::vertex_descriptor v = *vertices(graph).first;
  graph[v].x = 21.0;
  graph[v].y = 20.0;
  graph::edge_descriptor e = *out_edges(v, graph).first;
  graph[e].name = "I-87";
  graph[e].miles = 10.;
  graph[e].speed_limit = 65;
  graph[e].lanes = 4;
  graph[e].divided = true;




  add_edge(Jeanie, Debbie, g);
  add_edge(Jeanie, Rick, g);
  add_edge(Jeanie, John, g);
  add_edge(Debbie, Amanda, g);
  add_edge(Rick, Margaret, g);
  add_edge(John, Benjamin, g);

  graph_traits < MyGraph >::vertex_iterator i, end;
  graph_traits < adjacency_list <> >::adjacency_iterator ai, a_end;
  property_map < adjacency_list <>, vertex_index_t >::type
    index_map = get(vertex_index, g);

  for (boost::tie(i, end) = vertices(g); i != end; ++i) {
    std::cout << name[get(index_map, *i)];
    boost::tie(ai, a_end) = adjacent_vertices(*i, g);
    if (ai == a_end)
      std::cout << " has no children";
    else
      std::cout << " is the parent of ";
    for (; ai != a_end; ++ai) {
      std::cout << name[get(index_map, *ai)];
      if (boost::next(ai) != a_end)
        std::cout << ", ";
    }
    std::cout << std::endl;
  }
  return EXIT_SUCCESS;
}