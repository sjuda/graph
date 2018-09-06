#include "stdafx.h"
#include <algorithm>
#include <iostream>
#include <set>
#include <map>
#include <unordered_map>
#include <vector>
using namespace std;

template <typename CostType, typename VertexId>
class Graph
{
public:
	struct EDge
	{
		VertexId from;
		VertexId to;
		CostType cost;
	};
	struct Node
	{
		vector<VertexId> in;
		vector<VertexId> out;
	};

	Graph()
	{
		next_edge_id = 0;
	}
	~Graph()
	{
	}
	size_t VerticiesCount() const
	{
		return Vertices.size();
	}
	size_t EdgesCount() const
	{
		return Edges.size();
	}
	size_t VertexDegree(const VertexId &v)
	{
		size_t ret = 0;
		for (auto &i : Vertices.at(v).in)
			ret++;
		for (auto &i : Vertices.at(v).out)
			ret++;
		return ret;
	}
	Graph& AddVertex(const VertexId& v)
	{
		if(Vertices.find(v) != Vertices.end())
			return *this;
		Node t;
		Vertices.insert(pair<VertexId, Node>(v, t));
		return *this;
	}
	Graph& AddEdge(const VertexId& from, const VertexId& to, CostType cost)
	{
		for (auto &i : Edges)
		{
			if (i.second.from == from && i.second.to == to)
				return *this;
		}
		auto uid = get_next_edge_id();
		EDge ins;
		ins.from = from;
		Vertices[from].out.push_back(uid);
		ins.to = to;
		Vertices[to].in.push_back(uid);
		ins.cost = cost;
		Edges.insert(pair<VertexId, EDge>(uid, ins));
		return *this;
	}
	set<VertexId> GetVertices() const
	{
		set<VertexId> ret;
		for (auto &i : Vertices)
			ret.insert(i.first);
		return ret;
	}
	set<pair<VertexId, VertexId>> GetEdges() const
	{
		set<pair<VertexId, VertexId>> ret;
		for (auto &i : Edges)
			ret.insert(pair<VertexId, VertexId>(i.second.from, i.second.to));
		return ret;
	}
	set<VertexId> AdjacentVertices(const VertexId& v) const
	{
		set<VertexId> ret;
		for (auto &i : Vertices.at(v).in)
			ret.insert(Edges.at(i).from);
		for (auto &i : Vertices.at(v).out)
			ret.insert(Edges.at(i).to);
		return ret;
	}
	set<pair<VertexId, VertexId>> AdjacentEdges(const VertexId& v) const
	{
		set<pair<VertexId, VertexId>>ret;
		for (auto &i : Vertices.at(v).in)
			ret.insert(pair<VertexId, VertexId>(Edges.at(i).from, Edges.at(i).to));
		for (auto &i : Vertices.at(v).out)
			ret.insert(pair<VertexId, VertexId>(Edges.at(i).from, Edges.at(i).to));
		return ret;
	}

	Node GetNodeByVertexId(VertexId id)
	{
		return Vertices.at(id);
	}

	EDge GetEdgeById(VertexId id)
	{
		return Edges.at(id);
	}

	CostType Cost(const VertexId& from, const VertexId& to) const
	{
		for (auto &i : Edges)
		{
			if (i.second.from == from && i.second.to == to)
				return i.second.cost;
		}
		return -1;
	}
private:
	VertexId next_edge_id;
	VertexId get_next_edge_id() { return next_edge_id++; }
	map<VertexId, EDge> Edges;
	map<VertexId, Node> Vertices;
};

template <typename CostType, typename VertexId>
void dijkstra(Graph<CostType, VertexId> g, VertexId source, VertexId target, std::unordered_map<VertexId, CostType> &return_dist, std::unordered_map<VertexId, VertexId> &return_prev)
{
	const CostType INF = numeric_limits<CostType>::infinity();
	std::vector<VertexId> list;
	std::unordered_map<VertexId, CostType> dist;
	std::unordered_map<VertexId, VertexId> prev;

	for (auto n : g.GetVertices())
	{
		list.push_back(n);
		dist.emplace(n, INF);
		prev.emplace(n, -1);
	}
	dist.at(source) = 0;
	while (!list.empty())
	{
		VertexId node = list.at(0);
		for (auto id : list)
		{
			if (dist.at(id) < dist.at(node))
				node = id;
		}
		if (node == target)
			break;
		list.erase(std::remove(list.begin(), list.end(), node), list.end());
		if (dist.at(node) >= INF)
			break;

		for (auto &e : g.GetNodeByVertexId(node).out)
		{
			auto neighbour = g.GetEdgeById(e).to;
			auto new_dist = dist.at(node) + g.GetEdgeById(e).cost;
			if (new_dist < dist.at(neighbour))
			{
				dist.at(neighbour) = new_dist;
				prev.at(neighbour) = node;
			}

		}
	}

	return_dist = dist;
	return_prev = prev;
}

int main()
{
#define Max 5
	Graph<double, int> a;
	for (int i = 0; i <= Max; i++)
		a.AddVertex(i);

	a.AddEdge(0, 1, 4);
	a.AddEdge(0, 2, 2);
	a.AddEdge(1, 2, 5);
	a.AddEdge(1, 3, 10);
	a.AddEdge(2, 4, 3);
	a.AddEdge(4, 3, 4);
	a.AddEdge(3, 5, 11);
	
	for (auto &i : a.GetEdges())
		cout << i.first << " " << i.second << " " << a.Cost(i.first, i.second) << endl;
	cout << "Count " << a.EdgesCount() << " " << a.VerticiesCount() << endl;
	for (auto &i : a.AdjacentEdges(1))
		cout << i.first << " " << i.second << " " << a.Cost(i.first, i.second) << endl;
	cout << a.VertexDegree(1) << endl;

	std::unordered_map<int, double> dist; // id, distance
	std::unordered_map<int, int> prev; // id, previous
	dijkstra<double, int>(a, 0, Max, dist, prev);

	cout << "Prev result" << endl;
	for (auto n : prev)
		cout << n.first << ", " << n.second << endl;

	// Construct path
	std::vector<int> path;
	int u = Max;
	while (prev.at(u) >= 0)
	{
		path.push_back(u);
		u = prev.at(u);
	}
	// Add source (will be target if no feasible path is found)
	path.push_back(u);

	cout << "Path " << endl;
	for (auto n : path)
		cout << n << endl;

	// Print results
	cout << "The shortest path from " << 0 << " to " << Max << " is:" << endl;
	for (int i = path.size() - 1; i >= 0; i--)
	{
		auto n = path.at(i);
		cout << n << " (" << dist.at(n) << ")";
		if (i > 0)
			cout << " => ";
	}
	cout << endl;
}
