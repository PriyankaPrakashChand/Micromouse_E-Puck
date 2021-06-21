#ifndef __VERTEX__
#define __VERTEX__

#include <climits>
#include <list>
#include <array>

using std::array;
using std::list;

struct Location
{
	int row = -1;
	int col = -1;
};

class Vertex
{
public:
	class Edge
	{
	public:
		Edge() {}

		bool isReachable() const
		{
			return to;
		}

		int getEdgeLength() const
		{
			return len;
		}

		Vertex *getVertex()
		{
			return to;
		}

		void set(Vertex *v, int length)
		{
			to = v;
			len = length;
		}

		~Edge()
		{
			to = nullptr;
		}

	private:
		int len = INT_MAX;	  // length set to infinity
		Vertex *to = nullptr; // to which vertex

		friend class Vertex;
	};

private:
	bool visted = false; // to check if cell is visted
	int len = INT_MAX;	 // length set to infinity

	static const int EDGE_SIZE = 4;

	// Edge directions[EDGE_SIZE]; // Edges to top down left right cells
	array<Edge, EDGE_SIZE> directions; // Edges to top down left right cells

	// position of cell of in maze
	int row = -1;
	int col = -1;

public:
	using iterator = array<Edge, EDGE_SIZE>::iterator;
	using const_iterator = array<Edge, EDGE_SIZE>::const_iterator;

	~Vertex() {}

	iterator begin()
	{
		return directions.begin();
	}

	const_iterator begin() const
	{
		return directions.begin();
	}

	iterator end()
	{
		return directions.end();
	}

	const_iterator end() const
	{
		return directions.end();
	}

	array<Edge, EDGE_SIZE> &getEdges()
	{
		return directions;
	}

	int edgesSize()
	{
		return EDGE_SIZE;
	}

	// return row position
	int getRow()
	{
		return row;
	}

	// return column position
	int getColumn()
	{
		return col;
	}

	// set row and column position
	void setPosition(int _row, int _col)
	{
		row = _row;
		col = _col;
	}

	/*
		Set Vertex to direction and length to vertex
	*/

	void setTop(Vertex *v, int len = 1)
	{
		directions[0].to = v;
		directions[0].len = len;
	}

	void setDown(Vertex *v, int len = 1)
	{
		directions[1].to = v;
		directions[1].len = len;
	}

	void setLeft(Vertex *v, int len = 1)
	{
		directions[2].to = v;
		directions[2].len = len;
	}

	void setRight(Vertex *v, int len = 1)
	{
		directions[3].to = v;
		directions[3].len = len;
	}

	// set length of Vertex
	void setLength(int _len)
	{
		len = _len;
	}

	// get length of Vertex
	int getLength()
	{
		return len;
	}

	// set visted to b
	void setVisted(bool b)
	{
		visted = b;
	}

	// return visted
	bool isVisted()
	{
		return visted;
	}

	/*
		len set to infinity
		visited set to false
	*/
	void reset()
	{
		visted = false;
		len = INT_MAX;
	}

	Edge &connectingEdge(Vertex *to)
	{
		for (int i = 0; i < edgesSize(); ++i)
		{
			if (getEdges()[i].getVertex() == to)
			{
				return getEdges()[i];
			}
		}
		throw;
	}
};

#endif // !__VERTEX__