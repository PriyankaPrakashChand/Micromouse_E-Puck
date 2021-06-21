#ifndef __PATH__
#define __PATH__

#include <list>
#include <queue>
#include <math.h>
#include "Vertex.h"

class Path
{

protected:
	std::list<Vertex *> path; // path from start to end

	/*
		Inital check done before path finding 

		Parameters
		Start end : postion to start to end

		return true if specified path finding should continue
	*/
	bool intialCheck(Vertex *start, Vertex *end)
	{
		if (!start || !end) // if start or end are nullptr path doesnot exist
			return false;

		if (start == end)
		{ // if start equals end path found return false
			path.push_back(start);
			return false;
		}

		return true; // find path
	}

public:
	// default constructor
	Path() {}

	//default
	~Path()
	{
		path.clear();
	}

	// get path
	const list<Vertex *> &getPath()
	{
		return path;
	}

public:
	void findPath(Vertex *start, Vertex *end)
	{
		// using namespace std::placeholders;
		if (!intialCheck(start, end))
			return;

		orderByAStar comparator;
		comparator.col = end->getColumn();
		comparator.row = end->getRow();

		start->setLength(0);

		std::priority_queue<Vertex *, std::vector<Vertex *>, orderByAStar> q{comparator};
		q.push(start);
		Vertex *next = nullptr;

		while (!q.empty())
		{

			next = q.top();
			q.pop();

			if (next->isVisted())
				continue;

			next->setVisted(true);

			if (next == end)
				break;

			for (auto &x : *next)
			{
				if (x.isReachable() && !x.getVertex()->isVisted())
				{
					setDistance(next, x.getVertex(), x.getEdgeLength());
					q.push(x.getVertex());
				}
			}
		}

		trace(start, end);
	}

	void trace(Vertex *start, Vertex *end)
	{

		if (!end->isVisted())
			return; // cannot reach end

		int min = 0;
		Vertex *next = end, *buffer = nullptr;
		path.push_front(next);
		while (next != start)
		{
			min = next->getLength();

			for (auto &x : *next)
			{
				if (x.isReachable() && min > x.getVertex()->getLength())
				{
					buffer = x.getVertex();
					min = x.getVertex()->getLength();
				}
			}

			path.push_front(buffer);
			next = buffer;
		}
	}

private:
	struct orderByAStar
	{
		// goal positions
		int row;
		int col;

		bool operator()(Vertex *lhs, Vertex *rhs)
		{
			return pow(col - rhs->getColumn(), 2) + pow(row - rhs->getRow(), 2) + rhs->getLength() < pow(col - lhs->getColumn(), 2) + pow(row - lhs->getRow(), 2) + lhs->getLength();
		}
	};

	int setDistance(Vertex *from, Vertex *to, int toLen = 1)
	{
		to->setLength(to->getLength() > from->getLength() + toLen ? from->getLength() + toLen : to->getLength());
		return to->getLength();
	}
};

#endif // !__PATH__
