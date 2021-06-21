#ifndef __MAZE__
#define __MAZE__

#include "Vertex.h"
#include "IMazeReader.h"
#include <list>

using std::list;

/*
	Maze Data Structure
*/

class Maze
{

public:
	/*
		To tranverse list of Vertexs
	*/
	using const_iterator = list<Vertex *>::const_iterator;
	using iterator = list<Vertex *>::iterator;

	/*
		Default Constructor
	*/
	Maze()
	{
		goal[0] = nullptr;
		goal[1] = nullptr;
	}

	/*
		Destructor
	*/
	~Maze()
	{
		clear();
	}

	/*
		Set size of maze
	*/
	void setSize(size_t row, size_t col)
	{
		_row = row;
		_col = col;
	}

	/*
		Getter for column
	*/
	size_t getColumn()
	{
		return _col;
	}

	/*
		Getter for row
	*/
	size_t getRow()
	{
		return _row;
	}

	/*
		push_back into list of vertex
	*/
	void push_back(Vertex *v)
	{
		_maze.push_back(v);
	}

	/*
		Iterators 
	*/
	iterator begin()
	{
		return _maze.begin();
	}

	const_iterator begin() const
	{
		return _maze.cbegin();
	}

	iterator end()
	{
		return _maze.end();
	}

	const_iterator end() const
	{
		return _maze.cend();
	}

	/*
		Reset of list of vertex
	*/
	void reset()
	{
		for (auto &x : _maze)
			x->reset();
	}

	/*
		return size of list
	*/
	int listSize() const
	{
		return _maze.size();
	}

	/*
		clear list
	*/
	void clear()
	{
		for (auto &x : *this)
			delete x;
		_maze.clear();
		goal[0] = nullptr;
		goal[1] = nullptr;
	}

	/*
		return true if maze is empty
	*/
	bool empty() const
	{
		return _maze.empty();
	}

	/*
		IMazeReader implement the following functions
		void setSize(size_t & row, size_t & col);
			Needed to set deminsions of the maze
		void isVertex(int y, int x, bool & top, bool & down, bool & left, bool & right, bool & isGoal);
			Needed to get configration of a cell at row y and col x
			top down left right -> true if you can move in the direction at that cell
			isGoal -> true if cell represent start or end point
	*/
	void setMaze(IMazeReader &mb)
	{
		clear();
		mb.setSize(_row, _col);
		int isGoal = 0;

		// for count
		int *top_count = new int[getColumn()]();
		Vertex **top_next = new Vertex *[getColumn()];
		list<Vertex *> reduntant; // to store reduntant vertex to later delete

		// for paths
		bool path_top = false, path_down = false, path_right = false, path_left = false;
		for (int line = 0; line < getRow(); ++line)
		{
			int right_count = 0;
			Vertex *left_next = nullptr;

			for (int index = 0; index < getColumn(); ++index)
			{
				int count = 0;
				path_top = false;
				path_down = false;
				path_right = false;
				path_left = false;

				// index=x and line=y;
				// get directions
				mb.isVertex(line, index, path_top, path_down, path_left, path_right, isGoal);

				// for number of open directions
				if (path_top)
					++count;
				if (path_down)
					++count;
				if (path_left)
					++count;
				if (path_right)
					++count;

				// Vertex requires a decision for...
				bool left_right = bool(path_left != path_right);				   // ...left or right
				bool up_down = bool(path_top != path_down);						   // ...up or down
				bool all = bool(path_top && path_down && path_left && path_right); //... all options are open -> could or could not be important

				// create new Vertex
				if (left_right || up_down || all || 0 < isGoal)
				{
					Vertex *current = new Vertex;
					current->setPosition(line, index);

					if (path_top)
					{ // top
						current->setTop(top_next[index], top_count[index] + 1);
						top_next[index]->setDown(current, top_count[index] + 1);
					}

					if (path_left)
					{ // left
						current->setLeft(left_next, right_count + 1);
						left_next->setRight(current, right_count + 1);
					}

					// set pointer
					top_next[index] = left_next = current;

					// reset count
					top_count[index] = 0;
					right_count = 0;

					if (isGoal == 1)
					{
						goal[0] = current;
						isGoal = 0;
					}
					else if (isGoal == 2)
					{
						goal[1] = current;
						isGoal = 0;
					}
					else if (isGoal == 3)
					{
						goal[0] = current;
						goal[1] = current;
						isGoal = 0;
					}

					push_back(current);
				}
				else
				{ // increment counter
					++right_count;
					++top_count[index];
				}
			}
		}

		// free memory
		for (int i = 0; i < getColumn(); ++i)
		{
			top_next[i] = nullptr;
		}
		delete[] top_next;

		if (!maze_begin())
			addToGoal(*begin());
		if (!maze_end())
			addToGoal(*--end());
	}

	/*
		Add vertex to goal
	*/
	void addToGoal(Vertex *v)
	{
		if (!goal[0])
			goal[0] = v;
		else
			goal[1] = v;
	}
	/*
		return start position
	*/
	Vertex *maze_begin()
	{
		return goal[0];
	}

	/*
		return end position
	*/
	Vertex *maze_end()
	{
		return goal[1];
	}

private:
	list<Vertex *> _maze; // to hold Vertex

	// size of maze
	size_t _row = 0;
	size_t _col = 0;

	Vertex *goal[2]; // start and end positions
};

#endif // !__MAZE__