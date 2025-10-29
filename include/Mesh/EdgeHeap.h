#pragma once

#include <vector>
#include <map>
#include <cassert>
using namespace std;


class SortableEdge
{
public:
	size_t edge_index;
	double edge_key;

	SortableEdge(size_t ei, double ek) : edge_index(ei), edge_key(ek) {}

	bool operator==(const SortableEdge& other) const
	{
		if ((this->edge_key == other.edge_key) &&
			(this->edge_index == other.edge_index))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool operator<(const SortableEdge& other) const
	{
		if (this->edge_key < other.edge_key)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool operator>(const SortableEdge& other) const
	{
		if (this->edge_key > other.edge_key)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
};

class EdgeHeap
{
private:
	size_t edgeAmt;
	vector<SortableEdge*> heap;
	map<size_t, size_t> edgeI2nodeI;

public:
	void changeEdgeInd(size_t oldEdgeInd, size_t newEdgeInd)
	{
		assert(edgeI2nodeI.find(newEdgeInd) == edgeI2nodeI.end());

		auto finder = edgeI2nodeI.find(oldEdgeInd);
		if (finder == edgeI2nodeI.end())
		{
			//puts("warning: updating nonexisted edge.");
			return;
		}

		heap[finder->second]->edge_index = newEdgeInd;
		edgeI2nodeI[newEdgeInd] = finder->second;
		edgeI2nodeI.erase(finder);
	}

	void insert(size_t edgeIndex, double key)
	{
		SortableEdge *edgeNodePtr = new SortableEdge(edgeIndex, key);
		heap.push_back(edgeNodePtr); edgeI2nodeI[edgeIndex] = heap.size() - 1;
		upHeap(heap.size() - 1);
	}

	void insert(SortableEdge* edgeNodePtr)
	{
		heap.push_back(edgeNodePtr); edgeI2nodeI[edgeNodePtr->edge_index] = heap.size() - 1;
		upHeap(heap.size() - 1);
	}

	void popTop(bool deleteNode = true)
	{
		assert(!heap.empty());
		edgeI2nodeI.erase(heap[0]->edge_index);
		if (deleteNode) { delete heap[0]; }
		if (heap.size() == 1)
		{
			heap.resize(0);
		}
		else
		{
			heap[0] = heap.back(); edgeI2nodeI[heap[0]->edge_index] = 0;
			heap.resize(heap.size() - 1);
			downHeap(0);
		}
	}

	void update(size_t edgeIndex, double key)
	{
		if (edgeI2nodeI.find(edgeIndex) == edgeI2nodeI.end())
		{
			//puts("warning: updating nonexisted edge.");
			return;
		}

		size_t heapPos = edgeI2nodeI[edgeIndex];
		heap[heapPos]->edge_key = key;
		repos(heapPos);
	}

	void remove(size_t edgeIndex)
	{
		if (edgeI2nodeI.find(edgeIndex) == edgeI2nodeI.end())
		{
			//puts("warning: removing nonexisted edge.");
			return;
		}

		size_t heapPos = edgeI2nodeI[edgeIndex];
		edgeI2nodeI.erase(edgeIndex);
		delete heap[heapPos];
		if (heapPos + 1 == heap.size())
		{
			heap.resize(heap.size() - 1);
		}
		else
		{
			heap[heapPos] = heap.back(); edgeI2nodeI[heap[heapPos]->edge_index] = heapPos;
			heap.resize(heap.size() - 1);
			repos(heapPos);
		}
	}


	SortableEdge *top(void) const { assert(!heap.empty());  return heap[0]; }
	bool empty(void) const { return heap.empty(); }
	size_t size(void) const { return heap.size(); }

	size_t getTop10(std::vector<size_t>& top10)
	{
		top10.resize(0);
		top10.reserve(10);
		std::vector<SortableEdge*> popped;
		popped.reserve(10);
		for (int i = 0; (i < 10) && !heap.empty(); i++)
		{
			popped.emplace_back(top());
			top10.emplace_back(top()->edge_index);
			popTop(false);
		}
		for (auto iter = popped.begin(); iter != popped.end(); iter++)
		{
			insert(*iter);
		}

		return top10.size();
	}


	~EdgeHeap(void)
	{
		for (auto i = heap.begin(); i != heap.end(); i++)
		{
			delete *i;
		}
	}

	void clear(void)
	{
		for (auto i = heap.begin(); i != heap.end(); i++)
		{
			delete *i;
		}
		heap.resize(0);
		edgeI2nodeI.clear();
	}


	void print(void) const
	{
		for (auto i = heap.begin(); i != heap.end(); i++)
		{
			if (*i)
			{
				printf("%d %le\n", (*i)->edge_index, (*i)->edge_key);
			}
		}
	}

	void check(void)
	{
		for (size_t i = 0; i < heap.size(); i++)
		{
			if (heap[i])
			{
				assert(edgeI2nodeI[heap[i]->edge_index] == i);
			}
		}
	}

private:
	void repos(size_t heapPos)
	{
		if (heapPos>0 && *heap[heapPos]>*heap[parent(heapPos)])
			upHeap(heapPos);
		else
			downHeap(heapPos);
	}

	void upHeap(size_t heapPos)
	{
		SortableEdge *moving = heap[heapPos];
		unsigned int index = heapPos;
		unsigned int p = parent(heapPos);

		while (index > 0 && *moving > *heap[p])
		{
			heap[index] = heap[p]; edgeI2nodeI[heap[index]->edge_index] = index;
			index = p;
			p = parent(p);
		}

		if (index != heapPos)
		{
			heap[index] = moving; edgeI2nodeI[heap[index]->edge_index] = index;
		}
	}

	void downHeap(size_t heapPos)
	{
		SortableEdge *moving = heap[heapPos];
		unsigned int index = heapPos;
		unsigned int l = left(heapPos);
		unsigned int r = right(heapPos);
		unsigned int largest;

		while (l<heap.size())
		{
			if (r < heap.size() && *heap[l] < *heap[r])
				largest = r;
			else
				largest = l;

			if (*moving < *heap[largest])
			{
				heap[index] = heap[largest]; edgeI2nodeI[heap[index]->edge_index] = index;
				index = largest;
				l = left(index);
				r = right(index);
			}
			else
				break;
		}

		if (index != heapPos)
		{
			heap[index] = moving; edgeI2nodeI[heap[index]->edge_index] = index;
		}
	}

	size_t parent(size_t i) const { return (i - 1) / 2; }
	size_t left(size_t i) const { return 2 * i + 1; }
	size_t right(size_t i) const { return 2 * i + 2; }
};