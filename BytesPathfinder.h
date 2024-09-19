// Copyright by BytesOfProcrastination all rights reserved CC 2024

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include <vector>
#include "BytesPathfinder.generated.h"

UENUM(BlueprintType)
enum class EBytesGraphType : uint8
{
	Distance2D,
	Square,
	Hexagonal,
};

// ==== Section | Pathfinder Utility Classes/Structs ==== //

// Simple Node with G & H Cost, Parent ID and a Node ID for referencing itself and Objects outside the Pathfinder
USTRUCT(BlueprintType)
struct FBytesNode
{
	GENERATED_BODY()

	// Node ID for referencing it back to request
	UPROPERTY()
	int32 NodeID = -1;

	UPROPERTY()
	int32 ParentID = -1;
	
	// Cost it took to get here
	UPROPERTY()
	int32 GCost = 0;

	// Heuristic guess about distance to Target
	UPROPERTY()
	int32 HCost = 0;

	// Needed for Calculation of Heuristic
	UPROPERTY()
	FVector2D Location2D;

	// Index important for Heap
	UPROPERTY()
	int32 HeapIndex;

	// A* Only, combines GCost + HCost
	int32 FCost() const
	{
		return GCost + HCost;
	}
};

// A Non-Templated Heap for the Pathfinding
class FBytesPathfindingHeap
{
public:
	explicit FBytesPathfindingHeap(const int32 Capacity)
	{
		// Init Vector with Size
		Items.resize(Capacity);

		// Init Item Count
		Size = 0;
	}

	void Add(FBytesNode* Node)
	{
		Node->HeapIndex = Size;
		Items[Size] = Node;
		SortUp(Node);
		Size++;
	}

	FBytesNode* RemoveFirst()
	{
		auto First = Items[0];
		Size--;
		Items[0] = Items[Size];
		Items[0]->HeapIndex = 0;

		SortDown(Items[0]);
		return First;
	}

	// In a Heap for Pathfinding, we only need to sort up, as they only get changed, when they
	// become better
	void UpdateItem(FBytesNode* Node)
	{
		SortUp(Node);
	}

	bool IsNotEmpty() const
	{
		return Size > 0;
	}

	bool Contains(const FBytesNode* Node) const
	{
		for (auto Elem : Items)
		{
			if (Node == Elem)
			{
				return true;
			}
		}

		return false;
	}

	void LogHeap() const
	{
		for (int i = 0; i < Size; i++)
		{
			UE_LOG(LogTemp, Warning, TEXT("Heap Position: %d | Distance: %d | ID: %d"), i, Items[i]->FCost(), Items[i]->NodeID);
		}
	}
	
private:
	// Container 
	std::vector<FBytesNode*> Items;

	// Current Size
	int32 Size;

	// Parent Shortcut
	int GetParent (const int32 Index) const { return (Index - 1) / 2; }
	int GetLeftChild (const int32 Index) const { return Index * 2 + 1; }
	int GetRightChild (const int32 Index) const { return Index * 2 + 2; }

	// Sorting
	void SortUp(FBytesNode* Node) 
	{
		// Get Parent
		FBytesNode* Parent = Items[GetParent(Node->HeapIndex)];

		// When the Child has a higher priority than the parent, we swap them
		if (Node->FCost() < Parent->FCost() || Node->FCost() == Parent->FCost() && Node->HCost < Parent->HCost)
		{
			// Switch Positions
			Swap(Parent, Node);

			// Recursive calling until Node is high enough
			SortUp(Node);
		}
	}

	void SortDown(FBytesNode* Node)
	{
		while (true)
		{
			auto LIndex = GetLeftChild(Node->HeapIndex);
			auto RIndex = GetRightChild(Node->HeapIndex);
			auto SwapIndex = 0;
			
			if (LIndex < Size)
			{
				SwapIndex = LIndex;

				if(RIndex < Size)
				{
					// Check if we switch with left or right child
					if (Items[LIndex]->FCost() > Items[RIndex]->FCost() || Items[LIndex]->FCost() == Items[RIndex]->FCost() && Items[LIndex]->HCost > Items[RIndex]->HCost)
					{
						SwapIndex = RIndex;
					}
				}

				// Check if we swap at all
				auto ItemToSwap = Items[SwapIndex];
				if (Node->FCost() > ItemToSwap->FCost() || Node->FCost() == ItemToSwap->FCost() && Node->HCost > ItemToSwap->HCost)
				{
					Swap(Node, Items[SwapIndex]);
				}
				else
				{
					return;
				}
			}
			else
			{
				return;
			}
		}
	}

	void Swap(FBytesNode* X, FBytesNode* Y)
	{
		// switch vector positions
		Items[X->HeapIndex] = Y;
		Items[Y->HeapIndex] = X;

		int Temp = X->HeapIndex;

		X->HeapIndex = Y->HeapIndex;
		Y->HeapIndex = Temp;
	}
};

// A Single Edge, containing End and Weight
USTRUCT(BlueprintType)
struct BYTESHEXGRIDPLUGIN_API FBytesEdge
{
	GENERATED_BODY()

	// The End of this Edge Basically
	UPROPERTY()
	int32 NodeID;

	// Travel Cost/Weight... basically what gets added to GCost...
	UPROPERTY()
	int32 Weight;
};

// A Container of Edges, forms inside a TArray an Adjacency Matrix
USTRUCT()
struct FBytesEdges
{
	GENERATED_BODY()

	UPROPERTY()
	TArray<FBytesEdge> NeighbouringEdges;
};

// ==== The Graph itself we perform our Pathfinding on ==== //
USTRUCT(BlueprintType)
struct BYTESHEXGRIDPLUGIN_API FBytesGraph
{
	GENERATED_BODY()
	
	UPROPERTY()
	TArray<FBytesNode> Nodes;

	UPROPERTY()
	TArray<FBytesEdges> Edges;

	UPROPERTY()
	EBytesGraphType GraphType;
};

// ==== Section | Pathfinder BP Function Library ==== //
UCLASS(BlueprintType, DefaultToInstanced)
class BYTESHEXGRIDPLUGIN_API UBytesPathfinder : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

	/*
	 * Change Name...
	 *
	 * This Basically performs a Dijkstra on a Graph.
	 * After this has been performed, you can use the "GetPath()" Method
	 * to get a TArray<int32> which contains the NodeID's.
	 */
	UFUNCTION(BlueprintCallable, Category="Pathfinder|Pathfinding")
	static void FindPathsToNodes(UPARAM(ref) FBytesGraph& Graph, const int32 StartID);

	UFUNCTION()
	static void FindPath(FBytesGraph& Graph, const int32 StartID, const int32 TargetID);

	/*
	 * Only call after "Find Paths to Nodes
	 * Returns all NodeID's of reachable Nodes.
	 */
	UFUNCTION(BlueprintCallable, Category="Pathfinder|Pathfinding")
	static TArray<int32> GetNodesInRange(UPARAM(ref) FBytesGraph& Graph, const int32 MaxTravelCost);

	/*
	 * StartNodeID: The ID of the starting Node
	 * TargetNodeID: The ID of the targeted Node
	 * bRecalculate: Performs A* if true, else uses Graph as is
	 */
	UFUNCTION(BlueprintCallable, Category="Pathfinder|Pathfinding")
	static TArray<int32> GetPath(UPARAM(ref) FBytesGraph& Graph, const int32 StartNodeID, const int32 TargetNodeID, bool bRecalculate);

	/*
	 * Returns a new Graph
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinder|Graph")
	static FBytesGraph CreateGraph();

	/*
	 * Creates a new "FBytesNode" and a "FBytesEdges"
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinder|Graph")
	static int32 AddNode(UPARAM(ref) FBytesGraph& Graph, FVector2D Location2D);

	/*
	 * Creates a new Edge and adds them to their designated "FBytesEdges" 
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinder|Graph")
	static void AddOrSetEdge(UPARAM(ref) FBytesGraph& Graph, const int32 NodeAID, const int32 NodeBID, const int32 Weight);
	
private:
	// Linear Search Through an Array...
	UFUNCTION()
	static int32 FindNodeWithLowestGCost(const FBytesGraph& Graph, const TArray<int32>& Unvisited);

	// Heuristic Calculations for 
	UFUNCTION()
	static int32 CalcHeuristicDistance(const FBytesGraph& Graph, const int32 StartID, const int32 TargetID);

	UFUNCTION()
	static void InitNodes(FBytesGraph& Graph);
	
};

/*
 * ToDo:
 * - Add Different Versions of Dijkstar and A* for Square and Hex Grids (Because of Heuristic and )
 * - Remove BlueprintType
 */