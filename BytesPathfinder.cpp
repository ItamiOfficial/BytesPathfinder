// Copyright by BytesOfProcrastination all rights reserved CC 2024

#include "Pathfinding/BytesPathfinder.h"

constexpr int32 INITIAL_DISTANCE = 2000000;

void UBytesPathfinder::FindPathsToNodes(FBytesGraph& Graph, const int32 StartID)
{
	/* Declare "Unvisited" TSet which contains ID's of unvisited Nodes
	*  easier to remove Nodes, but should be a sorted queue ideally,
	*  so we do not have to iterate over the hole Set to find
	*  the Node with the lowest GCost
	*/

	// This has to be a reference. When Nodes in the Grid get changed, they wont get changed inside the MinHeap
	// And because they are not changed, this thing does not work.
	FBytesPathfindingHeap Unvisited = FBytesPathfindingHeap(Graph.Nodes.Num());

	// Set Starting Node to 0
	Graph.Nodes[StartID].GCost = 0;

	// Add Nodes to Unvisited
	for (FBytesNode& Node : Graph.Nodes)
	{
		// Add Node to Unvisited Array
		Unvisited.Add(&Node);
	}

	// Remove Late Print
	// Unvisited.LogHeap();
	// Unvisited.UpdateItem(&Graph.Nodes[StartNodeID]);

	// As long as there are unvisited Tiles we keep searching
	while (Unvisited.IsNotEmpty())
	{
		// Find closest and  remove current
		const auto Node = Unvisited.RemoveFirst();
		
		// Was used before: "FindNodeWithLowestGCost(Graph, Unvisited);"
		// Was Used before: "Unvisited.RemoveSingle(CurrentNodeID);"

		for (const auto& NeighbourEdge : Graph.Edges[Node->NodeID].NeighbouringEdges)
		{
			// Calculate GCost to Neighbour, which is CurrentNode's GCost + the Edge Weight
			const int32 Distance = Graph.Nodes[Node->NodeID].GCost + NeighbourEdge.Weight;

			// if Distance is closer, update Parent and GCost
			if (Distance < Graph.Nodes[NeighbourEdge.NodeID].GCost)
			{
				Graph.Nodes[NeighbourEdge.NodeID].GCost = Distance;
				Graph.Nodes[NeighbourEdge.NodeID].ParentID = Node->NodeID;

				// Update Entry
				Unvisited.UpdateItem(&Graph.Nodes[NeighbourEdge.NodeID]);
			}
		}
		
		// Remove Late Print
		// Unvisited.LogHeap();
	}
}

void UBytesPathfinder::FindPath(FBytesGraph& Graph, const int32 StartID, const int32 TargetID)
{
	// Inits Nodes
	InitNodes(Graph);

	// Declare "Open List" and "Closed List"
	FBytesPathfindingHeap OpenSet = FBytesPathfindingHeap(Graph.Nodes.Num());
	TSet<int32> ClosedSet;

	// Add First Node
	Graph.Nodes[StartID].GCost = 0;
	Graph.Nodes[StartID].HCost = CalcHeuristicDistance(Graph, StartID, TargetID);
	OpenSet.Add(&Graph.Nodes[StartID]);

	while(OpenSet.IsNotEmpty())
	{
		// Get Current Node
		auto CurrentNode = OpenSet.RemoveFirst();

		// Add Current Node to Closed List
		ClosedSet.Add(CurrentNode->NodeID);

		// Check if Current is Target
		if (CurrentNode->NodeID == TargetID)
		{
			UE_LOG(LogTemp, Warning, TEXT("Pathfinding: Path Found"));
			return;
		}
		
		// Iterate over Neighbours
		for (const auto& Edge : Graph.Edges[CurrentNode->NodeID].NeighbouringEdges)
		{
			// Neighbour
			const auto Neighbour = &Graph.Nodes[Edge.NodeID];
			
			// if Closed List Contains Neighbour, return
			if (ClosedSet.Contains(Neighbour->NodeID))
			{
				continue;
			}

			// Calculate Cost to Neighbour
			int32 MovementCost = CurrentNode->GCost + Edge.Weight;

			// if Neighbour is in OpenList AND has higher GCost return
			if (OpenSet.Contains(Neighbour) &&  Neighbour->GCost < MovementCost)
			{
				continue;
			}

			// Set Current Node Parent
			Neighbour->ParentID = CurrentNode->NodeID;

			// Set New Movement Cost
			Neighbour->GCost = MovementCost;
			Neighbour->HCost = CalcHeuristicDistance(Graph, Neighbour->NodeID, TargetID);

			if (OpenSet.Contains(Neighbour))
			{
				OpenSet.UpdateItem(Neighbour);
			} else
			{
				OpenSet.Add(Neighbour);
			}
		}
		
	}

	UE_LOG(LogTemp, Warning, TEXT("Pathfinding: No Path Found"));
}

TArray<int32> UBytesPathfinder::GetNodesInRange(FBytesGraph& Graph, const int32 MaxTravelCost)
{
	TArray<int32> ReturnArray;

	for (const auto& Node : Graph.Nodes)
	{
		if(Node.GCost <= MaxTravelCost) {
			ReturnArray.Add(Node.NodeID);
		}
	}

	return ReturnArray;
}

TArray<int32> UBytesPathfinder::GetPath(FBytesGraph& Graph, const int32 StartNodeID, const int32 TargetNodeID, const bool bRecalculate)
{
	TArray<int32> Path;

	// Check if Indices are valid
	if (!Graph.Nodes.IsValidIndex(StartNodeID) || !Graph.Nodes.IsValidIndex(TargetNodeID))
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding: Invalid Node ID's. Out of Range"));
		return Path;
	}
	
	// Check if Target Node has ever been reached
	if (Graph.Nodes[TargetNodeID].ParentID == -1)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding: Target Node has never been Reached"));
		return Path;
	}

	// if we should recalculate the path, we use A*
	// It Could be that we had a Dijkstra before and cached it, so we dont need to recalc
	if (bRecalculate)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding: Path Found Using A*. Heuristics still SUCK!"));

		FindPath(Graph, StartNodeID, TargetNodeID);
	}
	
	// Retrace Path
	int32 CurrentNodeID = TargetNodeID;

	// Iterate over each parent of current node until we reached Start Node
	// We Iterate from Target -> Start
	while(CurrentNodeID != StartNodeID)
	{
		// Add Current Node to Path
		Path.Add(CurrentNodeID);

		// Set Current Node to Parent ID of current Node
		CurrentNodeID = Graph.Nodes[CurrentNodeID].ParentID;
	}

	// Reverse TArray
	Algo::Reverse(Path);
	
	return Path;
}

FBytesGraph UBytesPathfinder::CreateGraph()
{
	return FBytesGraph();
}

int32 UBytesPathfinder::AddNode(FBytesGraph& Graph, FVector2D Location2D)
{
	// ==== Sub Section | Node ==== //
	
	// Create new Node
	FBytesNode NewNode;

	// Set NodeID to MaxIndex
	NewNode.NodeID = Graph.Nodes.Num();

	// Add Node to Graph
	Graph.Nodes.Add(NewNode);

	// Set Location 2D
	NewNode.Location2D = Location2D;

	// ==== Sub Section | Edges ==== //

	// Create new Edges (Container for multiple "FBytesEdge" Structs)
	FBytesEdges NewEdges;

	// Add Edges Container, right after Node gets Created
	Graph.Edges.Add(NewEdges);
	
	// Return ID
	return NewNode.NodeID;
}

void UBytesPathfinder::AddOrSetEdge(FBytesGraph& Graph, const int32 NodeAID, const int32 NodeBID, const int32 Weight)
	{
	// Check if the Nodes even Exist
	if (!Graph.Nodes.IsValidIndex(NodeAID) || !Graph.Nodes.IsValidIndex(NodeBID))
	{
		UE_LOG(LogTemp, Warning, TEXT("Error, one of the Vertices is not in Graph"));
		return;
	}

	// Checks if the Edges already exist, and only overwrite their Weight
	if (Graph.Edges[NodeAID].NeighbouringEdges.ContainsByPredicate([NodeBID] (const FBytesEdge Edge)
	{
		return Edge.NodeID == NodeBID;
	}))
	{
		UE_LOG(LogTemp, Warning, TEXT("Edge Weight gets overridden"));

		// Change Edge A
		for (auto& Edge : Graph.Edges[NodeAID].NeighbouringEdges)
		{
			if (Edge.NodeID == NodeBID)
			{
				Edge.Weight = Weight;
			}
		}

		// Change Edge B
		for (auto& Edge : Graph.Edges[NodeBID].NeighbouringEdges)
		{
			if (Edge.NodeID == NodeAID)
			{
				Edge.Weight = Weight;
			}
		}

		// Return Early
		return;
	}

	
	// Create Structs
	FBytesEdge EdgeAB;
	FBytesEdge EdgeBA;

	// Set Edge A -> B
	EdgeAB.Weight = Weight;
	EdgeAB.NodeID = NodeBID;

	// Set Edge B -> A
	EdgeBA.Weight = Weight;
	EdgeBA.NodeID = NodeAID;

	// Sets Elements
	Graph.Edges[NodeAID].NeighbouringEdges.Add(EdgeAB);
	Graph.Edges[NodeBID].NeighbouringEdges.Add(EdgeBA);

}

int32 UBytesPathfinder::FindNodeWithLowestGCost(const FBytesGraph& Graph, const TArray<int32>& Unvisited)
{
	// Closest is random, but negative one will never be set, so there cant be any problems
	int32 Closest = Unvisited[0];
	// the "- 1" is very important, as it will loop forever if there are no edges to a Point.
	// So there is "normally" no problem, but for my debugging adventures this sucks...
	int32 CurrentCost = Graph.Nodes[Closest].GCost;

	for (const auto NodeID : Unvisited)
	{
		// Get Node
		const FBytesNode& Node = Graph.Nodes[NodeID];

		// Check if Node has lower than current
		if (Node.GCost < CurrentCost)
		{
			Closest = NodeID;
			CurrentCost = Node.GCost;
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Cost: %d"), CurrentCost);
	
	return Closest;
}

int32 UBytesPathfinder::CalcHeuristicDistance(const FBytesGraph& Graph, const int32 StartID, const int32 TargetID)
{
	auto NodeA = &Graph.Nodes[StartID];
	auto NodeB = &Graph.Nodes[TargetID];

	return FMath::FloorToInt32(FVector2D::Distance(NodeA->Location2D, NodeB->Location2D));
}

void UBytesPathfinder::InitNodes(FBytesGraph& Graph)
{
	// Init Node
	for (FBytesNode& Node : Graph.Nodes)
	{
		// Set GCosCostt to Max
		Node.GCost = INITIAL_DISTANCE;
		Node.HCost = 0;

		// Set Parent to "none" represented by -1.
		// This is Important for checking if there even is a path to Target!
		Node.ParentID = -1;
	}
}
