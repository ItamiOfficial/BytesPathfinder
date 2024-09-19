Hi

This Repository The Progress of my Pathfinding Project. The Current Version features a implementation of A* and Dijkstra using a Min Heap and Stores Edges (Neighbour, Weight) seperatly (for bidirectional Graphs).

The Codebase is going to change drastically however, as i will add support for Hexagonal and Square Grids (Kinda annoying to implement in Unreal and exposing to Blueprint).
Grid like Pathfinding has some Optimisations you can do, like getting rid of the Edges and store Neighbours Functionally, as well as the Heuristics become Way Easier.

I Currently Store Grids as a Struct, so we can access/modify them later. I'm Still not sure if i want to move them Inside a UObject/UActorComponent to make them accessable for BP Events/Delegates.

Feel free to leave Feedback. 

This Thing is supposed to get released for Free once its finished. It also is a neccessary componen of my actual bigger Goal: A Turn Based Game Framework with support for Node/Rect/Hex Maps in order to make Games
like RISK / Pokemon Mystery Dungeon / CRPG's / 4X / etc...

Have a nice day!
