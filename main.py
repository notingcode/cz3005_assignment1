import json
import math
import heapq as hq
# import time

class Graph:
  def __init__(self, nodes, adjNodes, costDict, distDict):
    self.nodes = nodes
    self.numNodes = len(nodes)
    self.adjNodes = adjNodes
    self.cost = costDict
    self.dist = distDict

  def getNode(self, nodeIdx):
    return self.nodes.get(str(nodeIdx))

  def getNeighbors(self, nodeIdx):
    return self.adjNodes[nodeIdx]

  def getEdgeDist(self, nodeA_idx, nodeB_idx):
    return self.dist[nodeA_idx+','+nodeB_idx]

  def getEdgeCost(self, nodeA_idx, nodeB_idx):
    return self.cost[nodeA_idx+','+nodeB_idx]

  def calcHeuristic(self, nodeA_idx, nodeB_idx):
    nodeA = self.getNode(nodeA_idx)
    nodeB = self.getNode(nodeB_idx)

    return abs(nodeA.x - nodeB.x) + abs(nodeA.y - nodeB.y)

class Node_aStar:
  def __init__(self, idx, xCoord, yCoord):
    self.idx = idx
    self.x = xCoord
    self.y = yCoord
    self.g = math.inf
    self.f = math.inf
    self.cost = math.inf
    self.parent = None
    self.inHeap = False

  def __lt__(self, other):
    return self.f < other.f

class Node_constraint:
  def __init__(self, idx, xCoord, yCoord):
    self.idx = idx
    self.g = math.inf
    self.cost = math.inf
    self.parent = None
    self.visited = False
    self.inHeap = False

  def __lt__(self, other):
    return self.g < other.g

class Node:
  def __init__(self, idx, xCoord, yCoord):
    self.idx = idx
    self.g = math.inf
    self.parent = None
    self.visited = False
    self.cost = math.inf
    self.inHeap = False

  def __lt__(self, other):
    return self.g < other.g

def aStarSearch(graph, srcIdx, destIdx, costCap):
  heap = []

  srcNode = graph.getNode(srcIdx)
  srcNode.g = 0
  srcNode.cost = 0
  h = graph.calcHeuristic(srcNode.idx, destIdx)
  srcNode.f = srcNode.g + h
  isDistChange = False

  hq.heappush(heap, srcNode)

  while(heap):
    curr_node = hq.heappop(heap)
    curr_node.inHeap = False
    if(curr_node.idx == destIdx):
      return curr_node
    neighbors = graph.getNeighbors(curr_node.idx)
    for neighbor_idx in neighbors:
      node = graph.getNode(neighbor_idx)
      newCost = curr_node.cost + graph.getEdgeCost(curr_node.idx, neighbor_idx)
      newG = curr_node.g + graph.getEdgeDist(curr_node.idx, neighbor_idx)
      h = graph.calcHeuristic(node.idx, destIdx)
      newF = newG + h
      if(newCost <= costCap and node.f > newF):
        node.parent = curr_node.idx
        node.g = newG
        node.f = newF
        node.cost = newCost
        if(node.inHeap is True):
          isDistChange = True
        else:
          node.inHeap = True
          hq.heappush(heap, node)
    if(isDistChange is True): # If a node in heap has changes in it's attributes,
      hq.heapify(heap)        # heap must be sorted.
      isDistChange = False
          
  return None

def ucs_constraint(graph, srcIdx, destIdx, costCap):
  heap = []

  srcNode = graph.getNode(srcIdx)
  srcNode.g = 0
  srcNode.cost = 0
  isDistChange = False

  hq.heappush(heap, srcNode)

  while(heap):
    curr_node = hq.heappop(heap)
    curr_node.visited = True
    if(curr_node.idx == destIdx):
      return curr_node
    neighbors = graph.getNeighbors(curr_node.idx)
    for neighbor_idx in neighbors:
      node = graph.getNode(neighbor_idx)
      if(node.visited is False):
        newCost = curr_node.cost + graph.getEdgeCost(curr_node.idx, neighbor_idx)
        newG = curr_node.g + graph.getEdgeDist(curr_node.idx, neighbor_idx)
        if(newCost <= costCap and node.g > newG):
          node.parent = curr_node.idx
          node.g = newG
          node.cost = newCost
          if(node.inHeap is True):
            isDistChange = True
          else:
            node.inHeap = True
            hq.heappush(heap, node)
    if(isDistChange is True):
      hq.heapify(heap)
      isDistChange = False
          
  return None

def ucs(graph, srcIdx, destIdx):
  heap = []

  srcNode = graph.getNode(srcIdx)
  srcNode.g = 0
  srcNode.cost = 0
  isDistChange = False

  hq.heappush(heap, srcNode)

  while(heap):
    curr_node = hq.heappop(heap)
    curr_node.visited = True
    if(curr_node.idx == destIdx):
      return curr_node
    neighbors = graph.getNeighbors(curr_node.idx)
    for neighbor_idx in neighbors:
      node = graph.getNode(neighbor_idx)
      if(node.visited is False):
        newG = curr_node.g + graph.getEdgeDist(curr_node.idx, neighbor_idx)
        newCost = curr_node.cost + graph.getEdgeCost(curr_node.idx, neighbor_idx)
        if(node.g > newG):
          node.parent = curr_node.idx
          node.g = newG
          node.cost = newCost
          if(node.inHeap is True):
            isDistChange = True
          else:
            node.inHeap = True
            hq.heappush(heap, node)
    if(isDistChange is True):
      hq.heapify(heap)
      isDistChange = False
          
  return None

def printResult(graph, destNode, srcIdx):
  order = []
  node = destNode

  if(node is not None):
    while(node.parent is not None):
      order.append(node.idx)
      node = graph.getNode(node.parent)

    print(f"Shortest path: {srcIdx}", end='')
    for idx in order[::-1]:
      print(f"->{idx}", end='')

    destNode = graph.getNode(destNode.idx)
    print(f"\nShortest distance: {destNode.g}")

    print(f"Total energy cost: {destNode.cost}")
  else:
    print("Search Path Failed")

def main():
  with open('G.json', 'r') as G:
    adj_nodesDict = json.load(G)
  with open('Coord.json', 'r') as Coord:
    coordDict = json.load(Coord)
  with open('Dist.json', 'r') as Dist:
    distDict = json.load(Dist)
  with open('Cost.json', 'r') as Cost:
    costDict = json.load(Cost)

  srcIdx = '1'
  destIdx = '50'
  costCap = 287932

  nodeDict = dict()
  for idx, coord in coordDict.items():
    nodeDict[idx] = Node(idx, coord[0], coord[1])

  graph = Graph(nodeDict, adj_nodesDict, costDict, distDict)

  destNode = ucs(graph, srcIdx, destIdx)

  print("Task 1: Uniform Cost Search")
  printResult(graph, destNode, srcIdx)
  
  nodeDict = dict()
  for idx, coord in coordDict.items():
    nodeDict[idx] = Node_constraint(idx, coord[0], coord[1])

  graph = Graph(nodeDict, adj_nodesDict, costDict, distDict)

  destNode = ucs_constraint(graph, srcIdx, destIdx, costCap)

  print(f"\nTask 2: Uniform Cost Search with Constraint")
  printResult(graph, destNode, srcIdx)

  nodeDict = dict()
  for idx, coord in coordDict.items():
    nodeDict[idx] = Node_aStar(idx, coord[0], coord[1])

  graph = Graph(nodeDict, adj_nodesDict, costDict, distDict)

  destNode = aStarSearch(graph, srcIdx, destIdx, costCap)

  print(f"\nTask 3: A* Search")
  printResult(graph, destNode, srcIdx)

main()