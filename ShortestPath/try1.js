const createGraph = (fullCoordinates, nodeCoordinates, maxDistance) => {
  const graph = new Map();

  // Create nodes for each node coordinate
  nodeCoordinates.forEach((coordinate, i) => {
    const nodeLabel = `Node ${i}`;
    graph.set(nodeLabel, coordinate);
  });

  // Connect nodes with edges based on distances
  for (let i = 0; i < nodeCoordinates.length - 1; i++) {
    const currentCoordinate = nodeCoordinates[i];
    const currentNodeLabel = `Node ${i}`;

    for (let j = i + 1; j < nodeCoordinates.length; j++) {
      const nextCoordinate = nodeCoordinates[j];
      const nextNodeLabel = `Node ${j}`;

      const distance = Math.hypot(nextCoordinate[0] - currentCoordinate[0], nextCoordinate[1] - currentCoordinate[1]);
      if (distance <= maxDistance) {
        if (!graph.has(currentNodeLabel)) {
          graph.set(currentNodeLabel, currentCoordinate);
        }
        if (!graph.has(nextNodeLabel)) {
          graph.set(nextNodeLabel, nextCoordinate);
        }
      }
    }
  }

  return graph;
};

// Example usage
const fullCoordinates = [[0, 0], [1, 0], [2, 1], [3, 1], [3, 2], [4, 2]];
const nodeCoordinates = [[0, 0], [2, 1], [4, 2]];
const maxDistance = 0.5;

const graph = createGraph(fullCoordinates, nodeCoordinates, maxDistance);
console.log(graph);
