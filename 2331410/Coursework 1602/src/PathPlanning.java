import java.util.Scanner;

public class PathPlanning {
    // A* algorithm for path planning
    public static Node[] findPath(Grid grid, RobotState start, RobotState goal) {
        CustomPriorityQueue openSet = new CustomPriorityQueue(grid.getNumRows() * grid.getNumCols());
        CustomHashSet closedSet = new CustomHashSet(grid.getNumRows() * grid.getNumCols());
        CustomHashMap gScoreMap = new CustomHashMap(grid.getNumRows() * grid.getNumCols());
        CustomHashMap cameFrom = new CustomHashMap(grid.getNumRows() * grid.getNumCols());

        Node startNode = new Node(start.getRow(), start.getCol(), start.getOrientation(), 0, calculateHeuristic(start, goal), null);
        openSet.offer(startNode);
        gScoreMap.put(startNode, startNode);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            if (current.getRow() == goal.getRow() && current.getCol() == goal.getCol()
                    && current.getOrientation() == goal.getOrientation()) {
                return reconstructPath(current, cameFrom);
            }

            closedSet.add(current);

            for (Node neighbor : getNeighbors(grid, current)) {
                int tentativeGScore = current.getFScore() + 1; // Assuming uniform cost

                if (!grid.isValidCell(neighbor.getRow(), neighbor.getCol()) || closedSet.containsNode(neighbor)) {
                    continue;
                }

                Node existingNeighbor = gScoreMap.get(neighbor);
                if (existingNeighbor == null || tentativeGScore < existingNeighbor.getFScore()) {
                    cameFrom.put(neighbor, current);

                    if (existingNeighbor != null) {
                        existingNeighbor.setHScore(calculateHeuristic(new RobotState(neighbor.getRow(), neighbor.getCol()), goal));
                    } else {
                        neighbor.setHScore(calculateHeuristic(new RobotState(neighbor.getRow(), neighbor.getCol()), goal));
                    }

                    gScoreMap.put(neighbor, new Node(neighbor.getRow(), neighbor.getCol(), neighbor.getOrientation(),
                            tentativeGScore, neighbor.getHScore(), current));
                    openSet.offer(neighbor);
                }
            }
        }
        return new Node[0]; // No path found
    }

    private static int calculateHeuristic(RobotState current, RobotState goal) {
        return Math.abs(current.getRow() - goal.getRow()) + Math.abs(current.getCol() - goal.getCol());
    }

    // To get neighboring cells
    private static Node[] getNeighbors(Grid grid, Node node) {
        int[][] directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}}; // Up, Left, Down, Right
        Node[] neighbors = new Node[4];
        int count = 0;

        for (int[] dir : directions) {
            int newRow = node.getRow() + dir[0];
            int newCol = node.getCol() + dir[1];

            // Combine the condition here
            if (grid.isValidCell(newRow, newCol)) {
                neighbors[count++] = new Node(newRow, newCol, 0, 0, null);
            }
        }

        Node[] actualNeighbors = new Node[count];
        System.arraycopy(neighbors, 0, actualNeighbors, 0, count);
        return actualNeighbors;
    }

    // To reconstruct the path from goal to start
    private static Node[] reconstructPath(Node goalNode, CustomHashMap cameFrom) {
        java.util.ArrayList<Node> path = new java.util.ArrayList<>();
        Node current = goalNode;

        while (current != null) {
            path.add(current);
            current = cameFrom.get(current);
        }

        Node[] result = new Node[path.size()];
        for (int i = 0; i < path.size(); i++) {
            result[i] = path.get(i);
        }
        return result;
    }

    public static void printGrid(Grid grid, RobotState start, RobotState goal, Node[] path) {
        int numRows = grid.getNumRows();
        int numCols = grid.getNumCols();

        System.out.println("Grid:");
        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numCols; j++) {
                if (i == start.getRow() && j == start.getCol()) {
                    System.out.print("S ");
                } else if (i == goal.getRow() && j == goal.getCol()) {
                    System.out.print("G ");
                } else if (grid.isValidCell(i, j)) {
                    boolean isPathNode = false;
                    for (Node node : path) {
                        if (node != null && node.getRow() == i && node.getCol() == j) {
                            System.out.print("* ");
                            isPathNode = true;
                            break;
                        }
                    }
                    if (!isPathNode) {
                        System.out.print(". ");
                    }
                } else {
                    System.out.print("? ");
                }
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        int numRows, numCols;
        double obstacleDensity;

        numRows = getInput(scanner, "rows");
        numCols = getInput(scanner, "columns");
        obstacleDensity = getDensity(scanner);

        Grid userGrid = new Grid(numRows, numCols);
        userGrid.randomlyPlaceObstacles(obstacleDensity);

        RobotState start = new RobotState(0, 0);
        RobotState goal = new RobotState(numRows - 1, numCols - 1);

        Node[] path = findPath(userGrid, start, goal);

        printGrid(userGrid, start, goal, path);

        if (path.length > 0) {
            System.out.println("Optimal path found:");
            for (Node node : path) {
                if (node != null) {
                    System.out.println("(" + node.getRow() + ", " + node.getCol() + ")");
                }
            }
        } else {
            System.out.println("No path found.");
        }
    }

    private static int getInput(Scanner scanner, String message) {
        int input;
        do {
            System.out.print("Enter the number of " + message + " (maximum 50): ");
            input = scanner.nextInt();
            if (input <= 0 || input > 50) {
                System.out.println("Please enter a number between 1 and 50.");
            }
        } while (input <= 0 || input > 50);
        return input;
    }

    private static double getDensity(Scanner scanner) {
        double density;
        do {
            System.out.print("Enter the density of obstacles (0-100): ");
            density = scanner.nextDouble();
            if (density < 0 || density > 100) {
                System.out.println("Please enter a number between 0 and 100.");
            }
        } while (density < 0 || density > 100);
        return density / 100.0;
    }
}

class Grid {
    private int[][] grid;
    private int numRows;
    private int numCols;

    public Grid(int numRows, int numCols) {
        this.numRows = numRows;
        this.numCols = numCols;
        this.grid = new int[numRows][numCols];
    }

    public int getNumRows() { return numRows; }

    public int getNumCols() { return numCols; }

    public boolean isValidCell(int row, int col) {
        return row >= 0 && row < numRows && col >= 0 && col < numCols && grid[row][col] == 0;
    }

    public void randomlyPlaceObstacles(double obstaclePercentage) {
        java.util.Random random = new java.util.Random();
        int totalCells = numRows * numCols;
        int obstacleCount = (int) (totalCells * obstaclePercentage);
        int placedObstacles = 0;

        while (placedObstacles < obstacleCount) {
            int row = random.nextInt(numRows);
            int col = random.nextInt(numCols);
            if (grid[row][col] == 0) {
                grid[row][col] = 1; // Place obstacle
                placedObstacles++;
            }
        }
    }
}

class RobotState {
    private int row;
    private int col;
    private int orientation; // Orientation of the robot (0 for North, 1 for East, 2 for South, 3 for West)

    public RobotState(int row, int col) {
        this.row = row;
        this.col = col;
        this.orientation = 0; // Default orientation
    }

    public int getRow() { return row; }
    public int getCol() { return col; }
    public int getOrientation() { return orientation; }
}

class Node {
    private int row;
    private int col;
    private int orientation;
    private int gScore; // Cost from start to current node
    private int hScore; // Heuristic (estimated) cost from current node to goal
    private Node parent;
    private Node next; // Next node in case of collision
    public int getHScore() { return hScore; }

    public Node(int row, int col, int gScore, int hScore, Node parent) {
        this.row = row;
        this.col = col;
        this.orientation = 0; // Default orientation
        this.gScore = gScore;
        this.hScore = hScore;
        this.parent = parent;
        this.next = null;
    }

    public Node(int row, int col, int orientation, int gScore, int hScore, Node parent) {
        this.row = row;
        this.col = col;
        this.orientation = orientation;
        this.gScore = gScore;
        this.hScore = hScore;
        this.parent = parent;
        this.next = null;
    }

    public int getRow() { return row; }

    public int getCol() { return col; }

    public int getOrientation() { return orientation; }
    public int getFScore() { return gScore + hScore; }

    public void setHScore(int hScore) {
        this.hScore = hScore;
    }
}

class CustomPriorityQueue {
    private Node[] queue;
    private int size;

    public CustomPriorityQueue(int capacity) {
        this.queue = new Node[capacity];
        this.size = 0;
    }

    public void offer(Node node) {
        if (size == queue.length) {
            // Resize the array if it's full
            Node[] newQueue = new Node[queue.length * 2];
            System.arraycopy(queue, 0, newQueue, 0, queue.length);
            queue = newQueue;
        }
        queue[size++] = node;
        java.util.Arrays.sort(queue, 0, size, (n1, n2) -> Integer.compare(n1.getFScore(), n2.getFScore()));
    }

    public Node poll() {
        if (!isEmpty()) {
            Node min = queue[0];
            System.arraycopy(queue, 1, queue, 0, --size);
            return min;
        }
        return null;
    }

    public boolean isEmpty() { return size == 0; }

    public boolean contains(Node node) {
        for (int i = 0; i < size; i++) {
            if (queue[i] == node) {
                return true;
            }
        }
        return false;
    }
}

class CustomHashSet {
    private Node[] set;
    private int capacity;
    private int size;

    private static final int RESIZE_INCREMENT = 10;

    public CustomHashSet(int capacity) {
        this.capacity = capacity;
        this.set = new Node[capacity];
        this.size = 0;
    }

    public void add(Node node) {
        // Contain only unique elements
        if (size == capacity) {
            // Resize the array by the fixed increment
            capacity += RESIZE_INCREMENT;
            Node[] newSet = new Node[capacity];
            System.arraycopy(set, 0, newSet, 0, size);
            set = newSet;
        }
        set[size++] = node;
    }

    public boolean containsNode(Node node) {
        for (int i = 0; i < size; i++) {
            if (set[i].getRow() == node.getRow() && set[i].getCol() == node.getCol()) {
                return true;
            }
        }
        return false;
    }
}

class CustomHashMap {
    private Entry[] table;
    private int size;

    public CustomHashMap(int initialCapacity) {
        this.table = new Entry[initialCapacity];
        this.size = 0;
    }

    private int hash(Node key) { return (key.getRow() * 31 + key.getCol()) % table.length; }

    public Node get(Node key) {
        int index = hash(key);
        Entry current = table[index];
        while (current != null) {
            if (current.key.getRow() == key.getRow() && current.key.getCol() == key.getCol()) {
                return current.value;
            }
            current = current.next;
        }
        return null;
    }

    public void put(Node key, Node value) {
        int index = hash(key);
        if (table[index] == null) {
            table[index] = new Entry(key, value);
            size++;
        } else {
            Entry current = table[index];
            while (current.next != null) {
                if (current.key.getRow() == key.getRow() && current.key.getCol() == key.getCol()) {
                    // Update value if the key already exists
                    current.value = value;
                    return;
                }
                current = current.next;
            }
            current.next = new Entry(key, value);
            size++;
        }
    }

    private static class Entry {
        Node key;
        Node value;
        Entry next;

        Entry(Node key, Node value) {
            this.key = key;
            this.value = value;
            this.next = null;
        }
    }
}