
#include "solver.h"

#define SIMULATION_BOOL 0 // 1 if you're running the simulation, 0 if you're running the actual robot

// #define SET_WALL_FUNCTION(x, y, direction) API_setWall(x, y, direction)

int distances[MAZE_SIZE][MAZE_SIZE] = {-1}; // 1000 if it hasn't been visited yet
int visited[MAZE_SIZE][MAZE_SIZE] = {0};
int wall_maze[MAZE_SIZE][MAZE_SIZE][4] = {0};

int priorityHeading = 0;

struct Coordinate position;
Heading heading;

int reached_center = 0; // "boolean" that stores whether the mouse should start exploring more squares
int change_index = 0;   // "boolean" that stores whether the mouse should run the fastRunSolver

//-------------------------------------------------------------------------------------------------------------------------------//
void setPriorityHeading(int direction);
void initialize();
void updateMaze();
void updatePosition(Action nextAction);
void updateHeading(Action nextAction);
void updateDistances();
void resetDistances();
void setPosition(int x, int y, int direction);
int xyToSquare(int x, int y);
struct Coordinate squareToCoord(int square);
int isWallInDirection(int x, int y, Heading direction);
Action solver();
Action fastRunSolver();
Action floodFill();
Action floodFillPriorityNorth(void);
void moveForwardWithVariableVelocity(int steps);
void fastRunWithVariableVelocity();
int getReachingCenter();

//-------------------------------------------------------------------------------------------------------------------------------//
void setPriorityHeading(int direction)
{
    priorityHeading = direction;
}

void searchRun()
{
    while (getReachingCenter() == 0)
    {
        Action nextMove = solver();

#if SIMULATION_BOOL
        printFullMazeDistances();
#endif

        switch (nextMove)
        {
        case FORWARD:
            MOVE_FORWARD_FUNCTION;
            break;
        case LEFT:
            TURN_LEFT_FUNCTION;
            break;
        case RIGHT:
            TURN_RIGHT_FUNCTION;
            break;
        case IDLE:
            break;
        }
    }
}

void searchCenterToStart()
{
    while (getReachingCenter())
    {
        Action nextMove = solver();

#if SIMULATION_BOOL
        printFullMazeDistances();
#endif

        switch (nextMove)
        {
        case FORWARD:
            MOVE_FORWARD_FUNCTION;
            break;
        case LEFT:
            TURN_LEFT_FUNCTION;
            break;
        case RIGHT:
            TURN_RIGHT_FUNCTION;
            break;
        case IDLE:
            break;
        }
    }

    if (heading == WEST)
    {
        TURN_RIGHT_FUNCTION;
    }
    else if (heading == SOUTH)
    {
        TURN_RIGHT_FUNCTION;
        TURN_RIGHT_FUNCTION;
    }
    heading = NORTH;
}

void fastRunWithVariableVelocity()
{
    struct path
    {
        int steps;
        int action;
    } path;

    while (distances[position.x][position.y] != 0)
    {
        path.steps = 0;
        path.action = IDLE;

        while (distances[position.x][position.y] != 0)
        {
            Action nextMove = fastRunSolver();

            if (nextMove == FORWARD) // if the next move is forward, increment the number of steps
            {
                path.steps++;
            }
            else // if the next move is not forward, break out of the loop
            {
                path.action = nextMove;
                break;
            }
        }

#if SIMULATION_BOOL
        debug_log("Step:");
        debug_int(path.steps);
        debug_log("Action:");
        if (path.action == LEFT)
        {
            debug_log("LEFT");
        }
        else if (path.action == RIGHT)
        {
            debug_log("RIGHT");
        }
        debug_log("\n");
#endif

        // move forward the number of steps
        moveForwardWithVariableVelocity(path.steps);

        // turn in the correct direction
        if (path.action == LEFT)
        {
            TURN_LEFT_FUNCTION;
        }
        else if (path.action == RIGHT)
        {
            TURN_RIGHT_FUNCTION;
        }
    }

    MOVE_FORWARD_FUNCTION;
    updatePosition(FORWARD);

    TURN_LEFT_FUNCTION;
    updateHeading(LEFT);
    TURN_LEFT_FUNCTION;
    updateHeading(LEFT);

    reached_center = 1;
}

void setPosition(int x, int y, int direction)
{
    position.x = x;
    position.y = y;
    heading = direction;
}

void markCenterWall()
{
    // set the walls around the center

    wall_maze[7][7][SOUTH] = 1;
    wall_maze[7][6][NORTH] = 1;

    wall_maze[7][7][WEST] = 1;
    wall_maze[6][7][EAST] = 1;

    wall_maze[8][7][EAST] = 1;
    wall_maze[9][7][WEST] = 1;

    wall_maze[8][7][SOUTH] = 1;
    wall_maze[8][6][NORTH] = 1;

    wall_maze[7][8][NORTH] = 1;
    wall_maze[7][9][SOUTH] = 1;

    wall_maze[7][8][WEST] = 1;
    wall_maze[6][8][EAST] = 1;

    wall_maze[8][8][NORTH] = 1;
    wall_maze[8][9][SOUTH] = 1;

    wall_maze[8][8][EAST] = 1;
    wall_maze[9][8][WEST] = 1;

    // clear the walls at the entrance/exit of the center
    if (heading == NORTH)
    {
        if (position.x == 7 && position.y == 7)
        {
            wall_maze[7][7][SOUTH] = 0;
            wall_maze[7][6][NORTH] = 0;
        }
        else if (position.x == 8 && position.y == 7)
        {
            wall_maze[8][7][SOUTH] = 0;
            wall_maze[8][6][NORTH] = 0;
        }
    }
    else if (heading == EAST)
    {
        if (position.x == 7 && position.y == 7)
        {
            wall_maze[7][7][WEST] = 0;
            wall_maze[6][7][EAST] = 0;
        }
        else if (position.x == 7 && position.y == 8)
        {
            wall_maze[7][8][WEST] = 0;
            wall_maze[6][8][EAST] = 0;
        }
    }
    else if (heading == SOUTH)
    {
        if (position.x == 7 && position.y == 8)
        {
            wall_maze[7][8][NORTH] = 0;
            wall_maze[7][9][SOUTH] = 0;
        }
        else if (position.x == 8 && position.y == 8)
        {
            wall_maze[8][8][NORTH] = 0;
            wall_maze[8][9][SOUTH] = 0;
        }
    }
    else if (heading == WEST)
    {
        if (position.x == 8 && position.y == 7)
        {
            wall_maze[8][7][EAST] = 0;
            wall_maze[9][7][WEST] = 0;
        }
        else if (position.x == 8 && position.y == 8)
        {
            wall_maze[8][8][EAST] = 0;
            wall_maze[9][8][WEST] = 0;
        }
    }
}

#if SIMULATION_BOOL
void printFullMazeDistances()
{
    for (int x = 0; x < MAZE_SIZE; ++x)
    {
        for (int y = 0; y < MAZE_SIZE; ++y)
        {
            API_setText(x, y, distances[x][y]);
        }
    }
}
#endif

void initialize()
{
    // setting the borders
    for (int i = 1; i < MAZE_SIZE - 1; ++i)
    {
        wall_maze[0][i][WEST] = 1;
        wall_maze[i][0][SOUTH] = 1;
        wall_maze[i][MAZE_SIZE - 1][NORTH] = 1;
        wall_maze[MAZE_SIZE - 1][i][EAST] = 1;
    }

    wall_maze[0][0][SOUTH] = 1;
    wall_maze[0][0][WEST] = 1;

    wall_maze[0][MAZE_SIZE - 1][NORTH] = 1;
    wall_maze[0][MAZE_SIZE - 1][WEST] = 1;

    wall_maze[MAZE_SIZE - 1][0][EAST] = 1;
    wall_maze[MAZE_SIZE - 1][0][SOUTH] = 1;

    wall_maze[MAZE_SIZE - 1][MAZE_SIZE - 1][NORTH] = 1;
    wall_maze[MAZE_SIZE - 1][MAZE_SIZE - 1][EAST] = 1;

    // setting initial distances
    resetDistances();

    // setting mouse position + heading
    setPosition(0, 0, NORTH);
}

/*
Updates the maze's walls based on what the mouse can currently see
*/
void updateMaze()
{
    int x = position.x;
    int y = position.y;
    // start by assuming there are no walls, this variable will be changed based on which walls you see
    // unsigned int walls = _0000;

    switch (heading)
    {
    case NORTH:
        if (CHECK_WALL_FRONT)
        {
            wall_maze[x][y][NORTH] = 1; // stores the wall to the north in wall_maze

            // updating neighboring squares as well (if there is one):
            if (y + 1 != MAZE_SIZE)
            {
                wall_maze[x][y + 1][SOUTH] = 1;
            }
        }
        if (CHECK_WALL_LEFT)
        {
            wall_maze[x][y][WEST] = 1;
            if (x - 1 >= 0)
            {
                wall_maze[x - 1][y][EAST] = 1;
            }
        }
        if (CHECK_WALL_RIGHT)
        {
            wall_maze[x][y][EAST] = 1;

            if (x + 1 != MAZE_SIZE)
            {
                wall_maze[x + 1][y][WEST] = 1;
            }
        }
        break;
    case EAST:
        if (CHECK_WALL_FRONT)
        {
            wall_maze[x][y][EAST] = 1;

            if (x + 1 != MAZE_SIZE)
            {
                wall_maze[x + 1][y][WEST] = 1;
            }
        }
        if (CHECK_WALL_LEFT)
        {
            wall_maze[x][y][NORTH] = 1;

            if (y + 1 != MAZE_SIZE)
            {
                wall_maze[x][y + 1][SOUTH] = 1;
            }
        }
        if (CHECK_WALL_RIGHT)
        {
            wall_maze[x][y][SOUTH] = 1;

            if (y - 1 >= 0)
            {
                wall_maze[x][y - 1][NORTH] = 1;
            }
        }
        break;
    case SOUTH:
        if (CHECK_WALL_FRONT)
        {
            wall_maze[x][y][SOUTH] = 1;

            if (y - 1 >= 0)
            {
                wall_maze[x][y - 1][NORTH] = 1;
            }
        }
        if (CHECK_WALL_LEFT)
        {
            wall_maze[x][y][EAST] = 1;

            if (x + 1 != MAZE_SIZE)
            {
                wall_maze[x + 1][y][WEST] = 1;
            }
        }
        if (CHECK_WALL_RIGHT)
        {
            wall_maze[x][y][WEST] = 1;

            if (x - 1 >= 0)
            {
                wall_maze[x - 1][y][EAST] = 1;
            }
        }
        break;
    case WEST:
        if (CHECK_WALL_FRONT)
        {
            wall_maze[x][y][WEST] = 1;

            if (x - 1 >= 0)
            {
                wall_maze[x - 1][y][EAST] = 1;
            }
        }
        if (CHECK_WALL_LEFT)
        {
            wall_maze[x][y][SOUTH] = 1;

            if (y - 1 >= 0)
            {
                wall_maze[x][y - 1][NORTH] = 1;
            }
        }
        if (CHECK_WALL_RIGHT)
        {
            wall_maze[x][y][NORTH] = 1;

            if (y + 1 != MAZE_SIZE)
            {
                wall_maze[x][y + 1][SOUTH] = 1;
            }
        }
        break;
    }

#if SIMULATION_BOOL
    if (wall_maze[x][y][NORTH] == 1)
    {
        API_setWall(x, y, 'n');
    }
    if (wall_maze[x][y][EAST] == 1)
    {
        API_setWall(x, y, 'e');
    }
    if (wall_maze[x][y][SOUTH] == 1)
    {
        API_setWall(x, y, 's');
    }
    if (wall_maze[x][y][WEST] == 1)
    {
        API_setWall(x, y, 'w');
    }
#endif
}

// convert xy to square number
int xyToSquare(int x, int y)
{
    return x + MAZE_SIZE * y;
}
// convert square number to xy
struct Coordinate squareToCoord(int square)
{
    struct Coordinate coord;
    coord.x = square % MAZE_SIZE;
    coord.y = square / MAZE_SIZE;
    return coord;
}

void resetDistances()
{
    // initially sets all the distances to -1 (invalid distance)
    for (int x = 0; x < MAZE_SIZE; ++x)
    {
        for (int y = 0; y < MAZE_SIZE; ++y)
        {
            distances[x][y] = -1;
        }
    }

    // if you haven't reached the center, set the goal to be the center
    if (!reached_center)
    {
        // sets goal distances
        if (MAZE_SIZE % 2 == 0)
        {
            distances[MAZE_SIZE / 2][MAZE_SIZE / 2] = 0;
            distances[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2] = 0;
            distances[MAZE_SIZE / 2][MAZE_SIZE / 2 - 1] = 0;
            distances[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2 - 1] = 0;
        }
        else
        {
            distances[MAZE_SIZE / 2][MAZE_SIZE / 2] = 0;
        }
    }
    else
    {
        distances[0][0] = 0; // go back to the start
    }
}

int isWallInDirection(int x, int y, Heading direction)
{
    switch (direction)
    {
    case NORTH:
        if (wall_maze[x][y][NORTH] == 1)
            return 1;
        break;
    case EAST:
        if (wall_maze[x][y][EAST] == 1)
            return 1;
        break;
    case SOUTH:
        if (wall_maze[x][y][SOUTH] == 1)
            return 1;
        break;
    case WEST:
        if (wall_maze[x][y][WEST] == 1)
            return 1;
        break;
    }
    return 0;
}

void updateDistances()
{
    resetDistances();
    queue squares = queue_create();

    // adds the goal squares to the queue (the middle of the maze or the starting position depending on if you've reached the center)
    for (int x = 0; x < MAZE_SIZE; ++x)
    {
        for (int y = 0; y < MAZE_SIZE; ++y)
        {
            if (distances[x][y] == 0)
                queue_push(squares, xyToSquare(x, y));
        }
    }

    while (!queue_is_empty(squares))
    {
        struct Coordinate square = squareToCoord(queue_pop(squares));
        int x = square.x;
        int y = square.y;

        // if there's no wall to the north && the square to the north hasn't been checked yet
        if (isWallInDirection(x, y, NORTH) == 0 && distances[x][y + 1] == -1)
        {
            distances[x][y + 1] = distances[x][y] + 1;
            queue_push(squares, xyToSquare(x, y + 1));
        }
        // same as ^ but for east
        if (isWallInDirection(x, y, EAST) == 0 && distances[x + 1][y] == -1)
        {
            distances[x + 1][y] = distances[x][y] + 1;
            queue_push(squares, xyToSquare(x + 1, y));
        }
        // same as ^ but for south
        if (isWallInDirection(x, y, SOUTH) == 0 && distances[x][y - 1] == -1)
        {
            distances[x][y - 1] = distances[x][y] + 1;
            queue_push(squares, xyToSquare(x, y - 1));
        }
        // same as ^ but for west
        if (isWallInDirection(x, y, WEST) == 0 && distances[x - 1][y] == -1)
        {
            distances[x - 1][y] = distances[x][y] + 1;
            queue_push(squares, xyToSquare(x - 1, y));
        }
    }

#if SIMULATION_BOOL
    printFullMazeDistances();
#endif
}

void calculateShortestPathDistances()
{
    resetDistances();

    // sets goal distances
    if (MAZE_SIZE % 2 == 0)
    {
        /*
        neu ma tran chan thi dat 4 o o trung tam la dich
        */
        distances[MAZE_SIZE / 2][MAZE_SIZE / 2] = 0;
        distances[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2] = 0;
        distances[MAZE_SIZE / 2][MAZE_SIZE / 2 - 1] = 0;
        distances[MAZE_SIZE / 2 - 1][MAZE_SIZE / 2 - 1] = 0;
    }
    else
    {
        distances[MAZE_SIZE / 2][MAZE_SIZE / 2] = 0;
    }
    distances[0][0] = -1;

    visited[7][7] = 1;
    visited[8][7] = 1;
    visited[7][8] = 1;
    visited[8][8] = 1;

    queue squares = queue_create();

    // adds the goal squares to the queue (the middle of the maze or the starting position depending on if you've reached the center)
    for (int x = 0; x < MAZE_SIZE; ++x)
    {
        for (int y = 0; y < MAZE_SIZE; ++y)
        {
            if (distances[x][y] == 0)
                queue_push(squares, xyToSquare(x, y));
        }
    }

    while (!queue_is_empty(squares))
    {
        struct Coordinate square = squareToCoord(queue_pop(squares));
        int x = square.x;
        int y = square.y;

        if (visited[x][y] == 0)
        {
            continue;
        }

        // if there's no wall to the north && the square to the north hasn't been checked yet
        if (isWallInDirection(x, y, NORTH) == 0 && distances[x][y + 1] == -1)
        {
            distances[x][y + 1] = distances[x][y] + 1;
            queue_push(squares, xyToSquare(x, y + 1));
        }
        // same as ^ but for east
        if (isWallInDirection(x, y, EAST) == 0 && distances[x + 1][y] == -1)
        {
            distances[x + 1][y] = distances[x][y] + 1;
            queue_push(squares, xyToSquare(x + 1, y));
        }
        // same as ^ but for south
        if (isWallInDirection(x, y, SOUTH) == 0 && distances[x][y - 1] == -1)
        {
            distances[x][y - 1] = distances[x][y] + 1;
            queue_push(squares, xyToSquare(x, y - 1));
        }
        // same as ^ but for west
        if (isWallInDirection(x, y, WEST) == 0 && distances[x - 1][y] == -1)
        {
            distances[x - 1][y] = distances[x][y] + 1;
            queue_push(squares, xyToSquare(x - 1, y));
        }
    }

#if SIMULATION_BOOL
    printFullMazeDistances();
#endif
}

void moveForwardWithVariableVelocity(int steps)
{
    for (int i = 0; i < steps; i++)
    {
        MOVE_FORWARD_FUNCTION;
    }
}

void updateHeading(Action nextAction)
{
    if (nextAction == FORWARD || nextAction == IDLE)
    {
        return;
    }
    else if (nextAction == LEFT)
    {
        switch (heading)
        {
        case NORTH:
            heading = WEST;
            break;
        case EAST:
            heading = NORTH;
            break;
        case SOUTH:
            heading = EAST;
            break;
        case WEST:
            heading = SOUTH;
            break;
        default:
            break;
        }
    }
    else if (nextAction == RIGHT)
    {
        switch (heading)
        {
        case NORTH:
            heading = EAST;
            break;
        case EAST:
            heading = SOUTH;
            break;
        case SOUTH:
            heading = WEST;
            break;
        case WEST:
            heading = NORTH;
            break;
        default:
            break;
        }
    }
}

void updatePosition(Action nextAction)
{
    if (nextAction != FORWARD)
    {
        return;
    }

    switch (heading)
    {
    case NORTH:
        position.y += 1;
        break;
    case SOUTH:
        position.y -= 1;
        break;
    case EAST:
        position.x += 1;
        break;
    case WEST:
        position.x -= 1;
        break;
    default:
        break;
    }
}

int getReachingCenter()
{
    // if you reached the center, go back to the start
    if (!reached_center && distances[position.x][position.y] == 0)
    {
        reached_center = 1;
        change_index++;
    }
    // if you went to the center & all the way back to the start, restart
    else if (reached_center && distances[position.x][position.y] == 0 && position.x == 0 && position.y == 0)
    {
        reached_center = 0;
        change_index++;
    }

    return reached_center;
}

Action solver()
{
    updateMaze();
    updateDistances();

    Action action = floodFill();

    updateHeading(action);
    updatePosition(action);
    return action;
}

Action fastRunSolver()
{
    Action action = floodFill();

    updateHeading(action);
    updatePosition(action);
    return action;
}

Action floodFill(void)
{
    unsigned int least_distance = 300; // just some large number, none of the distances will be over 300
    Action optimal_move = IDLE;

    visited[position.x][position.y] = 1;

    /*
    Basic Idea:
    - Look at the square in front of you, to the left, and to the right if there are no walls
    - Find the square with the lowest distance to the goal
    - Move in the direction of that square (move forward if it's the forward square, turn in the correct direction otherwise)
    */

    if (heading == NORTH)
    {
        if (!isWallInDirection(position.x, position.y, NORTH) && distances[position.x][position.y + 1] <= least_distance)
        {
            if (distances[position.x][position.y + 1] < least_distance || priorityHeading == NORTH)
            {
                least_distance = distances[position.x][position.y + 1];
                optimal_move = FORWARD;
            }
        }
        if (!isWallInDirection(position.x, position.y, EAST) && distances[position.x + 1][position.y] <= least_distance)
        {
            if (distances[position.x + 1][position.y] < least_distance || priorityHeading == EAST)
            {
                least_distance = distances[position.x + 1][position.y];
                optimal_move = RIGHT;
            }
        }
        if (!isWallInDirection(position.x, position.y, WEST) && distances[position.x - 1][position.y] <= least_distance)
        {
            if (distances[position.x - 1][position.y] < least_distance || priorityHeading == WEST)
            {
                least_distance = distances[position.x - 1][position.y];
                optimal_move = LEFT;
            }
        }
    }
    else if (heading == EAST)
    {
        if (!isWallInDirection(position.x, position.y, NORTH) && distances[position.x][position.y + 1] <= least_distance)
        {
            if (distances[position.x][position.y + 1] < least_distance || priorityHeading == NORTH)
            {
                least_distance = distances[position.x][position.y + 1];
                optimal_move = LEFT;
            }
        }
        if (!isWallInDirection(position.x, position.y, EAST) && distances[position.x + 1][position.y] <= least_distance)
        {
            if (distances[position.x + 1][position.y] < least_distance || priorityHeading == EAST)
            {
                least_distance = distances[position.x + 1][position.y];
                optimal_move = FORWARD;
            }
        }
        if (!isWallInDirection(position.x, position.y, SOUTH) && distances[position.x][position.y - 1] <= least_distance)
        {
            if (distances[position.x][position.y - 1] < least_distance || priorityHeading == SOUTH)
            {
                least_distance = distances[position.x][position.y - 1];
                optimal_move = RIGHT;
            }
        }
    }
    else if (heading == SOUTH)
    {
        if (!isWallInDirection(position.x, position.y, EAST) && distances[position.x + 1][position.y] <= least_distance)
        {
            if (distances[position.x + 1][position.y] < least_distance || priorityHeading == EAST)
            {
                least_distance = distances[position.x + 1][position.y];
                optimal_move = LEFT;
            }
        }
        if (!isWallInDirection(position.x, position.y, SOUTH) && distances[position.x][position.y - 1] <= least_distance)
        {
            if (distances[position.x][position.y - 1] < least_distance || priorityHeading == SOUTH)
            {
                least_distance = distances[position.x][position.y - 1];
                optimal_move = FORWARD;
            }
        }
        if (!isWallInDirection(position.x, position.y, WEST) && distances[position.x - 1][position.y] <= least_distance)
        {
            if (distances[position.x - 1][position.y] < least_distance || priorityHeading == WEST)
            {
                least_distance = distances[position.x - 1][position.y];
                optimal_move = RIGHT;
            }
        }
    }
    else if (heading == WEST)
    {
        if (!isWallInDirection(position.x, position.y, NORTH) && distances[position.x][position.y + 1] <= least_distance)
        {
            if (distances[position.x][position.y + 1] < least_distance || priorityHeading == NORTH)
            {
                least_distance = distances[position.x][position.y + 1];
                optimal_move = RIGHT;
            }
        }
        if (!isWallInDirection(position.x, position.y, SOUTH) && distances[position.x][position.y - 1] <= least_distance)
        {
            if (distances[position.x][position.y - 1] < least_distance || priorityHeading == SOUTH)
            {
                least_distance = distances[position.x][position.y - 1];
                optimal_move = LEFT;
            }
        }
        if (!isWallInDirection(position.x, position.y, WEST) && distances[position.x - 1][position.y] <= least_distance)
        {
            if (distances[position.x - 1][position.y] < least_distance || priorityHeading == WEST)
            {
                least_distance = distances[position.x - 1][position.y];
                optimal_move = FORWARD;
            }
        }
    }

    // handles dead ends (when there's no walls in front, to the right or to the left)
    if (least_distance == 300)
        optimal_move = RIGHT; // arbitrary, can be any turn

    return optimal_move;
}
