#ifndef SOLVER_H
#define SOLVER_H

#include <stdint.h>
#include <stdlib.h>

#define MAZE_SIZE 16

/* MAZE CONSTANTS */

typedef enum Heading
{
    NORTH,
    EAST,
    SOUTH,
    WEST
} Heading;

typedef enum Action
{
    LEFT,
    FORWARD,
    RIGHT,
    IDLE
} Action;

struct Coordinate
{
    int x;
    int y;
};

void setPriorityHeading(int32_t direction);
void searchRun(void);
void searchCenterToStart(void);
void firstFastRun(void);
void loopRun(void);
void setPosition(int32_t x, int32_t y, int32_t direction);
void markCenterWall(void);
void initialize(void);
void updateMaze(void);      // updates the maze array with the walls around the mouse's current position
void updateDistances(void); // the "floodfill" algorithm
void calculateShortestPathDistances(void);
void fastRunWithVariableVelocity(void);
void resetDistances(void);
int32_t xyToSquare(int32_t x, int32_t y);
struct Coordinate squareToCoord(int32_t square);
int32_t isWallInDirection(int32_t x, int32_t y, Heading direction);
void updateHeading(Action nextAction);
void updatePosition(Action nextAction);
int32_t getReachingCenter(void);
Action solver(void);
Action floodFill(void);

#endif
