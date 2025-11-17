/*
 * @file MyBot.cpp
 * @author AI3002 Team
 * @date 2025-11-17
 * Competitive Othello bot using Minimax with Alpha-Beta Pruning
 */

#include "Othello.h"
#include "OthelloBoard.h"
#include "OthelloPlayer.h"
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <limits>
#include <vector>
#include <sys/time.h>
using namespace std;
using namespace Desdemona;

class MyBot : public OthelloPlayer
{
public:
    MyBot(Turn turn);
    virtual Move play(const OthelloBoard &board);

private:
    // Positional weight matrix for board evaluation
    static const int WEIGHTS[8][8];
    
    // Time management
    double startTime;
    double timeLimit;
    bool timeExpired;
    
    // Helper functions
    double getCurrentTime();
    int evaluateBoard(const OthelloBoard &board, Turn turn);
    int getStability(const OthelloBoard &board, Turn turn);
    int getMobility(const OthelloBoard &board, Turn turn);
    int getCornerControl(const OthelloBoard &board, Turn turn);
    int getCoinParity(const OthelloBoard &board, Turn turn);
    
    // Minimax with alpha-beta pruning
    int minimax(OthelloBoard board, int depth, int alpha, int beta, bool isMaximizing, Turn currentTurn);
    Move getBestMove(const OthelloBoard &board, int depth);
};

// Positional weights - corners are most valuable, edges strategic, avoid X and C squares near empty corners
const int MyBot::WEIGHTS[8][8] = {
    {100, -20,  10,   5,   5,  10, -20, 100},
    {-20, -50,  -2,  -2,  -2,  -2, -50, -20},
    { 10,  -2,   5,   1,   1,   5,  -2,  10},
    {  5,  -2,   1,   1,   1,   1,  -2,   5},
    {  5,  -2,   1,   1,   1,   1,  -2,   5},
    { 10,  -2,   5,   1,   1,   5,  -2,  10},
    {-20, -50,  -2,  -2,  -2,  -2, -50, -20},
    {100, -20,  10,   5,   5,  10, -20, 100}
};

MyBot::MyBot(Turn turn)
    : OthelloPlayer(turn), timeLimit(1.8), timeExpired(false)
{
}

double MyBot::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

int MyBot::getCoinParity(const OthelloBoard &board, Turn turn)
{
    int myCoins = (turn == BLACK) ? board.getBlackCount() : board.getRedCount();
    int oppCoins = (turn == BLACK) ? board.getRedCount() : board.getBlackCount();
    
    if (myCoins + oppCoins == 0) return 0;
    return 100 * (myCoins - oppCoins) / (myCoins + oppCoins);
}

int MyBot::getCornerControl(const OthelloBoard &board, Turn turn)
{
    int myCorners = 0;
    int oppCorners = 0;
    Turn opponent = (turn == BLACK) ? RED : BLACK;
    
    // Check all four corners
    int corners[4][2] = {{0,0}, {0,7}, {7,0}, {7,7}};
    for (int i = 0; i < 4; i++)
    {
        int x = corners[i][0];
        int y = corners[i][1];
        if (board.get(x, y) == turn) myCorners++;
        else if (board.get(x, y) == opponent) oppCorners++;
    }
    
    if (myCorners + oppCorners == 0) return 0;
    return 100 * (myCorners - oppCorners) / (myCorners + oppCorners);
}

int MyBot::getMobility(const OthelloBoard &board, Turn turn)
{
    Turn opponent = (turn == BLACK) ? RED : BLACK;
    
    int myMoves = board.getValidMoves(turn).size();
    int oppMoves = board.getValidMoves(opponent).size();
    
    if (myMoves + oppMoves == 0) return 0;
    return 100 * (myMoves - oppMoves) / (myMoves + oppMoves);
}

int MyBot::getStability(const OthelloBoard &board, Turn turn)
{
    // Count stable pieces (simplified: pieces on edges and corners)
    int myStable = 0;
    int oppStable = 0;
    Turn opponent = (turn == BLACK) ? RED : BLACK;
    
    // Check edges
    for (int i = 0; i < 8; i++)
    {
        if (board.get(0, i) == turn) myStable++;
        else if (board.get(0, i) == opponent) oppStable++;
        
        if (board.get(7, i) == turn) myStable++;
        else if (board.get(7, i) == opponent) oppStable++;
        
        if (board.get(i, 0) == turn) myStable++;
        else if (board.get(i, 0) == opponent) oppStable++;
        
        if (board.get(i, 7) == turn) myStable++;
        else if (board.get(i, 7) == opponent) oppStable++;
    }
    
    if (myStable + oppStable == 0) return 0;
    return 100 * (myStable - oppStable) / (myStable + oppStable);
}

int MyBot::evaluateBoard(const OthelloBoard &board, Turn turn)
{
    Turn opponent = (turn == BLACK) ? RED : BLACK;
    
    int totalCoins = board.getBlackCount() + board.getRedCount();
    int score = 0;
    
    // Different strategies for different game phases
    if (totalCoins < 20) // Early game - focus on mobility and position
    {
        score += getMobility(board, turn) * 5;
        score += getCornerControl(board, turn) * 30;
        
        // Positional weights
        int posScore = 0;
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                if (board.get(i, j) == turn)
                    posScore += WEIGHTS[i][j];
                else if (board.get(i, j) == opponent)
                    posScore -= WEIGHTS[i][j];
            }
        }
        score += posScore;
    }
    else if (totalCoins < 50) // Mid game - balance everything
    {
        score += getMobility(board, turn) * 3;
        score += getCornerControl(board, turn) * 25;
        score += getStability(board, turn) * 2;
        score += getCoinParity(board, turn);
        
        // Positional weights with reduced importance
        int posScore = 0;
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 8; j++)
            {
                if (board.get(i, j) == turn)
                    posScore += WEIGHTS[i][j];
                else if (board.get(i, j) == opponent)
                    posScore -= WEIGHTS[i][j];
            }
        }
        score += posScore / 2;
    }
    else // End game - maximize coin count
    {
        score += getCoinParity(board, turn) * 10;
        score += getCornerControl(board, turn) * 15;
        score += getMobility(board, turn);
    }
    
    return score;
}

int MyBot::minimax(OthelloBoard board, int depth, int alpha, int beta, bool isMaximizing, Turn currentTurn)
{
    // Check time limit
    if (getCurrentTime() - startTime > timeLimit)
    {
        timeExpired = true;
        return 0;
    }
    
    // Base case
    if (depth == 0 || timeExpired)
    {
        return evaluateBoard(board, turn);
    }
    
    list<Move> moves = board.getValidMoves(currentTurn);
    
    // No valid moves - pass turn
    if (moves.empty())
    {
        Turn nextTurn = (currentTurn == BLACK) ? RED : BLACK;
        list<Move> oppMoves = board.getValidMoves(nextTurn);
        
        // Game over - both players can't move
        if (oppMoves.empty())
        {
            return evaluateBoard(board, turn);
        }
        
        // Pass turn to opponent
        return minimax(board, depth - 1, alpha, beta, !isMaximizing, nextTurn);
    }
    
    if (isMaximizing)
    {
        int maxEval = numeric_limits<int>::min();
        for (list<Move>::iterator it = moves.begin(); it != moves.end() && !timeExpired; ++it)
        {
            OthelloBoard newBoard = board;
            newBoard.makeMove(currentTurn, *it);
            
            Turn nextTurn = (currentTurn == BLACK) ? RED : BLACK;
            int eval = minimax(newBoard, depth - 1, alpha, beta, false, nextTurn);
            
            maxEval = max(maxEval, eval);
            alpha = max(alpha, eval);
            
            if (beta <= alpha)
                break; // Alpha-beta pruning
        }
        return maxEval;
    }
    else
    {
        int minEval = numeric_limits<int>::max();
        for (list<Move>::iterator it = moves.begin(); it != moves.end() && !timeExpired; ++it)
        {
            OthelloBoard newBoard = board;
            newBoard.makeMove(currentTurn, *it);
            
            Turn nextTurn = (currentTurn == BLACK) ? RED : BLACK;
            int eval = minimax(newBoard, depth - 1, alpha, beta, true, nextTurn);
            
            minEval = min(minEval, eval);
            beta = min(beta, eval);
            
            if (beta <= alpha)
                break; // Alpha-beta pruning
        }
        return minEval;
    }
}

Move MyBot::getBestMove(const OthelloBoard &board, int depth)
{
    list<Move> moves = board.getValidMoves(turn);
    
    if (moves.empty())
        return Move::pass();
    
    if (moves.size() == 1)
        return moves.front();
    
    Move bestMove = moves.front();
    int bestScore = numeric_limits<int>::min();
    
    for (list<Move>::iterator it = moves.begin(); it != moves.end() && !timeExpired; ++it)
    {
        OthelloBoard newBoard = board;
        newBoard.makeMove(turn, *it);
        
        Turn opponent = (turn == BLACK) ? RED : BLACK;
        int score = minimax(newBoard, depth - 1, numeric_limits<int>::min(), 
                           numeric_limits<int>::max(), false, opponent);
        
        if (score > bestScore)
        {
            bestScore = score;
            bestMove = *it;
        }
    }
    
    return bestMove;
}

Move MyBot::play(const OthelloBoard &board)
{
    startTime = getCurrentTime();
    timeExpired = false;
    
    list<Move> moves = board.getValidMoves(turn);
    
    if (moves.empty())
        return Move::pass();
    
    if (moves.size() == 1)
        return moves.front();
    
    // Use iterative deepening to maximize search depth within time limit
    Move bestMove = moves.front();
    int totalCoins = board.getBlackCount() + board.getRedCount();
    
    // Adaptive depth based on game phase
    int maxDepth = 8;
    if (totalCoins < 20)
        maxDepth = 6;  // Early game - less depth needed
    else if (totalCoins > 50)
        maxDepth = 10; // End game - search deeper
    
    for (int depth = 3; depth <= maxDepth; depth++)
    {
        if (getCurrentTime() - startTime > timeLimit)
            break;
        
        Move move = getBestMove(board, depth);
        if (!timeExpired)
            bestMove = move;
        else
            break;
    }
    
    return bestMove;
}

// The following lines are _very_ important to create a bot module for Desdemona

extern "C"
{
    OthelloPlayer *createBot(Turn turn)
    {
        return new MyBot(turn);
    }

    void destroyBot(OthelloPlayer *bot)
    {
        delete bot;
    }
}
