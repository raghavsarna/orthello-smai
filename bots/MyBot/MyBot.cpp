/*
 * @file MyBot.cpp
 * @author AI3002 Team
 * @date 2025-11-17
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
    int getFrontierDiscs(const OthelloBoard &board, Turn turn);
    int countValidMoves(const OthelloBoard &board, Turn turn);
    
    // PVS with Iterative Deepening
    int pvs(OthelloBoard board, int depth, int alpha, int beta, Turn currentTurn);
    Move getBestMove(const OthelloBoard &board, int depth);
    
    // Move ordering
    struct ScoredMove {
        Move move;
        int score;
        bool operator<(const ScoredMove& other) const {
            return score > other.score; // Descending order
        }
    };
    std::vector<ScoredMove> getSortedMoves(const OthelloBoard& board, Turn turn);
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
    : OthelloPlayer(turn), timeLimit(1.90), timeExpired(false)
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

int MyBot::countValidMoves(const OthelloBoard& board, Turn turn) {
    int count = 0;
    Turn opponent = (turn == BLACK) ? RED : BLACK;
    int dirs[8][2] = {{0,1}, {1,1}, {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0}, {-1,1}};

    for(int i=0; i<8; i++) {
        for(int j=0; j<8; j++) {
            if(board.get(i, j) != EMPTY) continue;
            
            bool valid = false;
            for(int k=0; k<8; k++) {
                int x = i + dirs[k][0];
                int y = j + dirs[k][1];
                
                if(x >= 0 && x < 8 && y >= 0 && y < 8 && board.get(x, y) == opponent) {
                    while(true) {
                        x += dirs[k][0];
                        y += dirs[k][1];
                        if(x < 0 || x >= 8 || y < 0 || y >= 8 || board.get(x, y) == EMPTY) break;
                        if(board.get(x, y) == turn) {
                            valid = true;
                            break;
                        }
                    }
                }
                if(valid) break;
            }
            if(valid) count++;
        }
    }
    return count;
}

int MyBot::getFrontierDiscs(const OthelloBoard& board, Turn turn) {
    int myFrontier = 0;
    int oppFrontier = 0;
    int dirs[8][2] = {{0,1}, {1,1}, {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0}, {-1,1}};
    
    for(int i=0; i<8; i++) {
        for(int j=0; j<8; j++) {
            Coin c = board.get(i, j);
            if(c == EMPTY) continue;
            
            bool isFrontier = false;
            for(int k=0; k<8; k++) {
                int x = i + dirs[k][0];
                int y = j + dirs[k][1];
                if(x >= 0 && x < 8 && y >= 0 && y < 8 && board.get(x, y) == EMPTY) {
                    isFrontier = true;
                    break;
                }
            }
            
            if(isFrontier) {
                if(c == turn) myFrontier++;
                else oppFrontier++;
            }
        }
    }
    
    if (myFrontier + oppFrontier == 0) return 0;
    return 100 * (oppFrontier - myFrontier) / (myFrontier + oppFrontier);
}

int MyBot::getMobility(const OthelloBoard &board, Turn turn)
{
    Turn opponent = (turn == BLACK) ? RED : BLACK;
    
    int myMoves = countValidMoves(board, turn);
    int oppMoves = countValidMoves(board, opponent);
    
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
        score += getMobility(board, turn) * 20;
        score += getCornerControl(board, turn) * 500;
        score += getFrontierDiscs(board, turn) * 10;
        
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
        score += getMobility(board, turn) * 10;
        score += getCornerControl(board, turn) * 500;
        score += getStability(board, turn) * 50;
        score += getFrontierDiscs(board, turn) * 10;
        score += getCoinParity(board, turn) * 5;
        
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
        score += posScore;
    }
    else // End game - maximize coin count
    {
        score += getCoinParity(board, turn) * 500;
        score += getCornerControl(board, turn) * 500;
        score += getMobility(board, turn) * 10;
    }
    
    return score;
}

std::vector<MyBot::ScoredMove> MyBot::getSortedMoves(const OthelloBoard& board, Turn turn) {
    list<Move> moves = board.getValidMoves(turn);
    std::vector<ScoredMove> scoredMoves;
    
    for (list<Move>::iterator it = moves.begin(); it != moves.end(); ++it) {
        OthelloBoard nextBoard = board;
        nextBoard.makeMove(turn, *it);
        // Simple static evaluation for sorting
        int score = evaluateBoard(nextBoard, turn);
        scoredMoves.push_back({*it, score});
    }
    
    // Sort moves to try best ones first
    std::sort(scoredMoves.begin(), scoredMoves.end());
    return scoredMoves;
}

int MyBot::pvs(OthelloBoard board, int depth, int alpha, int beta, Turn currentTurn)
{
    // Check time limit
    if (getCurrentTime() - startTime > timeLimit)
    {
        timeExpired = true;
        return 0;
    }
    
    // Base case
    if (depth == 0)
    {
        return evaluateBoard(board, currentTurn);
    }
    
    std::vector<ScoredMove> moves = getSortedMoves(board, currentTurn);
    
    // No valid moves - pass turn
    if (moves.empty())
    {
        Turn nextTurn = (currentTurn == BLACK) ? RED : BLACK;
        list<Move> oppMoves = board.getValidMoves(nextTurn);
        
        // Game over - both players can't move
        if (oppMoves.empty())
        {
            // Return final score from perspective of currentTurn
            // If currentTurn wins, positive. If loses, negative.
            int myCount = (currentTurn == BLACK) ? board.getBlackCount() : board.getRedCount();
            int oppCount = (currentTurn == BLACK) ? board.getRedCount() : board.getBlackCount();
            if (myCount > oppCount) return 10000 + (myCount - oppCount);
            if (myCount < oppCount) return -10000 + (myCount - oppCount);
            return 0;
        }
        
        // Pass turn to opponent
        // Negamax: -pvs(..., -beta, -alpha, nextTurn)
        return -pvs(board, depth - 1, -beta, -alpha, nextTurn);
    }
    
    for (size_t i = 0; i < moves.size(); ++i)
    {
        OthelloBoard newBoard = board;
        newBoard.makeMove(currentTurn, moves[i].move);
        Turn nextTurn = (currentTurn == BLACK) ? RED : BLACK;
        
        int score;
        if (i == 0)
        {
            // First move: full window search
            score = -pvs(newBoard, depth - 1, -beta, -alpha, nextTurn);
        }
        else
        {
            // Subsequent moves: null window search
            score = -pvs(newBoard, depth - 1, -alpha - 1, -alpha, nextTurn);
            
            // If this move is better than alpha but within bounds, re-search with full window
            if (alpha < score && score < beta && !timeExpired)
            {
                score = -pvs(newBoard, depth - 1, -beta, -score, nextTurn);
            }
        }
        
        if (timeExpired) return 0;
        
        alpha = max(alpha, score);
        if (alpha >= beta)
        {
            break; // Beta cut-off
        }
    }
    
    return alpha;
}

Move MyBot::getBestMove(const OthelloBoard &board, int depth)
{
    std::vector<ScoredMove> moves = getSortedMoves(board, turn);
    
    if (moves.empty())
        return Move::pass();
    
    if (moves.size() == 1)
        return moves[0].move;
    
    Move bestMove = moves[0].move;
    int bestScore = numeric_limits<int>::min();
    int alpha = numeric_limits<int>::min();
    int beta = numeric_limits<int>::max();
    
    for (size_t i = 0; i < moves.size(); ++i)
    {
        OthelloBoard newBoard = board;
        newBoard.makeMove(turn, moves[i].move);
        
        Turn opponent = (turn == BLACK) ? RED : BLACK;
        
        // Use PVS logic for the root node too
        int score;
        if (i == 0) {
             score = -pvs(newBoard, depth - 1, -beta, -alpha, opponent);
        } else {
             score = -pvs(newBoard, depth - 1, -alpha - 1, -alpha, opponent);
             if (alpha < score && score < beta && !timeExpired) {
                 score = -pvs(newBoard, depth - 1, -beta, -score, opponent);
             }
        }

        if (timeExpired) break;
        
        if (score > bestScore)
        {
            bestScore = score;
            bestMove = moves[i].move;
        }
        alpha = max(alpha, score);
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
    
    Move bestMove = moves.front();
    
    // Iterative deepening
    // Start from depth 1 to get a quick result
    // Increase max depth since PVS is more efficient
    int maxDepth = 64; 
    
    for (int depth = 1; depth <= maxDepth; depth++)
    {
        Move move = getBestMove(board, depth);
        
        if (timeExpired)
        {
            break;
        }
        else
        {
            bestMove = move;
        }
        
        // If we found a winning line or searched deep enough, we can stop?
        // For now, just use all available time.
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
