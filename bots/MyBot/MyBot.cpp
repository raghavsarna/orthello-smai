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
#include <unordered_map>
#include <cstdint>

using namespace std;
using namespace Desdemona;

// ==========================================
// Bitboard Implementation
// ==========================================

typedef uint64_t Bitboard;

class FastBoard {
public:
    Bitboard my_discs;
    Bitboard opp_discs;

    FastBoard() : my_discs(0), opp_discs(0) {}

    void reset() {
        my_discs = 0;
        opp_discs = 0;
    }

    // Convert Desdemona board to Bitboard
    // x is column (0-7), y is row (0-7)
    // We map (x, y) to index = 8 * y + x
    void load(const OthelloBoard& board, Turn myTurn) {
        my_discs = 0;
        opp_discs = 0;
        Turn oppTurn = (myTurn == BLACK) ? RED : BLACK;

        for (int y = 0; y < 8; y++) {
            for (int x = 0; x < 8; x++) {
                Coin c = board.get(x, y);
                if (c == myTurn) {
                    my_discs |= (1ULL << (8 * y + x));
                } else if (c == oppTurn) {
                    opp_discs |= (1ULL << (8 * y + x));
                }
            }
        }
    }

    inline int count_set_bits(Bitboard b) const {
        return __builtin_popcountll(b);
    }

    // Generate moves for the current player (my_discs)
    Bitboard get_valid_moves() const {
        Bitboard empty = ~(my_discs | opp_discs);
        Bitboard candidates = opp_discs & 0x7E7E7E7E7E7E7E7EULL; // Mask to avoid wrapping
        Bitboard moves = 0;

        // 8 directions
        // Right
        Bitboard t = (my_discs >> 1) & candidates;
        t |= (t >> 1) & candidates;
        t |= (t >> 1) & candidates;
        t |= (t >> 1) & candidates;
        t |= (t >> 1) & candidates;
        t |= (t >> 1) & candidates;
        moves |= (t >> 1);

        // Left
        t = (my_discs << 1) & candidates;
        t |= (t << 1) & candidates;
        t |= (t << 1) & candidates;
        t |= (t << 1) & candidates;
        t |= (t << 1) & candidates;
        t |= (t << 1) & candidates;
        moves |= (t << 1);

        // Down
        candidates = opp_discs & 0x00FFFFFFFFFFFF00ULL; // Vertical mask not strictly needed but good for safety
        candidates = opp_discs;
        
        t = (my_discs >> 8) & candidates;
        t |= (t >> 8) & candidates;
        t |= (t >> 8) & candidates;
        t |= (t >> 8) & candidates;
        t |= (t >> 8) & candidates;
        t |= (t >> 8) & candidates;
        moves |= (t >> 8);

        // Up
        t = (my_discs << 8) & candidates;
        t |= (t << 8) & candidates;
        t |= (t << 8) & candidates;
        t |= (t << 8) & candidates;
        t |= (t << 8) & candidates;
        t |= (t << 8) & candidates;
        moves |= (t << 8);

        // Diagonals need masking to prevent wrapping
        candidates = opp_discs & 0x7E7E7E7E7E7E7E7EULL;

        // Down-Right (>> 9)
        t = (my_discs >> 9) & candidates;
        t |= (t >> 9) & candidates;
        t |= (t >> 9) & candidates;
        t |= (t >> 9) & candidates;
        t |= (t >> 9) & candidates;
        t |= (t >> 9) & candidates;
        moves |= (t >> 9);

        // Up-Left (<< 9)
        t = (my_discs << 9) & candidates;
        t |= (t << 9) & candidates;
        t |= (t << 9) & candidates;
        t |= (t << 9) & candidates;
        t |= (t << 9) & candidates;
        t |= (t << 9) & candidates;
        moves |= (t << 9);

        // Down-Left (>> 7)
        t = (my_discs >> 7) & candidates;
        t |= (t >> 7) & candidates;
        t |= (t >> 7) & candidates;
        t |= (t >> 7) & candidates;
        t |= (t >> 7) & candidates;
        t |= (t >> 7) & candidates;
        moves |= (t >> 7);

        // Up-Right (<< 7)
        t = (my_discs << 7) & candidates;
        t |= (t << 7) & candidates;
        t |= (t << 7) & candidates;
        t |= (t << 7) & candidates;
        t |= (t << 7) & candidates;
        t |= (t << 7) & candidates;
        moves |= (t << 7);

        return moves & empty;
    }

    // Execute move. Assumes move is valid.
    // Returns a new board with roles swapped (my_discs becomes opp_discs of next state)
    FastBoard make_move(int moveIdx) const {
        FastBoard next;
        uint64_t move = 1ULL << moveIdx;
        uint64_t flips = 0;
        
        // Calculate flips in all 8 directions
        // This is a bit verbose but fast
        
        // Right
        uint64_t t = (move >> 1) & opp_discs;
        if (t) {
            for (int i = 0; i < 6; ++i) t |= (t >> 1) & opp_discs;
            if ((t >> 1) & my_discs) flips |= t;
        }
        
        // Left
        t = (move << 1) & opp_discs;
        if (t) {
            for (int i = 0; i < 6; ++i) t |= (t << 1) & opp_discs;
            if ((t << 1) & my_discs) flips |= t;
        }

        // Down
        t = (move >> 8) & opp_discs;
        if (t) {
            for (int i = 0; i < 6; ++i) t |= (t >> 8) & opp_discs;
            if ((t >> 8) & my_discs) flips |= t;
        }

        // Up
        t = (move << 8) & opp_discs;
        if (t) {
            for (int i = 0; i < 6; ++i) t |= (t << 8) & opp_discs;
            if ((t << 8) & my_discs) flips |= t;
        }

        // Down-Right
        t = (move >> 9) & opp_discs & 0x7E7E7E7E7E7E7E7EULL;
        if (t) {
            for (int i = 0; i < 6; ++i) t |= (t >> 9) & opp_discs & 0x7E7E7E7E7E7E7E7EULL;
            if ((t >> 9) & my_discs) flips |= t;
        }

        // Up-Left
        t = (move << 9) & opp_discs & 0x7E7E7E7E7E7E7E7EULL;
        if (t) {
            for (int i = 0; i < 6; ++i) t |= (t << 9) & opp_discs & 0x7E7E7E7E7E7E7E7EULL;
            if ((t << 9) & my_discs) flips |= t;
        }

        // Down-Left
        t = (move >> 7) & opp_discs & 0x7E7E7E7E7E7E7E7EULL;
        if (t) {
            for (int i = 0; i < 6; ++i) t |= (t >> 7) & opp_discs & 0x7E7E7E7E7E7E7E7EULL;
            if ((t >> 7) & my_discs) flips |= t;
        }

        // Up-Right
        t = (move << 7) & opp_discs & 0x7E7E7E7E7E7E7E7EULL;
        if (t) {
            for (int i = 0; i < 6; ++i) t |= (t << 7) & opp_discs & 0x7E7E7E7E7E7E7E7EULL;
            if ((t << 7) & my_discs) flips |= t;
        }

        // Apply flips
        // Next state: my_discs becomes opponent, opp_discs becomes me (with new piece and flips)
        next.my_discs = opp_discs ^ flips;
        next.opp_discs = my_discs | move | flips;
        
        return next;
    }
};

// ==========================================
// Transposition Table
// ==========================================

struct TTEntry {
    uint64_t key;
    int score;
    int depth;
    int flag; // 0: Exact, 1: Lowerbound (Alpha), 2: Upperbound (Beta)
    int bestMove;
};

class TranspositionTable {
    static const int SIZE = 1 << 20; // 1M entries
    TTEntry table[SIZE];

public:
    TranspositionTable() {
        for (int i = 0; i < SIZE; ++i) table[i].key = 0;
    }

    void store(uint64_t key, int score, int depth, int flag, int bestMove) {
        int idx = key % SIZE;
        // Always replace strategy or depth-based? 
        // Simple replacement is often good enough for Othello
        table[idx] = {key, score, depth, flag, bestMove};
    }

    bool probe(uint64_t key, int depth, int alpha, int beta, int& score, int& bestMove) {
        int idx = key % SIZE;
        if (table[idx].key == key) {
            bestMove = table[idx].bestMove;
            if (table[idx].depth >= depth) {
                if (table[idx].flag == 0) {
                    score = table[idx].score;
                    return true;
                }
                if (table[idx].flag == 1 && table[idx].score >= beta) { // Lowerbound >= beta -> Cutoff
                    score = table[idx].score;
                    return true;
                }
                if (table[idx].flag == 2 && table[idx].score <= alpha) { // Upperbound <= alpha -> Cutoff
                    score = table[idx].score;
                    return true;
                }
            }
        }
        return false;
    }
    
    int getBestMove(uint64_t key) {
        int idx = key % SIZE;
        if (table[idx].key == key) return table[idx].bestMove;
        return -1;
    }
};

// ==========================================
// MyBot Class
// ==========================================

class MyBot : public OthelloPlayer
{
public:
    MyBot(Turn turn);
    virtual Move play(const OthelloBoard &board);

private:
    TranspositionTable tt;
    double startTime;
    double timeLimit;
    bool timeExpired;
    int nodes;

    double getCurrentTime();
    
    // Search
    int solve(FastBoard board, int alpha, int beta);
    int alphaBeta(FastBoard board, int depth, int alpha, int beta, bool passed);
    
    // Evaluation
    int evaluate(const FastBoard& board);
    
    // Helpers
    uint64_t computeHash(const FastBoard& board);
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

// Simple hash function (not full Zobrist for simplicity, but effective enough for local TT)
// For a full engine we'd init random keys.
uint64_t MyBot::computeHash(const FastBoard& board) {
    // A simple mixing hash
    uint64_t h = board.my_discs + 0x9e3779b97f4a7c15ULL;
    h = (h ^ board.opp_discs) * 0xbf58476d1ce4e5b9ULL;
    return h;
}

// Static weights for evaluation
const int WEIGHTS[64] = {
    100, -20, 10,  5,  5, 10, -20, 100,
    -20, -50, -2, -2, -2, -2, -50, -20,
     10,  -2,  5,  1,  1,  5,  -2,  10,
      5,  -2,  1,  1,  1,  1,  -2,   5,
      5,  -2,  1,  1,  1,  1,  -2,   5,
     10,  -2,  5,  1,  1,  5,  -2,  10,
    -20, -50, -2, -2, -2, -2, -50, -20,
    100, -20, 10,  5,  5, 10, -20, 100
};

int MyBot::evaluate(const FastBoard& board) {
    int myCount = board.count_set_bits(board.my_discs);
    int oppCount = board.count_set_bits(board.opp_discs);
    int total = myCount + oppCount;

    // Endgame: Exact disc count difference
    if (total >= 64) {
        return (myCount - oppCount) * 1000;
    }

    int score = 0;

    // 1. Mobility (Number of valid moves)
    Bitboard myMoves = board.get_valid_moves();
    int myMobility = board.count_set_bits(myMoves);
    
    // To get opponent mobility, we need to swap perspective temporarily
    FastBoard oppBoard;
    oppBoard.my_discs = board.opp_discs;
    oppBoard.opp_discs = board.my_discs;
    Bitboard oppMoves = oppBoard.get_valid_moves();
    int oppMobility = board.count_set_bits(oppMoves);

    if (myMobility + oppMobility > 0)
        score += 50 * (myMobility - oppMobility); // Mobility is very important

    // 2. Positional Weights
    // Iterate set bits
    Bitboard temp = board.my_discs;
    while (temp) {
        int idx = __builtin_ctzll(temp);
        score += WEIGHTS[idx];
        temp &= temp - 1;
    }
    temp = board.opp_discs;
    while (temp) {
        int idx = __builtin_ctzll(temp);
        score -= WEIGHTS[idx];
        temp &= temp - 1;
    }

    // 3. Corner Control (Critical)
    uint64_t corners = 0x8100000000000081ULL;
    int myCorners = board.count_set_bits(board.my_discs & corners);
    int oppCorners = board.count_set_bits(board.opp_discs & corners);
    score += 5000 * (myCorners - oppCorners);

    // 4. Stability (Edges)
    // Simplified stability: just count edge pieces if we have corners?
    // Or just raw edge count
    uint64_t edges = 0xFF818181818181FFULL;
    int myEdges = board.count_set_bits(board.my_discs & edges);
    int oppEdges = board.count_set_bits(board.opp_discs & edges);
    score += 50 * (myEdges - oppEdges);
    
    // 5. X-Squares and C-Squares penalty (if corner is empty)
    // Top-Left (0)
    if (!((board.my_discs | board.opp_discs) & 1ULL)) {
        if (board.my_discs & (1ULL << 9)) score -= 500; // X-square
        if (board.my_discs & (1ULL << 1)) score -= 200; // C-square
        if (board.my_discs & (1ULL << 8)) score -= 200; // C-square
        
        if (board.opp_discs & (1ULL << 9)) score += 500;
        if (board.opp_discs & (1ULL << 1)) score += 200;
        if (board.opp_discs & (1ULL << 8)) score += 200;
    }
    // Top-Right (7)
    if (!((board.my_discs | board.opp_discs) & (1ULL << 7))) {
        if (board.my_discs & (1ULL << 14)) score -= 500;
        if (board.my_discs & (1ULL << 6)) score -= 200;
        if (board.my_discs & (1ULL << 15)) score -= 200;
        
        if (board.opp_discs & (1ULL << 14)) score += 500;
        if (board.opp_discs & (1ULL << 6)) score += 200;
        if (board.opp_discs & (1ULL << 15)) score += 200;
    }
    // Bottom-Left (56)
    if (!((board.my_discs | board.opp_discs) & (1ULL << 56))) {
        if (board.my_discs & (1ULL << 49)) score -= 500;
        if (board.my_discs & (1ULL << 57)) score -= 200;
        if (board.my_discs & (1ULL << 48)) score -= 200;
        
        if (board.opp_discs & (1ULL << 49)) score += 500;
        if (board.opp_discs & (1ULL << 57)) score += 200;
        if (board.opp_discs & (1ULL << 48)) score += 200;
    }
    // Bottom-Right (63)
    if (!((board.my_discs | board.opp_discs) & (1ULL << 63))) {
        if (board.my_discs & (1ULL << 54)) score -= 500;
        if (board.my_discs & (1ULL << 62)) score -= 200;
        if (board.my_discs & (1ULL << 55)) score -= 200;
        
        if (board.opp_discs & (1ULL << 54)) score += 500;
        if (board.opp_discs & (1ULL << 62)) score += 200;
        if (board.opp_discs & (1ULL << 55)) score += 200;
    }

    // 6. Parity (Last to move in empty areas)
    // Simplified: just total parity
    if (total > 45) {
        score += (myCount - oppCount) * 20;
    }

    return score;
}

// Endgame solver (Exact search)
int MyBot::solve(FastBoard board, int alpha, int beta) {
    if (timeExpired) return alpha;
    if ((nodes++ & 1023) == 0) {
        if (getCurrentTime() - startTime > timeLimit) timeExpired = true;
    }

    Bitboard moves = board.get_valid_moves();
    
    if (moves == 0) {
        // Pass
        FastBoard next;
        next.my_discs = board.opp_discs;
        next.opp_discs = board.my_discs;
        Bitboard oppMoves = next.get_valid_moves();
        
        if (oppMoves == 0) {
            // Game Over
            int diff = board.count_set_bits(board.my_discs) - board.count_set_bits(board.opp_discs);
            if (diff > 0) return 10000 + diff;
            if (diff < 0) return -10000 + diff;
            return 0;
        }
        return -solve(next, -beta, -alpha);
    }

    int bestScore = -20000;
    
    while (moves) {
        int idx = __builtin_ctzll(moves);
        FastBoard next = board.make_move(idx);
        int score = -solve(next, -beta, -alpha);
        
        if (score > bestScore) bestScore = score;
        if (score > alpha) alpha = score;
        if (alpha >= beta) break;
        
        moves &= moves - 1;
    }
    return bestScore;
}

int MyBot::alphaBeta(FastBoard board, int depth, int alpha, int beta, bool passed) {
    if (timeExpired) return alpha;
    if ((nodes++ & 2047) == 0) {
        if (getCurrentTime() - startTime > timeLimit) timeExpired = true;
    }

    uint64_t hash = computeHash(board);
    int ttScore, ttMove;
    if (tt.probe(hash, depth, alpha, beta, ttScore, ttMove)) {
        return ttScore;
    }

    if (depth == 0) {
        int val = evaluate(board);
        tt.store(hash, val, depth, 0, -1);
        return val;
    }

    Bitboard moves = board.get_valid_moves();
    if (moves == 0) {
        if (passed) {
            // Game Over
            int diff = board.count_set_bits(board.my_discs) - board.count_set_bits(board.opp_discs);
            int val = (diff > 0) ? (10000 + diff) : (diff < 0 ? -10000 + diff : 0);
            return val;
        }
        
        // Pass
        FastBoard next;
        next.my_discs = board.opp_discs;
        next.opp_discs = board.my_discs;
        int val = -alphaBeta(next, depth, -beta, -alpha, true);
        return val;
    }

    // Move Ordering
    std::vector<int> moveList;
    moveList.reserve(16);
    
    // 1. TT Move
    int bestMove = -1;
    if (ttMove != -1 && (moves & (1ULL << ttMove))) {
        moveList.push_back(ttMove);
        moves &= ~(1ULL << ttMove); // Remove from bitboard
    }
    
    // 2. Corners
    uint64_t corners = 0x8100000000000081ULL;
    Bitboard cornerMoves = moves & corners;
    while (cornerMoves) {
        int idx = __builtin_ctzll(cornerMoves);
        moveList.push_back(idx);
        moves &= ~(1ULL << idx);
        cornerMoves &= cornerMoves - 1;
    }
    
    // 3. Rest
    while (moves) {
        int idx = __builtin_ctzll(moves);
        moveList.push_back(idx);
        moves &= moves - 1;
    }

    int flag = 2; // Upperbound
    int score = -20000;

    for (int idx : moveList) {
        FastBoard next = board.make_move(idx);
        int val = -alphaBeta(next, depth - 1, -beta, -alpha, false);
        
        if (timeExpired) return alpha;

        if (val > score) {
            score = val;
            bestMove = idx;
        }
        
        if (score > alpha) {
            alpha = score;
            flag = 0; // Exact
        }
        
        if (alpha >= beta) {
            flag = 1; // Lowerbound
            break;
        }
    }

    tt.store(hash, score, depth, flag, bestMove);
    return score;
}

Move MyBot::play(const OthelloBoard &board)
{
    startTime = getCurrentTime();
    timeExpired = false;
    nodes = 0;

    FastBoard fb;
    fb.load(board, turn);

    Bitboard moves = fb.get_valid_moves();
    if (moves == 0) return Move::pass();

    // Count empty squares for endgame
    int emptyCount = 64 - fb.count_set_bits(fb.my_discs | fb.opp_discs);
    
    // If endgame is close, solve it
    if (emptyCount <= 14) {
        // Find winning move
        int bestIdx = -1;
        int bestScore = -30000;
        
        while (moves) {
            int idx = __builtin_ctzll(moves);
            FastBoard next = fb.make_move(idx);
            int score = -solve(next, -20000, 20000);
            
            if (score > bestScore) {
                bestScore = score;
                bestIdx = idx;
            }
            moves &= moves - 1;
        }
        
        if (bestIdx != -1) {
            int y = bestIdx / 8;
            int x = bestIdx % 8;
            return Move(x, y);
        }
    }

    // Iterative Deepening
    int bestIdx = -1;
    int maxDepth = 30; 
    
    // Get first valid move as fallback
    bestIdx = __builtin_ctzll(moves);

    for (int depth = 1; depth <= maxDepth; ++depth) {
        int alpha = -20000;
        int beta = 20000;
        int currentBestIdx = -1;
        int bestVal = -20000;

        // Root search with move ordering from TT
        std::vector<int> moveList;
        Bitboard m = fb.get_valid_moves();
        
        // Check TT for root
        uint64_t hash = computeHash(fb);
        int ttMove = tt.getBestMove(hash);
        
        if (ttMove != -1 && (m & (1ULL << ttMove))) {
            moveList.push_back(ttMove);
            m &= ~(1ULL << ttMove);
        }
        
        while (m) {
            int idx = __builtin_ctzll(m);
            moveList.push_back(idx);
            m &= m - 1;
        }

        for (int idx : moveList) {
            FastBoard next = fb.make_move(idx);
            int val = -alphaBeta(next, depth - 1, -beta, -alpha, false);
            
            if (timeExpired) break;

            if (val > bestVal) {
                bestVal = val;
                currentBestIdx = idx;
            }
            if (val > alpha) alpha = val;
        }

        if (timeExpired) break;
        
        if (currentBestIdx != -1) {
            bestIdx = currentBestIdx;
            // printf("Depth %d: Score %d, Move %d\n", depth, bestVal, bestIdx);
        }
    }

    int y = bestIdx / 8;
    int x = bestIdx % 8;
    return Move(x, y);
}

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
