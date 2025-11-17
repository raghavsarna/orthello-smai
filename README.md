# Othello Bot - AI3002 Assignment 3

**Team Members:** [Add your roll numbers here: U202300XX_U202300YY_U202300ZZ]

## Overview

This project implements a competitive Othello bot using advanced game-playing techniques. The bot uses **Minimax algorithm with Alpha-Beta pruning** and **Iterative Deepening** to make strategic decisions within the 2-second time constraint.

## Approach

### 1. Core Algorithm: Minimax with Alpha-Beta Pruning

The bot uses the **minimax algorithm** to search through possible game states and select the optimal move. The algorithm assumes both players play optimally and alternates between maximizing and minimizing evaluation scores.

**Alpha-Beta Pruning** is implemented to significantly reduce the search space by eliminating branches that cannot influence the final decision. This optimization allows the bot to search deeper within the time limit.

### 2. Iterative Deepening

To respect the 2-second time limit, the bot uses **iterative deepening**:
- Starts with depth 3 and progressively increases depth
- Adaptive maximum depth based on game phase:
  - **Early game (< 20 coins):** Max depth 6
  - **Mid game (20-50 coins):** Max depth 8
  - **End game (> 50 coins):** Max depth 10
- Always returns the best move found before time expires
- Time limit set conservatively at 1.8 seconds to ensure safety margin

### 3. Evaluation Function

The evaluation function uses multiple strategic heuristics with different weights depending on the game phase:

#### A. Positional Weights
A predefined 8x8 weight matrix assigns strategic values to board positions:
- **Corners (100):** Most valuable - stable and control adjacent squares
- **Edges (5-10):** Moderately valuable - harder to flip
- **X-squares (-50):** Adjacent diagonals to corners - dangerous if corner is empty
- **C-squares (-20):** Adjacent to corners - risky positions
- **Center (1-5):** Neutral to slightly positive

#### B. Corner Control (High Priority)
Measures control of the four corners:
```
Score = 100 * (My Corners - Opponent Corners) / (Total Corners)
```
Corners are critical because they can never be flipped once captured.

#### C. Mobility
Measures the number of available moves:
```
Score = 100 * (My Moves - Opponent Moves) / (Total Moves)
```
Higher mobility gives more options and can restrict opponent's choices.

#### D. Stability
Counts pieces on edges and corners that are harder to flip:
```
Score = 100 * (My Stable - Opponent Stable) / (Total Stable)
```

#### E. Coin Parity
Simple count of pieces on the board:
```
Score = 100 * (My Coins - Opponent Coins) / (Total Coins)
```
Less important in early/mid game, critical in endgame.

### 4. Game Phase Strategy

The bot adapts its strategy based on the number of coins on the board:

#### Early Game (< 20 coins)
- Focus: **Position** and **Mobility**
- Weights: Mobility ×5, Corner ×30, Full positional weights
- Rationale: Build strong position, maintain flexibility, avoid giving up corners

#### Mid Game (20-50 coins)
- Focus: **Balanced** approach
- Weights: Mobility ×3, Corner ×25, Stability ×2, Parity ×1, Reduced positional weights
- Rationale: Balance all factors while maintaining strategic advantages

#### End Game (> 50 coins)
- Focus: **Maximizing coin count**
- Weights: Parity ×10, Corner ×15, Mobility ×1
- Rationale: Win by having more pieces, corners still important for stability

## Implementation Details

### Key Features

1. **Time Management:** Uses `gettimeofday()` for precise time tracking with 1.8s limit
2. **Board Copying:** Creates new board states for each explored move (functional approach)
3. **Move Ordering:** Natural move ordering from `getValidMoves()` (can be enhanced with move sorting)
4. **Pass Handling:** Properly handles cases where a player has no valid moves
5. **Game Over Detection:** Recognizes when neither player can move

### Data Structures

- **OthelloBoard:** Provided by framework, copied for each state exploration
- **Move:** Simple (x, y) coordinate structure
- **list<Move>:** Standard library list for valid moves

### Complexity Analysis

- **Time Complexity:** O(b^d) where b ≈ 10-20 (branching factor) and d ≈ 6-10 (depth)
- **Space Complexity:** O(d) for recursion stack, O(b×d) for board copies
- **Alpha-Beta Pruning:** Reduces effective branching to O(b^(d/2)) in best case

## Testing Results

The bot was tested extensively against RandomBot:

```
Test 1: Win (BLACK) - 60 vs 1 (Score: +59)
Test 2: Win (BLACK) - 51 vs 13 (Score: +38)
Test 3: Win (BLACK) - 58 vs 0 (Score: +58)
Test 4: Win (RED) - 55 vs 0 (Score: +55)
Test 5: Win (RED) - 59 vs 0 (Score: +59)
```

**Win Rate:** 100% against RandomBot with dominant victory margins

## Building and Running

### Compile the Bot
```bash
cd Desdemona/bots/MyBot
make
```

This generates `bot.so` in the MyBot directory.

### Run Against Another Bot
```bash
cd Desdemona
./bin/Desdemona ./bots/MyBot/bot.so ./bots/RandomBot/RandomBot.so
```

The first bot plays as BLACK, the second as RED.

### View Game Log
After each game, check `game.log` for the move sequence:
```bash
cat game.log
```

## Potential Enhancements

Future improvements could include:

1. **Move Ordering:** Sort moves by heuristic value before exploring (best moves first)
2. **Transposition Table:** Cache evaluated positions to avoid redundant calculations
3. **Opening Book:** Pre-computed optimal opening moves
4. **Enhanced Stability:** More sophisticated stability analysis (true stable discs)
5. **Endgame Solver:** Perfect play in final moves using complete search
6. **Parity Analysis:** Consider odd/even square counting in endgame
7. **Pattern Recognition:** Recognize and evaluate common board patterns

## Algorithm Comparison

| Algorithm | Pros | Cons |
|-----------|------|------|
| Random | Fast, simple | Very weak play |
| Greedy | Fast, captures immediately | Short-sighted, loses strategically |
| Minimax | Optimal with sufficient depth | Slow without pruning |
| **Minimax + Alpha-Beta** | Near-optimal, efficient | Requires good evaluation function |
| Monte Carlo Tree Search | No evaluation needed | May need more time to converge |

## References

- Alpha-Beta Pruning: Russell & Norvig, "Artificial Intelligence: A Modern Approach"
- Othello Strategy: https://en.wikipedia.org/wiki/Reversi
- Game Theory: Minimax theorem and optimal play
- Iterative Deepening: Depth-first search with increasing depth limits

## Conclusion

This implementation demonstrates a strong Othello bot using classical AI game-playing techniques. The combination of minimax with alpha-beta pruning, adaptive evaluation heuristics, and iterative deepening within time constraints creates a competitive player that consistently defeats random opponents and should perform well in tournament play.

The bot successfully balances:
- **Strategic depth** (via minimax search)
- **Computational efficiency** (via alpha-beta pruning)
- **Time management** (via iterative deepening)
- **Positional understanding** (via multi-factor evaluation)

---

**Date:** November 17, 2025  
**Course:** AI3002 - Search Methods in AI  
**Assignment:** 3 - Othello Bot
